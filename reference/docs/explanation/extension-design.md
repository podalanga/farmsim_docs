# Extension and Controller Design

This document explains the design rationale behind FARMS' extension system and
controller architecture.

## Why extensions?

Traditional robotics frameworks often use deep class hierarchies — you subclass
the simulation class to add behavior. FARMS takes a different approach:
composition via extensions.

### Problems with inheritance

- **Fragile** — changes to the base class affect all subclasses
- **Inflexible** — can only have one chain of behaviors
- **Hard to share** — behavior is embedded in class hierarchy, not reusable

### Extension benefits

- **Composable** — mix any set of extensions without code changes
- **Orderable** — extension order is controlled by YAML declaration
- **Reusable** — the same extension works across different robots
- **Isolated** — each extension manages its own state

## The lifecycle design

FARMS extensions follow a lifecycle that mirrors the simulation loop:

```
initialize_episode()  →  before_step()  →  [physics step]  →  after_step()  →  ...  →  end_episode()
```

This maps directly to the dm_control `Task` interface:

| FARMS method | dm_control method | When |
|--------------|-------------------|------|
| `initialize_episode()` | `Task.initialize_episode()` | Once at start |
| `before_step()` | `Task.before_step()` | Before each physics step |
| `after_step()` | `Task.after_step()` | After each physics step |
| `end_episode()` | `Task.after_episode()` | At simulation end |

The key insight is that `before_step()` runs before physics, allowing
extensions to set forces and controllers to set joint targets. `after_step()`
runs after physics, allowing extensions to record the resulting state.

## Two extension levels

FARMS distinguishes simulation-level and animat-level extensions:

### TaskExtension (simulation-level)

- Registered in `simulation_config.yaml`
- Has access to `SimulationOptions` and `ExperimentData`
- Example: `ExperimentLogger` (saves all data), `ExperimentOptionsLogger`
  (saves configs)

### AnimatExtension (animat-level)

- Registered in `animat_config.yaml`
- Has access to `animat_data`, `animat_options`, and `animat_i`
- Example: `SwimmingExtension` (fluid forces), `CameraFollower` (visualization)

This separation reflects the scoping of concerns — simulation-level extensions
operate on the whole experiment, while animat-level extensions operate on a
single robot.

## Controllers as extensions

`AnimatController` extends `AnimatExtension`. This is deliberate: a controller
is just an extension that also produces joint targets. Verified against
`ExperimentTask.before_step()` (`farms_mujoco/simulation/task.py`), the real
sequencing is:

1. `controller_loader` (and any other extension) is instantiated once, up
   front, from the `extensions:` YAML list — controllers aren't created
   through a separate mechanism from ordinary extensions.
2. Every physics step, the task iterates its extensions **in YAML order**.
   For each one, it calls `extension.before_step(...)`, and — **immediately
   after that same call, still inside the same loop iteration** — checks
   `isinstance(extension, AnimatController)` and, if so, dispatches
   `positions()`/`velocities()`/`torques()` (whichever `ControlType`
   buckets are non-empty for that controller) and writes the results into
   `physics.data.ctrl` right there.
3. This repeats for every extension in the list before `physics.step()`
   actually advances the simulation.

This design means a controller's internal dynamics (e.g., CPG integration)
run at the same lifecycle point as other extensions, ensuring consistent
ordering — and that YAML declaration order controls not just extension
sequencing in general, but specifically whether an extension sees the
controller's own step's ctrl output or not.

## The from_options() contract

Both `TaskExtension` and `AnimatExtension` define `from_options()` as the
factory method — but they are **not** the same signature; `AnimatExtension`
adds three animat-scoped parameters on top of `TaskExtension`'s two, verified
against `farms_core/simulation/extensions.py` and
`farms_core/model/extensions.py`:

```python
# TaskExtension (farms_core/simulation/extensions.py)
@classmethod
def from_options(cls, config: dict, experiment_options: ExperimentOptions):

# AnimatExtension (farms_core/model/extensions.py) — extends TaskExtension
@classmethod
def from_options(cls, config, experiment_options, animat_i,
                animat_data, animat_options):
```

There is no separate `sim_options`/`experiment_data` parameter — a
`TaskExtension` that needs simulation options reads them off
`experiment_options.simulation`, and one that needs experiment data reads
it from `task.data` inside a lifecycle hook instead (data isn't available
yet at `from_options()` time — see `ExperimentLogger`, which stores
`self.data = task.data` in `initialize_episode()`, not in `from_options()`).

The framework passes all necessary context to the extension at creation time.
The extension stores what it needs and ignores the rest. This is a form of
constructor injection — the extension declares what it needs by accepting
these parameters.

## Control types and joint mapping

The `ControlType` enum allows different joints to use different control modes
within the same simulation. A robot might have position-controlled body joints
and torque-controlled leg joints.

`AnimatController.joints_from_control_types()` groups joints by control type,
producing a `tuple` of 7 lists — one per `ControlType`, in enum order
(`joints_names[ControlType.POSITION]`, `joints_names[ControlType.VELOCITY]`,
etc.), not a single flat list. Each per-`ControlType` sublist preserves the
original joint ordering, filtered down to only the joints using that control
type. This grouped-tuple shape is what every dispatch method
(`positions()`, `velocities()`, ...) indexes into to know which joints it's
responsible for.

## The substep parameter

`AnimatController.__init__` takes `substep=True`. When enabled, the controller
runs at the substep resolution of the MuJoCo physics engine. This is important
for stiff systems (e.g., Ekeberg muscle model) where the control frequency must
match the physics integration frequency.

## See also

- [Extension API](../reference/core/extension-api.md) — full class reference
- [Write an AnimatExtension](../how-to/write-extension.md) — practical guide
- [Write a Controller](../how-to/write-controller.md) — controller patterns
- [Simulation Lifecycle](simulation-lifecycle.md) — when methods are called
