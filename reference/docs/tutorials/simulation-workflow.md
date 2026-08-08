# Trace a Simulation Step

This tutorial traces the execution path from a YAML configuration file through
to a single MuJoCo physics step. Understanding this flow is essential for
extending FARMS — it shows where your code plugs in.

## The entry point

All simulations start from `run_sim.py` in the experiment directory:

```python
# experiments/zbot_bout_glide/run_sim.py
import sys, os
current_dir = os.path.abspath(os.path.dirname(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from farms_sim._bootstrap import main
sys.exit(main())
```

`run_sim.py` first inserts the experiment directory into `sys.path` itself
(so local modules such as `controller.zbot_controller` can be imported), then
calls `farms_sim._bootstrap.main()`. The same `farmsim` console command
installed by `farms_sim` (`farmsim --experiment_config ...`) calls the same
`main()` function.

`_bootstrap.main()` takes no arguments. On macOS it re-executes the process
under `mjpython` if available (MuJoCo's viewer requires it there), then
delegates to `farms_sim.farmsim.profile_simulation()`. On Linux/Windows it
calls `profile_simulation()` directly.

## Phase 1: Argument parsing

`profile_simulation()` calls `sim_parse_args()` from
`farms_sim.utils.parse_args`, then wraps `main()` (the *simulation* `main()`
in `farms_sim.farmsim`, not `_bootstrap.main()`) in `farms_core.utils.profile.profile()`:

```python
args = sim_parse_args()
```

Key CLI arguments (verified in `farms/farms_sim/farms_sim/utils/parse_args.py`):

| Argument | Type | Description |
|----------|------|-------------|
| `--experiment_config` | str | Path to `experiment_config.yaml` (required) |
| `--simulator` | str | Simulator backend: `mujoco` or `pybullet` (default: `mujoco`) |
| `--log_path` | str | Output directory (default: `./simulations`) |
| `--plot` | flag | Plot results after simulation |
| `--video` | str | Video name (empty = no video) |

## Phase 2: Loading options

The experiment YAML is loaded into an `ExperimentOptions` object:

```python
experiment_options = ExperimentOptions.load(args.experiment_config)
```

`yaml2pyobject()` itself is a thin wrapper around `yaml.load()` — it does
**not** resolve any dotted paths. The dotted-path resolution happens
separately, in `ExperimentOptions.load()` (`farms_core/experiment/options.py`),
which reads a `loaders:` block listing the classes to use, and treats
`simulation` / `animats` / `arenas` as plain filenames to feed into those
classes' own `.load()`:

```yaml
# experiment_config.yaml (simplified)
simulation: simulation_config.yaml
animats:
  - animat_config.yaml
arenas:
  - arena_config.yaml
loaders:
  simulation_options: farms_core.simulation.options.SimulationOptions
  animats_options:
    - farms_amphibious.model.options.AmphibiousOptions
  arenas_options:
    - farms_amphibious.model.options.AmphibiousArenaOptions
  experiment_data: farms_core.experiment.data.ExperimentData
  animats_data:
    - farms_core.model.data.AnimatData
```

`ExperimentOptions.load()` walks this in order: it builds an
`ExperimentLoadOptions` from the `loaders:` block, then for each of
`simulation` / `animats[i]` / `arenas[i]` that is still a string, calls
`import_item()` on the corresponding entry in `loaders` and invokes that
class's own `.load(filename, strict=strict)` to replace the string with the
parsed options object. `animats` and `arenas` must have exactly as many
entries as `loaders.animats_options` / `loaders.arenas_options` — mismatched
lengths raise an assertion error naming the file.

!!! note "Don't confuse this with extension `loader:`/`config:` pairs"
    Individual **extensions** (inside `simulation_config.yaml`'s or
    `animat_config.yaml`'s `extensions:` list) *do* use an inline
    `loader:`/`config:` pair per entry — that's a different, simpler
    mechanism (`ExtensionOptions`) unrelated to the top-level
    `ExperimentOptions` loading above. See
    [Options and YAML Design](../explanation/options-yaml-design.md) for
    both mechanisms side by side.

## Phase 3: Creating experiment data

```python
experiment_data = ExperimentData.from_options(experiment_options)
```

`ExperimentData` pre-allocates NumPy arrays for all sensor data, network states,
and timing information. Each animat gets an `AnimatData` object containing:

- `SensorsData` — arrays for links, joints, contacts, xfrc, muscles, adhesions,
  visuals
- `NetworkLog` (optional) — CPG oscillator state history

The array sizes are determined by the simulation duration, timestep, and the
number of sensors declared in the options.

## Phase 4: Simulation setup

```python
simulation = simulation_setup(
    simulator=args.simulator,
    experiment_options=experiment_options,
    experiment_data=experiment_data,
)
```

For MuJoCo (the default), this calls `MuJoCoSimulation.from_experiment()`:

1. **MJCF construction** — `setup_mjcf_xml()` builds a MuJoCo XML model from
   the animat SDF files, arena SDF, and all options (morphology, sensors,
   extensions)
2. **Task creation** — `ExperimentTask` is created as a dm_control `Task`
3. **Environment creation** — `dm_control.rl.control.Environment` wraps the
   task and physics

During this phase, all extensions listed in the YAML `extensions:` lists are
instantiated via their `from_options()` class methods.

## Phase 5: The simulation loop

```python
simulation.run(headless=True)
```

The `run()` method iterates:

```python
for iteration in range(n_iterations):
    # 1. Step the environment
    env.step(action=None)  # action unused; controllers drive via extensions
```

Inside `env.step()`, the dm_control framework calls:

### `task.initialize_episode()` — called once at start

- Builds name-to-index maps for joints, links, sensors
- Creates controller instances from `animat_options.control.controller_loader`
- Initializes all extensions
- Sets up sensor data arrays

### `task.before_step(physics)` — called every iteration

This is the core per-step pipeline:

```python
def before_step(self, physics):
    # 1. Update sensor readings from MuJoCo state
    self.update_sensors(iteration, physics)

    # 2. Call each extension's before_step()
    for extension in self.extensions:
        extension.before_step(self, None, physics)

    # 3. Collect controller outputs and apply to MuJoCo
    for controller in self.animat_controllers:
        positions = controller.positions(iteration, time, timestep)
        velocities = controller.velocities(iteration, time, timestep)
        torques = controller.torques(iteration, time, timestep)
        # ... map to physics.data.ctrl
```

Controllers are themselves `AnimatExtension` instances, so their `before_step()`
advances internal dynamics (e.g., CPG integration), and then `positions()` /
`velocities()` / `torques()` return the computed joint targets.

### `task.after_step(physics)` — called every iteration

```python
def after_step(self, physics):
    self.iteration += 1
    for extension in self.extensions:
        extension.after_step(self, None, physics)
```

Extensions like `ExperimentLogger` use `after_step()` to record data into the
pre-allocated HDF5 arrays.

## Where your code plugs in

| What you want to do | Where to plug in |
|---------------------|------------------|
| Add per-step behavior | `AnimatExtension.before_step()` / `after_step()` |
| Control joint targets | `AnimatController.positions()` / `velocities()` / `torques()` |
| Read sensor data | Access `AnimatData.sensors` in any extension |
| Add a physics force | `AnimatExtension.before_step()` → `physics.data.xfrc_applied` |
| Log custom data | `AnimatExtension.after_step()` → write to `AnimatData` |

## Next steps

- [Write a Custom Controller](custom-controller.md) — implement your own
  `AnimatController` subclass
- [Write an AnimatExtension](../how-to/write-extension.md) — add custom
  per-step behavior
- [Extension and Controller Design](../explanation/extension-design.md) —
  understand the lifecycle in depth
