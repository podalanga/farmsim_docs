# Write an AnimatExtension

This guide shows how to write a custom `AnimatExtension` — a plugin that runs
code at specific lifecycle points during a simulation.

## What is an extension?

Extensions are the primary extension point in FARMS. They hook into the
simulation lifecycle:

| Method | When called | Typical use |
|--------|-------------|-------------|
| `initialize_episode()` | Once at start | Set up data structures, maps |
| `before_step()` | Every physics step | Read sensors, compute forces, advance dynamics |
| `after_step()` | Every physics step | Log data, update counters |
| `end_episode()` | At simulation end | Save files, cleanup |

Extensions come in two flavors:

- **`TaskExtension`** (`farms_core/simulation/extensions.py`) — simulation-level,
  created from `simulation_config.yaml` `extensions:` list
- **`AnimatExtension`** (`farms_core/model/extensions.py`) — animat-level,
  created from `animat_config.yaml` `extensions:` list, has access to
  `animat_data` and `animat_options`

## Step 1: Choose the base class

| If you need... | Use |
|----------------|-----|
| Access to animat data and options | `AnimatExtension` |
| Only physics/simulation state | `TaskExtension` |
| To control joint targets | `AnimatController` (which extends `AnimatExtension`) |

## Step 2: Implement the extension

```python
import numpy as np
from farms_core.model.extensions import AnimatExtension
from farms_core.model.data import AnimatData
from farms_core.model.options import AnimatOptions
from farms_core.experiment.options import ExperimentOptions

class ForceLogger(AnimatExtension):
    """Log total external force on the animat at each step."""

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                    animat_data, animat_options):
        """Factory method — called by ExperimentTask during setup."""
        extension = cls(
            animat_i=animat_i,
            animat_data=animat_data,
            animat_options=animat_options,
        )
        n_iterations = experiment_options.simulation.run.n_iterations
        extension.force_log = np.zeros(n_iterations)
        return extension

    def initialize_episode(self, task, physics):
        """Called once before the first step."""
        pass

    def before_step(self, task, action, physics):
        """Called before each physics step."""
        # Read xfrc sensor data (set by SwimmingExtension, if present)
        iteration = task.iteration
        xfrc = self.animat_data.sensors.xfrc.array[iteration, :, :3]
        self.force_log[iteration] = np.linalg.norm(np.sum(xfrc, axis=0))

    def after_step(self, task, action, physics):
        """Called after each physics step."""
        pass

    def end_episode(self, task, physics):
        """Called at simulation end."""
        np.save('total_force.npy', self.force_log)
```

## Step 3: Register in YAML

Add your extension to the `extensions:` list in `animat_config.yaml`:

```yaml
extensions:
  - loader: my_package.force_logger.ForceLogger
    config:
      # Any config values you read in from_options()
      output_file: total_force.npy
```

The `loader` is a dotted Python path. The module must be importable — either
in the Python path or in the experiment directory (which the experiment's
own `run_sim.py` adds to `sys.path` itself, before calling
`farms_sim._bootstrap.main()` — `_bootstrap.main()` takes no arguments and
does not touch `sys.path`; see `reference/farms-sim.md`).

## Extension ordering

Extensions are called in the order they appear in the YAML `extensions:` list.
Within each step:

```
before_step:
  1. ExperimentTask.update_sensors()   # gated by `full_step or self.substeps_links`,
                                        # not unconditional — see note below
  2. Extension 1 before_step()
  3. Extension 2 before_step()
  4. ...
  5. Controller before_step()           # if controller is an extension
  6. Controller outputs applied to physics.data.ctrl

after_step:
  1. Extension 1 after_step()
  2. Extension 2 after_step()
  3. ...
```

!!! note "`update_sensors()` is not unconditional"
    In `ExperimentTask.before_step()` (`farms_mujoco/simulation/task.py`),
    sensors are only refreshed when `full_step or self.substeps_links` is
    true; `full_step` is true on the first iteration or every `cb_sub_steps`
    substeps. On a skipped substep, `update_sensors(links_only=not
    full_step)` may still run but restricted to links-only data. Likewise,
    each extension's own `before_step()` only runs when `full_step or
    extension.substep` — an extension with `substep=True` runs on every
    physics substep, others only on full steps. If your extension needs
    fresh sensor data every call, check `task.iteration`/`full_step`
    semantics rather than assuming a fixed per-physics-step cadence.

!!! warning "Extension order matters"
    If extension B depends on data computed by extension A in `before_step()`,
    list A before B in the YAML. For example, `SwimmingExtension` must run
    before any extension that reads xfrc data.

## The `from_options()` contract

```python
@classmethod
def from_options(cls, config, experiment_options, animat_i,
                animat_data, animat_options):
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `config` | dict | The `config` dict from the YAML extension entry |
| `experiment_options` | ExperimentOptions | Full experiment configuration |
| `animat_i` | int | Index of this animat (0-based) |
| `animat_data` | AnimatData | Pre-allocated data arrays for this animat |
| `animat_options` | AnimatOptions | Options for this animat |

The method must return an instance of the extension class.

## Accessing the physics engine

In `before_step()` and `after_step()`, you receive a `physics` object (dm_control
`Physics`):

```python
def before_step(self, task, action, physics):
    # Read MuJoCo state
    qpos = physics.data.qpos.copy()
    qvel = physics.data.qvel.copy()

    # Apply external force to a link
    body_id = physics.model.body('link_head').id
    physics.data.xfrc_applied[body_id] = [fx, fy, fz, tx, ty, tz]

    # Read time
    sim_time = physics.time() / task.units.seconds
    dt = physics.timestep() / task.units.seconds
```

## Common patterns

### Applying a force

```python
def before_step(self, task, action, physics):
    body_id = physics.model.body('link_tail').id
    physics.data.xfrc_applied[body_id, :3] = [0.0, 1.0, 0.0]  # force
    physics.data.xfrc_applied[body_id, 3:] = [0.0, 0.0, 0.0]  # torque
```

### Reading another extension's data

Extensions share the same `AnimatData` object. Data written by one extension
into `animat_data.sensors` arrays is visible to all others.

### Accessing experiment options

```python
@classmethod
def from_options(cls, config, experiment_options, animat_i,
                animat_data, animat_options):
    duration = experiment_options.simulation.run.duration
    timestep = experiment_options.simulation.run.timestep
    n_iterations = int(duration / timestep)
    # ...
```

## See also

- [Extension API](../reference/core/extension-api.md) — full class reference
- [Write a Controller](write-controller.md) — controllers are a special case
  of extensions
- [Extension and Controller Design](../explanation/extension-design.md) —
  design rationale
