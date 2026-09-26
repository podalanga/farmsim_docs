# Write an AnimatExtension

How to write an extension: code that runs at given points of the
simulation, for example to apply a force, log a quantity or change the
environment. The design is explained in
[Extension and Controller Design](../explanation/extension-design.md).

## The lifecycle

| Method | When called | Typical use |
|--------|-------------|-------------|
| `from_options()` | Once, when the task is created | Read the `config:`, allocate arrays |
| `initialize_episode(task, physics)` | Start of an episode | Resolve MuJoCo indices, reset state |
| `before_step(task, action, physics)` | Before the MuJoCo step: once per iteration, or every environment step with `substep=True` | Read sensors, apply forces, advance dynamics |
| `after_step(task, physics)` | Once per iteration, after its last step | Read the result of the step |
| `end_episode(task, physics)` | End of the simulation | Save files |

Two base classes:

- **`TaskExtension`** (`farms_core/simulation/extensions.py`): a
  simulation extension, listed in the `extensions:` of the simulation
  file, created with `from_options(config, experiment_options)`.
- **`AnimatExtension`** (`farms_core/model/extensions.py`): attached to
  an animat, listed in the `extensions:` of an animat file, created with
  `from_options(config, experiment_options, animat_i, animat_data, animat_options)`.

To control joints, subclass `AnimatController` instead (see
[Write a Controller](write-controller.md)).

## Step 1: Implement the extension

A lateral force on a link, and a log of the total external force on the
animat:

```python
import numpy as np
from farms_core.model.extensions import AnimatExtension
from farms_core.sensors.sensor_convention import sc
from farms_mujoco.simulation.mjcf import get_prefix


class SideForce(AnimatExtension):
    """Constant force on one link, and log of the total external force"""

    def __init__(self, animat_i, animat_data, link, force, n_iterations):
        super().__init__(substep=True)  # Apply the force at every environment step
        self.prefix = get_prefix(animat_i)  # MuJoCo names are prefixed: 'a0_'
        self.animat_data = animat_data
        self.link = link
        self.force = np.array(force, dtype=float)  # [N]
        self.body_id = None
        self.total_force = np.zeros(n_iterations)

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                     animat_data, animat_options):
        return cls(
            animat_i=animat_i,
            animat_data=animat_data,
            link=config['link'],
            force=config.get('force', [0, 0.1, 0]),
            n_iterations=experiment_options.simulation.runtime.n_iterations,
        )

    def initialize_episode(self, task, physics):
        # Resolve the MuJoCo body index once, not at every step
        self.body_id = physics.model.name2id(self.prefix + self.link, 'body')

    def before_step(self, task, action, physics):
        # xfrc_applied is in simulation units, world frame, at the body CoM
        physics.data.xfrc_applied[self.body_id, :3] += self.force*task.units.newtons

    def after_step(self, task, physics):
        iteration = task.iteration - 1  # The iteration that just ended
        xfrc = np.asarray(self.animat_data.sensors.xfrc.array)
        forces = xfrc[iteration % task.buffer_size, :, sc.xfrc_force_x:sc.xfrc_force_z+1]
        self.total_force[iteration] = np.linalg.norm(forces.sum(axis=0))

    def end_episode(self, task, physics):
        np.save('Output/total_force.npy', self.total_force)
```

Points to note:

- MuJoCo never clears `xfrc_applied`. `SwimmingExtension` sets (`=`) the
  wrench of its links at every step, so an extension listed after it can
  add to it (`+=`), as here. For a body that no other extension writes,
  set the value (`=`) instead, or the force would grow at every step.
- The `xfrc` sensors hold the forces written by the extensions that log
  them (the fluid forces), not your force.
- Convert to simulation units with `task.units` (`newtons`, `torques`,
  `meters`, `seconds`). With the default units, all factors are 1.

## Step 2: Register it in YAML

In `animat_config.yaml`, after the swimming extension:

```yaml
extensions:
  - loader: controller.zbot_controller.ZbotCPGController
    config:
      # ... controller options ...
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config: {}
  - loader: side_force.SideForce
    config:
      link: TailSegment
      force: [0, 0.1, 0]
```

`loader` is the dotted path of the class. The module must be importable:
put it in the experiment folder, which `run_sim.py` adds to `sys.path`.

## Extension ordering

Extensions run in the order of the files: simulation extensions first,
then the extensions of each animat, in the order of `extensions:`. In
each environment step:

```text
before_step:
  1. ExperimentTask.update_sensors()      (links only on the substeps)
  2. Extension 1 before_step()            (if first substep, or substep=True)
  3. Controller before_step(), then its commands written to MuJoCo
  4. Extension 3 before_step()
MuJoCo step(s)
after_step (at the end of an iteration only):
  Extension 1, 2, 3 after_step()
```

If extension B reads data that extension A writes in `before_step()`,
list A before B.

## Accessing the physics

`physics` is the dm_control `Physics`:

```python
def before_step(self, task, action, physics):
    qpos = physics.data.qpos.copy()
    sim_time = physics.time()/task.units.seconds
    dt = physics.timestep()/task.units.seconds  # MuJoCo step [s]
    # Name based access (slower, convenient outside the loop)
    head = physics.named.data.xpos[self.prefix + 'Head']
```

Resolve names to indices in `initialize_episode()` for code that runs
at every step, as `SwimmingHandler` does with the body ids of its links.

## Accessing the options

```python
@classmethod
def from_options(cls, config, experiment_options, animat_i,
                 animat_data, animat_options):
    sim_options = experiment_options.simulation
    n_iterations = sim_options.runtime.n_iterations
    timestep = sim_options.physics.timestep   # Iteration duration [s]
    duration = sim_options.duration()         # timestep*(n_iterations - 1)
    water = experiment_options.arenas[0].water
```

## See also

- [Extension and Controller Design](../explanation/extension-design.md)
- [Write a Controller](write-controller.md)
- [Use Built-in Extensions](use-extensions.md)
- [Controller and extension API](../reference/core/core-control.md)
