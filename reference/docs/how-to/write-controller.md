# Write a Controller

Patterns and details for writing a controller, an `AnimatExtension` that
also produces joint commands. For a first walkthrough, see
[Write a Custom Controller](../tutorials/custom-controller.md).

## The AnimatController base class

```python
class AnimatController(AnimatExtension):
    def __init__(self, animat_i, joints_names, muscles_names,
                 max_torques, substep=True): ...

    def positions(self, iteration, time, timestep) -> dict[str, float]: ...
    def velocities(self, iteration, time, timestep) -> dict[str, float]: ...
    def torques(self, iteration, time, timestep) -> dict[str, float]: ...
    def springrefs(self, iteration, time, timestep) -> dict[str, float]: ...
    def springcoefs(self, iteration, time, timestep) -> dict[str, float]: ...
    def dampingcoefs(self, iteration, time, timestep) -> dict[str, float]: ...
    def excitations(self, iteration, time, timestep): ...  # Array, one value per muscle
```

| Parameter | Description |
|-----------|-------------|
| `animat_i` | Index of the animat |
| `joints_names` | One list of joint names per `ControlType` (7 lists, in enum order). Build it with `AnimatController.joints_from_control_types()` |
| `muscles_names` | Names of the muscles driven by `excitations()` (can be empty) |
| `max_torques` | Torque limits, grouped like `joints_names`. Build it with `AnimatController.max_torques_from_control_types()` |
| `substep` | Run `before_step()` and the commands at every environment step (default) or once per iteration |

After `__init__`, `self.joints_names[ControlType.POSITION]` is the list of
the position controlled joints, in motor order.

!!! warning "Override `from_options()`"
    The base `AnimatController.from_options()` creates a controller whose
    `joints_names` are seven empty lists: no joint would be controlled.

## How ExperimentTask uses the controller

In `ExperimentTask.before_step()` (`farms_mujoco/simulation/task.py`),
right after the controller's `before_step()`:

1. if `joints_names[ControlType.POSITION]` is not empty, `positions()` is
   called and the values written to the position actuators
   (`physics.data.ctrl`, radians);
2. the same for `VELOCITY` (`velocities()`, scaled by
   `units.angular_velocity`);
3. if `joints_names[ControlType.TORQUE]` is not empty, `torques()` is
   written to the torque actuators (scaled by `units.torques`), and
   `springrefs()`, `springcoefs()` and `dampingcoefs()` are called and
   written to the **model**: `physics.model.qpos_spring`, `jnt_stiffness`
   and `dof_damping`;
4. if `muscles_names` is not empty, `excitations()` is written to the
   muscle actuators, in the order of `muscles_names`.

The `iteration` argument is the index in the sensor buffers
(`task.iteration % buffer_size`), `time` is the simulation time [s] and
`timestep` the iteration duration.

Only the joints of the matching control type need to be returned, and a
method that is not needed can keep the base implementation.

## Control types

The motor's `control_types` list in YAML sets the control types of a
joint. `ControlType.from_string_list()` converts it:

| String | Enum | Integer |
|--------|------|---------|
| `position` | `POSITION` | 0 |
| `velocity` | `VELOCITY` | 1 |
| `torque` | `TORQUE` | 2 |
| `springref` | `SPRINGREF` | 3 |
| `springcoef` | `SPRINGCOEF` | 4 |
| `dampingcoef` | `DAMPINGCOEF` | 5 |
| `muscle` | `MUSCLE` | 6 |

`joints_from_control_types(joints_names, joints_control_types)` returns a
tuple of 7 lists, the joints of each control type in the order of
`joints_names`. `max_torques_from_control_types(joints_names, max_torques, joints_control_types)`
returns the matching tuple of torque limit arrays.

## Implementation pattern

```python
import numpy as np
from farms_core.model.control import AnimatController, ControlType


class SineController(AnimatController):
    """Travelling sine wave, with the phase integrated in before_step()"""

    def __init__(self, frequency, amplitude, **kwargs):
        super().__init__(**kwargs)
        self.frequency = frequency  # [Hz]
        self.amplitude = amplitude  # [rad]
        self.phase = 0.0

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                     animat_data, animat_options):
        motors = animat_options.control.motors
        joints = [motor.joint_name for motor in motors]
        control_types = {
            motor.joint_name: ControlType.from_string_list(motor.control_types)
            for motor in motors
        }
        return cls(
            frequency=config.get('frequency', 1.0),
            amplitude=config.get('amplitude', 0.5),
            animat_i=animat_i,
            joints_names=cls.joints_from_control_types(
                joints_names=joints,
                joints_control_types=control_types,
            ),
            muscles_names=[],
            max_torques=cls.max_torques_from_control_types(
                joints_names=joints,
                max_torques={motor.joint_name: motor.limits_torque[1] for motor in motors},
                joints_control_types=control_types,
            ),
            substep=True,
        )

    def initialize_episode(self, task, physics):
        self.phase = 0.0

    def before_step(self, task, action, physics):
        dt = task.timestep/task.cb_sub_steps  # Environment step [s]
        self.phase += 2*np.pi*self.frequency*dt

    def positions(self, iteration, time, timestep):
        return {
            joint: self.amplitude*np.sin(self.phase - i*np.pi/3)
            for i, joint in enumerate(self.joints_names[ControlType.POSITION])
        }
```

## Registering the controller

A controller runs because it is listed in the animat's `extensions:`,
whose `config:` is passed to `from_options()`:

```yaml
control:
  motors:
    - joint_name: joint_1
      control_types: [position]
      limits_torque: [-10.0, 10.0]
      gains: [3.0, 0.01, 0]   # kp, kv of the position actuator, kv of the velocity actuator
    # ...
extensions:
  - loader: controller.sine_controller.SineController
    config:
      frequency: 2.0
      amplitude: 0.3
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config: {}
```

!!! note "`control.controller_loader`"
    `controller_loader` is parsed but not used: it does not create the
    controller. Only the `extensions:` entry matters.

## The built-in AmphibiousController

`farms_amphibious.control.amphibious.AmphibiousController` (a
`JointMuscleController`) drives the joints from a CPG network configured
in YAML (`control.network`, `control.muscles`), with one equation per
motor (`equation:`):

| Equation | Control types | Handler | Description |
|----------|---------------|---------|-------------|
| `position_phase` | position | `PositionPhaseCy` | Position from the oscillator phase |
| `position_muscle` | position | `PositionMuscleCy` | Position from the difference of the two oscillator outputs (the Zbot) |
| `ekeberg_muscle` | velocity, torque | `EkebergMuscleCy` | Ekeberg muscle, stiffness and damping through the MuJoCo joint |
| `ekeberg_muscle_explicit` | torque | `EkebergMuscleCy` | Ekeberg muscle, all terms as a torque |
| `passive` | velocity, torque | `PassiveJointCy` | Passive spring-damper |

See [Configure CPG Network Parameters](configure-cpg-network.md) and
[Mathematical Models](../explanation/mathematical-models.md#joint-equations).

## See also

- [Write a Custom Controller (Tutorial)](../tutorials/custom-controller.md)
- [Zbot Custom CPG Controller](../tutorials/zbot-custom-controller.md)
- [Extension and Controller Design](../explanation/extension-design.md)
