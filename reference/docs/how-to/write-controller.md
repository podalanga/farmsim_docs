# Write a Controller

This guide covers writing a custom locomotion controller — the most common
type of `AnimatExtension`. For the tutorial walkthrough, see
[Write a Custom Controller](../tutorials/custom-controller.md).

## Controller vs. extension

A controller is an `AnimatExtension` that also extends `AnimatController`
(`farms_core/model/control.py`). The key difference: controllers implement
`positions()`, `velocities()`, and/or `torques()` to return joint targets,
which the `ExperimentTask` applies to MuJoCo's `physics.data.ctrl`.

## The AnimatController base class

```python
class AnimatController(AnimatExtension):
    def __init__(self, animat_i, joints_names, muscles_names,
                 max_torques, substep=True):
        ...

    def positions(self, iteration, time, timestep) -> dict[str, float]:
        return {}

    def velocities(self, iteration, time, timestep) -> dict[str, float]:
        return {}

    def torques(self, iteration, time, timestep) -> dict[str, float]:
        return {}

    def before_step(self, task, action, physics):
        ...
```

### Constructor parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `animat_i` | int | Animat index |
| `joints_names` | list[str] | Joint names, grouped by control type |
| `muscles_names` | list[str] | Muscle names (can be empty) |
| `max_torques` | list[float] | Torque limits per joint |
| `substep` | bool | Whether to run at substep resolution |

## Control types

Each joint can be controlled via one or more control types, declared in the
motor's `control_types` list in YAML. `ControlType.from_string_list()` converts
string names to enum values:

| String | Enum | Integer |
|--------|------|---------|
| `"position"` | `POSITION` | 0 |
| `"velocity"` | `VELOCITY` | 1 |
| `"torque"` | `TORQUE` | 2 |
| `"springref"` | `SPRINGREF` | 3 |
| `"springcoef"` | `SPRINGCOEF` | 4 |
| `"dampingcoef"` | `DAMPINGCOEF` | 5 |
| `"muscle"` | `MUSCLE` | 6 |

## Helper methods

`AnimatController` provides two static helpers for setting up joint names and
torque limits:

### `joints_from_control_types(joints_names, joints_control_types)`

Given a dict mapping joint names to lists of `ControlType`, returns a flat list
of joint names grouped by control type (position joints first, then velocity,
then torque, etc.). This is what you pass to `__init__` as `joints_names`.

### `max_torques_from_control_types(joints_names, max_torques, joints_control_types)`

Given per-joint max torques, returns torque limits aligned with the control
type grouping from `joints_from_control_types()`.

## Implementation pattern

```python
import numpy as np
from farms_core.model.control import AnimatController, ControlType

class MyController(AnimatController):
    """Simple sine-wave controller."""

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                    animat_data, animat_options):
        # Read config
        frequency = config.get('frequency', 1.0)
        amplitude = config.get('amplitude', 0.5)

        # Get joint info from options
        joints_names = animat_options.control.joints_names()
        joints_control_types = {
            motor.joint_name: ControlType.from_string_list(motor.control_types)
            for motor in animat_options.control.motors
        }

        # Create controller
        controller = cls(
            animat_i=animat_i,
            joints_names=AnimatController.joints_from_control_types(
                joints_names=joints_names,
                joints_control_types=joints_control_types,
            ),
            muscles_names=[],
            max_torques=AnimatController.max_torques_from_control_types(
                joints_names=joints_names,
                max_torques={
                    motor.joint_name: motor.limits_torque[1]
                    for motor in animat_options.control.motors
                },
                joints_control_types=joints_control_types,
            ),
            substep=True,
        )

        controller.frequency = frequency
        controller.amplitude = amplitude
        controller.data = animat_data
        controller.phase = 0.0  # Initialize internal phase
        # Determine which joints are position-controlled
        controller.position_joints = [
            name for name, types in joints_control_types.items()
            if ControlType.POSITION in types
        ]
        return controller

    def before_step(self, task, action, physics):
        """Advance internal state."""
        self.phase += self.frequency * physics.timestep() / task.units.seconds

    def positions(self, iteration, time, timestep):
        """Return position targets for position-controlled joints."""
        return {
            name: self.amplitude * np.sin(self.phase + i * 0.5)
            for i, name in enumerate(self.position_joints)
        }
```

## Registering the controller

In `animat_config.yaml`:

```yaml
control:
  controller_loader: my_package.my_controller.MyController
  sensors:
    joints:
      - joint_0
      - joint_1
  motors:
    - joint_name: joint_0
      control_types: ["position"]
      limits_torque: [-1.0, 1.0]
      gains: [1.0]
    - joint_name: joint_1
      control_types: ["position"]
      limits_torque: [-1.0, 1.0]
      gains: [1.0]
  muscles: []
  adhesions: []
  visuals: []
extensions:
  - loader: my_package.my_controller.MyController
    config:
      frequency: 2.0
      amplitude: 0.3
```

!!! note "Two registrations"
    The controller is registered in two places: `controller_loader` (under
    `control:`) tells `ExperimentTask` which class to instantiate, and the
    top-level `extensions:` entry (sibling to `control:`, not nested under it)
    provides the `config` dict that gets passed to `from_options()`.

## The built-in JointMuscleController

`farms_amphibious` provides `JointMuscleController`
(`farms_amphibious/control/amphibious.py`) as the default controller for
amphibious robots. It supports multiple joint equations:

| Equation | Control types | Description |
|----------|---------------|-------------|
| `phase` | position | Phase-based position control from CPG |
| `position_muscle` | position | Position control with muscle mapping |
| `ekeberg_muscle` | velocity, torque | Ekeberg muscle model |
| `ekeberg_muscle_explicit` | torque | Explicit Ekeberg torque |
| `passive` | velocity, torque | Passive joint with spring-damper |
| `passive_explicit` | torque | Explicit passive torque |

Each equation is a Cython-compiled handler (`PositionPhaseCy`,
`PositionMuscleCy`, `EkebergMuscleCy`, `PassiveJointCy`) that maps CPG
oscillator outputs to joint commands.

## See also

- [Write a Custom Controller (Tutorial)](../tutorials/custom-controller.md)
- [Configure CPG Network Parameters](configure-cpg-network.md)
- [Extension API](../reference/core/extension-api.md)
