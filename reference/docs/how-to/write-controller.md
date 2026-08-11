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

    def springrefs(self, iteration, time, timestep) -> dict[str, float]:
        return {}

    def springcoefs(self, iteration, time, timestep) -> dict[str, float]:
        return {}

    def dampingcoefs(self, iteration, time, timestep) -> dict[str, float]:
        return {}

    def excitations(self, iteration, time, timestep) -> dict[str, float]:
        return {}

    def before_step(self, task, action, physics):
        ...
```

`AnimatController` defines seven override points in total, one per
`ControlType` value, and `farms_mujoco/simulation/task.py` calls all of
them. Beyond `positions()`/`velocities()`/`torques()`:

- `springrefs()` — writes into `physics.model.qpos_spring` (the passive
  spring rest position for `SPRINGREF`-controlled joints)
- `springcoefs()` — writes into `physics.model.jnt_stiffness`
- `dampingcoefs()` — writes into `physics.model.dof_damping`
- `excitations()` — muscle excitation values, read by muscle-equation
  handlers rather than written directly to `physics.data`/`physics.model`

The first three are called **unconditionally alongside `torques()`**
inside `step_joints_control_torque()` — not gated behind their own
`ControlType` check the way `positions()`/`velocities()`/`torques()` are —
because they set MuJoCo **model** parameters (not per-step `data.ctrl`
values) that only make sense for torque-driven joints with a passive
spring/damper component, such as the Zbot's `stiffness`/`springref`/
`damping` fields in `morphology.joints`. Return `{}` (the base class
default) from any of these you don't need; only populate a method if your
controller actually modulates that quantity at runtime (e.g. an
adaptive-stiffness controller would implement `springcoefs()`).

### Constructor parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `animat_i` | int | Animat index |
| `joints_names` | `tuple[list[str], ...]` (length 7) | Joint names **grouped by `ControlType`** — index `[ControlType.POSITION]` for the joints driven by `positions()`, `[ControlType.VELOCITY]` for `velocities()`, etc. Build this with `AnimatController.joints_from_control_types()`, don't hand-roll it — the length-7, enum-ordered shape is asserted in `__init__` (`len(joints_names) == len(ControlType)`). |
| `muscles_names` | `tuple[str, ...]` | Muscle names read by `excitations()` (can be empty) |
| `max_torques` | `tuple[NDArray, ...]` (length 7) | Torque limits, same length-7/`ControlType`-grouped shape as `joints_names`. Build with `AnimatController.max_torques_from_control_types()`. |
| `substep` | bool | Whether to run at substep resolution |

!!! tip "Reading back your own grouped joint names"
    Once constructed, `self.joints_names[ControlType.POSITION]` **is** the
    list of position-controlled joint names for this animat — there's no
    need to separately recompute or cache a `self.position_joints` list in
    your own subclass the way the example below does for clarity; the base
    class already has it, pre-filtered and pre-ordered, right after
    `__init__`.

!!! bug "The base `AnimatController.from_options()` is a non-functional stub — you must override it"
    `AnimatController.from_options()` computes `joints_names` from
    `animat_options.morphology.joints` but then **discards it**, calling
    `cls(..., joints_names=[[]]*7, muscles_names=[], max_torques=[[]]*7)` —
    seven empty lists, unconditionally. Verified in
    `farms_core/model/control.py`: this is not a working default
    implementation, it's a placeholder that produces a controller with no
    joints wired to any control type. Every real controller —
    `AmphibiousController` included — overrides `from_options()` completely
    rather than calling `super().from_options()`. Don't rely on inheriting
    this method; write your own as shown below.

## How `ExperimentTask` actually calls into your controller

This is the part that's normally invisible from the YAML/controller-authoring
side. Verified against `farms_mujoco/simulation/task.py`:

1. At `initialize_episode()`, the task builds one `ctrl` index map per
   animat per `ControlType`, keyed by `prefix + joint_name` (the prefix
   disambiguates joints across multiple animats sharing one MJCF model).
2. Every `before_step()`, for each registered controller, the task checks
   `controller.joints_names[ControlType.POSITION]` — **only if that list is
   non-empty** does it call `controller.positions(...)` and write the
   returned `{joint: value}` dict into `physics.data.ctrl` (radians, no unit
   scaling). The same pattern repeats independently for `VELOCITY` (scaled
   by `units.angular_velocity`) and `TORQUE` (scaled by `units.torques`).
3. If `TORQUE` joints exist, `springrefs()`/`springcoefs()`/`dampingcoefs()`
   are **also** called that step (see the bug note above) and written into
   `physics.model.qpos_spring` / `jnt_stiffness` / `dof_damping` — note this
   mutates the **model**, not just `physics.data`, so a controller that
   changes stiffness at runtime is changing MuJoCo's compiled joint
   properties on the fly, not just commanding a target.
4. Every dispatch call passes `iteration=index` where
   `index = self.iteration % self.buffer_size` — the *ring-buffer* index,
   not necessarily the raw simulation step count. For `n_iterations <=
   buffer_size` (the normal case — see
   [`buffer_size` sizing](../tutorials/zbot-experiment.md#key-parameters-explained))
   these are identical; only relevant if you deliberately run with a buffer
   smaller than the run length.

For the Zbot specifically, `control_types: [position]` on every motor in
`animat_config.yaml` means only step 2's `POSITION` branch — and never the
`TORQUE`/spring branches — actually fires; `AmphibiousController`'s
`position_muscle` equation computes each target position from the CPG
oscillator + Ekeberg-muscle state (see
[Mathematical Models](../explanation/mathematical-models.md)) and returns it
from `positions()`.

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
