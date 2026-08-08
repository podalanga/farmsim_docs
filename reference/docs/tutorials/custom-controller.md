# Write a Custom Controller

This tutorial shows how to write a custom locomotion controller for a FARMS
animat. We'll trace the ZbotCPGController from the zbot_bout_glide experiment
as a concrete example.

## What is a controller?

A controller is an `AnimatExtension` subclass that also extends
`AnimatController`. It is registered as an animat extension in
`animat_config.yaml` via the `controller_loader` field. At each simulation step:

1. `before_step()` advances the controller's internal dynamics
2. `positions()`, `velocities()`, or `torques()` return joint targets as
   `dict[str, float]` (joint name → value)
3. The `ExperimentTask` maps these outputs to MuJoCo's `physics.data.ctrl`

## Step 1: Understand the AnimatController base class

`AnimatController` (in `farms_core/model/control.py`) provides:

- `__init__(self, animat_i, joints_names, muscles_names, max_torques, substep)`
  — stores joint/muscle names and torque limits
- `positions(iteration, time, timestep) -> dict[str, float]` — override to
  return position targets
- `velocities(iteration, time, timestep) -> dict[str, float]` — override to
  return velocity targets
- `torques(iteration, time, timestep) -> dict[str, float]` — override to
  return torque targets
- `before_step(task, action, physics)` — override to advance internal dynamics

The `ControlType` enum determines which output types each joint uses:

| Value | Name | Description |
|-------|------|-------------|
| 0 | `POSITION` | Position control |
| 1 | `VELOCITY` | Velocity control |
| 2 | `TORQUE` | Torque control |
| 3 | `SPRINGREF` | Spring reference (Ekeberg muscle) |
| 4 | `SPRINGCOEF` | Spring coefficient (Ekeberg muscle) |
| 5 | `DAMPINGCOEF` | Damping coefficient (Ekeberg muscle) |
| 6 | `MUSCLE` | Muscle activation |

## Step 2: Register your controller in YAML

In `animat_config.yaml`, set the `controller_loader` to the dotted path of your
controller class:

```yaml
control:
  controller_loader: controller.zbot_controller.ZbotCPGController
  sensors:
    joints:
      - joint_head_yaw
      - joint_0
      # ... more joints
  motors:
    - joint_name: joint_head_yaw
      control_types: ["position"]
      limits_torque: [-0.3, 0.3]
      gains: [1.0]
    # ... more motors
  # network: ... (CPG network, see CPG guide)
  muscles: []
  adhesions: []
  visuals: []
extensions:
  - loader: controller.zbot_controller.ZbotCPGController
    config:
      bout_duration_s: 0.8
      bout_interval_s: 2.0
      tail_frequency: 2.5
      tail_amplitude: 0.4
```

!!! note "Controller as extension"
    In the zbot experiments, the controller is registered both as the
    `controller_loader` (so `ExperimentTask` creates it) and as an entry in
    the top-level `extensions:` list (sibling to `control:`, not nested under
    it) so it receives `from_options()` with the config dict. The
    `ExperimentTask` detects controllers among extensions and handles them
    specially.

## Step 3: Implement the controller

Here is the structure of `ZbotCPGController` from
`experiments/zbot_bout_glide/controller/zbot_controller.py`:

```python
from farms_core.model.control import AnimatController

class ZbotCPGController(AnimatController):
    """CPG-based bout-and-glide controller for the Zbot robot."""

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                    animat_data, animat_options):
        """Factory method called by ExperimentTask."""
        # Read configuration from the config dict
        bout_duration = config['bout_duration_s']
        bout_interval = config['bout_interval_s']
        tail_freq = config['tail_frequency']
        tail_amp = config['tail_amplitude']

        # Build parameters
        params = ZbotCPGParameters.from_bout_timing(
            bout_duration_s=bout_duration,
            bout_interval_s=bout_interval,
            tail_frequency=tail_freq,
            tail_amplitude=tail_amp,
        )

        # Get joint names and control types from animat_options
        joints_names = animat_options.control.joints_names()
        joints_control_types = {
            motor.joint_name: ControlType.from_string_list(motor.control_types)
            for motor in animat_options.control.motors
        }

        # Call the parent constructor
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

        # Store internal state
        controller.params = params
        controller.cpg = SegmentalCPG(n_segments=params.n_segments)
        controller.data = animat_data
        # ... initialize bout gate, vSPN, etc.
        return controller
```

### Key implementation details

**`from_options()` signature** (from `AnimatExtension`):

```python
@classmethod
def from_options(cls, config, experiment_options, animat_i,
                animat_data, animat_options):
```

- `config` — the `config` dict from the extension entry in YAML
- `experiment_options` — full `ExperimentOptions` object
- `animat_i` — index of this animat in the experiment
- `animat_data` — `AnimatData` with pre-allocated sensor arrays
- `animat_options` — `AnimatOptions` (or subclass) for this animat

**`before_step()` — advance internal dynamics:**

```python
def before_step(self, task, action, physics):
    iteration = task.iteration
    timestep = physics.timestep() / task.units.seconds
    time = physics.time() / task.units.seconds

    # Advance CPG, bout gate, and vSPN
    self.cpg.step(timestep)
    self.bout_gate.step(timestep)
    self.leaky_integrator.step(self.cpg.output)
```

**`positions()` — return joint position targets:**

```python
def positions(self, iteration, time, timestep):
    """Return position targets for all position-controlled joints."""
    motor_output = self.cpg.output * self.bout_gate.output
    return {
        joint_name: float(motor_output[i] * self.params.tail_amplitude)
        for i, joint_name in enumerate(self.position_joints)
    }
```

## Step 4: Control types and joint mapping

Each motor in the YAML declares its `control_types` as a list of strings.
`ControlType.from_string_list()` converts these to `ControlType` enum values.

The `AnimatController` base class provides helper methods:

- `joints_from_control_types(joints_names, joints_control_types)` — returns
  the flat list of joint names grouped by control type
- `max_torques_from_control_types(joints_names, max_torques, joints_control_types)`
  — returns torque limits aligned with the control type grouping

Your `positions()`, `velocities()`, and `torques()` methods only need to return
dicts for the joints that use the corresponding control type.

## Step 5: Run your controller

```bash
cd experiments/zbot_bout_glide
python run_sim.py --experiment_config experiment_config.yaml
```

If a display is available, the MuJoCo viewer will open and you can watch your
controller drive the robot.

## Summary

| Step | What you do |
|------|-------------|
| 1 | Subclass `AnimatController` |
| 2 | Implement `from_options()` to read YAML config |
| 3 | Implement `before_step()` to advance internal dynamics |
| 4 | Implement `positions()` / `velocities()` / `torques()` to return targets |
| 5 | Register in `animat_config.yaml` via `controller_loader` |

## Next steps

- [Write a Controller (How-to)](../how-to/write-controller.md) — more detailed
  patterns and API reference
- [Configure CPG Network Parameters](../how-to/configure-cpg-network.md) —
  configure the built-in amphibious CPG network
- [Extension and Controller Design](../explanation/extension-design.md) —
  understand the full lifecycle
