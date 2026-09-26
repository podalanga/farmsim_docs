# Write a Custom Controller

This tutorial shows how to write a locomotion controller for a FARMS animat,
using the `ZbotCPGController` of `experiments/zbot_bout_glide` as the worked
example.

## What is a controller?

A controller is a subclass of `farms_core.model.control.AnimatController`,
which is itself an `AnimatExtension`. It runs because it is listed in the
animat's `extensions:`. At each control step:

1. `before_step(task, action, physics)` advances the controller's internal
   dynamics.
2. `positions()`, `velocities()`, `torques()` (and, with torque joints,
   `springrefs()`, `springcoefs()`, `dampingcoefs()`) return the targets as
   `dict[str, float]` (joint name to value). `excitations()` returns an
   array of muscle excitations, ordered like `muscles_names`.
3. `ExperimentTask` writes them to the MuJoCo actuators
   (`physics.data.ctrl`) of the joints.

## Step 1: The AnimatController base class

```python
AnimatController.__init__(
    self, animat_i, joints_names, muscles_names, max_torques, substep=True,
)
```

- `joints_names`: a tuple with one list of joint names per `ControlType`
  (7 entries), telling which joints use which kind of control.
- `max_torques`: a tuple with one array of torque limits per `ControlType`.
- `substep`: whether `before_step()` also runs on the environment substeps
  (see [Trace a Simulation Step](simulation-workflow.md#phase-5-the-simulation-loop)).

`ControlType` (`farms_core.model.control`) lists the kinds of control:

| Value | Name | Returned by |
|---|---|---|
| 0 | `POSITION` | `positions()` |
| 1 | `VELOCITY` | `velocities()` |
| 2 | `TORQUE` | `torques()` |
| 3 | `SPRINGREF` | `springrefs()` |
| 4 | `SPRINGCOEF` | `springcoefs()` |
| 5 | `DAMPINGCOEF` | `dampingcoefs()` |
| 6 | `MUSCLE` | `excitations()` |

Two static helpers build the per-type tuples from the motor options:
`AnimatController.joints_from_control_types()` and
`AnimatController.max_torques_from_control_types()`.

## Step 2: Declare the motors and register the controller

In `animat_config.yaml`, each actuated joint has a motor, whose
`control_types` decide which method of your controller drives it:

```yaml
control:
  motors:
  - joint_name: joint_1
    control_types:
    - position
    limits_torque:
    - -10.0
    - 10.0
    gains:
    - 3.0
    - 0.01
    - 0
extensions:
- loader: controller.zbot_controller.ZbotCPGController
  config:
    swimming_mode: bout_and_glide
    bout_duration_s: 5.0
    bout_interval_s: 2.0
    tail_amplitude: 1.0
    tail_frequency: 1.0
- loader: farms_mujoco.swimming.extension.SwimmingExtension
  config:
    water_properties: null
```

`loader` is the dotted path of the class. It is importable because
`run_sim.py` puts the experiment folder on `sys.path`, where the
`controller/` package lives. `config` is passed as is to the class's
`from_options()`.

!!! note "`control.controller_loader`"
    The zbot configurations also set `control.controller_loader`. This option
    is parsed but not used to create the controller: only the `extensions:`
    entry matters.

## Step 3: Implement the controller

The structure of `ZbotCPGController`
(`experiments/zbot_bout_glide/controller/zbot_controller.py`), shortened:

```python
import numpy as np
from farms_core.model.control import AnimatController, ControlType


class ZbotCPGController(AnimatController):
    """Zbot segmental CPG controller"""

    def __init__(self, animat_i, joints_names, muscles_names, max_torques,
                 params, substep=True):
        super().__init__(
            animat_i=animat_i,
            joints_names=joints_names,
            muscles_names=muscles_names,
            max_torques=max_torques,
            substep=substep,
        )
        self.params = params
        self.cpg = SegmentalCPG(params.n_segments, params.cpg_frequency)
        self.motor_outputs = np.zeros(params.n_segments)
        # ... bout gate, vSPN and leaky integrator

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                     animat_data, animat_options):
        """Called by ExperimentTask with the extension's config"""
        motors = animat_options.control.motors
        all_joints = [motor.joint_name for motor in motors]
        joints_control_types = {
            motor.joint_name: ControlType.from_string_list(motor.control_types)
            for motor in motors
        }
        joints_names = cls.joints_from_control_types(
            joints_names=all_joints,
            joints_control_types=joints_control_types,
        )
        max_torques = cls.max_torques_from_control_types(
            joints_names=all_joints,
            max_torques={
                motor.joint_name: motor.limits_torque[1] for motor in motors
            },
            joints_control_types=joints_control_types,
        )
        config = dict(config)
        params = ZbotCPGParameters.from_bout_timing(
            bout_duration_s=config.pop('bout_duration_s'),
            bout_interval_s=config.pop('bout_interval_s'),
            tail_frequency=config.pop('tail_frequency'),
            tail_amplitude=config.pop('tail_amplitude', 1.0),
            **config,  # e.g. swimming_mode
        )
        return cls(
            animat_i=animat_i,
            joints_names=joints_names,
            muscles_names=(),
            max_torques=max_torques,
            params=params,
        )

    def initialize_episode(self, task, physics):
        """Reset the internal state at the start of an episode"""
        self.cpg.reset()
        self.motor_outputs[:] = 0.0

    def before_step(self, task, action, physics):
        """Advance the internal dynamics by one step"""
        timestep = physics.timestep()/task.units.seconds
        self.step(timestep)  # Updates self.motor_outputs

    def positions(self, iteration, time, timestep):
        """Joint position targets [rad], computed in before_step()"""
        return dict(zip(
            self.joints_names[ControlType.POSITION],
            self.motor_outputs,
        ))
```

Points to note:

- `from_options()` has the signature of `AnimatExtension.from_options()`:
  `config` (the YAML `config:` dictionary), `experiment_options`,
  `animat_i` (index of the animat), `animat_data` (its `AnimatData`, with the
  sensor arrays) and `animat_options`.
- Keep the dynamics in `before_step()` and make `positions()` side-effect
  free: `ExperimentTask` calls it right after `before_step()`.
- `physics.timestep()` is the MuJoCo step, in simulation units
  (`task.units.seconds` converts it to seconds). With `substep=True`,
  `before_step()` runs at every environment step, which is one MuJoCo step
  when `physics.num_sub_steps` is 1 (as for the Zbot). In general, an
  environment step lasts `task.timestep/task.cb_sub_steps`.
- `positions()` only needs to return the joints of the corresponding control
  type; the others are ignored.

## Step 4: Run your controller

```bash
cd experiments/zbot_bout_glide
python run_sim.py --experiment_config experiment_config.yaml
```

The MuJoCo viewer opens unless `runtime.headless` is true in
`simulation_config.yaml`.

## Summary

| Step | What you do |
|---|---|
| 1 | Subclass `AnimatController` |
| 2 | Declare motors and their `control_types`, list the controller in `extensions:` |
| 3 | Implement `from_options()` to read the `config:` dictionary |
| 4 | Implement `before_step()` to advance the internal dynamics |
| 5 | Implement `positions()` / `velocities()` / `torques()` to return targets |

## Next steps

- [Write a Controller (How-to)](../how-to/write-controller.md): more
  patterns, including sensor feedback
- [Configure CPG Network Parameters](../how-to/configure-cpg-network.md):
  the built-in amphibious CPG network
- [Extension and Controller Design](../explanation/extension-design.md): the
  full lifecycle
