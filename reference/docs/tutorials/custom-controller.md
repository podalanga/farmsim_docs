# Write a custom controller

!!! info "Tutorial overview"
    - **Goal**: write a controller that sends a travelling wave down
      AmphiBot's body, with a crawling gait on land and a swimming gait in
      water, switching on the head's height.
    - **Level**: intermediate
    - **Time**: 45 minutes
    - **Prerequisites**: [Trace a simulation step](simulation-workflow.md);
      Python classes

## Background

A controller is a subclass of `farms_core.model.control.AnimatController`,
which is itself an `AnimatExtension`. It runs because it is listed in the
animat's `extensions:`. At each control step:

1. `before_step(task, action, physics)` advances the controller's internal
   state.
2. `positions()`, `velocities()`, `torques()` (and, with torque joints,
   `springrefs()`, `springcoefs()`, `dampingcoefs()`) return the targets as
   `dict[str, float]` (joint name to value). `excitations()` returns an
   array of muscle excitations, ordered like `muscles_names`.
3. `ExperimentTask` writes them to the MuJoCo actuators of the joints.

The finished controller is `examples/amphibot/controller/traveling_wave.py`.
This tutorial builds it step by step. The gait, for joint *i* of *n*:

$$
\theta_i(t) = A_i \sin\left(\varphi(t) - i\,\frac{2\pi w}{n}\right) + \theta_{turn},
\qquad
A_i = A_{head} + (A_{tail} - A_{head})\frac{i}{n-1}
$$

where $\varphi$ advances at the gait frequency and $w$ is the number of
waves along the body:

| Gait | Frequency | $A_{head}$ | $A_{tail}$ | Waves $w$ |
|---|---|---|---|---|
| Land (serpentine crawling) | 0.6 Hz | 0.45 rad | 0.45 rad | 1.0 |
| Water (anguilliform swimming) | 1.0 Hz | 0.20 rad | 0.55 rad | 0.8 |

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
  (see [Trace a simulation step](simulation-workflow.md#phase-5-the-simulation-loop)).

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

## Step 2: Create the controller from the options

Create `controller/traveling_wave.py` with the gait table and the class.
`from_options()` is called by `ExperimentTask` with the extension's
`config:` dictionary and the animat's options and data:

```python
import numpy as np

from farms_core.model.control import AnimatController, ControlType

GAITS = {
    'land': {'freq': 0.6, 'amp_head': 0.45, 'amp_tail': 0.45, 'waves': 1.0},
    'water': {'freq': 1.0, 'amp_head': 0.20, 'amp_tail': 0.55, 'waves': 0.8},
}
WATER_ENTER_Z, WATER_EXIT_Z = 0.015, 0.035  # [m] hysteresis band
BLEND_TAU = 1.0  # [s] time constant of the blend between gaits


class TravelingWaveController(AnimatController):
    """Open-loop travelling wave with land/water gait switching"""

    def __init__(self, animat_i, joints_names, max_torques, links_sensors,
                 head_index, mode='auto', turn=0.0):
        super().__init__(
            animat_i=animat_i,
            joints_names=joints_names,
            muscles_names=[],
            max_torques=max_torques,
        )
        self.links = links_sensors
        self.head_index = head_index
        self.mode = mode
        self.turn = turn
        self.medium = 'land' if mode == 'auto' else mode
        self.blend = 0.0 if self.medium == 'land' else 1.0  # 0: land, 1: water
        self.phase = 0.0

    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data,
                     animat_options):
        """Controller from the animat options (config: the YAML `config:`)"""
        # Only the body joints are commanded, the wheel joints roll freely
        motors = [
            motor for motor in animat_options.control.motors
            if not motor.passive.is_passive
        ]
        names = [motor.joint_name for motor in motors]
        control_types = {
            motor.joint_name: ControlType.from_string_list(motor.control_types)
            for motor in motors
        }
        links_names = animat_data.sensors.links.names
        return cls(
            animat_i=animat_i,
            joints_names=cls.joints_from_control_types(
                joints_names=names,
                joints_control_types=control_types,
            ),
            max_torques=cls.max_torques_from_control_types(
                joints_names=names,
                max_torques={
                    motor.joint_name: motor.limits_torque[1]
                    for motor in motors
                },
                joints_control_types=control_types,
            ),
            links_sensors=animat_data.sensors.links,
            head_index=list(links_names).index(config.get('head', 'head')),
            mode=config.get('mode', 'auto'),
            turn=config.get('turn', 0.0),
        )
```

Points to note:

- The passive wheel joints have motors too (with `passive.is_passive`
  true), handled by FarmSim. The controller leaves them out, so they roll
  freely.
- `animat_data.sensors.links` is the link sensor array, filled at every
  step. The controller keeps a reference to read the head's position later.

## Step 3: Advance the gait in before_step()

Keep the dynamics in `before_step()` and make `positions()` free of side
effects: `ExperimentTask` calls `positions()` right after `before_step()`.

```python
    def initialize_episode(self, task, physics):
        """Reset the gait at the start of an episode"""
        self.medium = 'land' if self.mode == 'auto' else self.mode
        self.blend = 0.0 if self.medium == 'land' else 1.0
        self.phase = 0.0

    def before_step(self, task, action, physics):
        """Advance the gait by one step (switching, blending, phase)"""
        timestep = physics.timestep()/task.units.seconds
        if self.mode == 'auto':
            # Head height at the last logged iteration
            z = self.links.com_position(
                max(task.iteration-1, 0), self.head_index,
            )[2]
            if self.medium == 'land' and z < WATER_ENTER_Z:
                self.medium = 'water'
            elif self.medium == 'water' and z > WATER_EXIT_Z:
                self.medium = 'land'
        target = 1.0 if self.medium == 'water' else 0.0
        self.blend += (target - self.blend)*min(1.0, timestep/BLEND_TAU)
        self.phase += 2*np.pi*self.gait()['freq']*timestep

    def gait(self):
        """Gait parameters, blended between land and water"""
        return {
            key: (
                (1 - self.blend)*GAITS['land'][key]
                + self.blend*GAITS['water'][key]
            )
            for key in GAITS['land']
        }
```

- `physics.timestep()` is the MuJoCo step in simulation units;
  `task.units.seconds` converts it to seconds.
- The phase is integrated rather than computed as `2*pi*f*t`, so that it
  stays continuous when the frequency changes between gaits.
- The two thresholds form a hysteresis band: the head bobbing at the
  surface does not make the gait flicker.
- `com_position(iteration, link)` reads the logged sensor array; the
  current iteration is written after the step, so read the previous one.

## Step 4: Return the joint targets

```python
    def positions(self, iteration, time, timestep):
        """Joint position targets [rad], from the state of before_step()"""
        gait = self.gait()
        joints = self.joints_names[ControlType.POSITION]
        n_joints = len(joints)
        dphi = 2*np.pi*gait['waves']/n_joints
        return {
            joint: (
                (
                    gait['amp_head']
                    + (gait['amp_tail'] - gait['amp_head'])*i/max(1, n_joints-1)
                )*np.sin(self.phase - i*dphi)
                + self.turn
            )
            for i, joint in enumerate(joints)
        }
```

`positions()` only returns the position-controlled joints; FarmSim sends
each target to the joint's position actuator, with the motor `gains` of the
animat file.

## Step 5: Register the controller

Add `controller/__init__.py` (it can be empty), so that `controller` is a
package. In the animat file, list the controller in `extensions:`, before
the swimming extension:

```yaml
# examples/amphibot/animat_config_wave.yaml (end of the file)
extensions:
- loader: controller.traveling_wave.TravelingWaveController
  config:
    mode: auto
    turn: 0.0
- loader: farms_mujoco.swimming.extension.SwimmingExtension
  config:
    water_properties: null
```

`loader` is the dotted path of the class. It is importable because
`run_sim.py` puts the experiment folder on `sys.path`. `config` is passed
as is to `from_options()`. In the example, `generate_config.py` writes this
block.

## Step 6: Run it

```bash
python run_sim.py --experiment_config experiment_config_wave.yaml
```

AmphiBot crawls with a wide, slow wave, about 0.2 m/s. When its head goes
below the water surface, the gait blends over about a second into the
faster swimming wave with a larger tail amplitude.

Try the options of the `config:` block:

- `mode: water`: the swimming gait everywhere. With the wheels, its faster
  wave also crawls faster on land (about 0.3 m/s instead of 0.2 m/s in the
  first 3 s), at the cost of a larger tail swing.
- `turn: 0.1`: a constant offset on every joint curves the body, and
  AmphiBot turns right (clockwise seen from above). A negative offset turns
  it left.

## Summary

| Step | What you do |
|---|---|
| 1 | Subclass `AnimatController` |
| 2 | Implement `from_options()`: select the joints, read the `config:` |
| 3 | Advance the internal state in `before_step()` |
| 4 | Return the targets from `positions()` (or `velocities()`, `torques()`) |
| 5 | List the controller in the animat's `extensions:` |

## Next steps

- [Tutorial 5: Swimming and buoyancy](swimming.md)
- [Write a controller (how-to)](../how-to/write-controller.md): more
  patterns, including sensor feedback
- [Configure the CPG network](../how-to/configure-cpg-network.md): the
  built-in amphibious CPG
- [Extension and controller design](../explanation/extension-design.md):
  the full lifecycle
