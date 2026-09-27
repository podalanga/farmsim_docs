# Custom CPG Controller: step-by-step guide

This guide builds a Central Pattern Generator (CPG) controller for the Zbot
from scratch, by subclassing `AnimatController`: first an open-loop
travelling sine wave, then a network of coupled phase oscillators.

!!! note "Source files"
    - `farms_core/farms_core/model/control.py`: the `AnimatController` base class
    - `experiments/zbot_bout_glide/controller/`: the Zbot controllers
      (`zbot_controller.py`, `zbot_controller_sine.py`), a complete example
      of what this guide builds
    - `farms_amphibious/farms_amphibious/control/amphibious.py`: the built-in
      `AmphibiousController`

---

## Background: What Is a CPG?

A Central Pattern Generator is a neural circuit that produces rhythmic
output without rhythmic input. For the Zbot, it gives the joint commands
of a wave travelling from head to tail, the undulation that propels the
robot.

The built-in `AmphibiousController` (used by `experiments/zbot_swimming`)
implements a full CPG network configured in YAML (see
[Swimming Experiment](zbot-experiment.md#cpg-network)). Writing your own
controller is useful for simpler models, custom dynamics, sensory feedback
or reinforcement learning policies.

---

## Step 1, The `AnimatController` Interface

A controller is an animat extension. The methods you usually implement:

```python
from farms_core.model.control import AnimatController, ControlType

class MyController(AnimatController):

    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
        """Create the controller (called once, when the task is created)"""

    def initialize_episode(self, task, physics):
        """Reset the internal state at the start of an episode"""

    def before_step(self, task, action, physics):
        """Advance the internal dynamics (called before each MuJoCo step)"""

    def positions(self, iteration, time, timestep) -> dict[str, float]:
        """Position targets [rad] of the position controlled joints"""
```

`velocities()`, `torques()`, `springrefs()`, `springcoefs()`,
`dampingcoefs()` and `excitations()` are the equivalents for the other
control types. The base class implementations return empty dictionaries,
and the base `from_options()` creates a controller without any joint, so
you must override it.

`ExperimentTask` calls `before_step()` and then the command methods at
every environment step when the controller is created with `substep=True`
(once per iteration otherwise). The arguments of `positions()` are:

- `iteration`: the index of the current iteration in the sensor buffers
  (already taken modulo `runtime.buffer_size`)
- `time`: the simulation time [s]
- `timestep`: the iteration duration (`physics.timestep`)

### The `joints_names` tuple

`self.joints_names` has one list of joint names per `ControlType`:

| Index | `ControlType` | Command |
|-------|---------------|---------|
| 0 | `ControlType.POSITION` | Joint angles [rad] |
| 1 | `ControlType.VELOCITY` | Joint velocities [rad/s] |
| 2 | `ControlType.TORQUE` | Torques [N.m] |
| 3 | `ControlType.SPRINGREF` | Spring reference positions |
| 4 | `ControlType.SPRINGCOEF` | Spring stiffnesses |
| 5 | `ControlType.DAMPINGCOEF` | Damping coefficients |
| 6 | `ControlType.MUSCLE` | Muscle excitations |

A joint is in a list when its motor has that type in `control_types`
(`animat_config.yaml`). `AnimatController.joints_from_control_types()`
and `AnimatController.max_torques_from_control_types()` build these
tuples from the motor options.

---

## Step 2, An Open-Loop Sine CPG

Start from a copy of `experiments/zbot_bout_glide`, which already contains
a `run_sim.py` and a `controller/` package, and add
`controller/sine_cpg.py`:

```python
# controller/sine_cpg.py
"""Open-loop travelling sine wave for the Zbot"""

import numpy as np
from farms_core.model.control import AnimatController, ControlType


def motors_control(animat_options):
    """joints_names and max_torques tuples from the motor options"""
    motors = animat_options.control.motors
    all_joints = [motor.joint_name for motor in motors]
    control_types = {
        motor.joint_name: ControlType.from_string_list(motor.control_types)
        for motor in motors
    }
    joints_names = AnimatController.joints_from_control_types(
        joints_names=all_joints,
        joints_control_types=control_types,
    )
    max_torques = AnimatController.max_torques_from_control_types(
        joints_names=all_joints,
        max_torques={motor.joint_name: motor.limits_torque[1] for motor in motors},
        joints_control_types=control_types,
    )
    return joints_names, max_torques


class ZbotSineCPG(AnimatController):
    """Travelling sine wave over the body joints"""

    def __init__(self, animat_i, joints_names, max_torques,
                 frequency=1.0, amplitude=0.4, phase_lag=np.pi/3):
        super().__init__(
            animat_i=animat_i,
            joints_names=joints_names,
            muscles_names=(),
            max_torques=max_torques,
            substep=True,
        )
        self.frequency = frequency  # [Hz]
        self.amplitude = amplitude  # [rad]
        self.phase_lag = phase_lag  # [rad] between neighbouring joints

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                     animat_data, animat_options):
        """config is the `config:` dictionary of the extension entry"""
        joints_names, max_torques = motors_control(animat_options)
        return cls(
            animat_i=animat_i,
            joints_names=joints_names,
            max_torques=max_torques,
            frequency=config.get('frequency', 1.0),
            amplitude=config.get('amplitude', 0.4),
            phase_lag=config.get('phase_lag', np.pi/3),
        )

    def positions(self, iteration, time, timestep):
        """Joint i lags joint i-1 by phase_lag: a head to tail wave"""
        return {
            joint: self.amplitude*np.sin(
                2*np.pi*self.frequency*time - i*self.phase_lag
            )
            for i, joint in enumerate(self.joints_names[ControlType.POSITION])
        }
```

With a phase lag of $\pi/3$, the 6 joints span one full wavelength.

---

## Step 3, Register the Controller

A controller only runs if it is listed in the animat's `extensions:`. In
`animat_config.yaml`, replace the controller entry (the
`AmphibiousController` in `zbot_swimming`, the `ZbotCPGController` in
`zbot_bout_glide`) with yours, and keep the `SwimmingExtension`:

```yaml
extensions:
  - loader: controller.sine_cpg.ZbotSineCPG
    config:
      frequency: 1.5      # [Hz]
      amplitude: 0.35     # [rad]
      phase_lag: 1.0472   # [rad]
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null
```

The motors must be position controlled:

```yaml
control:
  motors:
    - joint_name: joint_1
      control_types: [position]
      limits_torque: [-10.0, 10.0]
      gains: [3.0, 0.01, 0]
      equation: position
    # ... joint_2 through joint_6 ...
```

!!! note "What is ignored"
    `equation`, `control.network` and `control.muscles` are only read by
    `AmphibiousController`, and `control.controller_loader` is not used.
    With your own controller they have no effect and can stay as they are
    (`zbot_bout_glide` keeps them and sets `equation: position`).

### Run

```bash
cd experiments/zbot_my_cpg
python run_sim.py --experiment_config experiment_config.yaml
```

`run_sim.py` puts the experiment folder on `sys.path`, which makes
`controller.sine_cpg` importable. With the `farmsim` command instead,
set `PYTHONPATH=.` so that Python finds the `controller` package.

---

## Step 4, Add Sensory Feedback

`from_options()` receives the `AnimatData` of the animat. Keep a reference
to it and read the sensor arrays at the `iteration` index:

```python
from farms_core.sensors.sensor_convention import sc


class ZbotFeedbackCPG(ZbotSineCPG):
    """Sine CPG reading the joint and fluid sensors"""

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                     animat_data, animat_options):
        controller = super().from_options(
            config, experiment_options, animat_i, animat_data, animat_options,
        )
        controller.animat_data = animat_data
        return controller

    def positions(self, iteration, time, timestep):
        sensors = self.animat_data.sensors
        joint_positions = sensors.joints.array[iteration, :, sc.joint_position]
        joint_velocities = sensors.joints.array[iteration, :, sc.joint_velocity]
        # Fluid wrench of each link [Fx, Fy, Fz, Tx, Ty, Tz], world frame
        fluid_forces = sensors.xfrc.array[iteration, :, :]
        commands = super().positions(iteration, time, timestep)
        # ... modify the commands from the sensor values ...
        return commands
```

The sensor arrays have shape `(buffer_size, n_elements, n_columns)`; see
[Add and Configure Sensors](../how-to/configure-sensors.md) for the
columns of each category. The order of the elements is the order of the
names in `control.sensors` (`sensors.joints.names`).

---

## Step 5, A Coupled Oscillator CPG

A network of coupled phase oscillators synchronises by itself, recovers
from perturbations, and can take sensory feedback in its equations. This
controller has, like the built-in network, one left and one right
oscillator per joint:

$$
\dot\theta_i = \omega + \sum_j A_j w_{ij} \sin(\theta_j - \theta_i - \varphi_{ij}),
\qquad
\dot A_i = a (R - A_i)
$$

At steady state, $\theta_j - \theta_i = \varphi_{ij}$. For a wave from
head to tail, joint $n$ must lead joint $n+1$ by the phase lag, so
$\varphi_{n+1,n} = +\text{lag}$ and $\varphi_{n,n+1} = -\text{lag}$.

```python
# controller/ode_cpg.py
"""Coupled phase oscillator CPG for the Zbot"""

import numpy as np
from farms_core.model.control import AnimatController, ControlType

from .sine_cpg import motors_control


def coupling_matrices(n_joints, weight, phase_lag):
    """Weights w[i, j] and phase biases phi[i, j], oscillator 2*joint+side"""
    n_osc = 2*n_joints
    w, phi = np.zeros((n_osc, n_osc)), np.zeros((n_osc, n_osc))
    for joint in range(n_joints):
        left, right = 2*joint, 2*joint + 1
        # Left and right in anti-phase
        w[left, right] = w[right, left] = weight
        phi[left, right] = phi[right, left] = np.pi
        if joint < n_joints - 1:
            for osc, nxt in ((left, left + 2), (right, right + 2)):
                w[osc, nxt] = w[nxt, osc] = weight
                phi[nxt, osc] = phase_lag    # osc leads nxt
                phi[osc, nxt] = -phase_lag
    return w, phi


class ZbotOdeCPG(AnimatController):
    """Coupled phase oscillators, integrated in before_step()"""

    def __init__(self, animat_i, joints_names, max_torques, frequency=1.0,
                 amplitude=0.4, phase_lag=np.pi/3, weight=30.0, rate=3.0):
        super().__init__(
            animat_i=animat_i,
            joints_names=joints_names,
            muscles_names=(),
            max_torques=max_torques,
            substep=True,
        )
        n_joints = len(joints_names[ControlType.POSITION])
        self.omega = 2*np.pi*frequency  # [rad/s]
        self.nominal_amplitude = amplitude
        self.rate = rate
        self.w, self.phi = coupling_matrices(n_joints, weight, phase_lag)
        self.phases = np.zeros(2*n_joints)
        self.amplitudes = np.zeros(2*n_joints)

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                     animat_data, animat_options):
        joints_names, max_torques = motors_control(animat_options)
        return cls(
            animat_i=animat_i,
            joints_names=joints_names,
            max_torques=max_torques,
            **config,  # frequency, amplitude, phase_lag, weight, rate
        )

    def initialize_episode(self, task, physics):
        rng = np.random.default_rng(0)
        self.phases[:] = rng.uniform(0, 2*np.pi, self.phases.size)
        self.amplitudes[:] = 0

    def before_step(self, task, action, physics):
        """Explicit Euler step of the oscillators over one environment step"""
        dt = task.timestep/task.cb_sub_steps  # [s]
        delta = self.phases[None, :] - self.phases[:, None] - self.phi
        dphases = self.omega + np.sum(
            self.w*self.amplitudes[None, :]*np.sin(delta), axis=1,
        )
        self.amplitudes += dt*self.rate*(self.nominal_amplitude - self.amplitudes)
        self.phases += dt*dphases

    def positions(self, iteration, time, timestep):
        """Joint command: difference of the left and right outputs"""
        left = self.amplitudes[0::2]*np.sin(self.phases[0::2])
        right = self.amplitudes[1::2]*np.sin(self.phases[1::2])
        commands = 0.5*(left - right)
        return dict(zip(self.joints_names[ControlType.POSITION], commands))
```

Points to note:

- The dynamics are in `before_step()`, and `positions()` only reads the
  state. `before_step()` runs at every environment step, which lasts
  `task.timestep/task.cb_sub_steps` seconds (`physics.timestep` of the
  simulation file divided by `cb_sub_steps`).
- The left and right oscillators are in anti-phase, so `0.5*(left - right)`
  oscillates with the amplitude of one oscillator.
- An explicit Euler step is enough for these smooth dynamics at this step
  size. `AmphibiousController` uses SciPy's `dopri5` instead.

The extension entry:

```yaml
extensions:
  - loader: controller.ode_cpg.ZbotOdeCPG
    config:
      frequency: 1.0          # [Hz]
      amplitude: 0.4          # [rad] nominal amplitude
      phase_lag: 1.0472       # [rad] between neighbouring joints
      weight: 30.0            # Coupling strength
      rate: 3.0               # Amplitude convergence rate [1/s]
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null
```

| Feature | Sine | Coupled oscillators |
|---------|------|---------------------|
| Phase coupling between joints | No | Yes |
| Smooth start (amplitude ramp) | No | Yes, at rate `rate` |
| Recovers from perturbations | No | Yes |
| Sensory feedback | In the commands | In the commands or the equations |

---

## Step 6, Drive Modulation

In the built-in network, a descending drive $d$ sets the frequency and
amplitude through linear functions (see
[Swimming Experiment](zbot-experiment.md#oscillators)). The same can be
added to `ZbotOdeCPG`:

```python
def set_drive(self, drive, frequency_gain=1.5708, amplitude_gain=0.15):
    """Frequency [rad/s] and nominal amplitude from a drive level"""
    self.omega = frequency_gain*drive
    self.nominal_amplitude = min(amplitude_gain*drive, 0.75)
```

With `drive=4`, the oscillators run at $2\pi$ rad/s (1 Hz) with a
nominal amplitude of 0.6. Call it from `before_step()` to change the gait
during the simulation, for example from sensor values.

---

## Common Pitfalls

| Problem | Cause | Fix |
|---------|-------|-----|
| The robot does not move | The controller is not in `extensions:` | Add it there. `control.controller_loader` is not used |
| `ModuleNotFoundError` for your controller | The experiment folder is not on `sys.path` | Run with `python run_sim.py`, or `PYTHONPATH=.` with `farmsim` |
| `KeyError` in `positions()` | Wrong joint name | Print `self.joints_names[ControlType.POSITION]` |
| No joint moves | `from_options()` not overridden | The base implementation creates a controller without joints |
| Joints move but not those expected | The motors are not `position` controlled | Set `control_types: [position]` |
| Wrong frequency | Hz and rad/s mixed up | $\omega = 2\pi f$ |
| The wave travels from tail to head | Sign of the phase biases | Joint $n$ must lead joint $n+1$ |

---

## File Structure of a Custom Experiment

```text
experiments/zbot_my_cpg/
├── experiment_config.yaml      # Copied from zbot_bout_glide
├── simulation_config.yaml      # Copied, adjust runtime.n_iterations
├── animat_config.yaml          # Controller entry in extensions:
├── arena_config.yaml           # Copied unchanged
├── run_sim.py                  # Copied unchanged
├── controller/
│   ├── __init__.py
│   ├── sine_cpg.py             # ZbotSineCPG (Step 2)
│   └── ode_cpg.py              # ZbotOdeCPG (Step 5)
└── analysis.py                 # Copied unchanged
```

---

## See Also

- [Write a Custom Controller](custom-controller.md): the structure of `ZbotCPGController`
- [`AnimatController` API](../reference/core/core-control.md)
- [Mathematical Models](../explanation/mathematical-models.md): the CPG and Ekeberg equations
- [`AmphibiousController` API](../reference/amphibious/amphibious-controller.md)
- [Swimming Experiment](zbot-experiment.md): the YAML files
