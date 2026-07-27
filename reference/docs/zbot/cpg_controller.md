# Custom CPG Controller

This guide shows you how to implement a **Central Pattern Generator (CPG)** controller for the Zbot from scratch by inheriting from `AnimatController`. By the end you will have a working sine-wave CPG that you can extend with coupling, sensory feedback, and drive modulation.

---

## Background: What Is a CPG?

A **Central Pattern Generator** is a neural circuit that produces rhythmic output without requiring rhythmic input. In the Zbot context, it generates coordinated sinusoidal commands for each joint that create a travelling wave — the undulation that propels the robot through water.

The built-in `AmphibiousController` implements a full CPG with ODE-integrated phase oscillators and the Ekeberg muscle model. This guide shows you how to build a simpler open-loop CPG yourself, and then progressively extend it.

---

## When to Use `AnimatController` vs `AmphibiousController`

| Goal | Recommended Base Class |
|------|----------------------|
| Simple sine wave (open-loop) | `AnimatController` |
| PD control with custom logic | `AnimatController` |
| Closed-loop with sensor feedback | `AnimatController` |
| Production CPG (full FARMS network) | `AmphibiousController` |
| Reinforcement learning policy | `AnimatController` |
| Custom ODE-based CPG | `AnimatController` |

---

## Step 1 — Understand the `AnimatController` Interface

`AnimatController` is an abstract base class in `farms_core.model.control`. You must implement these methods:

```python
from farms_core.model.control import AnimatController, ControlType

class MyController(AnimatController):

    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
        """Factory method. Called once by farms_sim to instantiate your controller."""
        ...

    def positions(self, iteration: int, time: float, timestep: float) -> dict[str, float]:
        """Return {joint_name: target_position_rad} for position-controlled joints."""
        ...

    def torques(self, iteration: int, time: float, timestep: float) -> dict[str, float]:
        """Return {joint_name: torque_Nm} for torque-controlled joints."""
        ...
```

### The `joints_names` Tuple

`self.joints_names` is a **7-tuple of lists**, indexed by `ControlType`:

| Index | `ControlType` constant | Controls |
|-------|----------------------|---------|
| `[0]` | `ControlType.POSITION` | Target joint angles (rad) |
| `[1]` | `ControlType.VELOCITY` | Target joint velocities (rad/s) |
| `[2]` | `ControlType.TORQUE` | Direct torques (Nm) |
| `[3]` | `ControlType.SPRINGREF` | Spring equilibrium position |
| `[4]` | `ControlType.SPRINGCOEF` | Dynamic stiffness coefficient |
| `[5]` | `ControlType.DAMPINGCOEF` | Dynamic damping coefficient |
| `[6]` | `ControlType.MUSCLE` | Muscle excitation levels |

!!! tip
    Always use `ControlType.POSITION` (not the raw integer `0`) — it makes your code readable and robust against future enum changes.

---

## Step 2 — Implement a Simple Open-Loop Sine CPG

Create a new file in the experiment directory:

```bash
# Inside the container
touch /app/experiments/zbot_swimming/my_cpg_controller.py
```

```python
# experiments/zbot_swimming/my_cpg_controller.py
"""Simple open-loop CPG controller for the Zbot.

Generates a travelling sine wave across all body joints to produce
anguilliform swimming. Each joint has a progressive phase lag relative
to the head to create a posterior-directed travelling wave.

Zbot segmental CPG controller.
Reference (C++) -> Python mapping
----------------------------------
zbot::CPG                  -> SegmentalCPG
zbot::ExponentialFilter    -> ExponentialFilter
zbot::LeakyIntegrator      -> LeakyIntegrator
zbot::ControllerParameters -> ZbotCPGParameters
zbot::Controller           -> ZbotCPGController
"""

from enum import Enum, IntEnum

import numpy as np

from farms_core.options import Options
from farms_core.model.control import AnimatController, ControlType


# --------------------------------------------------------------------------
# Enums
# --------------------------------------------------------------------------

class SwimmingMode(str, Enum):
    """Swimming behaviour mode (zbot::SwimmingMode)"""
    BOUT_AND_GLIDE = 'bout_and_glide'
    CONTINUOUS = 'continuous'


class Side(IntEnum):
    """Left/right side of the body (zbot::Side)"""
    LEFT = 0
    RIGHT = 1
    NONE = 2


class ControllerState(IntEnum):
    """Controller state (zbot::Controller::State)"""
    SWIMMING = 0
    GLIDING = 1


# --------------------------------------------------------------------------
# Parameters
# --------------------------------------------------------------------------

class ZbotCPGParameters(Options):
    """Zbot CPG controller parameters (zbot::ControllerParameters)

    Subclasses ``farms_core.options.Options``, so instances can be loaded
    and saved as YAML the same way as any other FARMS options object
    (``ZbotCPGParameters.load(path)`` / ``params.save(path)``).
    """

    def __init__(self, **kwargs):
        super().__init__()
        self.n_segments: int = kwargs.pop('n_segments', 6)
        self.swimming_mode = SwimmingMode(
            kwargs.pop('swimming_mode', SwimmingMode.BOUT_AND_GLIDE)
        )

        # Leaky integrator (bout-onset trigger)
        self.leaky_threshold: float = kwargs.pop('leaky_threshold', 100.0)
        # Rate at which the leaky integrator accumulates during gliding,
        # in [threshold-units / s]. Equivalent to the C++ per-tick
        # `descendingInput` but expressed as a continuous rate so it does
        # not depend on a fixed hardware control period.
        self.descending_input_rate: float = kwargs.pop(
            'descending_input_rate', 20,
        )
        #firing rate changed from 0.4 to 20 to make the trigger often ---> Joshua

        # CPG amplitude envelope (bout gate)
        self.cpg_initial_amplitude: float = kwargs.pop(
            'cpg_initial_amplitude', 1.25,
        )
        self.cpg_decay_factor: float = kwargs.pop('cpg_decay_factor', 0.63)
        self.cpg_integration_weight: float = kwargs.pop(
            'cpg_integration_weight', 0.975,
        )
        self.cpg_gliding_amplitude_threshold: float = kwargs.pop(
            'cpg_gliding_amplitude_threshold', 0.1,
        )

        # CPG oscillator network
        self.cpg_frequency: float = kwargs.pop('cpg_frequency', 1.0)

        # vSPN (turning drive envelope)
        self.vspn_initial_amplitude: float = kwargs.pop(
            'vspn_initial_amplitude', 1.25,
        )
        self.vspn_decay_factor: float = kwargs.pop('vspn_decay_factor', 0.63)
        self.vspn_integration_weight: float = kwargs.pop(
            'vspn_integration_weight', 0.95,
        )

        # Motor neurons (sigmoid layer)
        self.motor_neuron_omega: float = kwargs.pop('motor_neuron_omega', 1.4)
        self.motor_neuron_bias: float = kwargs.pop('motor_neuron_bias', 2.0)

        # Motor outputs
        self.tail_amplitude_ratio: float = kwargs.pop(
            'tail_amplitude_ratio', 1.0,
        )
        self.motor_amplitude_biases: list = list(kwargs.pop(
            'motor_amplitude_biases',
            [0.203, 0.364, 0.481, 0.594, 0.696, 1.0],
        ))
        # Scaling carried over from the original OMR paper code (1.5*1.5)
        self.motor_output_scaling: float = kwargs.pop(
            'motor_output_scaling', 1.5*1.5,
        )

        if kwargs.pop('strict', True) and kwargs:
            raise ValueError(f'Unknown ZbotCPGParameters keys: {list(kwargs)}')

        assert len(self.motor_amplitude_biases) == self.n_segments, (
            f'Expected {self.n_segments} motor_amplitude_biases '
            f'(one per segment), got {len(self.motor_amplitude_biases)}'
        )

    @classmethod
    def from_bout_timing(
            cls,
            bout_duration_s: float,
            bout_interval_s: float,
            tail_frequency: float,
            tail_amplitude: float = 1.0,
            **kwargs,
    ) -> 'ZbotCPGParameters':
        """Build parameters from high-level bout-and-glide timing.

        Mirrors zbot::ControllerParametersBuilder's
        setBoutDuration/setBoutInterval/setTailBeatingFrequency/
        setTailBeatingAmplitude convenience setters.
        """
        kwargs.setdefault('swimming_mode', SwimmingMode.BOUT_AND_GLIDE)
        params = cls(**kwargs)
        params.cpg_frequency = tail_frequency
        params.tail_amplitude_ratio = tail_amplitude
        params.cpg_decay_factor = (
            params.cpg_gliding_amplitude_threshold ** (1.0/bout_duration_s)
        )
        params.descending_input_rate = params.leaky_threshold/bout_interval_s
        return params


# --------------------------------------------------------------------------
# Stateful building blocks
# --------------------------------------------------------------------------

class ExponentialFilter:
    """First-order IIR low-pass filter driven by a decaying exponential
    (zbot::ExponentialFilter)"""

    def __init__(self, initial_value: float, decay_factor: float, beta: float):
        self.initial_value = initial_value
        self.decay_factor = decay_factor
        self.beta = beta
        self.time: float = 0.0
        self.value: float = 0.0
        self.exponential_value: float = 0.0
        self.reset()

    def reset(self):
        """Reset filter state (zbot::ExponentialFilter::reset)"""
        self.time = 0.0
        self.value = 0.0
        self.exponential_value = self.initial_value

    def step(self, timestep: float):
        """Advance the filter by one control step
        (zbot::ExponentialFilter::update)"""
        self.time += timestep
        self.exponential_value = (
            self.initial_value*self.decay_factor**self.time
        )
        self.value = (
            self.beta*self.value
            + (1 - self.beta)*self.exponential_value
        )


class LeakyIntegrator:
    """Accumulates a drive until a threshold is reached
    (zbot::LeakyIntegrator)"""

    def __init__(self, threshold: float):
        self.threshold = threshold
        self.value: float = 0.0

    def reset(self):
        """Reset integrator state (zbot::LeakyIntegrator::reset)"""
        self.value = 0.0

    def step(self, drive: float):
        """Accumulate drive (zbot::LeakyIntegrator::update)"""
        self.value += drive

    @property
    def activated(self) -> bool:
        """Whether the integrator has crossed its threshold
        (zbot::LeakyIntegrator::isActivated)"""
        return self.value >= self.threshold


class SegmentalCPG:
    """Segmental phase-oscillator network (zbot::CPG)

    ``2*n_segments`` oscillators (one left/right pair per body segment),
    all sharing the same intrinsic frequency, with a fixed rostrocaudal
    phase lag and left/right antiphase baked into the reset state (the
    couplings are structural, not integrated).
    """

    #: Rostrocaudal phase lag between consecutive segments [rad]
    SEGMENT_PHASE_LAG = 1.2*np.pi/6
    #: Left/right antiphase [rad]
    CONTRALATERAL_PHASE_LAG = np.pi

    def __init__(self, n_segments: int, frequency: float):
        self.n_segments = n_segments
        self.n_oscillators = 2*n_segments
        self.frequency = frequency
        self.phases = np.zeros(self.n_oscillators)
        self.outputs = np.zeros(self.n_oscillators)
        self.reset()

    @staticmethod
    def index(segment: int, side: Side) -> int:
        """Flat oscillator index for a given (segment, side)
        (zbot::CPG::getIndex)"""
        return 2*segment + int(side)

    def reset(self):
        """Reset phases to their structural (hard-coded) couplings and
        recompute outputs (zbot::CPG::reset)"""
        segments = np.arange(self.n_segments)
        left = self.index(segments, Side.LEFT)
        right = self.index(segments, Side.RIGHT)
        self.phases[left] = -segments*self.SEGMENT_PHASE_LAG
        self.phases[right] = (
            -segments*self.SEGMENT_PHASE_LAG + self.CONTRALATERAL_PHASE_LAG
        )
        self._update_outputs()

    def step(self, timestep: float):
        """Integrate oscillator phases by one control step
        (zbot::CPG::update)"""
        dtheta = 2*np.pi*self.frequency
        self.phases = np.mod(self.phases + dtheta*timestep, 2*np.pi)
        self._update_outputs()

    def _update_outputs(self):
        # Keep outputs in [0, 2] range, as in the reference implementation
        self.outputs[:] = np.sin(self.phases) + 1.0

    def phase(self, segment: int, side: Side) -> float:
        """Oscillator phase for a given (segment, side)
        (zbot::CPG::getPhase)"""
        return self.phases[self.index(segment, side)]

    def output(self, segment: int, side: Side) -> float:
        """Oscillator output for a given (segment, side)
        (zbot::CPG::getOutput)"""
        return self.outputs[self.index(segment, side)]


# --------------------------------------------------------------------------
# Controller
# --------------------------------------------------------------------------

class ZbotCPGController(AnimatController):
    """Zbot segmental CPG controller (zbot::Controller)

    Drives one position-controlled joint per body segment. Internal
    dynamics (CPG phases, bout gate, vSPN, leaky integrator) are advanced
    once per control step in ``before_step``; ``positions`` only exposes
    the resulting joint targets - the same split used by
    ``farms_amphibious.control.amphibious.AmphibiousController``
    (``before_step`` steps the network, ``positions_network`` reads it).
    """

    def __init__(
            self,
            animat_i: int,
            joints_names: tuple,
            muscles_names: tuple,
            max_torques: tuple,
            params: ZbotCPGParameters,
            substep: bool = True,
    ):
        super().__init__(
            animat_i=animat_i,
            joints_names=joints_names,
            muscles_names=muscles_names,
            max_torques=max_torques,
            substep=substep,
        )
        position_joints = self.joints_names[ControlType.POSITION]
        assert len(position_joints) == params.n_segments, (
            f'Expected {params.n_segments} position-controlled joints '
            f'(one per body segment), got {len(position_joints)}: '
            f'{position_joints}'
        )

        self.params = params
        self.cpg = SegmentalCPG(params.n_segments, params.cpg_frequency)
        self.leaky_integrator = LeakyIntegrator(params.leaky_threshold)
        self.bout_gate = ExponentialFilter(
            initial_value=params.cpg_initial_amplitude,
            decay_factor=params.cpg_decay_factor,
            beta=params.cpg_integration_weight,
        )
        self.vspn = ExponentialFilter(
            initial_value=params.vspn_initial_amplitude,
            decay_factor=params.vspn_decay_factor,
            beta=params.vspn_integration_weight,
        )

        self.state = ControllerState.SWIMMING
        self.turning_side = Side.NONE
        self.motor_neurons = np.zeros(2*params.n_segments)
        self.motor_outputs = np.zeros(params.n_segments)

    @classmethod
    def from_options(
            cls,
            config: dict,
            experiment_options,
            animat_i: int,
            animat_data,
            animat_options,
    ):
        """Build the controller from the animat's extension config.

        ``config`` is the dict under this extension's ``config:`` key in
        the animat YAML, e.g.::

            extensions:
              - loader: zbot_controller.ZbotCPGController
                config:
                  bout_duration_s: 5.0
                  bout_interval_s: 5.0
                  tail_frequency: 1.0
                  tail_amplitude: 1.0

        Joint names, control types and torque limits are pulled from
        ``animat_options.control.motors`` via the base
        ``AnimatController`` helpers - not reimplemented here.
        """
        del experiment_options, animat_data  # No sensory feedback in this model

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
                motor.joint_name: motor.limits_torque[1]
                for motor in motors
            },
            joints_control_types=joints_control_types,
        )

        config = dict(config)
        params = (
            ZbotCPGParameters.from_bout_timing(
                bout_duration_s=config.pop('bout_duration_s'),
                bout_interval_s=config.pop('bout_interval_s'),
                tail_frequency=config.pop('tail_frequency'),
                tail_amplitude=config.pop('tail_amplitude', 1.0),
                **config,
            )
            if 'bout_duration_s' in config
            else ZbotCPGParameters(**config)
        )

        return cls(
            animat_i=animat_i,
            joints_names=joints_names,
            muscles_names=(),
            max_torques=max_torques,
            params=params,
        )

    def set_turning_side(self, side: Side):
        """Bias the network to turn left/right (Side.NONE to swim straight)
        (zbot::Controller::setTurningSide)"""
        self.turning_side = side

    def initialize_episode(self, task, physics):
        """Reset all stateful components at the start of an episode"""
        del task, physics
        self.cpg.reset()
        self.leaky_integrator.reset()
        self.bout_gate.reset()
        self.vspn.reset()
        self.state = ControllerState.SWIMMING
        self.motor_neurons[:] = 0.0
        self.motor_outputs[:] = 0.0

    def before_step(self, task, action, physics):
        """Advance the network's internal dynamics by one control step"""
        del action
        timestep = physics.timestep()/task.units.seconds
        self.step(timestep)

    def step(self, timestep: float):
        """Advance CPG/bout-gate/vSPN dynamics and compute motor outputs
        (zbot::Controller::update)"""
        if self.state == ControllerState.GLIDING:
            self._gliding_step(timestep)
        else:
            self._swimming_step(timestep)
        self._update_motor_neurons()
        self._update_motor_outputs()

    def _gliding_step(self, timestep: float):
        """zbot::Controller::glidingUpdate"""
        self.leaky_integrator.step(self.params.descending_input_rate*timestep)
        if self.leaky_integrator.activated:
            self.state = ControllerState.SWIMMING
            self.leaky_integrator.reset()

    def _swimming_step(self, timestep: float):
        """zbot::Controller::swimmingUpdate"""
        if self.params.swimming_mode == SwimmingMode.BOUT_AND_GLIDE:
            self.bout_gate.step(timestep)
            self.vspn.step(timestep)
        self.cpg.step(timestep)

        threshold = (
            self.params.cpg_gliding_amplitude_threshold
            *self.params.cpg_initial_amplitude
        )
        if self.bout_gate.exponential_value < threshold:
            self.state = ControllerState.GLIDING
            self.bout_gate.reset()
            self.vspn.reset()
            self.cpg.reset()

    def _cpg_amplitude(self) -> float:
        """zbot::Controller::getCPGAmplitude"""
        if self.params.swimming_mode == SwimmingMode.CONTINUOUS:
            return self.params.cpg_initial_amplitude
        if self.state == ControllerState.GLIDING:
            return 0.0
        return self.bout_gate.value

    def _vspn_amplitude(self) -> float:
        """zbot::Controller::getVSPNAmplitude"""
        if self.params.swimming_mode == SwimmingMode.CONTINUOUS:
            return 0.0
        if self.state == ControllerState.GLIDING:
            return 0.0
        if self.turning_side == Side.NONE:
            return 0.0
        return self.vspn.value

    def _update_motor_neurons(self):
        """Sigmoid motor-neuron layer (zbot::Controller::update)"""
        amplitude_cpg = self._cpg_amplitude()
        amplitude_vspn = self._vspn_amplitude()
        omega = self.params.motor_neuron_omega
        bias = self.params.motor_neuron_bias
        for i in range(self.motor_neurons.size):
            output_cpg = amplitude_cpg*self.cpg.outputs[i]
            output_vspn = amplitude_vspn if i % 2 != self.turning_side else 0.0
            self.motor_neurons[i] = 1.0/(1.0 + np.exp(
                -omega*(output_cpg + output_vspn - bias)
            ))

    def _update_motor_outputs(self):
        """Left/right neuron combination into per-segment motor commands
        (zbot::Controller::update)"""
        left = self.motor_neurons[0::2]
        right = self.motor_neurons[1::2]
        self.motor_outputs = (
            self.params.tail_amplitude_ratio
            *np.asarray(self.params.motor_amplitude_biases)
            *(left - right)
            *self.params.motor_output_scaling
        )

    def positions(
            self,
            iteration: int,
            time: float,
            timestep: float,
    ) -> dict:
        """Return this step's joint position targets [rad].

        Values are computed once per step in ``before_step`` -> ``step``;
        this only exposes them, so calling it multiple times per step is
        safe and side-effect free.
        """
        del iteration, time, timestep
        return dict(zip(
            self.joints_names[ControlType.POSITION],
            self.motor_outputs,
        ))


```

---

## Step 3 — Wire the Controller to the Experiment

### 3a — Update `animat_config.yaml`

Change `controller_loader` to point to your new class:

```yaml
# animat_config.yaml
control:
  controller_loader: my_cpg_controller.ZbotSineCPG
  # ... rest unchanged ...
```

Remove or comment out the `network:` and `muscles:` sections since the simple CPG does not use them. Your minimal control section becomes:

```yaml
control:
  controller_loader: my_cpg_controller.ZbotSineCPG
  sensors:
    links: [Head, Segment1, Segment2, Segment3, Segment4, Segment5, Segment6, TailSegment]
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    xfrc: [Head, Segment1, Segment2, Segment3, Segment4, Segment5, Segment6, TailSegment]
    contacts: []
    muscles: []
  motors:
    - joint_name: joint_1
      control_types: [position]
      limits_torque: [-10.0, 10.0]
      gains: [3.0, 0.01, 0]
      equation: position         # <-- Change from position_muscle to position
    # ... joint_2 through joint_6 same ...
```

!!! important "Change `equation` from `position_muscle` to `position`"
    `position_muscle` requires the Ekeberg muscle model to be active. For a raw sine CPG,
    use `equation: position` so the value from `positions()` is used directly as the PD servo setpoint.

### 3b — Also update `experiment_config.yaml` loaders

Since you removed the `network:` section, you can switch from `AmphibiousOptions` to the simpler base class — **but only if you removed the network section entirely**. If you keep any `network:` YAML, keep `AmphibiousOptions`.

```yaml
# experiment_config.yaml — simplified loaders
loaders:
  simulation_options: farms_core.simulation.options.SimulationOptions
  animats_options:
    - farms_core.model.options.AnimatOptions   # <-- simplified if no network section
  arenas_options:
    - farms_amphibious.model.options.AmphibiousArenaOptions
  experiment_data: farms_amphibious.data.data.AmphibiousExperimentData
  animats_data:
    - farms_amphibious.data.data.AmphibiousData
```

### 3c — Run

```bash
cd /app/experiments/zbot_swimming
PYTHONPATH=. farmsim --experiment_config experiment_config.yaml
```

!!! important "PYTHONPATH"
    `PYTHONPATH=.` adds the current directory to the Python module search path so that `my_cpg_controller` is importable. Alternatively, install your file as a package or symlink it into the container's `site-packages`.

---

## Step 4 — Pass Parameters from YAML

Hard-coding CPG parameters in Python makes tuning painful. Instead, pass them from the YAML config:

In `animat_config.yaml`, update the extension entry:

```yaml
extensions:
  - loader: my_cpg_controller.ZbotSineCPG
    config:
      frequency: 1.5      # Hz
      amplitude: 0.35     # rad
      phase_lag: 1.0472   # rad (= π/3)
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null
```

In `from_options`, the `config` argument is the dict from that `config:` block:

```python
@classmethod
def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
    return cls(
        animat_i=animat_i,
        joints_names=animat_data.joints.names,
        muscles_names=(),
        max_torques=animat_data.joints.max_torques,
        frequency=config.get("frequency", 1.0),   # read from YAML, fall back to 1.0
        amplitude=config.get("amplitude", 0.4),
        phase_lag=config.get("phase_lag", np.pi / 3),
    )
```

---

## Step 5 — Add Sensory Feedback (Closed-Loop)

For closed-loop control, store a reference to `animat_data` in `from_options` and read sensor arrays inside your control methods.

```python
@classmethod
def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
    controller = cls(
        animat_i=animat_i,
        joints_names=animat_data.joints.names,
        muscles_names=(),
        max_torques=animat_data.joints.max_torques,
        frequency=config.get("frequency", 1.0),
        amplitude=config.get("amplitude", 0.4),
        phase_lag=config.get("phase_lag", np.pi / 3),
    )
    controller.animat_data = animat_data   # store for use in control methods
    return controller
```

Then inside your control method:

```python
def positions(self, iteration: int, time: float, timestep: float) -> dict:
    from farms_core.sensors.sensor_convention import sc

    sensors = self.animat_data.sensors
    buf = sensors.joints.array.shape[0]          # ring-buffer length (= buffer_size)
    idx = iteration % buf                         # safe modulo index

    # Read current joint positions (rad)
    joint_pos = sensors.joints.array[idx, :, sc.joint_position]

    # Read current joint velocities (rad/s)
    joint_vel = sensors.joints.array[idx, :, sc.joint_velocity]

    # Read hydrodynamic forces on each link (N) — shape: (n_links, 6)
    # Columns: [Fx, Fy, Fz, Tx, Ty, Tz] in world frame
    xfrc = sensors.xfrc.array[idx]

    position_joints = self.joints_names[ControlType.POSITION]
    commands = {}

    for i, joint_name in enumerate(position_joints):
        phase = 2.0 * np.pi * self.frequency * time - i * self.phase_lag

        # Example: reduce amplitude when joint position error is large
        desired = self.amplitude * np.sin(phase)
        error = desired - joint_pos[i]
        # (add any feedback logic here)

        commands[joint_name] = desired

    return commands
```

!!! warning "Always use `iteration % buffer_size` for sensor access"
    Sensor arrays are **ring buffers**. The current step's data is at `idx = iteration % buffer_size`.
    Accessing `sensors.joints.array[iteration]` directly will raise an `IndexError` after the buffer wraps.

---

## Step 6 — Implement a Phase-Coupled ODE CPG

For a biologically-inspired CPG, integrate coupled phase oscillators explicitly instead of using a static sine function. This allows oscillators to synchronise, adapt to perturbations, and be modulated by sensory feedback.

```python
# experiments/zbot_swimming/ode_cpg_controller.py
"""Phase-coupled ODE CPG for the Zbot.

Implements the Kuramoto-style coupled oscillator model:
  dθ_i/dt = ω_i + Σ_j w_ij · sin(θ_j - θ_i - φ_bias_ij)
  dA_i/dt = r_A · (A_nom - A_i)

Each joint has a Left and Right oscillator. Left-Right pairs are coupled
anti-phase (π rad) to generate lateral undulation. Adjacent pairs are
coupled with π/3 phase bias to produce a head-to-tail travelling wave.
"""

import numpy as np
from scipy.integrate import ode
from farms_core.model.control import AnimatController, ControlType


N_JOINTS = 6
N_OSC = N_JOINTS * 2          # 12 oscillators total (L + R per joint)


def _build_coupling_matrix(n_joints: int, w: float, lateral_bias: float, axial_bias: float):
    """Build the coupling weight and phase bias matrices.

    Indices: oscillator i = joint_i * 2 + side  (0=L, 1=R)
    """
    n = n_joints * 2
    W = np.zeros((n, n))
    Phi = np.zeros((n, n))

    for j in range(n_joints):
        L = j * 2
        R = j * 2 + 1

        # Left ↔ Right anti-phase coupling (lateral undulation)
        W[L, R] = W[R, L] = w
        Phi[L, R] = lateral_bias        # R → L phase bias
        Phi[R, L] = lateral_bias        # L → R phase bias (symmetric)

        # Front ↔ Back axial coupling (travelling wave)
        if j < n_joints - 1:
            L_next = (j + 1) * 2
            R_next = (j + 1) * 2 + 1

            # Forward coupling (j → j+1): positive phase bias = wave travels backward
            W[L, L_next] = W[R, R_next] = w
            W[L_next, L] = W[R_next, R] = w

            Phi[L, L_next] = axial_bias
            Phi[R, R_next] = axial_bias
            Phi[L_next, L] = 2 * np.pi - axial_bias  # reverse
            Phi[R_next, R] = 2 * np.pi - axial_bias

    return W, Phi


def _cpg_ode(t, y, omega, r_A, A_nom, W, Phi):
    """RHS of the CPG ODE system.

    State vector y = [θ_0, ..., θ_{N-1}, A_0, ..., A_{N-1}]
    """
    n = len(omega)
    theta = y[:n]
    A = y[n:]

    dtheta = omega.copy()
    for i in range(n):
        for j in range(n):
            if W[i, j] != 0:
                dtheta[i] += A[j] * W[i, j] * np.sin(theta[j] - theta[i] - Phi[i, j])

    dA = r_A * (A_nom - A)

    return np.concatenate([dtheta, dA])


class ZbotOdeCPG(AnimatController):
    """CPG with explicit phase-coupled ODE integration."""

    def __init__(
        self,
        animat_i: int,
        joints_names: tuple,
        muscles_names: tuple,
        max_torques: tuple,
        frequency: float = 1.0,
        amplitude: float = 0.4,
        phase_lag: float = np.pi / 3,
        coupling_weight: float = 30.0,
        amplitude_rate: float = 3.0,
        substep: bool = True,
    ):
        super().__init__(animat_i, joints_names, muscles_names, max_torques, substep)

        omega_single = 2.0 * np.pi * frequency    # rad/s
        self._omega = np.full(N_OSC, omega_single)
        self._A_nom = np.full(N_OSC, amplitude)
        self._r_A = amplitude_rate

        self._W, self._Phi = _build_coupling_matrix(
            n_joints=N_JOINTS,
            w=coupling_weight,
            lateral_bias=np.pi,           # anti-phase L↔R
            axial_bias=phase_lag,         # travelling wave
        )

        # Initial state: random phases, zero amplitude
        theta0 = np.random.uniform(0, 2 * np.pi, N_OSC)
        A0 = np.zeros(N_OSC)
        self._y = np.concatenate([theta0, A0])

        # ODE integrator (Dormand-Prince RK45)
        self._integrator = ode(_cpg_ode)
        self._integrator.set_integrator("dopri5")
        self._integrator.set_initial_value(self._y, 0.0)
        self._integrator.set_f_params(
            self._omega, self._r_A, self._A_nom, self._W, self._Phi
        )

    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
        return cls(
            animat_i=animat_i,
            joints_names=animat_data.joints.names,
            muscles_names=(),
            max_torques=animat_data.joints.max_torques,
            frequency=config.get("frequency", 1.0),
            amplitude=config.get("amplitude", 0.4),
            phase_lag=config.get("phase_lag", np.pi / 3),
            coupling_weight=config.get("coupling_weight", 30.0),
            amplitude_rate=config.get("amplitude_rate", 3.0),
        )

    def positions(self, iteration: int, time: float, timestep: float) -> dict:
        """Integrate CPG ODEs for one timestep and compute joint commands."""
        # Advance the ODE integrator
        self._integrator.integrate(time)
        self._y = self._integrator.y

        theta = self._y[:N_OSC]
        A = self._y[N_OSC:]

        position_joints = self.joints_names[ControlType.POSITION]
        commands = {}

        for j, joint_name in enumerate(position_joints):
            L = j * 2      # Left oscillator index
            R = j * 2 + 1  # Right oscillator index

            # Joint command = sum of left and right oscillator outputs
            # Left and right are anti-phase → their sine outputs subtract → undulation
            cmd = A[L] * np.sin(theta[L]) - A[R] * np.sin(theta[R])
            commands[joint_name] = cmd

        return commands

    def torques(self, iteration: int, time: float, timestep: float) -> dict:
        return {}
```

### What the ODE CPG Adds Over the Simple Sine

| Feature | Simple Sine | ODE CPG |
|---------|-------------|---------|
| Phase coupling between joints | No — open-loop | Yes — Kuramoto-style |
| Amplitude convergence (ramp-up) | No — instant | Yes — via ODE rate `r_A` |
| Adaptable to perturbations | No | Yes — coupling resynchronises |
| Sensory feedback can be added | Yes (separately) | Yes — add to `dtheta` |
| Computational cost | Minimal | Low (12 ODEs) |

---

## Step 7 — YAML Configuration for ODE CPG

```yaml
# animat_config.yaml — extension entry
extensions:
  - loader: ode_cpg_controller.ZbotOdeCPG
    config:
      frequency: 1.0          # Hz
      amplitude: 0.4          # rad (target steady-state amplitude)
      phase_lag: 1.0472       # rad (= π/3, 60° per joint)
      coupling_weight: 30.0   # Strength of phase coupling
      amplitude_rate: 3.0     # How fast amplitude ramps up (1/s)
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null
```

---

## Step 8 — Add Drive Modulation

In biological CPGs, a **descending drive signal** modulates frequency and amplitude. You can implement this by making `omega` and `A_nom` functions of a drive level:

```python
def set_drive(self, drive: float) -> None:
    """Modulate CPG frequency and amplitude based on drive level (1–5)."""
    frequency_gain = 1.5708    # rad/s per drive unit (matches animat_config.yaml)
    amplitude_gain = 0.15      # rad per drive unit

    omega_new = 2.0 * np.pi * (frequency_gain * drive / (2 * np.pi))
    self._omega[:] = omega_new

    A_nom_new = np.clip(amplitude_gain * drive, 0, 0.75)
    self._A_nom[:] = A_nom_new

    # Update ODE parameters
    self._integrator.set_f_params(
        self._omega, self._r_A, self._A_nom, self._W, self._Phi
    )
```

Call `self.set_drive(4)` in `from_options` for the default swimming behaviour, or vary it during the simulation based on sensory feedback.

---

## Common Pitfalls

| Problem | Cause | Fix |
|---------|-------|-----|
| Robot does not move | `controller_loader` path wrong | Check dotted path matches your file/class name exactly |
| `IndexError` in sensor access | `iteration` exceeds buffer size | Always use `idx = iteration % buffer_size` |
| CPG frequency is wrong | Mixed up Hz and rad/s | `ω = 2π·f`; `frequency_gain` in YAML is in rad/s per drive unit |
| `ImportError` for your module | Not in PYTHONPATH | Run with `PYTHONPATH=. farmsim ...` or install the module |
| `KeyError` in `positions()` | Wrong joint name | Print `self.joints_names[ControlType.POSITION]` to inspect |
| Simulation crashes immediately | Missing `from_options` override | You **must** override `from_options`; the base class has no default |
| Joints all at zero | `positions()` returns empty dict | Return dict must contain at least one entry |
| `equation: position_muscle` with no muscles | Muscle arrays not initialised | Change to `equation: position` |

---

## Full File Structure for a Custom Experiment

```
experiments/
└── zbot_my_cpg/
    ├── experiment_config.yaml      ← Loaders pointing to AmphibiousData + your class
    ├── simulation_config.yaml      ← Copy from zbot_swimming, adjust n_iterations
    ├── animat_config.yaml          ← Copy from zbot_swimming, change controller_loader
    │                                  and equation: position, remove network/muscles
    ├── arena_config.yaml           ← Copy from zbot_swimming unchanged
    ├── my_cpg_controller.py        ← Your ZbotSineCPG (Step 2)
    ├── ode_cpg_controller.py       ← Your ZbotOdeCPG (Step 6) — optional
    └── analysis.py                 ← Copy from zbot_swimming unchanged
```

---

## See Also

- [`AnimatController` API](../api/farms_core_control.md) — full method signatures
- [Mathematical Models](../mathematical_models.md) — CPG ODE and Ekeberg equations
- [`AmphibiousController` API](../api/farms_amphibious_controller.md) — production CPG
- [CPG Oscillator Network API](../api/cpg_oscillators.md) — oscillator data structures
- [Ekeberg Muscle Model API](../api/ekeberg_muscle.md) — muscle torque computation
- [Swimming Experiment](experiment.md) — full YAML reference
