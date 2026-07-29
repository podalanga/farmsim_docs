# Custom CPG Controller — Step-by-step guide

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
"""

import numpy as np
from farms_core.model.control import AnimatController, ControlType


class ZbotSineCPG(AnimatController):
    """Open-loop CPG: travelling sine wave over all body joints."""

    def __init__(
        self,
        animat_i: int,
        joints_names: tuple,
        muscles_names: tuple,
        max_torques: tuple,
        frequency: float = 1.0,
        amplitude: float = 0.4,
        phase_lag: float = np.pi / 3,
        substep: bool = True,
    ):
        super().__init__(animat_i, joints_names, muscles_names, max_torques, substep)

        # --- CPG parameters ---
        self.frequency = frequency      # Hz  — undulation cycles per second
        self.amplitude = amplitude      # rad — peak joint deflection
        self.phase_lag = phase_lag      # rad — phase lag between adjacent joints
        #   π/3 (60°) → 6 joints span 360° → one full wavelength along the body

    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
        """
        Factory method called by farms_sim at experiment setup.

        Parameters
        ----------
        config : dict
            Contents of the `config:` field under the controller's extension entry.
            Use this to pass tunable parameters from YAML without hardcoding them.
        experiment_options : ExperimentOptions
            Global experiment settings (timestep, etc.).
        animat_i : int
            Index of this animat in a multi-robot experiment.
        animat_data : AmphibiousData
            Data container holding pre-allocated sensor arrays and joint metadata.
        animat_options : AmphibiousOptions
            Full animat configuration loaded from animat_config.yaml.
        """
        return cls(
            animat_i=animat_i,
            joints_names=animat_data.joints.names,
            muscles_names=animat_data.muscles.names if animat_data.muscles else (),
            max_torques=animat_data.joints.max_torques,
            # Read tunable params from config dict (with defaults)
            frequency=config.get("frequency", 1.0),
            amplitude=config.get("amplitude", 0.4),
            phase_lag=config.get("phase_lag", np.pi / 3),
            substep=True,
        )

    def positions(self, iteration: int, time: float, timestep: float) -> dict:
        """
        Compute target joint positions for this control step.

        Called every cb_sub_steps (2× per MuJoCo env.step() at default settings).
        Returns a dict mapping each position-controlled joint name → angle in rad.
        """
        position_joints = self.joints_names[ControlType.POSITION]
        commands = {}

        for i, joint_name in enumerate(position_joints):
            # θ_i = 2π·f·t - i·Δφ
            # The minus sign propagates the wave from head (i=0) toward tail (i=5)
            phase = 2.0 * np.pi * self.frequency * time - i * self.phase_lag
            commands[joint_name] = self.amplitude * np.sin(phase)

        return commands

    def torques(self, iteration: int, time: float, timestep: float) -> dict:
        """No torque-controlled joints — return empty."""
        return {}
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
