# `farms_amphibious` — Amphibious Animat Control

`farms_amphibious` provides biologically inspired controllers for amphibious robots: a Central Pattern Generator (CPG) oscillator network, descending drive, Ekeberg muscle model, and passive/position joint actuators. It depends on `farms_core` for interfaces and data, and `farms_mujoco` for physics execution.

**Source**: `farms_amphibious/farms_amphibious/`

---

## Overview

The package implements three tightly coupled subsystems that together produce rhythmic, goal-directed locomotion:

1. **CPG network** (`control/network.py`, `control/ode.pyx`) — a system of coupled Hopf-type oscillators integrated with `dopri5` (Runge-Kutta 4/5). Generates rhythmic phase φᵢ and amplitude rᵢ outputs per oscillator node.
2. **Descending drive** (`control/drive.py`) — modulates CPG frequency and amplitude in response to goal-directed inputs (speed setpoints, heading error). Concrete implementations include `OrientationFollower` (PID + `PotentialMap`) and simple open-loop speed controllers.
3. **Joint actuators** (Cython `.pyx` files in `control/`) — convert CPG phase/amplitude outputs into joint torques or position setpoints, one actuator strategy per motor.

The top-level entry point for an amphibious simulation is:

```bash
python scripts/amphibious.py --experiment_config configs/salamander_walking.yaml
```

---

## Package Structure

```
farms_amphibious/farms_amphibious/
├── control/
│   ├── amphibious.py          ← AmphibiousController (top-level controller)
│   ├── drive.py               ← DescendingDrive ABC, OrientationFollower, PotentialMaps
│   ├── network.py             ← AnimatNetwork ABC, NetworkODE (scipy integrator)
│   ├── ode.pyx                ← Cython ODE kernels (ode_oscillators_sparse, etc.)
│   ├── ekeberg.pyx            ← Ekeberg muscle actuator (Cython)
│   ├── passive_cy.pyx         ← Passive joint spring-damper (Cython)
│   ├── muscle_cy.pyx          ← Generic muscle torque actuator (Cython)
│   ├── position_muscle_cy.pyx ← Position setpoint from antagonist excitation (Cython)
│   ├── position_phase_cy.pyx  ← Position setpoint tracking raw oscillator phase (Cython)
│   ├── joints_control_cy.pyx  ← Joint control dispatch (Cython)
│   ├── kinematics.py          ← KinematicsController (recorded trajectory replay)
│   └── manta_control.py       ← Manta ray fin controller
├── data/
│   ├── data.py                ← AmphibiousData, AmphibiousExperimentData
│   └── network.py             ← DriveArray, NetworkStateArray
├── model/
│   └── options.py             ← AmphibiousOptions, AmphibiousControlOptions, etc.
├── analysis/                  ← Post-simulation analysis utilities
└── bullet/                    ← PyBullet-specific implementations (legacy)
```

---

## Controller Architecture

The main controller is `AmphibiousController`, selected when `animat_options.control` is an `AmphibiousControlOptions` instance. The factory function `get_amphibious_controller()` in `amphibious.py` instantiates the right controller based on the options type:

```
get_amphibious_controller(animat_data, animat_options, sim_options)
    ├── if AmphibiousControlOptions  →  AmphibiousController
    │       ├── network: NetworkODE           ← CPG integrator
    │       ├── drive: DescendingDrive        ← goal-directed modulation
    │       └── muscle_maps: dict            ← per-motor Cython actuator instances
    └── if KinematicsControlOptions  →  KinematicsController
            └── kinematics: np.ndarray       ← pre-recorded trajectory (CSV)
```

### `AmphibiousController.before_step()` Execution Sequence

Each substep (called by `ExperimentTask.before_step` at `cb_sub_steps` times per outer step):

1. `drive.step(iteration, time, timestep)` — update `data.network.drives.array[iteration]`
2. `network.step(iteration, time, timestep)` — integrate CPG ODEs, write `data.state.array[iteration]`
3. For each motor: Cython actuator reads `data.state.array[iteration]` and writes torque/position to `data.sensors.*`

---

## CPG Network

### Oscillator ODEs

The CPG is a network of **N coupled oscillators**. Each oscillator *i* has state variables:
- **φᵢ** — phase (rad), unbounded, advances continuously
- **rᵢ** — amplitude (dimensionless), converges to a setpoint Rᵢ

**Phase dynamics** (`ode.pyx:ode_dphase`):

$$\frac{d\varphi_i}{dt} = \omega_i + \sum_{j \in \mathcal{C}(i)} r_j \cdot w_{ji} \cdot \sin(\varphi_j - \varphi_i - \Delta\varphi_{ji})$$

**Amplitude dynamics** (`ode.pyx:ode_damplitude`):

$$\frac{dr_i}{dt} = a_i (R_i - r_i)$$

Where:
- ωᵢ — angular frequency in rad/s (computed from `intrinsic_frequencies` in the drive array)
- *w*ᵢⱼ — coupling weight from oscillator *j* to *i*
- Δφᵢⱼ — desired phase difference (rad)
- *a*ᵢ — amplitude convergence rate

**Tegotae sensory feedback** (connection type `STRETCH2FREQTEGOTAE` in `ode.pyx`):

$$\frac{d\varphi_i}{dt} \mathrel{+}= w \cdot \theta_{\text{joint}} \cdot \sin(\varphi_i)$$

This couples the neural phase to physical joint stretch — the robot's body mechanics entrain the CPG rhythm, producing adaptive gait coordination.

### ODE Integration: `NetworkODE`

`NetworkODE` wraps `scipy.integrate.ode` with the `dopri5` (RK4/5) integrator. It calls `ode_oscillators_sparse` (a Cython function) at each timestep:

```python
# network.py — simplified
class NetworkODE(AnimatNetwork):
    def __init__(self, data, integrator='dopri5', **kwargs):
        self.solver = integrate.ode(f=ode_oscillators_sparse)
        self.solver.set_integrator(integrator)
        self.solver.set_initial_value(y=data.state.array[0, :], t=0.0)

    def step(self, iteration, time, timestep, **kwargs):
        if iteration == 0:
            self.copy_next_drive(iteration)
            return
        self.solver.set_f_params(self.dstate, iteration, self.data)
        while self.solver.successful() and self.solver.t < time + 0.99*timestep:
            self.data.state.array[iteration, :] = self.solver.integrate(time + timestep)
        if not self.solver.successful():
            # Logs warning and resets solver — does NOT crash by default
            pylog.warning('ODE not integrated properly at iteration=%d', iteration)
```

!!! warning "ODE Integration Failure"
    If `self.solver.successful()` returns `False`, the solver logs a warning and resets to the previous state by default (`strict=False`). Pass `strict=True` to `NetworkODE.step()` to raise `IntegrationException` instead. Solver failures are typically caused by excessively large timesteps or ill-conditioned network parameters.

### State Array Layout

`data.state.array` has shape `[n_iterations, n_state_vars]`. The state vector packs all oscillator phases followed by all amplitudes:

```
state[iteration] = [φ₀, φ₁, ..., φₙ₋₁, r₀, r₁, ..., rₙ₋₁]
```

Where `n` is the total number of oscillators. The Cython ODE kernel reads and writes this layout directly.

---

## Muscle Models (Joint Actuators)

CPG outputs (phase φᵢ and amplitude rᵢ) are converted to joint torques or position setpoints by one of five Cython actuator strategies, selected per motor via `motor.equation` in the options YAML:

| Class | Equation type | Source file |
|-------|---------------|-------------|
| `EkebergMuscleCy` | Non-linear active spring-damper (active stiffness + passive stiffness + damping + friction) | `ekeberg.pyx` |
| `PassiveJointCy` | Passive spring + damping + Coulomb friction; no active CPG drive | `passive_cy.pyx` |
| `PositionMuscleCy` | Position setpoint from antagonistic excitation difference: `q_ref = (e⁺ - e⁻) · scale` | `position_muscle_cy.pyx` |
| `PositionPhaseCy` | Position setpoint tracking raw oscillator phase: `q_ref = R · sin(φ)` | `position_phase_cy.pyx` |
| `MuscleActivationCy` | Generic muscle torque from activation array | `muscle_cy.pyx` |

### Ekeberg Muscle Model

The Ekeberg model (`ekeberg.pyx`) implements an antagonistic pair of muscles (flexor and extensor) producing a net joint torque:

$$\tau = (k^+ e^+ - k^- e^-)(q_0 - q) - \beta \dot{q} - f_c \cdot \text{sign}(\dot{q})$$

Where:
- *e*⁺, *e*⁻ — muscle excitations from CPG (range: 0–1)
- *k*⁺, *k*⁻ — active stiffness coefficients (Nm/rad)
- *q*₀ — reference angle (springref, rad)
- *q*, *q̇* — current joint angle and velocity
- β — damping coefficient (N·m·s/rad)
- *f*c — Coulomb friction (N·m)

The muscle equation type is selected by setting `motor.equation: ekeberg` in YAML.

---

## Descending Drive

`DescendingDrive` is an abstract base class (`ABC`) with a single abstract method `step()`. Concrete subclasses update `self.data.network.drives.array[iteration]` in place.

### Built-in Drive Implementations

| Class | Strategy |
|-------|---------|
| `OrientationFollower` | PID controller on heading error relative to a `PotentialMap` target |
| *(open-loop)* | Fixed drive values set at construction time |

### `PotentialMap` Implementations

`PotentialMap` defines the target heading as a function of 2D position `pos`. Two concrete implementations are provided:

| Class | Description |
|-------|-------------|
| `StraightLinePotentialMap` | Drives the animat toward a straight-line trajectory at angle `theta` through `origin` |
| `CirclePotentialMap` | Drives the animat to orbit a circular path of `radius` centred at `origin` |

```python
# drive.py — StraightLinePotentialMap.heading()
def heading(self, pos):
    pos_complex = complex(*(pos[:2] - self.origin))
    theta = np.angle(pos_complex)
    r_dot = self.gain * np.linalg.norm(pos_complex) * np.sin(self.theta - theta)
    return np.angle(
        r_dot * np.exp(1j * (self.theta + 0.5 * np.pi))
        + np.exp(1j * self.theta)
    )
```

### Implementing a Custom Drive

```python
from farms_amphibious.control.drive import DescendingDrive

class SpeedRampDrive(DescendingDrive):
    """Linearly ramps drive from 0 to max_speed over ramp_time seconds."""

    def __init__(self, data, max_speed: float, ramp_time: float):
        super().__init__(data)
        self.max_speed = max_speed
        self.ramp_time = ramp_time

    def step(self, iteration: int, time: float, timestep: float):
        speed = min(1.0, time / self.ramp_time) * self.max_speed
        # set_left_drives / set_right_drives write to drives.array[iteration]
        self.set_left_drives(iteration, speed)
        self.set_right_drives(iteration, speed)
```

---

## Kinematics Controller

When `animat_options.control` is a `KinematicsControlOptions` instance, `get_amphibious_controller()` returns a `KinematicsController` instead of `AmphibiousController`. This replays pre-recorded joint trajectories from a CSV file rather than computing CPG outputs:

```yaml
# animat_config.yaml — kinematics control
control:
  type: kinematics
  kinematics_file: data/recorded_gait.csv
  kinematics_sampling: 0.01          # Sampling period of recorded data [s]
  kinematics_degrees: false          # If true, CSV values are in degrees
  kinematics_time_index: 0           # Column index for time (or -1 for no time column)
  kinematics_start: 0.0              # Start time in recording [s]
  kinematics_end: null               # End time (null = entire recording)
  kinematics_invert: []             # Joint names whose sign should be flipped
```

---

## YAML Configuration

### Minimum Working Config Snippet

```yaml
# animat_config.yaml
sdf: models/zbot/model.sdf

spawn:
  loader: FARMS
  mode: free
  pose: [0, 0, 0.05, 0, 0, 0]
  velocity: [0, 0, 0, 0, 0, 0]

morphology:
  links:
    - name: Body
      collisions: true
      friction: [0.8, 0.01, 0.001]
      density: 950.0
      fluid_interaction: true
      drag_coefficients: [[-4.0, -4.0, -10.0], [-0.5, -0.5, -0.5]]
  joints:
    - name: joint_spine_1
      initial: [0.0, 0.0]          # [position_rad, velocity_rad_s]
      limits: [[-1.57, 1.57], [-10.0, 10.0]]  # [[pos_min, pos_max], [vel_min, vel_max]]
      stiffness: 0.0
      springref: 0.0
      damping: 0.001

control:
  controller_loader: farms_amphibious.control.amphibious.AmphibiousController
  sensors:
    links: [Body]
    joints: [joint_spine_1]
    contacts: []
    xfrc: [Body]
    muscles: []
  motors:
    - joint_name: joint_spine_1
      control_types: [position]
      limits_torque: [-2.0, 2.0]
      gains: [20.0, 1.0]           # [Kp, Kd] for position control

extensions:
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null
```

### Complete `LinkOptions` Parameter Reference

| Name | Type | Default | Description |

|-----------|------|---------|-------------|
| `name` | `str` | *(required)* | Link name matching SDF file exactly (case-sensitive) |
| `collisions` | `bool` | *(required)* | Enable collision geometry for this link |
| `friction` | `list[float]` | *(required)* | Friction coefficients (see MuJoCo docs for format) |
| `density` | `float` | `1000.0` | Volumetric mass density [kg/m³]; used for buoyancy computation |
| `fluid_interaction` | `bool` | `False` | Enable hydrodynamic drag/buoyancy for this link |
| `drag_coefficients` | `list[list[float]]` | `[0,0,0,0,0,0]` | `[[Vx, Vy, Vz], [Wx, Wy, Wz]]` drag coefficients; negative = resistive |

### Complete `MotorOptions` Parameter Reference

| Parameter | Type | Description |
|-----------|------|-------------|
| `joint_name` | `str` | Joint name to actuate (must match SDF and morphology.joints) |
| `control_types` | `list[str]` | List of control modes: `position`, `velocity`, `torque`, `springref`, `springcoef`, `dampingcoef`, `muscle` |
| `limits_torque` | `list[float]` | `[min, max]` torque limits in N·m |
| `gains` | `list[float]` | `[Kp, Kd]` gains for position control; behaviour for velocity/torque control is implementation-defined |

---

## Running an Amphibious Simulation

### From the Command Line

```bash
# Standard CPG-based walking simulation
python scripts/amphibious.py \
    --experiment_config configs/salamander_walking.yaml \
    --log_path Output/walk_01

# Kinematics replay
python scripts/amphibious.py \
    --experiment_config configs/kinematics_replay.yaml \
    --headless \
    --log_path Output/kinematic_01
```

### Programmatically

```python
from farms_sim.simulation import setup_from_clargs, run_simulation

from argparse import Namespace
clargs = Namespace(
    experiment_config='configs/salamander_walking.yaml',
    simulator='MUJOCO',
    log_path='Output',
    prompt=False,
    verify_save=False,
    test_configs=False,
    profile='',
)
clargs, exp_options, simulator = setup_from_clargs(clargs=clargs)
sim = run_simulation(exp_options, simulator=simulator)
```

---

## Common Pitfalls

!!! warning "Cython Rebuild After `.pyx` Edits"
    Editing any `.pyx` file (`ode.pyx`, `ekeberg.pyx`, etc.) does **not** take effect until the Cython extension is recompiled. Pure Python (`.py`) edits are live immediately. After any `.pyx` change:
    ```bash
    pip install -e ./farms/farms_amphibious --no-build-isolation
    ```

!!! warning "ODE State Array Indexing"
    `data.state.array` is **not** a ring buffer — it is allocated with full size `[n_iterations, n_state_vars]`. **Do not** use modulo indexing when reading from it. Use `data.state.array[iteration, :]` directly.

!!! warning "Motor `control_types` Must Match YAML Exactly"
    The `control_types` list values must match the string forms recognised by `ControlType.from_string()`: `'position'`, `'velocity'`, `'torque'`, `'springref'`, `'springcoef'`, `'dampingcoef'`, `'muscle'`. Any other string raises `KeyError` at config-load time.

!!! warning "Extension Order Affects Sensor Freshness"
    If `SwimmingExtension` is listed **before** `AmphibiousController` in the YAML `extensions` list, the hydrodynamics forces are computed from the previous timestep's joint positions (before the controller has moved them). List `AmphibiousController` first for tightest closed-loop behaviour.

!!! warning "`drive_config` Must Be an Absolute or Config-Relative Path"
    `drive_from_config()` calls `yaml2pyobject(filename)`. If `drive_config` is a relative path, it is resolved relative to the **Python working directory at runtime**, not relative to the YAML config file location. Use absolute paths to avoid `FileNotFoundError`.

---

## See Also

- [CPG Oscillator Network](../api/cpg_oscillators.md) — full ODE kernel signatures and state vector layout
- [Ekeberg Muscle Model](../api/ekeberg_muscle.md) — detailed muscle equation and parameter table
- [Joint Controllers](../api/joint_controllers.md) — all five actuator strategies
- [Amphibious Controller](../api/farms_amphibious_controller.md) — `AmphibiousController` class reference
- [Network ODE Integrator](../api/farms_amphibious_network.md) — `NetworkODE` and `AnimatNetwork` ABC
- [Descending Drive](../api/farms_amphibious_drive.md) — `DescendingDrive`, `PotentialMap`, `OrientationFollower`
- [Data Classes](../api/farms_amphibious_data.md) — `AmphibiousData`, `NetworkStateArray`, `DriveArray`
- [Options](../api/farms_amphibious_options.md) — `AmphibiousOptions`, `AmphibiousControlOptions`, `KinematicsControlOptions`
- [Mathematical Models](./mathematical_models.md) — coordinate transforms, CPG equations
- [farms_core — Extension System](./farms_core.md) — `TaskExtension`, `AnimatController` base classes

## Source Code

`farms_amphibious/farms_amphibious/control/`, `farms_amphibious/farms_amphibious/model/`, `farms_amphibious/farms_amphibious/data/`
