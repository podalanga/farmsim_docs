# `farms_amphibious` — Amphibious Animat Control

`farms_amphibious` provides biologically inspired controllers for amphibious robots: a Central Pattern Generator (CPG) oscillator network, descending drive, Ekeberg muscle model, and passive/position joint actuators. It depends on `farms_core` for interfaces and data, and `farms_mujoco` for physics execution.

**Source**: `farms_amphibious/farms_amphibious/`

---

## Overview

The module implements three coupled subsystems:

1. **CPG network** (`control/network.py`, `control/ode.pyx`) — a system of coupled Hopf-type oscillators integrated with `dopri5` (Runge-Kutta 4/5). Generates rhythmic phase and amplitude outputs.
2. **Descending drive** (`control/drive.py`) — modulates CPG frequency and amplitude in response to goal-directed inputs. The `OrientationFollower` class uses a PID controller on a `PotentialMap` strategy.
3. **Joint actuators** (`control/ekeberg.pyx`, `passive_cy.pyx`, `position_muscle_cy.pyx`, `position_phase_cy.pyx`) — Cython implementations converting CPG outputs to joint torques or position setpoints.

---

## CPG Network

The CPG is a network of N coupled oscillators. Each oscillator *i* has phase φᵢ (rad) and amplitude rᵢ. The ODEs are:

**Phase** (from `ode.pyx:ode_dphase`):

$$\frac{d\varphi_i}{dt} = \omega_i \cdot (1 + A^{mod}_i \cos(\varphi_i + \Phi^{mod}_i)) + \sum_{j \in \mathcal{C}(i)} r_j \cdot w_{ji} \cdot \sin(\varphi_j - \varphi_i - \Delta\varphi_{ji})$$

**Amplitude** (from `ode.pyx:ode_damplitude`):

$$\frac{dr_i}{dt} = a_i (R_i - r_i)$$

Where ωᵢ is **angular frequency in rad/s** (stored as `intrinsic_frequencies`, computed by `c_angular_frequency`).

**Tegotae sensory feedback** (`STRETCH2FREQTEGOTAE` connection type in `ode.pyx:ode_stretch`):

$$\frac{d\varphi_i}{dt} \mathrel{+}= w \cdot \theta_{joint} \cdot \sin(\varphi_i)$$

This couples the neural phase to physical joint stretch — the robot's body mechanics entrain the CPG rhythm.

For the full oscillator math, Cython kernel signatures, and state vector layout, see [CPG Oscillator Network](api/cpg_oscillators.md).

---

## Muscle Models

CPG outputs are converted to joint torques by one of four Cython actuator strategies, selected per motor via `motor.equation` in the options YAML:

| Class | Equation type | Source |
|-------|---------------|--------|
| `EkebergMuscleCy` | Non-linear spring-damper (active + passive stiffness, damping, friction) | `control/ekeberg.pyx` |
| `PassiveJointCy` | Spring + damping + Coulomb friction; no active drive | `control/passive_cy.pyx` |
| `PositionMuscleCy` | Position setpoint from antagonistic excitation difference | `control/position_muscle_cy.pyx` |
| `PositionPhaseCy` | Position setpoint tracking raw oscillator phase | `control/position_phase_cy.pyx` |

See [Ekeberg Muscle Model](api/ekeberg_muscle.md) and [Joint Controllers](api/joint_controllers.md) for the full model equations and parameter tables.

---

## Controller Architecture

```text
AmphibiousController(JointMuscleController)
├── network: NetworkODE          ← CPG integrator
├── drive: DescendingDrive       ← goal-directed modulation
└── muscle_maps: dict            ← per-motor Cython actuator instances
```

`step()` sequence inside `before_step()`:
1. `drive.step(iteration, time, timestep)` — update drive array
2. `network.step(iteration, time, timestep)` — integrate CPG ODEs
3. For each motor: Cython actuator computes and writes torque/position to `AnimatData`

See [Amphibious Controller](api/farms_amphibious_controller.md) for the full class reference.

---

## Descending Drive

`DescendingDrive` is an ABC with abstract `step()`. The concrete `OrientationFollower` uses a PID controller on a `PotentialMap` strategy (straight-line or circular orbit) to compute asymmetric left/right drive values that steer the animat.

```python
from farms_amphibious.control.drive import DescendingDrive

class SpeedController(DescendingDrive):
    def step(self, iteration, time, timestep):
        speed = self.compute_target_speed(time)
        self.set_left_drive(iteration, speed)
        self.set_right_drive(iteration, speed)
```

See [Descending Drive](api/farms_amphibious_drive.md) for all setter/getter signatures and subclass documentation.

---

## Running an Amphibious Simulation

The standard entry point for FARMS experiments is the `farmsim` console script:

```bash
farmsim --experiment_config experiment_config.yaml
```

The YAML configuration selects the controller via the `extensions` list in `animat_config.yaml`:
- `AmphibiousController` (CPG-based, via `farms_amphibious.control.amphibious.AmphibiousController`)
- `KinematicsController` (recorded kinematics replay)

See [Amphibious Controller](api/farms_amphibious_controller.md) for details on controller selection.

---

## See Also

- [CPG Oscillator Network](api/cpg_oscillators.md)
- [Ekeberg Muscle Model](api/ekeberg_muscle.md)
- [Joint Controllers](api/joint_controllers.md)
- [Amphibious Controller](api/farms_amphibious_controller.md)
- [Network ODE](api/farms_amphibious_network.md)
- [Descending Drive](api/farms_amphibious_drive.md)
- [Data Classes](api/farms_amphibious_data.md)
- [Options](api/farms_amphibious_options.md)
- [Mathematical Models](./mathematical_models.md)
- **Source**: `farms_amphibious/farms_amphibious/`