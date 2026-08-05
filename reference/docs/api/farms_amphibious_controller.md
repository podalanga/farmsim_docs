# farms_amphibious.control.amphibious

Amphibious animat controller — wires CPG network, descending drive, and joint actuator models.

**Overview**
The amphibious controller bridges the neural network integration with the physics backend. It maps neural oscillator outputs to physical joint torques or positions using specialized actuator models.

## `get_amphibious_controller`

```python
def get_amphibious_controller(
    animat_data: AnimatData,
    animat_options: AnimatOptions,
    sim_options: SimulationOptions,
    **kwargs
) -> AnimatController
```

Factory function for instantiating the appropriate controller.

| Name | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `animat_data` | `AnimatData` | | Simulation data buffers for the animat |
| `animat_options` | `AnimatOptions` | | Configuration options |
| `sim_options` | `SimulationOptions` | | Global simulation options |
| `**kwargs` | `dict` | | Additional keyword arguments |

Returns `AmphibiousController` if configured with `AmphibiousControlOptions`, or `KinematicsController` if configured with `KinematicsControlOptions`.

---

## `JointMuscleController`

```python
class JointMuscleController(AnimatController)
```

Base controller class that owns mapping from abstract joints to physics-backend arrays. Inherits `positions()`, `velocities()`, and `torques()` from `AnimatController`.

**Attributes**
- `joints_map`: Maps joint names to Cython array indices.
- `muscle_maps`: Per-motor Cython actuator models (`Ekeberg`, `Passive`, `PositionMuscle`, `PositionPhase`).
- `network2joints`: Mapping interface from oscillator outputs to joint commands.

---

## `AmphibiousController`

```python
class AmphibiousController(JointMuscleController)
```

Main controller class for amphibious animats.

**Attributes**
- `network`: Integrating `NetworkODE` that computes neural states.

**Methods**

### `step`

```python
def step(self, iteration: int, time: float, timestep: float) -> None
```

Executes the control step. Internally it updates the descending drive, steps the CPG `NetworkODE`, and then updates all Cython muscle models.

| Name | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `iteration` | `int` | | Current simulation step index |
| `time` | `float` | | Current physics time [s] |
| `timestep` | `float` | | Physics timestep [s] |

---

## `KinematicsController`

```python
class KinematicsController(AnimatController)
```

Replays pre-recorded joint kinematics instead of computing closed-loop physics. Reads trajectories from a file and applies them directly to the joints.

---

## Dynamic Mappings

The controller relies on dynamic mapping classes to translate abstract models to continuous simulation arrays.
- `JointsMap`: Maps abstract joint names to physics simulation indices.
- `MusclesMap`: Maps logical muscle definitions to physics actuator parameters.

These mappings are populated dynamically during `initialize_episode`, rendering the controller agnostic to the underlying physics engine.

!!! note
    The Cython actuator models (`EkebergMuscleCy`, `PassiveJointCy`, `PositionMuscleCy`, `PositionPhaseCy`) are documented in [api/ekeberg_muscle.md](ekeberg_muscle.md) and [api/joint_controllers.md](joint_controllers.md).

**See also:**
- [farms_amphibious.control.network](farms_amphibious_network.md)
- [Ekeberg Muscle Actuator Models](ekeberg_muscle.md)
- [Joint Controllers](joint_controllers.md)
- [Core Control Module](farms_core_control.md)
