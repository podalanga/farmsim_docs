# farms_amphibious.control.amphibious

Amphibious animat controller — wires CPG network, descending drive, and joint actuator models.

**Overview**
The amphibious controller bridges the neural network integration with the physics backend. It maps neural oscillator outputs to physical joint torques or positions using specialized actuator models.

## `get_amphibious_controller`

!!! warning "Unverified"
    `get_amphibious_controller` is **not defined** in the current source tree. It is
    imported by `farms_amphibious/scripts/run_control.py` and
    `farms_amphibious/scripts/amphibious.py` from
    `farms_amphibious.control.amphibious`, but no `def get_amphibious_controller`
    exists anywhere in the repository (the import would fail). The actual
    controller construction is done directly via `AmphibiousController.from_options`
    / `AmphibiousController(...)` (see `amphibious.py`). Treat the signature and
    behavior below as unverified/aspirational.

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

Base controller class that owns mapping from abstract joints to physics-backend arrays.

!!! note "Does not inherit `positions()`/`velocities()`/`torques()`"
    `JointMuscleController` **defines** `positions()`, `velocities()`, and
    `torques()` itself (`farms_amphibious/control/amphibious.py`); it does not
    inherit them from `AnimatController`. Each method iterates the equation
    handlers registered for its `ControlType` (see "Equation handlers" below)
    and collects their per-joint outputs into a `dict[str, float]`.

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

---

## `AmphibiousDriveController`

```python
class AmphibiousDriveController(AmphibiousController)
```

Subclass of `AmphibiousController` that additionally drives the animat's
visual feedback array (drive/phase colouring on the model) from the
descending drive and CPG state. Used only when an experiment wires
`controller_loader` to this class explicitly; the bundled Zbot experiments
use plain `AmphibiousController`, so this class is not exercised by default.

**Attributes**
- `cmap_drives`, `cmap_phases`: Matplotlib colormaps (`turbo`, `Greens`) used to shade the animat by drive/phase value.
- `norm`: `Normalize(vmin=0, vmax=6)` mapping drive values to colormap range.
- `visuals`: `VisualsArray` view into `animat_data.sensors.visuals`, written to each step.

### `step`

```python
def step(self, iteration: int, time: float, timestep: float) -> None
```

!!! bug "Confirmed: double-steps the descending drive"
    `AmphibiousDriveController.step()` calls `self.drive.step(iteration, time,
    timestep)` directly, and then calls `super().step(iteration, time,
    timestep)` — which is `AmphibiousController.step()`, and which
    *unconditionally* calls `self.drive.step(...)` again. The net effect is
    that `drive.step()` runs **twice per physics step** whenever a drive is
    configured on this subclass.

    This is not equivalent to running the drive at half the timestep, because
    `OrientationFollower` (the concrete drive implementation shipped in this
    repo) is **stateful**: `step()` advances exponential low-pass filters on
    `self.turn`, `self.fwds`, and `self.contact_value`, and drives an
    internal `simple_pid.PID` controller. Calling `step()` twice per physics
    tick applies the low-pass update and the PID update twice within one
    physics timestep, distorting the turn-rate and forward-speed response
    compared to a single call — the steering and gait-switching behavior will
    differ from what the configured PID/filter gains imply.

    **Status:** confirmed present in source, but dormant in this repository's
    bundled experiments: the Zbot experiments instantiate plain
    `AmphibiousController` (whose `step()` calls `drive.step()` exactly once)
    rather than `AmphibiousDriveController`. The bug only manifests for
    experiments/extensions that explicitly select
    `AmphibiousDriveController` as their `controller_loader`. See
    [Descending Drive](descending-drive.md) for the stateful filter/PID
    internals this affects.

After the (duplicated) drive/network/joint step, `set_visuals()` writes
drive-colour and phase-colour channels into `self.visuals.array` if any
visual channels are configured and the drive exposes a `drives` attribute;
otherwise `set_visuals_invisible()` zeroes the visuals array for that
iteration.

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
    The Cython actuator models (`EkebergMuscleCy`, `PassiveJointCy`, `PositionMuscleCy`, `PositionPhaseCy`) are documented in [ap./ekeberg-muscle.md](ekeberg-muscle.md) and [ap./joint-controllers.md](joint-controllers.md).

**See also:**
- [farms_amphibious.control.network](network-ode.md)
- [Descending Drive](descending-drive.md) — see the double-step bug affecting `AmphibiousDriveController`
- [Ekeberg Muscle Actuator Models](ekeberg-muscle.md)
- [Joint Controllers](joint-controllers.md)
- [Core Control Module](../core/core-control.md)
