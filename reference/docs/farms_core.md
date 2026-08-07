# `farms_core` — Framework Foundation

`farms_core` is the base layer of FARMS. It defines data schemas, pre-allocated sensor arrays, configuration dataclasses, the extension/controller interface, and I/O utilities. It has no dependency on any physics engine.

**Source**: `farms_core/farms_core/`

---

## Overview

`farms_core` separates two orthogonal concerns: **configuration** (Options classes, parsed from YAML before the simulation starts) and **runtime state** (Data classes, pre-allocated NumPy arrays written during the simulation loop).

A third concern — the **extension system** — defines the interface contract that all domain-specific code (controllers, hydrodynamics, loggers) must implement to hook into the simulation lifecycle.

---

## Options and Data

### Options

Options classes are **`dict` subclasses** deserialised from YAML via `Options.load(path)`. They are validated at load time; incorrect field types or missing required fields raise exceptions immediately, before any physics engine is initialised.

The hierarchy:

```
ExperimentOptions
├── simulation: SimulationOptions
├── animats: list[AnimatOptions]
│   ├── morphology: MorphologyOptions
│   │   ├── links: list[LinkOptions]     ← drag_coefficients, density
│   │   └── joints: list[JointOptions]  ← limits, initial state
│   ├── spawn: SpawnOptions
│   ├── control: ControlOptions
│   │   └── motors: list[MotorOptions]  ← equation, control_types
│   └── sensors: SensorsOptions
└── arenas: list[ArenaOptions]
    └── water: WaterOptions
```

See [farms_core.model.options](api/farms_core_options.md) for the full class reference and parameter tables.

### Data

Data objects are pre-allocated NumPy arrays initialised with shape `[buffer_size, n_sensors, data_dim]`. The buffer uses modulo indexing (`iteration % buffer_size`) to avoid dynamic allocation during the hot loop.

The hierarchy:

```
ExperimentData
└── animats: list[AnimatData]
    └── sensors: SensorsData
        ├── links: LinkSensorArray      ← positions, velocities, orientations
        ├── joints: JointSensorArray    ← positions, velocities, torques
        ├── contacts: ContactsArray     ← reaction forces
        ├── xfrc: XfrcArray            ← external forces applied
        └── muscles: MusclesArray       ← muscle torque components
```

See [farms_core.sensors.data](api/farms_core_sensors.md) for the full sensor array reference.

### Accessing Sensor Data in a Controller

Inside any controller or extension method, sensor data is accessed through the ring buffer. **Always use modulo indexing** — accessing raw `iteration` as the index will cause an `IndexError` when `iteration >= buffer_size`.

```python
from farms_core.model.control import AnimatController, ControlType
import numpy as np

class ClosedLoopController(AnimatController):

    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
        controller = cls(
            animat_i=animat_i,
            joints_names=animat_data.joints.names,
            muscles_names=(),
            max_torques=animat_data.joints.max_torques,
        )
        # Store reference for sensor access inside control methods
        controller._data = animat_data
        controller._buffer_size = experiment_options.simulation.runtime.buffer_size
        return controller

    def positions(self, iteration: int, time: float, timestep: float) -> dict:
        # IMPORTANT: Always use modulo index for ring buffer access
        idx = iteration % self._buffer_size

        # Read current joint positions [n_joints] in radians
        joint_positions = self._data.sensors.joints.positions.array[idx]

        # Read current joint velocities [n_joints] in rad/s
        joint_velocities = self._data.sensors.joints.velocities.array[idx]

        # Read contact forces (if contacts are configured) [n_contacts, 6]
        # contact_forces = self._data.sensors.contacts.array[idx]

        # Read external forces applied to links (hydrodynamic) [n_links, 6]
        # xfrc = self._data.sensors.xfrc.array[idx]

        # Example: simple PD control using current joint positions
        commands = {}
        for i, joint_name in enumerate(self.joints_names[ControlType.POSITION]):
            target = 0.4 * np.sin(2 * np.pi * time)
            error = target - joint_positions[i]
            commands[joint_name] = target + 0.1 * error
        return commands
```

---

## `ControlType` Enum

`ControlType` (`farms_core.model.control`) is an `IntEnum` that identifies which output signal a motor expects. Each `AnimatController` output method (`positions`, `velocities`, `torques`, etc.) corresponds to exactly one `ControlType` index.

| Value | `int` | String form | Method | Description |
|-------|-------|-------------|--------|-------------|
| `POSITION` | 0 | `'position'` | `positions()` | Target joint angle [rad] |
| `VELOCITY` | 1 | `'velocity'` | `velocities()` | Target joint velocity [rad/s] |
| `TORQUE` | 2 | `'torque'` | `torques()` | Direct torque command [N·m] |
| `SPRINGREF` | 3 | `'springref'` | `springrefs()` | Spring reference angle [rad] |
| `SPRINGCOEF` | 4 | `'springcoef'` | `springcoefs()` | Spring stiffness override [N·m/rad] |
| `DAMPINGCOEF` | 5 | `'dampingcoef'` | `dampingcoefs()` | Damping coefficient override [N·m·s/rad] |
| `MUSCLE` | 6 | `'muscle'` | `excitations()` | Muscle excitation (dimensionless, 0–1) |

The `AnimatController` constructor requires a tuple of exactly **7 lists** of joint names — one per `ControlType`. Motors not assigned to a given type get an empty list at that index.

Conversion utilities:

```python
from farms_core.model.control import ControlType

# String → enum
ControlType.from_string('position')   # → ControlType.POSITION

# Enum → string
ControlType.to_string(ControlType.TORQUE)  # → 'torque'

# Parse a list of strings (from YAML)
ControlType.from_string_list(['position', 'torque'])  # → [0, 2]
```

---

## `SpawnMode` Enum

`SpawnMode` (`farms_core.model.options`) controls the degrees of freedom applied to the animat's base link at spawn. Set via `animat.spawn.mode` in YAML.

| Value | Description |
|-------|-------------|
| `free` | Fully free-floating, 6-DoF (default) |
| `fixed` | Base link is world-fixed |
| `rotx` | Rotation around X-axis only |
| `roty` | Rotation around Y-axis only |
| `rotz` | Rotation around Z-axis only |
| `sagittal` | Move in sagittal plane (XZ), rotate around Y |
| `sagittal0` | Move in sagittal plane (XZ), no rotations |
| `sagittal3` | Move in sagittal plane (XZ), all rotations |
| `coronal` | Move in coronal plane (YZ), rotate around X |
| `coronal0` | Move in coronal plane (YZ), no rotations |
| `coronal3` | Move in coronal plane (YZ), all rotations |
| `transverse` | Move in transverse plane (XY), rotate around Z |
| `transverse0` | Move in transverse plane (XY), no rotations |
| `transverse3` | Move in transverse plane (XY), all rotations |

---

## `SimulationOptions` Parameter Reference

### `RuntimeSimulationOptions` (`simulation.runtime`)

| Name | Type | Default | Description |

|-------|------|---------|-------------|
| `n_iterations` | `int` | `1000` | Total simulation steps to run |
| `buffer_size` | `int` | `n_iterations` | Ring-buffer size for sensor arrays; `0` = same as `n_iterations` |
| `play` | `bool` | `True` | Start unpaused in interactive viewer |
| `rtl` | `float` | `1.0` | Real-time limiter (1.0 = real-time, 0.5 = half-speed) |
| `fast` | `bool` | `False` | Run as fast as possible, bypassing `rtl` |
| `headless` | `bool` | `False` | No viewer window; required for SSH/cluster runs |
| `show_progress` | `bool` | `True` | Show `tqdm` progress bar in headless mode |

### `PhysicsSimulationOptions` (`simulation.physics`)

| Name | Type | Default | Description |

|-------|------|---------|-------------|
| `timestep` | `float` | `0.001` | Physics integration timestep [s] |
| `gravity` | `list[float]` | `[0, 0, -9.81]` | Gravity vector [m/s²] |
| `num_sub_steps` | `int` | `1` | MuJoCo-internal sub-steps per `env.step()` call |
| `cb_sub_steps` | `int` | `0` | FARMS callback sub-steps per outer loop iteration (0 = disabled) |
| `n_solver_iters` | `int` | `50` | Maximum MuJoCo solver iterations per step |

!!! note "`cb_sub_steps` and Extension Call Rate"
    When `cb_sub_steps > 0`, `before_step()` is called `cb_sub_steps` times per outer `env.step()` call. For Zbot (`cb_sub_steps: 2`, `timestep: 0.002 s`), extensions run at 1 kHz. When `cb_sub_steps: 0`, `before_step()` is called once per step at the physics timestep rate.

### `SensorsOptions` (`animat.control.sensors`)

| Parameter | Type | Description |
|-----------|------|-------------|
| `links` | `list[str]` | Link names to track (position, velocity, orientation, quaternion) |
| `joints` | `list[str]` | Joint names to track (position, velocity, torque) |
| `contacts` | `list[str]` or `list[list[str]]` | Link names for contact sensing; or link pairs `[link_a, link_b]` for pairwise contacts |
| `xfrc` | `list[str]` | Link names for external force sensing (reads `physics.data.xfrc_applied`) |
| `muscles` | `list[str]` | Muscle names to track (torque components) |
| `adhesions` | `list[str]` | Adhesion names to track |
| `visuals` | `list[str]` | Visual marker names to track |

### `MotorOptions` (`animat.control.motors[i]`)

| Parameter | Type | Description |
|-----------|------|-------------|
| `joint_name` | `str` | Joint to actuate; must match `morphology.joints[*].name` exactly |
| `control_types` | `list[str]` | Active control modes for this motor (e.g., `['position']`) |
| `limits_torque` | `list[float]` | Torque limits `[min_Nm, max_Nm]` |
| `gains` | `list[float]` | PD gains `[Kp, Kd]` for position control |

---

## The Extension System

All domain-specific runtime logic hooks into the physics loop via the `TaskExtension` abstract base class. This is the primary extension point for custom controllers, force models, and loggers.

```python
class TaskExtension(ABC):
    def initialize_episode(self, task, physics): ...  # called once at start
    def before_step(self, task, action, physics): ... # called before physics.step()
    def after_step(self, task, physics): ...          # called after physics.step()
```

The specialisation chain:

```
TaskExtension (ABC)
└── AnimatExtension (ABC)        ← bound to a specific animat index
    └── AnimatController (ABC)   ← provides positions/velocities/torques/excitations
```

For the full interface contract including all 11 `TaskExtension` methods and the 7 `AnimatController` output methods, see [farms_core control interfaces](api/farms_core_control.md).

### Implementing a custom controller

```python
from farms_core.model.control import AnimatController
import numpy as np

class SineWaveController(AnimatController):
    def positions(self, iteration: int, time: float, timestep: float) -> dict[str, float]:
        target = np.sin(time * 2.0 * np.pi)
        return {
            joint: target * 0.5
            for joint in self.joints_names[0]  # ControlType.POSITION
        }
```

Register it in your experiment YAML under `animats[0].extensions`.

---

## I/O

| Module | Purpose |
|--------|---------|
| `farms_core.io.sdf` | Parse SDF/URDF robot model files → link and joint lists |
| `farms_core.io.hdf5` | Save/load `AnimatData` sensor arrays to HDF5 |
| `farms_core.io.yaml` | Load YAML to Python objects (`yaml2pyobject`) |

See [farms_core.io](api/farms_core_io.md) for the function reference.

---

## Built-in Extensions

`farms_core` ships two ready-to-use `TaskExtension` implementations:

| Class | Behaviour |
|-------|-----------|
| `ExperimentLogger` | Saves `simulation.hdf5` to `log_path` at end of episode |
| `ExperimentOptionsLogger` | Saves YAML copies of all options to `log_path` at episode start |

Register either in YAML under `simulation.extensions` — no custom code required.

---

## See Also

- [Controller & Extension Interfaces](api/farms_core_control.md)
- [Sensor Data Arrays](api/farms_core_sensors.md)
- [Options Hierarchy](api/farms_core_options.md)
- [I/O: SDF, HDF5, YAML](api/farms_core_io.md)
- [System Architecture](./architecture.md)
- **Source**: `farms_core/farms_core/`