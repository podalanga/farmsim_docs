# `farms_core` — Framework Foundation

`farms_core` is the base layer of FARMS. It defines data schemas, pre-allocated sensor arrays, configuration dataclasses, the extension/controller interface, and I/O utilities. It has no dependency on any physics engine.

**Source**: `farms_core/farms_core/`

---

## Overview

`farms_core` separates two orthogonal concerns: **configuration** (Options classes, parsed from YAML before the simulation starts) and **runtime state** (Data classes, pre-allocated NumPy arrays written during the simulation loop).

A third concern, the **extension system**, defines the interface contract that all domain-specific code (controllers, hydrodynamics, loggers) must implement to hook into the simulation lifecycle.

---

## Options and Data

### Options

Options classes are mutable dictionary subclasses (inheriting from `dict` with custom `__getattr__` and `__setattr__` behaviors) deserialised from YAML via `Options.load(path)`. They are validated at load time; incorrect field types or missing required fields raise exceptions immediately, before any physics engine is initialised.

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
│   │   ├── controller_loader: str
│   │   ├── hill_muscles: list[HillMuscleOptions]
│   │   └── motors: list[MotorOptions]  ← joint_name, control_types, limits_torque, gains
│   ├── sensors: SensorsOptions
│   └── extensions: list[str]
└── arenas: list[ArenaOptions]
    └── water: WaterOptions
```

See [farms_core.model.options](api/farms_core_options.md) for the full class reference and parameter tables.

### How YAML is Loaded

The `experiment_config.yaml` acts as a manifest. Its `loaders` section maps each YAML section to a specific Python class used to deserialise it:

```yaml
# experiment_config.yaml
loaders:
  simulation_options: farms_core.simulation.options.SimulationOptions
  animats_options:
    - farms_amphibious.model.options.AmphibiousOptions   # extends AnimatOptions
  arenas_options:
    - farms_amphibious.model.options.AmphibiousArenaOptions
  experiment_data: farms_amphibious.data.data.AmphibiousExperimentData
  animats_data:
    - farms_amphibious.data.data.AmphibiousData
```

`farms_sim` reads the `loaders` section, imports each class dynamically with `importlib.import_module`, and calls `ClassName.load(yaml_path)` on the corresponding section file. This is what allows the framework to use domain-specific option subclasses (like `AmphibiousOptions`) without `farms_sim` knowing about them in advance.

**What happens on error**: If a required field is missing or has the wrong type, `Options.load()` raises a `KeyError` or `TypeError` at startup — before MuJoCo is even initialised. This is intentional: fail fast, fail clearly.

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
        ├── muscles: MusclesArray       ← muscle torque components
        ├── adhesions: AdhesionsArray   ← adhesion forces
        └── visuals: VisualsArray       ← visual markers
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

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `n_iterations` | `int` | `1000` | Total simulation steps to run |
| `buffer_size` | `int` | `n_iterations` | Ring-buffer size for sensor arrays; `0` = same as `n_iterations` |
| `play` | `bool` | `True` | Start unpaused in interactive viewer |
| `rtl` | `float` | `1.0` | Real-time limiter (1.0 = real-time, 0.5 = half-speed) |
| `fast` | `bool` | `False` | Run as fast as possible, bypassing `rtl` |
| `headless` | `bool` | `False` | No viewer window; required for SSH/cluster runs |
| `show_progress` | `bool` | `True` | Show `tqdm` progress bar in headless mode |

### `PhysicsSimulationOptions` (`simulation.physics`)

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `timestep` | `float` | `0.001` | Physics integration timestep [s] |
| `gravity` | `list[float]` | `[0, 0, -9.81]` | Gravity vector [m/s²] |
| `num_sub_steps` | `int` | `1` | MuJoCo-internal sub-steps per `env.step()` call |
| `cb_sub_steps` | `int` | `0` | FARMS callback sub-steps per outer loop iteration (0 = disabled) |
| `n_solver_iters` | `int` | `50` | Maximum MuJoCo solver iterations per step |

!!! note "`cb_sub_steps` and Extension Call Rate"
    When `cb_sub_steps > 0`, `before_step()` is called `cb_sub_steps` times per outer `env.step()` call. For Zbot (`cb_sub_steps: 2`, `timestep: 0.002 s`), extensions run at 1 kHz. When `cb_sub_steps: 0`, `before_step()` is called once per step at the physics timestep rate.

### `SensorsOptions` (`animat.control.sensors`)

| Field | Type | Description |
|-------|------|-------------|
| `links` | `list[str]` | Link names to track (position, velocity, orientation, quaternion) |
| `joints` | `list[str]` | Joint names to track (position, velocity, torque) |
| `contacts` | `list[str]` or `list[list[str]]` | Link names for contact sensing; or link pairs `[link_a, link_b]` for pairwise contacts |
| `xfrc` | `list[str]` | Link names for external force sensing (reads `physics.data.xfrc_applied`) |
| `muscles` | `list[str]` | Muscle names to track (torque components) |
| `adhesions` | `list[str]` | Adhesion names to track |
| `visuals` | `list[str]` | Visual marker names to track |

### `MotorOptions` (`animat.control.motors[i]`)

| Field | Type | Description |
|-------|------|-------------|
| `joint_name` | `str` | Joint to actuate; must match `morphology.joints[*].name` exactly |
| `control_types` | `list[str]` | Active control modes for this motor (e.g., `['position']`) |
| `limits_torque` | `list[float]` | Torque limits `[min_Nm, max_Nm]` |
| `gains` | `list[float]` | PD gains `[Kp, Kd]` for position control |

---

## The Extension System

All domain-specific runtime logic hooks into the physics loop via the `TaskExtension` abstract base class. This is the primary extension point for custom controllers, force models, and loggers.

```python
class TaskExtension(ABC):
    def initialize_episode(self, task, physics): ...  # called once at iteration 0
    def before_step(self, task, action, physics): ... # called before physics.step()
    def after_step(self, task, physics): ...          # called after physics.step()
```

### Lifecycle Details

- **`initialize_episode`** is called exactly **once**, at iteration 0, before the first physics step. This is the correct place to pre-cache MuJoCo body IDs (which are integer indices into `physics.data` arrays). Looking up body IDs by name every step via `physics.model.body(name).id` is slow — cache them here.

- **`before_step`** is called **`cb_sub_steps` times** per outer `env.step()` call. At the default Zbot configuration (`cb_sub_steps: 2`, `timestep: 0.002 s`), this runs at 1 kHz. Keep this method fast — it is the hot path.

- **`after_step`** is called once per `env.step()`. Use it for post-step bookkeeping (e.g., computing derived quantities after the physics have settled).

- **Extension order**: Extensions are executed **in the order they appear in the YAML list**. If `SwimmingExtension` is listed before your controller, hydrodynamics will compute forces on the robot's state from the *previous* step. Order matters.

The specialisation chain:

```
TaskExtension (ABC)
└── AnimatExtension (ABC)        ← bound to a specific animat index
    └── AnimatController (ABC)   ← provides positions/velocities/torques/excitations
```

For the full interface contract including all 11 `TaskExtension` methods and the 7 `AnimatController` output methods, see [farms_core control interfaces](api/farms_core_control.md).

### Implementing a Custom Controller

```python
from farms_core.model.control import AnimatController, ControlType
import numpy as np

class SineController(AnimatController):

    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
        # Build the 7-tuple of joint name lists, one per ControlType
        all_joints = [m.joint_name for m in animat_options.control.motors]
        joints_by_type = cls.joints_from_control_types(
            joints_names=all_joints,
            joints_control_types={
                m.joint_name: ControlType.from_string_list(m.control_types)
                for m in animat_options.control.motors
            },
        )
        max_torques = cls.max_torques_from_control_types(
            joints_names=all_joints,
            max_torques={m.joint_name: m.limits_torque[1] for m in animat_options.control.motors},
            joints_control_types={
                m.joint_name: ControlType.from_string_list(m.control_types)
                for m in animat_options.control.motors
            },
        )
        return cls(
            animat_i=animat_i,
            joints_names=joints_by_type,
            muscles_names=[],
            max_torques=max_torques,
        )

    def positions(self, iteration: int, time: float, timestep: float) -> dict:
        return {
            name: 0.5 * np.sin(2 * np.pi * time)
            for name in self.joints_names[ControlType.POSITION]
        }
```

Register it in `animat_config.yaml` under `control.controller_loader`.

---

## I/O

| Module | Purpose |
|--------|---------|
| `farms_core.io.sdf` | Parse SDF/URDF robot model files → link and joint lists |
| `farms_core.io.hdf5` | Save/load `AnimatData` sensor arrays to HDF5 |
| `farms_core.io.yaml` | Load YAML to Python objects (`yaml2pyobject`) |

See [farms_core.io](api/farms_core_io.md) for the function reference.

---

## Built-in Utility Functions

`farms_core.utils.transform` provides Cython-accelerated quaternion and rotation utilities used throughout the hydrodynamics code. These are available for use in custom extensions.

| Function | Signature | Description |
|----------|-----------|-------------|
| `quat2euler` | `quat2euler(quat: ndarray, out: ndarray)` | Convert a quaternion `[x, y, z, w]` to Euler angles `[roll, pitch, yaw]` in radians |
| `quat_conj` | *(cdef, internal)* | Quaternion conjugate — equivalent to the inverse for unit quaternions |
| `quat_mult` | *(cdef, internal)* | Hamilton product of two quaternions |
| `quat_rot` | *(cdef, internal)* | Rotate a 3D vector using the quaternion sandwich product `q · v · q*` |

**Usage example** — extract Euler angles from a link's orientation during a simulation step:

```python
import numpy as np
from farms_core.utils.transform import quat2euler

class HeadingLogger(TaskExtension):
    def initialize_episode(self, task, physics):
        # Pre-allocate output arrays to avoid per-step allocation
        self._euler = np.zeros(3)

    def before_step(self, task, action, physics):
        idx = task.iteration % task.buffer_size
        # physics.data.xquat shape: [n_bodies, 4] in [x, y, z, w] order
        head_quat = physics.data.xquat[self._head_body_id]
        quat2euler(head_quat, self._euler)
        yaw_rad = self._euler[2]
```

---

## Built-in Extensions

`farms_core` ships two ready-to-use `TaskExtension` implementations:

| Class | Behaviour |
|-------|-----------|
| `ExperimentLogger` | Saves `simulation.hdf5` to `log_path` at end of episode. `skip` parameter controls how many steps are skipped between each logged frame (0 = log every step). |
| `ExperimentOptionsLogger` | Saves YAML copies of all options (`simulation_options.yaml`, `animat_0_options.yaml`, etc.) to `log_path` at episode start. Useful for reproducibility. |

Register either in YAML under `simulation.extensions`:

```yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: Output
      skip: 0   # Log every step; set to N to log every N+1 steps
  - loader: farms_core.simulation.extensions.ExperimentOptionsLogger
    config:
      log_path: Output
```

---

## Common Pitfalls

!!! warning "Ring Buffer Index"
    **Always use `iteration % buffer_size`** when accessing sensor arrays, never `iteration` directly. Once `iteration` exceeds `buffer_size`, a raw index will throw an `IndexError`. The modulo pattern is the only safe way to access the circular buffer.

    ```python
    # WRONG — will crash when iteration >= buffer_size
    pos = self._data.sensors.joints.positions.array[iteration]

    # CORRECT
    pos = self._data.sensors.joints.positions.array[iteration % self._buffer_size]
    ```

!!! warning "Extension Execution Order"
    Extensions run in the exact order they are listed in the YAML file. If your controller is listed *after* `SwimmingExtension`, the hydrodynamics forces applied to the robot are computed from the *previous timestep's* joint positions — the controller hasn't moved the joints yet. List your controller **first** if it needs to respond to current physics state, or **after** if it feeds forward.

!!! warning "Cython Rebuild Required"
    Editing a `.pyx` file does **not** take effect until you recompile the Cython extension. Pure Python (`.py`) changes are live immediately. After any `.pyx` edit:
    ```bash
    pip install -e ./farms/farms_core --no-build-isolation
    # or for farms_amphibious:
    pip install -e ./farms/farms_amphibious --no-build-isolation
    ```

---

## See Also

- [Controller & Extension Interfaces](api/farms_core_control.md)
- [Sensor Data Arrays](api/farms_core_sensors.md)
- [Options Hierarchy](api/farms_core_options.md)
- [I/O: SDF, HDF5, YAML](api/farms_core_io.md)
- [System Architecture](architecture.md)
- **Source**: `farms_core/farms_core/`
