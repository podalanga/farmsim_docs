# farms_core.model.options

Configuration dataclasses parsed from YAML, defining the full experiment setup.

## Overview

The `farms_core.model.options` module defines the configuration schemas that dictate the Animat and the environment properties. The parameters are usually defined in YAML files and deserialised into Python dataclass-like objects inheriting from the `Options` base class. This allows robust and explicit definition of morphological properties, control architectures, and simulation physics settings.

---

## Options Base Class

All configuration classes inherit from `Options`.

```python
class Options:
    @classmethod
    def from_options(cls, options) -> 'Options':
        pass

    @classmethod
    def load(cls, filename: str) -> 'Options':
        pass

    def save(cls, filename: str):
        pass
```

---

## ExperimentOptions

The top-level container for a complete simulation setup.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `animats` | list[AnimatOptions] | `[]` | List of animats in the simulation |
| `arenas` | list[ArenaOptions] | `[]` | List of arenas/terrains |
| `simulation` | SimulationOptions | - | Simulation physics and runtime parameters |

---

## AnimatOptions

Defines a single robot/animat entity.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `sdf` | str | - | Path to the SDF model file |
| `morphology` | MorphologyOptions | - | Overrides for link and joint properties |
| `spawn` | SpawnOptions | - | Rules for placing the animat in the world |
| `control` | ControlOptions | - | Actuation and controller logic |
| `sensors` | SensorsOptions | - | Telemetry and tracking configuration |
| `extensions` | list | `[]` | Additional plugins (e.g., swimming, contacts) |

---

## MorphologyOptions

Describes the morphological overrides for rigid bodies and joints.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `links` | list[LinkOptions] | `[]` | Properties for individual rigid bodies |
| `joints` | list[JointOptions] | `[]` | Properties for individual constraints/joints |

### LinkOptions

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `name` | str | - | Identifier matching the SDF link |
| `collisions` | bool | - | Toggles collision geometry |
| `friction` | list[float] | - | Lateral, spinning, rolling friction |
| `density` | float | `1000.0` | Mass density in kg/m³ |
| `drag_coefficients` | list[list[float]] | `[[0,0,0], [0,0,0]]`| Linear and angular hydrodynamic drag components |
| `fluid_interaction` | bool | `False` | Enables buoyancy and drag force calculation |

### JointOptions

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `name` | str | - | Identifier matching the SDF joint |
| `initial` | list[float] | - | Initial state `[position (rad), velocity (rad/s)]` |
| `limits` | list[list[float]] | - | Range of motion limits in rad |
| `stiffness` | float | - | Joint spring stiffness in Nm/rad |
| `damping` | float | - | Joint friction/damping in N·m·s/rad |
| `springref` | float | - | Spring equilibrium position in rad |

---

## Spawn Configuration

Dictates how and where the animat is instantiated into the simulation world.

### SpawnOptions

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `loader` | SpawnLoader | `FARMS` | Physics engine loading method |
| `mode` | SpawnMode | `FREE` | Base constraints |
| `pose` | list[float] | - | Spawn position (m) and orientation (rad) `[x,y,z,R,P,Y]` |
| `velocity` | list[float] | - | Spawn linear (m/s) and angular (rad/s) velocity |

### SpawnLoader (IntEnum)

- `FARMS (0)`: Recommended custom SDF loader.
- `PYBULLET (1)`: PyBullet's default SDF loader.

### SpawnMode (Enum)

| Mode | Description |
|------|-------------|
| `FREE` | Unconstrained floating base. |
| `FIXED` | Base link is rigidly attached to the world. |
| `ROTX` / `ROTY` / `ROTZ` | Constrained rotation around a single axis. |
| `SAGITTAL` / `SAGITTAL0` / `SAGITTAL3` | Constrained to the sagittal plane (XZ). Variants for rotation permissions. |
| `CORONAL` / `CORONAL0` / `CORONAL3` | Constrained to the coronal plane (YZ). |
| `TRANSVERSE` / `TRANSVERSE0` / `TRANSVERSE3` | Constrained to the transverse plane (XY). |

---

## ControlOptions

Defines how the animat thinks and acts.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `controller_loader` | str | - | Python class path for the controller |
| `sensors` | SensorsOptions | - | Telemetry extraction |
| `motors` | list[MotorOptions]| `[]` | Actuator mappings |

### MotorOptions

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `joint_name` | str | - | Target joint identifier |
| `control_types` | list[str] | - | Actuation modes (e.g., POSITION, TORQUE) |
| `limits_torque` | list[float] | - | Torque limits `[min, max]` in N·m |
| `gains` | list[float] | - | Kp/Kd gains |
| `equation` | str | - | Actuation control law |

### SensorsOptions

Lists the names of elements to track in telemetry. Typically contains lists of strings for `links`, `joints`, `contacts`, `xfrc`, `muscles`, `adhesions`, and `visuals`.

---

## Environment and Simulation Options

### WaterOptions

Defines fluid dynamics parameters used by hydrodynamic extensions.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `drag` | bool | - | Enables drag forces |
| `buoyancy` | bool | - | Enables buoyancy forces |
| `height` | float | - | Surface level $Z$-coordinate in m |
| `velocity` | list[float] | - | Flow vector `[Vx, Vy, Vz]` in m/s |
| `viscosity` | float | - | Fluid dynamic viscosity in Pa·s |
| `density` | float | - | Fluid density in kg/m³ |

### ArenaOptions

Defines the terrain, specifying properties such as a ground plane, friction limits, or a heightmap.

### SimulationOptions

Contains subsets of parameters defining the physics engine configuration. Contains options for `RuntimeSimulationOptions` and `PhysicsSimulationOptions`. Key fields often include the `timestep` (s) and `gravity` (m/s²).

---

## See Also

- [Configuration Reference](../configuration.md) — All YAML parameter definitions
- [Amphibious Options](./farms_amphibious_options.md) — Extended amphibious configuration
