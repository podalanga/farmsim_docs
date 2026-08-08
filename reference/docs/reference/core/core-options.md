# farms_core.model.options

Configuration dataclasses parsed from YAML, defining the full experiment setup.

## Overview

The `farms_core.model.options` module defines the configuration schemas that dictate the Animat and the environment properties. The parameters are usually defined in YAML files and deserialised into Python dataclass-like objects inheriting from the `Options` base class. This allows robust and explicit definition of morphological properties, control architectures, and simulation physics settings.

## Options Base Class

All configuration classes inherit from `Options`.

```python
class Options(dict):
    @classmethod
    def load(cls, filename: str, strict: bool = True) -> 'Options':
        pass

    def save(self, filename: str):
        pass

    def to_dict(self) -> dict:
        pass
```

**Note**: `Options` is a `dict` subclass, not a frozen dataclass. It provides attribute-style access via a custom `__getattr__` (which falls back to `self[name]`) and `__setattr__ = dict.__setitem__`.

!!! warning "`from_options` is not defined on the base class"
    The base `Options` class does **not** define `from_options`. It is implemented
    as a `@classmethod` by individual subclasses (e.g. `SpawnOptions.from_options`,
    `ControlOptions.from_options`, `SensorsOptions.from_options`), each with its
    own signature. `Options.load` and `Options.save` (which delegate to
    `yaml2pyobject` / `pyobject2yaml`) are the only serialisation helpers provided
    by the base class.

## ExperimentOptions

The top-level container for a complete simulation setup.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `simulation` | SimulationOptions | *(required)* | Simulation physics and runtime parameters |
| `animats` | list[AnimatOptions] | *(required)* | List of animats in the simulation |
| `arenas` | list[ArenaOptions] | *(required)* | List of arenas/terrains |
| `loaders` | ExperimentLoadOptions | *(required)* | Loader class paths for options and data |

## AnimatOptions

Defines a single robot/animat entity.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `sdf` | str | - | Path to the SDF model file |
| `morphology` | MorphologyOptions | - | Overrides for link and joint properties |
| `spawn` | SpawnOptions | - | Rules for placing the animat in the world |
| `control` | ControlOptions | - | Actuation and controller logic (includes `sensors`) |
| `extensions` | list[AnimatExtensionOptions] | `[]` | Additional plugins (e.g., swimming, contacts) |

## MorphologyOptions

Describes the morphological overrides for rigid bodies and joints.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `links` | list[LinkOptions] | - | Properties for individual rigid bodies |
| `self_collisions` | list[list[str]] | - | Pairs of links allowed/denied to self-collide |
| `joints` | list[JointOptions] | - | Properties for individual constraints/joints |
| `tendons` | list[TendonOptions] | `[]` | Tendon transmission properties |

### LinkOptions

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `name` | str | - | Identifier matching the SDF link |
| `collisions` | bool | - | Toggles collision geometry |
| `friction` | list[float] | - | Lateral, spinning, rolling friction |
| `fluid_interaction` | bool | `False` | Enables buoyancy and drag force calculation |
| `density` | float | `1000` | Mass density in kg/m³ |
| `drag_coefficients` | list[float] | `[0, 0, 0, 0, 0, 0]` | Linear and angular hydrodynamic drag components `[Vx, Vy, Vz, Wx, Wy, Wz]` |
| `sites` | list[SiteOptions] | `[]` | Reference markers for motion tracking |
| `solref` | - | `None` | MuJoCo constraint solver reference |
| `solimp` | - | `None` | MuJoCo constraint solver impedance |
| `extras` | dict | `{}` | Extra options (deprecated) |

### JointOptions

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `name` | str | - | Identifier matching the SDF joint |
| `initial` | list[float] | - | Initial state `[position (rad), velocity (rad/s)]` |
| `limits` | list[list[float]] | - | Range of motion limits in rad |
| `stiffness` | float | - | Joint spring stiffness in Nm/rad |
| `damping` | float | - | Joint friction/damping in N·m·s/rad |
| `springref` | float | - | Spring equilibrium position in rad |

## Spawn Configuration

Dictates how and where the animat is instantiated into the simulation world.

### SpawnOptions

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `loader` | SpawnLoader | *(required)* | Physics engine loading method (defaults to `FARMS` only via `from_options`) |
| `mode` | SpawnMode | `FREE` | Base constraints |
| `pose` | list[float] | - | Spawn position (m) and orientation (rad) `[x,y,z,R,P,Y]` |
| `velocity` | list[float] | - | Spawn linear (m/s) and angular (rad/s) velocity |
| `extras` | dict | `{}` | Extra options (deprecated) |

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

## ControlOptions

Defines how the animat thinks and acts.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `controller_loader` | str | `''` | Python class path for the controller |
| `sensors` | SensorsOptions | - | Telemetry extraction |
| `motors` | list[MotorOptions] | - | Actuator mappings |
| `hill_muscles` | list[MuscleOptions] | `[]` | Hill-type muscle definitions |

### MotorOptions

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `joint_name` | str | - | Target joint identifier |
| `control_types` | list[str] | - | Actuation modes (e.g., position, velocity, torque) |
| `limits_torque` | list[float] | - | Torque limits `[min, max]` in N·m |
| `gains` | list[float] | - | Proportional and derivative gains `[Kp, Kd]` for position control |

### SensorsOptions

Lists the names of elements to track in telemetry. Typically contains lists of strings for `links`, `joints`, `contacts`, `xfrc`, `muscles`, `adhesions`, and `visuals`.

## Environment and Simulation Options

### WaterOptions

Defines fluid dynamics parameters used by hydrodynamic extensions.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `sdf` | str | - | Path to water SDF file |
| `drag` | bool | - | Enables drag forces |
| `buoyancy` | bool | - | Enables buoyancy forces |
| `height` | float | - | Surface level *Z*-coordinate in m |
| `velocity` | list[float] | - | Flow vector `[Vx, Vy, Vz]` in m/s |
| `viscosity` | float | - | Fluid dynamic viscosity in Pa·s |
| `density` | float | - | Fluid density in kg/m³ |
| `maps` | list[str] | - | Water maps sourced from images |

### ArenaOptions

Defines the terrain, specifying properties such as a ground plane, friction limits, or a heightmap. Inherits `sdf` and `spawn` from `ModelOptions`.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `sdf` | str | - | Path to the arena SDF model file |
| `spawn` | SpawnOptions | - | Spawn pose/velocity for the arena |
| `water` | WaterOptions | - | Fluid dynamics configuration |
| `ground_height` | float | - | Height offset at which to place the arena |

### SimulationOptions

Contains subsets of parameters defining the physics engine configuration. Top-level fields are `units` (`SimulationUnitScaling`), `runtime` (`RuntimeSimulationOptions`), `physics` (`PhysicsSimulationOptions`), `mujoco` (`MuJoCoSimulationOptions`), `pybullet` (`PybulletSimulationOptions`), and `extensions` (list of `SimulationExtensionOptions`). The `timestep` (s) and `gravity` (m/s²) live under the `physics` sub-options (`physics.timestep`, `physics.gravity`).

## See Also

- [Configuration Reference](../env/yaml-schema.md)
- [farms_amphibious.model.options](../amphibious/amphibious-options.md)
