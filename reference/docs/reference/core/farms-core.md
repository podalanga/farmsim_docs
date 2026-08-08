# farms_core Reference

API reference for `farms_core` — the core library providing options, model
definitions, sensors, simulation infrastructure, I/O, and experiment management.

## Module structure

```
farms_core/
├── options.py           # Options base class
├── pylog.py             # Logging
├── array.py             # Array utilities
├── units.py             # Unit definitions
├── extensions/
│   └── extensions.py    # import_item()
├── io/
│   └── yaml.py          # yaml2pyobject, pyobject2yaml
├── model/
│   ├── options.py       # ModelOptions, AnimatOptions, LinkOptions, ...
│   ├── control.py       # AnimatController, ControlType
│   ├── data.py          # AnimatData, SensorsData
│   └── extensions.py    # AnimatExtension
├── sensors/
│   └── sensor_convention.py  # sc (sensor convention)
├── simulation/
│   ├── options.py       # SimulationOptions, RunOptions, etc.
│   └── extensions.py    # TaskExtension, ExperimentLogger
├── experiment/
│   ├── options.py       # ExperimentOptions
│   └── data.py          # ExperimentData
└── analysis/            # Analysis utilities
```

## Options base class

**Source:** `farms_core/options.py`

```python
class Options(dict):
    """Base class for all FARMS configuration objects."""

    @classmethod
    def load(cls, filename: str) -> 'Options':
        """Load from YAML file using yaml2pyobject()."""

    def save(self, filename: str):
        """Save to YAML file using pyobject2yaml()."""
```

`Options` is a `dict` subclass. All option classes (SimulationOptions,
AnimatOptions, etc.) extend it. The `load()` classmethod uses
`yaml2pyobject()` to deserialize YAML into the appropriate Python objects,
using `loader` dotted paths to resolve classes.

## Simulation options

**Source:** `farms_core/simulation/options.py`

### SimulationOptions

```python
class SimulationOptions(Options):
    def __init__(self, units, run, physics, mujoco=None,
                 pybullet=None, extensions=None):
        ...
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `units` | `UnitOptions` | Unit scaling (length, angle, seconds, etc.) |
| `run` | `RunOptions` | Runtime parameters (duration, timestep) |
| `physics` | `PhysicsOptions` | Physics parameters (gravity) |
| `mujoco` | `MujocoOptions` | MuJoCo-specific options |
| `pybullet` | `PybulletOptions` | PyBullet-specific options |
| `extensions` | list[`ExtensionOptions`] | Sim-level extension configs |

### RunOptions

```python
class RunOptions(Options):
    def __init__(self, duration, timestep, n_iterations=None):
        ...
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `duration` | float | Simulation duration [s] |
| `timestep` | float | Physics timestep [s] |
| `n_iterations` | int | Total iterations (computed if not given) |

## Model options

**Source:** `farms_core/model/options.py`

### ModelOptions

```python
class ModelOptions(Options):
    def __init__(self, sdf, **kwargs):
        ...
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `sdf` | str | Path to SDF model file |

### AnimatOptions

```python
class AnimatOptions(ModelOptions):
    def __init__(self, sdf, spawn, morphology, control, extensions):
        ...
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `sdf` | str | SDF file path |
| `spawn` | `SpawnOptions` | Spawn position/orientation |
| `morphology` | `MorphologyOptions` | Links, joints, collisions |
| `control` | `ControlOptions` | Controller, sensors, motors |
| `extensions` | list[`AnimatExtensionOptions`] | Animat extensions |

### MorphologyOptions

```python
class MorphologyOptions(Options):
    def __init__(self, links, self_collisions, joints, tendons=None):
        ...
```

### LinkOptions

```python
class LinkOptions(Options):
    def __init__(self, name, collisions, friction, fluid_interaction=False,
                 density=1000, drag_coefficients=None, sites=None,
                 solref=None, solimp=None, extras=None):
        ...
```

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | str | — | Link name |
| `collisions` | bool | — | Enable collisions |
| `friction` | list[float] | — | [lateral, spinning, rolling] |
| `fluid_interaction` | bool | `False` | Enable fluid forces |
| `density` | float | `1000` | Density [kg/m³] |
| `drag_coefficients` | list[float] | `[0,0,0,0,0,0]` | 6 drag coefficients |
| `sites` | list | `[]` | Site definitions |

### JointOptions

```python
class JointOptions(Options):
    def __init__(self, name, initial, limits, stiffness, springref,
                 damping, extras=None):
        ...
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `name` | str | Joint name |
| `initial` | list[float] | [position, velocity] |
| `limits` | list[list[float]] | [[pos_min, pos_max], [vel_min, vel_max]] |
| `stiffness` | float | Joint stiffness |
| `springref` | float | Spring reference |
| `damping` | float | Joint damping |

### ControlOptions

```python
class ControlOptions(Options):
    def __init__(self, controller_loader, sensors, motors, hill_muscles=None):
        ...
```

### MotorOptions

```python
class MotorOptions(Options):
    def __init__(self, joint_name, control_types, limits_torque, gains):
        ...
```

### SensorsOptions

```python
class SensorsOptions(Options):
    def __init__(self, links, joints, contacts, xfrc, muscles,
                 adhesions, visuals):
        ...
```

All attributes are `list[str]` — lists of sensor names.

### SpawnOptions

```python
class SpawnOptions(Options):
    def __init__(self, position, orientation, mode):
        ...
```

### ArenaOptions

```python
class ArenaOptions(Options):
    def __init__(self, sdf, spawn, water=None, ground_height=0.0):
        ...
```

### WaterOptions

```python
class WaterOptions(Options):
    def __init__(self, sdf, drag, buoyancy, height,
                 velocity=None, viscosity=0.0, density, maps=None):
        ...
```

## Experiment options

**Source:** `farms_core/experiment/options.py`

### ExperimentOptions

```python
class ExperimentOptions(Options):
    def __init__(self, simulation, animats, arenas):
        ...
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `simulation` | SimulationOptions | Simulation configuration |
| `animats` | list[AnimatOptions] | Animat configurations |
| `arenas` | list[ArenaOptions] | Arena configurations |

## Data classes

**Source:** `farms_core/model/data.py`, `farms_core/experiment/data.py`

### ExperimentData

```python
class ExperimentData:
    def __init__(self, times, timestep, simulation, animats):
        ...

    @classmethod
    def from_options(cls, options: ExperimentOptions) -> 'ExperimentData':
        """Pre-allocate all data arrays from options."""

    def to_file(self, filename: str):
        """Save to HDF5."""

    @classmethod
    def from_file(cls, filename: str) -> 'ExperimentData':
        """Load from HDF5."""
```

### AnimatData

```python
class AnimatData:
    def __init__(self, sensors, state=None, network=None):
        ...
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `sensors` | `SensorsData` | All sensor arrays |
| `state` | `StateData` | CPG network state (optional) |
| `network` | `NetworkLog` | Network log (optional) |

### SensorsData

```python
class SensorsData:
    def __init__(self, links, joints, contacts, xfrc, muscles,
                 adhesions, visuals):
        ...
```

Each attribute is an array object with `.array` (numpy ndarray) and `.names`
(list[str]).

## I/O utilities

**Source:** `farms_core/io/yaml.py`

### yaml2pyobject

```python
def yaml2pyobject(filename, loader_class=None):
    """Deserialize YAML to Python object using loader dotted paths."""
```

### pyobject2yaml

```python
def pyobject2yaml(obj, filename, test=False):
    """Serialize Python Options object to YAML."""
```

## import_item

**Source:** `farms_core/extensions/extensions.py`

```python
def import_item(dotted_path: str):
    """Import a Python object by its dotted path (e.g., 'pkg.module.Class')."""
```

Used to resolve `loader` fields in YAML configuration.
