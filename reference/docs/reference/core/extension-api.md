# Extension API

Reference for the FARMS extension system — the lifecycle interfaces, base
classes, and registration mechanism.

## Class hierarchy

```
TaskExtension (ABC)                     farms_core.simulation.extensions
├── ExperimentLogger                    farms_core.simulation.extensions
├── ExperimentOptionsLogger             farms_core.simulation.extensions
├── MjcfSaver                           farms_mujoco.simulation.extensions
├── CameraFollower                      farms_mujoco.simulation.extensions
├── CameraRecording                     farms_mujoco.sensors.camera
├── CoMViewer                           farms_mujoco.simulation.extensions
├── TrailCoMViewer                      farms_mujoco.simulation.extensions
├── TrailLinkViewer                     farms_mujoco.simulation.extensions
├── ArrowViewer                         farms_mujoco.simulation.extensions
└── AnimatExtension (ABC)               farms_core.model.extensions
    ├── AnimatController                farms_core.model.control
    │   ├── JointMuscleController        farms_amphibious.control.amphibious
    │   └── ZbotCPGController             experiments/zbot_bout_glide/controller
    └── SwimmingExtension                farms_mujoco.swimming.extension
```

!!! bug "Corrected — `MjcfSaver`/`CameraFollower`/marker viewers were nested under `AnimatExtension`"
    An earlier draft of this diagram nested `MjcfSaver`, `CameraFollower`,
    `CoMViewer`, `TrailCoMViewer`, `TrailLinkViewer`, and `ArrowViewer`
    under `AnimatExtension`. They subclass `TaskExtension` directly
    (`farms_mujoco/simulation/extensions.py`), and their `from_options`
    takes the 2-argument `TaskExtension` signature (`config`,
    `experiment_options`), not the 5-argument `AnimatExtension` one. They're
    registered in `simulation_config.yaml`, not `animat_config.yaml` — see
    [Use Built-in Extensions](../../how-to/use-extensions.md). Also added
    `CameraRecording` (`farms_mujoco/sensors/camera.py`), an offscreen
    moving-camera video-export `TaskExtension` that was missing from this
    reference entirely.

## TaskExtension

**Source:** `farms_core/simulation/extensions.py`

Abstract base class for simulation-level extensions.

```python
class TaskExtension(ABC):
    @classmethod
    @abstractmethod
    def from_options(cls, config: dict, experiment_options: ExperimentOptions):
        """Factory method — called during simulation setup."""
        ...

    def __init__(self, substep=False):
        """`substep=True` runs before_step()/after_step() on every physics
        substep instead of only on full simulation steps."""
        ...

    def initialize_episode(self, task, physics):
        """Called once at the start of an episode. Optional — default is no-op."""
        ...

    def before_step(self, task, action, physics):
        """Called before each physics step. Optional — default is no-op."""
        ...

    def after_step(self, task, physics):
        """Called after each physics step. Optional — default is no-op."""
        ...

    def end_episode(self, task, physics):
        """Called at episode end. Optional — default is no-op."""
        ...
```

!!! note "Only `from_options` is actually abstract"
    `initialize_episode`/`before_step`/`after_step`/`end_episode` all have
    concrete no-op default bodies in `TaskExtension` — only `from_options`
    carries `@abstractmethod`. A subclass is free to override only the
    lifecycle methods it needs. Also note `after_step(self, task, physics)`
    takes no `action` parameter (unlike `before_step`).

### TaskExtension.from_options() signature

```python
@classmethod
def from_options(cls, config: dict, experiment_options: ExperimentOptions):
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `config` | dict | The `config:` block from this extension's YAML entry |
| `experiment_options` | ExperimentOptions | Full experiment configuration |

## AnimatExtension

**Source:** `farms_core/model/extensions.py`

Abstract base class for animat-level extensions. Extends `TaskExtension`.

```python
class AnimatExtension(TaskExtension):
    @classmethod
    @abstractmethod
    def from_options(cls, config, experiment_options, animat_i,
                    animat_data, animat_options):
        """Factory method — called during simulation setup."""
        ...
```

### AnimatExtension.from_options() signature

```python
@classmethod
def from_options(cls, config, experiment_options, animat_i,
                animat_data, animat_options):
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `config` | dict | Config dict from YAML `extensions` entry |
| `experiment_options` | ExperimentOptions | Full experiment configuration |
| `animat_i` | int | Animat index (0-based) |
| `animat_data` | AnimatData | Pre-allocated data for this animat |
| `animat_options` | AnimatOptions | Options for this animat |

## AnimatController

**Source:** `farms_core/model/control.py`

Base class for controllers. Extends `AnimatExtension`.

### Constructor

```python
def __init__(self, animat_i, joints_names, muscles_names,
             max_torques, substep=True):
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `animat_i` | int | Animat index |
| `joints_names` | list[str] | Joint names grouped by control type |
| `muscles_names` | list[str] | Muscle names |
| `max_torques` | list[float] | Max torque per joint |
| `substep` | bool | Run at substep resolution (default: True) |

### Output methods

```python
def positions(self, iteration, time, timestep) -> dict[str, float]
def velocities(self, iteration, time, timestep) -> dict[str, float]
def torques(self, iteration, time, timestep) -> dict[str, float]
```

Each method returns a dict mapping joint names to target values. Only joints
using the corresponding control type need to be included.

### Static helper methods

```python
@staticmethod
def joints_from_control_types(joints_names, joints_control_types) -> list[str]

@staticmethod
def max_torques_from_control_types(joints_names, max_torques,
                                   joints_control_types) -> list[float]
```

### before_step()

```python
def before_step(self, task, action, physics):
    """Advances internal dynamics. Called before output methods."""
```

The `ExperimentTask` calls `before_step()` on all extensions (including
controllers), then collects controller outputs via `positions()`,
`velocities()`, `torques()`, and applies them to `physics.data.ctrl`.

## ControlType

**Source:** `farms_core/model/control.py`

```python
class ControlType(IntEnum):
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2
    SPRINGREF = 3
    SPRINGCOEF = 4
    DAMPINGCOEF = 5
    MUSCLE = 6
```

### ControlType.from_string_list()

```python
@classmethod
def from_string_list(cls, control_types: list[str]) -> list[ControlType]
```

Converts string names (e.g., `["position", "torque"]`) to a list of
`ControlType` enum values.

## Registration mechanism

Extensions are registered in YAML configuration files:

```yaml
# Simulation-level (simulation_config.yaml)
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: ./simulation.hdf5

# Animat-level (animat_config.yaml)
extensions:
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config: {}
```

During `ExperimentTask` initialization:

1. `extract_extensions()` reads the `extensions:` list from each config
2. For each entry, it imports the class via `import_item(loader)` (dotted path)
3. For sim-level extensions: calls `TaskExtension.from_options(config, experiment_options)`
4. For animat-level extensions: calls `AnimatExtension.from_options(config, experiment_options, animat_i, animat_data, animat_options)`
5. Controllers are identified among animat extensions and handled specially

## Built-in extensions

### farms_core.simulation.extensions

| Class | Type | Config keys | Description |
|-------|------|-------------|-------------|
| `ExperimentLogger` | TaskExtension | `log_path`, `skip` | Save HDF5 data |
| `ExperimentOptionsLogger` | TaskExtension | `log_path` | Save YAML copies |

### farms_mujoco.simulation.extensions / farms_mujoco.sensors.camera

| Class | Type | Config keys | Description |
|-------|------|-------------|-------------|
| `MjcfSaver` | TaskExtension | `path` | Save MJCF XML |
| `CameraFollower` | TaskExtension | `animat_id`, `azimuth`, `distance`, `elevation`, `angular_velocity` | Moves the interactive viewer's camera; no effect headless or on `CameraRecording` |
| `CameraRecording` | TaskExtension | `path`, `resolution`, `fps`, `speed`, `animat_id`, `offset`, `distance`, `azimuth`, `elevation`, `angular_velocity`, `geomgroups` | Offscreen, independently-cameraed video export (works headless) |
| `CoMViewer` | TaskExtension | `animat_id`, `size`, `rgba` | CoM marker (ephemeral, viewer-only) |
| `TrailCoMViewer` | TaskExtension | `animat_id`, `width`, `rgba`, `spacing` | CoM trail (ephemeral, viewer-only) |
| `TrailLinkViewer` | TaskExtension | `animat_id`, `link`, `width`, `rgba`, `spacing` | Link trail (ephemeral, viewer-only) |
| `ArrowViewer` | TaskExtension | `animat_id`, `size`, `rgba`, `offset` | Rotating pointer (ephemeral, viewer-only) |

All registered in `simulation_config.yaml`'s `extensions:` list — see
[Use Built-in Extensions](../../how-to/use-extensions.md) for full config
reference and the "objects vs ephemeral markers" distinction.

### farms_mujoco.swimming.extension

| Class | Type | Config keys | Description |
|-------|------|-------------|-------------|
| `SwimmingExtension` | AnimatExtension | (none — reads arena water options) | Hydrodynamic forces |
