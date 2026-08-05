# `farms_core.simulation.extensions` / `farms_core.model.control`

Base classes for simulation extensions and animat controllers.

## TaskExtension

```python
class TaskExtension(ABC):
    def __init__(self, substep: bool = False):
        ...
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `substep` | `bool` | `False` | Whether this extension operates on physics substeps. |

The `TaskExtension` is an abstract base class for any simulation hook. Custom extensions should inherit from this class to inject logic into the physics loop.

### Methods

| Method | Description |
|--------|-------------|
| `from_options(cls, config: dict, experiment_options: ExperimentOptions)` | **Abstract.** Instantiates the extension from configuration options. |
| `initialize_episode(self, task: Task, physics: Physics)` | Called at simulation iteration 0. |
| `before_step(self, task: Task, action, physics: Physics)` | Called before the physics engine steps. |
| `after_step(self, task: Task, physics: Physics)` | Called after the physics engine steps. |
| `action_spec(self, task: Task, physics: Physics)` | Defines the action specification. |
| `step_spec(self, task: Task, physics: Physics)` | Defines the timestep specifications. |
| `get_observation(self, task: Task, physics: Physics)` | Retrieves the environment observation. |
| `get_reward(self, task: Task, physics: Physics)` | Computes the reward. |
| `get_termination(self, task: Task, physics: Physics)` | Returns final discount if episode should end, else None. |
| `observation_spec(self, task: Task, physics: Physics)` | Defines the observation specification. |
| `end_episode(self, task: Task, physics: Physics)` | Called at the end of the simulation. |

## AnimatExtension

```python
class AnimatExtension(TaskExtension, ABC):
    ...
```

Inherits `initialize_episode()`, `before_step()`, and all other simulation lifecycle methods from `TaskExtension` — see [TaskExtension](#taskextension). This abstract class associates the extension with a specific animat via the `animat_i` index.

### Methods

| Method | Description |
|--------|-------------|
| `from_options(cls, config: dict, experiment_options: ExperimentOptions, animat_i: int, animat_data: AnimatData, animat_options: AnimatOptions)` | **Abstract.** Instantiates the extension for a specific animat. |

## AnimatController

```python
class AnimatController(AnimatExtension):
    def __init__(self, animat_i: int, joints_names: tuple[list[str], ...], muscles_names: tuple[str, ...], max_torques: tuple[NDARRAY_V1, ...], substep: bool = True):
        ...
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `animat_i` | `int` | *(required)* | Index of the animat this controller drives. |
| `joints_names` | `tuple[list[str], ...]` | *(required)* | 7-tuple of joint name lists, one for each `ControlType`. |
| `muscles_names` | `tuple[str, ...]` | *(required)* | Tuple of muscle names. |
| `max_torques` | `tuple[NDARRAY_V1, ...]` | *(required)* | 7-tuple of maximum torques per `ControlType`. |
| `substep` | `bool` | `True` | Whether controller runs on substeps. |

The `AnimatController` bridges high-level behavior (like a CPG) to low-level physical actuation.

### Methods

The following methods output the active commands for the current iteration. Each method takes `(iteration: int, time: float, timestep: float)` and returns a `dict[str, float]` mapping the joint or actuator name to its commanded value.

| Method | Returns | Description |
|--------|---------|-------------|
| `positions` | `dict[str, float]` | Positional setpoints. |
| `velocities` | `dict[str, float]` | Velocity setpoints. |
| `torques` | `dict[str, float]` | Direct torque or force commands. |
| `springrefs` | `dict[str, float]` | Dynamic spring equilibrium references. |
| `springcoefs` | `dict[str, float]` | Dynamic stiffness coefficients. |
| `dampingcoefs` | `dict[str, float]` | Dynamic damping coefficients. |
| `excitations` | `dict[str, float]` | Muscle activation levels. |

## ControlType

```python
class ControlType(IntEnum):
    ...
```

Standardizes the modes by which joints and actuators are driven.

| Value | Code | Controls |
|-------|------|----------|
| `POSITION` | `0` | Standard positional targets. |
| `VELOCITY` | `1` | Target joint velocities. |
| `TORQUE` | `2` | Direct effort, forces, or torques. |
| `SPRINGREF` | `3` | Equilibrium point (rest length/angle) of a simulated spring. |
| `SPRINGCOEF` | `4` | Joint stiffness (spring constant *K*). |
| `DAMPINGCOEF` | `5` | Joint damping (*D*). |
| `MUSCLE` | `6` | Muscle excitation levels (activations). |

## ExperimentLogger

```python
class ExperimentLogger(TaskExtension):
    def __init__(self, experiment_options: ExperimentOptions, log_path: str, skip: int):
        ...
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `experiment_options` | `ExperimentOptions` | *(required)* | The options for the experiment. |
| `log_path` | `str` | *(required)* | Directory to save the HDF5 file. |
| `skip` | `int` | *(required)* | Number of frames to skip between logging. |

A simulation extension that logs animat data arrays into an HDF5 file (`simulation.hdf5`) at the end of the simulation.

## ExperimentOptionsLogger

```python
class ExperimentOptionsLogger(TaskExtension):
    def __init__(self, experiment_options: ExperimentOptions, log_path: str):
        ...
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `experiment_options` | `ExperimentOptions` | *(required)* | The options for the experiment. |
| `log_path` | `str` | *(required)* | Directory to save the YAML files. |

A simulation extension that writes the initial simulation, animat, and arena configuration options into distinct YAML files (`simulation_options.yaml`, `animat_X_options.yaml`, `arena_X_options.yaml`) before the simulation steps begin.

## Usage Example

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

## See Also

- [farms_core_options.md](farms_core_options.md)
- [farms_mujoco_simulation.md](farms_mujoco_simulation.md)
