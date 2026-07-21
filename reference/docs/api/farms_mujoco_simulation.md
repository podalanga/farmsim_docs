# farms_mujoco.simulation

MuJoCo physics backend — MJCF generation, task lifecycle, sensor maps, and visual extensions.

## Overview

The `farms_mujoco.simulation` module bridges the abstract FARMS definitions with the concrete MuJoCo physics engine. It handles translating experiment options into a valid MuJoCo XML, managing the exact execution order of controllers and sensors during a physics step, and providing a suite of visual extensions for debugging and rendering.

---

## Class Reference: Simulation

```python
Simulation.from_experiment(cls, experiment_options, **kwargs)
```

| Name | Type | Default | Description |
|---|---|---|---|
| `experiment_options` | `ExperimentOptions` | Required | Global experiment configuration object. |
| `**kwargs` | `dict` | | Additional keyword arguments passed to environment generation. |

```python
Simulation.run(self)
```
Executes the simulation loop either headless or through the passive interactive viewer depending on `experiment_options.runtime`.

---

## Class Reference: ExperimentTask

Inherits from `dm_control.rl.control.Task`. This is the core orchestrator of the physics loop, responsible for routing data between the Cython arrays and MuJoCo's C-structs.

```python
ExperimentTask.__init__(self, base_links, n_iterations, timestep, **kwargs)
```

| Name | Type | Default | Description |
|---|---|---|---|
| `base_links` | `list[str]` | Required | Names of base link bodies. |
| `n_iterations` | `int` | Required | Total number of simulation steps. |
| `timestep` | `float` | Required | Simulation timestep in seconds. |
| `**kwargs` | `dict` | | Must contain `data` (ExperimentData) and `experiment_options`. |

```python
ExperimentTask.extract_extensions(experiment_options, experiment_data)
```
Static method. Extracts and initializes simulation and animat extensions (`TaskExtension` and `AnimatExtension` instances) directly from the configuration YAML.

```python
ExperimentTask.initialize_episode(self, physics, viewer=None)
```
Called at the start of an episode. Resets physics to keyframe 0, retrieves body masses, builds `maps` linking MuJoCo arrays to Cython arrays, and triggers `initialize_episode` for all extensions.

```python
ExperimentTask.update_sensors(self, physics, links_only=False)
```
Copies the current physics state (`qpos`, `qvel`, `cfrc`, energy, contacts) into the FARMS `AnimatData` arrays.

```python
ExperimentTask.before_step(self, action, physics)
```
Executes control logic before the physics engine calculates the next state. Orchestration order is strict: updates sensors, triggers `extension.before_step()`, and finally injects torques, positions, and velocities into the physics engine.

```python
ExperimentTask.after_step(self, physics)
```
Executes post-physics cleanup. Increments the iteration counter and calls `extension.after_step()` for all registered extensions.

!!! warning "Extension Ordering Fragility"
    Extension execution order is determined by their sequence in the YAML configuration. Controllers must be listed before the `SwimmingExtension` to ensure hydrodynamic forces are calculated using the current timestep's target joint states.

---

## Visual Extensions

The following `TaskExtension` subclasses provide simulation utilities and rendering capabilities.

### MjcfSaver
Saves the generated MJCF XML model to a file during initialization.

```python
MjcfSaver.__init__(self, path)
```
| Name | Type | Default | Description |
|---|---|---|---|
| `path` | `str` | Required | Path to save the MJCF XML string. |

### CameraFollower
Automatically controls the camera azimuth, elevation, and target to track a specific animat.

| Name | Type | Default | Description |
|---|---|---|---|
| `animat_id` | `int` | `0` | ID of the animat to track. |
| `azimuth` | `float` | `0` | Initial camera azimuth. |
| `distance` | `float` | `1` | Camera distance in meters. |
| `elevation` | `float` | `0` | Camera elevation. |
| `angular_velocity` | `float` | `0` | Continuous camera rotation speed (deg/s). |
| `units` | `SimulationUnitScaling` | `SimulationUnits()` | System unit scaling. |

### CoMViewer
Renders a translucent sphere at the exact Center of Mass (CoM) of the model.

| Name | Type | Default | Description |
|---|---|---|---|
| `animat_id` | `int` | `0` | Target animat ID. |
| `size` | `list[float]` | `[0.01, 0.0, 0.0]` | Radius of the sphere. |
| `rgba` | `list[float]` | `[1.0, 1.0, 1.0, 0.3]` | Sphere color and alpha. |

### TrailCoMViewer
Draws a continuous line trail following the Center of Mass.

| Name | Type | Default | Description |
|---|---|---|---|
| `animat_id` | `int` | `0` | Target animat ID. |
| `width` | `float` | `5` | Line width. |
| `rgba` | `list[float]` | `[1.0, 0.3, 0.0, 0.7]` | Line color and alpha. |
| `spacing` | `int` | `10` | Iteration interval to drop a new trail point. |

### TrailLinkViewer
Draws a continuous line trail tracking a specific named link body.

| Name | Type | Default | Description |
|---|---|---|---|
| `link_name` | `str` | `''` | Name of the specific link body to track. |
| `animat_id` | `int` | `0` | Target animat ID. |
| `width` | `float` | `5` | Line width. |
| `rgba` | `list[float]` | `[1.0, 0.3, 0.0, 0.7]` | Line color and alpha. |
| `spacing` | `int` | `10` | Iteration interval to drop a new trail point. |

### ArrowViewer
Renders a rotating arrow to visualize orientation and position, optionally scaling dynamically with mass.

| Name | Type | Default | Description |
|---|---|---|---|
| `animat_id` | `int` | `0` | Target animat ID. |
| `size` | `list[float]` | `[0.03, 0.03, 0.3]` | Base, body, and head sizes. |
| `rgba` | `list[float]` | `[1.0, 1.0, 1.0, 0.3]` | Arrow color and alpha. |
| `offset` | `float` | `None` | Vertical offset to prevent overlapping geometries. |

---

## See Also
- [farms_core.model.control](farms_core_control.md)
- [farms_mujoco.swimming](farms_mujoco_swimming.md)
- [architecture](../architecture.md)
