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
| `**kwargs` | `dict` | | Must contain `experiment_options` (ExperimentOptions). `data` (ExperimentData) is optional and defaults to `None` (default data is initialized in `initialize_episode` if absent). |

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
Executes control logic before the physics engine calculates the next state. Orchestration order is strict: updates sensors, then iterates over all extensions calling `extension.before_step()`; for each `AnimatController` extension, torques, positions, and velocities are injected into the physics engine (`physics.data.ctrl`, `qpos_spring`, `jnt_stiffness`, `dof_damping`) immediately after that controller's `before_step()` returns, interleaved with the extension loop rather than as a separate final step.

```python
ExperimentTask.after_step(self, physics)
```
Executes post-physics cleanup. Increments the iteration counter and calls `extension.after_step()` for all registered extensions.

!!! warning "Extension Ordering Fragility"
    Extension execution order is determined by their sequence in the YAML configuration. Controllers must be listed before the `SwimmingExtension` to ensure hydrodynamic forces are calculated using the current timestep's target joint states.

---

## Visual Extensions

The following `TaskExtension` subclasses provide simulation utilities and rendering capabilities.

!!! note "Two unrelated camera mechanisms, plus an ephemeral marker mechanism"
    FARMS has three separate ways to put a camera or a visible marker in
    the scene, and they don't interoperate:

    1. **MJCF-embedded cameras** (`add_cameras()` in `mjcf.py`) — real
       `<camera>` elements baked into the compiled model at build time
       (see below), fixed relative poses (several `mode="trackcom"`),
       selectable by MuJoCo `camera_id` for `physics.render(camera_id=...)`.
       Not configured through the extension system at all.
    2. **`CameraFollower`** — moves the *interactive viewer's* live camera
       (`task.viewer.cam`) each `after_step()`. Only affects what a human
       sees in an open window; no effect headless.
    3. **`CameraRecording`** — its own free-floating `mujoco.MjvCamera` +
       offscreen `mujoco.Renderer`, independent of the viewer; the tool for
       a moving camera baked into an exported video (works headless).

    Markers (`CoMViewer`, `TrailCoMViewer`, `TrailLinkViewer`,
    `ArrowViewer`) are yet another mechanism: they write scratch geometry
    into `viewer.user_scn` via `mujoco.mjv_initGeom` — **not** physics
    bodies, no collision/mass, not part of the MJCF, not visible in
    `CameraRecording` renders (which draw from `physics.model`/
    `physics.data`, not the viewer's scratch buffer), and a no-op headless.
    The scratch buffer has a fixed capacity; long-running trail viewers that
    never clear old segments can eventually exceed it.

### MJCF-embedded cameras (`add_cameras`)

`farms_mujoco/simulation/mjcf.py::add_cameras(link, dist, rot,
simulation_options)` attaches four `<camera>` elements directly to a given
MJCF body during model construction — not a runtime extension, no YAML
config. For each animat's base link it adds, in order: three
`mode="trackcom"` cameras (front/top-down-ish, side, and a third side angle)
and one `mode="fixed"` camera at the same pose as the third. Each is named
`camera_{link.name}_{i}` and can be selected by index/name for offscreen
rendering (`physics.render(camera_id=...)`) or in the interactive viewer's
camera-cycle. These are genuine MuJoCo cameras, distinct from the ephemeral
markers below — but see the bug note under `CameraRecording` before trying
to point that extension at one of them by id.

### MjcfSaver
Saves the generated MJCF XML model to a file during initialization.

```python
MjcfSaver.__init__(self, path)
```
| Name | Type | Default | Description |
|---|---|---|---|
| `path` | `str` | Required | Path to save the MJCF XML string. |

!!! note "`from_options` default"
    When instantiated via `from_options`, `path` defaults to `'simulation_mjcf.xml'` if omitted from the config.

### CameraFollower
Mutates `task.viewer.cam` (the **interactive** passive-viewer's camera state)
every `after_step()`: adds `angular_velocity * dt` to `viewer.cam.azimuth`
(continuous orbit) and low-pass-filters `viewer.cam.lookat` toward the
tracked animat's global CoM (`motion_filter = min(1, 10*timestep)`, applied
fresh each call rather than stored). `initialize_episode()` is a no-op
unless `task.viewer` is truthy, so this extension has **no effect** running
headless or during `CameraRecording` offscreen export — it only moves the
camera you'd see in an open interactive window.

| Name | Type | Default | Description |
|---|---|---|---|
| `animat_id` | `int` | `0` | ID of the animat to track. |
| `azimuth` | `float` | `0` | Initial camera azimuth. |
| `distance` | `float` | `1` | Camera distance in meters. |
| `elevation` | `float` | `0` | Camera elevation. |
| `angular_velocity` | `float` | `0` | Continuous camera rotation speed (deg/s). |
| `units` | `SimulationUnitScaling` | `SimulationUnits()` | System unit scaling. |

### CameraRecording

Source: `farms_mujoco/sensors/camera.py` — a **separate mechanism** from
`CameraFollower`, unrelated to `task.viewer`. It owns its own
`mujoco.MjvCamera` (created with `type = mjCAMERA_FREE` when no `camera` id
is given) and an offscreen `mujoco.Renderer(physics.model.ptr, width,
height)`. Every `before_step()`, if `not iteration % (skips+1)`, it: adds
`angular_velocity * elapsed_time` to `camera.azimuth`, recomputes
`camera.lookat` from `offset` plus (if `animat_id` isn't `None`) the tracked
animat's `global_com_position()`, re-renders the scene into a pre-allocated
`(n_frames, height, width, 3)` uint8 buffer via
`renderer.update_scene(...)` + `renderer.render(out=...)`, and — if `cv2`
is available — writes the frame straight to a `cv2.VideoWriter`. Encoding
finishes in `end_episode()`, either releasing the `cv2.VideoWriter` or, if
`cv2` isn't installed, driving a matplotlib `FuncAnimation` writer over the
full in-memory frame buffer.

| Name | Type | Default | Description |
|---|---|---|---|
| `path` | `str` | required | Output path *without* extension — the extension read from the actual `path` config value (`.mp4`/`.html`) selects `ffmpeg`/`cv2` vs `html` as the writer. |
| `resolution` | `[int, int]` | `[640, 480]` (this extension's own default; `CameraRecordingOptions` defaults to `[1280, 720]` — pass explicitly) | Frame `[width, height]`. |
| `fps` | `float` | `30` | Target output framerate. |
| `speed` | `float` | `1.0` | Playback speed factor; changes the extension's internal `timestep` (`timestep/speed`) and thus the derived `skips`/`fps`. |
| `animat_id` | `int \| None` | `0` | Animat whose global CoM the camera tracks; `None` = fixed camera. |
| `offset` | `[float, float, float]` | `[0, 0, 0]` | Added to `camera.lookat` every frame. |
| `distance`, `azimuth`, `elevation` | `float` | `2`, `0`, `-15` | Initial `MjvCamera` pose. |
| `angular_velocity` | `float` | `0` | Orbit rate [deg/s], applied using real elapsed physics time since the last capture. |
| `geomgroups` | `list[int]` | `[1, 1, 0, 1, 0, 0]` | `MjvOption.geomgroup` render mask — independent of what's shown in the interactive viewer. |
| `skips` | `int` | derived from `speed/(timestep*fps) - 1` | Physics-step interval between captured frames; override directly if the derived cadence isn't what you want. |

!!! bug "Confirmed: passing a `camera` id crashes with the default `MuJoCo` viewer"
    `CameraRecordingOptions`/`CameraRecording.__init__` accept an optional
    `camera` kwarg (e.g. to target one of the MJCF-embedded cameras from
    `add_cameras()` by id instead of the default free camera). But
    `initialize_episode()` only builds `self.renderer` and converts
    `self.camera` into a full `mujoco.MjvCamera` inside its `if self.camera
    is None:` branch — supplying a `camera` id skips that branch entirely,
    leaving `self.renderer` as `None`. Then, for `viewer != 'dm_control'`
    (the default `viewer: MuJoCo` used throughout the Zbot experiments),
    `before_step()` unconditionally runs `self.camera.azimuth +=
    self.angular_velocity*timediff` *before* it checks `self.renderer is
    not None` — and an id has no `.azimuth` attribute, so this raises
    `AttributeError` on the first captured frame. Leave `camera` unset
    (default free camera) unless you're also using the `dm_control` viewer,
    where `physics.render(camera_id=self.camera)` is used instead and
    doesn't hit this code path.

!!! note "Works headless — this is the tool for a moving camera in exported video"
    Unlike `CameraFollower`, `CameraRecording` doesn't depend on
    `task.viewer` at all — it renders directly from `physics.model`/
    `physics.data` through its own `mujoco.Renderer`. Use it (not
    `CameraFollower`) whenever the goal is a moving/orbiting camera baked
    into an output video file rather than an interactive session.

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
| `link` | `str` | `''` | Name of the specific link body to track (stored as attribute `link_name`). |
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

- [Controller Base Classes](../core/core-control.md) — How controllers plug into the physics loop
- [Swimming Extension](mujoco-swimming.md) — Hydrodynamic forces
- [Architecture Overview](../../explanation/architecture.md) — Cross-module data flow
