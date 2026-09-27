# farms_mujoco.simulation

MuJoCo physics backend: MJCF generation, task lifecycle, sensor maps, and visual extensions.

## Overview

The `farms_mujoco.simulation` module bridges the abstract FARMS definitions with the concrete MuJoCo physics engine. It handles translating experiment options into a valid MuJoCo XML, managing the exact execution order of controllers and sensors during a physics step, and providing a suite of visual extensions for debugging and rendering.

---

## Simulation

`Simulation.from_experiment(experiment_options)` builds the MuJoCo model
and the dm_control environment, and `run()` runs the loop, with the
viewer or headless (`runtime.headless`). See [farms_mujoco](farms-mujoco.md#simulation).

::: farms_mujoco.simulation.simulation.Simulation
    options:
      show_root_heading: false
      heading_level: 3
      members: [from_experiment, run, iterator]

---

## ExperimentTask

The dm_control `Task` of FARMS (`farms_mujoco/simulation/task.py`):

- `extract_extensions()` (static) creates the simulation extensions, then
  the extensions of each animat, from their `loader` and `config`.
- `initialize_episode(physics, viewer=None)` resets the physics to
  keyframe 0, builds the maps between MuJoCo and the sensor arrays,
  collects the controllers, and calls `initialize_episode()` on every
  extension.
- `update_sensors(physics, links_only=False)` copies the MuJoCo state into
  the sensor arrays, and the time, number of contacts, solver iterations
  and energy into the simulation data.
- `before_step(action, physics)` updates the sensors, then calls
  `before_step()` on each extension. Right after a controller's
  `before_step()`, its commands are written to MuJoCo: positions,
  velocities and torques to the actuators (`physics.data.ctrl`), spring
  references, stiffnesses and damping to `physics.model.qpos_spring`,
  `jnt_stiffness` and `dof_damping`.
- `after_step(physics)` counts the environment steps and, at the end of
  an iteration, increments `iteration` and calls `after_step()` on every
  extension.

The extensions run in the order of the files: simulation extensions
first, then those of each animat in the order of `extensions:`. All of
them run before the MuJoCo step, from the state at the start of the step.

::: farms_mujoco.simulation.task.ExperimentTask
    options:
      show_root_heading: false
      heading_level: 3
      members: [extract_extensions, initialize_episode, update_sensors, before_step, after_step]

---

## Visual Extensions

The following `TaskExtension` subclasses provide simulation utilities and rendering capabilities.

!!! note "Two unrelated camera mechanisms, plus an ephemeral marker mechanism"
    FARMS has three separate ways to put a camera or a visible marker in
    the scene, and they don't interoperate:

    1. **MJCF-embedded cameras** (`add_cameras()` in `mjcf.py`): real
       `<camera>` elements baked into the compiled model at build time
       (see below), fixed relative poses (several `mode="trackcom"`),
       selectable by MuJoCo `camera_id` for `physics.render(camera_id=...)`.
       Not configured through the extension system at all.
    2. **`CameraFollower`**: moves the *interactive viewer's* live camera
       (`task.viewer.cam`) each `after_step()`. Only affects what a human
       sees in an open window; no effect headless.
    3. **`CameraRecording`**: its own free-floating `mujoco.MjvCamera` +
       offscreen `mujoco.Renderer`, independent of the viewer; the tool for
       a moving camera baked into an exported video (works headless).

    Markers (`CoMViewer`, `TrailCoMViewer`, `TrailLinkViewer`,
    `ArrowViewer`) are yet another mechanism: they draw scratch geometry
    (`mujoco.mjv_initGeom`), not physics bodies: no collision or mass, not
    part of the MJCF. They are drawn in the viewer's `user_scn`, and also
    in the `CameraRecording` videos, which call their `render_scene()` on
    the recording scene (unless the extension has `show_on_camera: false`).
    The scenes have a fixed capacity, so long trails can exceed it.

### MJCF-embedded cameras (`add_cameras`)

`farms_mujoco/simulation/mjcf.py::add_cameras(link, dist, rot,
simulation_options)` attaches four `<camera>` elements directly to a given
MJCF body during model construction. It is not a runtime extension and has no YAML
config. For each animat's base link it adds, in order: three
`mode="trackcom"` cameras (front/top-down-ish, side, and a third side angle)
and one `mode="fixed"` camera at the same pose as the third. Each is named
`camera_{link.name}_{i}` and can be selected by index/name for offscreen
rendering (`physics.render(camera_id=...)`) or in the interactive viewer's
camera-cycle. These are genuine MuJoCo cameras, distinct from the ephemeral
markers below, but see the bug note under `CameraRecording` before trying
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
headless or during `CameraRecording` offscreen export. It only moves the
camera you'd see in an open interactive window.

| Name | Type | Default | Description |
|---|---|---|---|
| `animat_id` | `int` | `0` | ID of the animat to track. |
| `azimuth` | `float` | `0` | Initial camera azimuth. |
| `distance` | `float` | `1` | Camera distance in meters. |
| `elevation` | `float` | `0` | Camera elevation. |
| `angular_velocity` | `float` | `0` | Continuous camera rotation speed (deg/s). |

### CameraRecording

Source: `farms_mujoco/sensors/camera.py`. This is a **separate mechanism** from
`CameraFollower`, unrelated to `task.viewer`. It owns its own
`mujoco.MjvCamera` (created with `type = mjCAMERA_FREE` when no `camera` id
is given) and an offscreen `mujoco.Renderer(physics.model.ptr, width,
height)`. Every `before_step()`, if `not iteration % (skips+1)`, it: adds
`angular_velocity * elapsed_time` to `camera.azimuth`, recomputes
`camera.lookat` from `offset` plus (if `animat_id` isn't `None`) the tracked
animat's `global_com_position()`, re-renders the scene into a pre-allocated
`(n_frames, height, width, 3)` uint8 buffer via
`renderer.update_scene(...)` + `renderer.render(out=...)`, and, if `cv2`
is available, writes the frame straight to a `cv2.VideoWriter`. Encoding
finishes in `end_episode()`, either releasing the `cv2.VideoWriter` or, if
`cv2` isn't installed, driving a matplotlib `FuncAnimation` writer over the
full in-memory frame buffer.

Defaults below are those of `CameraRecordingOptions`, used when the
extension is created from YAML. A frame is captured every
`skips + 1 = speed/(timestep*fps)` iterations.

| Name | Type | Default | Description |
|---|---|---|---|
| `path` | `str` | required | Output file. Its extension selects the writer: `.mp4` (OpenCV when installed, else Matplotlib with ffmpeg) or `.html`. |
| `resolution` | `[int, int]` | `[1280, 720]` | Frame `[width, height]`. |
| `fps` | `float` | `30` | Target output framerate. |
| `speed` | `float` | `1.0` | Playback speed factor; changes the extension's internal `timestep` (`timestep/speed`) and thus the derived `skips`/`fps`. |
| `animat_id` | `int \| None` | `0` | Animat whose global CoM the camera tracks; `None` = fixed camera. |
| `offset` | `[float, float, float]` | `[0, 0, 0]` | Added to `camera.lookat` every frame. |
| `distance`, `azimuth`, `elevation` | `float` | `2`, `0`, `-15` | Initial `MjvCamera` pose. |
| `angular_velocity` | `float` | `0` | Orbit rate [deg/s], applied using real elapsed physics time since the last capture. |
| `geomgroups` | `list[int]` | `[0, 1, 0, 1, 0, 0]` | `MjvOption.geomgroup` render mask (group 1: visuals, group 2: collisions), independent of the interactive viewer. |
| `camera` | `int \| None` | `None` | MuJoCo camera id, see the bug below. |

!!! bug "Confirmed: passing a `camera` id crashes with the default `MuJoCo` viewer"
    `CameraRecordingOptions`/`CameraRecording.__init__` accept an optional
    `camera` kwarg (e.g. to target one of the MJCF-embedded cameras from
    `add_cameras()` by id instead of the default free camera). But
    `initialize_episode()` only builds `self.renderer` and converts
    `self.camera` into a full `mujoco.MjvCamera` inside its `if self.camera
    is None:` branch, so supplying a `camera` id skips that branch entirely,
    leaving `self.renderer` as `None`. Then, for `viewer != 'dm_control'`
    (the default `viewer: MuJoCo` used throughout the Zbot experiments),
    `before_step()` unconditionally runs `self.camera.azimuth +=
    self.angular_velocity*timediff` *before* it checks `self.renderer is
    not None`, and an id has no `.azimuth` attribute, so this raises
    `AttributeError` on the first captured frame. Leave `camera` unset
    (default free camera) unless you're also using the `dm_control` viewer,
    where `physics.render(camera_id=self.camera)` is used instead and
    doesn't hit this code path.

!!! note "Works headless: use this for a moving camera in exported video"
    Unlike `CameraFollower`, `CameraRecording` doesn't depend on
    `task.viewer` at all. It renders directly from `physics.model`/
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

- [Controller Base Classes](../core/core-control.md): How controllers plug into the physics loop
- [Swimming Extension](mujoco-swimming.md): Hydrodynamic forces
- [Architecture Overview](../../explanation/architecture.md): Cross-module data flow
