# Use Built-in Extensions

This guide lists the built-in extensions provided by FARMS and their
configuration options.

!!! bug "Corrected — viewer/camera extensions are simulation-level, not animat-level"
    An earlier draft of this page listed `MjcfSaver`, `CameraFollower`, and
    the marker/trail viewers under "Animat-level extensions", registered in
    `animat_config.yaml`. That's wrong: all of these subclass `TaskExtension`
    directly (`farms_mujoco/simulation/extensions.py`) — not `AnimatExtension`
    — and their `from_options(cls, config, experiment_options)` takes the
    2-argument `TaskExtension` signature, not the 5-argument
    `AnimatExtension` one (`animat_i`, `animat_data`, `animat_options`).
    They are registered in **`simulation_config.yaml`**'s top-level
    `extensions:` list, alongside `ExperimentLogger` — confirmed against
    `experiments/zbot_swimming/simulation_config.yaml`, which lists
    `ExperimentLogger`, `ExperimentOptionsLogger`, `MjcfSaver`, and
    `CameraFollower` together in that one list. Because they're
    `TaskExtension`s and don't get `animat_data` injected automatically,
    each one that needs animat data reaches for it manually via
    `task.data.animats[self.animat_id].sensors.links` in
    `initialize_episode()` — that's why every one of them takes an
    `animat_id` config key even though they live in the simulation-level
    list, not the animat-level one.

## Simulation-level extensions

These are registered in `simulation_config.yaml` under `extensions:` and
extend `TaskExtension` (`farms_core/simulation/extensions.py`).

### ExperimentLogger

Saves all simulation data to HDF5 at episode end.

```yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: ./simulation.hdf5
      skip: 1
```

| Config key | Type | Default | Description |
|------------|------|---------|-------------|
| `log_path` | str | — | Output HDF5 file path |
| `skip` | int | `1` | Save every N iterations (1 = every step) |

### ExperimentOptionsLogger

Saves copies of all YAML configuration files to the output directory.

```yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentOptionsLogger
    config:
      log_path: ./options
```

| Config key | Type | Default | Description |
|------------|------|---------|-------------|
| `log_path` | str | — | Output directory for YAML files |

### MjcfSaver

Saves the compiled MuJoCo model as MJCF XML, at `initialize_episode()` time
(before physics runs), by calling `mjcf2str(task.mjcf)` on the fully-built
in-memory MJCF tree.

```yaml
extensions:
  - loader: farms_mujoco.simulation.extensions.MjcfSaver
    config:
      path: Output/simulation_mjcf.xml
```

| Config key | Type | Default | Description |
|------------|------|---------|-------------|
| `path` | str | `'simulation_mjcf.xml'` | Full output file path (not a directory — despite the name, pass the target `.xml` file path itself, as in the example above). |

### CameraFollower — moving the *interactive* viewer camera

Makes the **interactive passive-viewer camera** (`task.viewer.cam`) follow an
animat: every `after_step()` it nudges `viewer.cam.azimuth` by
`angular_velocity * dt` (continuous orbiting) and low-pass-filters
`viewer.cam.lookat` toward the tracked animat's global center-of-mass
position.

```yaml
extensions:
  - loader: farms_mujoco.simulation.extensions.CameraFollower
    config:
      animat_id: 0
      azimuth: 90
      distance: 2.0
      elevation: -30
      angular_velocity: 0.0   # deg/s — set non-zero for a continuously orbiting camera
```

| Config key | Type | Default | Description |
|------------|------|---------|-------------|
| `animat_id` | int | `0` | Index into `experiment_options.animats` / `task.data.animats` of the animat to track. |
| `azimuth` | float | `0` | Initial camera azimuth [deg], set once in `initialize_episode()`. |
| `distance` | float | `1` | Camera distance from the look-at point [m], scaled by `units.meters`. |
| `elevation` | float | `0` | Camera elevation [deg]. |
| `angular_velocity` | float | `0` | Continuous azimuth rotation rate [deg/s] — set non-zero for an orbiting camera; `0` keeps azimuth fixed while `lookat` still tracks the animat. |

!!! warning "Only affects the interactive viewer — has no effect headless or in offscreen video export"
    `CameraFollower` mutates `task.viewer.cam`, and `initialize_episode()`
    is a no-op unless `task.viewer` is truthy (`if self.viewer:`). It only
    does anything when `mujoco.viewer` is showing a live, interactive
    window (`runtime.headless: false` / `play: true` in
    `simulation_config.yaml`). It has **no effect** on offscreen video
    export — for a moving camera in an exported video, use
    [`CameraRecording`](#camerarecording-moving-camera-for-offscreen-video-export)
    below instead, which drives its own independent `mujoco.MjvCamera` and
    `mujoco.Renderer`.

### CameraRecording — moving camera for offscreen video export

A **separate, independent** camera/rendering mechanism from `CameraFollower`
above, defined in `farms_mujoco/sensors/camera.py`. Instead of touching the
interactive viewer, it drives its own free-floating `mujoco.MjvCamera`
(`mjCAMERA_FREE` mode) through an offscreen `mujoco.Renderer`, captures a
frame into a pre-allocated `(n_frames, height, width, 3)` uint8 buffer every
`before_step()`, and encodes the buffer to a video file (`.mp4` via
`cv2.VideoWriter`+ffmpeg codec fallback chain, or `.html` via
matplotlib's `FuncAnimation` writers) in `end_episode()`. It works
identically whether or not an interactive window is open, which makes it the
right tool for headless/batch video export.

```yaml
extensions:
  - loader: farms_mujoco.sensors.camera.CameraRecording
    config:
      path: Output/video          # extension (.mp4/.html) is appended automatically
      resolution: [1280, 720]
      fps: 30
      speed: 1.0
      animat_id: 0                # camera tracks this animat's global CoM; null = fixed at `offset`
      offset: [0, 0, 0]
      distance: 2
      azimuth: 0
      elevation: -15
      angular_velocity: 0         # deg/s — orbiting camera around the tracked point
      motion_filter: null         # defaults to 10*timestep if omitted
      geomgroups: [1, 1, 0, 1, 0, 0]
```

| Config key | Type | Default | Description |
|------------|------|---------|-------------|
| `path` | str | required | Output path *without* extension; the file extension (`.mp4`/`.html`, from `os.path.splitext`) is what selects the writer backend. Anything else falls back to `ffmpeg` with a warning. |
| `resolution` | `[int, int]` | `[1280, 720]` in `CameraRecordingOptions`, but `[640, 480]` in the `CameraRecording` extension's own default — pass it explicitly to be sure. | Frame `[width, height]`. |
| `fps` | float | `30` | Target output framerate; actual capture cadence (`skips`) is derived from `fps`, `speed`, and the physics `timestep` so that captured frames land close to real-time playback at `speed`. |
| `speed` | float | `1.0` | Playback speed factor — internally divides the extension's own notion of `timestep` (`timestep/speed`), which changes `skips` and thus `fps`, not just metadata. |
| `animat_id` | int \| `None` | `0` | Animat whose global CoM the camera's `lookat` tracks each frame. `None` = fixed camera at `offset`, not following anything. |
| `offset` | `[float, float, float]` | `[0, 0, 0]` | Added to `lookat` every frame — either the fixed look-at point (`animat_id: null`) or an offset from the tracked animat's CoM. |
| `distance`, `azimuth`, `elevation` | float | `2`, `0`, `-15` | Initial `MjvCamera` distance/azimuth/elevation. |
| `angular_velocity` | float | `0` | Added to `camera.azimuth` every frame, scaled by the elapsed physics time since the last capture — an orbiting shot. |
| `geomgroups` | `list[int]` (6 entries) | `[1, 1, 0, 1, 0, 0]` | `MjvOption.geomgroup` mask controlling which MuJoCo geom groups are rendered into the video (independent of what's visible in the interactive viewer). |
| `skips` | int | derived from `speed/(timestep*fps) - 1` | Number of physics steps to skip between captured frames; you can override it directly instead of relying on the `fps`/`speed` derivation. |

!!! bug "Don't pass a `camera` id (e.g. to target an MJCF-embedded camera) with the default viewer"
    See the full bug writeup in
    [`mujoco-simulation.md`](../reference/mujoco/mujoco-simulation.md#camerarecording) —
    supplying `camera` in config crashes with `AttributeError` on the first
    frame for the default `viewer: MuJoCo` setting. Leave it unset.

!!! note "Requires `cv2` for `.mp4`, falls back to matplotlib otherwise"
    If OpenCV (`cv2`) is importable, `.mp4` output goes through
    `cv2.VideoWriter`, trying H.264 codecs (`avc1`, `H264`, `X264`) before
    falling back to `mp4v`. If `cv2` isn't installed, the whole recording
    pipeline falls back to a matplotlib `FuncAnimation` writer
    (`manimation.writers[...]`), which is slower and produces the frame
    buffer in memory for the entire episode — expensive for long runs at
    high resolution. `.html` output always goes through the matplotlib path.

### Visualization / marker extensions

These render lightweight, **ephemeral** debug geometry (spheres, lines,
arrows) directly into the interactive MuJoCo viewer's scratch scene buffer
(`viewer.user_scn`), via `mujoco.mjv_initGeom`. This is **not physics
geometry** — it has no collision, no mass, isn't part of the MJCF model, and
is not visible in offscreen `CameraRecording` renders (which render from the
actual `physics.model`/`physics.data`, not the viewer's scratch buffer).
Like `CameraFollower`, these all require `task.viewer` to be truthy —
they're no-ops when running headless.

| Extension | Description | Config |
|-----------|-------------|--------|
| `CoMViewer` | Creates one sphere (`create_sphere`) at `initialize_episode()`, then repositions it (`sphere.pos = ...`) every `after_step()` to track the animat's global CoM. Auto-sizes the sphere radius from total link mass if `size` isn't given. | `animat_id`, `size`, `rgba` |
| `TrailCoMViewer` | Every `spacing` iterations, draws a new short `create_line` segment from the previous CoM sample to the current one — accumulates into a visible trail over time (each segment is a separate scratch geom; the buffer is never cleared by this extension itself). | `animat_id`, `width`, `rgba`, `spacing` |
| `TrailLinkViewer` | Same as `TrailCoMViewer`, but tracks one named link's `com_position` instead of the whole-animat CoM. Asserts `link` is a valid name in `animat_data.sensors.links.names` at `initialize_episode()` — this will raise if the link isn't in your `sensors.links` YAML list, so it depends on [sensor configuration](configure-sensors.md). | `animat_id`, `link`, `width`, `rgba`, `spacing` |
| `ArrowViewer` | Creates one `create_arrow` primitive, repositioned above the animat's CoM (`+ [0, 0, offset]`) each step and continuously rotated (`0.2*pi*time` about the local x-axis) — a generic rotating pointer, not bound to any physical torque/force value by default. Auto-sizes from mass like `CoMViewer` if `size` is omitted. | `animat_id`, `size`, `rgba`, `offset` |

!!! warning "Scratch-geom buffer has a fixed capacity"
    `create_primitive()` (in `farms_mujoco/simulation/extensions.py`) writes
    into `viewer.user_scn.geoms[viewer.user_scn.ngeom]` and increments
    `ngeom`. MuJoCo's scratch scene buffer has a fixed maximum size
    (`mujoco.mjMAXOVERLAY`/renderer-dependent limit); a long-running
    `TrailCoMViewer`/`TrailLinkViewer` that never removes old segments will
    eventually exceed it and raise or silently stop drawing. Keep
    `spacing` large enough, or the trail short enough, for the run length
    you need.

## Animat-level extensions

These are registered in `animat_config.yaml` under `extensions:` and extend
`AnimatExtension` (`farms_core/model/extensions.py`) or a subclass —
verified against `experiments/zbot_swimming/animat_config.yaml`, whose
`extensions:` list contains `AmphibiousController` and `SwimmingExtension`
(both `AnimatExtension` subclasses; `AmphibiousController` further extends
it via `AnimatController`).

### SwimmingExtension

Computes hydrodynamic forces (drag + buoyancy) and applies them as
`xfrc_applied` in MuJoCo. This is the core fluid dynamics extension for
swimming robots.

```yaml
extensions:
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null
```

| Config key | Type | Default | Description |
|------------|------|---------|-------------|
| `water_properties` | dict \| `null` | `null` | Optional explicit water-properties override; when `null`/omitted, the extension falls back to reading `water.drag`, `water.buoyancy`, `water.height`, and `water.density` from `experiment_options.arenas[0]` (the first arena's `WaterOptions`). |

**How it works** (verified in `farms_mujoco/swimming/extension.py`):

1. `from_options()` receives arena water options
2. `before_step()` calls `SwimmingHandler.step()` which:
   - Computes drag forces from link velocities and drag coefficients
   - Computes buoyancy forces from link volume and water height
   - Sets forces in `physics.data.xfrc_applied`

## Typical extension configuration

A typical swimming experiment uses this combination — note `MjcfSaver` and
`CameraFollower` live in `simulation_config.yaml`, while `SwimmingExtension`
lives in `animat_config.yaml`:

```yaml
# simulation_config.yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: ./simulation.hdf5
      skip: 1
  - loader: farms_core.simulation.extensions.ExperimentOptionsLogger
    config:
      log_path: ./options
  - loader: farms_mujoco.simulation.extensions.MjcfSaver
    config:
      path: ./Output/simulation_mjcf.xml
  - loader: farms_mujoco.simulation.extensions.CameraFollower
    config:
      animat_id: 0
      azimuth: 90
      distance: 2.0
      elevation: -30
      angular_velocity: 0.0

# animat_config.yaml
extensions:
  - loader: farms_amphibious.control.amphibious.AmphibiousController
    config: {}
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null
```

## Adding objects to the scene

There is no runtime "add object" API — physical (collidable) objects are
part of the compiled MJCF model, built once at `setup_mjcf_xml()` time from:

- The **arena SDF** (`arena_options.sdf`), converted via `sdf2mjcf()` — any
  static geometry, obstacles, or terrain you want in the world belongs here.
- An optional **water body** (`arena_options.water.sdf` + `water.height`),
  added as a second, separately-loaded SDF model with contacts disabled
  (`contype=0, conaffinity=0` — it exists for the `SwimmingExtension`'s
  drag/buoyancy math and for the water visual, not for collision).
- Additional **animats** — every entry in `experiment_options.animats` gets
  its own `sdf2mjcf()` pass and its own contact bitmask
  (`contype=2**(animat_i+1)`), so a second manipulable/collidable body is
  most naturally added as a second animat entry, not as a special "object"
  type.

To add a new static or dynamic object, add it to the arena's SDF file (or
give it its own SDF and register it as an additional `animats` entry if it
needs independent spawn options); there's no equivalent of a Python
`add_object(mesh, pos)` call at simulation-setup time. If you only need a
non-physical visual marker (not a collidable object), use one of the
[marker/trail viewer extensions](#visualization-marker-extensions) above,
or write your own `TaskExtension` calling `create_sphere`/`create_line`/
`create_arrow`/`create_cylinder` from
`farms_mujoco.simulation.extensions`.

## Extension ordering

Extensions execute in YAML declaration order. `SwimmingExtension` should be
listed first among animat extensions so that its xfrc data is available to
other extensions that read `animat_data.sensors.xfrc`.

## See also

- [Write an AnimatExtension](write-extension.md) — write your own
- [Extension API](../reference/core/extension-api.md) — full class reference
- [Configure an Experiment YAML](configure-yaml.md) — where to register extensions
- [Add and Configure Sensors](configure-sensors.md) — required for `TrailLinkViewer`'s `link` lookup
