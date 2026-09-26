# Use Built-in Extensions

The extensions provided by FARMS, where to list them and their options.
Each entry of an `extensions:` list has a `loader` (the dotted path of the
class) and a `config` (its options).

| File | Extensions |
|------|------------|
| `simulation_config.yaml` | Loggers, `MjcfSaver`, cameras, viewer markers (all `TaskExtension`) |
| `animat_config.yaml` | Controllers, `SwimmingExtension` (`AnimatExtension`) |

The camera and marker extensions follow one animat (`animat_id`) but are
simulation extensions: they read its data from `task.data.animats[animat_id]`.

## Saving data

### ExperimentLogger

Writes the experiment data to `<log_path>/simulation.hdf5` at the end of
the simulation.

```yaml
- loader: farms_core.simulation.extensions.ExperimentLogger
  config:
    log_path: Output
    skip: 1
```

| Key | Description |
|-----|-------------|
| `log_path` | Output folder |
| `skip` | Stored but not used: all the iterations in the buffers are saved |

### ExperimentOptionsLogger

Writes the options as loaded (`simulation_options.yaml`,
`animat_<i>_options.yaml`, `arena_<i>_options.yaml`) to `log_path` at the
start of the episode.

```yaml
- loader: farms_core.simulation.extensions.ExperimentOptionsLogger
  config:
    log_path: Output
```

### MjcfSaver

Writes the compiled MuJoCo model (MJCF XML) at the start of the episode.

```yaml
- loader: farms_mujoco.simulation.extensions.MjcfSaver
  config:
    path: Output/simulation_mjcf.xml   # File path (default simulation_mjcf.xml)
```

## Cameras

### CameraFollower: the viewer camera

Makes the camera of the interactive viewer follow an animat: at each
iteration it moves the look-at point towards the centre of mass of the
animat (low-pass filtered) and turns the azimuth at `angular_velocity`.
It has no effect headless or on recorded videos.

```yaml
- loader: farms_mujoco.simulation.extensions.CameraFollower
  config:
    animat_id: 0
    distance: 2.0          # [m]
    azimuth: 90            # [deg]
    elevation: -30         # [deg]
    angular_velocity: 0.0  # [deg/s], non-zero for an orbiting camera
```

Defaults: `animat_id: 0`, `distance: 1`, `azimuth: 0`, `elevation: 0`,
`angular_velocity: 0`.

### CameraRecording: video files

Renders offscreen with its own camera (`mujoco.Renderer`), with or
without the viewer, and writes a video. It is defined in
`farms_mujoco/sensors/camera.py`.

```yaml
- loader: farms_mujoco.sensors.camera.CameraRecording
  config:
    path: Output/video.mp4    # .mp4 or .html, the extension selects the writer
    resolution: [1280, 720]
    fps: 30
    speed: 1.0                # Playback speed
    animat_id: 0              # Tracked animat, null for a fixed look-at point
    offset: [0, 0, 0]         # Added to the look-at point [m]
    distance: 2
    azimuth: 0
    elevation: -15
    angular_velocity: 0       # [deg/s]
    geomgroups: [0, 1, 0, 1, 0, 0]  # Rendered geom groups (1: visuals, 2: collisions)
```

| Key | Default | Description |
|-----|---------|-------------|
| `path` | required | Output file. `.mp4` uses OpenCV when installed (H.264 codecs, then `mp4v`), otherwise Matplotlib with ffmpeg; `.html` uses Matplotlib |
| `resolution` | `[1280, 720]` | `[width, height]` |
| `fps` | `30` | Frame rate. A frame is captured every `speed/(timestep*fps)` iterations |
| `speed` | `1.0` | Playback speed factor |
| `animat_id` | `0` | Animat whose centre of mass the camera follows, `null` for a fixed camera |
| `offset` | `[0, 0, 0]` | Look-at point offset (or the fixed look-at point) |
| `distance`, `azimuth`, `elevation` | `2`, `0`, `-15` | Camera pose |
| `angular_velocity` | `0` | Orbit rate [deg/s] |
| `geomgroups` | `[0, 1, 0, 1, 0, 0]` | MuJoCo geom groups to render |
| `camera` | `null` | MuJoCo camera id. Only works with `mujoco.viewer: dm_control`: with the default viewer it raises an `AttributeError` on the first frame |

The marker extensions below are drawn in the videos too (their
`show_on_camera` attribute is true by default). Without OpenCV, the whole video is kept in
memory until the end: use short runs or a low resolution.

## Viewer markers

These draw debug geometry (spheres, lines, arrows) that is not part of the
physics: no collision, no mass. It is drawn in the interactive viewer and
in the `CameraRecording` videos.

| Extension (`farms_mujoco.simulation.extensions`) | Draws | Config |
|-----------|-------------|--------|
| `CoMViewer` | A sphere at the centre of mass of the animat | `animat_id`, `size`, `rgba` (required) |
| `TrailCoMViewer` | The trail of the centre of mass, a segment every 10 iterations | `animat_id`, `width` and `rgba` (required) |
| `TrailLinkViewer` | The trail of a link, which must be in `control.sensors.links` | `animat_id`, `link`, `width` and `rgba` (required) |
| `ArrowViewer` | A rotating arrow above the animat | `animat_id`, `size`, `rgba` (required), `offset` |

```yaml
- loader: farms_mujoco.simulation.extensions.TrailCoMViewer
  config:
    animat_id: 0
    width: 5
    rgba: [1.0, 0.3, 0.0, 0.7]
```

!!! note "Options not read from YAML"
    The trail viewers' `from_options()` does not apply `width` (it is
    passed as `size`) nor `spacing` (fixed to 10), and no marker reads
    `show_on_camera` from YAML (it defaults to true). Set them in Python
    when creating the extension.

!!! warning "Scene capacity"
    MuJoCo scenes hold a fixed number of geoms. A trail adds one segment
    every 10 iterations and never removes them, which can exceed the
    capacity on long runs.

`farms_mujoco.simulation.extensions.SnakeGame` (a food collecting game)
and `farms_mujoco.viewer.Targets2Reach` (target markers) are further
examples of viewer extensions.

## Fluid forces

### SwimmingExtension

Computes the fluid forces (buoyancy, drag, added mass) of the links with
`fluid_interaction: true` and applies them through `xfrc_applied`, at
every environment step. It reads the `water` block of the first arena;
its `config` is ignored.

```yaml
# animat_config.yaml
extensions:
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config: {}
```

The options and models are described in
[farms_mujoco.swimming](../reference/mujoco/mujoco-swimming.md).

## A typical configuration

```yaml
# simulation_config.yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: Output
      skip: 1
  - loader: farms_core.simulation.extensions.ExperimentOptionsLogger
    config:
      log_path: Output
  - loader: farms_mujoco.simulation.extensions.MjcfSaver
    config:
      path: Output/simulation_mjcf.xml
  - loader: farms_mujoco.simulation.extensions.CameraFollower
    config:
      animat_id: 0
      distance: 2.0
      azimuth: 90
      elevation: -30
      angular_velocity: 0.0
```

```yaml
# animat_config.yaml
extensions:
  - loader: farms_amphibious.control.amphibious.AmphibiousController
    config: {}
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config: {}
```

## Adding objects to the scene

There is no API to add physical objects at runtime: they are part of the
MuJoCo model, built once from:

- the arena SDF file (`sdf` of the arena file): static geometry,
  obstacles, terrain;
- the water SDF file (`water.sdf`), a visual without collisions;
- the animats: an extra movable object can be added as another animat,
  with its own spawn options.

For a visual marker only, use the marker extensions, or write a
`TaskExtension` that calls `create_sphere()`, `create_line()`,
`create_arrow()` or `create_cylinder()` from
`farms_mujoco.simulation.extensions`.

## Extension ordering

Extensions run in the order of the files and lists. All of them run before
the MuJoCo step: the order only matters when one extension reads what
another writes in the same step. For example, an extension reading the
`xfrc` sensors (the fluid forces) must be listed after `SwimmingExtension`.

## See also

- [Write an AnimatExtension](write-extension.md)
- [Extension and Controller Design](../explanation/extension-design.md)
- [Configure an Experiment YAML](configure-yaml.md)
