# Configure an Experiment YAML

This guide explains how to configure a FARMS simulation using YAML files.

## YAML file hierarchy

Every FARMS simulation is driven by a set of YAML files:

```
experiment_config.yaml       # Top-level: links to all sub-configs
├── simulation_config.yaml   # Physics, timestep, duration, sim extensions
├── animat_config.yaml       # Robot: SDF model, morphology, control, extensions
└── arena_config.yaml         # Environment: SDF, water, ground
```

`experiment_config.yaml` keeps `simulation` / `animats` / `arenas` as plain
filename strings (relative to the config file, or absolute), and names the
`Options` subclass that parses each one in a separate `loaders:` block. This
indirection is handled by `ExperimentOptions.load()`
(`farms_core/experiment/options.py`), **not** by a generic dotted-path
loader baked into every options class:

```yaml
simulation: simulation_config.yaml
animats:
  - animat_config.yaml
arenas:
  - arena_config.yaml
loaders:
  simulation_options: farms_core.simulation.options.SimulationOptions
  animats_options:
    - farms_amphibious.model.options.AmphibiousOptions
  arenas_options:
    - farms_amphibious.model.options.AmphibiousArenaOptions
  experiment_data: farms_core.experiment.data.ExperimentData
  animats_data:
    - farms_core.model.data.AnimatData
```

`animats` and `arenas` are lists because an experiment can spawn multiple
animats/arenas. `loaders.animats_options`/`loaders.arenas_options` must
have exactly as many entries, matched by index.

!!! warning "This is a different mechanism from extension `loader:`/`config:` pairs"
    Individual entries in an `extensions:` list (see below) use an inline
    `{loader, config}` pair instead. That is `ExtensionOptions`, resolved by
    whatever creates the extensions (e.g. `ExperimentTask`), not by
    `ExperimentOptions.load()`. See
    [Options and YAML Design](../explanation/options-yaml-design.md) for
    both mechanisms explained together.

## Configuration reference

Every option of the four files, with its type and description, is listed
in the [Configuration Parameter Reference](../reference/env/configuration-reference.md),
which is generated from the code at each documentation build. The main
blocks:

| File | Parsed by | Main keys |
|------|-----------|-----------|
| `experiment_config.yaml` | `ExperimentOptions` | `simulation`, `animats`, `arenas`, `loaders` |
| `simulation_config.yaml` | `SimulationOptions` | `units`, `runtime` (`n_iterations`, `buffer_size`, `headless`, `fast`, `rtl`, ...), `physics` (`timestep`, `gravity`, `cb_sub_steps`, `num_sub_steps`, ...), `mujoco`, `pybullet`, `extensions` |
| `animat_config.yaml` | `AnimatOptions` or `AmphibiousOptions` | `sdf`, `spawn`, `morphology`, `control`, `extensions`, and for `AmphibiousOptions` `show_xfrc`, `scale_xfrc`, `mujoco`, `control.network`, `control.muscles` |
| `arena_config.yaml` | `ArenaOptions` or `AmphibiousArenaOptions` | `sdf`, `spawn`, `water`, `ground_height` |

The fluid model options of the `water` block (`cob_method`,
`fluid_model`, `added_mass`, ...) are described in
[farms_mujoco.swimming](../reference/mujoco/mujoco-swimming.md).

!!! note "Timing"
    An iteration lasts `physics.timestep` and is split into
    `physics.cb_sub_steps` environment steps, each of
    `physics.num_sub_steps` MuJoCo steps. Sensors are logged once per
    iteration, for `runtime.n_iterations` iterations.

## Spawn configuration

The `spawn` block places an animat or an arena in the world:

```yaml
spawn:
  loader: 0
  mode: free                        # See SpawnMode below
  pose: [0, 0, 0.01, 0, -1.5708, 3.1416]  # x, y, z [m], roll, pitch, yaw [rad]
  velocity: [0, 0, 0, 0, 0, 0]      # Linear [m/s] and angular [rad/s]
```

`SpawnMode` (`farms_core.model.options.SpawnMode`) constrains the base
link:

| Mode | Description |
|------|-------------|
| `free` | Free-floating, no constraint |
| `fixed` | Fixed base |
| `rotx`, `roty`, `rotz` | Only rotation about one axis |
| `sagittal`, `coronal`, `transverse` | Motion in one anatomical plane (variants with suffixes `0` and `3` exist) |

## Adding extensions

Extensions are configured in both `simulation_config.yaml` and
`animat_config.yaml`: but **which file** an extension goes in depends on
whether it's a `TaskExtension` (simulation-level) or an `AnimatExtension`
(animat-level), not on what it conceptually "does". Camera and viewer
extensions all subclass `TaskExtension` directly and belong in
`simulation_config.yaml`, even though they visually track one animat by
`animat_id`: they reach for `task.data.animats[animat_id]` themselves
rather than receiving it automatically the way an `AnimatExtension` does.

```yaml
# In simulation_config.yaml (sim-level extensions, includes ALL camera/viewer extensions)
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: Output          # Folder of simulation.hdf5
      skip: 1
  - loader: farms_mujoco.simulation.extensions.MjcfSaver
    config:
      path: Output/simulation_mjcf.xml
  - loader: farms_mujoco.simulation.extensions.CameraFollower
    config:
      animat_id: 0
      azimuth: 90
      distance: 2.0
      elevation: -30
      angular_velocity: 0.0   # deg/s; non-zero for a continuously orbiting live-viewer camera
  - loader: farms_mujoco.sensors.camera.CameraRecording
    config:
      path: Output/video.mp4      # The extension (.mp4 or .html) selects the writer
      resolution: [1280, 720]
      fps: 30
      speed: 1.0
      animat_id: 0
      offset: [0, 0, 0]
      distance: 2
      azimuth: -30
      elevation: -15
      angular_velocity: 0
```

```yaml
# In animat_config.yaml (animat-level extensions, controller + physics only)
extensions:
  - loader: farms_amphibious.control.amphibious.AmphibiousController
    config: {}
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null
```

Each extension entry has:

| Key | Type | Required | Description |
|-----|------|----------|-------------|
| `loader` | str | Yes | Dotted Python path to the extension class |
| `config` | dict | No | Configuration passed to `from_options()` |

`CameraFollower` moves the **live interactive viewer's** camera only (no
effect headless, no effect on exported video). `CameraRecording` is a fully
independent offscreen renderer that produces an actual video file and works
identically whether or not a viewer window is open. Use it whenever you
need output you can share, not just a nicer live view. See
[Use Built-in Extensions](use-extensions.md) for the full extension catalog,
every config field, and known gotchas for each one (including a documented
`CameraRecording` bug around the `camera` config key).

!!! note "Controllers"
    A controller runs only if it is listed in the animat's `extensions:`.
    `control.controller_loader` is parsed but not used.

## Multiple animats

To simulate multiple animats, add filenames to `animats` **and** a matching
loader class to `loaders.animats_options` at the same index. The two lists
are matched by position, not by any key inside the animat entry itself:

```yaml
animats:
  - animat_config.yaml
  - second_animat_config.yaml
loaders:
  animats_options:
    - farms_amphibious.model.options.AmphibiousOptions
    - farms_amphibious.model.options.AmphibiousOptions
  # ...
```

`ExperimentOptions.load()` asserts that `len(animats) ==
len(loaders.animats_options)`; a mismatch raises immediately with the
offending config filename in the message. Each animat gets an index
(`animat_i`) starting from 0, used to access its data and options in
`experiment_options.animats[animat_i]`.

## See also

- [YAML Configuration Schema](../reference/env/yaml-schema.md): complete schema reference
- [Configure CPG Network Parameters](configure-cpg-network.md): CPG network YAML
- [Options and YAML Design](../explanation/options-yaml-design.md): how YAML loading works
