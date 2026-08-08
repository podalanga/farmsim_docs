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
animats/arenas — `loaders.animats_options`/`loaders.arenas_options` must
have exactly as many entries, matched by index.

!!! warning "This is a different mechanism from extension `loader:`/`config:` pairs"
    Individual entries in an `extensions:` list (see below) use an inline
    `{loader, config}` pair instead — that's `ExtensionOptions`, resolved by
    whatever creates the extensions (e.g. `ExperimentTask`), not by
    `ExperimentOptions.load()`. See
    [Options and YAML Design](../explanation/options-yaml-design.md) for
    both mechanisms explained together.

## Configuration reference

### experiment_config.yaml

| Key | Type | Required | Parsed by | Notes |
|-----|------|----------|-----------|-------|
| `simulation` | str | Yes | `ExperimentOptions.load` | Path to `simulation_config.yaml` |
| `animats` | list[str] | Yes | `ExperimentOptions.load` | Paths to animat config files |
| `arenas` | list[str] | Yes | `ExperimentOptions.load` | Paths to arena config files |
| `loaders` | dict | Yes | `ExperimentLoadOptions` | Dotted-path classes for the above, plus `experiment_data`/`animats_data` |

### simulation_config.yaml

Parsed by `SimulationOptions` (`farms_core/simulation/options.py`):

| Key | Type | Required | Default | Parsed by | Notes |
|-----|------|----------|---------|-----------|-------|
| `units` | dict | No | `meters=seconds=kilograms=1` | `SimulationUnitScaling` | Unit scaling factors; a flat `meters`/`seconds`/`kilograms` at the top level of `simulation_config.yaml` also works |
| `runtime` | dict | No | see below | `RuntimeSimulationOptions` | Iteration count, playback speed |
| `runtime.n_iterations` | int | No | `1000` | `RuntimeSimulationOptions` | Number of simulation steps to run |
| `runtime.buffer_size` | int | No | `n_iterations` | `RuntimeSimulationOptions` | Ring-buffer size for logged data; wraps if smaller than `n_iterations` |
| `runtime.play` | bool | No | `True` | `RuntimeSimulationOptions` | Start playing immediately (interactive mode) |
| `runtime.rtl` | float | No | `1.0` | `RuntimeSimulationOptions` | Real-time limiter factor (interactive mode) |
| `runtime.fast` | bool | No | `False` | `RuntimeSimulationOptions` | Bypass the real-time limiter |
| `runtime.headless` | bool | No | `False` | `RuntimeSimulationOptions` | Run without a viewer |
| `runtime.show_progress` | bool | No | `True` | `RuntimeSimulationOptions` | Display a progress bar |
| `physics` | dict | No | see below | `PhysicsSimulationOptions` | Timestep, gravity, solver iterations |
| `physics.timestep` | float | No | `1e-3` | `PhysicsSimulationOptions` | Physics timestep [s] |
| `physics.gravity` | list[float] | No | `[0, 0, -9.81]` | `PhysicsSimulationOptions` | 3-vector |
| `physics.num_sub_steps` | int | No | `1` | `PhysicsSimulationOptions` | Physics-engine-level substeps |
| `physics.cb_sub_steps` | int | No | `0` | `PhysicsSimulationOptions` | FARMS callback substeps |
| `physics.n_solver_iters` | int | No | `50` | `PhysicsSimulationOptions` | Max solver iterations per step |
| `mujoco` | dict | No | `{}` | `MuJoCoSimulationOptions` | MuJoCo-specific settings (`integrator`, `solver`, `cone`, viewer options, ...) |
| `pybullet` | dict | No | `{}` | `PybulletSimulationOptions` | PyBullet-specific settings |
| `extensions` | list[dict] | No | `[]` | `SimulationOptions.__init__` | Sim-level extensions |

Each extension entry:

```yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: ./simulation.hdf5
      skip: 1
```

### animat_config.yaml

Parsed by `AnimatOptions` (`farms_core/model/options.py`) or a subclass like
`AmphibiousOptions` (`farms_amphibious/model/options.py`):

| Key | Type | Required | Default | Parsed by | Notes |
|-----|------|----------|---------|-----------|-------|
| `sdf` | str | Yes | — | `ModelOptions.__init__` | Path to SDF model file |
| `spawn` | dict | Yes | — | `SpawnOptions` | Position, orientation, mode |
| `morphology` | dict | Yes | — | `MorphologyOptions` | Links, joints, collisions |
| `morphology.links` | list[dict] | Yes | — | `LinkOptions` | Per-link properties |
| `morphology.joints` | list[dict] | Yes | — | `JointOptions` | Per-joint properties |
| `morphology.self_collisions` | list[list[str]] | Yes | — | `MorphologyOptions` | Collision pairs |
| `control` | dict | Yes | — | `ControlOptions` | Controller, sensors, motors |
| `control.controller_loader` | str | No | `None` | `ControlOptions.__init__` | Dotted path to controller class |
| `control.sensors` | dict | Yes | — | `SensorsOptions` | Sensor name lists |
| `control.motors` | list[dict] | Yes | — | `MotorOptions` | Per-joint motor config |
| `extensions` | list[dict] | No | `[]` | `AnimatOptions.__init__` | Animat-level extensions |

For `AmphibiousOptions`, additional keys:

| Key | Type | Required | Default | Parsed by | Notes |
|-----|------|----------|---------|-----------|-------|
| `show_xfrc` | bool | No | `False` | `AmphibiousOptions.__init__` | Visualize external forces |
| `scale_xfrc` | int | No | `1` | `AmphibiousOptions.__init__` | Force visualization scale |
| `mujoco` | dict | No | `{}` | `AmphibiousOptions.__init__` | MuJoCo-specific options |
| `control.network` | dict | No | `None` | `AmphibiousNetworkOptions` | CPG network config (see [CPG guide](configure-cpg-network.md)) |

!!! warning "sdf vs sdf_path"
    The `AnimatOptions.__init__` constructor takes `sdf` as the key. However,
    `AmphibiousOptions.from_options()` reads `sdf_path` from kwargs and maps
    it to `sdf`. In YAML files loaded via the standard `Options.load()` path,
    the key is `sdf`. The `sdf_path` key is used only in the `from_options()`
    shorthand path.

### arena_config.yaml

Parsed by `ArenaOptions` (`farms_core/model/options.py`):

| Key | Type | Required | Default | Parsed by | Notes |
|-----|------|----------|---------|-----------|-------|
| `sdf` | str | Yes | — | `ArenaOptions.__init__` | Path to arena SDF file |
| `spawn` | dict | Yes | — | `SpawnOptions` | Arena spawn position |
| `water` | dict | No | — | `WaterOptions` | Fluid properties |
| `ground_height` | float | No | `0.0` | `ArenaOptions.__init__` | Ground plane height |

#### Water options

| Key | Type | Required | Default | Parsed by | Notes |
|-----|------|----------|---------|-----------|-------|
| `sdf` | str | No | `''` | `WaterOptions.__init__` | Water visual SDF |
| `drag` | list[float] | Yes | — | `WaterOptions.__init__` | Drag coefficients [6 values] |
| `buoyancy` | bool | Yes | — | `WaterOptions.__init__` | Enable buoyancy |
| `height` | float | Yes | — | `WaterOptions.__init__` | Water surface height |
| `velocity` | list[float] | No | `[0,0,0,0,0,0]` | `WaterOptions.__init__` | Fluid velocity [6 values] |
| `viscosity` | float | No | `0.0` | `WaterOptions.__init__` | Fluid viscosity |
| `density` | float | Yes | — | `WaterOptions.__init__` | Fluid density [kg/m³] |
| `maps` | dict | No | `{}` | `WaterOptions.__init__` | Per-link fluid maps |

## Spawn configuration

The `spawn` dict controls how an animat or arena is placed in the world:

```yaml
spawn:
  position: [0.0, 0.0, 0.1]
  orientation: [0.0, 0.0, 0.0]  # RPY or quaternion
  mode: sagittal  # free | fixed | sagittal | coronal | transverse | ...
```

`SpawnMode` values (verified in `farms_core/model/options.py`):

| Mode | Description |
|------|-------------|
| `free` | Free-floating, no constraints |
| `fixed` | Fixed base |
| `rotx` / `roty` / `rotz` | Constrained to rotation about one axis |
| `sagittal` | Sagittal plane (2D swimming) |
| `coronal` | Coronal plane |
| `transverse` | Transverse plane |

## Adding extensions

Extensions are configured in both `simulation_config.yaml` and
`animat_config.yaml`:

```yaml
# In simulation_config.yaml (sim-level extensions)
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: ./simulation.hdf5
      skip: 1
  - loader: farms_mujoco.simulation.extensions.MjcfSaver
    config:
      path: ./model

# In animat_config.yaml (animat-level extensions)
extensions:
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config: {}
  - loader: farms_mujoco.simulation.extensions.CameraFollower
    config:
      animat_id: 0
      azimuth: 90
      distance: 2.0
      elevation: -30
```

Each extension entry has:

| Key | Type | Required | Description |
|-----|------|----------|-------------|
| `loader` | str | Yes | Dotted Python path to the extension class |
| `config` | dict | No | Configuration passed to `from_options()` |

See [Use Built-in Extensions](use-extensions.md) for the full extension catalog.

## Multiple animats

To simulate multiple animats, add filenames to `animats` **and** a matching
loader class to `loaders.animats_options`, at the same index — the two lists
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

- [YAML Configuration Schema](../reference/env/yaml-schema.md) — complete schema reference
- [Configure CPG Network Parameters](configure-cpg-network.md) — CPG network YAML
- [Options and YAML Design](../explanation/options-yaml-design.md) — how YAML loading works
