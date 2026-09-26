# YAML Configuration Schema

This reference documents every YAML configuration key, its type, and where it
is parsed in the FARMS source code.

## File hierarchy

```
experiment_config.yaml
├── simulation_config.yaml
├── animat_config.yaml (one or more)
└── arena_config.yaml (one or more)
```

Each sub-config is referenced by filename in `simulation`/`animats`/`arenas`,
with the parsing class named separately in a sibling `loaders:` block
**not** as an inline `loader`/`config` pair per sub-config. (Inline
`{loader, config}` pairs are a different, unrelated mechanism used only for
`extensions:` list entries, see below.)

## experiment_config.yaml

Parsed by `ExperimentOptions` (`farms_core/experiment/options.py`).

| Key | Type | Required | Description |
|-----|------|----------|-------------|
| `simulation` | str | Yes | Path to `simulation_config.yaml` |
| `animats` | list[str] | Yes | Paths to animat config files |
| `arenas` | list[str] | Yes | Paths to arena config files |
| `loaders` | dict | Yes | `ExperimentLoadOptions`, see below |
| `loaders.simulation_options` | str | Yes | Dotted path to the `SimulationOptions` subclass |
| `loaders.animats_options` | list[str] | Yes | Dotted paths, one per `animats` entry (same index) |
| `loaders.arenas_options` | list[str] | Yes | Dotted paths, one per `arenas` entry (same index) |
| `loaders.experiment_data` | str | Yes | Dotted path to the `ExperimentData` subclass |
| `loaders.animats_data` | list[str] | Yes | Dotted paths, one per animat's `AnimatData` subclass |

`ExperimentOptions.load()` requires `len(animats) ==
len(loaders.animats_options)` and `len(arenas) ==
len(loaders.arenas_options)`, or it raises an assertion error naming the
config file.

Example (matches `experiments/zbot_bout_glide/experiment_config.yaml`):

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

## simulation_config.yaml

Parsed by `SimulationOptions` (`farms_core/simulation/options.py`).

| Key | Type | Required | Default | Parsed by |
|-----|------|----------|---------|-----------|
| `units.meters` | float | No | `1.0` | `SimulationUnitScaling` |
| `units.seconds` | float | No | `1.0` | `SimulationUnitScaling` |
| `units.kilograms` | float | No | `1.0` | `SimulationUnitScaling` |
| `runtime.n_iterations` | int | No | `1000` | `RuntimeSimulationOptions` |
| `runtime.buffer_size` | int | No | `n_iterations` | `RuntimeSimulationOptions` |
| `runtime.play` | bool | No | `True` | `RuntimeSimulationOptions` |
| `runtime.rtl` | float | No | `1.0` | `RuntimeSimulationOptions` |
| `runtime.fast` | bool | No | `False` | `RuntimeSimulationOptions` |
| `runtime.headless` | bool | No | `False` | `RuntimeSimulationOptions` |
| `runtime.show_progress` | bool | No | `True` | `RuntimeSimulationOptions` |
| `physics.timestep` | float | No | `1e-3` | `PhysicsSimulationOptions` |
| `physics.gravity` | list[float] | No | `[0, 0, -9.81]` | `PhysicsSimulationOptions` |
| `physics.num_sub_steps` | int | No | `1` | `PhysicsSimulationOptions` |
| `physics.cb_sub_steps` | int | No | `0` | `PhysicsSimulationOptions` |
| `physics.n_solver_iters` | int | No | `50` | `PhysicsSimulationOptions` |
| `mujoco` | dict | No | `{}` | `MuJoCoSimulationOptions` |
| `pybullet` | dict | No | `{}` | `PybulletSimulationOptions` |
| `extensions` | list[dict] | No | `[]` | `SimulationOptions` |

!!! note "Top-level `meters`/`seconds`/`kilograms` also work"
    `SimulationOptions.__init__` accepts either a nested `units:` dict or
    flat `meters`/`seconds`/`kilograms` keys at the top level of
    `simulation_config.yaml`: both populate the same
    `SimulationUnitScaling`.

Each `extensions` entry:

| Key | Type | Required | Description |
|-----|------|----------|-------------|
| `loader` | str | Yes | Dotted Python path to extension class |
| `config` | dict | No | Config dict passed to `from_options()` |

### Built-in simulation-level extension catalog

These are the `TaskExtension` subclasses shipped with FARMS that go in
`simulation_config.yaml`'s `extensions:` list. Every one of them takes an
`animat_id` key even though it's registered at the simulation level, because
none of them receive `animat_data` automatically, each reaches for
`task.data.animats[self.animat_id]` itself in `initialize_episode()`.
Full field-by-field detail, defaults, and known gotchas for each are in
[Use Built-in Extensions](../../how-to/use-extensions.md); this table is the
quick-lookup index.

| Loader path | Purpose | Key config fields |
|-------------|---------|--------------------|
| `farms_core.simulation.extensions.ExperimentLogger` | Writes `simulation.hdf5` at episode end | `log_path`, `skip` |
| `farms_core.simulation.extensions.ExperimentOptionsLogger` | Writes YAML snapshots of the options actually used, for reproducibility | `log_path` |
| `farms_mujoco.simulation.extensions.MjcfSaver` | Dumps the compiled MJCF XML to disk at episode start | `path` (full file path, not a directory) |
| `farms_mujoco.simulation.extensions.CameraFollower` | Moves the **interactive viewer's** camera; no effect headless or in exported video | `animat_id`, `azimuth`, `distance`, `elevation`, `angular_velocity` |
| `farms_mujoco.sensors.camera.CameraRecording` | Independent offscreen camera + renderer that captures frames to an `.mp4`/`.html` video file, works headless | `path`, `resolution`, `fps`, `speed`, `animat_id`, `offset`, `distance`, `azimuth`, `elevation`, `angular_velocity`, `motion_filter`, `geomgroups`, `skips` |
| `farms_mujoco.simulation.extensions.CoMViewer` | Draws a sphere at the animat's centre of mass, in the interactive viewer only | `animat_id`, `size`, `rgba` |
| `farms_mujoco.simulation.extensions.TrailCoMViewer` | Draws a line trail following the animat's CoM over time | `animat_id`, `width`, `rgba`, `spacing` |
| `farms_mujoco.simulation.extensions.TrailLinkViewer` | Same as above but for one named link instead of the whole-animat CoM | `animat_id`, `link`, `width`, `rgba`, `spacing` |
| `farms_mujoco.simulation.extensions.ArrowViewer` | Draws a rotating arrow above the animat's CoM (generic, not bound to a physical vector by default) | `animat_id`, `size`, `rgba`, `offset` |

!!! tip "`CameraFollower` vs `CameraRecording`"
    These are two independent implementations that both move a camera and
    are easy to confuse. `CameraFollower` only touches the live,
    interactive `mujoco.viewer` window and does nothing when
    `runtime.headless: true`. `CameraRecording` renders offscreen with its
    own `mujoco.Renderer` and works identically headless or not, use it
    whenever you need an actual video file, not just a nicer live view.

### Built-in animat-level extension catalog

These extend `AnimatExtension` (`farms_core/model/extensions.py`) and go in
`animat_config.yaml`'s `extensions:` list instead:

| Loader path | Purpose | Key config fields |
|-------------|---------|--------------------|
| `farms_amphibious.control.amphibious.AmphibiousController` | The CPG controller itself, registered as an extension so it participates in the same `before_step`/`after_step` lifecycle as everything else | `{}` (all real configuration comes from `control.network`/`control.muscles` in the same file, not from this extension's own `config`) |
| `farms_mujoco.swimming.extension.SwimmingExtension` | Computes hydrodynamic drag + buoyancy per step and writes them into `xfrc_applied` | `water_properties` (`null` to inherit from the arena's `water:` block) |

!!! warning "Extension order matters"
    Both simulation- and animat-level extensions execute in YAML declaration
    order every step. For the Zbot, `AmphibiousController` must run before
    `SwimmingExtension` so hydrodynamic forces are computed from
    up-to-date joint torques rather than lagging by one step, see
    [Extension ordering](../../how-to/use-extensions.md#extension-ordering).

## animat_config.yaml (AnimatOptions)

Parsed by `AnimatOptions` (`farms_core/model/options.py`).

| Key | Type | Required | Default | Parsed by |
|-----|------|----------|---------|-----------|
| `sdf` | str | Yes | n/a | `ModelOptions` |
| `spawn` | dict | Yes | n/a | `SpawnOptions` |
| `morphology` | dict | Yes | n/a | `MorphologyOptions` |
| `morphology.links` | list[dict] | Yes | n/a | `LinkOptions` |
| `morphology.joints` | list[dict] | Yes | n/a | `JointOptions` |
| `morphology.self_collisions` | list[list[str]] | Yes | n/a | `MorphologyOptions` |
| `control` | dict | Yes | n/a | `ControlOptions` |
| `extensions` | list[dict] | No | `[]` | `AnimatOptions` |

### SpawnOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `loader` | int (`SpawnLoader`) | Yes | n/a | `0` = FARMS loader (recommended), `1` = PyBullet loader |
| `mode` | str (`SpawnMode`) | No | `free` | Spawn constraint mode |
| `pose` | list[float] (6) | Yes | n/a | `[X, Y, Z, Rx, Ry, Rz]`, position [m] + Euler orientation [rad] |
| `velocity` | list[float] (6) | Yes | n/a | `[Vx, Vy, Vz, Wx, Wy, Wz]`, initial linear + angular velocity |
| `extras` | dict | No | `{}` | Deprecated extra options |

!!! warning "Two unrelated meanings of `loader` in this file"
    `SpawnOptions.loader` is an integer `SpawnLoader` enum (0 or 1) chosen
    from a fixed set of built-in loaders, it has nothing to do with the
    dotted-path `loader:` strings used elsewhere (`controller_loader`,
    `ExtensionOptions.loader`, `ExperimentLoadOptions`'s `*_options`
    fields). Don't assume every `loader` key is a Python import path.

SpawnMode values (`farms_core/model/options.py`): `free`, `fixed`, `rotx`,
`roty`, `rotz`, `sagittal`, `sagittal0`, `sagittal3`, `coronal`, `coronal0`,
`coronal3`, `transverse`, `transverse0`, `transverse3`.

### LinkOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `name` | str | Yes | n/a | Link name (must match SDF) |
| `collisions` | bool | Yes | n/a | Enable collision detection |
| `friction` | list[float] | Yes | n/a | [lateral, spinning, rolling] |
| `fluid_interaction` | bool | No | `False` | Enable fluid forces |
| `density` | float | No | `1000` | Density [kg/m³] |
| `drag_coefficients` | list[list[float]] | No | `[0,0,0,0,0,0]`\* | `[[Vx,Vy,Vz],[Wx,Wy,Wz]]`, linear/angular drag coefficients |
| `sites` | list | No | `[]` | Site definitions |
| `solref` | list | No | `None` | MuJoCo solref |
| `solimp` | list | No | `None` | MuJoCo solimp |
| `extras` | dict | No | `{}` | Extra properties |

\* `LinkOptions.__init__`'s default value (`[0, 0, 0, 0, 0, 0]`, a flat
6-list) doesn't match the nested `[[Vx,Vy,Vz],[Wx,Wy,Wz]]` shape documented
for and used by real configs, a pre-existing inconsistency in
`farms_core/model/options.py`, not a documentation error. Always supply
`drag_coefficients` explicitly as two 3-lists.

### JointOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `name` | str | Yes | n/a | Joint name (must match SDF) |
| `initial` | list[float] | Yes | n/a | [position, velocity] |
| `limits` | list[list[float]] | Yes | n/a | [[pos_min, pos_max], [vel_min, vel_max]] |
| `stiffness` | float | Yes | n/a | Joint stiffness |
| `springref` | float | Yes | n/a | Spring reference |
| `damping` | float | Yes | n/a | Joint damping |
| `extras` | dict | No | `{}` | Extra properties |

### ControlOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `controller_loader` | str | No | `None` | Dotted path to controller class |
| `sensors` | dict | Yes | n/a | `SensorsOptions` |
| `motors` | list[dict] | Yes | n/a | List of `MotorOptions` |
| `hill_muscles` | list | No | `[]` | Hill muscle definitions |

### SensorsOptions

Declared under `control.sensors` in `animat_config.yaml`. Each field is a
list of link/joint/etc. names to record; sensor data is written every step
into fixed-shape NumPy arrays under `AnimatData.sensors`, with the exact
per-category column layout defined by the `sc` (sensor convention) enum in
`farms_core/sensors/sensor_convention.pyx`: see
[Add and Configure Sensors](../../how-to/configure-sensors.md#sensor-types)
for the full column-by-column table (link/joint/contact/xfrc/muscle
layouts) and code examples reading each array.

| Key | Type | Required | Records | Array shape |
|-----|------|----------|---------|-------------|
| `links` | list[str] | Yes | CoM + URDF-frame position/orientation, linear/angular velocity, per named link | `(n_iters, n_links, 20)` |
| `joints` | list[str] | Yes | Position, velocity, torque, commanded values, torque decomposition, per named joint | `(n_iters, n_joints, 17)` |
| `contacts` | list[str] \| list[list[str]] | Yes | Reaction/friction/total force + contact position, per named link or `[link_a, link_b]` pair | `(n_iters, n_contacts, 12)` |
| `xfrc` | list[str] | Yes | External applied force/torque (e.g. from `SwimmingExtension`), per named link | `(n_iters, n_links, 6)` |
| `muscles` | list[str] | Yes | Excitation/activation, tendon/fibre length & velocity, force, spindle feedback, per named muscle | `(n_iters, n_muscles, 17)` |
| `adhesions` | list[str] | Yes | Adhesion force, per named adhesion actuator | `(n_iters, n_adhesions, 1)` |
| `visuals` | list[str] | Yes | Colour + emission RGBA, per named visual | `(n_iters, n_visuals, 8)` |

!!! tip "Empty lists are the normal state for unused sensor categories"
    The Zbot config sets `muscles: []`, `adhesions: []`, and `visuals: []`
    this is expected, not a gap: those categories only apply to
    Hill-muscle-actuated or adhesion/visual-effector morphologies. Only
    `links`, `joints`, and `xfrc` are populated for a plain swimming
    experiment.

### MotorOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `joint_name` | str | Yes | n/a | Target joint name |
| `control_types` | list[str] | Yes | n/a | Control type strings |
| `limits_torque` | list[float] | Yes | n/a | [min_torque, max_torque] |
| `gains` | list[float] | Yes | n/a | Motor gains |

## animat_config.yaml (AmphibiousOptions)

Parsed by `AmphibiousOptions` (`farms_amphibious/model/options.py`). Extends
`AnimatOptions` with:

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `show_xfrc` | bool | No | `False` | Visualize external forces |
| `scale_xfrc` | int | No | `1` | Force visualization scale |
| `mujoco` | dict | No | `{}` | MuJoCo-specific options |
| `control.network` | dict | No | `None` | CPG network config (see below) |
| `control.muscles` | list[dict] | No | `[]` | Muscle set definitions |
| `control.adhesions` | list[dict] | No | `[]` | Adhesion definitions |
| `control.visuals` | list[dict] | No | `[]` | Visual definitions |

### AmphibiousMotorOptions

Extends `MotorOptions` with:

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `equation` | str | Yes | n/a | Motor equation type |
| `transform` | dict | No | `None` | `AmphibiousMotorTransformOptions` |
| `offsets` | dict | No | `None` | `AmphibiousMotorOffsetOptions` |
| `passive` | dict | No | `None` | `AmphibiousPassiveJointOptions` |

Motor equation types: `phase`, `position_muscle`, `ekeberg_muscle`,
`ekeberg_muscle_explicit`, `passive`, `passive_explicit`

### Network options

See [Configure CPG Network Parameters](../../how-to/configure-cpg-network.md)
for the full network schema.

## arena_config.yaml

Parsed by `ArenaOptions` (`farms_core/model/options.py`).

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `sdf` | str | Yes | n/a | Arena SDF file path |
| `spawn` | dict | Yes | n/a | `SpawnOptions` |
| `water` | dict | No | n/a | `WaterOptions` |
| `ground_height` | float | No | `0.0` | Ground plane height |

### WaterOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `sdf` | str | No | `''` | Water visual/volume SDF |
| `drag` | bool | Yes | n/a | Whether to apply hydrodynamic drag forces at all (not a coefficient list, the per-link coefficients live in each link's own `drag_coefficients`, see `LinkOptions` above) |
| `buoyancy` | bool | Yes | n/a | Enable buoyancy forces |
| `height` | float | Yes | n/a | Water surface height [m] |
| `velocity` | list[float] (3) | No | `[0,0,0]` | Fluid current `[Vx, Vy, Vz]` [m/s] |
| `viscosity` | float | No | `0.0` | Used as a drag multiplier by `SwimmingHandler` |
| `density` | float | Yes | n/a | Fluid density [kg/m³], used for buoyancy |
| `maps` | list[str] | No | `['', '']` | Optional spatially-varying velocity/height callback references; empty strings disable spatial variation and use the uniform `velocity`/`height` values everywhere |

