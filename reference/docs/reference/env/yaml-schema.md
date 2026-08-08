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
with the parsing class named separately in a sibling `loaders:` block —
**not** as an inline `loader`/`config` pair per sub-config. (Inline
`{loader, config}` pairs are a different, unrelated mechanism used only for
`extensions:` list entries — see below.)

## experiment_config.yaml

Parsed by `ExperimentOptions` (`farms_core/experiment/options.py`).

| Key | Type | Required | Description |
|-----|------|----------|-------------|
| `simulation` | str | Yes | Path to `simulation_config.yaml` |
| `animats` | list[str] | Yes | Paths to animat config files |
| `arenas` | list[str] | Yes | Paths to arena config files |
| `loaders` | dict | Yes | `ExperimentLoadOptions` — see below |
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
    `simulation_config.yaml` — both populate the same
    `SimulationUnitScaling`.

Each `extensions` entry:

| Key | Type | Required | Description |
|-----|------|----------|-------------|
| `loader` | str | Yes | Dotted Python path to extension class |
| `config` | dict | No | Config dict passed to `from_options()` |

## animat_config.yaml (AnimatOptions)

Parsed by `AnimatOptions` (`farms_core/model/options.py`).

| Key | Type | Required | Default | Parsed by |
|-----|------|----------|---------|-----------|
| `sdf` | str | Yes | — | `ModelOptions` |
| `spawn` | dict | Yes | — | `SpawnOptions` |
| `morphology` | dict | Yes | — | `MorphologyOptions` |
| `morphology.links` | list[dict] | Yes | — | `LinkOptions` |
| `morphology.joints` | list[dict] | Yes | — | `JointOptions` |
| `morphology.self_collisions` | list[list[str]] | Yes | — | `MorphologyOptions` |
| `control` | dict | Yes | — | `ControlOptions` |
| `extensions` | list[dict] | No | `[]` | `AnimatOptions` |

### SpawnOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `loader` | int (`SpawnLoader`) | Yes | — | `0` = FARMS loader (recommended), `1` = PyBullet loader |
| `mode` | str (`SpawnMode`) | No | `free` | Spawn constraint mode |
| `pose` | list[float] (6) | Yes | — | `[X, Y, Z, Rx, Ry, Rz]` — position [m] + Euler orientation [rad] |
| `velocity` | list[float] (6) | Yes | — | `[Vx, Vy, Vz, Wx, Wy, Wz]` — initial linear + angular velocity |
| `extras` | dict | No | `{}` | Deprecated extra options |

!!! warning "Two unrelated meanings of `loader` in this file"
    `SpawnOptions.loader` is an integer `SpawnLoader` enum (0 or 1) chosen
    from a fixed set of built-in loaders — it has nothing to do with the
    dotted-path `loader:` strings used elsewhere (`controller_loader`,
    `ExtensionOptions.loader`, `ExperimentLoadOptions`'s `*_options`
    fields). Don't assume every `loader` key is a Python import path.

SpawnMode values (`farms_core/model/options.py`): `free`, `fixed`, `rotx`,
`roty`, `rotz`, `sagittal`, `sagittal0`, `sagittal3`, `coronal`, `coronal0`,
`coronal3`, `transverse`, `transverse0`, `transverse3`.

### LinkOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `name` | str | Yes | — | Link name (must match SDF) |
| `collisions` | bool | Yes | — | Enable collision detection |
| `friction` | list[float] | Yes | — | [lateral, spinning, rolling] |
| `fluid_interaction` | bool | No | `False` | Enable fluid forces |
| `density` | float | No | `1000` | Density [kg/m³] |
| `drag_coefficients` | list[list[float]] | No | `[0,0,0,0,0,0]`\* | `[[Vx,Vy,Vz],[Wx,Wy,Wz]]` — linear/angular drag coefficients |
| `sites` | list | No | `[]` | Site definitions |
| `solref` | list | No | `None` | MuJoCo solref |
| `solimp` | list | No | `None` | MuJoCo solimp |
| `extras` | dict | No | `{}` | Extra properties |

\* `LinkOptions.__init__`'s default value (`[0, 0, 0, 0, 0, 0]`, a flat
6-list) doesn't match the nested `[[Vx,Vy,Vz],[Wx,Wy,Wz]]` shape documented
for and used by real configs — a pre-existing inconsistency in
`farms_core/model/options.py`, not a documentation error. Always supply
`drag_coefficients` explicitly as two 3-lists.

### JointOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `name` | str | Yes | — | Joint name (must match SDF) |
| `initial` | list[float] | Yes | — | [position, velocity] |
| `limits` | list[list[float]] | Yes | — | [[pos_min, pos_max], [vel_min, vel_max]] |
| `stiffness` | float | Yes | — | Joint stiffness |
| `springref` | float | Yes | — | Spring reference |
| `damping` | float | Yes | — | Joint damping |
| `extras` | dict | No | `{}` | Extra properties |

### ControlOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `controller_loader` | str | No | `None` | Dotted path to controller class |
| `sensors` | dict | Yes | — | `SensorsOptions` |
| `motors` | list[dict] | Yes | — | List of `MotorOptions` |
| `hill_muscles` | list | No | `[]` | Hill muscle definitions |

### SensorsOptions

| Key | Type | Required | Description |
|-----|------|----------|-------------|
| `links` | list[str] | Yes | Link names to sense |
| `joints` | list[str] | Yes | Joint names to sense |
| `contacts` | list[str] | Yes | Contact link names |
| `xfrc` | list[str] | Yes | XFRC link names |
| `muscles` | list[str] | Yes | Muscle names |
| `adhesions` | list[str] | Yes | Adhesion names |
| `visuals` | list[str] | Yes | Visual sensor names |

### MotorOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `joint_name` | str | Yes | — | Target joint name |
| `control_types` | list[str] | Yes | — | Control type strings |
| `limits_torque` | list[float] | Yes | — | [min_torque, max_torque] |
| `gains` | list[float] | Yes | — | Motor gains |

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
| `equation` | str | Yes | — | Motor equation type |
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
| `sdf` | str | Yes | — | Arena SDF file path |
| `spawn` | dict | Yes | — | `SpawnOptions` |
| `water` | dict | No | — | `WaterOptions` |
| `ground_height` | float | No | `0.0` | Ground plane height |

### WaterOptions

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `sdf` | str | No | `''` | Water visual SDF |
| `drag` | list[float] | Yes | — | 6 drag coefficients |
| `buoyancy` | bool | Yes | — | Enable buoyancy |
| `height` | float | Yes | — | Water surface height |
| `velocity` | list[float] | No | `[0,0,0,0,0,0]` | Fluid velocity |
| `viscosity` | float | No | `0.0` | Fluid viscosity |
| `density` | float | Yes | — | Fluid density [kg/m³] |
| `maps` | dict | No | `{}` | Per-link fluid maps |
