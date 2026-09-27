# YAML Configuration Schema

How the YAML files of an experiment fit together. The list of every key,
with its type and description, is the
[Configuration Parameter Reference](configuration-reference.md), generated
from the code at each build.

## File hierarchy

```text
experiment_config.yaml        ExperimentOptions
├── simulation_config.yaml    SimulationOptions
├── animat_config.yaml        AnimatOptions or AmphibiousOptions (one per animat)
└── arena_config.yaml         ArenaOptions or AmphibiousArenaOptions (one per arena)
```

The experiment file lists the other files by name, and names the classes
that read them in a separate `loaders:` block. The `extensions:` lists of
the simulation and animat files use a different mechanism: each entry is a
`{loader, config}` pair, created by the task.

## experiment_config.yaml

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

Paths are relative to the experiment file. `ExperimentOptions.load()`
requires as many `loaders.animats_options` as `animats`, and as many
`loaders.arenas_options` as `arenas` (matched by index). See
[ExperimentOptions](configuration-reference.md#experimentoptions) and
[ExperimentLoadOptions](configuration-reference.md#experimentloadoptions).

## simulation_config.yaml

| Block | Content | Reference |
|-------|---------|-----------|
| `units` | Unit scaling (`meters`, `seconds`, `kilograms`) | [SimulationUnitScaling](configuration-reference.md#simulationunitscaling) |
| `runtime` | `n_iterations`, `buffer_size`, `headless`, `fast`, `rtl`, `play`, `show_progress`, ... | [RuntimeSimulationOptions](configuration-reference.md#runtimesimulationoptions) |
| `physics` | `timestep`, `gravity`, `cb_sub_steps`, `num_sub_steps`, `n_solver_iters`, ... | [PhysicsSimulationOptions](configuration-reference.md#physicssimulationoptions) |
| `mujoco` | Solver, integrator, cone, viewer, ... | [MuJoCoSimulationOptions](configuration-reference.md#mujocosimulationoptions) |
| `pybullet` | PyBullet options | [PybulletSimulationOptions](configuration-reference.md#pybulletsimulationoptions) |
| `extensions` | Loggers, `MjcfSaver`, cameras, viewer markers | [Use Built-in Extensions](../../how-to/use-extensions.md) |

Timing: an iteration lasts `physics.timestep`, is split into
`physics.cb_sub_steps` environment steps, each of `physics.num_sub_steps`
MuJoCo steps, and the sensors are logged once per iteration.

## animat_config.yaml

| Block | Content | Reference |
|-------|---------|-----------|
| `sdf` | Model file | [AnimatOptions](configuration-reference.md#animatoptions) |
| `spawn` | `loader`, `mode`, `pose`, `velocity` | [SpawnOptions](configuration-reference.md#spawnoptions), [SpawnMode](configuration-reference.md#spawnmode) |
| `morphology` | `links`, `joints`, `self_collisions`, ... | [MorphologyOptions](configuration-reference.md#morphologyoptions), [LinkOptions](configuration-reference.md#linkoptions), [JointOptions](configuration-reference.md#jointoptions) |
| `control.sensors` | Sensed links, joints, contacts, ... | [SensorsOptions](configuration-reference.md#sensorsoptions), [Add and Configure Sensors](../../how-to/configure-sensors.md) |
| `control.motors` | Actuated joints, control types, gains, limits, and for `AmphibiousOptions` the joint `equation`, `transform`, `offsets`, `passive` | [MotorOptions](configuration-reference.md#motoroptions) |
| `control.network`, `control.muscles` | CPG network (`AmphibiousOptions`) | [Configure CPG Network Parameters](../../how-to/configure-cpg-network.md) |
| `control.hill_muscles` | MuJoCo muscles | [MuscleOptions](configuration-reference.md#muscleoptions) |
| `extensions` | Controller, `SwimmingExtension` | [AnimatExtensionOptions](configuration-reference.md#animatextensionoptions) |
| `show_xfrc`, `scale_xfrc`, `mujoco` | Display of the external forces, MuJoCo options (`AmphibiousOptions`) | [AmphibiousOptions](configuration-reference.md#amphibiousoptions) |

Notes:

- `control.controller_loader` is parsed but not used: controllers are
  created from `extensions:`.
- Link `density` is only used by `cob_method: ramp`, `mass_multiplier` only
  by PyBullet, and the link `solref`/`solimp` are not applied by the MuJoCo
  builder.
- Motor `gains` are `[kp of the position actuator, kv of the position actuator, kv of the velocity actuator]`.

## arena_config.yaml

| Block | Content | Reference |
|-------|---------|-----------|
| `sdf` | Arena model file | [ArenaOptions](configuration-reference.md#arenaoptions) |
| `spawn` | Arena pose | [SpawnOptions](configuration-reference.md#spawnoptions) |
| `water` | Water surface, density, velocity, drag and buoyancy switches, fluid model | [WaterOptions](configuration-reference.md#wateroptions), [Fluid model options](configuration-reference.md#fluid-model-options) |
| `ground_height` | Height of the ground | [ArenaOptions](configuration-reference.md#arenaoptions) |

The fluid model is described in [farms_mujoco.swimming](../mujoco/mujoco-swimming.md).

## See also

- [Configuration Parameter Reference](configuration-reference.md)
- [Configure an Experiment YAML](../../how-to/configure-yaml.md)
- [Swimming Experiment](../../tutorials/zbot-experiment.md): the Zbot files, block by block
- [Options and YAML Design](../../explanation/options-yaml-design.md)
