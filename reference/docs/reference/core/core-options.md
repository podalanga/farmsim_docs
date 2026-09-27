# farms_core.model.options

The options classes of the animat and arena files
(`farms_core/model/options.py`), and the `Options` base class. The keys,
types and descriptions of every class are in the generated
[Configuration Parameter Reference](../env/configuration-reference.md);
this page adds what a table cannot say.

## The Options base class

`Options` (`farms_core/options.py`) is a `dict` subclass with attribute
access (`options.spawn.pose`). `load(filename, strict=True)` reads a YAML
file and calls the class's `__init__` with its content; `save(filename)`
writes it back. Each class pops its keys in `__init__` and fails on
unknown keys (unless loaded with `strict=False`, for the `farms_core`
classes). Some classes also have a `from_options()` class method, which
builds the options from a flat dictionary with defaults.

::: farms_core.options.Options
    options:
      show_root_heading: false
      heading_level: 3
      members: [load, save]

## Animat options

| Class | YAML block | Reference |
|-------|-----------|-----------|
| `AnimatOptions` | the animat file | [AnimatOptions](../env/configuration-reference.md#animatoptions) |
| `SpawnOptions` | `spawn` | [SpawnOptions](../env/configuration-reference.md#spawnoptions), [SpawnLoader](../env/configuration-reference.md#spawnloader), [SpawnMode](../env/configuration-reference.md#spawnmode) |
| `MorphologyOptions` | `morphology` | [MorphologyOptions](../env/configuration-reference.md#morphologyoptions) |
| `LinkOptions` | `morphology.links[]` | [LinkOptions](../env/configuration-reference.md#linkoptions) |
| `JointOptions` | `morphology.joints[]` | [JointOptions](../env/configuration-reference.md#jointoptions) |
| `ControlOptions` | `control` | [ControlOptions](../env/configuration-reference.md#controloptions) |
| `SensorsOptions` | `control.sensors` | [SensorsOptions](../env/configuration-reference.md#sensorsoptions) |
| `MotorOptions` | `control.motors[]` | [MotorOptions](../env/configuration-reference.md#motoroptions) |
| `MuscleOptions` | `control.hill_muscles[]` | [MuscleOptions](../env/configuration-reference.md#muscleoptions) |

`farms_amphibious` extends them (`AmphibiousOptions`, ...), see
[farms_amphibious.model.options](../amphibious/amphibious-options.md).

### What the options do in MuJoCo

| Option | Effect with MuJoCo |
|--------|--------------------|
| `spawn.pose` | `[x, y, z, roll, pitch, yaw]` [m, rad] of the base link, used as keyframe 0 |
| `spawn.mode` | Constraints on the base link (`free`, `fixed`, `rotx`, `sagittal`, ...) |
| `links[].friction` | Friction of the link's collision geoms |
| `links[].collisions` | Not used by the MuJoCo builder: the collision geoms are those of the SDF file |
| `links[].fluid_interaction` | Whether `SwimmingExtension` applies fluid forces to the link |
| `links[].drag_coefficients` | `[[cx, cy, cz], [c'x, c'y, c'z]]`, linear and rotational drag in the link frame, negative (legacy fluid model) |
| `links[].density` | Only used by `cob_method: ramp`, to estimate the link volume as `mass/density`. Masses come from the SDF file |
| `links[].solref`, `solimp` | Parsed but not applied per link |
| `joints[].stiffness`, `damping`, `springref` | Added to the MuJoCo joint. The joint limits come from the SDF file |
| `joints[].initial` | Initial `[position, velocity]` |
| `motors[].control_types` | Actuators created for the joint (`position`, `velocity`, `torque`, ...) |
| `motors[].gains` | `[kp of the position actuator, kv of the position actuator, kv of the velocity actuator]` |
| `motors[].limits_torque` | Force range of the actuators |
| `morphology.self_collisions` | Link pairs that collide; the other links of an animat do not collide with each other |
| `control.controller_loader` | Parsed but not used: controllers are listed in `extensions:` |

## Arena and water options

| Class | YAML block | Reference |
|-------|-----------|-----------|
| `ArenaOptions` | the arena file | [ArenaOptions](../env/configuration-reference.md#arenaoptions) |
| `WaterOptions` | `water` | [WaterOptions](../env/configuration-reference.md#wateroptions), [Fluid model options](../env/configuration-reference.md#fluid-model-options) |

`water.viscosity` scales the quadratic drag (it is not a viscosity in
Pa.s; the ellipsoid model uses `dynamic_viscosity` for that).
`water.velocity` is a 3-vector, or the ranges and area of velocity maps
given in `water.maps`. The fluid model is described in
[farms_mujoco.swimming](../mujoco/mujoco-swimming.md).

## Simulation options

`SimulationOptions` (`farms_core/simulation/options.py`) has `units`,
`runtime`, `physics`, `mujoco`, `pybullet` and `extensions`; see
[SimulationOptions](../env/configuration-reference.md#simulationoptions).
`duration()` returns `physics.timestep*(runtime.n_iterations - 1)` and
`times()` the times of the iterations.

## See Also

- [Configuration Parameter Reference](../env/configuration-reference.md)
- [YAML Configuration Schema](../env/yaml-schema.md)
- [Options and YAML Design](../../explanation/options-yaml-design.md)
