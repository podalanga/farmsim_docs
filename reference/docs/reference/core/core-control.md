# `farms_core.simulation.extensions` / `farms_core.model.control`

Base classes of the extensions and controllers, and the logging
extensions. The lifecycle (when each method is called) is described in
[Extension and Controller Design](../../explanation/extension-design.md).

## TaskExtension

The base class of every extension: code called by `ExperimentTask` during
the simulation. A simulation extension is listed in the `extensions:` of
the simulation file and created with
`from_options(config, experiment_options)`.

With `substep=False` (the default), `before_step()` is called once per
iteration. With `substep=True`, it is called at every environment step
(`physics.cb_sub_steps` times per iteration). `after_step()` is called
once per iteration.

The `action_spec`, `step_spec`, `get_observation`, `get_reward`,
`get_termination` and `observation_spec` methods let an extension define
a dm_control / reinforcement learning interface.

::: farms_core.simulation.extensions.TaskExtension
    options:
      show_root_heading: false
      heading_level: 3

## AnimatExtension

An extension attached to one animat, listed in the `extensions:` of an
animat file. Its `from_options()` also receives the index of the animat,
its `AnimatData` and its options.

::: farms_core.model.extensions.AnimatExtension
    options:
      show_root_heading: false
      heading_level: 3

## AnimatController

An `AnimatExtension` whose commands `ExperimentTask` writes to the
actuators after its `before_step()`. Each command method takes
`(iteration, time, timestep)` and returns a `dict[str, float]` from joint
(or muscle) name to value:

| Method | Command | `ControlType` |
|--------|---------|---------------|
| `positions` | Position targets | `POSITION` |
| `velocities` | Velocity targets | `VELOCITY` |
| `torques` | Torques | `TORQUE` |
| `springrefs` | Spring reference positions | `SPRINGREF` |
| `springcoefs` | Spring stiffnesses | `SPRINGCOEF` |
| `dampingcoefs` | Damping coefficients | `DAMPINGCOEF` |
| `excitations` | Muscle excitations | `MUSCLE` |

`joints_names` and `max_torques` hold one entry per `ControlType`. The
static methods `joints_from_control_types()` and
`max_torques_from_control_types()` build them from the motor options.
The default `from_options()` creates a controller without joints:
override it. See [Write a Custom Controller](../../tutorials/custom-controller.md).

::: farms_core.model.control.AnimatController
    options:
      show_root_heading: false
      heading_level: 3

## ControlType

| Value | Code | Controls |
|-------|------|----------|
| `POSITION` | `0` | Position targets |
| `VELOCITY` | `1` | Velocity targets |
| `TORQUE` | `2` | Torques or forces |
| `SPRINGREF` | `3` | Spring reference position |
| `SPRINGCOEF` | `4` | Joint stiffness |
| `DAMPINGCOEF` | `5` | Joint damping |
| `MUSCLE` | `6` | Muscle excitations |

`ControlType.from_string_list(['position', 'velocity'])` converts the
`control_types` of a motor.

## ExperimentLogger

Keeps a reference to the `ExperimentData` and writes it to
`<log_path>/simulation.hdf5` at the end of the episode (`end_episode()`).
The `skip` option is stored but not used: every iteration held in the
buffers is saved.

```yaml
# simulation_config.yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: Output
      skip: 1
```

## ExperimentOptionsLogger

Writes the options of the simulation, animats and arenas to
`<log_path>/simulation_options.yaml`, `animat_<i>_options.yaml` and
`arena_<i>_options.yaml` at the start of the episode.

```yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentOptionsLogger
    config:
      log_path: Output
```

## See Also

- [Options](core-options.md)
- [MuJoCo Simulation](../mujoco/mujoco-simulation.md)
- [API reference: `farms_core.model.control`](../api/farms_core/model/control.md)
