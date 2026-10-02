# Extension and Controller Design

Extensions are how code plugs into a FARMS simulation: controllers, fluid
forces, loggers, cameras and viewer markers are all extensions. This page
explains the design, the lifecycle and the contracts of the base classes.
The signatures are in the [API reference](../reference/core/core-control.md).

## Why extensions?

Instead of subclassing the simulation to add behaviour, FARMS composes
independent extensions listed in the YAML files:

- **Composable**: any set of extensions can be combined without code
  changes.
- **Ordered**: they run in the order of the YAML lists.
- **Reusable**: the same extension works with different robots.
- **Isolated**: each extension keeps its own state.

## Class hierarchy

```text
TaskExtension (ABC)                   farms_core.simulation.extensions
├── ExperimentLogger                  farms_core.simulation.extensions
├── ExperimentOptionsLogger           farms_core.simulation.extensions
├── MjcfSaver                         farms_mujoco.simulation.extensions
├── CameraRecording                   farms_mujoco.sensors.camera
├── AnimatViewerExtension             farms_mujoco.simulation.extensions
│   ├── CameraFollower
│   ├── CoMViewer, TrailCoMViewer, TrailLinkViewer, ArrowViewer
│   └── SnakeGame
└── AnimatExtension (ABC)             farms_core.model.extensions
    ├── SwimmingExtension             farms_mujoco.swimming.extension
    ├── Targets2Reach                 farms_mujoco.viewer
    └── AnimatController              farms_core.model.control
        ├── JointMuscleController     farms_amphibious.control.amphibious
        │   └── AmphibiousController
        └── TravelingWaveController   examples/amphibot/controller (your own)
```

The viewer extensions (`AnimatViewerExtension`) follow one animat
(`animat_id`) but are simulation extensions: they are listed in the
simulation file.

## Two extension levels

| | `TaskExtension` | `AnimatExtension` |
|---|---|---|
| Listed in | `extensions:` of the simulation file | `extensions:` of an animat file |
| Created with | `from_options(config, experiment_options)` | `from_options(config, experiment_options, animat_i, animat_data, animat_options)` |
| Examples | `ExperimentLogger`, `CameraRecording`, `CoMViewer` | `SwimmingExtension`, controllers |

Each entry has a `loader` (the dotted path of the class) and a `config`
(a free dictionary passed to `from_options()`):

```yaml
# simulation_config.yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: Output
      skip: 1
```

```yaml
# animat_config.yaml
extensions:
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config: {}
```

When the task is created, `ExperimentTask.extract_extensions()` imports
each class (`farms_core.extensions.extensions.import_item`) and calls its
`from_options()`: first the simulation extensions, then the extensions of
each animat. The experiment data is not ready yet at that point: an
extension that needs it takes it from `task.data` in
`initialize_episode()` (as `ExperimentLogger` does).

## The lifecycle

| Method | Called by | When |
|---|---|---|
| `initialize_episode(task, physics)` | `ExperimentTask.initialize_episode()` | Start of an episode, after the reset |
| `before_step(task, action, physics)` | `ExperimentTask.before_step()` | Before the MuJoCo step: once per iteration, or at every environment step with `substep=True` |
| `after_step(task, physics)` | `ExperimentTask.after_step()` | Once per iteration, after its last environment step |
| `end_episode(task, physics)` | `Simulation.end_extensions()` | End of the simulation |

An iteration is `physics.cb_sub_steps` environment steps (see
[Simulation Lifecycle](simulation-lifecycle.md)). In `before_step()`,
extensions apply forces (`physics.data.xfrc_applied`) and controllers
advance their dynamics. `after_step()` sees the state after the step. Only
`from_options()` is abstract: the other methods do nothing by default.

`action_spec`, `step_spec`, `get_observation`, `get_reward`,
`get_termination` and `observation_spec` let an extension provide the
dm_control environment interface, for example for reinforcement learning.

## Controllers are extensions

`AnimatController` is an `AnimatExtension` that also produces joint
commands. There is no separate mechanism to create controllers: a
controller runs because it is listed in the animat's `extensions:`
(`control.controller_loader` is parsed but not used).

In `ExperimentTask.before_step()`, for each extension in order:

1. `extension.before_step(task, action, physics)` is called;
2. if the extension is an `AnimatController`, its commands are read right
   away and written to MuJoCo: `positions()`, `velocities()` and
   `torques()` to the actuators when the controller has joints of that
   type, `springrefs()`, `springcoefs()` and `dampingcoefs()` to the model
   along with the torques, and `excitations()` to the muscle actuators.

All extensions run before the MuJoCo step, from the state at the start of
the step, so their order only matters when one extension reads what
another one wrote in the same step.

### Control types and joint mapping

`ControlType` lets joints of the same robot use different kinds of
control. `joints_names` has one list of joints per `ControlType`, in enum
order (`joints_names[ControlType.POSITION]`, ...), each keeping the order
of the motors. `AnimatController.joints_from_control_types()` builds it
from the motors' `control_types`, and `max_torques_from_control_types()`
the matching torque limits. Each command method only returns the joints of
its control type.

### The substep parameter

With `substep=True` (the default for controllers), `before_step()` runs at
every environment step, and the commands are updated at that rate. This
matters for stiff dynamics, such as the Ekeberg muscle model, and for
forces that depend on the state, such as the fluid forces
(`SwimmingExtension` also uses `substep=True`).

## Contracts

### TaskExtension and AnimatExtension

- `from_options()` is the factory: the framework passes the context, and
  the extension keeps what it needs.
- The sensor arrays are ring buffers of `runtime.buffer_size` iterations:
  index them with `task.iteration % task.buffer_size`. `AnimatController`
  methods receive this index directly as `iteration`.
- Arrays of MuJoCo (`physics.data.*`) are owned by MuJoCo: modify them in
  place, in simulation units (`task.units`).
- To add a force, write a world frame wrench to
  `physics.data.xfrc_applied[body_id]` in `before_step()`. It is applied
  at the body CoM. See [Hydrodynamics Internals](../internals/hydrodynamics-internals.md)
  for an example.

### AnimatController

- `joints_names` must have one list per `ControlType` (7 lists).
- The command methods take `(iteration, time, timestep)` and return a
  `dict[str, float]` whose keys are joint names of that control type.
- Keep the dynamics in `before_step()` and the command methods free of
  side effects.

### farms_amphibious networks and drives

| Base class | Method to implement | Contract |
|---|---|---|
| `farms_amphibious.control.network.AnimatNetwork` | `step(iteration, time, timestep)` | Update the network state (`data.state.array`) at `iteration` (a buffer index), in place. `NetworkODE` integrates the CPG ODE |
| `farms_amphibious.control.drive.DescendingDrive` | `step(iteration, time, timestep)` | Set the drives of the iteration with `set_left_drives()` / `set_right_drives()` |
| `farms_amphibious.control.drive.PotentialMap` | `heading(pos)` | Desired heading at a 2D position, used by `OrientationFollower` |

## See also

- [Controller and extension API](../reference/core/core-control.md)
- [Write an AnimatExtension](../how-to/write-extension.md)
- [Write a Controller](../how-to/write-controller.md)
- [Use Built-in Extensions](../how-to/use-extensions.md)
- [Simulation Lifecycle](simulation-lifecycle.md)
