# farms_amphibious.control.amphibious

The controllers of `farms_amphibious`: they integrate the CPG network and
the descending drive, and convert the oscillator outputs into joint
commands with the joint equations.

```text
AnimatController                       farms_core.model.control
└── JointMuscleController              Joint equations, JointsMap, MusclesMap
    └── AmphibiousController           + NetworkODE and descending drive
        └── AmphibiousDriveController  + colours the visuals from drives and phases
AnimatController
└── KinematicsController               farms_amphibious.control.kinematics: replays recorded kinematics
```

## JointMuscleController

Built with `(animat_i, animat_options, animat_data, animat_network)`. At
construction, it groups the motors by `equation` and creates one handler
per equation (`PositionPhaseCy`, `PositionMuscleCy`, `EkebergMuscleCy`,
`PassiveJointCy`), stored in `network2joints`. `JointsMap` holds the
per-joint transform gains and biases, and `MusclesMap` the two oscillators
and the coefficients of each joint. `positions()`, `velocities()` and
`torques()` collect the commands of the handlers of each control type,
and `springrefs()`, `springcoefs()`, `dampingcoefs()` those of
`ekeberg_muscle` and `passive`.

## AmphibiousController

The controller used by `experiments/zbot_swimming`, listed in the animat's
`extensions:`. `from_options()`:

- creates a `NetworkODE` (`dopri5`, `nsteps=1000`, maximum step
  `physics.timestep`) when the animat data has a CPG state
  (`AmphibiousData`);
- when `control.network.drive_config` names a YAML file, loads it and
  creates the drive of `control.network.drive_loader` (for example
  `OrientationFollower`).

`before_step()` calls `step(index, time, timestep)` at every environment
step, which steps the drive, then the network, then every joint handler.
`initialize_episode()` clears the sensor and drive buffers (except the
first iteration) and resets the network.

::: farms_amphibious.control.amphibious.AmphibiousController
    options:
      show_root_heading: false
      heading_level: 3
      members: [from_options, initialize_episode, before_step, step]

## AmphibiousDriveController

Also colours the animat (`sensors.visuals`) from the drives (`turbo`
colormap) and the phases (`Greens`).

!!! bug "The drive is stepped twice"
    `AmphibiousDriveController.step()` calls `self.drive.step()`, then
    `super().step()`, which calls it again. Drives such as
    `OrientationFollower` are stateful (filters and a PID controller), so
    their response differs from a single step per control step. The Zbot
    experiments use `AmphibiousController`, which is not affected.

## KinematicsController

Selected by `AmphibiousOptions` when the `control` block has a
`kinematics_file` (`KinematicsControlOptions`): it replays recorded joint
kinematics instead of a CPG.

!!! note "Legacy scripts"
    `farms_amphibious/scripts/amphibious.py` and `run_control.py` import a
    `get_amphibious_controller` function that no longer exists.

## See also

- [NetworkODE](network-ode.md)
- [Joint Controllers](joint-controllers.md) and [Ekeberg Muscle Model](ekeberg-muscle.md)
- [Descending Drive](descending-drive.md)
- [Core Control](../core/core-control.md)
