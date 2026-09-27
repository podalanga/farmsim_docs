# farms_amphibious Reference

`farms_amphibious` provides locomotion control for amphibious animats: CPG
networks, descending drives, sensory feedback and joint equations, with
their options and data classes. Its control code does not depend on the
physics engine.

## Module map

```text
farms_amphibious/
├── model/
│   ├── options.py         # AmphibiousOptions and its sub-options, AmphibiousArenaOptions, DriveKind
│   └── convention.py      # AmphibiousConvention: names and indices of joints, oscillators, drives
├── control/
│   ├── amphibious.py      # JointMuscleController, AmphibiousController, AmphibiousDriveController
│   ├── network.py         # AnimatNetwork, NetworkODE
│   ├── drive.py           # DescendingDrive, OrientationFollower, PotentialMap classes
│   ├── kinematics.py      # KinematicsController (replays recorded kinematics)
│   ├── ode.pyx            # ode_oscillators_sparse: the CPG equations
│   ├── position_phase_cy.pyx, position_muscle_cy.pyx, ekeberg.pyx, passive_cy.pyx  # Joint equations
│   ├── muscle_cy.pyx, joints_control_cy.pyx  # Shared bases of the joint equations
│   └── manta_control.py   # Experiment specific code, not used by the other modules
├── data/
│   ├── data.py            # AmphibiousData, AmphibiousExperimentData
│   ├── data_cy.pyx        # Cython data, ConnectionType
│   └── network.py         # OscillatorNetworkState, DriveArray, Oscillators, connectivity classes
├── bullet/                # PyBullet engine (needs pybullet, not used with MuJoCo)
├── callbacks.py           # Imports a module that no longer exists (legacy)
├── analysis/, utils/, scripts/
```

## Pages

| Topic | Page |
|-------|------|
| Options (`control.network`, `control.muscles`, motors) | [Amphibious Options](amphibious-options.md), [Configure CPG Network Parameters](../../how-to/configure-cpg-network.md) |
| `AmphibiousController` | [Amphibious Controller](amphibious-controller.md) |
| CPG equations and data | [CPG Oscillators](cpg-oscillators.md), [NetworkODE](network-ode.md) |
| Joint equations | [Joint Controllers](joint-controllers.md), [Ekeberg Muscle Model](ekeberg-muscle.md) |
| Data classes | [Amphibious Data](amphibious-data.md) |
| Descending drives | [Descending Drive](descending-drive.md) |
| Full API | [Generated API reference](../api/farms_amphibious/index.md) |

## Using the CPG controller

The requirements, all met by `experiments/zbot_swimming`:

1. `loaders.animats_options`: `farms_amphibious.model.options.AmphibiousOptions`;
2. `loaders.animats_data`: `farms_amphibious.data.data.AmphibiousData`,
   which allocates the CPG state (with `farms_core.model.data.AnimatData`
   there is no state, and no network);
3. `control.network`, `control.muscles`, and a joint `equation` for each
   motor in the animat file;
4. `farms_amphibious.control.amphibious.AmphibiousController` in the
   animat's `extensions:`.

`AmphibiousController.from_options()` creates a `NetworkODE` (`dopri5`,
maximum step `physics.timestep`) and, when `control.network.drive_config`
names a YAML file, the descending drive of `drive_loader`. At every
environment step, it integrates the network and computes the joint
commands of each equation.

## Joint equations

| `equation` | Handler | Control types | Command |
|------------|---------|---------------|---------|
| `position_phase` | `PositionPhaseCy` | position | Position from the oscillator phase |
| `position_muscle` | `PositionMuscleCy` | position | $k(\tfrac{1}{2}(M_2 - M_1) + \phi_{off}) + b$ (the Zbot) |
| `ekeberg_muscle` | `EkebergMuscleCy` | velocity, torque | Ekeberg active torque, stiffness and damping set on the MuJoCo joint |
| `ekeberg_muscle_explicit` | `EkebergMuscleCy` | torque | Ekeberg torque, all terms explicit |
| `passive` | `PassiveJointCy` | velocity, torque | Passive spring-damper |

The equations are in
[Mathematical Models](../../explanation/mathematical-models.md#joint-equations).

## See also

- [CPG Control Architecture](../../explanation/cpg-architecture.md)
- [Configure CPG Network Parameters](../../how-to/configure-cpg-network.md)
