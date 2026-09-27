# Data Flow and Persistence

How data moves through a FARMS simulation, from the YAML files to the
pre-allocated arrays and the HDF5 output, and the reference of the data
classes. See [Save, Load, and Inspect Data](../how-to/save-load-data.md)
for practical usage.

## From the YAML files to the arrays

```text
YAML files
    │  ExperimentOptions.load() (classes of loaders:)
    ▼
ExperimentOptions
    ├── simulation: SimulationOptions
    ├── animats: list[AnimatOptions]     (AmphibiousOptions for the Zbot)
    └── arenas: list[ArenaOptions]
    │  ExperimentData.from_options()
    ▼
ExperimentData                           farms_core.experiment.data
    ├── times: (n_iterations,)
    ├── timestep: float                  physics.timestep [s]
    ├── simulation: SimulationData       farms_core.simulation.data
    │   ├── ncon: (n_iterations,)        number of contacts
    │   ├── niter: (n_iterations,)       solver iterations
    │   └── energy: (n_iterations, 2)    potential and kinetic energy
    └── animats: list[AnimatData]        farms_core.model.data
        ├── sensors: SensorsData         farms_core.sensors.data
        │   ├── links     (buffer_size, n_links, 20)
        │   ├── joints    (buffer_size, n_joints, 17)
        │   ├── contacts  (buffer_size, n_contacts, 12)
        │   ├── xfrc      (buffer_size, n_xfrc, 6)
        │   ├── muscles   (buffer_size, n_muscles, 17)
        │   ├── adhesions (buffer_size, n_adhesions, 1)
        │   ├── visuals   (buffer_size, n_visuals, 8)
        │   └── rays      (buffer_size, n_rays, 8)
        ├── state        (AmphibiousData: CPG phases, amplitudes, offsets)
        └── network      (AmphibiousData: drives and connectivity)
```

The number of elements of each sensor category is the length of the
corresponding list in `control.sensors` of the animat file. The arrays
hold `runtime.buffer_size` iterations (by default `runtime.n_iterations`).

## Pre-allocation

`ExperimentData.from_options()` allocates every array before the
simulation starts, with the data class of each animat
(`loaders.animats_data`). This means:

1. no allocation in the simulation loop, which only writes to existing
   arrays;
2. memory known at setup time;
3. a direct HDF5 export of the arrays;
4. index based access: `array[iteration % buffer_size, element, column]`.

## Per-step data flow

```text
1. MuJoCo state (sensors, xpos, xquat, qpos, ...)
       │  ExperimentTask.update_sensors() -> physics2data()
2. SensorsData arrays at index iteration % buffer_size
       │  extensions and controllers read them in before_step()
3. Forces and commands
       │  physics.data.xfrc_applied, physics.data.ctrl, ...
4. MuJoCo step
       │  ExperimentTask.after_step(), next iteration
5. ExperimentLogger.end_episode(): ExperimentData.to_file()
```

The sensors are copied from the MuJoCo sensors and arrays through maps
built at the start of the episode (see
[Physics to Sensor Mapping](../internals/physics-sensor-mapping.md)). On
the substeps of an iteration, only the links are updated (when an
extension runs on substeps). The `xfrc` array is written by the extensions
that apply forces, such as `SwimmingExtension`, not read from MuJoCo.

## Sensor arrays

Each category has an `.array` of shape `(buffer_size, n_elements, n_columns)`
and `.names`, the element names. The columns are the constants of
`farms_core/sensors/sensor_convention.pxd`, available in Python as
`farms_core.sensors.sensor_convention.sc` (for example
`sc.joint_position`). The arrays also have accessor methods, for example
`links.com_position(iteration, link_i)` or
`links.global_com_position(iteration)`.

| Category | Columns |
|----------|---------|
| `links` (20) | CoM position (0:3), CoM orientation quaternion x, y, z, w (3:7), link frame position (7:10), link frame orientation (10:14), CoM linear velocity (14:17), CoM angular velocity (17:20) |
| `joints` (17) | Position, velocity, torque, reaction force (3) and torque (3), position, velocity and torque commands, active, stiffness, damping and friction torques, limit force |
| `contacts` (12) | Reaction force (3), friction force (3), total force (3), position (3) |
| `xfrc` (6) | Force (3), torque (3), world frame, about the CoM |
| `muscles` (17) | Excitation, activation, tendon unit length, velocity and force, fiber length and velocity, pennation angle, force-length, force-velocity, active and passive forces, tendon length and force, Ia, II and Ib feedback |
| `adhesions` (1) | Adhesion force |
| `visuals` (8) | Colour RGBA (4), emission RGB and intensity (4) |
| `rays` (8) | Distance, origin (3), direction (3), hit |

See [Add and Configure Sensors](../how-to/configure-sensors.md) for how to
enable them.

## Units

The simulation may use scaled units (`simulation.units` in the simulation
file, `SimulationUnitScaling`). The sensor arrays are in SI units. MuJoCo
quantities are converted with `task.units`:

```python
sim_time = physics.time()/task.units.seconds
dt = physics.timestep()/task.units.seconds
physics.data.xfrc_applied[body, :3] = force*task.units.newtons
```

`SimulationUnitScaling` has `meters`, `seconds` and `kilograms`, and
derived factors such as `newtons`, `torques`, `velocity`,
`angular_velocity`, `stiffness`, `damping` and `inertia`.

## HDF5 files

`ExperimentLogger` calls `ExperimentData.to_file('<log_path>/simulation.hdf5', iteration)`
at the end of the simulation. The file mirrors `ExperimentData.to_dict()`
(lists are stored as groups whose name starts with `FARMSLIST`):

```text
times                     (n_iterations,)
timestep                  ()
simulation/ncon, niter, energy
FARMSLISTanimats/0/sensors/links/array     (buffer_size, n_links, 20)
FARMSLISTanimats/0/sensors/links/names
FARMSLISTanimats/0/sensors/links/masses
FARMSLISTanimats/0/sensors/joints/array ... (and the other categories)
FARMSLISTanimats/0/state                   (AmphibiousData only)
FARMSLISTanimats/0/network/...             (AmphibiousData only)
```

`ExperimentData.from_file()` reads it back, with `AnimatData` for the
animats.

## Options files

`ExperimentOptionsLogger` writes, at the start of the simulation, the
options as loaded (with the defaults filled in) to its `log_path`:

```text
Output/
├── simulation_options.yaml
├── animat_0_options.yaml
└── arena_0_options.yaml
```

They can be used to reproduce the simulation, and are never read back by
the simulation itself. `MjcfSaver` writes the MuJoCo model
(`simulation_mjcf.xml`).

## API reference

The generated API reference lists every method:
[`ExperimentData`](../reference/api/farms_core/experiment/data.md),
[`AnimatData`](../reference/api/farms_core/model/data.md),
[`SensorsData` and the sensor arrays](../reference/api/farms_core/sensors/data.md).

## See also

- [Save, Load, and Inspect Data](../how-to/save-load-data.md)
- [Add and Configure Sensors](../how-to/configure-sensors.md)
- [Simulation Lifecycle](simulation-lifecycle.md)
- [Data Allocation Internals](../internals/data-allocation-internals.md)
