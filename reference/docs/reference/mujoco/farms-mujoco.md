# farms_mujoco

The MuJoCo engine of FARMS, built on dm_control: it converts the SDF
models and the options into a MuJoCo model, runs the simulation loop, and
provides the swimming forces and the viewer extensions.

```text
farms_mujoco/
├── simulation/
│   ├── simulation.py    # Simulation: model, dm_control environment, run loop
│   ├── task.py          # ExperimentTask: the dm_control Task
│   ├── mjcf.py          # SDF to MJCF conversion (setup_mjcf_xml)
│   ├── physics.py       # MuJoCo state to sensor arrays
│   ├── extensions.py    # MjcfSaver, CameraFollower, viewer extensions
│   └── application.py   # dm_control viewer application
├── sensors/
│   ├── sensors.pyx      # Sensor copy kernels
│   └── camera.py        # CameraRecording (video export)
├── swimming/            # Fluid forces, see farms_mujoco.swimming
└── viewer.py            # Targets2Reach viewer extension
```

## Simulation

**Source:** `farms_mujoco/simulation/simulation.py`

`Simulation.from_experiment(experiment_options)` builds the MuJoCo model
with `setup_mjcf_xml()`, then creates the `ExperimentTask` and the
dm_control `Environment` (with `n_sub_steps = physics.num_sub_steps`).
`run()` then runs the simulation:

- With the viewer (`runtime.headless: false`), it uses MuJoCo's passive
  viewer, or dm_control's viewer when `mujoco.viewer: dm_control`. Keys:
  Space (pause), Q (quit), + and - (speed), Right arrow (one iteration
  when paused).
- Headless, it runs `runtime.n_iterations` iterations, with a progress
  bar when `runtime.show_progress` is true.

Each iteration is `physics.cb_sub_steps` environment steps, each of
`physics.num_sub_steps` MuJoCo steps: the MuJoCo timestep is
`physics.timestep/(cb_sub_steps*num_sub_steps)`.

The data and options are saved by the `ExperimentLogger` and
`ExperimentOptionsLogger` extensions (see
[Save and Load Data](../../how-to/save-load-data.md)).

::: farms_mujoco.simulation.simulation.Simulation
    options:
      show_root_heading: false
      heading_level: 3
      members: [from_experiment, run, iterator, save_mjcf_xml]

## ExperimentTask

**Source:** `farms_mujoco/simulation/task.py`

The dm_control `Task` that connects FARMS to MuJoCo. When it is created,
it instantiates every extension with its `from_options()` class method:
first those of `simulation.extensions`, then those of each animat's
`extensions`.

| Method | When | What it does |
|---|---|---|
| `initialize_episode()` | Start of an episode | Builds the name to index maps and the sensor maps, collects the controllers (extensions that are `AnimatController` instances), resets to keyframe 0, calls `initialize_episode()` on every extension |
| `before_step()` | Every environment step | Updates the sensors (links only on the substeps), calls `before_step()` on the extensions (on the first substep of an iteration, or on every substep for extensions created with `substep=True`), and writes the controller commands to the actuators |
| `after_step()` | Every environment step | At the end of an iteration, increments `iteration` and calls `after_step()` on every extension |

Useful attributes for extensions:

| Attribute | Description |
|-----------|-------------|
| `iteration` | Current iteration (use `iteration % buffer_size` to index the sensor arrays) |
| `buffer_size` | Number of iterations held by the sensor arrays |
| `timestep` | Iteration duration [s] |
| `cb_sub_steps` | Environment steps per iteration |
| `physics_timestep` | MuJoCo timestep [s] |
| `units` | `SimulationUnitScaling`, conversion from simulation units |
| `data` | The `ExperimentData` |
| `extensions` | All the extensions |

See [ExperimentTask Internals](../../internals/experiment-task-internals.md)
for the details.

## MJCF builder

**Source:** `farms_mujoco/simulation/mjcf.py`

`setup_mjcf_xml()` converts the SDF files of the arenas and animats into
one MuJoCo model:

- bodies, joints and geoms from the SDF links, joints, visuals and
  collisions (visual geoms in group 1 without collisions, collision geoms
  in group 2)
- link options: friction of the collision geoms, and MuJoCo's own fluid
  model disabled (the fluid forces come from `farms_mujoco.swimming`)
- joint options: stiffness, damping and spring reference, added to the
  MuJoCo joint
- motors: position and velocity actuators with the motor gains
  (`[kp, kv of the position actuator, kv of the velocity actuator]`) and
  torque limits
- sensors: MuJoCo sensors for the links, joints and actuators
- contacts: the geoms of an animat do not collide with each other, except
  the pairs listed in `morphology.self_collisions`
- the options of the `mujoco` block of the simulation file (solver,
  integrator, ...), and the `physics` timestep and gravity

See [MJCF Builder Internals](../../internals/mjcf-builder-internals.md).

## Sensors

**Source:** `farms_mujoco/simulation/physics.py`, `farms_mujoco/sensors/sensors.pyx`

`get_physics2data_maps()` maps the MuJoCo sensors and arrays to the
`SensorsData` arrays of each animat, and `physics2data()` copies them at
each step. See [Physics to Sensor Mapping](../../internals/physics-sensor-mapping.md).

## Built-in extensions

Simulation extensions, listed in the `extensions:` of the simulation file
(`loader:` is the class path, `config:` the options below):

| Class | Config | Use |
|---|---|---|
| `farms_mujoco.simulation.extensions.MjcfSaver` | `path` (default `simulation_mjcf.xml`) | Saves the MuJoCo model |
| `farms_mujoco.simulation.extensions.CameraFollower` | `animat_id`, `distance`, `azimuth`, `elevation`, `angular_velocity` | Viewer camera following an animat |
| `farms_mujoco.simulation.extensions.CoMViewer` | `animat_id`, `size`, `rgba` | Sphere at the centre of mass |
| `farms_mujoco.simulation.extensions.TrailCoMViewer` | `animat_id`, `width`, `rgba`, `spacing` | Trail of the centre of mass |
| `farms_mujoco.simulation.extensions.TrailLinkViewer` | `animat_id`, `link`, `width`, `rgba`, `spacing` | Trail of a link |
| `farms_mujoco.simulation.extensions.ArrowViewer` | `animat_id`, `size`, `rgba`, `offset` | Arrow on the animat |
| `farms_mujoco.sensors.camera.CameraRecording` | See `CameraRecordingOptions` | Offscreen video recording |

Animat extensions, listed in the `extensions:` of an animat file:

| Class | Use |
|---|---|
| `farms_mujoco.swimming.extension.SwimmingExtension` | Fluid forces, see [farms_mujoco.swimming](mujoco-swimming.md) |

The generated [API reference](../api/farms_mujoco/index.md) lists every
option of these classes. See also
[Use Built-in Extensions](../../how-to/use-extensions.md).

## See Also

- [MuJoCo Simulation](mujoco-simulation.md)
- [farms_mujoco.swimming](mujoco-swimming.md)
- [Trace a Simulation Step](../../tutorials/simulation-workflow.md)
