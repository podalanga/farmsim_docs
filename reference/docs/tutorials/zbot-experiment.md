# Swimming Experiment: YAML config walkthrough

This page is a complete walkthrough of the `experiments/zbot_swimming/` directory. Every key field in every config file is explained with its actual value and the effect it has on the simulation.

!!! note "Source Files"
    - `experiments/zbot_swimming/experiment_config.yaml`: Top-level experiment config
    - `experiments/zbot_swimming/simulation_config.yaml`: Physics and runtime settings
    - `experiments/zbot_swimming/animat_config.yaml`: Robot morphology, sensors, motors, CPG network
    - `experiments/zbot_swimming/arena_config.yaml`: Ground plane and water properties
    - `experiments/zbot_swimming/analysis.py`: Post-processing and plotting script

---

## Directory Structure

```
experiments/
└── zbot_swimming/
    ├── experiment_config.yaml   ← Top-level manifest (pass this to farmsim)
    ├── simulation_config.yaml   ← Physics engine and logging settings
    ├── animat_config.yaml       ← Robot morphology, sensors, motors, CPG network
    ├── arena_config.yaml        ← World ground plane and water properties
    ├── analysis.py              ← Post-processing script (plots from HDF5)
    └── Output/                  ← Generated at runtime
        ├── simulation.hdf5
        ├── simulation_mjcf.xml
        ├── simulation_options.yaml
        ├── animat_0_options.yaml
        └── arena_0_options.yaml
```

---

## Running the Experiment

Inside the Docker container:

```bash
cd /app/experiments/zbot_swimming
farmsim --experiment_config experiment_config.yaml
```

The other command line options are listed in the [CLI reference](../reference/env/cli.md). There is no flag to disable the viewer: set `runtime.headless: true` in `simulation_config.yaml` for cluster or batch runs.

---

## `experiment_config.yaml`: The Manifest

This is the **only file** you pass to `farmsim`. It points to all other configs and declares which Python classes deserialise them.

```yaml
# experiments/zbot_swimming/experiment_config.yaml
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
  experiment_data: farms_amphibious.data.data.AmphibiousExperimentData
  animats_data:
    - farms_amphibious.data.data.AmphibiousData
```

### `loaders`: Class Injection

The `loaders` section tells `farms_sim` which Python class to instantiate for each section. This is what allows you to use `AmphibiousOptions` (which carries CPG and muscle fields) instead of the minimal `AnimatOptions`.

| Loader key | Class used | Why |
|------------|-----------|-----|
| `simulation_options` | `SimulationOptions` | Standard sim settings |
| `animats_options` | `AmphibiousOptions` | Adds CPG network + muscle fields |
| `arenas_options` | `AmphibiousArenaOptions` | Adds water physics fields |
| `experiment_data` | `AmphibiousExperimentData` | Container for all animat/arena data |
| `animats_data` | `AmphibiousData` | Per-animat data arrays (sensors, joints) |

!!! note "Custom Controllers Still Need AmphibiousOptions"
    Even if you write your own controller, keep `AmphibiousOptions` in the loaders as long as you use the CPG network section in `animat_config.yaml`. Only switch to `AnimatOptions` if you remove the `network:` section entirely.

---

## `simulation_config.yaml`: Physics & Logging

```yaml
# experiments/zbot_swimming/simulation_config.yaml
units:
  meters: 1
  seconds: 1
  kilograms: 1

runtime:
  n_iterations: 5001      # Number of iterations (logging/control steps)
  buffer_size: 5001       # Iterations kept in the sensor buffers
  play: true              # Start unpaused in the viewer
  rtl: 1.0                # Viewer speed relative to real time (2.0 = twice as fast)
  fast: false             # Viewer runs up to 256 times faster than real time
  headless: false         # Set true to run without the viewer (never time-limited)
  show_progress: true     # Progress bar in headless mode

physics:
  timestep: 0.002         # Iteration period = 2 ms
  gravity: [0, 0, -9.81]  # Standard gravity (m/s²)
  num_sub_steps: 1        # MuJoCo steps per environment step
  cb_sub_steps: 2         # Environment steps (extension callbacks) per iteration
  n_solver_iters: 1000    # Constraint solver iteration limit

mujoco:
  solver: CG              # Constraint solver: CG (faster) or Newton (more accurate)
  integrator: implicitfast # Integration scheme
  cone: elliptic          # Friction cone model
  impratio: 1
  ccd_iterations: 1000
  ccd_tolerance: 1.0e-06
  noslip_iterations: 1000
  noslip_tolerance: 1.0e-06
  viewer: MuJoCo
  texture_repeat: 1
  shadow_size: 1024
  visual_scale: 1.0
  extent: 100.0

extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: Output
      skip: 0              # Stored but currently unused
  - loader: farms_core.simulation.extensions.ExperimentOptionsLogger
    config:
      log_path: Output
  - loader: farms_mujoco.simulation.extensions.MjcfSaver
    config:
      path: Output/simulation_mjcf.xml
  - loader: farms_mujoco.simulation.extensions.CameraFollower
    config:
      animat_id: 0
      distance: 2
      azimuth: 30
      elevation: -20
      angular_velocity: 0
```

### Key Parameters Explained

#### Timing

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `timestep` | 0.002 s | One iteration (sensor logging, non-substep extensions) every 2 ms |
| `n_iterations` | 5001 | Total simulation = 5001 × 0.002 = **~10 seconds** |
| `cb_sub_steps` | 2 | Two environment steps per iteration: MuJoCo steps of 0.002/2 = 1 ms, and substep extensions (controller, swimming) run at 1 kHz |
| `num_sub_steps` | 1 | One MuJoCo step per environment step |
| `buffer_size` | 5001 | Iterations kept in memory |

!!! warning "Buffer size"
    Sensor data is stored at index `iteration % buffer_size`. If `buffer_size < n_iterations`, the buffer wraps around and early data is overwritten, so keep `buffer_size >= n_iterations` unless you only need recent data.

#### Physics Solver

| Parameter | Value | Description |
|-----------|-------|-------------|
| `solver: CG` | Conjugate Gradient | Faster but less accurate than Newton for stiff contacts |
| `integrator: implicitfast` | Implicit fast | MuJoCo's semi-implicit integrator, good for stiff joints |
| `cone: elliptic` | Elliptic friction cone | More realistic than pyramidal but costs more computation |

#### Simulation Extensions Wired Up by Default

Extensions are simulation-level hooks that run **globally** (not per-animat). They execute in the order listed:

| Extension | What it does |
|-----------|-------------|
| `ExperimentLogger` | Writes `simulation.hdf5` at end of run |
| `ExperimentOptionsLogger` | Writes YAML snapshots of all options used |
| `MjcfSaver` | Saves the generated MuJoCo XML for debugging |
| `CameraFollower` | Moves the viewer camera to follow `animat_id=0` |

#### Simulation Extensions You Can Add

The four extensions above are only the ones the default `zbot_swimming` and
`zbot_bout_glide` configs happen to enable. FARMS ships several more
`TaskExtension`s that slot into the same `simulation_config.yaml`
`extensions:` list without touching any Python (verified against
`farms_mujoco/farms_mujoco/simulation/extensions.py` and
`farms_mujoco/farms_mujoco/sensors/camera.py`). Everything below is written
against the Zbot's real link names (`Head`, `Segment1` to `Segment6`,
`TailSegment`) so it can be copy-pasted straight into
`experiments/zbot_swimming/simulation_config.yaml`.

##### `CameraRecording`: offscreen video export

Unlike `CameraFollower`, which only moves the interactive viewer camera,
`CameraRecording` (`farms_mujoco.sensors.camera.CameraRecording`) renders
offscreen with its own camera and `mujoco.Renderer`, so it also works in
headless runs. Frames are captured in `before_step()`, and the video is
written at `end_episode()`:

```yaml
# simulation_config.yaml, added to the existing extensions
extensions:
  - loader: farms_mujoco.sensors.camera.CameraRecording
    config:
      path: Output/video.mp4      # .mp4 (OpenCV/ffmpeg) or .html (matplotlib)
      animat_id: 0                # Camera follows the animat CoM; null for a fixed camera at offset
      fps: 30
      speed: 1.0                  # Playback speed relative to real time
      azimuth: -30
      elevation: -15
      distance: 2
      angular_velocity: 0         # [deg/s], non-zero for an orbiting shot
      offset: [0, 0, 0.0]
      resolution: [1280, 720]
```

| Field | Notes |
|-------|-------|
| `path` | The extension selects the writer: `.mp4` uses OpenCV when available (H.264, falling back to `mp4v`), `.html` uses matplotlib. Other extensions are written with ffmpeg after a warning. |
| `animat_id` | Index of the animat to follow (the zbot experiments have one animat, `0`). |
| `fps`, `speed` | A frame is captured every `int(speed/(timestep*fps))` iterations: with `physics.timestep: 0.002`, `fps: 30` and `speed: 1.0`, every 16 iterations. |
| `geomgroups` | MuJoCo geom groups rendered (default `[0, 1, 0, 1, 0, 0]`: visuals and group 3). |

!!! warning "Memory"
    All frames are kept in memory for the whole episode (an array of
    `n_iterations/(skips+1)` frames of `resolution`). For the 100 001-iteration
    `zbot_bout_glide` configuration at 1280×720 that is about 17 GB, so reduce
    the resolution, `fps` or the run length when recording long runs.

!!! note
    `motion_filter` appears in the option documentation but is not accepted by
    `CameraRecordingOptions`, and leaving `camera` unset (a free camera
    driven by `distance`/`azimuth`/`elevation`/`offset`) is the tested path.

##### Visual debugging extensions

These draw non-physical markers (no mass, no collision, not part of the
MJCF): in the interactive viewer, and also in `CameraRecording` videos.

```yaml
extensions:
  - loader: farms_mujoco.simulation.extensions.CoMViewer
    config:
      animat_id: 0
      size: [0.01, 0.0, 0.0]
      rgba: [1.0, 1.0, 1.0, 0.3]
  - loader: farms_mujoco.simulation.extensions.TrailCoMViewer
    config:
      animat_id: 0
      width: 5
      rgba: [1.0, 0.3, 0.0, 0.7]  # A new trail segment every 10 iterations
  - loader: farms_mujoco.simulation.extensions.TrailLinkViewer
    config:
      animat_id: 0
      link: TailSegment           # Must be a sensed link (control.sensors.links)
      width: 5
      rgba: [1.0, 0.3, 0.0, 0.7]
  - loader: farms_mujoco.simulation.extensions.ArrowViewer
    config:
      animat_id: 0
      size: [0.03, 0.03, 0.3]
      rgba: [1.0, 1.0, 1.0, 0.3]
      offset: null
```

| Extension | Use it to... |
|-----------|--------------|
| `CoMViewer` | Show the whole-robot centre of mass as a sphere. |
| `TrailCoMViewer` | Draw the path of the centre of mass. |
| `TrailLinkViewer` | Draw the path of one link, for example the tail. The link must be listed in `control.sensors.links`. |
| `ArrowViewer` | Draw an arrow above the centre of mass (an orientation indicator). |

!!! warning "Marker capacity"
    Trails are never cleared, and MuJoCo scenes hold a limited number of
    geoms. The trail `width`, the spacing (10 iterations) and
    `show_on_camera` are not read from YAML (see
    [Use Built-in Extensions](../how-to/use-extensions.md#viewer-markers)).

##### Sensors You Can Add

`control.sensors` lists the elements recorded by each of the eight sensor
categories of `SensorsData` (`links`, `joints`, `contacts`, `xfrc`,
`muscles`, `adhesions`, `visuals` and `rays`). The Zbot configurations only
fill `links`, `joints` and `xfrc`. The other categories are available when an
experiment needs them:

```yaml
control:
  sensors:
    links: [Head, Segment1, Segment2, Segment3, Segment4, Segment5, Segment6, TailSegment]
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    xfrc: [Head, Segment1, Segment2, Segment3, Segment4, Segment5, Segment6, TailSegment]
    contacts: [Head, TailSegment]  # Contacts of these links with anything
    muscles: []    # Hill muscles (control.hill_muscles), unused by the Zbot
    adhesions: []  # Adhesion actuators, unused by the Zbot
    visuals: []    # Link colours
```

Each category is stored as an array of shape
`(n_iterations, n_elements, n_columns)`, with the column counts of
`farms_core/sensors/sensor_convention.pxd`:

| Category | Columns | When to use it with the Zbot |
|---|---|---|
| `links` | 20 | Positions, orientations and velocities of the links |
| `joints` | 17 | Joint positions, velocities, torques and commands |
| `contacts` | 12 | Detect the robot touching the ground (`ground_height` in `arena_config.yaml`) |
| `xfrc` | 6 | External forces and torques, including the fluid forces |
| `muscles` | 17 | Only with Hill-type muscles (`control.hill_muscles`). The Zbot motors use the `position_muscle` equation, which is not a muscle model |
| `adhesions` | 1 | Not relevant for a swimmer |
| `visuals` | 8 | Colour and emission of the link visuals |
| `rays` | 8 | Ray casting range sensors (distance, origin, direction) |

See [Add and Configure Sensors](../how-to/configure-sensors.md) for the full
column-by-column layout of every category and how to read the resulting
arrays out of `AnimatData.sensors` in a controller or `analysis.py`, and
[Use Built-in Extensions](../how-to/use-extensions.md) for the complete
extension reference (including `SwimmingExtension`, which lives in
`animat_config.yaml` rather than `simulation_config.yaml`).

## `arena_config.yaml`: World and Water

```yaml
# experiments/zbot_swimming/arena_config.yaml (shortened)
sdf: ../../models/arena_flat_v0/sdf/arena_flat.sdf  # Flat ground plane

spawn:
  loader: 0
  mode: free
  pose: [0, 0, 0, 0, 0, 0]       # Arena at world origin
  velocity: [0, 0, 0, 0, 0, 0]

water:
  sdf: ../../models/arena_water_v0/sdf/arena_water.sdf  # Visual only
  drag: true            # Enable drag forces
  buoyancy: true        # Enable buoyancy forces
  height: 0             # Z coordinate of the water surface [m]
  velocity: [0, 0, 0]   # Water current [m/s]
  viscosity: 1.0        # Scale of the drag forces
  density: 1000.0       # Water density [kg/m^3]
  maps: ['', '']        # Optional water surface and velocity maps
  cob_method: analytical  # Exact centre of buoyancy (alias of `exact`)

ground_height: -1       # Z coordinate of the ground [m]
```

### Water Properties

| Field | Value | Effect |
|-------|-------|--------|
| `height: 0` | Water surface at z=0 | The robot, spawned at z=0.01, starts at the surface |
| `velocity: [0,0,0]` | Still water | Drag uses the velocity of the link relative to the water |
| `viscosity: 1.0` | Drag scale `$\mu$` | Per-axis drag `$F_i = \mu\, c_i\, v_i |v_i|$` in the link frame, with the `drag_coefficients` `$c_i$` of the link |
| `density: 1000.0` | Fresh water | Buoyancy `$F = -\rho V_{\text{sub}} g$`, applied at the centre of buoyancy |
| `cob_method: analytical` | Exact method | `$V_{\text{sub}}$` and the centre of buoyancy are computed from the collision geoms at every step (see [Swimming reference](../reference/mujoco/mujoco-swimming.md)) |

The other fluid keys (`cob_method`, `fluid_model`, `added_mass`, ...) are
listed in the [Configuration reference](../reference/env/configuration-reference.md#fluid-model-options).

!!! tip "Adding a water current"
    Set `velocity: [0.2, 0, 0]` for a 0.2 m/s current along X. The drag is
    computed from the relative velocity `v_link - v_water`, so the robot
    has to swim against the current to stay in place.

---

## `animat_config.yaml`: The Robot Config

This is the largest and most important file. It is split into four logical sections: **spawn**, **morphology**, **control**, and **extensions**.

### Spawn

```yaml
sdf: ../../models/zbot/sdf/zbot.sdf
spawn:
  mode: free
  pose: [0, 0, 0.01, 0, -1.5708, 3.14159]
  #       x  y   z  roll  pitch    yaw
```

The pose `[x, y, z, roll, pitch, yaw]` uses **radians**. See [Zbot Model → Spawn Pose](zbot-model.md#spawn-pose) for a full explanation of why these angles orient the robot correctly.

### Morphology

```yaml
morphology:
  links:
    - name: Head
      collisions: true
      fluid_interaction: true   # This link has fluid forces
      density: 950.0            # [kg/m^3] only used by cob_method: ramp
      drag_coefficients:
        - [-4.0, -4.0, -0.1]   # Linear drag [cx, cy, cz] in the link frame, negative opposes motion
        - [0, 0, 0]             # Rotational drag
    # ... Segment1 through Segment6 identical to Head ...
    - name: TailSegment
      fluid_interaction: true
      density: 950.0
      drag_coefficients:
        - [-10.0, -10.0, -0.1] # Higher drag → more thrust from tail undulation
        - [0, 0, 0]

  joints:
    - name: joint_1
      initial: [0, 0]           # [initial_position (rad), initial_velocity (rad/s)]
      limits: [[-inf, inf], [-inf, inf]]  # Not applied: MuJoCo uses the SDF joint limits
      stiffness: 0
      springref: 0
      damping: 0
    # ... joint_2 through joint_6 identical ...

  n_joints_body: 6
  n_dof_legs: 0
  n_legs: 0
  n_joints_passive: 0
```

### Morphology Fields Not Shown Above

The Zbot configuration also sets these fields, omitted above for
readability (`farms_core.model.options.LinkOptions`,
`farms_amphibious.model.options.AmphibiousLinkOptions` and
`farms_core.model.options.MorphologyOptions`):

| Field | Zbot value | Meaning |
|-------|-----------|---------|
| `friction` | `[0, 0, 0]` | MuJoCo friction (sliding, torsional, rolling) of the link's collision geoms |
| `solref`, `solimp` | `null` | Parsed, but not applied per link by the MuJoCo builder: the contact parameters come from the simulation file (`mujoco` options) |
| `mass_multiplier` | `1` | Only used by the PyBullet engine. It has no effect in MuJoCo |
| `extras` | `restitution`, `linearDamping`, `angularDamping` | PyBullet parameters, unused in MuJoCo |
| `self_collisions` | `[]` | Pairs `[link_a, link_b]` that collide with each other. Links of the same animat do not collide otherwise |

!!! note "Mass and density"
    The masses and inertias come from the SDF file. The link `density` is
    not used to recompute them: it is only used by `cob_method: ramp`, to
    estimate the link volume as `mass/density`. With the exact method, the
    volume comes from the collision geoms, so the robot floats if its mass
    is below `$\rho V$` of its geoms. See the buoyancy note of
    [The Zbot Model](zbot-model.md).

### Control: Sensors, Motors, and CPG Network

#### Sensors

```yaml
control:
  controller_loader: farms_amphibious.control.amphibious.AmphibiousController
  sensors:
    links: [Head, Segment1, Segment2, Segment3, Segment4, Segment5, Segment6, TailSegment]
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    xfrc: [Head, Segment1, Segment2, Segment3, Segment4, Segment5, Segment6, TailSegment]
    contacts: []
    muscles: []
```

!!! note "`controller_loader` is not used"
    `controller_loader` is parsed but not used to create the controller.
    The controller is the `AmphibiousController` listed in the animat's
    `extensions:` (see [Animat-level extensions](#animat-level-extensions)).

The sensor arrays are filled at every iteration and are available to
extensions and controllers as `animat_data.sensors` (see
[Sensors You Can Add](#sensors-you-can-add)).

#### Motors

Each actuated joint has a motor. All six Zbot motors are identical:

```yaml
motors:
  - joint_name: joint_1
    control_types: [position]    # Driven by the controller's positions()
    limits_torque: [-10.0, 10.0] # Torque limits [N.m]
    gains: [3.0, 0.01, 0]        # [kp, kv of the position actuator, kv of the velocity actuator]
    equation: position_muscle    # Joint command computed from the CPG outputs
    transform:
      gain: 1
      bias: 0
    offsets:                     # Joint offset set by the left/right drive difference
      gain: 0.05
      bias: 0
      low: 1
      high: 5
      saturation_low: 0
      saturation_high: 0
      rate: 3                    # Convergence rate of the offset [1/s]
    passive:
      is_passive: false
  # ... joint_2 through joint_6 identical ...
```

With `equation: position_muscle`, the position command of a joint is

$$
q_{\text{cmd}} = g_t \left(\tfrac{1}{2}(x_R - x_L) + q_{\text{off}}\right) + b_t,
\qquad x = A\,(1 + \cos\theta)
$$

where $x_L$ and $x_R$ are the outputs of the two oscillators of the joint
(`osc1` and `osc2` in `muscles:`), $g_t$ and $b_t$ are `transform.gain`
and `transform.bias`, and $q_{\text{off}}$ is the offset, which converges
at `rate` towards `offsets.gain*(drive_R - drive_L) + offsets.bias`. The
MuJoCo position actuator then tracks $q_{\text{cmd}}$ with the gains.

#### CPG Network

`control.network` describes the oscillator network: 12 oscillators (a left
and a right one for each of the 6 joints), 14 drives and their couplings.

##### Drives

Drives are the descending inputs that set the frequency and amplitude of
the oscillators:

```yaml
drives:
  - name: drive_brain_L
    initial_value: 4
  - name: drive_brain_R
    initial_value: 4
  - name: drive_body_0_L
    initial_value: 4
  - name: drive_body_0_R
    initial_value: 4
  # ... drive_body_1_L/R through drive_body_5_L/R ...
```

`drive_loader` and `drive_config` are empty in the Zbot configuration, so
the drives keep their initial values.

##### Oscillators

Each oscillator has a phase $\theta$ and an amplitude $A$. Its intrinsic
frequency and nominal amplitude depend on its drive $d$ (see `drive2osc`):

```yaml
oscillators:
  - name: osc_body_0_L
    initial_phase: 1.0489
    initial_amplitude: 0.0
    frequency_gain: 1.5708         # [rad/s per drive unit]
    frequency_bias: 0.0
    frequency_low: 1
    frequency_high: 5
    frequency_saturation_low: 0
    frequency_saturation_high: 0
    amplitude_gain: 0.15
    amplitude_bias: 0.0
    amplitude_low: 0.9
    amplitude_high: 5
    amplitude_saturation_low: 0
    amplitude_saturation_high: 0.75
    rate: 3.0                      # Amplitude convergence rate [1/s]
  # ... 11 more oscillators ...
```

Every drive dependent parameter follows the same rule
(`DriveDependentArrayCy.c_value`):

$$
p(d) =
\begin{cases}
\text{gain}\cdot d + \text{bias} & \text{low} \le d \le \text{high} \\
\text{saturation\_low} & d < \text{low} \\
\text{saturation\_high} & d > \text{high}
\end{cases}
$$

With $d = 4$: $\omega = 1.5708 \times 4 = 2\pi$ rad/s, so the oscillators
run at 1 Hz, and the nominal amplitude is $0.15 \times 4 = 0.6$. Note that
a drive above `frequency_high` gives a frequency of
`frequency_saturation_high` (0 here), which stops the oscillator. To swim
at 2 Hz with the same drive, set `frequency_gain: 3.1416`.

The oscillator equations (`farms_amphibious/control/ode.pyx`) are

$$
\dot\theta_i = \omega_i + \sum_j A_j w_{ij} \sin(\theta_j - \theta_i - \varphi_{ij}),
\qquad
\dot A_i = a_i (R_i - A_i)
$$

with $R_i$ the nominal amplitude and $a_i$ the `rate`.

##### Oscillator-to-Oscillator Couplings (`osc2osc`)

In a coupling, `in` is the oscillator that receives the coupling term
($i$ above) and `out` is the oscillator it is coupled to ($j$), so that at
steady state $\theta_{\text{out}} - \theta_{\text{in}} = \varphi$
(`phase_bias`). The Zbot has 32 couplings, all of weight 30:

```yaml
osc2osc:
  # Left and right of the same joint: anti-phase
  - in: osc_body_0_R
    out: osc_body_0_L
    type: OSC2OSC
    weight: 30.0
    phase_bias: 3.14159
  - in: osc_body_0_L
    out: osc_body_0_R
    type: OSC2OSC
    weight: 30.0
    phase_bias: 3.14159
  # Neighbouring joints of the same side
  - in: osc_body_1_L
    out: osc_body_0_L
    type: OSC2OSC
    weight: 30.0
    phase_bias: 1.0472       # osc_body_0_L leads osc_body_1_L by pi/3
  - in: osc_body_0_L
    out: osc_body_1_L
    type: OSC2OSC
    weight: 30.0
    phase_bias: 5.2360       # 5pi/3 = -pi/3, consistent with the coupling above
  # ... same pattern for the right side and the other joints ...
```

| Coupling | Phase bias | Effect |
|---|---|---|
| Left and right of a joint | $\pi$ | Anti-phase: the joint bends left and right |
| Joint $n$ to joint $n+1$ | $\pi/3$ | Each joint lags the previous one by 60°: a wave travelling from head to tail |

##### Drive-to-Oscillator and Drive-to-Joint Mapping

```yaml
drive2osc:       # Drive of each oscillator
  - drive: drive_body_0_L
    oscillator: osc_body_0_L
  - drive: drive_body_0_R
    oscillator: osc_body_0_R
  # ... through drive_body_5_R ...
drive2joint:     # Drives setting the offset of each joint
  - drive0: drive_body_0_L
    drive1: drive_body_0_R
    joint: joint_1
  # ... through joint_6 ...
```

The two `drive_brain` drives are not connected to any oscillator.

##### Muscles (`muscles`)

`muscles` pairs each joint with its left (`osc1`) and right (`osc2`)
oscillators, which the `position_muscle` equation reads:

```yaml
muscles:
  - joint_name: joint_1
    osc1: osc_body_0_L
    osc2: osc_body_0_R
    alpha: 0.5
    beta: 1.0
    gamma: 0.1
    delta: 0.001
    epsilon: 0
  # ... joint_2 through joint_6 ...
```

The `alpha` to `epsilon` coefficients are those of the Ekeberg muscle
model. They are only used by the `ekeberg_muscle` and
`ekeberg_muscle_explicit` equations, not by `position_muscle`. See
[Mathematical Models](../explanation/mathematical-models.md) for the
Ekeberg model.

### Control Fields Not Shown Above

| Field | Zbot value | Meaning |
|-------|-----------|---------|
| `hill_muscles` | `[]` | Hill-type muscles (MuJoCo muscle actuators) |
| `adhesions` | `[]` | Adhesion actuators, used by climbing models |
| `visuals` | `[]` | Links whose colour is controlled |

They are empty because `AmphibiousOptions` is shared by all the
amphibious models (walking, climbing, swimming).

### Animat-Level Extensions

```yaml
extensions:
  - loader: farms_amphibious.control.amphibious.AmphibiousController
    config: {}
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null    # Use the water block of arena_config.yaml
```

Extensions run in the order of the list. Both run before the MuJoCo step:
the controller sets the actuator commands and the swimming extension
writes the fluid forces to `xfrc_applied`, from the state at the start of
the step, so their order does not change the result here.

The animat file also sets `show_xfrc` and `scale_xfrc` (display of the
external forces in the viewer) and a `mujoco` block of MuJoCo specific
options.

---

## From YAML to Physics: What Consumes These Options

Where each Zbot option is read, to know which source file to open when a
parameter does not behave as expected:

| YAML block | Read by | What happens |
|------------|---------|--------------|
| `arena.water.*` | `farms_mujoco.swimming.extension.SwimmingExtension` | With `water_properties: null`, the extension uses the arena's `water` block. `density`, `viscosity`, `velocity` and `height` become a `WaterProperties` object, and the fluid keys (`cob_method`, ...) a `FluidOptions` |
| `morphology.links[*].fluid_interaction`, `drag_coefficients`, `density` | `farms_mujoco/swimming/hydrodynamics.pyx` (`SwimmingHandler`) | At the start of the episode, the handler builds the centre of buoyancy model of each link from its MuJoCo collision geoms. At every environment step it computes the buoyancy (at the centre of buoyancy) and the drag (from `drag_coefficients`), and writes the wrench to `xfrc_applied` and to the `xfrc` sensors. `density` is only used by `cob_method: ramp` |
| `control.motors[*].equation: position_muscle` | `farms_amphibious/control/position_muscle_cy.pyx` (`PositionMuscleCy`) | Chosen by `AmphibiousController` when it is created. Computes the position command of each joint from the oscillator outputs (see [Motors](#motors)) |
| `control.network.*` | `farms_amphibious/control/network.py` and `farms_amphibious/control/ode.pyx` | Assembled into one ODE system, integrated with SciPy's `dopri5` (adaptive Runge-Kutta) at every environment step. See [ODE internals](../internals/ode-internals.md) |
| `control.muscles[*]` | `farms_amphibious/control/amphibious.py` | Pairs each joint with its two oscillators. The `alpha` to `epsilon` coefficients are only used with the Ekeberg equations (`farms_amphibious/control/ekeberg.pyx`) |
| `morphology.joints[*].stiffness`, `damping`, `springref` | MJCF builder (`farms_mujoco/simulation/mjcf.py`) | Added to the MuJoCo `<joint>` attributes: passive joint dynamics on top of the actuators. The joint limits come from the SDF file |

!!! tip "Where to look when a YAML change has no visible effect"
    The options are read once, when the simulation is created, from the
    files given on the command line. The files written to `Output/`
    (`animat_0_options.yaml`, ...) are snapshots for reproducibility and are
    never read back: edit `animat_config.yaml` and run again.

## `analysis.py`: Post-Processing

After the simulation, run:

```bash
cd experiments/zbot_swimming
python analysis.py
```

The script loads `Output/simulation.hdf5` and generates the following plots:

### Time-Series Plots

```python
# analysis.py (excerpt)
from farms_core.experiment.data import ExperimentData
from farms_core.sensors.sensor_convention import sc

data = ExperimentData.from_file('Output/simulation.hdf5')
joints = data.animats[0].sensors.joints

# Available sensor channels (via sensor convention sc):
joints_pos     = joints.array[:, :, sc.joint_position]      # rad
joints_vel     = joints.array[:, :, sc.joint_velocity]      # rad/s
joints_trq_cmd = joints.array[:, :, sc.joint_cmd_torque]    # N·m (torque command)
joints_trq_act = joints.array[:, :, sc.joint_torque_active] # N·m (active component)
joints_trq_stf = joints.array[:, :, sc.joint_torque_stiffness] # N·m (stiffness)
joints_trq_dmp = joints.array[:, :, sc.joint_torque_damping]   # N·m (damping)
joints_trq_frc = joints.array[:, :, sc.joint_torque_friction]  # N·m (friction)
```

| Plot | X-axis | Y-axis |
|------|--------|--------|
| Joint positions vs time | Time (s) | Position (rad) |
| Joint velocities vs time | Time (s) | Velocity (rad/s) |
| Torque commands vs time | Time (s) | Torque N·m |
| Phase portrait (pos-vel) | Position (rad) | Velocity (rad/s) |
| Torque components | Position (rad) | Active/stiffness/damping torques |

---

## Output Files

| File | Contents |
|------|---------|
| `Output/simulation.hdf5` | Sensor data of every iteration (`ExperimentData`) |
| `Output/simulation_options.yaml` | Snapshot of `SimulationOptions` for reproducibility |
| `Output/animat_0_options.yaml` | Snapshot of `AmphibiousOptions` (CPG params, etc.) |
| `Output/arena_0_options.yaml` | Snapshot of `AmphibiousArenaOptions` |
| `Output/simulation_mjcf.xml` | Generated MuJoCo MJCF XML (useful for debugging geometry) |

---

## See Also

- [Zbot Model](zbot-model.md): SDF geometry and physical properties
- [Custom CPG Controller](zbot-custom-controller.md): replace the default controller
- [Configuration Reference](../reference/env/configuration-reference.md): all YAML options (generated)
- [Mathematical Models](../explanation/mathematical-models.md): CPG and Ekeberg equations
