# Swimming Experiment — YAML config walkthrough

This page is a complete walkthrough of the `experiments/zbot_swimming/` directory. Every key field in every config file is explained with its actual value and the effect it has on the simulation.

!!! note "Source Files"
    - `experiments/zbot_swimming/experiment_config.yaml` — Top-level experiment config
    - `experiments/zbot_swimming/simulation_config.yaml` — Physics and runtime settings
    - `experiments/zbot_swimming/animat_config.yaml` — Robot morphology, sensors, motors, CPG network
    - `experiments/zbot_swimming/arena_config.yaml` — Ground plane and water properties
    - `experiments/zbot_swimming/analysis.py` — Post-processing and plotting script

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

Optional CLI flags:

| Flag | Default | Description |
|------|---------|-------------|
| `--simulator MUJOCO` | `MUJOCO` | Physics backend |
| `--headless` | off | Disable MuJoCo viewer |
| `--log_path Output/` | *(empty — uses config `Output`)* | Where to write HDF5 and YAML snapshots |
| `--profile` | off | Print Python call graph to stdout |

!!! tip
    For cluster or batch runs, set `headless: true` inside `simulation_config.yaml` instead of passing the flag every time.

---

## `experiment_config.yaml` — The Manifest

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

### `loaders` — Class Injection

The `loaders` section tells `farms_sim` which Python class to instantiate for each section. This is what allows you to use `AmphibiousOptions` (which carries CPG and muscle fields) instead of the minimal `AnimatOptions`.

| Loader key | Class used | Why |
|------------|-----------|-----|
| `simulation_options` | `SimulationOptions` | Standard sim settings |
| `animats_options` | `AmphibiousOptions` | Adds CPG network + muscle fields |
| `arenas_options` | `AmphibiousArenaOptions` | Adds water physics fields |
| `experiment_data` | `AmphibiousExperimentData` | Container for all animat/arena data |
| `animats_data` | `AmphibiousData` | Per-animat data arrays (sensors, joints) |

!!! important "Custom Controllers Still Need AmphibiousOptions"
    Even if you write your own controller, keep `AmphibiousOptions` in the loaders as long as you use the CPG network section in `animat_config.yaml`. Only switch to `AnimatOptions` if you remove the `network:` section entirely.

---

## `simulation_config.yaml` — Physics & Logging

```yaml
# experiments/zbot_swimming/simulation_config.yaml
units:
  meters: 1
  seconds: 1
  kilograms: 1

runtime:
  n_iterations: 5001      # Total physics steps
  buffer_size: 5001       # Sensor ring-buffer size (must be >= n_iterations)
  play: true              # Start unpaused in the viewer
  rtl: 1.0                # Real-time limiter (1.0 = real-time, 0 = free-run)
  fast: false             # Skip real-time limiter entirely
  headless: false         # Set true to run without the MuJoCo GUI
  show_progress: true     # Print progress bar to stdout

physics:
  timestep: 0.002         # Physics integration step = 2 ms
  gravity: [0, 0, -9.81]  # Standard gravity (m/s²)
  num_sub_steps: 1        # MuJoCo sub-steps per control step
  cb_sub_steps: 2         # Controller callbacks per env.step() call
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
      skip: 0              # Log every step (skip=1 would log every other step)
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
| `timestep` | 0.002 s | Each physics step is 2 ms |
| `n_iterations` | 5001 | Total simulation = 5001 × 0.002 = **~10 seconds** |
| `cb_sub_steps` | 2 | Controller runs **twice per `env.step()`** call → 1 kHz effective controller rate |
| `buffer_size` | 5001 | Must be **≥ n_iterations** or old sensor data will be overwritten |

!!! warning "Buffer Size"
    If `buffer_size` < `n_iterations`, the sensor ring buffer wraps around and early data is lost. Always keep `buffer_size >= n_iterations`.

#### Physics Solver

| Parameter | Value | Description |
|-----------|-------|-------------|
| `solver: CG` | Conjugate Gradient | Faster but less accurate than Newton for stiff contacts |
| `integrator: implicitfast` | Implicit fast | MuJoCo's semi-implicit integrator — good for stiff joints |
| `cone: elliptic` | Elliptic friction cone | More realistic than pyramidal but costs more computation |

#### Simulation Extensions — What's Wired Up by Default

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
`extensions:` list without touching any Python — verified against
`farms_mujoco/farms_mujoco/simulation/extensions.py` and
`farms_mujoco/farms_mujoco/sensors/camera.py`. Everything below is written
against the Zbot's real link names (`Head`, `Segment1`–`Segment6`,
`TailSegment`) so it can be copy-pasted straight into
`experiments/zbot_swimming/simulation_config.yaml`.

##### `CameraRecording` — offscreen video export

Unlike `CameraFollower` (which only moves the *interactive* viewer camera
and does nothing headless), `CameraRecording` drives its own independent
`mujoco.MjvCamera` + offscreen `mujoco.Renderer`, captures a frame every
`before_step()`, and encodes an `.mp4` (via `cv2`, H.264 with an `mp4v`
fallback) or `.html` (via matplotlib) at `end_episode()`. This is what you
want for batch/headless rendering or for a video that survives without an
open GUI window:

```yaml
# simulation_config.yaml — add alongside the existing extensions
extensions:
  # ... ExperimentLogger, ExperimentOptionsLogger, MjcfSaver, CameraFollower ...
  - loader: farms_mujoco.sensors.camera.CameraRecording
    config:
      path: Output/video          # '.mp4' or '.html' is appended from the writer, keep this extension-free
      animat_id: 0                # tracks Zbot's global CoM every frame; null = fixed camera at `offset`
      fps: 30
      speed: 1.0                  # >1 = sped-up video; internally rescales capture cadence, not just metadata
      azimuth: -30
      elevation: -15
      distance: 2
      angular_velocity: 0         # deg/s, set non-zero for a continuously orbiting shot
      offset: [0, 0, 0.0]
      resolution: [1280, 720]
```

| Field | Zbot-relevant notes |
|-------|---------------------|
| `path` | Give it **without** an extension — `Output/video`, not `Output/video.mp4`. The writer backend is chosen from whichever extension you'd normally append (`.mp4` → `cv2`/ffmpeg, `.html` → matplotlib), and that extension is appended internally from `os.path.splitext` on the *target* filename, so writing `path: Output/video.mp4` in the config is harmless but redundant — the code re-derives the extension either way. |
| `animat_id` | Same `AmphibiousData.animats[0]` index used everywhere else in the Zbot configs. Leave at `0` — there is only one animat in both `zbot_swimming` and `zbot_bout_glide`. |
| `speed` | Recomputes `skips` (physics steps between captured frames) from `speed/(timestep*fps)`, so at the Zbot's `physics.timestep: 0.002`, `speed: 1.0`, `fps: 30` → a frame is captured roughly every 16–17 physics steps, not every step. |
| `distance`/`azimuth`/`elevation` | Independent of `CameraFollower`'s equivalent fields above — the two cameras don't share state, so tune them separately if you run both extensions together (harmless; one drives the live viewer, the other drives the offscreen renderer). |
| `geomgroups` | Not shown above (defaults to `[1, 1, 0, 1, 0, 0]` in the extension, `[0, 1, 0, 1, 0, 0]` in `CameraRecordingOptions` — pass it explicitly if you need a specific group visible/hidden). Controls which MuJoCo geom groups (collision vs. visual meshes, etc.) are rendered into the video, independent of the interactive viewer's own display settings. |

!!! bug "Don't pass a `camera` id with the default viewer"
    Supplying a MJCF-embedded `camera` id in `config` crashes with
    `AttributeError` on the first frame under `viewer: MuJoCo` (the setting
    both Zbot experiments use). Leave `camera` unset — the extension
    defaults to a free-floating `mjCAMERA_FREE` camera driven by
    `distance`/`azimuth`/`elevation`/`offset` instead.

!!! note "Requires `cv2` for `.mp4`"
    If `cv2` isn't importable in your environment, `CameraRecording` falls
    back to a matplotlib writer for every format, which buffers the entire
    episode's frames in memory before encoding — fine for the Zbot's
    default ~10 s run, but budget accordingly for the 100 001-step
    `zbot_bout_glide` config if you enable this there.

##### Marker/trail viewer extensions — cheap visual debugging

These draw non-physical debug geometry directly into the interactive
viewer's scratch scene (`viewer.user_scn`) — no mass, no collision, not
part of the compiled MJCF, and **not visible in `CameraRecording`'s
offscreen renders** (which render the real physics model, not the viewer's
scratch buffer). All four require an open interactive window
(`headless: false`), matching the Zbot's default `simulation_config.yaml`.

```yaml
extensions:
  - loader: farms_mujoco.simulation.extensions.CoMViewer
    config:
      animat_id: 0
      size: [0.01, 0.0, 0.0]     # auto-scaled from total link mass if left at this default
      rgba: [1.0, 1.0, 1.0, 0.3]
  - loader: farms_mujoco.simulation.extensions.TrailCoMViewer
    config:
      animat_id: 0
      width: 5
      rgba: [1.0, 0.3, 0.0, 0.7]
      spacing: 10                 # draw a new trail segment every 10 iterations
  - loader: farms_mujoco.simulation.extensions.TrailLinkViewer
    config:
      animat_id: 0
      link: TailSegment           # must be one of the Zbot's real SDF link names
      width: 5
      rgba: [1.0, 0.3, 0.0, 0.7]
      spacing: 10
  - loader: farms_mujoco.simulation.extensions.ArrowViewer
    config:
      animat_id: 0
      size: [0.03, 0.03, 0.3]
      rgba: [1.0, 1.0, 1.0, 0.3]
      offset: null                 # auto-derived from mass if omitted
```

| Extension | Use it to... |
|-----------|--------------|
| `CoMViewer` | Watch the whole-robot center of mass as a floating sphere — good for spotting drift or an unexpectedly off-axis CoM. |
| `TrailCoMViewer` | Leave a breadcrumb trail of the CoM path over the run — useful for eyeballing swim-path curvature or drift from a straight line. |
| `TrailLinkViewer` | Same trail, but for one specific link (e.g. `TailSegment` to see the tail-tip trajectory that actually produces thrust). **Requires** the named link to already be present as a `control.sensors.links` entry — it asserts against `animat_data.sensors.links.names` at `initialize_episode()` and raises if the link isn't sensed. |
| `ArrowViewer` | A rotating pointer above the CoM — useful as a generic orientation/heading indicator while iterating on a controller, though by default it just spins at a fixed rate and isn't wired to any real torque/force value. |

!!! warning "Scratch-geom buffer has a fixed capacity"
    `TrailCoMViewer`/`TrailLinkViewer` never delete old segments. Over the
    `zbot_bout_glide` config's 100 001-iteration run, a `spacing` of `10`
    would draw 10 000 line segments into a buffer with a fixed maximum
    size — increase `spacing` for long runs, or expect the trail to stop
    drawing (or raise) once the limit is hit.

##### Sensors You Can Add

`control.sensors` in `animat_config.yaml` currently populates four of the
seven available categories (`links`, `joints`, `xfrc`, plus empty
`contacts`/`muscles`/`adhesions`/`visuals`). The three left empty are not
missing configuration — they are simply unused by the default rigid-joint,
free-swimming Zbot — but each is available immediately if your experiment
needs it, verified against `SensorsOptions` in `farms_core/model/options.py`
and the `sc` column layout in `sensor_convention.pyx`:

```yaml
control:
  sensors:
    links: [Head, Segment1, Segment2, Segment3, Segment4, Segment5, Segment6, TailSegment]
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    xfrc: [Head, Segment1, Segment2, Segment3, Segment4, Segment5, Segment6, TailSegment]

    # Currently empty in zbot_swimming/zbot_bout_glide — populate as needed:
    contacts: [Head, TailSegment]        # per-link contact reaction totals ...
    # contacts: [[Head, TailSegment]]    # ... or restrict to contact between a specific pair
    muscles: []    # only relevant if you add `hill_muscles` — the Zbot uses Ekeberg `position_muscle`, not Hill muscles
    adhesions: []  # suction/gripping actuators — not applicable to a fully submerged swimmer
    visuals: []    # non-physical visual-only markers, separate from the debug viewer extensions above
```

| Category | When to turn it on for the Zbot | Array shape |
|----------|----------------------------------|-------------|
| `contacts` | You want to detect the robot touching the arena floor/walls (`ground_height: -1` in `arena_config.yaml`) — e.g. to end an episode early or penalize a controller for grounding out. Add the relevant link names (or `[link_a, link_b]` pairs for a specific contact) here first, since nothing populates this list automatically for a swimming morphology. | `(n_iters, n_contacts, 12)` |
| `muscles` | Only if you replace the Ekeberg `position_muscle` motors with Hill-type muscles (`control.hill_muscles`, currently `[]` in both Zbot configs) — the sensor array tracks activation, tendon/fiber length and velocity, force, and spindle feedback fields. | `(n_iters, n_muscles, 17)` |
| `adhesions` | Not meaningful for a fully submerged swimmer with no adhesion actuators configured; included here for completeness since it's part of the shared `AmphibiousControlOptions` schema used by walking/climbing FARMS models too. | `(n_iters, n_adhesions, 1)` |
| `visuals` | Track programmatic color/emission changes on links (e.g. a controller that flashes a segment a different color on contact) — distinct from, and independent of, the marker/trail viewer extensions above, which draw separate scratch geometry rather than recolor the animat's own MJCF materials. | `(n_iters, n_visuals, 8)` |

See [Add and Configure Sensors](../how-to/configure-sensors.md) for the full
column-by-column layout of every category and how to read the resulting
arrays out of `AnimatData.sensors` in a controller or `analysis.py`, and
[Use Built-in Extensions](../how-to/use-extensions.md) for the complete
extension reference (including `SwimmingExtension`, which lives in
`animat_config.yaml` rather than `simulation_config.yaml`).

## `arena_config.yaml` — World and Water

```yaml
# experiments/zbot_swimming/arena_config.yaml
sdf: ../../models/arena_flat_v0/sdf/arena_flat.sdf  # Flat ground plane

spawn:
  loader: 0
  mode: free
  pose: [0, 0, 0, 0, 0, 0]       # Arena at world origin
  velocity: [0, 0, 0, 0, 0, 0]

water:
  sdf: ../../models/arena_water_v0/sdf/arena_water.sdf
  drag: true            # Enable hydrodynamic drag forces
  buoyancy: true        # Enable buoyancy forces
  height: 0             # Z-coordinate of water surface (m)
  velocity: [0, 0, 0]   # Water current vector [Vx, Vy, Vz] m/s
  viscosity: 1.0        # Dynamic viscosity (Pa·s) — used as drag multiplier
  density: 1000.0       # Fluid density (kg/m³)

ground_height: -1       # Z-coordinate of the ground plane (m)
```

### Water Properties

| Field | Value | Effect |
|-------|-------|--------|
| `height: 0` | Water surface at z=0 | Robot spawned at z=0.01 is immediately submerged |
| `velocity: [0,0,0]` | Still water | No current; all relative velocity is robot motion |
| `viscosity: 1.0` | Used as drag multiplier `μ` | `$F_{\text{drag}} = C_d \cdot \mu \cdot v|v|$` |
| `density: 1000.0` | Fresh water | Used for buoyancy: `$F_{\text{buoy}} \propto m \cdot g / \rho_{\text{link}}$` |

!!! tip "Adding a Water Current"
    Set `velocity: [0.2, 0, 0]` to add a 0.2 m/s current along X. The `SwimmingExtension` computes drag from **relative** velocity `v_link - v_water`, so the robot will need to swim against the current to stay in place.

---

## `animat_config.yaml` — The Robot Config

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
      fluid_interaction: true   # This link participates in drag/buoyancy
      density: 950.0            # kg/m³ — overrides SDF inertia to achieve this density
      drag_coefficients:
        - [-4.0, -4.0, -0.1]   # Translational [Cx, Cy, Cz] — negative = resistive
        - [0, 0, 0]             # Rotational [Croll, Cpitch, Cyaw]
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
      limits: [[-inf, inf], [-inf, inf]]
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

The Zbot config sets several `LinkOptions`/`AmphibiousLinkOptions` and
`MorphologyOptions` fields to their default/inactive value, so the excerpt
above omits them for readability. They are still present in
`animat_config.yaml` for every link, and are worth understanding before
tuning contact or mass properties — verified against
`farms_core/model/options.py::LinkOptions` and
`farms_amphibious/model/options.py::AmphibiousLinkOptions`:

| Field | Zbot value | Class | Meaning |
|-------|-----------|-------|---------|
| `solref` | `null` | `LinkOptions.solref` | MuJoCo contact solver reference `[timeconst, dampratio]`. `null` → MuJoCo's own default is used. Set this per-link to make a specific contact softer/stiffer than the rest of the model. |
| `solimp` | `null` | `LinkOptions.solimp` | MuJoCo contact solver impedance parameters. `null` → MuJoCo default. Rarely needed unless you see contact penetration or excessive bounce on a specific link. |
| `mass_multiplier` | `1` | `AmphibiousLinkOptions.mass_multiplier` (Zbot links only; not on plain `LinkOptions`) | Scales the link's SDF-derived mass by this factor **after** the `density` override is applied. `1` = no change. Useful for quick "what if this segment were heavier" sweeps without editing the SDF or re-deriving `density`. |
| `sites` | *(omitted, defaults to `[]`)* | `LinkOptions.sites` | Named attachment points on the link (e.g. for sensors or visual markers). Zbot does not use any. |
| `self_collisions` | `[]` | `MorphologyOptions.self_collisions` | List of `[link_a, link_b]` name pairs that are allowed to collide with each other. Empty means the SDF's own default adjacency exclusion applies — no segment-vs-neighbour self-collision is added on top of it. Populate this if you need e.g. the head to be able to collide with the tail during a tight turn. |
| `tendons` | *(omitted, defaults to `[]`)* | `MorphologyOptions.tendons` | List of `TendonOptions` for spatial tendons (e.g. antagonist cable actuation). Not used by the default rigid-joint Zbot. |

!!! note "`density` vs `mass_multiplier`"
    `density` (950 kg/m³ for every Zbot link) is used to **recompute** the
    link's mass and inertia from its collision geometry, overriding
    whatever mass the SDF originally specified — this is how the Zbot is
    made slightly buoyant (950 < water's 1000 kg/m³, see
    [Water Properties](#water-properties)). `mass_multiplier` is applied
    **on top of** that already-recomputed mass. Changing `density` changes
    both mass *and* how strongly the link floats; changing
    `mass_multiplier` changes mass only, density-derived buoyancy math
    stays the same.

### Control — Sensors, Motors, and CPG Network

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

The `controller_loader` is the **dotted Python class path** FARMS will dynamically import and instantiate. This is the key hook for custom controllers — see [Custom CPG Controller](zbot-custom-controller.md).

Sensor data is written to the `animat_data` object every step and is accessible in your controller via `self.animat_data.sensors.*`.

#### Motors

Each joint has a `motor` entry specifying how torque is computed. For the Zbot, all 6 joints share the same gains:

```yaml
motors:
  - joint_name: joint_1
    control_types: [position]    # Joint is position-controlled
    limits_torque: [-10.0, 10.0] # Clamp output torque to ±10 N·m
    gains: [3.0, 0.01, 0]        # [Kp, Kd, Ki] for PD servo
    equation: position_muscle    # CPG drives a Ekeberg muscle that outputs a target position
    transform:
      gain: 1
      bias: 0
    offsets:
      gain: 0.05                 # Scale factor for joint offset modulation
      bias: 0
      low: 1                     # Drive activation threshold
      high: 5
      saturation_low: 0
      saturation_high: 0
      rate: 3                    # Convergence rate for offset (1/s)
    passive:
      is_passive: false
  # ... joint_2 through joint_6 identical ...
```

#### CPG Network

The CPG section defines the full neural oscillator network. The Zbot uses **12 oscillators** — one Left/Right pair per joint — that are coupled to produce a swimming travelling wave.

##### Drives

Drives are the **descending input signals** that set the target frequency and amplitude for each oscillator. `initial_value: 4` puts the oscillator in the active swimming range.

```yaml
drives:
  - name: drive_body_0_L
    initial_value: 4        # Drive level (dimensionless). Range: ~1–5
    kind: spine_left        # Input to the left-side oscillator chain
  - name: drive_body_0_R
    initial_value: 4
    kind: spine_right
  # ... drive_body_1_L/R through drive_body_5_L/R ...
```

Drive level `4` is the default "swim fast" setting. A lower value (e.g., `2`) produces slower, lower-amplitude oscillations.

##### Oscillators

Each oscillator computes a phase `θ` and amplitude `A` via the CPG ODEs. The key parameters:

```yaml
oscillators:
  - name: osc_body_0_L
    initial_phase: 1.0489          # Starting phase (rad) — pre-tuned for stable gait
    initial_amplitude: 0.0         # Amplitude ramps up from 0 via the ODE
    frequency_gain: 1.5708         # ω = frequency_gain × drive_level (rad/s)
                                   # At drive=4: ω = 1.5708×4 = 6.28 rad/s → 1 Hz
    frequency_bias: 0.0
    frequency_low: 1               # Drive level threshold where frequency activates
    frequency_high: 5
    amplitude_gain: 0.15           # A_nom = amplitude_gain × drive_level
                                   # At drive=4: A_nom = 0.15×4 = 0.6 rad
    amplitude_bias: 0.0
    amplitude_low: 0.9
    amplitude_high: 5
    amplitude_saturation_high: 0.75  # Maximum amplitude is capped at 0.75 rad
    rate: 3.0                      # Amplitude convergence rate (1/s)
  # ... osc_body_0_R through osc_body_5_L/R ...
```

!!! note "How frequency_gain maps to swimming frequency"
    `ω = frequency_gain × drive_level`
    → At `drive=4` and `frequency_gain=1.5708 (≈ π/2)`:
    → `ω = 6.28 rad/s` → **f = 1.0 Hz**

    To swim at 2 Hz, set `frequency_gain: 3.1416 (≈ π)`.

##### Oscillator-to-Oscillator Couplings (`osc2osc`)

Couplings enforce **phase relationships** between oscillators. The Zbot uses two types:

```yaml
osc2osc:
  # Left-Right anti-phase (undulation)
  - in: osc_body_0_R
    out: osc_body_0_L
    type: OSC2OSC
    weight: 30.0
    phase_bias: 3.14159      # π rad = 180° → anti-phase (undulation)
  - in: osc_body_0_L
    out: osc_body_0_R
    weight: 30.0
    phase_bias: 3.14159      # Bidirectional coupling

  # Front-to-Back travelling wave (60° lag per segment)
  - in: osc_body_1_L
    out: osc_body_0_L
    weight: 30.0
    phase_bias: 1.0472       # π/3 rad = 60° → travelling wave head-to-tail
  - in: osc_body_0_L
    out: osc_body_1_L
    weight: 30.0
    phase_bias: 5.2360       # 5π/3 rad = 300° (reverse direction coupling)
  # ... same pattern for joints 2–5 ...
```

| Coupling type | Phase bias | Effect |
|---------------|-----------|--------|
| L ↔ R (same segment) | π (180°) | Anti-phase → lateral undulation |
| N → N+1 (L chain) | π/3 (60°) | 60° lag front-to-back → travelling wave |

##### Drive-to-Oscillator Mapping (`drive2osc`)

Each drive signal connects to its corresponding oscillator:

```yaml
drive2osc:
  - drive: drive_body_0_L
    oscillator: osc_body_0_L
  - drive: drive_body_0_R
    oscillator: osc_body_0_R
  # ... through drive_body_5_R ...
```

##### Muscles (`muscles`)

Each joint maps to a Left+Right oscillator pair via the **Ekeberg muscle model**:

```yaml
muscles:
  - joint_name: joint_1
    osc1: osc_body_0_L      # Left (flexor) oscillator
    osc2: osc_body_0_R      # Right (extensor) oscillator
    alpha: 0.5              # Active torque gain
    beta: 1.0               # Active + passive stiffness coefficient
    gamma: 0.1              # Passive stiffness ratio (relative to beta)
    delta: 0.001            # Viscous damping coefficient
    epsilon: 0              # Coulomb friction (disabled)
  # ... joint_2 through joint_6 ...
```

The Ekeberg torque equation:

$$
\tau = \underbrace{\alpha (A_L \sin\theta_L - A_R \sin\theta_R)}_{\text{active}}
    + \underbrace{\beta (A_L \sin\theta_L + A_R \sin\theta_R)(\phi_{off} - \phi)}_{\text{active stiffness}}
    + \underbrace{\gamma \beta (\phi_{off} - \phi)}_{\text{passive stiffness}}
    - \underbrace{\delta \dot\phi}_{\text{damping}}
$$

See [Mathematical Models](../explanation/mathematical-models.md) for the full derivation.

### Control Fields Not Shown Above

Three more `control:` fields sit alongside `muscles:` and are set to empty
lists in the Zbot config. They belong to `AmphibiousControlOptions`
(`farms_amphibious/model/options.py`) and only matter once you move beyond
rigid position-controlled joints:

| Field | Zbot value | Meaning |
|-------|-----------|---------|
| `hill_muscles` | `[]` | Hill-type muscle models (force-length-velocity actuation) as an alternative to the Ekeberg spring-damper model used here. Leave empty unless you are replacing `equation: position_muscle` with a biomechanical Hill muscle per joint. |
| `adhesions` | `[]` | `AmphibiousAdhesionsOptions` — per-link adhesion (suction/gripping) actuators, used by legged/climbing amphibious models. Not applicable to a fully submerged swimmer. |
| `visuals` | `[]` | `AmphibiousVisualsOptions` — extra non-physical visual-only geometry (e.g. debug markers) attached to links. |

Leaving these as empty lists is the normal, expected state for a pure
swimming experiment — they exist because `AmphibiousOptions` is shared
across every animat morphology in FARMS (walking, climbing, swimming), and
the Zbot only exercises the subset relevant to undulatory locomotion.

### Animat-Level Extensions

```yaml
extensions:
  - loader: farms_amphibious.control.amphibious.AmphibiousController
    config: {}
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null    # Inherits from arena_config.yaml water section
```

!!! warning "Extension Order Matters"
    Extensions execute **in the order listed**. `AmphibiousController` is first, so the CPG computes joint torques **before** `SwimmingExtension` applies hydrodynamic forces. If you swap the order, fluid forces will lag by one timestep.

---

## From YAML to Physics: What Actually Consumes These Options

The tables above describe *what* each field does in isolation. This section
traces *where in the codebase* each Zbot-relevant YAML block is actually
read, so you know which source file to open when a parameter doesn't behave
the way its description implies.

| YAML block | Consumed by | What happens |
|------------|-------------|--------------|
| `morphology.links[*].density`, `.drag_coefficients`, `.fluid_interaction` | `farms_mujoco/swimming/hydrodynamics.pyx`, `swimming/buoyancy.py` (via `SwimmingHandler`) | At episode start, `SwimmingHandler.__init__` builds a per-link `PrimitiveCache` (bounding sphere + collision primitives) from the compiled MJCF geometry. Every control step, `compute_link_forces` reads `drag_coefficients` to compute quadratic drag and `density`/`mass_multiplier` (via the link's mass) to compute buoyancy, then rotates the combined force/torque from the link's URDF frame into world frame via `urdf2global` before it is written into `animat_data.sensors.xfrc`. |
| `arena.water.*` | `farms_amphibious.model.options.AmphibiousArenaOptions` → `farms_mujoco.swimming.extension.SwimmingExtension` | `density`/`viscosity`/`velocity`/`height` are wrapped into a `WaterProperties` object once at setup. If `animat_config.yaml`'s `extensions: [...SwimmingExtension...] config: {water_properties: null}` is left `null` (as in the default Zbot config), the extension falls back to the arena's own water block instead of a per-animat override. |
| `control.motors[*].equation: position_muscle` | `farms_amphibious/control/position_muscle_cy.pyx` (`PositionMuscleCy`) | Selected per-joint by `AmphibiousController` at construction time — this is the Cython class that actually integrates the Ekeberg torque equation shown below into a target joint position/torque every controller sub-step (`cb_sub_steps` times per physics step). |
| `control.network.oscillators[*]`, `osc2osc`, `drive2osc` | `farms_amphibious/control/network.py` → `farms_amphibious/control/ode.pyx` (`NetworkODE`) | Assembled once into a single coupled ODE system and integrated with `scipy`'s `dopri5` (Dormand-Prince, adaptive-step Runge-Kutta) once per controller sub-step. The YAML values become the initial state vector and the weight/connectivity matrices — see [`internals/ode-internals.md`](../internals/ode-internals.md) for the full per-function walkthrough. |
| `control.muscles[*]` (`alpha`…`epsilon`) | `farms_amphibious/control/ekeberg.pyx` | Read once at controller construction into a `EkebergMuscleCy` parameter struct per joint; not re-read from YAML during the run, so changing these values requires restarting the simulation, not editing the running process. |
| `morphology.joints[*].limits/stiffness/damping/springref` | MJCF builder (`farms_mujoco/simulation/mjcf.py::mjc_add_link`) | Written directly into the compiled `<joint>` element's `range`, `stiffness`, `springref`, and `damping` XML attributes at MJCF-build time — these become MuJoCo's own passive joint physics, layered underneath (not instead of) the active `position_muscle` torque from the controller. |

!!! warning "A known implementation gotcha: `bound_radii` and multi-body stability"
    `SwimmingHandler.__init__` computes each link's bounding-sphere radius
    (`bound_radii`) once from the compiled collision geometry, used as the
    fast-path check before falling back to the exact mesh/analytic
    submerged-volume computation. Halving this value incorrectly — e.g. by
    passing a diameter where a radius is expected — silently shrinks every
    link's fast-path buoyancy/drag footprint without raising an error, and
    only shows up as growing numerical instability (NaN velocities) once
    several segments interact hydrodynamically at once. If you see NaNs
    appear only in multi-body swimming (not in a single-link sanity check),
    audit `SwimmingHandler.__init__` and the `cob_options.py` method
    selection (`cob_method: ramp | analytic | mesh`) before suspecting the
    CPG or the MJCF solver settings — see
    [`internals/hydrodynamics-internals.md`](../internals/hydrodynamics-internals.md)
    for the confirmed frame-rotation bug in the same code path.

!!! tip "Where to look when a YAML change has no visible effect"
    A common source of confusion: `control.muscles[*]` and
    `control.network.oscillators[*]` are only read **once**, at controller
    construction, from the YAML file passed on the command line — not from
    `Output/animat_0_options.yaml`. If you are iterating on gains, edit
    `animat_config.yaml` directly and re-run; editing the `Output/` snapshot
    does nothing (it is write-only, produced by `ExperimentOptionsLogger`
    for reproducibility, never read back in).

## `analysis.py` — Post-Processing

After the simulation, run:

```bash
cd /app/experiments/zbot_swimming
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
joints_trq_cmd = joints.array[:, :, sc.joint_cmd_torque]    # N·m (commanded)
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
| `Output/simulation.hdf5` | Full telemetry at every logged step |
| `Output/simulation_options.yaml` | Snapshot of `SimulationOptions` for reproducibility |
| `Output/animat_0_options.yaml` | Snapshot of `AmphibiousOptions` (CPG params, etc.) |
| `Output/arena_0_options.yaml` | Snapshot of `AmphibiousArenaOptions` |
| `Output/simulation_mjcf.xml` | Generated MuJoCo MJCF XML (useful for debugging geometry) |

---

## See Also

- [Zbot Model](zbot-model.md) — SDF geometry and physical properties
- [Custom CPG Controller](zbot-custom-controller.md) — replace the default controller
- [Configuration Reference](../reference/env/yaml-schema.md) — all YAML parameter definitions
- [Mathematical Models](../explanation/mathematical-models.md) — CPG and Ekeberg equations
