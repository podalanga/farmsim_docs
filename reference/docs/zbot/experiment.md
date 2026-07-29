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
| `--log_path Output/` | `Output` | Where to write HDF5 and YAML snapshots |
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

| Parameter | Value | Notes |
|-----------|-------|-------|
| `solver: CG` | Conjugate Gradient | Faster but less accurate than Newton for stiff contacts |
| `integrator: implicitfast` | Implicit fast | MuJoCo's semi-implicit integrator — good for stiff joints |
| `cone: elliptic` | Elliptic friction cone | More realistic than pyramidal but costs more computation |

#### Simulation Extensions

Extensions are simulation-level hooks that run **globally** (not per-animat). They execute in the order listed:

| Extension | What it does |
|-----------|-------------|
| `ExperimentLogger` | Writes `simulation.hdf5` at end of run |
| `ExperimentOptionsLogger` | Writes YAML snapshots of all options used |
| `MjcfSaver` | Saves the generated MuJoCo XML for debugging |
| `CameraFollower` | Moves the viewer camera to follow `animat_id=0` |

---

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
| `viscosity: 1.0` | Used as drag multiplier `μ` | `F_drag = C_d × μ × v|v|` |
| `density: 1000.0` | Fresh water | Used for buoyancy: `F_buoy ∝ m × g / ρ_link` |

!!! tip "Adding a Water Current"
    Set `velocity: [0.2, 0, 0]` to add a 0.2 m/s current along X. The `SwimmingExtension` computes drag from **relative** velocity `v_link − v_water`, so the robot will need to swim against the current to stay in place.

---

## `animat_config.yaml` — The Robot Config

This is the largest and most important file. It is split into four logical sections: **spawn**, **morphology**, **control**, and **extensions**.

### 1. Spawn

```yaml
sdf: ../../models/zbot/sdf/zbot.sdf
spawn:
  mode: free
  pose: [0, 0, 0.01, 0, -1.5708, 3.14159]
  #       x  y   z  roll  pitch    yaw
```

The pose `[x, y, z, roll, pitch, yaw]` uses **radians**. See [Zbot Model → Spawn Pose](model.md#spawn-pose) for a full explanation of why these angles orient the robot correctly.

### 2. Morphology

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

### 3. Control — Sensors, Motors, and CPG Network

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

The `controller_loader` is the **dotted Python class path** FARMS will dynamically import and instantiate. This is the key hook for custom controllers — see [Custom CPG Controller](cpg_controller.md).

Sensor data is written to the `animat_data` object every step and is accessible in your controller via `self.animat_data.sensors.*`.

#### Motors

Each joint has a `motor` entry specifying how torque is computed. For the Zbot, all 6 joints share the same gains:

```yaml
motors:
  - joint_name: joint_1
    control_types: [position]    # Joint is position-controlled
    limits_torque: [-10.0, 10.0] # Clamp output torque to ±10 Nm
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

See [Mathematical Models](../mathematical_models.md) for the full derivation.

### 4. Animat-Level Extensions

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
joints_trq_cmd = joints.array[:, :, sc.joint_cmd_torque]    # Nm (commanded)
joints_trq_act = joints.array[:, :, sc.joint_torque_active] # Nm (active component)
joints_trq_stf = joints.array[:, :, sc.joint_torque_stiffness] # Nm (stiffness)
joints_trq_dmp = joints.array[:, :, sc.joint_torque_damping]   # Nm (damping)
joints_trq_frc = joints.array[:, :, sc.joint_torque_friction]  # Nm (friction)
```

| Plot | X-axis | Y-axis |
|------|--------|--------|
| Joint positions vs time | Time (s) | Position (rad) |
| Joint velocities vs time | Time (s) | Velocity (rad/s) |
| Torque commands vs time | Time (s) | Torque Nm |
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

- [Zbot Model](model.md) — SDF geometry and physical properties
- [Custom CPG Controller](cpg_controller.md) — replace the default controller
- [Configuration Reference](../configuration.md) — all YAML parameter definitions
- [Mathematical Models](../mathematical_models.md) — CPG and Ekeberg equations
