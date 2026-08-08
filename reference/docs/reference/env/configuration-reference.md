
# Configuration Reference

This document enumerates the tunable parameters (Options classes) across the FARMS framework, specifically checking `farms_core` and `farms_amphibious` models, simulations, and experiments.

## `farms_core.model.options`

### `LinkOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `name` | - | - | SDF Builder | Link identifier |
| `collisions` | - | - | Physics Engine | Toggles collision geometry |
| `friction` | - | - | Physics Engine | Lateral, spinning, rolling friction |
| `density` | 1000 | kg/m³ | Fluid interaction | Computes buoyancy / mass |
| `drag_coefficients` | [0,0,0,0,0,0] | - | Fluid interaction | Computes hydrodynamic drag. In practice, a 2×3 nested list `[[tx,ty,tz],[rx,ry,rz]]` is used. |
| `fluid_interaction` | False | - | Fluid interaction | Toggles buoyancy/drag calculations |

### `JointOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `name` | - | - | SDF Builder | Joint identifier |
| `initial` | - | rad, rad/s | Simulator init | Initial pos and vel state |
| `limits` | - | rad | Simulator limits | Range of motion limits |
| `stiffness` | - | Nm/rad | Physics Engine | Joint spring stiffness |
| `damping` | - | N·m·s/rad | Physics Engine | Joint friction/damping |
| `springref` | - | rad | Physics Engine | Spring equilibrium position |

### `SpawnOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `loader` | FARMS | - | Simulator loader | Physics engine loading method |
| `mode` | FREE | - | Simulator loader | Base constraints (fixed, rotx, etc.) |
| `pose` | - | m, rad | Simulator init | Spawn position and orientation |
| `velocity` | - | m/s, rad/s | Simulator init | Spawn linear/angular velocity |

### `MotorOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `joint_name` | - | - | Controller | Target joint to actuate |
| `control_types` | - | - | Controller | Actuation mode (position/velocity/torque) |
| `limits_torque` | - | N·m | Controller | Max/min torque allowed |
| `gains` | - | - | Controller | `[Kp, Kd]` gains for position control (per the source `ClassDoc`); proceed with caution when using for velocity/torque control |

!!! note "`equation` is not a base `MotorOptions` field"
    The `equation` field (actuation control law, e.g. `position`, `position_muscle`, `ekeberg_muscle`, `passive`) belongs to the **`AmphibiousMotorOptions`** subclass in `farms_amphibious/model/options.py`, not to the base `MotorOptions` in `farms_core/model/options.py`.

### `SensorsOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `links` | [] | - | Logger/Controller | Tracks specific link states |
| `joints` | [] | - | Logger/Controller | Tracks specific joint states |
| `contacts` | [] | - | Logger/Controller | Tracks contact forces |
| `xfrc` | [] | - | Logger/Controller | Tracks external forces |
| `muscles` | [] | - | Logger/Controller | Tracks muscle states |
| `adhesions` | [] | - | Logger/Controller | Tracks adhesion states |
| `visuals` | [] | - | Logger/Controller | Tracks visual states |

### `WaterOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `sdf` | - | - | SDF Builder | Path to the water/arena SDF file |
| `drag` | - | - | Fluid interaction | Enables drag forces |
| `buoyancy` | - | - | Fluid interaction | Enables buoyancy forces |
| `height` | - | m | Fluid interaction | Surface level of water |
| `velocity` | - | m/s | Fluid interaction | Flow speed and direction (3-element constant vector, or a longer list encoding range + spatial bounds for map-based flow) |
| `viscosity` | - | Pa·s | Fluid interaction | Fluid friction |
| `density` | - | kg/m³ | Fluid interaction | Fluid density |
| `maps` | - | - | Fluid interaction | List of PNG flow-field map file paths (used when `velocity` is not a 3-element constant) |

### `MuscleOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `name` | - | - | Muscle Model | Muscle name identifier |
| `model` | - | - | Muscle Model | Muscle model type (e.g. `hill`, `ekeberg`, `mujoco`, `brown`, `rigidtendon`) |
| `max_force` | - | N | Muscle Model | Maximum contractile force |
| `optimal_fiber` | - | m | Muscle Model | Optimal muscle length |
| `tendon_slack` | - | m | Muscle Model | Tendon resting length |
| `max_velocity` | - | m/s | Muscle Model | Max contraction velocity |
| `pennation_angle` | - | rad | Muscle Model | Muscle fiber angle |
| `lmtu_min` | - | m | Muscle Model | Minimum muscle-tendon unit length |
| `lmtu_max` | - | m | Muscle Model | Maximum muscle-tendon unit length |
| `waypoints` | - | - | Muscle Model | Muscle routing waypoints |

!!! note "Additional optional `MuscleOptions` fields"
    `MuscleOptions` in `farms_core/model/options.py` also accepts many optional fields with defaults, including `act_tconst` (0.001), `deact_tconst` (0.001), `lmin`, `lmax`, `init_activation` (0.0), `init_fiber`, and type I/Ib/II afferent constants. The table above lists only the required fields.

---

## `farms_core.simulation.options`

### `RuntimeSimulationOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `n_iterations` | 1000 | - | Simulation Loop | Total steps to run |
| `buffer_size` | n_iterations | - | Data Logger | Logging buffer limit (falls back to `n_iterations` if 0) |
| `play` | True | - | GUI | Starts unpaused |
| `rtl` | 1.0 | - | Timer | Real-time limiter |
| `fast` | False | - | Simulation Loop | Runs headless without sleep |
| `headless` | False | - | GUI/Renderer | Disables rendering |
| `show_progress` | True | - | GUI | Displays a progress bar in headless mode |

### `PhysicsSimulationOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `timestep` | 1e-3 | s | Physics Engine | Integration time step |
| `gravity` | [0,0,-9.81] | m/s² | Physics Engine | Global gravity vector |
| `num_sub_steps` | 1 | - | Physics Engine | Physics Substepping |
| `cb_sub_steps` | 0 | - | Callback Engine | Callback frequency |
| `n_solver_iters` | 50 | - | Physics Solver | Solver precision limit |

### `MuJoCoSimulationOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `cone` | pyramidal | - | MuJoCo | Friction cone type |
| `solver` | Newton | - | MuJoCo | Constraint solver algorithm |
| `integrator` | Euler | - | MuJoCo | Integration scheme |
| `impratio` | 1 | - | MuJoCo | Impedance ratio |
| `ccd_iterations` | 50 | - | MuJoCo | Continuous collision checks |
| `ccd_tolerance` | 1e-6 | - | MuJoCo | Continuous collision detection tolerance |
| `noslip_iterations` | 0 | - | MuJoCo | No-slip solver iterations |
| `noslip_tolerance` | 1e-6 | - | MuJoCo | No-slip solver tolerance |
| `viewer` | MuJoCo | - | GUI | Viewer backend (`MuJoCo` or `dm_control`) |
| `texture_repeat` | 1 | - | MuJoCo | Repeating texture |
| `shadow_size` | 1024 | - | MuJoCo | Shadow size |
| `visual_scale` | 1.0 | - | MuJoCo | Visual scale |
| `extent` | 100.0 | - | MuJoCo | View extent |

### `PybulletSimulationOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `opengl2` | False | - | PyBullet | Fallback renderer toggle |
| `lcp` | dantzig | - | PyBullet | LCP Solver |
| `cfm` | 1e-10 | - | PyBullet | Constraint Force Mixing |
| `erp` | 0 | - | PyBullet | Error Reduction Parameter |
| `contact_erp` | 0 | - | PyBullet | Contact Error Reduction Parameter |
| `friction_erp` | 0 | - | PyBullet | Friction Error Reduction Parameter |
| `residual_threshold` | 1e-6 | - | PyBullet | Residual threshold |
| `max_num_cmd_per_1ms` | 1e8 | - | PyBullet | Max number of commands per 1ms |
| `report_solver_analytics` | 0 | - | PyBullet | Whether to report solver analytics |

---

## `farms_amphibious.model.options`

### `AmphibiousMorphologyOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `n_joints_body` | - | - | Convention | Defines spinal joints count |
| `n_links_body` | n_joints_body+1 | - | Convention | Number of body links |
| `n_legs` | - | - | Convention | Number of limbs |
| `n_dof_legs` | - | - | Convention | DoF per limb |
| `n_joints_passive` | - | - | Convention | Number of passive joints |
| `feet_friction` | default_lateral_friction (1) | - | `from_options` | Sets friction for foot links (consumed by `AmphibiousMorphologyOptions.from_options`, not a direct `__init__` field) |
| `mass_multiplier` | 1.0 | - | `AmphibiousLinkOptions` | Scales per-link mass (set on each `AmphibiousLinkOptions`, not on `AmphibiousMorphologyOptions` directly) |

### `AmphibiousControlOptions`

| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `controller_loader` | `farms_amphibious.control.amphibious.AmphibiousController` | - | Controller | AnimatController loader |
| `sensors` | - | - | `AmphibiousSensorsOptions` | Sensor tracking configuration |
| `motors` | - | - | `AmphibiousMotorOptions` | List of per-joint actuator options |
| `network` | None | - | `AmphibiousNetworkOptions` | CPG network options (None if no oscillators) |
| `muscles` | [] | - | `AmphibiousMuscleSetOptions` | List of muscle-set options |
| `hill_muscles` | [] | - | `MuscleOptions` | List of Hill-type muscle options |
| `adhesions` | - | - | `AmphibiousAdhesionsOptions` | List of adhesion options |
| `visuals` | - | - | `AmphibiousVisualsOptions` | List of visual options |

!!! note "`defaults_from_convention` kwargs (not direct `__init__` fields)"
    The following are consumed by `AmphibiousControlOptions.defaults_from_convention` (a config-parsing helper), not by `__init__` directly: `motor_gains` (default `[[0]]*n_joints`), `leg_turn_gain` (default `[0, 0]` for 4 legs), `max_torques` (default `default_max_torque`, which defaults to `inf`), and `equations` (default `phase`/`position_muscle` per joint). They appear in YAML configs and are applied to the per-motor options during convention-based initialization.
