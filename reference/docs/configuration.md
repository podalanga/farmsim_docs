# Configuration Reference — YAML parameter definitions

This document enumerates the tunable parameters (Options classes) across the FARMS framework, specifically checking `farms_core` and `farms_amphibious` models, simulations, and experiments.

!!! note "Source Files"
    - `farms_core/farms_core/model/options.py` — `AnimatOptions`, `MorphologyOptions`, `ControlOptions`
    - `farms_core/farms_core/model/extensions.py` — `TaskExtension`, `AnimatExtension`
    - `farms_amphibious/farms_amphibious/model/options.py` — `AmphibiousOptions`, `AmphibiousArenaOptions`
    - `farms_amphibious/farms_amphibious/control/network.py` — `NetworkConfig`

## `farms_core.model.options`

### `LinkOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `name` | - | - | SDF Builder | Link identifier |
| `collisions` | - | - | Physics Engine | Toggles collision geometry |
| `friction` | - | - | Physics Engine | Lateral, spinning, rolling friction |
| `density` | 1000 | kg/m³ | Fluid interaction | Computes buoyancy / mass |
| `drag_coefficients` | [0,0,0,0,0,0] | - | Fluid interaction | Computes hydrodynamic drag |
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
| `velocity` | - | m/s, rad/s| Simulator init | Spawn linear/angular velocity |

### `MotorOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `joint_name` | - | - | Controller | Target joint to actuate |
| `control_types` | - | - | Controller | Actuation mode (position/torque) |
| `limits_torque` | - | N·m | Controller | Max/min torque allowed |
| `gains` | - | - | Controller | Kp, Kd for PID position control |

### `SensorsOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `links` | [] | - | Logger/Controller | Tracks specific link states |
| `joints` | [] | - | Logger/Controller | Tracks specific joint states |
| `contacts` | [] | - | Logger/Controller | Tracks contact forces |
| `xfrc` | [] | - | Logger/Controller | Tracks external forces |

### `WaterOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `drag` | - | - | Fluid interaction | Enables drag forces |
| `buoyancy` | - | - | Fluid interaction | Enables buoyancy forces |
| `height` | - | m | Fluid interaction | Surface level of water |
| `velocity` | - | m/s | Fluid interaction | Flow speed and direction |
| `viscosity` | - | Pa·s | Fluid interaction | Fluid friction |
| `density` | - | kg/m³ | Fluid interaction | Fluid density |

### `MuscleOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `max_force` | - | N | Muscle Model | Maximum contractile force |
| `optimal_fiber` | - | m | Muscle Model | Optimal muscle length |
| `tendon_slack` | - | m | Muscle Model | Tendon resting length |
| `max_velocity` | - | m/s | Muscle Model | Max contraction velocity |
| `pennation_angle`| - | rad | Muscle Model | Muscle fiber angle |

---

## `farms_core.simulation.options`

### `RuntimeSimulationOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `n_iterations` | 1000 | - | Simulation Loop | Total steps to run |
| `buffer_size` | n_iter | - | Data Logger | Logging buffer limit |
| `play` | True | - | GUI | Starts unpaused |
| `rtl` | 1.0 | - | Timer | Real-time limiter |
| `fast` | False | - | Simulation Loop | Runs headless without sleep |
| `headless` | False | - | GUI/Renderer | Disables rendering |

### `PhysicsSimulationOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `timestep` | 1e-3 | s | Physics Engine | Integration time step |
| `gravity` | [0,0,-9.81]| m/s² | Physics Engine | Global gravity vector |
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
| `ccd_iterations`| 50 | - | MuJoCo | Continuous collision checks |

### `PybulletSimulationOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `opengl2` | False | - | PyBullet | Fallback renderer toggle |
| `lcp` | dantzig | - | PyBullet | LCP Solver |
| `cfm` | 1e-10 | - | PyBullet | Constraint Force Mixing |
| `erp` | 0 | - | PyBullet | Error Reduction Parameter |

---

## `farms_amphibious.model.options`

### `AmphibiousMorphologyOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `n_joints_body`| - | - | Convention | Defines spinal joints count |
| `n_legs` | - | - | Convention | Number of limbs |
| `n_dof_legs` | - | - | Convention | DoF per limb |
| `feet_friction`| 1.0 | - | LinkOptions | Sets friction for foot links |
| `mass_multiplier`| 1.0 | - | LinkOptions | Scales total mass |

### `AmphibiousControlOptions`
| Name | Default | SI Unit | Consumed By | Downstream Effect |
|------|---------|---------|-------------|-------------------|
| `motor_gains` | 0 | - | MotorOptions | Set Kp/Kd for all motors |
| `leg_turn_gain`| 0 | - | Controller | Modifies leg stroke amplitude |
| `max_torques` | inf | N·m | MotorOptions | Sets actuator limits |
| `equations` | phase/pos | - | Controller | Actuation control law |
