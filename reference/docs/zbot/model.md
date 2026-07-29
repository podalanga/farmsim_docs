# Zbot Model — SDF geometry and physical properties

The Zbot's physical description lives in the SDF file at:

```
models/
└── zbot/
    └── sdf/
        ├── zbot.sdf          ← Main robot description
        └── meshes/           ← Visual mesh files (.stl)
            ├── head_red.stl
            ├── head_white.stl
            ├── segment.stl
            ├── tail_segment.stl
            └── tube_connector.stl
```

The SDF is loaded by FARMS at runtime and converted to MuJoCo's MJCF format internally. You should **not** modify the SDF directly for physics tuning — instead use the `morphology` section of `animat_config.yaml`, which overrides the SDF at load time.

---

## Link Anatomy

The Zbot body is a serial chain of **8 links** connected by **6 revolute joints**. All joints rotate around the **Y-axis** (lateral bending), creating a planar undulation in the horizontal plane.

```
                   ┌─────────┐
                   │  Head   │  mass = 1.9 kg
                   │ (rigid) │  density = 950 kg/m³
                   └────┬────┘
                        │  joint_1  (revolute, Y-axis)
                   ┌────┴────┐
                   │Segment1 │  mass = 0.16 kg
                   └────┬────┘
                        │  joint_2
                   ┌────┴────┐
                   │Segment2 │
                   └────┬────┘
                        │  joint_3
                   ┌────┴────┐
                   │Segment3 │
                   └────┬────┘
                        │  joint_4
                   ┌────┴────┐
                   │Segment4 │
                   └────┬────┘
                        │  joint_5
                   ┌────┴────┐
                   │Segment5 │
                   └────┬────┘
                        │  joint_6
                   ┌────┴────┐
                   │Segment6 │
                   └────┬────┘
                        (rigid, no joint)
                   ┌────┴────┐
                   │  Tail   │  drag_coeff = -10.0 (higher thrust)
                   │ Segment │
                   └─────────┘
```

---

## Link Physical Properties

All values come directly from `models/zbot/sdf/zbot.sdf` and are overridden/annotated in `animat_config.yaml`.

### Head

| Property | Value |
|----------|-------|
| SDF pose | `[0, 0, 0, 0, 0, 0]` (world origin) |
| Inertial CoM offset | `[0, -0.02, 0.04]` m |
| Mass | **1.9 kg** |
| Ixx | 0.00856 kg·m² |
| Iyy | 0.007695 kg·m² |
| Izz | 0.002965 kg·m² |
| Collision geometry | Cylinder (r=0.05 m, L=0.108 m) + Box (0.084×0.108×0.15 m) |
| Visual meshes | `head_red.stl`, `head_white.stl`, `tube_connector.stl` |
| Fluid density override | **950 kg/m³** (set in `animat_config.yaml`) |
| Drag coefficients (trans.) | `[-4.0, -4.0, -0.1]` N·s/m |
| Drag coefficients (rot.) | `[0, 0, 0]` |

!!! note "Density < Water"
    The Head density of 950 kg/m³ is **less than water** (1000 kg/m³), so the robot is positively buoyant. The `SwimmingExtension` computes buoyancy forces at runtime based on link submersion depth.

### Body Segments (Segment1 – Segment6)

All six body segments share identical inertia and drag properties.

| Property | Value |
|----------|-------|
| Mass | **0.16 kg** |
| SDF pose | Segment1 at `z=0.18 m`; each segment offset by ~0.13 m |
| Collision geometry | Cylinder per segment |
| Fluid density override | 950 kg/m³ |
| Drag coefficients (trans.) | `[-4.0, -4.0, -0.1]` N·s/m |
| Drag coefficients (rot.) | `[0, 0, 0]` |

### TailSegment

| Property | Value |
|----------|-------|
| Mass | ~0.16 kg (same as segments) |
| Drag coefficients (trans.) | **`[-10.0, -10.0, -0.1]`** N·s/m |
| Drag coefficients (rot.) | `[0, 0, 0]` |

!!! important "Why the tail has higher drag"
    The tail magnitude `-10.0` is **2.5× larger** than the body segments. This generates greater reactive thrust when the tail undulates — mimicking the caudal-fin propulsion of real anguilliform swimmers. Increasing this value amplifies thrust; reducing it weakens it.

---

## Joint Properties

All six revolute joints are **position-controlled** via a PD servo defined in `animat_config.yaml`. The SDF defines the joint axis and parent/child links; all gains live in the config YAML.

| Joint | Connects | Axis | Initial pos | Torque limits |
|-------|----------|------|-------------|---------------|
| `joint_1` | Head → Segment1 | Y | 0 rad | ±10 Nm |
| `joint_2` | Segment1 → Segment2 | Y | 0 rad | ±10 Nm |
| `joint_3` | Segment2 → Segment3 | Y | 0 rad | ±10 Nm |
| `joint_4` | Segment3 → Segment4 | Y | 0 rad | ±10 Nm |
| `joint_5` | Segment4 → Segment5 | Y | 0 rad | ±10 Nm |
| `joint_6` | Segment5 → Segment6 | Y | 0 rad | ±10 Nm |

Joint properties in `animat_config.yaml`:

```yaml
joints:
  - name: joint_1
    initial: [0, 0]           # [position (rad), velocity (rad/s)]
    limits:
      - [-inf, inf]           # position limits [min, max]
      - [-inf, inf]           # velocity limits [min, max]
    stiffness: 0              # passive structural stiffness (overridden by motor gains)
    springref: 0              # equilibrium angle for passive spring
    damping: 0                # passive damping (overridden by motor gains)
```

!!! tip "Unlimited joint range"
    `[-inf, inf]` means no hard stop is enforced by the physics engine. The Ekeberg muscle model's active stiffness term provides the effective soft limit in practice.

---

## Motor Control Configuration

Each joint has a corresponding **motor** definition in `animat_config.yaml` that specifies how the CPG's output torque is computed:

```yaml
motors:
  - joint_name: joint_1
    control_types: [position]    # PD position servo
    limits_torque: [-10.0, 10.0] # Saturation limits (Nm)
    gains: [3.0, 0.01, 0]        # [Kp, Kd, Ki]
    equation: position_muscle    # Uses Ekeberg muscle output as the position setpoint
    transform:
      gain: 1                    # Scales the CPG position command
      bias: 0                    # Adds offset to the CPG position command
    offsets:
      gain: 0.05                 # Amplitude of the joint offset modulation
      bias: 0
      low: 1                     # Drive threshold where offset activates
      high: 5
      saturation_low: 0
      saturation_high: 0
      rate: 3                    # Rate of offset convergence (1/s)
    passive:
      is_passive: false
      stiffness_coefficient: 0
      damping_coefficient: 0
      friction_coefficient: 0
```

### `equation: position_muscle` — What This Means

When `equation: position_muscle` is set, the `AmphibiousController` does **not** command a raw position. Instead, it computes the **Ekeberg muscle torque** and feeds it through the PD servo. The flow is:

```
CPG Phase (θ_L, θ_R)
       ↓  Ekeberg:  τ = α(A_L sin θ_L - A_R sin θ_R) + β(co-contraction)(φ_off - φ) - δφ̇
Desired position setpoint
       ↓  PD servo: u = Kp·(φ_des - φ) + Kd·(φ̇_des - φ̇)
Final torque sent to MuJoCo
```

### PD Gains Reference

| Gain | Symbol | Value | Effect |
|------|--------|-------|--------|
| `gains[0]` | Kp | **3.0** | Proportional — stiffness of position tracking |
| `gains[1]` | Kd | **0.01** | Derivative — damping of velocity error |
| `gains[2]` | Ki | **0** | Integral — not used |

---

## Spawn Pose

The robot spawns at a position slightly above ground (`z=0.01 m`) rotated into the correct swimming orientation:

```yaml
spawn:
  pose: [0, 0, 0.01, 0, -1.5708, 3.14159]
  #       x  y   z  roll  pitch    yaw
```

- **pitch = -π/2** rotates the robot so its long axis aligns with the X-axis (swimming forward).
- **yaw = π** flips the head to face the positive-X direction.
- **z = 0.01 m** spawns it just above the ground plane to avoid initial collision penetration.

---

## Arena Models

The arena uses two additional SDF models:

```
models/
├── arena_flat_v0/sdf/arena_flat.sdf    ← Ground plane with flat terrain
└── arena_water_v0/sdf/arena_water.sdf  ← Water volume geometry for buoyancy
```

The `arena_water.sdf` provides the geometry that `SwimmingExtension` uses to determine which links are submerged. The water surface height is controlled by `water.height` in `arena_config.yaml`.

---

## See Also

- [Swimming Experiment](experiment.md) — YAML config walkthrough
- [Custom CPG Controller](cpg_controller.md) — write your own controller
- [Mathematical Models](../mathematical_models.md) — drag and buoyancy equations
- [`SwimmingExtension` API](../api/farms_mujoco_swimming.md) — hydrodynamics implementation
