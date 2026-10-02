# Zbot Model: SDF geometry and physical properties

!!! abstract "Project page"
    This page belongs to the [Zbot project](index.md), whose repository,
    `farms_zbot`, holds the robot model, the experiments and a Docker
    workspace. The repository is private: ask the maintainers for access.
    Paths such as `experiments/` and `models/` are relative to it. For
    FarmSim itself, start with [Get started](../../get-started/index.md).

The Zbot's physical description lives in the SDF file at:

```
models/
└── zbot/
    └── sdf/
        ├── zbot.sdf          ← Main robot description
        └── meshes/           ← Visual mesh files (.stl)
            ├── head_red.stl
            ├── head_white.stl
            ├── head_white_fins.stl
            ├── segment.stl
            ├── tail_connector.stl
            ├── tail_fin.stl
            └── tube_connector.stl
```

The SDF is loaded by FARMS at runtime and converted to MuJoCo's MJCF format internally. Masses, inertias and geometry come from the SDF. The `morphology` section of `animat_config.yaml` adds the simulation properties of each link and joint (fluid interaction, drag coefficients, friction, joint stiffness, damping and spring reference). Joint limits also come from the SDF file.

---

## Link Anatomy

The Zbot body is a serial chain of **8 links**: the Head and six segments are connected by **6 revolute joints**, and the TailSegment is rigidly fixed to Segment6. All joints rotate around the link **Y-axis** (lateral bending), creating a planar undulation.

```
                   ┌─────────┐
                   │  Head   │  mass = 1.9 kg
                   │ (rigid) │
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
                   │  Tail   │  drag coefficients -10.0 (x, y)
                   │ Segment │
                   └─────────┘
```

---

## Link Physical Properties

Masses, inertias and geometry come from `models/zbot/sdf/zbot.sdf`; densities and drag coefficients from the `morphology` section of the experiments' `animat_config.yaml` (the values below are those of `zbot_swimming` and `zbot_bout_glide`).

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
| Link `density` | 950 kg/m³ (`animat_config.yaml`) |
| Drag coefficients (linear) | `[-4.0, -4.0, -0.1]` kg/m |
| Drag coefficients (angular) | `[0, 0, 0]` (`zbot_swimming`), `[-0.0005, -0.0005, -0.0005]` (`zbot_bout_glide`) |

!!! note "Buoyancy comes from the geometry, not from `density`"
    With the default `cob_method: exact`, buoyancy is `rho_water * g * V`, where `V` is the submerged volume of the link's collision geoms: it depends on the geometry only, and whether the robot floats depends on its mass (from the SDF) compared with `rho_water * V`. The link `density` is only used by the legacy `cob_method: ramp`. The collision geoms of a link overlap (for example the segments' cylinder and boxes), and `exact` counts the overlapping volume twice, which makes the zbot float. With the true union volume (`cob_method: lut` or `cob_overlap: scale`) the zbot is slightly heavier than the water it displaces. Use `farms/farms_mujoco/benchmarks/inspect_buoyancy.py` to print the buoyancy budget of each link. See [MuJoCo Swimming](../../reference/mujoco/mujoco-swimming.md).

### Body Segments (Segment1 to Segment6)

All six body segments share identical inertia and drag properties.

| Property | Value |
|----------|-------|
| Mass | **0.16 kg** |
| SDF pose | Segment1 at `z=0.18 m`, Segment2 at `z=0.261 m`, then every 0.065 m |
| Collision geometry | Cylinder (r=0.0175 m, L=0.0652 m) + boxes 0.035×0.065×0.045, 0.005×0.015×0.06 and 0.005×0.02×0.06 m |
| Visual mesh | `segment.stl` |
| Link `density` | 950 kg/m³ |
| Drag coefficients (linear) | `[-4.0, -4.0, -0.1]` kg/m |
| Drag coefficients (angular) | as the Head |

### TailSegment

| Property | Value |
|----------|-------|
| Mass | **0.086 kg** (lighter than body segments) |
| SDF pose | `z=0.606 m`, fixed to Segment6 |
| Collision geometry | Boxes 0.008×0.085×0.05 and 0.003×0.085×0.05 m |
| Visual meshes | `tail_connector.stl`, `tail_fin.stl` |
| Drag coefficients (linear) | **`[-10.0, -10.0, -0.1]`** kg/m |
| Drag coefficients (angular) | as the Head |

!!! note "Why the tail has higher drag"
    The legacy drag model is per axis: `F_i = viscosity * c_i * v_i * |v_i|` in the link frame. The tail's lateral coefficient (`-10.0`, 2.5 times the body segments) represents the caudal fin: the large lateral resistance of the tail is what generates reactive thrust when it undulates. Increasing it amplifies thrust, reducing it weakens it.

---

## Joint Properties

All six revolute joints are **position-controlled** via a PD servo defined in `animat_config.yaml`. The SDF defines the joint axis and parent/child links; all gains live in the config YAML.

| Joint | Connects | Axis | Initial position | Torque limits (`limits_torque`) |
|-------|----------|------|-------------|---------------|
| `joint_1` | Head → Segment1 | Y | 0 rad | ±10 N·m |
| `joint_2` | Segment1 → Segment2 | Y | 0 rad | ±10 N·m |
| `joint_3` | Segment2 → Segment3 | Y | 0 rad | ±10 N·m |
| `joint_4` | Segment3 → Segment4 | Y | 0 rad | ±10 N·m |
| `joint_5` | Segment4 → Segment5 | Y | 0 rad | ±10 N·m |
| `joint_6` | Segment5 → Segment6 | Y | 0 rad | ±10 N·m |

Joint properties in `animat_config.yaml`:

```yaml
joints:
  - name: joint_1
    initial: [0, 0]           # [position (rad), velocity (rad/s)]
    limits:
      - [-inf, inf]           # position limits [min, max]
      - [-inf, inf]           # velocity limits [min, max]
    stiffness: 0              # passive joint stiffness
    springref: 0              # equilibrium angle of the passive spring
    damping: 0                # passive joint damping
```

!!! tip "Unlimited joint range"
    `[-inf, inf]` means no hard stop is enforced by the physics engine; the joint range is only limited by the actuators and the dynamics.

---

## Motor Control Configuration

Each joint has a **motor** definition in `animat_config.yaml` (this excerpt is from `zbot_swimming`, which uses the built-in `AmphibiousController`):

```yaml
motors:
  - joint_name: joint_1
    control_types: [position]    # Driven by the controller's positions()
    limits_torque: [-10.0, 10.0] # Actuator force range [N.m]
    gains: [3.0, 0.01, 0]        # [kp, kv of the position actuator, kv of the velocity actuator]
    equation: position_muscle    # AmphibiousController: position from the oscillator outputs
    transform:
      gain: 1                    # Scales the position command
      bias: 0                    # Adds an offset to the position command
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

### `equation: position_muscle`: what this means

`equation` selects how the `AmphibiousController` turns the CPG state into a joint command (`position_muscle`, `position_phase`, `ekeberg_muscle`, `ekeberg_muscle_explicit` or `passive`). With `position_muscle` (`farms_amphibious/control/position_muscle_cy.pyx`), the position command of the joint is computed from the outputs `n_L`, `n_R` of its two oscillators and the drive-dependent offset:

```
command = transform.gain * (0.5 * (n_R - n_L) + offset) + transform.bias
```

The MuJoCo position actuator then applies the torque
`kp * (command - q) - kv * dq/dt`, saturated to `limits_torque`. The Ekeberg muscle model is a different equation (`ekeberg_muscle`), which computes a torque directly.

Custom controllers such as `ZbotCPGController` (`zbot_bout_glide`) ignore `equation` and compute their own position commands.

### Actuator gains

MuJoCo creates a position, a velocity and a torque actuator for every motor joint (`farms_mujoco/simulation/mjcf.py`):

| Gain | Used as | Value | Effect |
|------|---------|-------|--------|
| `gains[0]` | `kp` of the position actuator | **3.0** | Stiffness of the position tracking |
| `gains[1]` | `kv` of the position actuator | **0.01** | Damping on the joint velocity |
| `gains[2]` | `kv` of the velocity actuator | **0** | Gain of velocity control (unused here) |

---

## Spawn Pose

The robot spawns at the water surface (`z=0.01 m`, with `water.height: 0` and the ground at `ground_height: -1`), rotated into the swimming orientation:

```yaml
spawn:
  pose: [0, 0, 0.01, 0, -1.5708, 3.14159]
  #       x  y   z  roll  pitch    yaw
```

- **pitch = -π/2** rotates the robot so its long axis aligns with the X-axis (swimming forward).
- **yaw = π** flips the head to face the positive-X direction.
- **z = 0.01 m** places it at the water surface, 1 m above the ground.

---

## Arena Models

The arena uses two additional SDF models:

```
models/
├── arena_flat_v0/sdf/arena_flat.sdf    ← Ground plane with flat terrain
└── arena_water_v0/sdf/arena_water.sdf  ← Visual water surface
```

`arena_water.sdf` only draws the water. The fluid forces use the water surface height `water.height` (and density, viscosity and velocity) of `arena_config.yaml`; the submerged part of each link is computed from its geoms (see [MuJoCo Swimming](../../reference/mujoco/mujoco-swimming.md)).

---

## See Also

- [Swimming Experiment](experiment.md): YAML config walkthrough
- [Custom CPG Controller](custom-controller.md): write your own controller
- [Mathematical Models](../../explanation/mathematical-models.md): drag and buoyancy equations
- [`SwimmingExtension` API](../../reference/mujoco/mujoco-swimming.md): hydrodynamics implementation
