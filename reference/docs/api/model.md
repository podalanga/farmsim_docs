# Zbot Model — SDF Geometry and Physical Properties

The Zbot model is defined in SDF format at `models/zbot/sdf/zbot.sdf`. The configuration for the simulation is in `experiments/zbot_swimming/animat_config.yaml`.

## Robot Structure

The Zbot is a serial chain of 8 rigid bodies connected by 6 revolute joints:

```
Head → Segment1 → Segment2 → Segment3 → Segment4 → Segment5 → Segment6 → TailSegment
```

Each joint allows rotation about a single axis, enabling planar undulation.

## Physical Properties

### Links

Each link has the following physical properties:

| Property | Value | Unit |
|----------|-------|------|
| `density` | 1000.0 | kg/m³ |
| `friction` | [0.0, 0, 0] | - |
| `fluid_interaction` | true | - |
| `drag_coefficients` | [[-4.0, -4.0, -0.1], [0, 0, 0]] | - |

The drag coefficients are configured to provide realistic hydrodynamic resistance. The first triplet controls translational drag along x, y, z axes. The second triplet controls rotational drag.

### Joints

Each joint has:

| Property | Value | Unit |
|----------|-------|------|
| `initial` | [0, 0] | rad, rad/s |
| `limits` | [[-inf, inf], [-inf, inf]] | rad, rad/s |
| `stiffness` | 0 | N·m/rad |
| `springref` | 0 | rad |
| `damping` | 0 | N·m·s/rad |

### Motors

Each joint is actuated with:

| Property | Value | Unit |
|----------|-------|------|
| `control_types` | [position] | - |
| `limits_torque` | [-10.0, 10.0] | N·m |
| `gains` | [3.0, 0.01, 0] | Kp, Kd, Kv |
| `equation` | position_muscle | - |

The `position_muscle` equation means the CPG oscillator outputs are converted to position setpoints via the `PositionMuscleCy` actuator model.

## See Also

- [Swimming Experiment](./experiment.md)
- [Custom CPG Controller](./cpg_controller.md)
- [Configuration Reference](../configuration.md)
