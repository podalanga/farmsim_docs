# farms_mujoco.swimming

Fluid forces (buoyancy, drag and added mass) on the links of swimming
animats. They are computed in C once per MuJoCo step, on a single core and
without allocation in the simulation loop, and applied through MuJoCo's
`xfrc_applied`. MuJoCo's own fluid model is disabled on the FARMS geoms
(`fluidcoef` set to 0).

| Model | Options | What it computes |
|---|---|---|
| Buoyancy | `buoyancy`, `cob_method`, `cob_geom_group`, `cob_overlap` | $-\rho V g$ at the centre of buoyancy, with $V$ and the centre computed from the link geoms |
| Legacy drag | `drag`, `fluid_model: legacy`, link `drag_coefficients` | Per-axis quadratic drag in the link frame |
| Ellipsoid drag | `drag`, `fluid_model: ellipsoid`, `ellipsoid_*` | Form, viscous and rotational drag of an ellipsoid fitted to each link |
| Added mass | `added_mass` (ellipsoid model only) | Lamb added mass, Kirchhoff and Munk terms |

The equations are in [Mathematical Models](../../explanation/mathematical-models.md#2-hydrodynamics),
and the implementation in [Hydrodynamics Internals](../../internals/hydrodynamics-internals.md).

## Configuration

A link has fluid forces when `fluid_interaction: true` in the animat file.
The water is described by the `water` block of the arena file:

```yaml
water:
  sdf: ../../models/arena_water_v0/sdf/arena_water.sdf  # Visual only
  drag: true
  buoyancy: true
  height: 0                    # Surface height [m]
  density: 1000.0              # [kg/m^3]
  viscosity: 1.0               # Scale of the quadratic drag
  velocity: [0, 0, 0]          # Water velocity [m/s], or maps (see below)
  # Fluid model options, all optional (defaults shown)
  cob_method: exact            # exact | lut | ramp
  cob_geom_group: 2            # 2: collision geoms, 1: visual meshes
  cob_overlap: ignore          # ignore | scale
  cob_lut_resolution: [32, 64] # LUT directions per side, depths
  drag_implicit: false         # Semi-implicit legacy drag
  fluid_model: legacy          # legacy | ellipsoid
  ellipsoid_fit: mvee          # mvee | inertia
  ellipsoid_coefficients: [1.0, 1.0, 2.5]  # Form, viscous, rotational
  dynamic_viscosity: 1.0e-3    # [Pa.s] ellipsoid viscous drag
  added_mass: off              # off | implicit | explicit
```

The fluid model options can also be grouped in a nested `cob` block
(`cob_method` becomes `cob.method`, and so on). Their full list, generated
from the code, is in the
[Configuration reference](../env/configuration-reference.md#fluid-model-options).

!!! note "Previous option values"
    `cob_method: analytic`, `analytical`, `analytic_fast` and `mesh` are
    aliases of `exact`. The other previous `cob_*` keys are accepted and
    ignored.

The animat's extension entry has no required configuration:

```yaml
extensions:
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config:
      water_properties: null  # Ignored: the arena's water block is used
```

## Choosing the options

**`cob_method`** (centre of buoyancy):

| Value | Accuracy | Cost per link | Use it for |
|---|---|---|---|
| `exact` | Exact for every geom type | about 6 to 170 ns per geom crossing the surface, 8 ns otherwise | Default |
| `lut` | About 1 % of the link volume | about 60 ns, whatever the geoms | Many geoms per link, overlapping geoms, many robots |
| `ramp` | Approximate, no buoyancy torque | minimal | Legacy behaviour |

`exact` counts overlapping geoms twice. `cob_overlap: scale` rescales them
by the union fraction, and `lut` uses the true union. See the buoyancy
note of [The Zbot Model](../../tutorials/zbot-model.md) for a model where
this matters.

`cob_geom_group: 1` uses the visual meshes instead of the collision
geoms, for bodies whose shape is poorly described by primitives. Meshes
that are not watertight are replaced by their convex hull.

**`fluid_model`**: `legacy` uses the per-link `drag_coefficients` (tuned
by hand). `ellipsoid` derives the drag and the added mass from the link
geometry, with no per-link coefficient.

**`added_mass`**: use `implicit` with the ellipsoid model. It adds the
added mass to the MuJoCo body masses and inertias, which is
unconditionally stable. `explicit` applies it as a force from filtered
accelerations and is unstable when the added mass exceeds about half the
link mass (a warning is printed).

**`drag_implicit`**: makes the legacy drag stable for large timesteps or
light links with strong drag.

## Water velocity maps

When `water.velocity` is not a 3-vector, it holds the velocity ranges and
the area of two PNG maps listed in `water.maps` (horizontal x and y
velocities):

```yaml
# check-docs: skip
water:
  velocity: [vx_min, vy_min, 0, vx_max, vy_max, 0, x_min, y_min, x_max, y_max]
  maps: [velocity_x.png, velocity_y.png]
```

The pixel values are scaled linearly to the velocity ranges, and the
velocity at a point is the value of the nearest pixel (zero outside the
area).

## Classes

### SwimmingExtension

The animat extension that applies the fluid forces. It is created with
`substep=True`, so the forces are computed at every environment step.
`initialize_episode()` creates a `SwimmingHandler`, and `before_step()`
calls its `step()`.

::: farms_mujoco.swimming.extension.SwimmingExtension
    options:
      show_root_heading: false
      heading_level: 4

### SwimmingHandler

Computes the fluid wrench of every link in a single C loop. It is also
useful on its own, for example to read the submerged volumes
(`links_submerged()`).

::: farms_mujoco.swimming.hydrodynamics.SwimmingHandler
    options:
      show_root_heading: false
      heading_level: 4

### Water properties

`SwimmingHandler` reads the water through a `WaterProperties` object:

| Class | Surface, density, viscosity | Velocity |
|---|---|---|
| `WaterPropertiesConstant` | Constant | Constant, can be changed with `set_velocity()` |
| `WaterPropertiesMaps` | Constant | Nearest pixel of the velocity maps |
| `WaterPropertiesExtension` | Python callables of time and position | Python callable |

`WaterPropertiesExtension` makes the water fully programmable (waves,
currents), at the cost of Python calls in the loop.

### FluidOptions

::: farms_mujoco.swimming.fluid_options.FluidOptions
    options:
      show_root_heading: false
      heading_level: 4

## Tools

| Script (`farms_mujoco/benchmarks/`) | Use |
|---|---|
| `inspect_buoyancy.py --experiment DIR` | Per-link volumes and buoyancy/weight budget of a model, with each method |
| `bench_fluid.py --experiment DIR` | Timing of the fluid forces in a full simulation |
| `bench_cob.py` | Timing of the centre of buoyancy kernels |

## See Also

- [Hydrodynamics Internals](../../internals/hydrodynamics-internals.md)
- [Mathematical Models](../../explanation/mathematical-models.md#2-hydrodynamics)
- [MuJoCo Simulation](mujoco-simulation.md)
- [API reference: `farms_mujoco.swimming`](../api/farms_mujoco/swimming.md)
