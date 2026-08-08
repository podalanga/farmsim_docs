# farms_mujoco.swimming

Phenomenological hydrodynamic drag model for aquatic and amphibious robots.

## Overview

Standard rigid-body physics engines like MuJoCo excel at contact dynamics but lack native computational fluid dynamics (CFD) solvers. To simulate amphibious and aquatic locomotion, this module implements an efficient phenomenological hydrodynamic drag model. It computes real-time fluid forces and injects them directly into the physics engine loop.

!!! warning "Limitations"
    Added mass and lift are **NOT** implemented in this model. Only translational drag, rotational drag, and buoyancy are calculated.

---

## Class Reference: SwimmingExtension

Inherits from `AnimatExtension`. Computes hydrodynamic forces on all individual rigid bodies and injects them into the physics step.

```python
SwimmingExtension.__init__(self, animat_i, animat_data, animat_options, arena_options, substep=True, water_properties=None)
```

| Name | Type | Default | Description |
|---|---|---|---|
| `animat_i` | `int` | Required | Index of the animat in the simulation. |
| `animat_data` | `AnimatData` | Required | Central data object for telemetry. |
| `animat_options` | `AnimatOptions` | Required | Configuration of the animat morphology. |
| `arena_options` | `ArenaOptions` | Required | Configuration for the arena and water. |
| `substep` | `bool` | `True` | Whether to step the extension on physics substeps. |
| `water_properties` | `WaterPropertiesExtension` | `None` | Reference to water definitions. |

Note: The `__init__` method will load and process PNG water velocity flow field maps from `arena_options.water.maps` whenever `arena_options.water.velocity` is not a 3-element constant vector (i.e. `len(velocity) != 3`). In that case `velocity` carries the velocity range and spatial bounds, and a `WaterPropertiesExtension` backed by the maps is constructed.

```python
SwimmingExtension.from_options(cls, config, experiment_options, animat_i, animat_data, animat_options)
```
Instantiates the extension by reading the global `experiment_options.arenas[0]` settings.

```python
SwimmingExtension.initialize_episode(self, task, physics)
```
Constructs the core `SwimmingHandler`, parsing the morphology's geometric shapes and constructing the fluid resistance profiles for each link.

```python
SwimmingExtension.before_step(self, task, action, physics)
```
Steps the hydrodynamic drag computations. Modifies the underlying `physics.data.xfrc_applied` array to inject the resulting computed global torques and forces on a per-body basis.

!!! danger "Known bug: forces are rotated twice"
    `before_step()` currently applies an extra `xmat` rotation on top of
    forces that `compute_link_forces` (in `hydrodynamics.pyx`) already
    rotated into the world frame — see [Hydrodynamics Internals § Common
    failure modes](../../internals/hydrodynamics-internals.md#3-double-rotation-of-forces-confirmed-bug-see-before_step-above)
    for the confirmed root cause. Force/torque direction on any link away
    from identity orientation is currently wrong.

---

## Drag Force Computation

The handler computes forces based on a simplified quadratic drag model. For a rigid body moving through a fluid, the force and torque vectors are computed locally relative to the body's orientation. The implementation lives in `farms_mujoco/swimming/drag.pyx` (pure drag math), orchestrated together with buoyancy by `farms_mujoco/swimming/hydrodynamics.pyx` (`SwimmingHandler`), which is the file `SwimmingExtension` actually imports.

$$ F_{D, local} = C_{D, lin} \circ \mu \circ V_{local} \circ |V_{local}| + F_{buoyancy, local} $$
$$ \tau_{D, local} = C_{D, ang} \circ \omega_{local} \circ |\omega_{local}| $$

Where:
- $\mu$ is the fluid viscosity (as returned by `WaterProperties.viscosity`).
- $V_{local}$ and $\omega_{local}$ are linear and angular velocities relative to the surrounding water (the ambient fluid velocity is rotated into the URDF frame and subtracted from the link velocity).
- $C_{D, lin}$ and $C_{D, ang}$ are coefficients defining resistance along local axes (`coefficients[0]` for force, `coefficients[1]` for torque).
- $F_{buoyancy, local}$ is the buoyancy force (computed by `buoyancy_cy.pyx`) folded into the drag force as the last step.
- $\circ$ denotes the Hadamard (element-wise) product.

!!! note "Sign convention"
    The code computes $v|v|$ (i.e. $v^2 \cdot \operatorname{sgn}(v)$) and multiplies by $\mu \cdot C_D$. There is no explicit $-0.5 \cdot \rho$ prefactor in the implementation; whether the resulting force acts as drag (opposing motion) or thrust depends on the sign of the configured coefficients $C_D$.

### Axial vs Normal Decomposition

By separating drag coefficients for the local X axis (axial/forward) versus local Y/Z axes (normal/perpendicular), the model simulates anisotropic drag. This enables undulatory propulsion where sliding forward encounters less resistance than lateral oscillation.

### Configuration Format

Drag coefficients are configured in `MorphologyOptions` as a $2 \times 3$ matrix per link:
```yaml
drag_coefficients:
  - [C_x, C_y, C_z] # Translational coefficients
  - [C_wx, C_wy, C_wz] # Rotational coefficients
```

---

## Water Properties and Maps

### WaterPropertiesExtension
Provides callbacks to query specific localized properties of the fluid. It handles queries for water surface height, density, velocity, and viscosity (the four `WaterProperties` cdef methods: `surface`, `density`, `velocity`, `viscosity`), enabling dynamically structured fluids.

### water_velocity_from_maps
```python
water_velocity_from_maps(position, water_maps)
```
When complex current flows are required, FARMS uses PNG-based flow fields. This function samples the $V_{water}$ vector at any spatial `position` by mapping the position into the image's 2D bounding box and picking the nearest pixel (nearest-neighbor sampling via `round(...)`, not bilinear interpolation). The lookup is only performed when the position lies inside the configured `pos_min`/`pos_max` bounds; otherwise a zero velocity is returned.

---

## See Also

- [Controller Base Classes](../core/core-control.md) — How hydrodynamic forces are applied
- [MuJoCo Simulation](mujoco-simulation.md) — Physics backend integration
