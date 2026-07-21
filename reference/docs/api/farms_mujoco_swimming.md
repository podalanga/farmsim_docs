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

Note: The `__init__` method will load and process PNG water velocity flow field maps if `arena_options.water.velocity` points to map files.

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

---

## Drag Force Computation

The handler computes forces based on a simplified quadratic drag model. For a rigid body moving through a fluid, the force and torque vectors are computed locally relative to the body's orientation:

$$ F_{D, local} = -0.5 \cdot \rho \cdot \left( C_{D, lin} \circ |V_{local}| \circ V_{local} \right) $$
$$ \tau_{D, local} = -0.5 \cdot \rho \cdot \left( C_{D, ang} \circ |\omega_{local}| \circ \omega_{local} \right) $$

Where:
- $\rho$ is the fluid density.
- $V_{local}$ and $\omega_{local}$ are linear and angular velocities relative to the surrounding water.
- $C_{D, lin}$ and $C_{D, ang}$ are coefficients defining resistance along local axes.
- $\circ$ denotes the Hadamard (element-wise) product.

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
Provides callbacks to query specific localized properties of the fluid. It handles queries for water surface height, density, and viscosity, enabling dynamically structured fluids.

### water_velocity_from_maps
```python
water_velocity_from_maps(position, water_maps)
```
When complex current flows are required, FARMS uses PNG-based flow fields. This function performs bilinear interpolation of the pixel intensity within a localized 2D bounding box to sample the exact $V_{water}$ vector at any spatial `position`.

---

## See Also
- [farms_core.model.control](farms_core_control.md)
- [farms_mujoco.simulation](farms_mujoco_simulation.md)
- [Math Extraction](../math_extraction.md)
