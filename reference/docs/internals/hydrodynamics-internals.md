# Hydrodynamics Internals

This page documents the swimming hydrodynamics system in detail. The force computation is split across three Cython files: `drag.pyx` (drag forces), `buoyancy_cy.pyx` (buoyancy forces), and `hydrodynamics.pyx` (the orchestration layer that combines them). The Python `SwimmingExtension` drives the per-step loop.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_mujoco/swimming/hydrodynamics.pyx` | 512 | Orchestration: `SwimmingHandler`, `WaterProperties*`, `compute_link_forces`, `apply_swimming_forces` |
| `farms_mujoco/swimming/drag.pyx` | — | Pure drag force/torque computation (`compute_link_drag_fast`) |
| `farms_mujoco/swimming/buoyancy_cy.pyx` | — | Pure buoyancy force/torque computation (`compute_link_buoyancy_fast`) |
| `farms_mujoco/swimming/extension.py` | 216 | `SwimmingExtension` — Python wrapper, water map loading, force application |
| `farms_mujoco/swimming/cob_options.py` | — | `CobOptions` — center-of-buoyancy method configuration |

## Architecture

```
SwimmingExtension (Python, extension.py)
  ├─ initialize_episode(): Creates SwimmingHandler
  └─ before_step():
       ├─ handler.step(time, iteration)           [Cython, hydrodynamics.pyx]
       │    └─ for each link:
       │         apply_swimming_forces(...)         [Cython]
       │           └─ compute_link_forces(...)       [Cython, pure computation]
       │                ├─ link_swimming_info(...)    [kinematic state]
       │                ├─ compute_link_buoyancy_fast  [buoyancy_cy.pyx, C call]
       │                └─ compute_link_drag_fast     [drag.pyx, C call]
       └─ Write forces to physics.data.xfrc_applied  [Python]
```

## `WaterProperties` class hierarchy

```cython
cdef class WaterProperties:
    cdef double surface(self, double t, double x, double y)    # returns 0
    cdef double density(self, double t, double x, double y, double z)  # returns 1000
    cdef DTYPEv1 velocity(self, double t, double x, double y, double z)  # returns [0,0,0]
    cdef double viscosity(self, double t, double x, double y, double z)  # returns 1.0

cdef class WaterPropertiesConstant(WaterProperties):
    # Stores constant values, returns them regardless of position/time

cdef class WaterPropertiesExtension(WaterProperties):
    # Stores Python callbacks, calls them with position/time
```

### `WaterProperties` (base)

Base class with default values: surface=0, density=1000, velocity=[0,0,0], viscosity=1.0. All methods are `cdef` (C-level only, not callable from Python).

### `WaterPropertiesConstant`

Stores constant scalar/array values. Constructor:

```cython
def __init__(self, surface, density, velocity, viscosity):
    self._surface = surface
    self._density = density
    self._velocity = velocity
    self._viscosity = viscosity
```

All methods return the stored constant. Has an additional `cpdef void set_velocity(self, double t, double vx, double vy, double vz)` method for updating velocity at runtime.

### `WaterPropertiesExtension`

Stores Python callables. Each method calls the stored callback:

```cython
cdef double surface(self, double t, double x, double y):
    return self._surface(t, x, y)

cdef DTYPEv1 velocity(self, double t, double x, double y, double z):
    return self._velocity(t, x, y, z)
```

This allows spatially-varying water properties (e.g., velocity fields from PNG maps).

## `link_swimming_info()` — kinematic state

```cython
cdef void link_swimming_info(
    LinkSensorArrayCy data_links,
    unsigned int iteration,
    int sensor_i,
    DTYPEv1 urdf2global,
    DTYPEv1 com2global,
    DTYPEv1 global2urdf,
    DTYPEv1 com2urdf,
    DTYPEv1 urdf2com,
    DTYPEv1 link_lin_velocity,
    DTYPEv1 link_ang_velocity,
    DTYPEv1 quat_c,
    DTYPEv1 tmp4,
):
```

### Purpose

Computes the kinematic state shared by both drag and buoyancy. All outputs are written into caller-provided scratch arrays (no allocation).

### Computation

1. **Orientations**:
   - `urdf2global` = link's URDF frame orientation in global frame
   - `com2global` = link's CoM frame orientation in global frame
   - `global2urdf` = conjugate of `urdf2global` (inverse rotation)
   - `com2urdf` = `global2urdf * com2global` (CoM to URDF transform)
   - `urdf2com` = conjugate of `com2urdf`

2. **Velocities in URDF frame**:
   - `link_lin_velocity` = `quat_rot(com_lin_velocity, global2urdf)` — linear velocity rotated from global to URDF frame
   - `link_ang_velocity` = `quat_rot(com_ang_velocity, global2urdf)` — angular velocity rotated from global to URDF frame

The `quat_c` and `tmp4` are temporary scratch arrays used by `quat_rot`.

## `compute_link_forces()` — pure computation

```cython
cdef bint compute_link_forces(
    double time, unsigned int iteration,
    LinkSensorArrayCy data_links, unsigned int links_index,
    DTYPEv2 coefficients, DTYPEv2 z3, DTYPEv2 z4,
    WaterProperties water,
    double mass, double bound_radius, double density, double gravity,
    bint use_buoyancy, object primitives,
    bint use_exact_cob, bint force_mesh,
    DTYPEv1 force_out, DTYPEv1 torque_out,
):
```

### Return value

Returns `False` if the link's bounding sphere is entirely above the water surface (nothing to compute). Returns `True` if forces were computed.

### Walkthrough

**Step 1: Water surface check** (lines 124–130)

```cython
cdef double pos_z = data_links.array[iteration, links_index, 2]
cdef double surface = water.surface(time, pos_x, pos_y)
if pos_z - bound_radius > surface:
    return False
```

If the link's lowest point (`pos_z - bound_radius`) is above the water surface, skip all computation. This is the primary optimization: links out of water don't incur any cost.

**Step 2: Scratch array allocation** (lines 132–140)

All scratch arrays are pre-allocated rows from `z3` (12 rows of 3) and `z4` (7 rows of 4):

| Array | Source | Purpose |
|---|---|---|
| `force` | `z3[0]` | Drag force (URDF frame) |
| `torque` | `z3[1]` | Drag torque (URDF frame) |
| `buoyancy` | `z3[2]` | Buoyancy force |
| `tmp` | `z3[3]` | Temporary |
| `link_lin_velocity` | `z3[4]` | Link linear velocity (URDF) |
| `link_ang_velocity` | `z3[5]` | Link angular velocity (URDF) |
| `fluid_velocity_urdf` | `z3[6]` | Fluid velocity (URDF) |
| `pos_urdf` | `z3[7]` | Position in URDF frame |
| `com_position` | `z3[8]` | CoM position |
| `buoyancy_torque` | `z3[9]` | Buoyancy torque |
| `_force_out` | `z3[10]` | Final force (world frame) |
| `_torque_out` | `z3[11]` | Final torque (world frame) |

**Step 3: Kinematic state** (lines 142–155)

Calls `link_swimming_info()` to populate orientation and velocity arrays.

**Step 4: Water properties** (line 157)

```cython
cdef double water_density = water.density(time, pos_x, pos_y, pos_z)
```

Queries the water density at the link's position. For `WaterPropertiesConstant`, this is a fixed value. For `WaterPropertiesExtension`, this calls the Python callback.

**Step 5: CoM position** (lines 162–166)

Only computed if `use_buoyancy and use_exact_cob and primitives` — the CoM position is only needed for exact center-of-buoyancy methods.

**Step 6: Buoyancy computation** (lines 168–191)

```cython
compute_link_buoyancy_fast(
    use_buoyancy=use_buoyancy,
    use_exact_cob=use_exact_cob,
    force_mesh=force_mesh,
    primitives=primitives,
    density=density,
    water_density=water_density,
    bound_radius=bound_radius,
    pos_x=pos_x, pos_y=pos_y, pos_z=pos_z,
    global2urdf=global2urdf, urdf2global=urdf2global,
    mass=mass, surface=surface, gravity=gravity,
    buoyancy=buoyancy, buoyancy_torque=buoyancy_torque,
    com_position=com_position, pos_urdf=pos_urdf,
    quat_c=quat_c, tmp4=tmp4, tmp=tmp,
)
```

This is a direct C-level call to `buoyancy_cy.pyx`. The buoyancy force and torque are computed in the URDF frame.

**Step 7: Drag computation** (lines 193–206)

```cython
compute_link_drag_fast(
    force=force, torque=torque,
    link_lin_velocity=link_lin_velocity,
    link_ang_velocity=link_ang_velocity,
    fluid_velocity_world=water.velocity(time, pos_x, pos_y, pos_z),
    global2urdf=global2urdf,
    quat_c=quat_c, tmp4=tmp4,
    fluid_velocity_urdf=fluid_velocity_urdf,
    coefficients=coefficients,
    buoyancy=buoyancy,
    viscosity=water.viscosity(time, pos_x, pos_y, pos_z),
)
```

This is a direct C-level call to `drag.pyx`. The drag force and torque are computed in the URDF frame. Note that `buoyancy` is passed to the drag computation — the buoyancy force affects the effective velocity for drag (added mass effect).

**Step 8: Combine and rotate** (lines 207–214)

```cython
for i in range(3):
    torque[i] += buoyancy_torque[i]  # Add buoyancy torque to drag torque

# Rotate to world frame
quat_rot(force, urdf2global, quat_c, tmp4, force_out)
quat_rot(torque, urdf2global, quat_c, tmp4, torque_out)
```

The drag and buoyancy torques are combined, then both force and torque are rotated from the URDF frame to the world frame.

## `apply_swimming_forces()` — write to data

```cython
cpdef bint apply_swimming_forces(
    double time, unsigned int iteration,
    LinkSensorArrayCy data_links, unsigned int links_index,
    XfrcArrayCy data_xfrc, unsigned int xfrc_index,
    DTYPEv2 coefficients, DTYPEv2 z3, DTYPEv2 z4,
    WaterProperties water,
    double mass, double bound_radius, double density, double gravity,
    bint use_buoyancy, object primitives=None,
    bint use_exact_cob=False, bint force_mesh=False,
):
```

Calls `compute_link_forces()` and, if the link is in water, writes the result into `data_xfrc`:

```cython
in_water = compute_link_forces(..., force_out=_force_out, torque_out=_torque_out)
if not in_water:
    return False

for i in range(3):
    data_xfrc.array[iteration, xfrc_index, i] = _force_out[i]       # Force [x, y, z]
    data_xfrc.array[iteration, xfrc_index, i+3] = _torque_out[i]    # Torque [x, y, z]
return True
```

The xfrc array stores 6 values per link per iteration: `[fx, fy, fz, tx, ty, tz]`.

## `SwimmingHandler` — per-simulation configuration

```cython
cdef class SwimmingHandler:
    cdef object links              # LinkSensorArrayCy
    cdef object xfrc               # XfrcArrayCy
    cdef bint drag                 # Whether drag is enabled
    cdef bint sph                  # Whether SPH is enabled
    cdef bint buoyancy             # Whether buoyancy is enabled
    cdef bint use_exact_cob        # Whether to use exact COB (not ramp)
    cdef bint force_mesh           # Whether to use mesh-based COB
    cdef WaterProperties water     # Water properties object
    cdef DTYPEv1 masses            # Per-link masses
    cdef DTYPEv1 bound_radii       # Per-link bounding sphere radii
    cdef DTYPEv1 densities         # Per-link body densities
    cdef DTYPEv2 z3                # Scratch arrays [12, 3]
    cdef DTYPEv2 z4                # Scratch arrays [7, 4]
    cdef DTYPEv3 links_coefficients  # Per-link drag coefficients
```

### Constructor walkthrough

**Step 1: Water properties** (lines 400–405)

```cython
self.water = WaterPropertiesConstant(
    surface=float(water_options.height),
    density=float(water_options.density),
    velocity=np.array(water_options.velocity, dtype=float),
    viscosity=float(water_options.viscosity),
) if water is None else water
```

If no custom water properties are passed, creates a `WaterPropertiesConstant` from the arena's water options. The `SwimmingExtension` may pass a `WaterPropertiesExtension` with spatially-varying velocity.

**Step 2: Scratch arrays** (lines 409–410)

```cython
self.z3 = np.zeros([12, 3])  # 12 rows of 3-element scratch
self.z4 = np.zeros([7, 4])   # 7 rows of 4-element scratch (quaternions)
```

Pre-allocated once and reused every step. This avoids per-step allocation.

**Step 3: COB method** (lines 419–423)

```cython
cob_options = CobOptions.from_water_options(water_options)
self.cob_method = cob_options.method
self.use_exact_cob = self.cob_method != 'ramp'
self.force_mesh = self.cob_method == 'mesh'
```

The center-of-buoyancy method is resolved once into two booleans:
- `use_exact_cob`: True for `'mesh'` and `'analytic'` methods, False for `'ramp'`
- `force_mesh`: True only for `'mesh'` method

This avoids string comparison in the hot loop.

**Step 4: Link filtering** (lines 425–460)

```cython
links = [
    link for link in self.animat_options.morphology.links
    if link.fluid_interaction
]
```

Only links with `fluid_interaction=True` are included in the swimming computation. This allows non-fluid links (e.g., sensors) to be excluded.

**Step 5: Per-link properties** (lines 431–460)

| Property | Source | Description |
|---|---|---|
| `masses` | `physics.model.body_mass` | Link mass from MuJoCo model |
| `bound_radii` | `physics.model.geom_rbound` | Bounding sphere radius from geom group 2 |
| `densities` | `link.density` | Body density from animat options |
| `xfrc_indices` | `self.xfrc.names.index(link.name)` | Index into xfrc sensor array |
| `links_indices` | `self.links.names.index(link.name)` | Index into link sensor array |
| `links_coefficients` | `link.drag_coefficients` | Per-link drag coefficients |

The `bound_radii` are extracted from geoms in collision group 2 (the swimming collision group). The code takes the first matching geom's `geom_rbound` (which is already a radius, not a diameter).

**Step 6: Collision primitives** (lines 467–480)

```cython
if self.use_exact_cob:
    from .geom_utils import gather_link_collision_primitives
    self.link_primitives = [
        gather_link_collision_primitives(
            physics=physics, link_name=link.name, prefix=prefix,
            meters=self.meters, mesh_resolution=mesh_resolution,
        )
        for link in links
    ]
else:
    self.link_primitives = [None]*self.n_links
```

For exact COB methods, gathers collision primitives (geom type, size, mesh) for each link. This is needed by both `'mesh'` (which tessellates the surface) and `'analytic'` (which uses closed-form solutions for simple shapes).

**Step 7: SPH mode** (lines 482–483)

```cython
if self.sph:
    self.water._surface = 1e8
```

In SPH mode, the water surface is set to a very high value so all links are "underwater."

### `step()` method

```cython
cpdef step(self, double time, unsigned int iteration):
    if self.drag or self.sph or self.buoyancy:
        for i in range(self.n_links):
            if self.drag or self.buoyancy:
                in_water = apply_swimming_forces(
                    time=time, iteration=iteration,
                    data_links=self.links, links_index=self.links_indices[i],
                    data_xfrc=self.xfrc, xfrc_index=self.xfrc_indices[i],
                    coefficients=self.links_coefficients[i],
                    z3=self.z3, z4=self.z4,
                    water=self.water,
                    mass=self.masses[i], bound_radius=self.bound_radii[i],
                    density=self.densities[i], gravity=-9.81,
                    use_buoyancy=self.buoyancy,
                    primitives=self.link_primitives[i],
                    use_exact_cob=self.use_exact_cob,
                    force_mesh=self.force_mesh,
                )
```

Note: `gravity=-9.81` is hardcoded, NOT taken from the simulation options. The gravity from `setup_mjcf_xml` is used by MuJoCo's solver, but the hydrodynamics always uses -9.81 for buoyancy computation.

## `SwimmingExtension` — Python integration

```python
class SwimmingExtension(AnimatExtension):
    def __init__(self, animat_i, animat_data, animat_options, arena_options, substep=True, water_properties=None):
        super().__init__(substep=substep)
        self.animat_i = animat_i
        self.animat_data = animat_data
        self.animat_options = animat_options
        self.arena_options = arena_options
        self._handler = None
        self._water_properties = water_properties
```

### Water map loading

If `arena_options.water.velocity` has more than 3 elements, it's interpreted as a spatially-varying velocity field:

```python
self.constant_velocity = len(arena_options.water.velocity) == 3
if not self.constant_velocity:
    water_maps = [os.path.expandvars(p) for p in arena_options.water.maps]
    pngs = [np.flipud(imread(water_maps[i])).T for i in range(2)]
    # Normalize PNG pixel values to velocity range
    vels = [
        (png.astype(np.double) - info.min) * (vel_max - vel_min) / (info.max - info.min) + vel_min
        for png_i, (png, info) in enumerate(zip(pngs, pngs_info))
    ]
    self.water_maps = {
        'pos_min': np.array(water_velocity[6:8]),
        'pos_max': np.array(water_velocity[8:10]),
        'vel_x': +vels[0],
        'vel_y': -vels[1],  # Y is flipped
    }
    self._water_properties = WaterPropertiesExtension(
        surface=maps_surface_callback(float(wtr_options.height)),
        density=maps_density_callback(float(wtr_options.density)),
        viscosity=maps_viscosity_callback(float(wtr_options.viscosity)),
        velocity=maps_velocity_callback(self.water_maps),
    )
```

The velocity field is loaded from two PNG images (`vel_x` and `vel_y`). The `water.velocity` array format is: `[vel_x_min, vel_x_max, vel_y_min, vel_y_max, _, _, pos_x_min, pos_y_min, pos_x_max, pos_y_max]`.

### `water_velocity_from_maps()`

```python
def water_velocity_from_maps(position, water_maps):
    vel = np.zeros(3)
    if all(water_maps['pos_min'][i] < position[i] < water_maps['pos_max'][i] for i in range(2)):
        vel[:2] = [
            water_maps[png][tuple(
                min(water_maps[png].shape[index]-1,
                    round(water_maps[png].shape[index] * (
                        (position[index] - water_maps['pos_min'][index])
                        / (water_maps['pos_max'][index] - water_maps['pos_min'][index])
                    ))
                )
                for index in range(2)
            )]
            for png in ['vel_x', 'vel_y']
        ]
    return vel
```

Uses nearest-neighbor interpolation (not bilinear) to look up velocity from the PNG maps. The position is normalized to [0, 1] within the map bounds, then scaled to pixel indices and rounded.

### `before_step()`

```python
def before_step(self, task, action, physics):
    del action
    buff_iter = task.iteration % task.buffer_size
    self._handler.step(physics.time()/task.units.seconds, buff_iter)

    # Write forces to MuJoCo
    indices = task.maps[self.animat_i]['sensors']['data2xfrc']
    physics.data.xfrc_applied[indices, :] = (
        self.animat_data.sensors.xfrc.array[buff_iter, :, sc.xfrc_force_x:sc.xfrc_torque_z+1]
    )
    # Rotate from local to global frame
    for force_i, (rotation_mat, force_local) in enumerate(zip(
        physics.data.xmat[indices], physics.data.xfrc_applied[indices]
    )):
        physics.data.xfrc_applied[indices[force_i]] = (
            rotation_mat.reshape([3, 3]) @ force_local.reshape([3, 2], order='F')
        ).flatten(order='F')
    physics.data.xfrc_applied[indices, :3] *= task.units.newtons
    physics.data.xfrc_applied[indices, 3:] *= task.units.torques
```

!!! danger "This is a confirmed double rotation — a real bug, not just a docs ambiguity"
    Three independent facts pin this down (no longer "needs verification"):

    1. **`compute_link_forces` already outputs world-frame vectors.** Its
       last two lines are `quat_rot(force, urdf2global, ...)` /
       `quat_rot(torque, urdf2global, ...)`, and the function's own
       docstring says "combines drag + buoyancy for one link... Writes into
       `force_out`/`torque_out`" after "rotates the result to world frame."
    2. **`urdf2global` and `physics.data.xmat` encode the *same* rotation.**
       `urdf2global` comes from `data_links.urdf_orientation_cy()`, which is
       populated in `simulation/physics.py`'s `physicslinks2data()` directly
       from `physics.data.xquat[...]` (reordered to `[x,y,z,w]`) — the same
       per-body world orientation `xmat` is a 3×3 rotation-matrix view of.
       So `xmat.reshape([3,3]) @ v` and `quat_rot(v, urdf2global)` rotate a
       local-frame vector into world frame by the same amount.
    3. **MuJoCo's `xfrc_applied` is defined in the world frame**, not the
       body frame (confirmed both by the MuJoCo team and by this project's
       own code comment "Local to global frame" immediately preceding the
       rotation — which is precisely the operation `compute_link_forces`
       already performed).

    Net effect: `before_step()` takes a vector that
    `compute_link_forces` has *already* rotated from body frame to world
    frame, and rotates it a **second** time by the same body→world
    rotation. The only case where this is harmless is when a link's
    orientation is the identity rotation (`xmat` = identity) — e.g. a body
    that hasn't rotated away from its spawn orientation. For any link that
    has actually rotated (which, in an undulatory swimmer, is *every*
    segment except possibly the head at t=0), the applied force direction
    is wrong. This is a strong candidate to check first for orientation-
    dependent force errors (forces that look right when the animat is
    level and wrong once it pitches/rolls/yaws) such as
    body-segment-dependent instability in multi-link swimming animats.

    **The likely fix** (not yet applied — this documents the bug, it
    doesn't patch the source) is to remove the `xmat` rotation loop
    entirely and assign `physics.data.xfrc_applied[indices, :]` directly
    from `self.animat_data.sensors.xfrc.array[...]`, since that array is
    already in world frame by the time `before_step` reads it.

## How to integrate: custom water properties

```python
from farms_mujoco.swimming.hydrodynamics import WaterPropertiesExtension

def my_surface(t, x, y):
    return 0.5 * np.sin(0.1 * x)  # Wavy surface

def my_density(t, x, y, z):
    return 1000 + 0.5 * z  # Density gradient

def my_velocity(t, x, y, z):
    return np.array([0.1 * y, 0, 0])  # Shear flow

def my_viscosity(t, x, y, z):
    return 1.5  # Higher viscosity

water = WaterPropertiesExtension(
    surface=my_surface,
    density=my_density,
    velocity=my_velocity,
    viscosity=my_viscosity,
)

extension = SwimmingExtension(
    animat_i=0, animat_data=data, animat_options=options,
    arena_options=arena, water_properties=water,
)
```

## How to integrate: adding a new force type

To add a new hydrodynamic force (e.g., lift):

1. Create `lift_cy.pyx` with a `compute_link_lift_fast()` function.
2. In `hydrodynamics.pyx`, add a `cimport` from the new module.
3. In `compute_link_forces()`, call the new function and add its result to the combined force/torque.
4. Add a `use_lift` boolean to `SwimmingHandler` and the `apply_swimming_forces` parameters.
5. Add a `lift` option to the water options YAML schema.

## Common failure modes

### 1. Gravity hardcoded to -9.81

`SwimmingHandler.step()` passes `gravity=-9.81` to `apply_swimming_forces`, regardless of the simulation's gravity setting. If the simulation uses a different gravity (e.g., for a different planet), the buoyancy will be wrong.

### 2. `bound_radii` from geom group 2

The bounding radius is extracted from the first geom in collision group 2. If the animat's SDF doesn't have geoms in group 2, the list comprehension will fail with an `IndexError` (empty list `[0]`).

### 3. Double rotation of forces (confirmed bug — see `before_step()` above)

`compute_link_forces` rotates drag+buoyancy forces to the world frame.
`SwimmingExtension.before_step` then applies the same body→world rotation
(via `xmat`) a second time before handing the result to MuJoCo. Verified
against `physics.py`'s sensor population code (`urdf2global` and `xmat` are
sourced from the same `xquat`) and MuJoCo's own semantics (`xfrc_applied` is
world-frame). Force/torque direction is wrong for any link not at identity
orientation.

### 4. SPH surface override

When `sph=True`, `self.water._surface = 1e8` is set. This directly modifies the `WaterPropertiesConstant` object. If the same water object is shared between animats, this side effect will affect all of them.

### 5. Nearest-neighbor velocity interpolation

The water velocity map uses nearest-neighbor interpolation (rounding to pixel indices), not bilinear. This can cause discontinuities in the velocity field at pixel boundaries.

### 6. Dead computation: `com2urdf`/`urdf2com`/`com2global` are computed but never used

`link_swimming_info()` computes `com2global`, `com2urdf` (via
`quat_mult(global2urdf, com2global, ...)`), and `urdf2com` (via
`quat_conj(com2urdf, ...)`) every call. None of the three are read again —
`compute_link_forces` only passes `global2urdf`/`urdf2global` onward to
`compute_link_buoyancy_fast`/`compute_link_drag_fast`. This is 3 quaternion
ops (1 conjugate, 1 multiply, 1 conjugate) wasted per link per step; not a
correctness bug, just unnecessary work in a hot loop. `com_position` (used
by the exact-COB buoyancy path) is read directly from
`data_links.com_position_cy()`, which already returns world-frame
coordinates — it does not need `com2urdf`/`urdf2com` at all.

## What NOT to assume

1. **`drag.pyx` is NOT the only hydrodynamics file.** The old docs incorrectly attributed all hydrodynamics to `drag.pyx`. The orchestration is in `hydrodynamics.pyx`, with drag and buoyancy in separate files.

2. **`bound_radii` is NOT half-height.** It's the bounding sphere radius from `physics.model.geom_rbound`, which is already a radius. The comment in the source says: "already a radius, not a diameter -- do not halve."

3. **Gravity is hardcoded.** The `-9.81` in `SwimmingHandler.step()` is not configurable via YAML.

4. **Only `fluid_interaction` links are included.** Links without `fluid_interaction=True` in their morphology options are completely skipped by the hydrodynamics.

5. **The `z3` scratch array has 12 rows, not 10.** Two extra rows (10, 11) were added for `apply_swimming_forces`' dedicated `force_out`/`torque_out` scratch.

6. **Water velocity maps use nearest-neighbor, not bilinear interpolation.** The `round()` call in `water_velocity_from_maps` rounds to the nearest pixel.

7. **`before_step()` double-rotates forces into an incorrect frame for any non-identity link orientation** (confirmed bug, not a docs ambiguity — see "Common failure modes" #3 above and the `!!! danger` block under `before_step()`).
