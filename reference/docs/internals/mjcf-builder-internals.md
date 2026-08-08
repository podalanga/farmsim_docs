# MJCF Builder Internals

This page documents the MJCF builder (`farms_mujoco/simulation/mjcf.py`, 1732 lines) in detail. This module converts SDF model files into MuJoCo MJCF XML, creates actuators and sensors, sets up keyframes, configures physics options, and assembles the complete simulation scene.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_mujoco/simulation/mjcf.py` | 1732 | SDF→MJCF conversion, actuator/sensor creation, scene setup |
| `farms_core/io/sdf.py` | — | `ModelSDF`, `Link`, `Mesh`, `Visual`, `Collision` (SDF parsing) |
| `farms_core/model/options.py` | — | `SpawnMode`, `AnimatOptions`, `MorphologyOptions` |
| `farms_core/model/control.py` | — | `ControlType` (POSITION=0, VELOCITY=1, TORQUE=2, MUSCLE=3) |

## Call graph / entry points

```
Simulation.run()
  └─ setup_mjcf_xml(experiment_options, ...)
       ├─ sdf2mjcf(arena_sdf, fixed_base=True, ...)     [Arena → MJCF]
       ├─ sdf2mjcf(water_sdf, ...)                      [Water plane → MJCF (optional)]
       ├─ for animat_i, animat_options in animats:
       │    sdf2mjcf(animat_sdf, prefix=get_prefix(animat_i), ...)  [Animat → MJCF]
       │    └─ add_link_recursive(root, ...)            [Tree traversal]
       │         └─ mjc_add_link(link, ...)              [Per-link: body, joint, geoms, inertial]
       │    └─ Keyframe setup, actuator creation, sensor creation
       └─ Compiler, visual, physics options configuration
       └─ Joint stiffness/damping/muscle passive parameters
       └─ mjcf2str(mjcf_model)                          [XML serialization]
```

## Utility functions

### `get_prefix(animat_i)`

```python
def get_prefix(animat_i):
    """Get animat prefix"""
    return f'a{animat_i}_'
```

Returns the namespace prefix for multi-animat simulations. Animat 0 gets `a0_`, animat 1 gets `a1_`, etc. All link, joint, actuator, and sensor names are prefixed to avoid collisions.

### `quat2mjcquat(quat)`

```python
def quat2mjcquat(quat):
    quat = np.array(quat)[[3, 0, 1, 2]]
    return quat_type(quat)
```

Converts from `[x, y, z, w]` (scipy convention) to `[w, x, y, z]` (MuJoCo convention). The index reordering `[3, 0, 1, 2]` moves the scalar component to the front.

### `euler2mjcquat(euler)`

```python
def euler2mjcquat(euler):
    return quat2mjcquat(Rotation.from_euler(angles=euler, seq='xyz').as_quat())
```

Converts XYZ Euler angles to MuJoCo quaternion. Uses scipy's `Rotation` class with `seq='xyz'`.

### `get_local_transform(parent_pose, child_pose)`

Computes the local transform of a child link relative to its parent. Returns `(local_pos, local_euler)` — the position and XYZ Euler angles of the link in the parent's frame.

If `parent_pose` is `None`, the parent transform is identity (world frame).

## `mjc_add_link()` — per-link conversion

```python
def mjc_add_link(mjcf_model, mjcf_map, sdf_link, prefix='', **kwargs):
```

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `sdf_parent` | None | Parent SDF link |
| `mjc_parent` | None | Parent MJCF body (defaults to worldbody) |
| `sdf_joint` | None | Joint connecting this link to parent |
| `directory` | '' | Directory for mesh files |
| `spawn_mode` | None | Base link spawn mode (FREE, FIXED, etc.) |
| `contype` | 1 | Collision type bitmask |
| `conaffinity` | 2^31-1 | Collision affinity bitmask |
| `concave` | False | Whether to use concave mesh collision |
| `units` | SimulationUnitScaling() | Unit scaling |
| `use_site` | False | Whether to add sites for force-torque sensors |
| `friction` | [0, 0, 0] | Friction coefficients |

### Walkthrough

**Step 1: Body creation** (lines 170–184)

```python
link_name = sdf_link.name
link_local_pos, link_local_euler = get_local_transform(
    parent_pose=None if sdf_parent is None else sdf_parent.pose,
    child_pose=sdf_link.pose,
)
body = mjc_parent.add('body', name=f'{prefix}{link_name}', pos=..., quat=...)
mjcf_map['links'][f'{prefix}{link_name}'] = body
```

Each SDF link becomes a MuJoCo body. The position is computed relative to the parent and scaled by `units.meters`.

**Step 2: Base link spawn mode** (lines 190–274)

If the link is a `ModelSDF` (the root), the spawn mode determines the base joint:

| SpawnMode | Joint types | Axes |
|---|---|---|
| `FREE` | `freejoint` | — |
| `ROTX` | `hinge` | `[[1,0,0]]` |
| `ROTY` | `hinge` | `[[0,1,0]]` |
| `ROTZ` | `hinge` | `[[0,0,1]]` |
| `SAGITTAL` | `slide, slide, hinge` | `[[1,0,0], [0,0,1], [0,1,0]]` |
| `CORONAL` | `slide, slide, hinge` | `[[0,1,0], [0,0,1], [1,0,0]]` |
| `TRANSVERSE` | `slide, slide, hinge` | `[[1,0,0], [0,1,0], [0,0,1]]` |
| `SAGITTAL0` | `slide, slide` | `[[1,0,0], [0,0,1]]` |
| `CORONAL0` | `slide, slide` | `[[0,1,0], [0,0,1]]` |
| `TRANSVERSE0` | `slide, slide` | `[[1,0,0], [0,1,0]]` |
| `SAGITTAL3` | `slide, slide, hinge, hinge, hinge` | `[[1,0,0], [0,0,1], [1,0,0], [0,1,0], [0,0,1]]` |
| `CORONAL3` | `slide, slide, hinge, hinge, hinge` | `[[0,1,0], [0,0,1], [1,0,0], [0,1,0], [0,0,1]]` |
| `TRANSVERSE3` | `slide, slide, hinge, hinge, hinge` | `[[1,0,0], [0,1,0], [1,0,0], [0,1,0], [0,0,1]]` |
| `FIXED` | (none) | — |

For multi-DOF spawn modes (e.g., SAGITTAL), intermediate bodies are created: `root0_b_{name}`, `root1_b_{name}`, etc., each with one joint.

**Step 3: Child link joints** (lines 277–300)

For non-root links with a parent joint:
- `revolute` / `continuous` → `hinge` joint
- `prismatic` → `slide` joint

The joint axis is rotated by the joint's Euler pose: `euler2mat(sdf_joint.pose[3:]) @ sdf_joint.axis.xyz`.

**Step 4: Geometries** (lines 300–600)

For each visual and collision element on the SDF link, the corresponding MJCF geom is created. Geometry types: `Box`, `Cylinder`, `Capsule`, `Sphere`, `Plane`, `Mesh`, `Heightmap`.

Collision geoms get `contype` and `conaffinity` bitmasks. Visual geoms get `contype=0, conaffinity=0` (no collision).

**Step 5: Inertial properties** (lines 650–715)

```python
body.add('inertial',
    pos=[...],
    quat=...,
    mass=max(MIN_MASS, inertial.mass)*units.kilograms,
    fullinertia=[
        max(MIN_INERTIA, inertia_mat[0][0])*units.inertia,
        max(MIN_INERTIA, inertia_mat[1][1])*units.inertia,
        max(MIN_INERTIA, inertia_mat[2][2])*units.inertia,
        inertia_mat[0][1]*units.inertia,
        inertia_mat[0][2]*units.inertia,
        inertia_mat[1][2]*units.inertia,
    ],
)
```

Mass and inertia are clamped to `MIN_MASS` (1e-15) and `MIN_INERTIA` (1e-15) to avoid MuJoCo issues with zero-inertia bodies. The `fullinertia` format is `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`.

!!! note "The inertial `quat` is never set — the tensor is pre-rotated instead"
    The source has a commented-out `quat=euler2mjcquat(inertial.pose[3:])` line with the
    note `# Not working in MuJoCo?`. Instead of relying on MJCF's `<inertial quat=.../>`
    to orient the inertia tensor, the code rotates the full 3×3 inertia tensor in Python
    **before** writing it out:
    ```python
    rot_mat = euler2mat(inertial.pose[3:])
    inertia_mat = rot_mat @ inertia_mat @ rot_mat.T
    ```
    and only ever emits `fullinertia` in the body's own (unrotated) frame. This means:
    the eigenvalue-positivity assertion (`eigvals > 0`) runs on the *un-rotated* diagonal
    matrix built from `inertial.inertias`, then the rotation is applied afterward — so a
    physically valid (positive-definite) tensor stays valid under rotation, but if you
    ever add a code path that sets `quat=` on the `inertial` element directly, you will
    double-rotate the tensor. This is the same root bug pattern documented for `zbot`'s
    SDF/MuJoCo inertia export (see the SDF fidelity notes in `reference/core/farms-core.md`):
    diagonalizing to principal axes and then discarding or duplicating the frame
    rotation silently corrupts mass distribution. Here it is done correctly — this note
    exists to stop a future edit from "fixing" it by uncommenting the `quat=` line.

## `add_link_recursive()`

```python
def add_link_recursive(mjcf_model, mjcf_map, sdf, **kwargs):
    sdf_link = kwargs.pop('sdf_link')
    sdf_parent = kwargs.pop('sdf_parent', None)
    sdf_joint = kwargs.pop('sdf_joint', None)

    mjc_add_link(mjcf_model, mjcf_map, sdf_link=sdf_link, ...)

    for child in sdf.get_children(link=sdf_link):
        add_link_recursive(
            mjcf_model=mjcf_model, mjcf_map=mjcf_map, sdf=sdf,
            sdf_link=child, sdf_parent=sdf_link,
            sdf_joint=sdf.get_parent_joint(link=child),
            ...
        )
```

Recursively traverses the SDF link tree, adding each link and its children. The `sdf.get_children(link=...)` and `sdf.get_parent_joint(link=...)` methods from `ModelSDF` provide the tree structure.

## `sdf2mjcf()` — full model conversion

```python
def sdf2mjcf(sdf, **kwargs) -> (mjcf.RootElement, Dict):
```

### Key parameters

| Parameter | Default | Description |
|---|---|---|
| `prefix` | '' | Name prefix (from `get_prefix`) |
| `mjcf_model` | None | Existing model to add to (for multi-model scenes) |
| `fixed_base` | False | If True, use `SpawnMode.FIXED` |
| `use_sensors` | False | Whether to add MuJoCo sensors |
| `use_actuators` | False | Whether to create actuators |
| `use_muscles` | False | Whether to create Hill muscle actuators |
| `animat_options` | None | Animat configuration (for actuators, keyframes) |
| `simulation_options` | None | Simulation configuration (for units, texture) |

### Walkthrough

**Step 1: Model initialization** (lines 819–846)

Creates a `mjcf.RootElement` if none provided. Initializes `mjcf_map` with empty dicts for `links`, `joints`, `sites`, `visuals`, `collisions`, `actuators`, `tendons`, `muscles`.

**Step 2: Root link and tree** (lines 848–878)

Adds the model root link (the `ModelSDF` itself), then recursively adds all base links and their children.

**Step 3: Keyframes** (lines 880–949)

If `animat_options` is provided, creates a keyframe with initial joint positions and velocities:

```python
for joint in mjcf_model.find_all('joint'):
    joint_name_index_pos[joint.name] = pos_index
    pos_index += 7 if joint.tag == 'freejoint' else 1
    joint_name_index_vel[joint.name] = vel_index
    vel_index += 6 if joint.tag == 'freejoint' else 1
```

Free joints take 7 qpos entries (3 position + 4 quaternion) and 6 qvel entries (3 linear + 3 angular). Regular joints take 1 qpos and 1 qvel.

Initial joint positions come from `joint.initial[0]` and velocities from `joint.initial[1]` in the animat options.

For free base spawn, the spawn pose and velocity from `animat_options.spawn` are written into the keyframe.

**Step 4: Actuators** (lines 951–1130)

For each joint in `animat_options.control.joints_names()`, THREE actuators are created:

1. **Position actuator** (`actuator_position_{prefix}{joint}`):
   - Type: `position`
   - `kp` = `motor.gains[0] * units.torques` (proportional gain)
   - `kv` = `motor.gains[1] * units.angular_damping` (derivative gain)
   - Group: `ControlType.POSITION` (0)

2. **Velocity actuator** (`actuator_velocity_{prefix}{joint}`):
   - Type: `velocity`
   - `kv` = `motor.gains[2] * units.angular_damping` (velocity gain)
   - Group: `ControlType.VELOCITY` (1)

3. **Torque actuator** (`actuator_torque_{prefix}{joint}`):
   - Type: `motor`
   - Group: `ControlType.TORQUE` (2)

If `motor.limits_torque` is set, all three actuators get `forcelimited=True` and `forcerange=[min, max] * units.torques`.

**Adhesion actuators** (lines 1036–1047): If `animat_options.control.adhesions` exists, creates `adhesion` actuators with `ctrlrange=[0.999*force, force]`.

**Hill muscle actuators** (lines 1049–1129): If `use_muscles=True`, creates `general` actuators with `dyntype='muscle'`, `gaintype='user'`, `biastype='user'`. The `gainprm`/`biasprm` arrays contain: `[max_force, optimal_fiber, tendon_slack, max_velocity*optimal_fiber, pennation_angle]`. The `user` array contains proprioceptive sensor parameters (Type Ia, II, Ib).

**Step 5: Sensors** (lines 1131–1196)

| Sensor type | Condition | Object |
|---|---|---|
| `framepos`, `framequat` | `use_link_sensors` | Each link body |
| `touch` | `use_site` | Each link site |
| `framelinvel`, `frameangvel` | `use_link_vel_sensors` | Each link body |
| `jointpos`, `jointvel`, `jointlimitfrc` | `use_joint_sensors` | Each joint |
| `force`, `torque` | `use_frc_trq_sensors` | Each joint site |
| `actuatorfrc` | `use_actuator_sensors` | Each actuator |
| `actuatorfrc` (muscle) | `use_muscle_sensors` | Each muscle actuator |

**Step 6: Self-collision pairs** (lines 1198–1220)

```python
for pair_i, (link1, link2) in enumerate(animat_options.morphology.self_collisions):
    for col1_name in collision_map[link1]:
        for col2_name in collision_map[link2]:
            mjcf_model.contact.add('pair', geom1=..., geom2=..., condim=6, friction=[0]*5)
```

Creates explicit contact pairs for self-collisions defined in the animat options. `condim=6` enables full 6-DOF contact (friction + torsional + rolling).

## `setup_mjcf_xml()` — complete scene assembly

```python
def setup_mjcf_xml(experiment_options, **kwargs) -> (mjcf.RootElement, list, dict):
```

### Walkthrough

**Step 1: Arena** (lines 1388–1407)

Converts the arena SDF to MJCF with `fixed_base=True`. Sets the arena position from `arena_options.spawn.pose`. Adjusts for `ground_height` if set.

**Step 2: Water** (lines 1408–1422)

If `arena_options.water.height` is set, converts the water SDF to MJCF. The water body is positioned at the water surface height. Water has `contype=0, conaffinity=0` (no collision — it's visual only).

**Step 3: Animats** (lines 1426–1450)

For each animat:
- Reads the animat SDF
- Calls `sdf2mjcf` with `prefix=get_prefix(animat_i)`, `use_actuators=True`, `use_sensors=True`
- Sets `contype=2^(animat_i+1)` so each animat has a unique collision group
- `conaffinity=2*31-1` (note: this is `2*31-1 = 61`, NOT `2^31-1` — this is likely a bug, should be `2**31-1`)

**Step 4: Compiler options** (lines 1452–1463)

```python
mjcf_model.compiler.angle = 'radian'
mjcf_model.compiler.eulerseq = 'xyz'
mjcf_model.compiler.boundmass = MIN_MASS * units.kilograms
mjcf_model.compiler.boundinertia = MIN_INERTIA * units.inertia
mjcf_model.compiler.balanceinertia = False
mjcf_model.compiler.inertiafromgeom = False
mjcf_model.compiler.fusestatic = True
mjcf_model.compiler.lengthrange.mode = "none"  # Disable for muscles
```

**Step 5: Physics options** (lines 1519–1601)

| Option | Source | Default |
|---|---|---|
| `timestep` | `simulation_options.physics.timestep / substeps` | 1e-3 |
| `gravity` | `simulation_options.physics.gravity` | [0, 0, -9.81] |
| `integrator` | `simulation_options.mujoco.integrator` | 'Euler' |
| `solver` | `simulation_options.mujoco.solver` | 'Newton' |
| `iterations` | `simulation_options.physics.n_solver_iters` | 1000 |
| `cone` | `simulation_options.mujoco.cone` | 'pyramidal' |
| `impratio` | `simulation_options.mujoco.impratio` | 1 |
| `ccd_iterations` (MuJoCo >= 323) | `simulation_options.mujoco.ccd_iterations` | 1000 |
| `mpr_iterations` (MuJoCo < 323) | `simulation_options.mujoco.mpr_iterations` | 1000 |
| `noslip_iterations` | `simulation_options.mujoco.noslip_iterations` | 0 |

**Step 6: Per-animat post-processing** (lines 1603–1694)

For each animat:

1. **Spawn position**: Sets the base link position and orientation from `animat_options.spawn.pose`.

2. **Link friction**: For each link, sets `geom.friction` from `link.friction` and disables MuJoCo's built-in fluid model (`fluidshape=None, fluidcoef=[0,0,0,0,0]`).

3. **Joint properties**: Sets `springref`, adds `stiffness` and `damping` from joint options. Also handles `solreflimit`, `solimplimit`, and `margin` from joint extras.

4. **Passive muscle properties**: For Ekeberg muscle equations, adds `beta*gamma` to joint stiffness and `delta` to joint damping. This implements the passive component of the Ekeberg model at the MuJoCo level.

**Step 7: Visual setup** (lines 1480–1517)

Configures visual scaling for forces, contacts, actuators, etc. All scales use `simulation_options.mujoco.visual_scale`.

**Step 8: Lights, cameras, sky** (lines 1700–1717)

Adds a tracking light, 4 cameras (3 tracking + 1 fixed), and a gradient skybox to the first animat's base link.

**Step 9: XML serialization** (lines 1720–1731)

Calls `mjcf2str()` to produce the final XML string. Optionally saves to file.

## `mjcf2str()`

```python
def mjcf2str(mjcf_model, remove_temp=True):
    mjcf_xml = mjcf_model.to_xml()
    if remove_temp:
        # Strip unique identifiers from mesh/texture file paths
        for mesh in mjcf_xml.find('asset').findall('mesh'):
            mesh.attrib['file'] = mjcf_mesh.file.prefix + mjcf_mesh.file.extension
    xml_str = ET.tostring(mjcf_xml, encoding='utf8', method='xml').decode('utf8')
    dom = xml.dom.minidom.parseString(xml_str)
    return dom.toprettyxml(indent='  ')
```

Serializes the MJCF model to a pretty-printed XML string. The `remove_temp` option strips temporary directory prefixes from mesh and texture file paths, making the XML portable.

## How to integrate: adding a custom geom type

To add support for a new SDF geometry type (e.g., `ellipsoid`):

1. In `mjc_add_link`, add a handler in the collision/visual geometry section:

```python
elif isinstance(collision, Ellipsoid):
    geom = body.add('geom',
        type='ellipsoid',
        size=[r*units.meters for r in collision.radii],
        ...
    )
```

2. Ensure the SDF parser (`farms_core/io/sdf.py`) can parse the new geometry type.

3. No changes needed to `sdf2mjcf` or `setup_mjcf_xml` — they call `mjc_add_link` which handles all geometry types.

## How to integrate: adding a new actuator type

To add a new actuator type (e.g., a custom `muscle` actuator):

1. In `sdf2mjcf`, add the actuator creation block after the existing actuator section:

```python
if use_custom_actuator:
    for custom in animat_options.control.custom_actuators:
        name = f'actuator_custom_{prefix}{custom.joint_name}'
        mjcf_map['actuators'][name] = mjcf_model.actuator.add(
            'general',
            name=name,
            joint=f'{prefix}{custom.joint_name}',
            group=ControlType.CUSTOM,
            gaintype='user',
            gainprm=[...],
        )
```

2. Add a sensor for the new actuator in the sensor section.

3. In `ExperimentTask.initialize_control`, add the actuator name mapping.

## Common failure modes

### 1. `conaffinity` bug

Line 1398 and 1448: `conaffinity=2*31-1` (which is 61) instead of `2**31-1` (which is 2147483647). This means animats only collide with collision group 61, not all groups. This is likely a typo but may affect collision behavior.

### 2. Missing joints in SDF

If `animat_options.control.joints_names()` references a joint that doesn't exist in the SDF, the assertion at line 965 fails: `Joint "{joint_name}" required by animat options not found in newly created MJCF file.`

### 3. Zero inertia bodies

MuJoCo requires non-zero mass and inertia. The `MIN_MASS` and `MIN_INERTIA` constants (1e-15) prevent this, but if a link has no inertial properties in the SDF, the computed inertia may be wrong.

### 4. Spawn mode joint count mismatch

If the spawn mode creates N root joints but the keyframe setup expects a different count, the qpos/qvel arrays will be misaligned. The keyframe code handles free joints (7 qpos, 6 qvel) and regular joints (1 qpos, 1 qvel), but multi-DOF spawn modes create intermediate bodies with their own joints.

### 5. Mesh file path issues

`mjcf2str` with `remove_temp=True` strips directory prefixes from mesh paths. If the mesh files are not in the working directory when MuJoCo loads the XML, loading will fail.

## What NOT to assume

1. **`get_prefix` uses `a{i}_` format**, NOT `animat_{i}_`. The `ExperimentTask` uses `get_prefix` for naming, so all MuJoCo names use the `a{i}_` prefix.

2. **Three actuators are ALWAYS created per joint** (position, velocity, torque), regardless of the motor's `control_types`. The `ExperimentTask` later disables unused actuators via force limiting.

3. **The keyframe is named `"initial"`** and has `time=0.0`. `ExperimentTask.initialize_episode` resets to `keyframe_id=0`, which is this keyframe.

4. **`inertiafromgeom=False`** means MuJoCo uses the inertial properties from the SDF, not computed from geometry. If the SDF inertial properties are wrong, the simulation will have wrong dynamics.

5. **`fusestatic=True**` means MuJoCo fuses static bodies into their parents for efficiency. This changes the body count but not the dynamics.

6. **The `conaffinity` value `2*31-1`** is 61, not 2^31-1. This is likely a bug but has been in the codebase for a long time. Do not assume it means "collide with everything."
