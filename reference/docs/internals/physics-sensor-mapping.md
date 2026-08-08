# Physics Sensor Mapping Internals

This page documents how FARMS maps between MuJoCo's physics state and FARMS sensor data arrays (`farms_mujoco/simulation/physics.py`, 561 lines). This mapping is critical for performance — it is pre-computed once during initialization and then used as direct array index lookups during every simulation step.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_mujoco/simulation/physics.py` | 561 | `get_sensor_maps()`, `get_physics2data_maps()`, `physics2data()`, and helper functions |
| `farms_mujoco/sensors/sensors.py` | — | `cycontacts2data()`, `cymusclesensors2data()` (Cython sensor transfer) |
| `farms_core/sensors/sensor_convention.py` | — | `sc` (sensor convention constants for array column indices) |

## Call graph / entry points

```
ExperimentTask.initialize_episode()
  ├─ get_sensor_maps(physics)               — Build sensor type → index mapping
  ├─ get_physics2data_maps(physics, ...)     — Build physics state → data array mapping
  └─ (stored in self.maps[animat_i])

ExperimentTask.update_sensors()
  └─ physics2data(physics, iteration, data, maps, units)
       ├─ physicslinks2data()               — Link positions/orientations
       ├─ physicslinksvelsensors2data()      — Link velocities
       ├─ physicsjointssensors2data()       — Joint forces/torques/limits
       ├─ physicsjoints2data()              — Joint positions/velocities
       ├─ physicsactuators2data()           — Actuator forces
       ├─ cycontacts2data()                 — Contact forces (Cython)
       └─ physics_muscles_sensors2data()    — Muscle sensor data
```

## `get_sensor_maps(physics, verbose=True)`

Builds a dictionary mapping each MuJoCo sensor type prefix to its sensor names and data indices.

### Sensor types recognized

```python
sensors = [
    # Links
    'framepos', 'framequat', 'framelinvel', 'frameangvel',
    # Joints
    'jointpos', 'jointvel', 'jointlimitfrc',
    'force', 'torque',
    # Joints control
    'actuatorfrc_position', 'actuatorfrc_velocity', 'actuatorfrc_motor',
    # Muscles
    'musclefrc', 'tendonpos', 'tendonvel',
    'musclefiberlen', 'musclefibervel',
    'musclepenn', 'muscleactivefrc', 'musclepassivefrc',
    'muscleIa', 'muscleII', 'muscleIb',
    # Contacts
    'touch',
]
```

### How it works

```python
sensors_row = physics.named.data.sensordata.axes.row
sensors_names = sensors_row.names

sensor_maps = {
    sensor: {
        'names': [name for name in sensors_names if name.startswith(sensor)],
    }
    for sensor in sensors
}
```

For each sensor type, it filters all MuJoCo sensor names that start with that prefix. For example, `framepos` matches `framepos_link_body_0`, `framepos_link_body_1`, etc.

Then it converts each name to a data index:

```python
for sensor_info in sensor_maps.values():
    sensor_info['indices'] = np.array([
        [np.arange(id_slice.start, id_slice.stop, id_slice.step)
         for id_slice in [sensors_row.convert_key_item(name)]][0]
        for name in sensor_info['names']
    ])
```

`row2index()` (see below) converts MuJoCo named axis items to integer indices or slices.

### Return value

A dict keyed by sensor type string, each containing:
- `names`: list of sensor name strings
- `indices`: numpy array of index arrays (each sensor may occupy multiple data slots)

## `row2index(row, name, single=False)`

```python
def row2index(row, name, single=False):
    identifier = row.convert_key_item(name)
    if isinstance(identifier, slice):
        indices = [
            np.arange(id_slice.start, id_slice.stop, id_slice.step)
            for id_slice in [identifier]
        ][0]
        return indices[0] if single else indices
    return identifier
```

Converts a MuJoCo named axis key to an integer index or array of indices.

- If the name resolves to a **slice** (multi-element sensor), returns either the first index (`single=True`) or the full range.
- If the name resolves to a **single integer**, returns it directly.

**Usage**: `single=True` is used for joint-related lookups where each joint maps to exactly one data slot. `single=False` (default) is used for link-related lookups where each link may map to multiple data slots (e.g., 3 for position, 4 for quaternion).

## `get_physics2data_maps(physics, sensor_data, sensor_maps, prefix='')`

Builds the mapping from MuJoCo physics state arrays to FARMS data arrays. This is the core mapping function.

### Parameters

| Parameter | Type | Description |
|---|---|---|
| `physics` | `mujoco.Physics` | MuJoCo physics object |
| `sensor_data` | `AnimatData` | FARMS data container (provides link/joint/muscle names) |
| `sensor_maps` | dict | Output of `get_sensor_maps()` |
| `prefix` | str | Namespace prefix for MuJoCo names (used when animat is spawned as a sub-model) |

### Maps built

#### Link maps (lines 196–215)

| Map key | Source | Description |
|---|---|---|
| `xpos2data` | `physics.named.data.xpos` | Global link positions |
| `xquat2data` | `physics.named.data.xquat` | Global link orientations (quaternions) |
| `xipos2data` | `physics.named.data.xipos` | Center-of-mass link positions |
| `cvel2data` | `physics.named.data.cvel` | Center-of-mass link velocities |

Each maps link names to indices in the corresponding MuJoCo data array.

#### Joint maps (lines 217–227)

| Map key | Source | Description |
|---|---|---|
| `qpos2data` | `physics.named.data.qpos` | Joint positions |
| `qvel2data` | `physics.named.data.qvel` | Joint velocities |

Built with `single=True` because each joint maps to exactly one position/velocity slot.

#### Link sensor maps (lines 229–246)

For each of `framepos`, `framequat`, `framelinvel`, `frameangvel`:

```python
sensor_maps[f'{identifier}2data'] = np.array([
    sensor_maps[identifier]['indices'][
        sensor_maps[identifier]['names'].index(f'{identifier}_{prefix}{link_name}')
    ]
    for link_name in links_names
]) if all(
    f'{identifier}_{prefix}{link_name}' in sensor_maps[identifier]['names']
    for link_name in links_names
) else []
```

If ANY link name is missing from the sensor names, the entire map is set to empty list `[]`. This is a fail-safe: if sensors are not defined for all links, the corresponding data transfer is skipped.

**Quaternion reordering** (line 243–246):

```python
if len(sensor_maps['framequat2data']) > 0:
    sensor_maps['framequat2data'][:, :] = (
        sensor_maps['framequat2data'][:, [1, 2, 3, 0]]
    )
```

MuJoCo uses `[w, x, y, z]` quaternion ordering. FARMS uses `[x, y, z, w]` ordering. This line reorders the quaternion indices so that when data is read, it comes out in FARMS convention. The index array columns are permuted: column 0 (w) goes to position 3, and columns 1,2,3 (x,y,z) go to positions 0,1,2.

#### Joint sensor maps (lines 248–278)

For `jointpos`, `jointvel`, `jointlimitfrc`, `actuatorfrc_position`, `actuatorfrc_velocity`, `actuatorfrc_motor`: same pattern as link sensors but with `[0]` appended (single value per joint).

For `force` and `torque`: built by filtering names that exist in the row.

#### Muscle sensor maps (lines 280–343)

For `musclefrc`, `musclefiberlen`, `musclefibervel`, `musclepenn`, `muscleactivefrc`, `musclepassivefrc`, `muscleIa`, `muscleII`, `muscleIb`: same pattern.

**Tendon maps** (lines 300–309):

| Map key | Source |
|---|---|
| `tendonpos2data` | `physics.named.data.ten_length` |
| `tendonvel2data` | `physics.named.data.ten_velocity` |

**7-field muscle sensor map** (lines 311–343):

```python
sensor_maps['musclesensors2data'] = np.array([
    [
        row2index(row=physics.named.data.ctrl.axes.row, name=f'{prefix}{muscle_name}'),           # ctrl command
        row2index(row=physics.named.data.act.axes.row, name=f'{prefix}{muscle_name}'),             # activation
        row2index(row=physics.named.data.actuator_length.axes.row, name=f'{prefix}{muscle_name}'), # length
        row2index(row=physics.named.data.actuator_velocity.axes.row, name=f'{prefix}{muscle_name}'), # velocity
        row2index(row=physics.named.data.actuator_force.axes.row, name=f'{prefix}{muscle_name}'),  # force
        row2index(row=physics.named.model.actuator_gainprm.axes.row, name=f'{prefix}{muscle_name}'), # gain
        row2index(row=physics.named.model.actuator_user.axes.row, name=f'{prefix}{muscle_name}'),   # user data
    ]
    for muscle_name in muscles_names
])
```

Each muscle maps to 7 MuJoCo data fields:

| Index | Source | Description |
|---|---|---|
| 0 | `data.ctrl` | Control command (activation or position) |
| 1 | `data.act` | Muscle activation state |
| 2 | `data.actuator_length` | Muscle length |
| 3 | `data.actuator_velocity` | Muscle velocity |
| 4 | `data.actuator_force` | Muscle force |
| 5 | `model.actuator_gainprm` | Gain parameter |
| 6 | `model.actuator_user` | User-defined parameters |

This 7-field mapping is used by `cymusclesensors2data()` (Cython) to efficiently copy all muscle data in one call.

#### Contact maps (lines 362–394)

```python
contacts_pairs = [
    (prefix+name1 if name1 else name1, prefix+name2 if name2 else name2)
    for (name1, name2) in sensor_data.contacts.names
]
body_names = physics.named.model.body_pos.axes.row.names
sensor_maps['geompair2data'] = {
    (geom_id, -1): contacts_pairs.index((body_names[body_id], ''))
    for geom_id, body_id in enumerate(physics.model.geom_bodyid)
    if (body_names[body_id], '') in contacts_pairs
}
sensor_maps['geompair2data'].update({
    (geom_id1, geom_id2): contacts_pairs.index(
        (body_names[body_id1], body_names[body_id2])
    )
    for geom_id1, body_id1 in enumerate(physics.model.geom_bodyid)
    for geom_id2, body_id2 in enumerate(physics.model.geom_bodyid)
    if (body_names[body_id1], body_names[body_id2]) in contacts_pairs
})
```

Contact mapping works by:
1. Getting the list of contact pairs from `sensor_data.contacts.names` (pairs of body names).
2. For each geom in the model, finding its body name via `geom_bodyid`.
3. Building two types of mappings:
   - `(geom_id, -1)`: Single-body contacts (ground contact, where the second body is empty string).
   - `(geom_id1, geom_id2)`: Two-body contacts.

**Warning for missing pairs** (lines 382–394):

```python
for pair_i, pair in enumerate(contacts_pairs):
    assert not isinstance(pair, str) and len(pair) == 2, (...)
    if pair_i not in geompair2data_values:
        pylog.warning(f'WARNING: {pair=} was not found in collisions ...')
```

If a contact pair from the data is not found in the physics model's collisions, a warning is logged. This typically means the SDF model defines a contact sensor for a body pair that doesn't have collision enabled in MuJoCo.

#### External force maps (lines 396–405)

| Map key | Description |
|---|---|
| `data2xfrc` | Maps xfrc sensor names to MuJoCo xfrc indices |
| `datalinks2xfrc` | Maps link names to MuJoCo xfrc indices (for per-link external forces) |

## Data transfer functions

### `physics2data(physics, iteration, data, maps, units, links_only=False)`

The main per-step transfer function. Called by `ExperimentTask.update_sensors()`.

```python
def physics2data(physics, iteration, data, maps, units, links_only=False):
    sensor_maps = maps['sensors']
    physicslinks2data(physics, iteration, data, sensor_maps, units)
    physicslinksvelsensors2data(physics, iteration, data, sensor_maps, units)
    if not links_only:
        physicsjointssensors2data(physics, iteration, data, sensor_maps, units)
        physicsjoints2data(physics, iteration, data, sensor_maps, units)
        physicsactuators2data(physics, iteration, data, sensor_maps, units)
        cycontacts2data(physics=physics, iteration=iteration, data=data.sensors.contacts,
                        geompair2data=sensor_maps['geompair2data'],
                        meters=units.meters, newtons=units.newtons)
        if data.sensors.muscles.names:
            physics_muscles_sensors2data(physics, iteration, data, sensor_maps, units)
```

When `links_only=True`, only link position and velocity data is transferred. This is used for pre-physics sensor updates (e.g., computing initial link positions before the first physics step).

### `physicslinks2data()` — link positions and orientations

```python
def physicslinks2data(physics, iteration, data, sensor_maps, units):
    # URDF (link frame) positions
    data.sensors.links.array[iteration, :, sc.link_urdf_position_x:sc.link_urdf_position_z+1] = (
        physics.data.xpos[sensor_maps['xpos2data']] / units.meters
    )
    # URDF orientations (quaternion reordered w,x,y,z → x,y,z,w)
    data.sensors.links.array[iteration, :, sc.link_urdf_orientation_x:sc.link_urdf_orientation_w+1] = (
        physics.data.xquat[sensor_maps['xquat2data']][:, [1, 2, 3, 0]]
    )
    # CoM positions
    data.sensors.links.array[iteration, :, sc.link_com_position_x:sc.link_com_position_z+1] = (
        physics.data.xipos[sensor_maps['xipos2data']] / units.meters
    )
    # CoM orientations (same quaternion reorder)
    data.sensors.links.array[iteration, :, sc.link_com_orientation_x:sc.link_com_orientation_w+1] = (
        physics.data.xquat[sensor_maps['xquat2data']][:, [1, 2, 3, 0]]
    )
```

**Key details**:
1. Uses `sc.link_urdf_position_x` through `sc.link_urdf_position_z` for column indices — these are constants from `farms_core.sensors.sensor_convention`.
2. Quaternion reordering happens HERE (at data copy time), not at map build time. The map indices are NOT reordered, but the data is: `[:, [1, 2, 3, 0]]` swaps the w component to the end.
3. Both URDF frame (xpos/xquat) and CoM frame (xipos/xquat) orientations use the SAME quaternion indices (`xquat2data`). The CoM orientation is the same as the URDF orientation — only the position differs.
4. Units are divided: `units.meters` converts from MuJoCo's internal units to the desired unit system.

### `physicslinksvelsensors2data()` — link velocities from sensors

```python
def physicslinksvelsensors2data(physics, iteration, data, sensor_maps, units):
    data.sensors.links.array[iteration, :, sc.link_com_velocity_lin_x:sc.link_com_velocity_lin_z+1] = (
        physics.data.sensordata[sensor_maps['framelinvel2data']] / units.velocity
    )
    data.sensors.links.array[iteration, :, sc.link_com_velocity_ang_x:sc.link_com_velocity_ang_z+1] = (
        physics.data.sensordata[sensor_maps['frameangvel2data']] / units.angular_velocity
    )
```

Uses MuJoCo frame velocity sensors (`framelinvel`, `frameangvel`). These are sensor-based velocities, not state-based.

### `physicslinksvel2data()` — link velocities from state

```python
def physicslinksvel2data(physics, iteration, data, sensor_maps, units):
    data.sensors.links.array[iteration, :, sc.link_com_velocity_lin_x:sc.link_com_velocity_lin_z+1] = (
        physics.data.cvel[sensor_maps['cvel2data'], 3:] / units.velocity
    )
    data.sensors.links.array[iteration, :, sc.link_com_velocity_ang_x:sc.link_com_velocity_ang_z+1] = (
        physics.data.cvel[sensor_maps['cvel2data'], :3] / units.angular_velocity
    )
```

Uses `physics.data.cvel` (center-of-mass velocity). The `cvel` array has 6 components: `[ang_x, ang_y, ang_z, lin_x, lin_y, lin_z]`. Angular velocity is the first 3 components, linear is the last 3.

**Note**: This function is NOT called by `physics2data()`. It's an alternative velocity source that reads from the physics state directly rather than from sensors. It may be used in specific contexts where sensor-based velocities are not available.

### `physicsjoints2data()` — joint positions and velocities

```python
def physicsjoints2data(physics, iteration, data, sensor_maps, units):
    if len(sensor_maps['qpos2data']) > 0:
        data.sensors.joints.array[iteration, :, sc.joint_position] = (
            physics.data.qpos[sensor_maps['qpos2data']]
        )
        data.sensors.joints.array[iteration, :, sc.joint_velocity] = (
            physics.data.qvel[sensor_maps['qvel2data']] / units.angular_velocity
        )
```

Reads joint positions from `qpos` and velocities from `qvel`. Note that position is NOT divided by any unit (assumed to be in radians already), while velocity is divided by `units.angular_velocity`.

### `physicsactuators2data()` — actuator forces

```python
def physicsactuators2data(physics, iteration, data, sensor_maps, units):
    itorques = 1. / units.torques
    # Reset cumulative torque to avoid double-counting
    data.sensors.joints.array[iteration, :, sc.joint_torque] = 0.0
    if len(sensor_maps['actuatorfrc_position2data']) > 0:
        data.sensors.joints.array[iteration, :, sc.joint_torque] += (
            physics.data.sensordata[sensor_maps['actuatorfrc_position2data']]
        ) * itorques
    if len(sensor_maps['actuatorfrc_velocity2data']) > 0:
        data.sensors.joints.array[iteration, :, sc.joint_torque] += (
            physics.data.sensordata[sensor_maps['actuatorfrc_velocity2data']]
        ) * itorques
    if len(sensor_maps['actuatorfrc_motor2data']) > 0:
        data.sensors.joints.array[iteration, :, sc.joint_torque] += (
            physics.data.sensordata[sensor_maps['actuatorfrc_motor2data']]
        ) * itorques
```

**Key detail**: The total torque is computed by ADDING three actuator force components: position-based, velocity-based, and motor-based. The array is zeroed first to prevent accumulation across iterations (since this function may be called multiple times per step in multi-substep scenarios).

### `physics_muscles_sensors2data()` — muscle data

```python
def physics_muscles_sensors2data(physics, iteration, data, sensor_maps, units):
    # Tendon lengths
    data.sensors.muscles.array[iteration, :, sc.muscle_tendon_unit_length] = (
        physics.data.ten_length[sensor_maps['tendonpos2data']] / units.meters
    )
    # Tendon velocities
    data.sensors.muscles.array[iteration, :, sc.muscle_tendon_unit_velocity] = (
        physics.data.ten_velocity[sensor_maps['tendonvel2data']] / units.velocity
    )
    # Tendon forces
    data.sensors.muscles.array[iteration, :, sc.muscle_tendon_unit_force] = (
        physics.data.sensordata[sensor_maps['musclefrc2data']] / units.newtons
    )
    # Remaining 7 fields via Cython
    cymusclesensors2data(
        physics=physics, iteration=iteration, data=data.sensors.muscles,
        musclesensor2data=sensor_maps['musclesensors2data'],
        meters=units.meters, velocity=units.velocity, newtons=units.newtons,
    )
```

The first three fields are read directly in Python. The remaining 7 fields (ctrl, activation, length, velocity, force, gain, user) are read by `cymusclesensors2data()` in Cython for performance.

## How to integrate: adding a new sensor type

To add support for a new MuJoCo sensor type (e.g., `accelerometer`):

1. **Add the sensor prefix** to the `sensors` list in `get_sensor_maps()`:
   ```python
   sensors = [
       ...
       'accelerometer',
   ]
   ```

2. **Add a map entry** in `get_physics2data_maps()`:
   ```python
   for identifier in ['accelerometer']:
       sensor_maps[f'{identifier}2data'] = np.array([
           sensor_maps[identifier]['indices'][
               sensor_maps[identifier]['names'].index(
                   f'{identifier}_{prefix}{link_name}'
               )
           ]
           for link_name in links_names
       ]) if all(
           f'{identifier}_{prefix}{link_name}' in sensor_maps[identifier]['names']
           for link_name in links_names
       ) else []
   ```

3. **Add a data transfer function**:
   ```python
   def physicsaccelsensors2data(physics, iteration, data, sensor_maps, units):
       if len(sensor_maps['accelerometer2data']) > 0:
           data.sensors.links.array[iteration, :, sc.link_acceleration_x:sc.link_acceleration_z+1] = (
               physics.data.sensordata[sensor_maps['accelerometer2data']] / units.acceleration
           )
   ```

4. **Add the sensor convention constants** (`sc.link_acceleration_x`, etc.) in `farms_core/sensors/sensor_convention.py`.

5. **Call the transfer function** from `physics2data()`.

## How to integrate: mapping a new MuJoCo sensor to a data array

If you have a custom MuJoCo sensor with a non-standard naming convention:

```python
# In get_physics2data_maps, after the existing maps:
custom_row = physics.named.data.sensordata.axes.row
sensor_maps['custom2data'] = np.array([
    row2index(row=custom_row, name=f'custom_{prefix}{name}')
    for name in sensor_data.links.names
    if f'custom_{prefix}{name}' in custom_row.names
])
```

## Common failure modes

### 1. Sensor name not found

If a sensor name in FARMS data doesn't match any MuJoCo sensor name, the map is set to `[]` (empty list). The corresponding data transfer is silently skipped. Symptoms: sensor arrays contain zeros for certain links/joints.

**Fix**: Check that SDF sensor names match MuJoCo sensor names. The prefix is applied to all names — if the animat is spawned with a prefix, ensure the SDF names account for it.

### 2. Quaternion convention mismatch

MuJoCo uses `[w, x, y, z]` quaternion ordering. FARMS uses `[x, y, z, w]`. The reordering `[:, [1, 2, 3, 0]]` is done in two places: at map build time (index reordering) and at data copy time (data reordering). If you add a new quaternion-related sensor, you MUST handle the reordering.

**Symptoms of incorrect quaternion handling**: Animat orientation is wrong, rotations appear inverted or scrambled.

### 3. Contact pair not found in collisions

If a contact pair from the SDF doesn't match any collision pair in the MuJoCo model, a warning is logged but the simulation continues. Contact data for that pair will be zero.

**Fix**: Ensure collision filtering in the MJCF model includes the desired body pairs.

### 4. Actuator force accumulation

`physicsactuators2data()` zeroes `joint_torque` before adding the three actuator force components. If this function is called multiple times in one iteration (e.g., during multi-substep physics), the zeroing prevents accumulation. But if you call it manually for debugging, be aware that previous torque data for that iteration is erased.

### 5. Index out of bounds

If the number of links/joints/muscles in FARMS data doesn't match the MuJoCo model, array indexing will fail with an `IndexError`. This typically happens when the SDF model is modified but the FARMS data containers are not updated.

**Fix**: Ensure `ExperimentData.from_options()` is called after any model changes to re-allocate arrays with the correct sizes.

## What NOT to assume

1. **Not all MuJoCo sensor types are mapped.** Only the types listed in the `sensors` list in `get_sensor_maps()` are recognized. Adding a new sensor type in the MJCF without updating this list means it will be silently ignored.

2. **The `actuator_moment` mapping is commented out.** Lines 346–360 contain commented-out code for `actuator_moment` mapping with a TODO comment: "actuator_moment is not a named axis anymore." This functionality is not available.

3. **Quaternion reordering is done at TWO levels.** The index map `framequat2data` is reordered at build time, and the data is reordered again at copy time (`xquat2data` data with `[:, [1, 2, 3, 0]]`). The two reorderings are independent — one affects sensor data reads, the other affects state data reads.

4. **`physicslinksvel2data()` is NOT called by `physics2data()`.** It exists as an alternative to `physicslinksvelsensors2data()`. The standard path uses sensor-based velocities, not state-based.

5. **Link CoM orientation uses the same quaternion indices as URDF orientation.** `xquat2data` is used for both `link_urdf_orientation` and `link_com_orientation`. This is because in MuJoCo, the body orientation quaternion is the same for both frames — only the position differs (xpos vs xipos).

6. **The `prefix` parameter is applied to ALL names.** When an animat is spawned as a sub-model in MuJoCo, all its body, joint, and sensor names get a prefix. The mapping functions apply this prefix when looking up names. If the prefix is wrong, ALL mappings will fail silently (empty arrays).

7. **Contact mapping is O(n²) in the number of geoms.** The `geompair2data` dict is built by iterating over all pairs of geoms. For models with many geoms, this can be slow during initialization. It's a one-time cost, but for very large models it may be noticeable.
