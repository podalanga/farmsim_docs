# Data Allocation Internals

This page documents the data allocation and management system in detail. FARMS uses a hierarchical data structure where `ExperimentData` contains multiple `AnimatData` instances, each with `SensorsData` (links, joints, contacts, xfrc, muscles, adhesions, visuals) and optionally `NetworkParameters` (for CPG-driven animats). All arrays are pre-allocated based on the simulation's `n_iterations` and `buffer_size`.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_core/model/data.py` | 140 | `AnimatData` — base animat data container |
| `farms_core/sensors/data.py` | 2106 | `SensorsData`, `LinkSensorArray`, `JointSensorArray`, etc. |
| `farms_core/experiment/data.py` | 155 | `ExperimentData` — top-level data container |
| `farms_core/simulation/data.py` | — | `SimulationData` — simulation-level data |
| `farms_core/io/hdf5.py` | 140 | HDF5 serialization (`hdf5_to_dict`, `dict_to_hdf5`) |
| `farms_amphibious/data/data.py` | 319 | `AmphibiousData`, `AmphibiousExperimentData` |
| `farms_amphibious/data/network.py` | 637 | `OscillatorNetworkState`, `NetworkParameters`, connectivity maps |

## Class hierarchy

```
ExperimentData
  ├─ times: np.ndarray [buffer_size]
  ├─ timestep: float
  ├─ simulation: SimulationData
  │    ├─ ncon: IntegerArray1D [buffer_size]
  │    ├─ niter: IntegerArray1D [buffer_size]
  │    └─ energy: DoubleArray2D [buffer_size, 6]
  └─ animats: list[AnimatData]
       └─ AnimatData (AnimatDataCy)
            ├─ sensors: SensorsData (SensorsDataCy)
            │    ├─ links: LinkSensorArray [buffer, n_links, link_fields]
            │    ├─ joints: JointSensorArray [buffer, n_joints, joint_fields]
            │    ├─ contacts: ContactsArray [buffer, n_contacts, contact_fields]
            │    ├─ xfrc: XfrcArray [buffer, n_xfrc, 6]
            │    ├─ muscles: MusclesArray [buffer, n_muscles, muscle_fields]
            │    ├─ adhesions: AdhesionsArray [buffer, n_adhesions, adhesion_fields]
            │    └─ visuals: VisualsArray [buffer, n_visuals, visual_fields]
            └─ network: NetworkLog | None  (for CPG animats)

AmphibiousData (extends AnimatData)
  ├─ state: OscillatorNetworkState [buffer, n_oscillators*3]
  ├─ network: NetworkParameters
  │    ├─ drives: DriveArray [buffer, n_drives]
  │    ├─ oscillators: Oscillators (freq, amp, phase, offset, etc.)
  │    └─ connectivity maps (osc2osc, joints2osc, contacts2osc, xfrc2osc)
  └─ joints: JointsControlArray [buffer, n_joints, 7]
```

## `AnimatData` (farms_core/model/data.py)

```python
class AnimatData(AnimatDataCy):
    def __init__(self, sensors: SensorsData, network: NetworkLog | None = None):
        super().__init__()
        self.sensors: SensorsData = sensors
        self.network: NetworkLog | None = network
```

### Factory methods

| Method | Description |
|---|---|
| `from_options(animat_options, simulation_options)` | Creates from options (sensors only, no network) |
| `from_sensors_names(buffer_size, **kwargs)` | Creates from explicit sensor name lists |
| `from_file(filename)` | Loads from HDF5 file |
| `from_dict(dictionary)` | Loads from dictionary |

### `from_options()`

```python
@classmethod
def from_options(cls, animat_options, simulation_options):
    return cls(
        sensors=SensorsData.from_options(
            animat_options=animat_options,
            simulation_options=simulation_options,
        ),
    )
```

Creates sensors data from the animat's sensor configuration and simulation's buffer size. No network data is created (that's for `AmphibiousData`).

### `from_sensors_names()`

```python
@classmethod
def from_sensors_names(cls, buffer_size, **kwargs):
    return cls(
        sensors=SensorsData.from_names(
            buffer_size=buffer_size,
            links_names=kwargs.pop('links'),
            joints_names=kwargs.pop('joints'),
            contacts_names=kwargs.pop('contacts', []),
            xfrc_names=kwargs.pop('xfrc', []),
            muscles_names=kwargs.pop('muscles', []),
            adhesions_names=kwargs.pop('adhesions', []),
            visuals_names=kwargs.pop('visuals', []),
        ),
    )
```

Used by `ExperimentTask.initialize_episode` when no pre-allocated data is available. The sensor names come from the MuJoCo model's named arrays.

### Serialization

```python
def to_dict(self, iteration=None):
    _data = {'sensors': self.sensors.to_dict(iteration)}
    if self.network is not None:
        _data['network'] = self.network.to_dict(iteration)
    return _data

def to_file(self, filename, iteration=None):
    data_dict = self.to_dict(iteration)
    dict_to_hdf5(filename=filename, data=data_dict)
```

The `iteration` parameter, if set, saves only data up to that iteration (for partial runs). If `None`, saves all data.

## `SensorsData` (farms_core/sensors/data.py)

```python
class SensorsData(SensorsDataCy):
    @classmethod
    def from_names(cls, buffer_size, links_names, joints_names, contacts_names,
                   xfrc_names, muscles_names, adhesions_names, visuals_names):
        return SensorsData(
            links=LinkSensorArray.from_names(names=links_names, buffer_size=buffer_size),
            joints=JointSensorArray.from_names(names=joints_names, buffer_size=buffer_size),
            contacts=ContactsArray.from_names(names=contacts_names, buffer_size=buffer_size),
            xfrc=XfrcArray.from_names(names=xfrc_names, buffer_size=buffer_size),
            muscles=MusclesArray.from_names(names=muscles_names, buffer_size=buffer_size),
            adhesions=AdhesionsArray.from_names(names=adhesions_names, buffer_size=buffer_size),
            visuals=VisualsArray.from_names(names=visuals_names, buffer_size=buffer_size),
        )
```

### `from_options()`

```python
@classmethod
def from_options(cls, animat_options, simulation_options):
    sensors = animat_options.control.sensors
    return cls.from_names(
        buffer_size=simulation_options.runtime.buffer_size,
        links_names=sensors.links,
        joints_names=sensors.joints,
        contacts_names=sensors.contacts,
        xfrc_names=sensors.xfrc,
        muscles_names=sensors.muscles,
        adhesions_names=sensors.adhesions,
        visuals_names=sensors.visuals,
    )
```

The sensor name lists come from `animat_options.control.sensors`, which is populated from the YAML configuration. The buffer size comes from `simulation_options.runtime.buffer_size`.

### Array shapes

Each sensor array is 3D: `[buffer_size, n_sensors, n_fields]`

| Array | Fields | Key field indices |
|---|---|---|
| `links` | position(3), orientation(4), lin_vel(3), ang_vel(3), etc. | See `sc.link_*` |
| `joints` | position, velocity, cmd_position, cmd_velocity, cmd_torque, torque_active, torque_stiffness, torque_damping, torque_friction | See `sc.joint_*` |
| `contacts` | position(3), normal(3), force(3), etc. | See `sc.contact_*` |
| `xfrc` | force(3), torque(3) | `sc.xfrc_force_x` through `sc.xfrc_torque_z` |
| `muscles` | activation, length, velocity, force, etc. | See `sc.muscle_*` |
| `adhesions` | force(3) | See `sc.adhesion_*` |
| `visuals` | color(4), emission(4) | See `sc.visual_*` |

### `from_dict()` — loading from HDF5

```python
@classmethod
def from_dict(cls, dictionary):
    return cls(
        links=LinkSensorArray.from_dict(dictionary['links']) if 'links' in dictionary
              else LinkSensorArray.from_names(names=[], buffer_size=0),
        joints=JointSensorArray.from_dict(dictionary['joints']) if 'joints' in dictionary
              else JointSensorArray.from_names(names=[], buffer_size=0),
        # ... same pattern for all sensor types
    )
```

Each sensor type is optional — if missing from the dictionary, an empty array is created. This allows loading partial data files.

## `AmphibiousData` (farms_amphibious/data/data.py)

```python
class AmphibiousData(AmphibiousDataCy, AnimatData):
    def __init__(self, state, network, joints, **kwargs):
        super().__init__(**kwargs)
        self.state = state
        self.network = network
        self.joints = joints
```

### `from_options()` — complete walkthrough

```python
@classmethod
def from_options(cls, animat_options, simulation_options):
    # 1. Sensors (same as AnimatData)
    sensors = SensorsData.from_options(animat_options, simulation_options)

    # 2. Oscillators
    oscillators = Oscillators.from_options(network=animat_options.control.network) \
        if animat_options.control.network is not None else None

    # 3. Name → index maps
    oscillators_map, joints_map, contacts_map, xfrc_map = [
        {name: i for i, name in enumerate(element.names)}
        for element in (oscillators, sensors.joints, sensors.contacts, sensors.xfrc)
    ] if animat_options.control.network is not None else (None, None, None, None)

    # 4. CPG state
    state = OscillatorNetworkState.from_initial_state(
        initial_state=animat_options.state_init(),
        n_iterations=simulation_options.runtime.n_iterations,
        n_oscillators=animat_options.control.network.n_oscillators(),
    ) if animat_options.control.network is not None else None

    # 5. Network parameters
    network = NetworkParameters(
        drives=DriveArray.from_animat_options(animat_options, n_iterations),
        oscillators=oscillators,
        osc2osc_map=OscillatorConnectivity.from_connectivity(...),
        joints2osc_map=JointsConnectivity.from_connectivity(...),
        contacts2osc_map=ContactsConnectivity.from_connectivity(...),
        xfrc2osc_map=XfrcConnectivity.from_connectivity(...),
    ) if animat_options.control.network is not None else None

    # 6. Joints control array
    joints = JointsControlArray.from_options(animat_options.control) \
        if animat_options.control.network is not None else None

    return cls(sensors=sensors, state=state, network=network, joints=joints)
```

### Name maps

The name maps convert between string names (from YAML) and integer indices (for array access):

```python
oscillators_map = {name: i for i, name in enumerate(oscillators.names)}
joints_map = {name: i for i, name in enumerate(sensors.joints.names)}
```

These are used by the connectivity classes to convert YAML connectivity entries (which use string names) into integer index pairs for the Cython ODE solver.

### `OscillatorNetworkState`

```python
class OscillatorNetworkState(OscillatorNetworkStateCy):
    @classmethod
    def from_initial_state(cls, initial_state, n_iterations, n_oscillators):
        state_size = len(initial_state)
        state_array = np.full(
            shape=[n_iterations, state_size],
            fill_value=0,
            dtype=NPDTYPE,
        )
        state_array[0, :] = initial_state
        return cls(array=state_array, n_oscillators=n_oscillators)
```

The state array has shape `[n_iterations, n_oscillators * 3]` (phases, amplitudes, offsets for each oscillator). Only the first row is initialized; the rest are zeros (filled by the ODE solver during simulation).

### `JointsControlArray`

```python
class JointsControlArray(JointsControlArrayCy):
    @classmethod
    def from_options(cls, control):
        drives = [drive.name for drive in control.network.drives]
        drive2joint_map = {
            element['joint']: [drives.index(element['drive0']), drives.index(element['drive1'])]
            for element in control.network.drive2joint
        }
        joints = [element['joint'] for element in control.network.drive2joint]
        return cls(
            array=np.array([
                [offset['gain'], offset['bias'], offset['low'], offset['high'],
                 offset['saturation_low'], offset['saturation_high'], rate]
                for offset, rate in zip(control.motors_offsets(), control.motors_offset_rates())
            ], dtype=np.double),
            drive2joint_map=IntegerArray2D(np.array([...], dtype=np.uintc)),
        )
```

Each joint has 7 parameters: `[gain, bias, low, high, saturation_low, saturation_high, offset_rate]`. The `drive2joint_map` maps each joint to its two drive indices.

## `ExperimentData` (farms_core/experiment/data.py)

```python
class ExperimentData:
    def __init__(self, times, timestep, animats, simulation=None):
        self.times = times
        self.timestep = timestep
        self.animats = animats
        self.simulation = simulation if simulation is not None else SimulationData.from_size(len(times))
```

### `from_options()`

```python
@classmethod
def from_options(cls, experiment_options, animat_class=AnimatData):
    timestep = experiment_options.simulation.physics.timestep
    n_iterations = experiment_options.simulation.runtime.n_iterations
    buffer_size = experiment_options.simulation.runtime.buffer_size
    times = np.linspace(0, n_iterations*timestep, n_iterations, dtype=float)
    animats = [
        animat_class.from_options(
            animat_options=animat_options,
            simulation_options=experiment_options.simulation,
        )
        for animat_options in experiment_options.animats
    ]
    return cls(times=times, timestep=timestep, animats=animats)
```

The `times` array is pre-computed as `linspace(0, n_iterations * timestep, n_iterations)`. Each animat gets its own data container.

### `AmphibiousExperimentData`

```python
class AmphibiousExperimentData(ExperimentData):
    @classmethod
    def from_file(cls, filename):
        data_experiment = hdf5_to_dict(filename=filename)
        for animat_data in data_experiment['animats']:
            animat_data['n_oscillators'] = len(animat_data['network']['oscillators']['names'])
        return cls.from_dict(data_experiment)
```

When loading from file, `n_oscillators` is injected into each animat's data dictionary because it's not stored directly in HDF5 — it's inferred from the oscillator names list.

## HDF5 serialization (farms_core/io/hdf5.py)

### `dict_to_hdf5(filename, data)`

```python
def dict_to_hdf5(filename, data, mode='w'):
    hfile = hdf5_open(filename, mode=mode)
    _dict_to_hdf5(hfile, data)
    hfile.close()
```

### `_dict_to_hdf5(handler, dict_data, group=None)`

Recursively writes the dictionary to HDF5:
- Nested dicts → HDF5 groups
- Lists of dicts → Groups with `FARMSLIST` prefix (e.g., `FARMSLISTlinks`)
- Scalars → Datasets
- Arrays → Compressed datasets

### `_hdf5_to_dict(handler, dict_data)`

Recursively reads HDF5 back to dictionary:
- Groups → dicts
- `FARMSLIST` groups → lists (ordered by integer keys)
- String datasets → decoded UTF-8 strings
- Numeric datasets → numpy arrays or scalars

### `hdf5_open(filename, mode, max_attempts=10, attempt_delay=0.1)`

Opens HDF5 files with retry logic to handle file locking:

```python
for attempt in range(max_attempts):
    try:
        hfile = h5py.File(name=filename, mode=mode)
        break
    except OSError:
        if attempt == max_attempts - 1:
            raise
        time.sleep(attempt_delay)
```

## `get_amphibious_data()` — factory function

```python
def get_amphibious_data(animat_options, simulation_options):
    return (
        AmphibiousKinematicsData.from_options(animat_options, simulation_options)
        if isinstance(animat_options.control, KinematicsControlOptions)
        else AmphibiousData.from_options(animat_options, simulation_options)
        if isinstance(animat_options.control, AmphibiousControlOptions)
        else AnimatData.from_options(animat_options, simulation_options)
    )
```

Selects the data class based on the control options type:
- `KinematicsControlOptions` → `AmphibiousKinematicsData` (sensors only, no CPG)
- `AmphibiousControlOptions` → `AmphibiousData` (full CPG + sensors)
- Other → `AnimatData` (sensors only)

## How to integrate: adding a new sensor type

1. **Create the sensor array class** (in `farms_core/sensors/data.py`):

```python
class MySensorArray(SensorData):
    @classmethod
    def from_names(cls, names, buffer_size):
        array = np.zeros([buffer_size, len(names), MY_SENSOR_FIELDS])
        return cls(array=array, names=names)
```

2. **Add to `SensorsData`**:

```python
class SensorsData(SensorsDataCy):
    def __init__(self, ..., my_sensors=None):
        super().__init__(...)
        self.my_sensors = my_sensors

    @classmethod
    def from_names(cls, buffer_size, ..., my_sensors_names=[]):
        return cls(..., my_sensors=MySensorArray.from_names(my_sensors_names, buffer_size))
```

3. **Add to `SensorsOptions`** in `farms_core/model/options.py`:

```python
class SensorsOptions(Options):
    def __init__(self, **kwargs):
        ...
        self.my_sensors = kwargs.pop('my_sensors', [])
```

4. **Add to YAML schema**: Users can now specify `my_sensors: [sensor1, sensor2]` in the animat's `control.sensors` section.

5. **Add to `physics2data`**: In `farms_mujoco/simulation/physics.py`, add a transfer function for the new sensor type.

## How to integrate: adding a new data field to an existing sensor

1. **Add the field index** to `farms_core/sensors/sensor_convention.py` (the `sc` object).

2. **Increase the array width**: The `from_names` method creates arrays with a fixed width. Update the width to accommodate the new field.

3. **Add a transfer function** in `physics2data` to populate the field.

4. **Update `to_dict`/`from_dict`**: The serialization is automatic (arrays are saved as-is), but you may want to add the field name to the doc.

## Common failure modes

### 1. Buffer size mismatch

When `buffer_size = 1`, data is overwritten every step. If you try to access data from a previous iteration, you'll get the current step's data. This is the most common source of subtle bugs in data-dependent control logic.

### 2. Sensor name not in map

If a sensor name in the YAML doesn't match a name in the MuJoCo model, the name map will fail. The `connections_from_connectivity` function asserts that all connection names exist in the map.

### 3. HDF5 file locking

On some systems, HDF5 files can be locked by other processes. The `hdf5_open` function retries 10 times with 0.1s delay, but if the lock persists, it raises an `OSError`.

### 4. `n_oscillators` not in HDF5

`AmphibiousExperimentData.from_file` injects `n_oscillators` into each animat's data because it's not stored as a separate HDF5 field. If you load an HDF5 file without going through this method, you'll need to inject it manually.

### 5. `FARMSLIST` prefix in HDF5

Lists of dicts are stored with a `FARMSLIST` prefix (e.g., `FARMSLISTlinks`). If you manually inspect an HDF5 file, these groups will have the prefix. The loading code strips it automatically.

## What NOT to assume

1. **`AnimatData` does NOT include network data.** Only `AmphibiousData` has `state`, `network`, and `joints`. If you're working with a non-CPG animat, these will be `None`.

2. **The `times` array uses `linspace`, not `arange`.** `np.linspace(0, n_iterations*timestep, n_iterations)` includes the endpoint. This means the last time is `(n_iterations-1)*timestep`, not `n_iterations*timestep`.

3. **All sensor arrays are 3D.** Even single-value sensors have shape `[buffer, n_sensors, n_fields]`. The third dimension is never squeezed.

4. **`from_dict` creates empty arrays for missing sensor types.** If a sensor type is not in the dictionary, `from_names(names=[], buffer_size=0)` is called, creating a `[0, 0, n_fields]` array.

5. **The `drive2joint_map` uses `IntegerArray2D`, not a regular numpy array.** This is a Cython-compatible wrapper. Don't access it as `map[i][j]` — use `map.array[i, j]`.

6. **`n_oscillators` is inferred from the oscillator names list, not stored directly.** This means the state array width (`n_oscillators * 3`) must be consistent with the number of oscillator names.
