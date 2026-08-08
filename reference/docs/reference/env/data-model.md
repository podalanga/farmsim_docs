# Data Model Reference

Reference for FARMS data classes — the in-memory and on-disk representations
of simulation state.

## Class hierarchy

```
ExperimentData                    farms_core.experiment.data
├── times: np.ndarray             # (n_iterations,)
├── timestep: float
├── simulation: SimulationData
│   └── units: SimulationUnitScaling
└── animats: list[AnimatData]
    └── AnimatData                farms_core.model.data
        ├── sensors: SensorsData
        │   ├── links:    LinkSensorData     (n_iters, n_links, 19)
        │   ├── joints:   JointSensorData    (n_iters, n_joints, 3)
        │   ├── contacts: ContactSensorData  (n_iters, n_contacts, 7)
        │   ├── xfrc:     XfrcSensorData     (n_iters, n_links, 6)
        │   ├── muscles:  MuscleSensorData   (n_iters, n_muscles, ...)
        │   ├── adhesions: AdhesionSensorData (n_iters, n_adhesions, 3)
        │   └── visuals:  VisualSensorData   (n_iters, n_visuals, ...)
        ├── state: StateData (optional)
        └── network: NetworkLog (optional)
```

## ExperimentData

**Source:** `farms_core/experiment/data.py`

```python
class ExperimentData:
    def __init__(self, times, timestep, simulation, animats):
        ...

    @classmethod
    def from_options(cls, options: ExperimentOptions) -> 'ExperimentData':
        """Pre-allocate all arrays from experiment options."""

    def to_file(self, filename: str):
        """Serialize to HDF5."""

    @classmethod
    def from_file(cls, filename: str) -> 'ExperimentData':
        """Deserialize from HDF5."""
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `times` | np.ndarray | `(n_iterations,)` — time at each step [s] |
| `timestep` | float | Physics timestep [s] |
| `simulation` | SimulationData | Simulation-level data |
| `animats` | list[AnimatData] | One per animat |

## AnimatData

**Source:** `farms_core/model/data.py`

```python
class AnimatData:
    def __init__(self, sensors, state=None, network=None):
        ...
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `sensors` | SensorsData | All sensor arrays |
| `state` | StateData | CPG state (phases, amplitudes) — optional |
| `network` | NetworkLog | Network drive/connectivity log — optional |

## SensorsData

**Source:** `farms_core/model/data.py`

```python
class SensorsData:
    def __init__(self, links, joints, contacts, xfrc, muscles,
                 adhesions, visuals):
        ...
```

Each attribute is a sensor data object with:

- `.array` — NumPy ndarray of shape `(n_iterations, n_sensors, data_dim)`
- `.names` — list[str] of sensor names (for index lookup)

### Link sensor data

Array shape: `(n_iterations, n_links, 19)`

| Columns | Data |
|---------|------|
| 0:3 | Position (x, y, z) |
| 3:7 | Orientation quaternion (w, x, y, z) |
| 7:10 | Linear velocity |
| 10:13 | Angular velocity |
| 13:16 | Linear acceleration |
| 16:19 | Angular acceleration |

### Joint sensor data

Array shape: `(n_iterations, n_joints, 3)`

| Columns | Data |
|---------|------|
| 0 | Position [rad] |
| 1 | Velocity [rad/s] |
| 2 | Torque [N·m] |

### Contact sensor data

Array shape: `(n_iterations, n_contacts, 7)`

| Columns | Data |
|---------|------|
| 0:3 | Force (x, y, z) [N] |
| 3:6 | Torque (x, y, z) [N·m] |
| 6 | Normal force magnitude [N] |

### XFRC sensor data

Array shape: `(n_iterations, n_links, 6)`

| Columns | Data |
|---------|------|
| 0:3 | Force (x, y, z) [N] |
| 3:6 | Torque (x, y, z) [N·m] |

## SimulationData

Contains simulation-level metadata, primarily unit scaling.

### SimulationUnitScaling

```python
class SimulationUnitScaling:
    """Unit conversion factors between simulation and SI units."""
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `length` | float | Meters per simulation unit |
| `angle` | float | Radians per simulation unit |
| `seconds` | float | Seconds per simulation unit |
| `kilograms` | float | Kilograms per simulation unit |
| `newton` | float | Newtons per simulation unit |
| `torque` | float | N·m per simulation unit |

Access from extensions via `task.units`:

```python
sim_time = physics.time() / task.units.seconds
dt = physics.timestep() / task.units.seconds
```

## HDF5 serialization

### Writing

```python
experiment_data.to_file('simulation.hdf5')
```

The HDF5 file preserves the full class hierarchy:

- Datasets for each sensor array
- Attributes for metadata (names, timestep, etc.)
- Groups for hierarchical structure (animat_0, animat_1, etc.)

### Reading

```python
data = ExperimentData.from_file('simulation.hdf5')
```

Reconstructs the full `ExperimentData` object with all arrays and metadata.

## State data (CPG networks)

When using the amphibious CPG network, `AnimatData.state` contains:

| Attribute | Type | Description |
|-----------|------|-------------|
| `array` | np.ndarray | `(n_iterations, n_states)` — interleaved phases and amplitudes |

State layout: `[phase_0, phase_1, ..., phase_N, amp_0, amp_1, ..., amp_N, joint_0, ...]`

## Network log

When using the CPG network, `AnimatData.network` contains:

| Attribute | Type | Description |
|-----------|------|-------------|
| `drives.array` | np.ndarray | `(n_iterations, n_drives)` — drive values over time |
| `state.array` | np.ndarray | `(n_iterations, n_states)` — network state |
