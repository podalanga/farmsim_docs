# Data Flow and Persistence

This document traces how data flows through a FARMS simulation — from YAML
configuration through pre-allocated arrays to HDF5 output.

## Configuration → Options → Data

```
YAML files
    ↓  (yaml2pyobject with loader dotted paths)
ExperimentOptions
    ├── simulation: SimulationOptions
    ├── animats: list[AnimatOptions]
    └── arenas: list[ArenaOptions]
    ↓  (ExperimentData.from_options)
ExperimentData
    ├── times: ndarray(n_iterations)
    ├── timestep: float
    ├── simulation: SimulationData
    │   └── units: SimulationUnitScaling
    └── animats: list[AnimatData]
        └── sensors: SensorsData
            ├── links:    ndarray(n_iterations, n_links, 19)
            ├── joints:   ndarray(n_iterations, n_joints, 3)
            ├── contacts:  ndarray(n_iterations, n_contacts, 7)
            └── xfrc:     ndarray(n_iterations, n_links, 6)
```

## Pre-allocation strategy

`ExperimentData.from_options()` pre-allocates all arrays before the simulation
starts. Array dimensions are computed from:

- `n_iterations = int(duration / timestep)` — from `RunOptions`
- Sensor counts — from `SensorsOptions` name lists
- Data dimensions — fixed per sensor type (19 for links, 3 for joints, etc.)

This approach has several benefits:

1. **No allocation during simulation** — the hot loop only writes to
   pre-existing arrays
2. **Memory predictability** — total memory is known at setup time
3. **Efficient HDF5 writing** — arrays can be written in a single operation
4. **Index-based access** — extensions write to `array[iteration, ...]`,
   avoiding dynamic data structures

## Per-step data flow

During each simulation step, data flows through the system in this order:

```
1. MuJoCo physics state (qpos, qvel, xpos, xquat, etc.)
       ↓  (ExperimentTask.update_sensors)
2. AnimatData.sensors arrays (pre-allocated)
       ↓  (extensions read sensor data)
3. Extension computations (forces, control outputs)
       ↓  (controllers write joint targets)
4. physics.data.ctrl (MuJoCo control inputs)
       ↓  (MuJoCo physics.step())
5. New MuJoCo physics state
       ↓  (ExperimentTask.after_step)
6. Extensions log to AnimatData (e.g., ExperimentLogger)
```

## Sensor update

`ExperimentTask.update_sensors(iteration, physics)` reads from MuJoCo's physics
state and writes into the pre-allocated arrays:

| Sensor array | MuJoCo source | Columns |
|--------------|---------------|---------|
| `links.array[i, :, 0:3]` | `physics.data.xpos` | Position |
| `links.array[i, :, 3:7]` | `physics.data.xquat` | Orientation |
| `links.array[i, :, 7:10]` | `physics.data.cvel` | Linear velocity |
| `links.array[i, :, 10:13]` | `physics.data.cvel` | Angular velocity |
| `joints.array[i, :, 0]` | `physics.data.qpos` | Position |
| `joints.array[i, :, 1]` | `physics.data.qvel` | Velocity |
| `joints.array[i, :, 2]` | `physics.data.actuator_force` | Torque |
| `xfrc.array[i, :, :3]` | `physics.data.xfrc_applied` | Force |
| `contacts.array[i, :, :3]` | `physics.data.collision` | Contact force |

## Force application

Extensions that apply forces (e.g., `SwimmingExtension`) write to
`physics.data.xfrc_applied`:

```python
# In SwimmingExtension.before_step():
physics.data.xfrc_applied[body_id, :3] = drag_force + buoyancy_force
physics.data.xfrc_applied[body_id, 3:] = torque
```

MuJoCo reads `xfrc_applied` during `physics.step()` and incorporates these
external forces into the dynamics computation.

## Controller output mapping

Controller outputs are dicts mapping joint names to float values:

```python
# In ExperimentTask.before_step():
positions = controller.positions(iteration, time, timestep)
# positions = {'joint_0': 0.5, 'joint_1': -0.3, ...}
```

The task maps these to MuJoCo's `ctrl` array using the joint name → actuator
index map built during `initialize_episode()`.

## HDF5 persistence

At episode end (or simulation completion), `ExperimentLogger` calls
`ExperimentData.to_file()`:

```python
def to_file(self, filename):
    """Write all data to HDF5."""
```

The HDF5 file structure mirrors the Python object hierarchy:

- Top-level datasets: `times`, `timestep`
- Groups for each animat: `animat_0/`, `animat_1/`, ...
- Sub-groups for sensors: `animat_0/sensors/links/`, etc.
- Array datasets within each sensor group
- Attributes for metadata (names, units, etc.)

### Round-trip fidelity

`ExperimentData.from_file()` reconstructs the full object tree from HDF5.
The round-trip is lossless — all array data and metadata are preserved.

## Options persistence

`ExperimentOptionsLogger` saves YAML copies of all configuration files:

```
options/
├── experiment_config.yaml
├── simulation_config.yaml
├── animat_config.yaml
└── arena_config.yaml
```

These are the original YAML files as loaded (after any convention-based
defaults have been expanded). They can be used to reproduce the simulation.

## See also

- [Save, Load, and Inspect Data](../how-to/save-load-data.md) — practical usage
- [Data Model](../reference/env/data-model.md) — class reference
- [Simulation Lifecycle](simulation-lifecycle.md) — when data is written
