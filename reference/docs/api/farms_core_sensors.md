# `farms_core.sensors.data`

Pre-allocated sensor data arrays for all animat sensor modalities.

## Overview

The `farms_core.sensors.data` module forms the backbone of data collection and telemetry within FARMS. It allocates static data buffers based on the expected number of simulation iterations, ensuring zero-overhead data logging that bridges Python and C structs (like MuJoCo's physics engine) without allocations during the main loop.

---

## SensorDataBase

```python
class SensorDataBase:
    def __init__(self, array=None, name=None):
        ...
```

The base class for sensory arrays, providing serialization functionality.

### Methods

| Method | Description |
|--------|-------------|
| `from_dict(cls, dictionary: dict)` | Instantiates the sensor data from a dictionary representation. |
| `to_dict(self, iteration: int | None = None) -> dict` | Returns the data as a dictionary. If `iteration` is provided, truncates the array. |

---

## SensorData

```python
class SensorData(SensorDataBase, DoubleArray3D):
    def __init__(self, array: NDARRAY_V3_D, names: list[str]):
        ...
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `array` | `NDARRAY_V3_D` | *Required* | The underlying pre-allocated numpy array. |
| `names` | `list[str]` | *Required* | Names associated with the sensors (e.g., link or joint names). |

A 3D array mapping representing `[buffer_size × n_elements × sc_size]`, where:
- `buffer_size` is the total number of iterations to log.
- `n_elements` is the number of sensors (matching `names`).
- `sc_size` is the data size per element as specified by the sensor convention (`sc`).

---

## SensorsData

```python
class SensorsData(SensorsDataCy):
    ...
```

The top-level container for all different sensor types attached to a single animat. It acts as a namespace to access the specialized arrays.

### Fields

| Name | Type | Description |
|------|------|-------------|
| `links` | `LinkSensorArray` | Kinematic data for rigid bodies. |
| `joints` | `JointSensorArray` | Kinematic and dynamic data for joints. |
| `contacts` | `ContactsArray` | Collision and contact normal/friction data. |
| `xfrc` | `XfrcArray` | External applied forces (e.g., hydrodynamic drag). |
| `muscles` | `MusclesArray` | Active muscle state and tension data. |
| `adhesions` | `AdhesionsArray` | Adhesion force data. |
| `visuals` | `VisualsArray` | Visual telemetry. |

---

## Cython Array Classes

These classes inherit from `SensorData` and wrap highly optimized Cython routines for rapid telemetry ingestion.

### LinkSensorArray

Logs data related to the rigid bodies (links) of the robot.

| Method | Description |
|--------|-------------|
| `com_position(iter, link_i)` | Returns `[x,y,z]` Center of Mass position for `link_i` at `iter`. |
| `urdf_position(iter, link_i)`| Returns `[x,y,z]` URDF origin position. |
| `com_lin_velocity(iter, link_i)` | Returns `[vx, vy, vz]` linear velocity. |
| `heading(iter, indices)` | Computes the yaw heading angle based on an array of link indices. |
| `global_com_position(iter)` | Computes total system COM weighted by link `masses`. |

### JointSensorArray

Tracks the internal state and dynamics of the robot's joints. 

| Method | Maps to `sc` index | Description |
|--------|--------------------|-------------|
| `position(iter, joint_i)` | `joint_position` | Current angle (rad) or extension (m). |
| `velocity(iter, joint_i)` | `joint_velocity` | Current joint velocity. |
| `motor_torque(iter, joint_i)` | `joint_torque` | Total applied torque. |
| `force(iter, joint_i)` | `joint_force_x/y/z` | 3D reaction forces at the joint hinge. |
| `cmd_position(iter, joint_i)` | `joint_cmd_position` | Last positional setpoint. |
| `cmd_torque(iter, joint_i)` | `joint_cmd_torque` | Last torque command. |
| `active(iter, joint_i)` | `joint_torque_active` | Torque generated purely by motors/muscles. |
| `spring(iter, joint_i)` | `joint_torque_stiffness` | Torque resulting from joint stiffness compliance. |

### ContactsArray

Logs collision events and reaction forces. It tracks normal forces, friction forces, and contact positions for geometry explicitly tracked in the animat options.

## See Also

- [Controller Base Classes](farms_core_control.md) — How sensor data is consumed
- [Amphibious Data](farms_amphibious_data.md) — Extended sensor data for amphibious animats
