# farms_amphibious.control.drive

Descending drive system — goal-directed modulation of CPG amplitude and frequency.

## Overview

The `farms_amphibious.control.drive` module implements the descending command signals that govern locomotion. By modulating the amplitude and frequency parameters of the underlying CPG network, the descending drive acts as the steering and gait-selection mechanism, transitioning the animat between behaviors like walking and swimming based on high-level goals and contact feedback.

---

## PotentialMap

Abstract base class defining a heading strategy for navigation.

### `heading`

```python
@abstractmethod
def heading(self, pos)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `pos` | `NDARRAY_V1` | _Required_ | Current 2D or 3D cartesian position in meters. |

### `heading_cartesian`

```python
def heading_cartesian(self, pos, radius=1)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `pos` | `NDARRAY_V1` | _Required_ | Current cartesian position in meters. |
| `radius` | `float` | `1` | Radius of the output vector. |

### `limit_cycle`

```python
@staticmethod
def limit_cycle()
```

Returns the target limit cycle trajectory array, or `None`.

### `mesh`

```python
def mesh(self, lin_x, lin_y, radius=1)
```

Generates a vector field mesh for visualization.

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `lin_x` | `NDARRAY_V1` | _Required_ | X-axis evaluation points. |
| `lin_y` | `NDARRAY_V1` | _Required_ | Y-axis evaluation points. |
| `radius` | `float` | `1` | Vector radius normalization. |

---

## StraightLinePotentialMap

Inherits from `PotentialMap`. Navigates the animat along a straight line trajectory.

```python
def __init__(self, **kwargs)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `gain` | `float` | `1` | Lateral correction gain. |
| `origin` | `NDARRAY_V1` | `[0, 0]` | Origin point of the line in meters. |
| `theta` | `float` | `0` | Angle of the line in radians. |

---

## CirclePotentialMap

Inherits from `PotentialMap`. Navigates the animat in a circular orbit.

```python
def __init__(self, **kwargs)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `gain` | `float` | `1` | Radial correction gain. |
| `origin` | `NDARRAY_V1` | `[0, 0]` | Center point of the circle in meters. |
| `radius` | `float` | `4` | Radius of the orbit in meters. |
| `direction` | `int` | `-1` | Orbit direction (`1` for CCW, `-1` for CW). |

---

## DescendingDrive

Abstract base class for all drive modulation strategies.

```python
def __init__(self, drives: DriveArray)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `drives` | `DriveArray` | _Required_ | Pre-allocated time-series array of drive channels. |

### `step`

```python
@abstractmethod
def step(self, iteration: int, time: float, timestep: float)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Current simulation iteration. |
| `time` | `float` | _Required_ | Current simulation time in seconds. |
| `timestep` | `float` | _Required_ | Simulation timestep in seconds. |

### `get_left_drives`

```python
def get_left_drives(self, iteration: int)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Simulation iteration index. |

### `get_right_drives`

```python
def get_right_drives(self, iteration: int)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Simulation iteration index. |

### `set_left_drives`

```python
def set_left_drives(self, iteration: int, values, brain: bool = True)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Simulation iteration index. |
| `values` | `NDARRAY_V1` | _Required_ | Array of drive values to assign. |
| `brain` | `bool` | `True` | Whether to also write to the brain left indices. |

### `set_right_drives`

```python
def set_right_drives(self, iteration: int, values, brain: bool = True)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Simulation iteration index. |
| `values` | `NDARRAY_V1` | _Required_ | Array of drive values to assign. |
| `brain` | `bool` | `True` | Whether to also write to the brain right indices. |

### `set_left_drive`

```python
def set_left_drive(self, iteration: int, value: float, brain: bool = True)
```

Sets all left drive channels (spine and optionally brain) to a single scalar value.

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Simulation iteration index. |
| `value` | `float` | _Required_ | Scalar drive value. |
| `brain` | `bool` | `True` | Whether to also write to the brain left indices. |

### `set_right_drive`

```python
def set_right_drive(self, iteration: int, value: float, brain: bool = True)
```

Sets all right drive channels (spine and optionally brain) to a single scalar value.

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Simulation iteration index. |
| `value` | `float` | _Required_ | Scalar drive value. |
| `brain` | `bool` | `True` | Whether to also write to the brain right indices. |

---

## OrientationFollower

Inherits from `DescendingDrive`. Implements PID-based orientation tracking using a `PotentialMap`. Drives turn commands via left/right asymmetry and switches gait (walking vs swimming) based on a global contact threshold.

```python
def __init__(self, strategy: PotentialMap, animat_data: AmphibiousData, timestep: float, **kwargs)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `strategy` | `PotentialMap` | _Required_ | The heading strategy to follow. |
| `animat_data` | `AmphibiousData` | _Required_ | Simulation animat data. |
| `timestep` | `float` | _Required_ | Controller timestep in seconds. |
| `pid_p` | `float` | `0.2` | Proportional gain for steering. |
| `pid_i` | `float` | `0.0` | Integral gain for steering. |
| `pid_d` | `float` | `0.0` | Derivative gain for steering. |
| `output_limits` | `tuple` | `(-0.7, 0.7)` | PID output clamping bounds. |
| `links_indices` | `list` | `[0]` | Indices of links to use for position tracking. |
| `heading_offset` | `float` | `0` | Constant offset added to the observed heading in radians. |
| `contact_threshold` | `float` | `0` | Contact force threshold for global gait switching. |

---

## DistributedOrientationFollower

Inherits from `OrientationFollower`. Extends the global gait switching logic to a distributed model, allowing per-limb gait selection based on localized contact forces.

```python
def __init__(self, *args, **kwargs)
```

Parameters map identically to `OrientationFollower`, but the controller manages a distinct `fwds_raw` decision per contact group rather than a single global state.

---

## drive_from_config

```python
def drive_from_config(filename, animat_data, simulation_options)
```

Factory function that instantiates the appropriate `DescendingDrive` subclass based on a YAML configuration file.

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `filename` | `str` | _Required_ | Path to the YAML configuration file. |
| `animat_data` | `AmphibiousData` | _Required_ | Simulation animat data. |
| `simulation_options` | `SimulationOptions` | _Required_ | Global simulation options object. |

---

## Usage Example

Subclassing `DescendingDrive` to implement a simple speed-control policy that forces a walk gait and gradually increases speed:

```python
class SpeedController(DescendingDrive):
    def __init__(self, animat_data):
        super().__init__(drives=animat_data.network.drives)
        self.base_drive = 2.0  # Walk regime

    def step(self, iteration: int, time: float, timestep: float):
        # Gradually increase drive speed up to 3.0
        current_drive = min(self.base_drive + 0.1 * time, 3.0)
        
        self.set_left_drive(iteration, current_drive)
        self.set_right_drive(iteration, current_drive)
```

## See Also

- [farms_amphibious_controller.md](farms_amphibious_controller.md)
- [farms_amphibious_data.md](farms_amphibious_data.md)
- Source: `farms_amphibious/control/drive.py`
