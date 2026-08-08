# Descending Drive and Sensory Feedback Internals

This page documents the `DescendingDrive` system (`farms_amphibious/control/drive.py`, 582 lines) that provides high-level locomotion commands to the CPG network. The descending drive is the bridge between high-level navigation goals (follow a path, turn left/right) and the low-level CPG oscillator dynamics.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_amphibious/control/drive.py` | 582 | `DescendingDrive` ABC, `OrientationFollower`, `DistributedOrientationFollower`, `PotentialMap` classes |
| `farms_amphibious/data/network.py` | — | `DriveArray` with `spine_left_indices`, `brain_left_indices`, etc. |
| `farms_amphibious/model/options.py` | — | `DriveKind` enum |

## Call graph / entry points

```
ExperimentTask.before_step()
  └─ AmphibiousController.before_step()
       ├─ drive.step(iteration, time, timestep)    [DescendingDrive]
       │    └─ update_intention()
       │         ├─ update_turn_command(pos)       [PotentialMap.heading]
       │         ├─ get_turn_control()              [PID controller]
       │         ├─ get_foward_control()            [contact-based speed]
       │         ├─ compute_intention()             [L/R drive calculation]
       │         └─ write_to_brain()                [write to brain drive indices]
       └─ network.step(iteration, time, timestep)   [NetworkODE integration]
```

## PotentialMap (abstract base class)

```python
class PotentialMap(ABC):
    @abstractmethod
    def heading(self, pos):
        """Heading"""
        raise NotImplementedError

    @staticmethod
    def limit_cycle():
        """Limit cycle"""
        return None

    def heading_cartesian(self, pos, radius=1):
        """Heading cartesian"""
        heading_complex = radius * np.exp(1j * self.heading(pos))
        return heading_complex.real, heading_complex.imag

    def mesh(self, lin_x, lin_y, radius=1):
        """Mesh"""
        dimensions = (len(lin_x), len(lin_y))
        vec_x, vec_y = np.meshgrid(lin_x, lin_y, indexing='ij')
        vec_u, vec_v = np.zeros(dimensions), np.zeros(dimensions)
        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                vec_u[i, j], vec_v[i, j] = self.heading_cartesian(
                    pos=np.array([vec_x[i, j], vec_y[i, j]]),
                    radius=radius,
                )
        return vec_x, vec_y, vec_u, vec_v
```

### Methods

| Method | Description |
|---|---|
| `heading(pos)` | Abstract: returns desired heading angle [rad] for a given 2D position |
| `limit_cycle()` | Returns points defining the limit trajectory (for plotting). Default: `None`. |
| `heading_cartesian(pos, radius=1)` | Converts heading angle to (x, y) unit vector scaled by `radius` |
| `mesh(lin_x, lin_y, radius=1)` | Generates a vector field mesh for visualization. Returns `(vec_x, vec_y, vec_u, vec_v)`. |

## StraightLinePotentialMap

```python
class StraightLinePotentialMap(PotentialMap):
    def __init__(self, **kwargs):
        self.gain = kwargs.pop('gain', 1)
        self.origin = kwargs.pop('origin', np.zeros(2))
        self.theta = kwargs.pop('theta', 0)
        assert not kwargs, kwargs
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `gain` | float | 1 | Attraction gain toward the line |
| `origin` | np.ndarray | `[0, 0]` | A point on the desired line |
| `theta` | float | 0 | Desired heading angle [rad] |

### `heading(pos)` walkthrough

```python
def heading(self, pos):
    pos_complex = complex(*(pos[:2] - self.origin))
    theta = np.angle(pos_complex)                              # Current bearing from origin
    r_dot = self.gain * np.linalg.norm(pos_complex) * np.sin(self.theta - theta)
    return np.angle(
        r_dot * np.exp(1j * (self.theta + 0.5 * np.pi))      # Lateral correction
        + np.exp(1j * self.theta)                              # Forward direction
    )
```

1. Compute position relative to origin as a complex number.
2. `theta = angle(pos_complex)`: Current bearing from origin to animat.
3. `r_dot`: Proportional to distance from the line (cross-track error) times `sin(desired_theta - current_theta)`.
4. The heading is the angle of a vector sum: a lateral correction term (perpendicular to desired direction, scaled by cross-track error) plus the forward direction.

### `limit_cycle()`

Returns two points defining the desired straight-line trajectory:

```python
def limit_cycle(self):
    vector_complex = np.exp(1j * self.theta)
    vector = np.array([vector_complex.real, vector_complex.imag])
    return np.array([self.origin + 1e3*vector, self.origin - 1e3*vector])
```

Two points: origin ± 1000 units along the desired direction. This creates a very long line segment for plotting.

## CirclePotentialMap

```python
class CirclePotentialMap(PotentialMap):
    def __init__(self, **kwargs):
        self.gain = kwargs.pop('gain', 1)
        self.origin = kwargs.pop('origin', np.zeros(2))
        self.radius = kwargs.pop('radius', 4)
        self.direction = kwargs.pop('direction', -1)
        assert not kwargs, kwargs
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `gain` | float | 1 | Attraction gain toward the circle |
| `origin` | np.ndarray | `[0, 0]` | Center of the circle |
| `radius` | float | 4 | Desired circle radius |
| `direction` | float | -1 | Rotation direction (-1 = clockwise, 1 = counterclockwise) |

### `heading(pos)` walkthrough

```python
def heading(self, pos):
    pos_complex = complex(*(pos[:2] - self.origin))
    r_dot = self.gain * (self.radius - np.abs(pos_complex))   # Radial error
    theta = np.angle(pos_complex)                              # Current bearing
    return np.angle(
        r_dot * np.exp(1j * theta)                             # Radial correction
        + np.exp(1j * (theta + 0.5 * np.sign(self.direction) * np.pi))  # Tangential
    )
```

1. `r_dot = gain * (radius - |pos|)`: Positive when inside the circle (push outward), negative when outside (pull inward).
2. The heading combines a radial correction term (toward/away from center) and a tangential term (perpendicular to radius, direction-dependent).

## EllipsoidPotentialMap

```python
class EllipsoidPotentialMap(PotentialMap):
    def __init__(self, **kwargs):
        self.gain = kwargs.pop('gain', 1)
        self.origin = np.array(kwargs.pop('origin', np.zeros(2)))
        self.radius1 = kwargs.pop('radius1', 4)
        self.radius2 = kwargs.pop('radius2', 3)
        self.theta = kwargs.pop('theta', 0)
        self.direction = kwargs.pop('direction', -1)
        rotation = Rotation.from_euler('z', self.theta, degrees=False)
        self.rotation = rotation.as_matrix()[:2, :2]
        assert not kwargs, kwargs
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `gain` | float | 1 | Attraction gain |
| `origin` | np.ndarray | `[0, 0]` | Center of ellipse |
| `radius1` | float | 4 | Semi-axis along x (after rotation) |
| `radius2` | float | 3 | Semi-axis along y (after rotation) |
| `theta` | float | 0 | Rotation angle [rad] |
| `direction` | float | -1 | Rotation direction (-1 = clockwise) |

### `heading(pos)` walkthrough

```python
def heading(self, pos):
    new_pos = np.dot(self.rotation.T, pos[:2] - self.origin)  # Transform to ellipse frame
    pos_complex = complex(new_pos[0] / self.radius1, new_pos[1] / self.radius2)  # Normalize to unit circle
    r_dot = self.gain * (1 - np.abs(pos_complex))              # Error from unit circle
    theta = np.angle(pos_complex)
    angle = np.angle(
        r_dot * np.exp(1j * theta)
        + np.exp(1j * (theta + 0.5 * np.sign(self.direction) * np.pi))
    )
    vector = np.array([self.radius1 * np.cos(angle), self.radius2 * np.sin(angle)])
    return np.arctan2(vector[1], vector[0]) + self.theta
```

1. Transform position into the ellipse's local frame using the rotation matrix.
2. Normalize by dividing by semi-axes, mapping the ellipse to a unit circle.
3. Apply the same circular potential field logic as `CirclePotentialMap`.
4. Transform the resulting angle back to the global frame.

## DescendingDrive (abstract base class)

```python
class DescendingDrive(ABC):
    def __init__(self, drives: DriveArray):
        super().__init__()
        self.drives: DriveArray = drives
        self.n_drives: int = np.shape(drives.array)[1]
        self.n_iterations: int = np.shape(drives.array)[0]
        self._drives_vector = np.ones(self.n_drives)
```

### Attributes

| Attribute | Type | Description |
|---|---|---|
| `drives` | `DriveArray` | The drive array (shape `[n_iterations, n_drives]`) |
| `n_drives` | int | Total number of drive entries (brain + body + leg) |
| `n_iterations` | int | Total number of simulation iterations |
| `_drives_vector` | np.ndarray | All-ones vector of length `n_drives`, used for broadcasting |

### Abstract method

```python
@abstractmethod
def step(self, iteration: int, time: float, timestep: float):
    """Step"""
    raise NotImplementedError
```

Called every simulation step before `NetworkODE.step()`. Must write drive values to `self.drives.array[iteration, :]`.

### Drive access methods

```python
def get_left_drives(self, iteration: int):
    return self.drives.array[iteration, self.drives.spine_left_indices[0]]

def get_right_drives(self, iteration: int):
    return self.drives.array[iteration, self.drives.spine_right_indices[0]]
```

`get_left_drives` / `get_right_drives` return the drive value at the first spine left/right index. These are convenience methods for reading the current drive state.

```python
def set_left_drives(self, iteration: int, values, brain: bool = True):
    for index in self.drives.spine_left_indices:
        self.drives.array[iteration, index] = values[index]
    if brain:
        for index in self.drives.brain_left_indices:
            self.drives.array[iteration, index] = values[index]

def set_right_drives(self, iteration: int, values, brain: bool = True):
    for index in self.drives.spine_right_indices:
        self.drives.array[iteration, index] = values[index]
    if brain:
        for index in self.drives.brain_right_indices:
            self.drives.array[iteration, index] = values[index]
```

`set_left_drives` / `set_right_drives` write `values[index]` to ALL left/right spine drives. If `brain=True` (default), brain drives are also written. The `values` array must be indexed by the same drive indices.

```python
def set_left_drive(self, iteration: int, value: float, brain: bool = True):
    self.set_left_drives(iteration, value * self._drives_vector, brain)

def set_right_drive(self, iteration: int, value: float, brain: bool = True):
    self.set_right_drives(iteration, value * self._drives_vector, brain)
```

`set_left_drive` / `set_right_drive` set ALL left/right drives to a single scalar value by multiplying with `_drives_vector` (all ones).

### The `brain` parameter

When `brain=True` (default), drive values are written to BOTH brain and spine indices. When `brain=False`, only spine drives are updated, leaving brain drives at their previous value. This is useful for distributed control where brain and spine drives may be set independently.

## OrientationFollower

```python
class OrientationFollower(DescendingDrive):
    def __init__(
        self,
        strategy: PotentialMap,
        animat_data: AmphibiousData,
        timestep: float,
        **kwargs,
    ):
```

### Constructor parameters

| Parameter | Type | Required | Default | Description |
|---|---|---|---|---|
| `strategy` | `PotentialMap` | Yes | — | Navigation strategy (line, circle, ellipsoid) |
| `animat_data` | `AmphibiousData` | Yes | — | Animat data container (for reading sensors) |
| `timestep` | float | Yes | — | Simulation timestep [s] |
| `links_indices` | np.ndarray | No | `[0]` | Link indices used for heading computation |
| `heading_offset` | float | No | 0 | Heading offset [rad] |
| `contact_threshold` | float | No | 0 | Contact force threshold for gait switching |
| `pid_p` | float | No | 0.2 | PID proportional gain |
| `pid_i` | float | No | 0.0 | PID integral gain |
| `pid_d` | float | No | 0.0 | PID derivative gain |
| `output_limits` | tuple | No | `(-0.9, 0.9)` | PID output limits |
| `contact_threshold_dis` | Any | No | None | Popped and discarded (legacy parameter) |

**Strict kwargs**: `assert not kwargs, kwargs` at the end rejects any unknown parameters.

### Internal state

| Attribute | Type | Description |
|---|---|---|
| `setpoints` | np.ndarray | Desired heading at each iteration `[n_iterations]` |
| `control` | np.ndarray | PID control output at each iteration `[n_iterations]` |
| `contact_value` | float | Low-pass filtered contact force magnitude |
| `fwds` | np.ndarray | Low-pass filtered forward drive values `[n_drives]` |
| `fwds_raw` | np.ndarray | Raw forward drive values (before filtering) `[n_drives]` |
| `turn` | float | Low-pass filtered turn command |
| `drive_types` | list | Per-drive-index classification (BRAIN_LEFT, BRAIN_RIGHT, SPINE_LEFT, SPINE_RIGHT, or None) |
| `pid` | `simple_pid.PID` | PID controller for heading |

### `drive_types` computation (lines 279–290)

```python
self.drive_types = [
    DriveKind.BRAIN_LEFT if index in self.drives.brain_left_indices
    else DriveKind.BRAIN_RIGHT if index in self.drives.brain_right_indices
    else DriveKind.SPINE_LEFT if index in self.drives.spine_left_indices
    else DriveKind.SPINE_RIGHT if index in self.drives.spine_right_indices
    else None
    for index in range(self.n_drives)
]
```

This classifies every drive index as brain-left, brain-right, spine-left, spine-right, or None. Drives classified as None (e.g., leg drives) are not affected by the orientation follower.

### `step()` — complete walkthrough

```python
def step(self, iteration: int, time: float, timestep: float):
    intention = self.update_intention(
        iteration=iteration,
        time=time,
        timestep=timestep,
        pos=np.array(self.animat_data.sensors.links.urdf_position(
            iteration=iteration,
            link_i=self.links_indices[0],
        )),
        heading=self.animat_data.sensors.links.heading(
            iteration=iteration,
            indices=self.links_indices,
        ) + self.heading_offset,
    )
    self.set_left_drives(iteration=iteration, values=intention)
    self.set_right_drives(iteration=iteration, values=intention)
```

1. Read the animat's current 2D position from `links.urdf_position(iteration, link_i)`.
2. Read the animat's current heading from `links.heading(iteration, indices)` and add `heading_offset`.
3. Call `update_intention()` which computes the drive values.
4. Write left and right drive values to the drive array.

### `update_intention()` walkthrough

```python
def update_intention(self, iteration, time, timestep, pos, heading):
    # 1. Compute desired heading from potential map
    self.setpoints[iteration] = self.update_turn_command(pos=pos)

    # 2. PID control on heading error
    drive_turn = self.get_turn_control(
        iteration=iteration, time=time, timestep=timestep,
        command=self.setpoints[iteration], heading=heading,
    )

    # 3. Low-pass filter the turn command
    self.turn += min(100*timestep, 1) * (drive_turn - self.turn)

    # 4. Compute forward drive from contacts
    drive_fwds = self.get_foward_control(iteration=iteration, timestep=timestep)

    # 5. Low-pass filter the forward drive
    self.fwds += min(100*timestep, 1) * (drive_fwds - self.fwds)

    # 6. Record control output
    self.control[iteration] = self.turn

    # 7. Compute intention (L/R drive values)
    intention = self.compute_intention()

    # 8. Write to brain drives
    self.write_to_brain(iteration)

    return intention
```

**Low-pass filter**: `min(100*timestep, 1)` is the filter coefficient. At 1ms timestep, this is 0.1 (slow filter). At 10ms timestep, it's 1.0 (no filtering). The filter smooths out sudden changes in drive commands.

### `get_turn_control()` walkthrough

```python
def get_turn_control(self, iteration, time, timestep, command, heading):
    command, heading = float(command), float(heading)
    error: float = ((command - heading + np.pi) % (2*np.pi)) - np.pi
    self.current_time = time
    self.pid.setpoint = 0
    val: float = self.pid(error)
    return val
```

1. Compute heading error wrapped to \([-\pi, \pi]\): `((command - heading + π) % 2π) - π`.
2. Set PID setpoint to 0 (we want `error = 0`, i.e., heading matches command).
3. Call `self.pid(error)` which computes the PID output.
4. The PID output is clamped to `output_limits` (default: ±0.9).

### `get_foward_control()` walkthrough

```python
def get_foward_control(self, iteration, timestep):
    threshold = self.contact_threshold
    contacts = np.nan_to_num(
        self.animat_data.sensors.contacts.reactions()[iteration],
        copy=True, nan=0.0,
    )
    contacts_sum = np.sum(np.linalg.norm(contacts, axis=1))
    self.contact_value += min(100*timestep, 1) * (contacts_sum - self.contact_value)
    self.fwds_raw[:] = 2 if self.contact_value > threshold else 4
    return self.fwds_raw
```

1. Read contact force reactions for this iteration.
2. Replace NaN values with 0 (contacts may have NaN if not yet computed).
3. Sum the magnitudes of all contact force vectors.
4. Low-pass filter the contact sum into `self.contact_value`.
5. If contact exceeds threshold: set forward drive to 2 (walking gait, lower speed).
   If below threshold: set forward drive to 4 (swimming gait, higher speed).

**Gait switching**: The `contact_threshold` determines the transition between swimming and walking. When the animat's feet touch the ground (contacts > threshold), the forward drive drops to 2, which typically corresponds to a walking gait frequency. When in water (contacts < threshold), the drive is 4, corresponding to swimming.

### `compute_intention()` walkthrough

```python
def compute_intention(self):
    return [
        (self.fwds[index] - self.turn)
        if self.drive_types[index] in (DriveKind.SPINE_LEFT, DriveKind.BRAIN_LEFT)
        else (self.fwds[index] + self.turn)
        if self.drive_types[index] in (DriveKind.SPINE_RIGHT, DriveKind.BRAIN_RIGHT)
        else 0
        for index in range(self.n_drives)
    ]
```

The intention vector combines forward drive and turn command:
- **Left drives**: `fwds - turn` (turning left reduces left drive)
- **Right drives**: `fwds + turn` (turning left increases right drive)
- **Other drives** (legs): 0 (not affected by orientation follower)

This creates a differential drive: turning is achieved by reducing speed on one side and increasing on the other.

### `write_to_brain()` walkthrough

```python
def write_to_brain(self, iteration):
    for drive_index in self.drives.brain_left_indices:
        self.drives.array[min(iteration, self.n_iterations-1), drive_index] = (
            self.fwds[drive_index] - self.turn
        )
    for drive_index in self.drives.brain_right_indices:
        self.drives.array[min(iteration, self.n_iterations-1), drive_index] = (
            self.fwds[drive_index] + self.turn
        )
```

Writes the computed intention directly to brain drive indices. Uses `min(iteration, n_iterations-1)` to prevent array index overflow on the last iteration. Note: this writes the low-pass filtered `self.fwds` values, NOT the `intention` list.

## DistributedOrientationFollower

```python
class DistributedOrientationFollower(OrientationFollower):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.intention = np.zeros(self.n_drives)
        self.reaction = np.zeros(self.n_drives)
        self.contact_value = 0
```

Extends `OrientationFollower` with a more sophisticated forward control that distributes contact information across drives.

### `get_foward_control()` override

```python
def get_foward_control(self, iteration, timestep):
    threshold = 9.81 * self.contact_threshold  # Scale by gravity
    contacts = np.array(self.animat_data.sensors.contacts.totals()[iteration], copy=True)

    # Low-pass filter total contact
    self.contact_value += min(100*timestep, 1) * (np.sum(np.abs(contacts)) - self.contact_value)
    self.fwds_raw[:] = 2 if self.contact_value > threshold else 4

    # Per-drive contact reaction (distributed)
    self.contacts_values += min(100*timestep, 1) * (
        [np.sum(np.abs(contacts[indices])) for indices in self.drives.contacts_indices]
        - self.contacts_values
    )

    # Distributed decision: adjust individual drives based on local contacts
    threshold2 = 1e-3 * threshold
    self.fwds_raw[np.logical_and(self.fwds_raw > 3, self.contacts_values > threshold2)] = 2.9
    self.fwds_raw[np.logical_and(self.fwds_raw < 3, self.contacts_values < threshold2)] = 3.1

    return self.fwds_raw
```

**Key difference from OrientationFollower**:

1. Contact threshold is multiplied by `9.81` (gravity), so the threshold is in units of Newtons rather than g-force.
2. Uses `contacts.totals()` instead of `contacts.reactions()` (different contact data).
3. Computes per-drive contact reactions using `self.drives.contacts_indices`, which maps each drive to relevant contact sensors.
4. Adjusts individual drives based on local contact information: drives associated with contacts above the threshold get reduced to 2.9 (intermediate between walk=2 and swim=4), while drives without contacts get increased to 3.1.

**Note**: `self.contacts_values` is used but NOT initialized in `__init__`. This is likely a bug — it should be initialized as `np.zeros(self.n_drives)` or similar. It relies on the parent class or external initialization.

## Factory function: `get_orientation_follower_kwargs()`

```python
def get_orientation_follower_kwargs(drive_config, animat_data, simulation_options):
    potential_config = drive_config.pop('potential_map')
    potential_type: str = potential_config.pop('type')
    return {
        'strategy': {
            'line': StraightLinePotentialMap,
            'circle': CirclePotentialMap,
            'ellipsoid': EllipsoidPotentialMap,
            'disline': StraightLinePotentialMap,
            'discircle': CirclePotentialMap,
        }[potential_type](**potential_config),
        'animat_data': animat_data,
        'timestep': simulation_options.physics.timestep,
        **drive_config,
    }
```

**Potential map types**: `line`, `circle`, `ellipsoid`, `disline` (alias for line), `discircle` (alias for circle). An unknown type raises a `KeyError`.

**Mutates input**: `drive_config.pop('potential_map')` and `potential_config.pop('type')` modify the input dictionaries. This is a destructive operation — the `drive_config` dict will be missing `potential_map` after this call, and all remaining keys are unpacked as kwargs to the OrientationFollower constructor.

## How to integrate: creating a custom PotentialMap

```python
from farms_amphibious.control.drive import PotentialMap
import numpy as np

class Figure8PotentialMap(PotentialMap):
    """Figure-8 trajectory potential map"""

    def __init__(self, **kwargs):
        self.gain = kwargs.pop('gain', 1)
        self.origin = kwargs.pop('origin', np.zeros(2))
        self.scale = kwargs.pop('scale', 1)
        assert not kwargs, kwargs

    def heading(self, pos):
        # Compute desired heading for figure-8 trajectory
        rel_pos = pos[:2] - self.origin
        # ... custom trajectory math ...
        return desired_heading

    def limit_cycle(self):
        t = np.linspace(0, 2*np.pi, 200)
        x = self.scale * np.sin(t)
        y = self.scale * np.sin(t) * np.cos(t)
        return self.origin + np.column_stack([x, y])
```

Then register it in the factory function's type dict, or construct `OrientationFollower` directly:

```python
follower = OrientationFollower(
    strategy=Figure8PotentialMap(gain=0.5, scale=2.0),
    animat_data=data,
    timestep=0.001,
    pid_p=0.3,
    output_limits=(-0.8, 0.8),
)
```

## How to integrate: creating a custom DescendingDrive

```python
from farms_amphibious.control.drive import DescendingDrive

class SpeedController(DescendingDrive):
    """Simple speed controller without navigation"""

    def __init__(self, drives, forward_speed=3.0, **kwargs):
        super().__init__(drives=drives)
        self.forward_speed = forward_speed

    def step(self, iteration, time, timestep):
        # Set all left and right drives to forward_speed
        self.set_left_drive(iteration, self.forward_speed)
        self.set_right_drive(iteration, self.forward_speed)
```

## Common failure modes

### 1. PID instability

If `pid_p` is too high, the turn command oscillates violently. Symptoms: the animat spirals or zig-zags instead of following the path.

**Fix**: Reduce `pid_p` (default 0.2), increase `pid_d` for damping, or tighten `output_limits`.

### 2. Contact threshold too high/low

If `contact_threshold` is too high, the animat never switches to walking gait even when on ground. If too low, it switches to walking in water.

**Fix**: Set `contact_threshold` based on the expected contact force magnitude. For ground contact, typical values are 1–10 N (depending on animat mass). For `DistributedOrientationFollower`, the threshold is multiplied by 9.81, so use mass-equivalent values.

### 3. Drive values out of range

The CPG oscillator parameters (frequency, amplitude) are functions of the drive value. Drive values outside the expected range (typically 0–5) may produce undefined behavior — frequencies may become zero or negative, amplitudes may saturate.

**Fix**: Use `output_limits` on the PID to clamp the turn command. Check the drive-dependent function parameters in the YAML to ensure they cover the range of drive values you produce.

### 4. `contacts_values` not initialized in DistributedOrientationFollower

`DistributedOrientationFollower.get_foward_control()` uses `self.contacts_values` which is NOT initialized in `__init__`. This will raise `AttributeError` on the first call unless it was set elsewhere.

**Fix**: Add `self.contacts_values = np.zeros(self.n_drives)` to `DistributedOrientationFollower.__init__()`.

### 5. Drive array not propagated

`copy_next_drive` in `NetworkODE` copies drive values forward. If `DescendingDrive.step()` is called AFTER `NetworkODE.step()`, the new drive values won't be used until the NEXT iteration.

**Fix**: Ensure `drive.step()` is called BEFORE `network.step()` in the controller's `before_step()`. The standard `AmphibiousController` does this in the correct order.

## What NOT to assume

1. **Drive values are NOT continuous.** They are stored per-iteration in a pre-allocated array. The drive at iteration `i` is a single scalar per drive index. There is no interpolation between iterations.

2. **The forward drive values (2 and 4) are NOT hardcoded gait frequencies.** They are drive signal values that get mapped to frequencies/amplitudes through the drive-dependent piecewise-linear functions defined in the YAML. The actual frequencies depend on the oscillator parameters.

3. **`OrientationFollower` does NOT control leg drives.** The `compute_intention()` method returns 0 for drive indices that are not brain or spine left/right. Leg drives are left at their previous values (propagated by `copy_next_drive`).

4. **The heading is computed from link positions, not IMU data.** `links.heading()` computes the heading from the positions of the links specified by `links_indices`. This is a geometric heading, not an inertial measurement.

5. **`get_orientation_follower_kwargs()` mutates its input.** `drive_config.pop('potential_map')` removes the key from the dict. If you need to reuse `drive_config`, pass a copy.

6. **The `contact_threshold_dis` parameter is popped and discarded.** It exists in the constructor for backward compatibility but has no effect.
