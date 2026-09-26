# Add and Configure Sensors

This guide explains how to configure sensors in FARMS, what sensor types are
available, how to declare them in YAML, and how to access sensor data in code.

## Sensor types

FARMS defines eight sensor categories, each storing data as a NumPy array in
`AnimatData.sensors`. The categories are defined in `SensorsOptions`
(`farms_core/model/options.py`); the exact per-sensor column layout for each
category is defined by the `sc` (sensor convention) Cython enum in
`farms_core/sensors/sensor_convention.pyx`, and every array is shaped
`(buffer_size, n_sensors, sc.<category>_size)`:

| Category | YAML key | `sc` size constant | Columns | Array shape |
|----------|----------|--------------------|---------|-------------|
| Links | `links` | `link_size` = 20 | `com_position` (3) + `com_orientation` quat (4) + `urdf_position` (3) + `urdf_orientation` quat (4) + `com_velocity_lin` (3) + `com_velocity_ang` (3) | `(n_iters, n_links, 20)` |
| Joints | `joints` | `joint_size` = 17 | `position`, `velocity`, `torque` (1 each) + `force_x/y/z` (3) + `torque_x/y/z` (3) + `cmd_position`, `cmd_velocity`, `cmd_torque` (1 each) + `torque_active`, `torque_stiffness`, `torque_damping`, `torque_friction`, `limit_force` (1 each) | `(n_iters, n_joints, 17)` |
| Contacts | `contacts` | `contact_size` = 12 | `reaction_x/y/z` (3) + `friction_x/y/z` (3) + `total_x/y/z` (3) + `position_x/y/z` (3) | `(n_iters, n_contacts, 12)` |
| XFRC | `xfrc` | `xfrc_size` = 6 | `force_x/y/z` (3) + `torque_x/y/z` (3) | `(n_iters, n_links, 6)` |
| Muscles | `muscles` | `muscle_size` = 17 | excitation/activation + tendon/fiber length, velocity, force fields + Ia/II/Ib feedback (see `sensor_convention.pyx`) | `(n_iters, n_muscles, 17)` |
| Adhesions | `adhesions` | `adhesion_size` = 1 | `force` | `(n_iters, n_adhesions, 1)` |
| Visuals | `visuals` | `visual_size` = 8 | `color_r/g/b/a` (4) + `emission_r/g/b/i` (4) | `(n_iters, n_visuals, 8)` |
| Rays | `rays` | `ray_size` = 8 | `distance` + `origin_x/y/z` (3) + `direction_x/y/z` (3) + `hit_x` | `(n_iters, n_rays, 8)` |

## Declaring sensors in YAML

Sensors are declared in `animat_config.yaml` under `control.sensors`:

```yaml
control:
  sensors:
    links: [Head, Segment1, Segment2]
    joints: [joint_1, joint_2]
    contacts: [Head, TailSegment]
    xfrc: [Head, Segment1, Segment2]
    muscles: []
    adhesions: []
    visuals: []
    rays:
      - name: head_range
        link_name: Head
        pos: [0, 0, 0]
        quat: [0.7071, 0, 0.7071, 0]  # Rotates the site z axis onto the link x axis
        cutoff: 2.0                   # [m], null for no limit
```

`rays` also accepts a list of link names. A ray is a MuJoCo rangefinder
cast along the local z axis of its site (`RaySensorOptions`).

Each entry is a list of link or joint names. The names must match the SDF/MJCF
model definition.

!!! note "`contacts` entries can be link-pairs, not just link names"
    `SensorsOptions.contacts` is typed `list[str] | list[list[str]]`
    (`farms_core/model/options.py`). A plain list of link names tracks total
    contact reaction on each named link (contact with anything); a list of
    `[link_a, link_b]` pairs restricts tracking to contacts between that
    specific pair of links. Mixing forms across a single YAML `contacts:`
    list is only as safe as whatever consumes it downstream, check
    `AmphibiousSensorsOptions.defaults_from_convention()` or your loader
    before relying on mixed forms.

### Column layout per category

See the table above for the authoritative column counts and shapes, taken
directly from `sc` in `sensor_convention.pyx`, do not re-derive them from
memory, as they don't map onto an obvious "3 position + 3 velocity" pattern
for every category (joints and contacts in particular carry several extra
derived/decomposed fields beyond the raw physical quantities).

## Amphibious sensor defaults

`AmphibiousOptions.from_options()` (options built in Python from a
morphology convention, not from YAML) fills the sensors from the
convention with `AmphibiousSensorsOptions.defaults_from_convention()`:
all links for `links` and `xfrc`, all joints for `joints`, and the feet
for `contacts`, unless `sensors_links`, `sensors_joints`,
`sensors_contacts` or `sensors_xfrc` are given. A YAML file must list the
sensors explicitly.

## Accessing sensor data in code

An `AnimatExtension` receives its `AnimatData` in `from_options()`: keep a
reference to it. The arrays are ring buffers of `runtime.buffer_size`
iterations, indexed with `task.iteration % task.buffer_size`:

```python
from farms_core.model.extensions import AnimatExtension
from farms_core.sensors.sensor_convention import sc


class MyExtension(AnimatExtension):

    def __init__(self, animat_data):
        super().__init__()
        self.animat_data = animat_data

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                     animat_data, animat_options):
        return cls(animat_data=animat_data)

    def before_step(self, task, action, physics):
        index = task.iteration % task.buffer_size
        sensors = self.animat_data.sensors
        joint_positions = sensors.joints.array[index, :, sc.joint_position]
        joint_velocities = sensors.joints.array[index, :, sc.joint_velocity]
        com = sensors.links.array[index, :, sc.link_com_position_x:sc.link_com_position_z+1]
        contact_forces = sensors.contacts.array[index, :, sc.contact_total_x:sc.contact_total_z+1]
        fluid_forces = sensors.xfrc.array[index, :, sc.xfrc_force_x:sc.xfrc_force_z+1]
```

In an `AnimatController`, the `iteration` argument of `positions()` and
the other command methods is already this index. Other extensions can
reach the data through `task.data.animats[animat_i]`.

Each category has a `.names` list, in the order of the YAML list:

```python
joint_i = sensors.joints.names.index('joint_1')
```

The arrays also have accessors, for example
`sensors.links.global_com_position(iteration)` or
`sensors.joints.position(iteration, joint_i)`.

## How sensors are updated

`ExperimentTask.update_sensors()` (`farms_mujoco/simulation/task.py`)
copies the MuJoCo state into the arrays at the start of each environment
step, before the extensions run (only the links on the substeps). It uses
maps built at the start of the episode between the MuJoCo sensors and
arrays and the sensor arrays (`farms_mujoco/simulation/physics.py`, see
[Physics Sensor Mapping](../internals/physics-sensor-mapping.md)):

- links: MuJoCo frame sensors and body poses, CoM velocities;
- joints: joint positions and velocities, actuator forces, joint force
  and torque sensors;
- contacts: MuJoCo contacts involving the listed links
  (`farms_mujoco/sensors/sensors.pyx`);
- muscles: MuJoCo muscle actuators;
- xfrc: not read from MuJoCo, but written by the extensions that apply
  external forces (`SwimmingExtension` writes the fluid wrench).

## Adding a custom quantity

There is no plugin mechanism for new sensor categories. Store custom data
in an extension, allocated in `from_options()`:

```python
import numpy as np


class PowerLogger(AnimatExtension):
    """Mechanical power of each joint at each iteration"""

    def __init__(self, animat_data, n_iterations):
        super().__init__()
        self.animat_data = animat_data
        n_joints = len(animat_data.sensors.joints.names)
        self.power = np.zeros((n_iterations, n_joints))

    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                     animat_data, animat_options):
        n_iterations = experiment_options.simulation.runtime.n_iterations
        return cls(animat_data=animat_data, n_iterations=n_iterations)

    def after_step(self, task, physics):
        index = (task.iteration - 1) % task.buffer_size  # Iteration just completed
        joints = np.asarray(self.animat_data.sensors.joints.array)
        self.power[task.iteration - 1] = (
            joints[index, :, sc.joint_torque]*joints[index, :, sc.joint_velocity]
        )

    def end_episode(self, task, physics):
        np.save('Output/power.npy', self.power)
```

## See also

- [Configuration Parameter Reference](../reference/env/configuration-reference.md): sensor option keys
- [Data Flow and Data Model](../explanation/data-flow.md): the sensor arrays
- [Write an AnimatExtension](write-extension.md): extension lifecycle
