# Add and Configure Sensors

This guide explains how to configure sensors in FARMS — what sensor types are
available, how to declare them in YAML, and how to access sensor data in code.

## Sensor types

FARMS defines seven sensor categories, each storing data as a NumPy array in
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

!!! bug "Previous revision of this page had wrong column counts"
    An earlier draft of this table listed `links` as 19 columns including
    "3 linear + 3 angular acceleration" fields, `joints` as 3 columns
    (position/velocity/torque only), and `contacts` as 7 columns
    ("3 force + 3 torque + 1 normal magnitude"). None of that matches the
    `sc` enum: there are **no acceleration fields** on links at all (links
    store *two* position/orientation pairs instead — `com_*` and `urdf_*` —
    which is where the extra columns actually come from); joints have 17
    columns, not 3, including command values and torque decomposition
    (`active`/`stiffness`/`damping`/`friction`); and contacts have 12
    columns organized as `reaction`/`friction`/`total`/`position` vectors,
    not a force+torque+magnitude layout. The table above is read directly
    from `sensor_convention.pxd`/`sensor_convention.pyx`.

## Declaring sensors in YAML

Sensors are declared in `animat_config.yaml` under `control.sensors`:

```yaml
control:
  sensors:
    links:
      - link_head
      - link_body_0
      - link_body_1
    joints:
      - joint_head_yaw
      - joint_0
      - joint_1
    contacts:
      - link_head
      - link_tail
    xfrc:
      - link_head
      - link_body_0
      - link_body_1
    muscles: []
    adhesions: []
    visuals: []
```

Each entry is a list of link or joint names. The names must match the SDF/MJCF
model definition.

!!! note "`contacts` entries can be link-pairs, not just link names"
    `SensorsOptions.contacts` is typed `list[str] | list[list[str]]`
    (`farms_core/model/options.py`). A plain list of link names tracks total
    contact reaction on each named link (contact with anything); a list of
    `[link_a, link_b]` pairs restricts tracking to contacts between that
    specific pair of links. Mixing forms across a single YAML `contacts:`
    list is only as safe as whatever consumes it downstream — check
    `AmphibiousSensorsOptions.defaults_from_convention()` or your loader
    before relying on mixed forms.

### Column layout per category

See the table above for the authoritative column counts and shapes, taken
directly from `sc` in `sensor_convention.pyx` — do not re-derive them from
memory, as they don't map onto an obvious "3 position + 3 velocity" pattern
for every category (joints and contacts in particular carry several extra
derived/decomposed fields beyond the raw physical quantities).

## Amphibious sensor defaults

When using `AmphibiousOptions` (`farms_amphibious/model/options.py`), sensors
can be auto-populated from the morphology convention via
`AmphibiousSensorsOptions.defaults_from_convention()`:

| Sensor | Default if not specified |
|--------|------------------------|
| `links` | All link names from convention |
| `joints` | All joint names from convention |
| `contacts` | Feet link names from convention |
| `xfrc` | All link names from convention |

To override, specify the names explicitly in the YAML.

## Accessing sensor data in code

Sensor data is stored in `AnimatData.sensors`, accessible from any extension
or controller:

```python
class MyExtension(AnimatExtension):
    def before_step(self, task, action, physics):
        iteration = task.iteration

        # Access joint sensor data
        joint_positions = self.animat_data.sensors.joints.array[iteration, :, 0]
        joint_velocities = self.animat_data.sensors.joints.array[iteration, :, 1]
        joint_torques = self.animat_data.sensors.joints.array[iteration, :, 2]

        # Access link sensor data
        link_positions = self.animat_data.sensors.links.array[iteration, :, :3]

        # Access contact forces
        contact_forces = self.animat_data.sensors.contacts.array[iteration, :, :3]

        # Access xfrc (external forces)
        xfrc = self.animat_data.sensors.xfrc.array[iteration, :, :3]
```

### Sensor name lookup

Each sensor array has a `.names` list for index lookup:

```python
joint_names = self.animat_data.sensors.joints.names
joint_i = joint_names.index('joint_0')
joint_position = self.animat_data.sensors.joints.array[iteration, joint_i, 0]
```

## How sensors are updated

Sensor data is updated every step in `ExperimentTask.update_sensors()` (in
`farms_mujoco/simulation/task.py`). This method reads from the MuJoCo physics
state and writes into the pre-allocated arrays. It is called at the beginning
of each `before_step()`, before any extension code runs.

The update reads from:

- `physics.data.qpos` — joint positions
- `physics.data.qvel` — joint velocities
- `physics.data.actuator_force` — joint torques (applied)
- `physics.data.xpos` / `physics.data.xquat` — link positions/orientations
- `physics.data.collision` — contact data
- `physics.data.xfrc_applied` — external forces (e.g., from SwimmingExtension)

## Adding a custom sensor type

FARMS does not provide a plugin mechanism for new sensor array types. If you
need custom sensor data, the recommended approach is to store it in an
`AnimatExtension`:

```python
class MySensorExtension(AnimatExtension):
    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                    animat_data, animat_options):
        n_iterations = experiment_options.simulation.run.n_iterations
        extension = cls(
            animat_i=animat_i,
            animat_data=animat_data,
            animat_options=animat_options,
        )
        # Pre-allocate custom sensor array
        extension.custom_data = np.zeros((n_iterations, n_sensors, data_dim))
        return extension

    def after_step(self, task, action, physics):
        # Record custom sensor data
        self.custom_data[task.iteration] = compute_sensor_reading(physics)
```

## See also

- [YAML Configuration Schema](../reference/env/yaml-schema.md) — complete sensor
  option keys
- [Data Model](../reference/env/data-model.md) — `SensorsData` class reference
- [Write an AnimatExtension](write-extension.md) — extension lifecycle
