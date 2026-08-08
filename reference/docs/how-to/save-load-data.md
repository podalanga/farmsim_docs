# Save, Load, and Inspect Data

This guide explains how FARMS persists simulation data and how to load and
analyze it after a run.

## Data persistence format

FARMS saves simulation data as HDF5 files via the `ExperimentData` class
(`farms_core/experiment/data.py`). The `ExperimentLogger` extension triggers
the save at episode end.

### HDF5 structure

```
simulation.hdf5
├── times              # (n_iterations,) — simulation time at each step
├── timestep           # scalar — physics timestep
├── simulation/        # SimulationData (units, etc.)
│   ├── units/
│   │   ├── length
│   │   ├── angle
│   │   ├── seconds
│   │   └── ...
│   └── ...
├── animat_0/          # AnimatData for first animat
│   ├── state/         # CPG network state (phases, amplitudes)
│   ├── network/       # Network logs (drives, connectivity)
│   ├── sensors/
│   │   ├── links/     # (n_iterations, n_links, 19) — link state
│   │   ├── joints/    # (n_iterations, n_joints, 3) — joint state
│   │   ├── contacts/  # (n_iterations, n_contacts, 7) — contact forces
│   │   ├── xfrc/      # (n_iterations, n_links, 6) — external forces
│   │   ├── muscles/   # muscle activations (if applicable)
│   │   ├── adhesions/ # adhesion forces (if applicable)
│   │   └── visuals/   # visual sensor data (if applicable)
│   └── ...
├── animat_1/          # Second animat (if multi-animat)
└── ...
```

## Loading data

### From Python

```python
from farms_core.experiment.data import ExperimentData

# Load the full experiment data
data = ExperimentData.from_file('simulation.hdf5')

# Access simulation metadata
print(f"Duration: {data.times[-1]:.2f} s")
print(f"Timestep: {data.timestep:.4f} s")
print(f"N iterations: {len(data.times)}")

# Access first animat's data
animat = data.animats[0]
joint_positions = animat.sensors.joints.array[:, :, 0]  # (n_iters, n_joints)
link_positions = animat.sensors.links.array[:, :, :3]    # (n_iters, n_links, 3)
```

### Accessing specific sensors

```python
# Joint data
joints = animat.sensors.joints
joint_names = joints.names  # list of joint name strings
positions = joints.array[:, :, 0]   # all iterations, all joints, position
velocities = joints.array[:, :, 1] # all iterations, all joints, velocity
torques = joints.array[:, :, 2]     # all iterations, all joints, torque

# Link data (19 columns: 3 pos + 4 quat + 3 lin_vel + 3 ang_vel + 3 lin_acc + 3 ang_acc)
links = animat.sensors.links
link_names = links.names
positions = links.array[:, :, :3]       # x, y, z position
quaternions = links.array[:, :, 3:7]     # w, x, y, z quaternion
linear_vel = links.array[:, :, 7:10]    # linear velocity
angular_vel = links.array[:, :, 10:13]  # angular velocity

# Contact forces (7 columns: 3 force + 3 torque + 1 normal)
contacts = animat.sensors.contacts
contact_names = contacts.names
forces = contacts.array[:, :, :3]

# External forces (6 columns: 3 force + 3 torque)
xfrc = animat.sensors.xfrc
xfrc_names = xfrc.names
forces = xfrc.array[:, :, :3]
```

### Network state data

If the CPG network was active, state data is available:

```python
# CPG oscillator states (phases and amplitudes interleaved)
state = animat.state.array  # (n_iterations, n_states)
phases = state[:, :n_oscillators]
amplitudes = state[:, n_oscillators:2*n_oscillators]

# Network drives
drives = animat.network.drives.array  # (n_iterations, n_drives)
```

## Saving data programmatically

```python
from farms_core.experiment.data import ExperimentData

# Save to HDF5
data.to_file('output.hdf5')

# Load from HDF5
data = ExperimentData.from_file('output.hdf5')
```

## Loading saved options

The `ExperimentOptionsLogger` extension saves copies of all YAML configuration
files. These can be reloaded:

```python
from farms_core.experiment.options import ExperimentOptions

options = ExperimentOptions.load('options/experiment_config.yaml')
print(options.simulation.run.duration)
```

## Analysis tips

### Computing forward velocity

```python
import numpy as np

# Get head link position over time
head_idx = link_names.index('link_head')
head_positions = links.array[:, head_idx, :3]

# Forward velocity (x-direction)
forward_velocity = np.gradient(head_positions[:, 0], data.times)
```

### Computing tail beat frequency

```python
# Get tail joint angle over time
tail_idx = joint_names.index('joint_10')  # adjust to your robot
tail_angle = joints.array[:, tail_idx, 0]

# FFT to find dominant frequency
from scipy.fft import fft, fftfreq
dt = data.timestep
yf = np.abs(fft(tail_angle))
xf = fftfreq(len(tail_angle), dt)
dominant_freq = xf[np.argmax(yf[:len(yf)//2])]
```

## The ExperimentData class

`ExperimentData` (`farms_core/experiment/data.py`) is the top-level container:

| Attribute | Type | Description |
|-----------|------|-------------|
| `times` | np.ndarray | Time at each iteration [s] |
| `timestep` | float | Physics timestep [s] |
| `simulation` | SimulationData | Simulation-level data (units, etc.) |
| `animats` | list[AnimatData] | One per animat |

### AnimatData

| Attribute | Type | Description |
|-----------|------|-------------|
| `sensors` | SensorsData | All sensor arrays |
| `state` | StateData | CPG network state (if applicable) |
| `network` | NetworkLog | Network logs (if applicable) |

### SimulationUnitScaling

Unit scaling is stored in `SimulationData.units` and provides conversion factors
between simulation units and SI units:

| Attribute | Description |
|-----------|-------------|
| `length` | Meters per simulation length unit |
| `angle` | Radians per simulation angle unit |
| `seconds` | Seconds per simulation time unit |

Access via `task.units` in extensions:

```python
sim_time = physics.time() / task.units.seconds
dt = physics.timestep() / task.units.seconds
```

## See also

- [Data Model](../reference/env/data-model.md) — full class reference
- [Use Built-in Extensions](use-extensions.md) — configuring ExperimentLogger
- [Data Flow and Persistence](../explanation/data-flow.md) — design rationale
