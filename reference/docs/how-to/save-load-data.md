# Save, Load, and Inspect Data

How FARMS saves the simulation data, and how to load and analyse it after
a run. The data classes are described in
[Data Flow and Data Model](../explanation/data-flow.md).

## Saving

Two simulation extensions save a run (in `simulation_config.yaml`):

```yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentOptionsLogger
    config:
      log_path: Output   # simulation_options.yaml, animat_0_options.yaml, arena_0_options.yaml
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: Output   # simulation.hdf5, written at the end of the simulation
      skip: 1
  - loader: farms_mujoco.simulation.extensions.MjcfSaver
    config:
      path: Output/simulation_mjcf.xml
```

`ExperimentLogger` writes the arrays held in memory, that is the last
`runtime.buffer_size` iterations. Keep `buffer_size` equal to
`n_iterations` (the default) to save the whole run.

From Python, `ExperimentData.to_file('simulation.hdf5')` saves the data.

## HDF5 structure

The file mirrors `ExperimentData.to_dict()`. Lists are stored as groups
whose name starts with `FARMSLIST`:

```text
simulation.hdf5
├── times                          (n_iterations,)
├── timestep                       ()
├── simulation/
│   ├── ncon                       (n_iterations,) number of contacts
│   ├── niter                      (n_iterations,) solver iterations
│   └── energy                     (n_iterations, 2)
└── FARMSLISTanimats/
    └── 0/
        ├── sensors/
        │   ├── links/array        (buffer_size, n_links, 20), names, masses
        │   ├── joints/array       (buffer_size, n_joints, 17), names
        │   ├── contacts/array     (buffer_size, n_contacts, 12), names
        │   ├── xfrc/array         (buffer_size, n_xfrc, 6), names
        │   ├── muscles/, adhesions/, visuals/, rays/
        ├── state                  AmphibiousData only: CPG state
        └── network/               AmphibiousData only: drives, connectivity
```

## Loading

```python
import numpy as np
from farms_core.experiment.data import ExperimentData
from farms_core.sensors.sensor_convention import sc

data = ExperimentData.from_file('Output/simulation.hdf5')
print(f'{len(data.times)} iterations, timestep {data.timestep} s')

animat = data.animats[0]
joints = animat.sensors.joints
links = animat.sensors.links
xfrc = animat.sensors.xfrc
```

The columns are given by the sensor convention `sc`:

```python
# Joints: (n_iterations, n_joints)
positions = np.asarray(joints.array)[:, :, sc.joint_position]      # [rad]
velocities = np.asarray(joints.array)[:, :, sc.joint_velocity]     # [rad/s]
commands = np.asarray(joints.array)[:, :, sc.joint_cmd_position]   # [rad]
print(joints.names)

# Links: CoM position (n_iterations, n_links, 3) and velocity
com = np.asarray(links.array)[:, :, sc.link_com_position_x:sc.link_com_position_z+1]
com_velocity = np.asarray(links.array)[:, :, sc.link_com_velocity_lin_x:sc.link_com_velocity_lin_z+1]
print(links.names)

# Centre of mass of the whole animat at an iteration
print(links.global_com_position(iteration=len(data.times) - 1))

# External forces, including the fluid forces: (n_iterations, n_links, 3)
forces = np.asarray(xfrc.array)[:, :, sc.xfrc_force_x:sc.xfrc_force_z+1]
```

### CPG state

`ExperimentData.from_file()` loads the animats as `AnimatData`, without the
CPG state. Read it from the file directly:

```python
from farms_core.io.hdf5 import hdf5_to_dict

raw = hdf5_to_dict('Output/simulation.hdf5')
state = np.asarray(raw['animats'][0]['state'])  # (n_iterations, 2*n_osc + n_joints)
n_osc = 14                                      # AmphiBot: 2 per body joint
phases = state[:, :n_osc]
amplitudes = state[:, n_osc:2*n_osc]
offsets = state[:, 2*n_osc:]
```

!!! note
    `AmphibiousData.from_dict()` cannot load this dictionary (it expects an
    `n_oscillators` entry that `to_dict()` does not write).

## Loading the options

The options saved by `ExperimentOptionsLogger` can be loaded with their
classes:

```python
from farms_core.simulation.options import SimulationOptions
from farms_amphibious.model.options import AmphibiousOptions

sim_options = SimulationOptions.load('Output/simulation_options.yaml')
animat_options = AmphibiousOptions.load('Output/animat_0_options.yaml')
print(sim_options.duration(), sim_options.physics.timestep)
```

## Analysis examples

### Forward speed

```python
head = list(links.names).index('head')
head_xy = com[:, head, :2]
speed = np.linalg.norm(np.gradient(head_xy, data.times, axis=0), axis=1)
print(f'Mean speed: {speed[len(speed)//2:].mean():.3f} m/s')
```

### Tail beat frequency

```python
from scipy.fft import rfft, rfftfreq

tail = list(joints.names).index('joint7')
angle = positions[:, tail] - positions[:, tail].mean()
spectrum = np.abs(rfft(angle))
frequencies = rfftfreq(len(angle), data.timestep)
print(f'Dominant frequency: {frequencies[np.argmax(spectrum[1:]) + 1]:.2f} Hz')
```

`examples/amphibot/plot_run.py` plots the trajectory, the joint angles
and the water forces of a run, see
[Record, load and plot data](../tutorials/record-and-plot.md).

## See also

- [Data Flow and Data Model](../explanation/data-flow.md)
- [Add and Configure Sensors](configure-sensors.md)
- [Use Built-in Extensions](use-extensions.md)
