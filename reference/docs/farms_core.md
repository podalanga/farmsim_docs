# `farms_core` — Framework Foundation

`farms_core` is the base layer of FARMS. It defines data schemas, pre-allocated sensor arrays, configuration dataclasses, the extension/controller interface, and I/O utilities. It has no dependency on any physics engine.

**Source**: `farms_core/farms_core/`

---

## Overview

`farms_core` separates two orthogonal concerns: **configuration** (Options classes, parsed from YAML before the simulation starts) and **runtime state** (Data classes, pre-allocated NumPy arrays written during the simulation loop).

A third concern — the **extension system** — defines the interface contract that all domain-specific code (controllers, hydrodynamics, loggers) must implement to hook into the simulation lifecycle.

---

## Options and Data

### Options

Options classes are **`dict` subclasses** deserialised from YAML via `Options.load(path)`. They are validated at load time; incorrect field types or missing required fields raise exceptions immediately, before any physics engine is initialised.

The hierarchy:

```
ExperimentOptions
├── simulation: SimulationOptions
├── animats: list[AnimatOptions]
│   ├── morphology: MorphologyOptions
│   │   ├── links: list[LinkOptions]     ← drag_coefficients, density
│   │   └── joints: list[JointOptions]  ← limits, initial state
│   ├── spawn: SpawnOptions
│   ├── control: ControlOptions
│   │   └── motors: list[MotorOptions]  ← equation, control_types
│   └── sensors: SensorsOptions
└── arenas: list[ArenaOptions]
    └── water: WaterOptions
```

See [farms_core.model.options](api/farms_core_options.md) for the full class reference and parameter tables.

### Data

Data objects are pre-allocated NumPy arrays initialised with shape `[buffer_size, n_sensors, data_dim]`. The buffer uses modulo indexing (`iteration % buffer_size`) to avoid dynamic allocation during the hot loop.

The hierarchy:

```
ExperimentData
└── animats: list[AnimatData]
    └── sensors: SensorsData
        ├── links: LinkSensorArray      ← positions, velocities, orientations
        ├── joints: JointSensorArray    ← positions, velocities, torques
        ├── contacts: ContactsArray     ← reaction forces
        ├── xfrc: XfrcArray            ← external forces applied
        └── muscles: MusclesArray       ← muscle torque components
```

See [farms_core.sensors.data](api/farms_core_sensors.md) for the full sensor array reference.

---

## The Extension System

All domain-specific runtime logic hooks into the physics loop via the `TaskExtension` abstract base class. This is the primary extension point for custom controllers, force models, and loggers.

```python
class TaskExtension(ABC):
    def initialize_episode(self, task, physics): ...  # called once at start
    def before_step(self, task, action, physics): ... # called before physics.step()
    def after_step(self, task, physics): ...          # called after physics.step()
```

The specialisation chain:

```
TaskExtension (ABC)
└── AnimatExtension (ABC)        ← bound to a specific animat index
    └── AnimatController (ABC)   ← provides positions/velocities/torques/excitations
```

For the full interface contract including all 11 `TaskExtension` methods and the 7 `AnimatController` output methods, see [farms_core control interfaces](api/farms_core_control.md).

### Implementing a custom controller

```python
from farms_core.model.control import AnimatController
import numpy as np

class SineWaveController(AnimatController):
    def positions(self, iteration: int, time: float, timestep: float) -> dict[str, float]:
        target = np.sin(time * 2.0 * np.pi)
        return {
            joint: target * 0.5
            for joint in self.joints_names[0]  # ControlType.POSITION
        }
```

Register it in your experiment YAML under `animats[0].extensions`.

---

## I/O

| Module | Purpose |
|--------|---------|
| `farms_core.io.sdf` | Parse SDF/URDF robot model files → link and joint lists |
| `farms_core.io.hdf5` | Save/load `AnimatData` sensor arrays to HDF5 |
| `farms_core.io.yaml` | Load YAML to Python objects (`yaml2pyobject`) |

See [farms_core.io](api/farms_core_io.md) for the function reference.

---

## Built-in Extensions

`farms_core` ships two ready-to-use `TaskExtension` implementations:

| Class | Behaviour |
|-------|-----------|
| `ExperimentLogger` | Saves `simulation.hdf5` to `log_path` at end of episode |
| `ExperimentOptionsLogger` | Saves YAML copies of all options to `log_path` at episode start |

Register either in YAML under `simulation.extensions` — no custom code required.

---

## See Also

- [Controller & Extension Interfaces](api/farms_core_control.md)
- [Sensor Data Arrays](api/farms_core_sensors.md)
- [Options Hierarchy](api/farms_core_options.md)
- [I/O: SDF, HDF5, YAML](api/farms_core_io.md)
- [System Architecture](./architecture.md)
- **Source**: `farms_core/farms_core/`