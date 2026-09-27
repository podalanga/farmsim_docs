# farms_core Reference

`farms_core` is the foundation of FARMS: options (YAML files), data arrays
and sensors, the extension and controller base classes, I/O, and unit
scaling. Every other package imports it.

## Module map

```text
farms_core/
├── options.py           # Options: dict subclass, load()/save() YAML
├── doc.py               # ClassDoc/ChildDoc: option descriptions (doc())
├── units.py             # SimulationUnitScaling
├── pylog/               # Logging
├── array/               # Typed Cython arrays (DoubleArray1D, ...), array helpers
├── io/
│   ├── yaml.py          # yaml2pyobject, pyobject2yaml
│   ├── hdf5.py          # dict_to_hdf5, hdf5_to_dict
│   └── sdf.py           # SDF model parser (ModelSDF, Link, Joint, ...)
├── extensions/
│   └── extensions.py    # import_item(), ExtensionOptions
├── experiment/
│   ├── options.py       # ExperimentOptions, ExperimentLoadOptions
│   └── data.py          # ExperimentData
├── simulation/
│   ├── options.py       # SimulationOptions, Runtime/Physics/MuJoCo/Pybullet options, Simulator
│   ├── data.py          # SimulationData (contacts, solver iterations, energy)
│   ├── extensions.py    # TaskExtension, ExperimentLogger, ExperimentOptionsLogger
│   └── parse_args.py    # Command line helpers
├── model/
│   ├── options.py       # AnimatOptions, ArenaOptions, SpawnOptions, MorphologyOptions,
│   │                    # LinkOptions, JointOptions, ControlOptions, MotorOptions,
│   │                    # SensorsOptions, WaterOptions, ...
│   ├── data.py          # AnimatData
│   ├── control.py       # AnimatController, ControlType
│   └── extensions.py    # AnimatExtension
├── sensors/
│   ├── data.py          # SensorsData and the sensor arrays
│   ├── sensor_convention.pxd/.pyx  # Column indices, sc enum
│   └── data_cy.pyx      # Cython accessors
├── utils/               # profile(), transforms (quaternions)
└── analysis/            # Plot style and metrics
```

## Where to find what

| Topic | Page |
|-------|------|
| Every YAML key | [Configuration Parameter Reference](../env/configuration-reference.md) (generated) |
| Options classes | [farms_core.model.options](core-options.md) |
| Extensions, controllers, loggers | [Core Control](core-control.md) |
| Sensor arrays | [Core Sensors](core-sensors.md) |
| SDF, HDF5, YAML | [Core I/O](core-io.md) |
| Data classes and files | [Data Flow and Data Model](../../explanation/data-flow.md) |
| Full API | [Generated API reference](../api/farms_core/index.md) |

## import_item

`farms_core.extensions.extensions.import_item(path)` imports a class or
function from its dotted path (`package.module.Name`). It resolves every
`loader` of the YAML files: the classes of `loaders:` in the experiment
file, and the extensions.

::: farms_core.extensions.extensions.import_item
    options:
      show_root_heading: false
      heading_level: 3

## Units

`SimulationUnitScaling` (`simulation.units` in the simulation file) scales
the MuJoCo model: 1 m in reality is `meters` in the simulation, and so on
for `seconds` and `kilograms`. The derived factors (`newtons`, `torques`,
`velocity`, `angular_velocity`, `stiffness`, `damping`, `inertia`, ...)
convert the other quantities. The data arrays are in SI units. Extensions
get the factors from `task.units`.

::: farms_core.units.SimulationUnitScaling
    options:
      show_root_heading: false
      heading_level: 3
      members: false

## See also

- [System Architecture](../../explanation/architecture.md)
- [Options and YAML Design](../../explanation/options-yaml-design.md)
