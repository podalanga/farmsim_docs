# farms_amphibious Reference

API reference for `farms_amphibious` — amphibious robot control with CPG
oscillator networks, descending drives, and multiple muscle equations.

## Module structure

The tree below reflects the actual repository layout
(`farms/farms_amphibious/farms_amphibious/`). Cython source lives in `.pyx`
files with matching `.pxd` declaration files, not plain `.py`.

```
farms_amphibious/
├── model/
│   ├── options.py         # AmphibiousOptions + all sub-options
│   └── convention.py      # AmphibiousConvention (naming/indexing)
├── control/
│   ├── amphibious.py      # JointMuscleController, AmphibiousController,
│   │                      # AmphibiousDriveController (default controllers)
│   ├── network.py         # AnimatNetwork, NetworkODE (ODE integrator)
│   ├── drive.py           # DescendingDrive, OrientationFollower, PotentialMaps
│   ├── kinematics.py      # KinematicsController (replay-only, no physics)
│   ├── manta_control.py   # Manta-ray control experiment code — not
│   │                      # imported/referenced anywhere else in the repo;
│   │                      # dead code as far as the Zbot runtime is concerned
│   ├── ode.pyx / ode.pxd                          # ode_oscillators_sparse (CPG ODE function, Cython)
│   ├── ekeberg.pyx / ekeberg.pxd                  # Ekeberg muscle model (Cython)
│   ├── passive_cy.pyx / passive_cy.pxd            # Passive joint control (Cython)
│   ├── position_phase_cy.pyx / .pxd               # Phase-based position control (Cython)
│   ├── position_muscle_cy.pyx / .pxd              # Muscle position control (Cython)
│   ├── muscle_cy.pyx / muscle_cy.pxd              # Shared muscle actuator base (Cython)
│   └── joints_control_cy.pyx / .pxd               # Joint control base (Cython)
├── data/
│   ├── data.py            # AmphibiousData
│   ├── data_cy.pyx / data_cy.pxd   # Cython-backed data containers
│   └── network.py         # DriveArray, OscillatorNetworkState,
│                           # OscillatorConnectivity, JointsConnectivity,
│                           # ContactsConnectivity, XfrcConnectivity, NetworkParameters
├── bullet/                # PyBullet physics backend (alternative to MuJoCo;
│   ├── animat.py          # not used by the bundled Zbot MuJoCo experiments)
│   ├── interface.py
│   └── simulation.py
├── analysis/
│   ├── metrics.py         # Post-hoc gait/trajectory metrics
│   └── plot.py            # Plotting helpers for analysis scripts
└── utils/
    ├── network.py         # Network utility functions
    ├── parse_args.py      # CLI argument parsing helpers
    └── prompt.py           # Interactive prompt helpers
```

!!! note "Not part of the Zbot runtime path"
    `control/manta_control.py` (a Manta-ray experiment left in the control
    package, unreferenced elsewhere) and the whole `bullet/` backend
    (PyBullet integration, superseded by the MuJoCo backend used by the
    bundled Zbot experiments) can both be treated as inactive/legacy code
    when tracing how a Zbot simulation actually runs.

## Options classes

**Source:** `farms_amphibious/model/options.py`

### AmphibiousOptions

```python
class AmphibiousOptions(AnimatOptions):
    def __init__(self, sdf, **kwargs):
        ...
```

Extends `AnimatOptions` with:

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `show_xfrc` | bool | — | Visualize external forces |
| `scale_xfrc` | int | — | Force visualization scale |
| `mujoco` | dict | — | MuJoCo-specific options |
| `morphology` | `AmphibiousMorphologyOptions` | — | Extended morphology |
| `control` | `AmphibiousControlOptions` or `KinematicsControlOptions` | — | Extended control |

The control type is `KinematicsControlOptions` if `kinematics_file` is present
in the control dict, otherwise `AmphibiousControlOptions`.

### AmphibiousMorphologyOptions

Extends `MorphologyOptions` with body/leg convention parameters:

| Attribute | Type | Description |
|-----------|------|-------------|
| `n_joints_body` | int | Number of body joints |
| `n_links_body` | int | Number of body links (default: n_joints_body + 1) |
| `n_dof_legs` | int | DOF per leg |
| `n_legs` | int | Number of legs |
| `n_joints_passive` | int | Number of passive joints |

### AmphibiousControlOptions

```python
class AmphibiousControlOptions(ControlOptions):
    def __init__(self, **kwargs):
        ...
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `controller_loader` | str | Default: `farms_amphibious.control.amphibious.JointMuscleController` |
| `sensors` | `AmphibiousSensorsOptions` | Sensor configuration |
| `motors` | list[`AmphibiousMotorOptions`] | Motor configurations |
| `network` | `AmphibiousNetworkOptions` \| None | CPG network (None if not configured) |
| `muscles` | list[`AmphibiousMuscleSetOptions`] | Muscle set configurations |
| `hill_muscles` | list | Hill muscle definitions |
| `adhesions` | list[`AmphibiousAdhesionsOptions`] | Adhesion configurations |
| `visuals` | list[`AmphibiousVisualsOptions`] | Visual configurations |

### AmphibiousNetworkOptions

```python
class AmphibiousNetworkOptions(Options):
    def __init__(self, **kwargs):
        ...
```

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `drive_loader` | str | `''` | Drive loader path |
| `drive_config` | str | `''` | Drive config file |
| `drives` | list[`AmphibiousDriveOptions`] | `[]` | Descending drives |
| `oscillators` | list[`AmphibiousOscillatorOptions`] | `[]` | CPG oscillators |
| `single_osc_body` | bool | `False` | One osc per body joint |
| `single_osc_legs` | bool | `False` | One osc per leg joint |
| `osc2osc` | list \| None | `None` | Oscillator connections |
| `joint2osc` | list \| None | `None` | Joint sensor → osc |
| `contact2osc` | list \| None | `None` | Contact → osc |
| `xfrc2osc` | list \| None | `None` | XFRC → osc |
| `drive2osc` | list \| None | `None` | Drive → osc mapping |
| `drive2joint` | list \| None | `None` | Drive → joint mapping |

### AmphibiousOscillatorOptions

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | str | — | Oscillator name |
| `initial_phase` | float | — | Initial phase [rad] |
| `initial_amplitude` | float | — | Initial amplitude |
| `frequency_gain` | float | — | Frequency × drive |
| `frequency_bias` | float | — | Frequency bias [Hz] |
| `frequency_low` | float | — | Min frequency [Hz] |
| `frequency_high` | float | — | Max frequency [Hz] |
| `frequency_saturation_low` | float | — | Low saturation freq |
| `frequency_saturation_high` | float | — | High saturation freq |
| `amplitude_gain` | float | — | Amplitude × drive |
| `amplitude_bias` | float | — | Amplitude bias |
| `amplitude_low` | float | — | Min amplitude |
| `amplitude_high` | float | — | Max amplitude |
| `amplitude_saturation_low` | float | — | Low saturation amp |
| `amplitude_saturation_high` | float | — | High saturation amp |
| `rate` | float | — | Filter rate |
| `modular_phase` | float | `0` | Modular phase |
| `modular_amplitude` | float | `0` | Modular amplitude |

!!! todo "Saturation field naming"
    The `defaults_from_convention()` method uses `saturation` (single key)
    while `__init__` expects `frequency_saturation_low` and
    `frequency_saturation_high` (split keys). This inconsistency should be
    verified against the ODE function if you use saturation behavior.

### AmphibiousDriveOptions

| Attribute | Type | Description |
|-----------|------|-------------|
| `name` | str | Drive name |
| `initial_value` | float | Initial drive value |
| `kind` | `DriveKind` | Drive type |
| `contacts` | list[tuple[str, str]] | Associated contacts |

### DriveKind

```python
class DriveKind(str, Enum):
    BRAIN_LEFT = 'brain_left'
    BRAIN_RIGHT = 'brain_right'
    SPINE_LEFT = 'spine_left'
    SPINE_RIGHT = 'spine_right'
```

### AmphibiousMotorOptions

Extends `MotorOptions` with:

| Attribute | Type | Description |
|-----------|------|-------------|
| `equation` | str | Motor equation type |
| `transform` | `AmphibiousMotorTransformOptions` | Gain/bias transform |
| `offsets` | `AmphibiousMotorOffsetOptions` | Motor offset parameters |
| `passive` | `AmphibiousPassiveJointOptions` | Passive joint config |

Motor equation types: `phase`, `position_muscle`, `ekeberg_muscle`,
`ekeberg_muscle_explicit`, `passive`, `passive_explicit`

### AmphibiousMotorTransformOptions

| Attribute | Type | Description |
|-----------|------|-------------|
| `gain` | float | Transform gain |
| `bias` | float | Transform bias |

### AmphibiousMotorOffsetOptions

| Attribute | Type | Description |
|-----------|------|-------------|
| `gain` | float | Offset gain |
| `bias` | float | Offset bias |
| `low` | float | Low limit |
| `high` | float | High limit |
| `saturation_low` | float | Low saturation |
| `saturation_high` | float | High saturation |
| `rate` | float | Rate |

### AmphibiousMuscleSetOptions

| Attribute | Type | Description |
|-----------|------|-------------|
| `joint_name` | str | Target joint |
| `osc1` | str | Primary oscillator |
| `osc2` | str | Antagonist oscillator |
| `alpha` | float | Gain |
| `beta` | float | Stiffness gain |
| `gamma` | float | Tonic gain |
| `delta` | float | Damping coefficient |
| `epsilon` | float | Friction coefficient |

## Controller classes

**Source:** `farms_amphibious/control/amphibious.py`

### JointMuscleController

```python
class JointMuscleController(AnimatController):
    def __init__(self, animat_i, animat_options, animat_data, animat_network):
        ...
```

The default controller for amphibious robots. Supports multiple joint equations
simultaneously. Each joint's equation type is determined by its motor's
`equation` field.

### Output methods

```python
def positions(self, iteration, time, timestep) -> dict[str, float]
def velocities(self, iteration, time, timestep) -> dict[str, float]
def torques(self, iteration, time, timestep) -> dict[str, float]
def springrefs(self, iteration, time, timestep) -> dict[str, float]
def springcoefs(self, iteration, time, timestep) -> dict[str, float]
def dampingcoefs(self, iteration, time, timestep) -> dict[str, float]
```

Each method iterates over registered equations for the corresponding
`ControlType` and collects outputs into a dict.

### Equation handlers

| Handler | Equation | Control types |
|---------|----------|----------------|
| `PositionPhaseCy` | `phase` | position |
| `PositionMuscleCy` | `position_muscle` | position |
| `EkebergMuscleCy` | `ekeberg_muscle` | velocity, torque |
| `PassiveJointCy` | `passive` | velocity, torque |

All handlers are Cython-compiled for performance.

## Network classes

**Source:** `farms_amphibious/control/network.py`

### AnimatNetwork (ABC)

```python
class AnimatNetwork(ABC):
    def __init__(self, data, n_iterations):
        ...

    @abstractmethod
    def step(self, iteration, time, timestep, **kwargs):
        """Advance the network by one step."""
```

### NetworkODE

```python
class NetworkODE(AnimatNetwork):
    def __init__(self, data, integrator='dopri5', **kwargs):
        ...
```

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `data` | AnimatData | — | Data container with state array |
| `modulo` | int | 1 | Integrate every N steps |
| `ode` | Callable | `ode_oscillators_sparse` | ODE function |
| `integrator` | str | `dopri5` | scipy ODE integrator name |

#### step()

```python
def step(self, iteration, time, timestep, checks=False, strict=False):
    """Integrate the CPG ODE forward by one timestep."""
```

- Uses `scipy.integrate.ode` with the specified integrator
- Calls `ode_oscillators_sparse(state, iteration, data)` to compute derivatives
- If `modulo > 1`, only integrates every N steps (copies state otherwise)
- On integration failure: warns and resets if `strict=False`, raises
  `IntegrationException` if `strict=True`

## Data classes

**Source:** `farms_amphibious/data/data.py`

### AmphibiousData

Extends `AnimatData` with CPG network-specific data structures. Contains:

- `sensors` — standard `SensorsData`
- `state` — `StateData` with phase/amplitude arrays for all oscillators
- `network` — `NetworkLog` with drive history and connectivity data
