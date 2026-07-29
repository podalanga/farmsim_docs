# farms_amphibious.data

CPG network state containers and drive arrays for time-series data logging.

**Overview**
Data models used to maintain network states and high-level drives throughout the simulation. They wrap optimized Cython data structures to enable efficient access and iteration.

## `OscillatorNetworkState`

```python
class OscillatorNetworkState(OscillatorNetworkStateCy)
```

Pre-allocated state array storing phases and amplitudes of the CPG network. Dimensions are `[n_iterations × (2·N_osc + N_joints)]`.

### `from_initial_state`

```python
@classmethod
def from_initial_state(cls, initial_state: np.ndarray, n_iterations: int, n_oscillators: int) -> "OscillatorNetworkState"
```

Factory method.

| Name | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `initial_state` | `np.ndarray` | | Initial phase and amplitude vector |
| `n_iterations` | `int` | | Total simulation steps to pre-allocate |
| `n_oscillators` | `int` | | Total number of neural oscillators |

Provides helper methods `plot_phases`, `plot_amplitudes`, and `plot_neural_activity_normalised` to visualize network activity.

---

## `DriveArray`

```python
class DriveArray(DriveArrayCy)
```

Maintains descending drives organized by brain/spine and left/right locations.

### `from_animat_options`

```python
@classmethod
def from_animat_options(cls, animat_options: AmphibiousOptions, n_iterations: int) -> "DriveArray"
```

Factory method based on options.

| Name | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `animat_options` | `AmphibiousOptions` | | Simulation settings |
| `n_iterations` | `int` | | Total length of array in steps |

**Properties**
- `spine_indices`: Indices representing spinal drive values.
- `brain_indices`: Indices representing brain drive values.

Provides helper methods `plot_brain`, `plot_spine`, and `plot_all`.

---

## `DriveDependentArray`

```python
class DriveDependentArray(DriveDependentArrayCy)
```

A mapping function that converts a drive value into parameter adjustments using a piecewise-linear relationship.

### `from_vectors`

```python
@classmethod
def from_vectors(cls, gain: float, bias: float, low: float, high: float, saturation_low: float, saturation_high: float) -> "DriveDependentArray"
```

Instantiates the 6-parameter drive function.

| Name | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `gain` | `float` | | Multiplier applied to drive |
| `bias` | `float` | | Baseline offset value |
| `low` | `float` | | Drive threshold to apply saturation low |
| `high` | `float` | | Drive threshold to apply saturation high |
| `saturation_low` | `float` | | Minimum saturated value |
| `saturation_high` | `float` | | Maximum saturated value |

---

## `Oscillators`

```python
class Oscillators(OscillatorsCy)
```

Collection array for all oscillator parameters.

### `__init__`

```python
def __init__(self, names: list[str], drive2osc_map: np.ndarray, intrinsic_frequencies: np.ndarray, nominal_amplitudes: np.ndarray, rates: np.ndarray, modular_phases: np.ndarray, modular_amplitudes: np.ndarray) -> None
```

| Name | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `names` | `list[str]` | | String identifiers for oscillators |
| `drive2osc_map` | `np.ndarray` | | Mapping array linking drives to oscillators |
| `intrinsic_frequencies` | `np.ndarray` | | Frequencies [rad/s] |
| `nominal_amplitudes` | `np.ndarray` | | Desired nominal amplitudes |
| `rates` | `np.ndarray` | | Convergence rates |
| `modular_phases` | `np.ndarray` | | Modular phase values |
| `modular_amplitudes` | `np.ndarray` | | Modular amplitude values |

Use `from_options(cls, network)` to initialize from the options tree.

---

## Network Connection Types

The neural simulation supports various sensorimotor mapping connections.

| Connection | Description |
| ---------- | ----------- |
| `OSC2OSC` | Excitatory or inhibitory connections between oscillators |
| `DRIVE2OSC` | Descending drive modulation of oscillators |
| `STRETCH2FREQTEGOTAE` | Tegotae-based local frequency stretch reflex |
| `STRETCH2AMPTEGOTAE` | Tegotae-based local amplitude stretch reflex |
| `POS2FREQ`, `VEL2FREQ`, `TOR2FREQ` | Kinematic and kinetic feedback to frequency |
| `POS2AMP`, `VEL2AMP`, `TOR2AMP` | Kinematic and kinetic feedback to amplitude |
| `STRETCH2FREQ`, `STRETCH2AMP` | Generic stretch feedback |
| `REACTION2FREQ`, `REACTION2AMP` | General reaction force feedback |
| `REACTION2FREQTEGOTAE` | Tegotae reaction force to frequency |
| `FRICTION2FREQ`, `FRICTION2AMP` | Foot friction feedback |
| `LATERAL2FREQ`, `LATERAL2AMP` | Lateral force feedback |

---

## `AmphibiousData`

```python
class AmphibiousData(AmphibiousDataCy, AnimatData)
```

Top-level experiment data container that aggregates the full state of the animat.

**Attributes**
- `sensors`: `SensorsData` container recording proprioceptive data.
- `network`: `OscillatorNetworkState` containing the integrated CPG parameters.

Use `from_options` to initialize empty containers for simulation, or `from_file` to load saved logging files.

## See Also

- [CPG Oscillator Data](cpg_oscillators.md) — Oscillator data structures
- [CPG Network API](farms_amphibious_network.md) — Network state management
- [Sensor Data Arrays](farms_core_sensors.md) — Pre-allocated telemetry arrays
