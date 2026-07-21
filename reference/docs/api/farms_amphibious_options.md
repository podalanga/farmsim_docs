# `farms_amphibious.model.options`

Options classes extending `farms_core` for amphibious animat configuration.

This module provides configuration classes for the FARMS amphibious models. It extends `farms_core` by adding options specific to segmented amphibious bodies and multi-legged morphologies. Additionally, it defines the parameters for the central pattern generator (CPG) network, sensor topologies, and provides a pure kinematics controller mode for replay experiments.

---

## `DriveKind`

Enum defining the functional location and side of neural descending drives.

```python
class DriveKind(str, Enum):
```

| Name | Description |
|------|-------------|
| `BRAIN_LEFT` | Higher-level descending command from the left brain hemisphere. |
| `BRAIN_RIGHT` | Higher-level descending command from the right brain hemisphere. |
| `SPINE_LEFT` | Local spinal cord drive acting on the left side. |
| `SPINE_RIGHT` | Local spinal cord drive acting on the right side. |

---

## `AmphibiousOptions`

Inherits from `AnimatOptions` — see [farms_core_options](farms_core_options.md). Core configuration for an amphibious animat, encompassing morphology, spawning, control, and physics extensions.

```python
def __init__(self, sdf: str, **kwargs):
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `sdf` | `str` | - | Path to the SDF model file. |
| `spawn` | `dict` | - | Dictionary parsed into `SpawnOptions`. |
| `morphology` | `dict` | - | Dictionary parsed into `AmphibiousMorphologyOptions`. |
| `control` | `dict` | - | Dictionary parsed into `AmphibiousControlOptions` or `KinematicsControlOptions`. |
| `extensions` | `list` | - | List of `AnimatExtensionOptions` definitions. |
| `show_xfrc` | `bool` | - | Toggles visualization of external forces. |
| `scale_xfrc` | `float` | - | Scaling factor for external force rendering. |
| `mujoco` | `dict` | - | Dictionary of Mujoco-specific solver and physics options. |

### `default`
```python
@classmethod
def default(cls):
```
Instantiates a default set of options using an empty configuration dictionary.

### `from_options`
```python
@classmethod
def from_options(cls, kwargs=None):
```
Instantiates the class from a configuration dictionary. Routes control generation to either `KinematicsControlOptions` or `AmphibiousControlOptions` based on the presence of the `kinematics_file` key.

### `state_init`
```python
def state_init(self):
```
Computes and returns the initial state vector. Concatenates oscillator phases (rad), oscillator amplitudes, and active joint positions (rad).

---

## `AmphibiousExperimentOptions`

Inherits from `ExperimentOptions`. Configuration for simulation experiments involving amphibious animats.

```python
def __init__(
        self,
        simulation: str | SimulationOptions,
        animats: list[str] | list[AmphibiousOptions],
        arenas: list[str] | list[ArenaOptions]
):
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `simulation` | `str` \| `SimulationOptions` | - | Simulation runtime configuration. |
| `animats` | `list[str]` \| `list[AmphibiousOptions]` | - | List of animats to simulate. |
| `arenas` | `list[str]` \| `list[ArenaOptions]` | - | List of arenas to use in the experiment. |

---

## `AmphibiousControlOptions`

Inherits from `ControlOptions`. Configures controllers, sensors, and the oscillator network for the amphibious animat.

```python
def __init__(self, **kwargs):
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `controller_loader` | `str` | `'farms_amphibious.control.amphibious.AmphibiousController'` | Path to the controller implementation class. |
| `sensors` | `dict` | - | Sensors configuration, parsed into `AmphibiousSensorsOptions`. |
| `motors` | `list` | - | List of motor configurations (`AmphibiousMotorOptions`). |
| `network` | `dict` | `None` | Network configuration, parsed into `AmphibiousNetworkOptions`. |
| `muscles` | `list` | `[]` | List of muscle sets (`AmphibiousMuscleSetOptions`). |
| `hill_muscles` | `list` | `[]` | List of Hill muscle parameters. |
| `adhesions` | `list` | `[]` | Adhesion forces and actuators configuration. |
| `visuals` | `list` | `[]` | Visual markers for rendering tracking. |

---

## `AmphibiousNetworkOptions`

Configures the central pattern generator (CPG) network, including oscillators, drives, and synaptic connectivity.

```python
def __init__(self, **kwargs):
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `drives` | `list` | `[]` | Drive definitions list. |
| `oscillators` | `list` | `[]` | Oscillator nodes configuration. |
| `single_osc_body` | `bool` | `False` | Uses a single oscillator per body segment instead of a left/right pair. |
| `single_osc_legs` | `bool` | `False` | Uses a single oscillator per leg instead of an antagonistic pair. |
| `osc2osc` | `dict` | `None` | Connections between oscillators (coupling weights and phases). |
| `joint2osc` | `dict` | `None` | Sensory feedback connections from joints to oscillators (e.g., stretch reflexes). |
| `contact2osc` | `dict` | `None` | Sensory feedback from contact sensors to oscillators. |
| `xfrc2osc` | `dict` | `None` | External force feedback from links to oscillators. |
| `drive2osc` | `list` | `None` | Mapping of drives to oscillators. |
| `drive2joint` | `list` | `None` | Mapping of drives to joints. |

!!! note
    The constructor relies heavily on `from_options` and `defaults_from_convention` to extract and populate network parameters directly from kwargs.

### Network Parameters
Common variables extracted from dictionaries and mapped to the network structure include:
- **Oscillator Properties**: Gain, bias, low, high, saturation bounds for both frequency (rad/s) and amplitude.
- **Coupling Weights**: `weight_osc_body_side`, `weight_osc_body_down`, `weight_osc_legs_internal`, `weight_osc_legs_opposite`, `weight_osc_legs_following`, `weight_osc_legs2body`, `weight_osc_body2legs`.
- **Sensory Feedback**: Joint stretch reflexes (`weight_sens_stretch_freq_up/down/same`), ground contact (`weight_sens_contact_body_freq_up`, `weight_sens_contact_intralimb`, etc.), external force (`xfrc`), and Tegotae phase feedback.
- **Drives Initialization**: The `drives_init` array initializes baseline drive values.

---

## `AmphibiousMorphologyOptions`

Inherits from `MorphologyOptions`. Configures segmented bodies and legs for amphibious entities.

```python
def __init__(self, **kwargs):
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `links` | `list` | - | Body and leg link definitions (`AmphibiousLinkOptions`). |
| `self_collisions` | `list` | - | Allowed internal collision pairs. |
| `joints` | `list` | - | Articulations between links. |
| `n_joints_body` | `int` | - | Number of spinal joints. |
| `n_dof_legs` | `int` | - | Degrees of freedom per leg. |
| `n_legs` | `int` | - | Total number of legs. |
| `n_joints_passive`| `int` | - | Number of unactuated passive joints. |

---

## `KinematicsControlOptions`

Inherits from `ControlOptions`. Configures control for pure kinematics playback (replay mode) bypassing the CPG network simulation.

```python
def __init__(self, **kwargs):
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `kinematics_file` | `str` | `''` | Path to the target data file to replay. |
| `kinematics_v_file`| `str` | `None` | Path to the optional velocity data file. |
| `kinematics_sampling`| `float`| `0` | Sampling interval (s) for data playback. |
| `kinematics_indices`| `list[int]`| `None` | List of selected time indices from the data file. |
| `kinematics_time_index`| `int`| `None` | Target time index column in the data. |
| `kinematics_invert`| `bool` | `False` | Negates the loaded kinematics values. |
| `kinematics_degrees`| `bool` | `False` | Assumes loaded data is in degrees instead of radians. |
| `kinematics_start`| `int` | `0` | Start frame index. |
| `kinematics_end` | `int` | `0` | End frame index. |

---

## Usage Example

```python
from farms_amphibious.model.options import AmphibiousOptions

# Initialize amphibious configuration from a dictionary
options = AmphibiousOptions.from_options({
    'sdf_path': 'model.sdf',
    'morphology': {
        'n_joints_body': 10,
        'n_legs': 4,
        'n_dof_legs': 3,
        'mass_multiplier': 1.0,
        'feet_friction': 1.0
    },
    'control': {
        'network': {
            'drives_init': [1.5, 1.5],
            'single_osc_body': False
        }
    }
})
```

---

**See also:**
- [farms_core_options](farms_core_options.md)
- [farms_amphibious_controller](farms_amphibious_controller.md)
- [configuration](../configuration.md)
- Source: `farms_amphibious/model/options.py`
