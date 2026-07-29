# farms_core.io

I/O utilities for SDF model parsing, HDF5 data serialisation, and YAML configuration loading.

## Overview

The `farms_core.io` module handles all file-based operations in the FARMS framework. It parses rigid body tree structures from SDF files, facilitates the saving and loading of massive simulation data structures in HDF5 format, and parses user configurations via YAML.

---

## SDF Parser (`farms_core.io.sdf`)

The SDF sub-module extracts robot kinematics and dynamics from XML files. It yields link geometries, inertial properties, and joint properties for ingestion into the simulation.

```python
def get_floats_from_text(text: str, split: str = ' ') -> list[float]:
    """Parse a list of floats from a string."""
```

```python
def get_pose_from_xml(data: xml.etree.ElementTree.Element) -> list[float]:
    """Extract pose (position and orientation) from an XML node."""
```

```python
def get_inertia_tensor_from_vector(inertia_vector: list) -> np.ndarray:
    """Get the inertia tensor from the inertia vector of six elements."""
```

```python
def get_inertia_vector_from_tensor(inertia_tensor: np.ndarray) -> list:
    """Get the inertia vector of six elements from the inertia tensor."""
```

```python
def get_homogenous_matrix_from_pose(pose: list[float]) -> np.ndarray:
    """Construct a 4x4 homogenous matrix from a 6D pose."""
```

```python
def get_pose_from_homogenous_matrix(homogenous_matrix: np.ndarray) -> list[float]:
    """Extract a 6D pose from a 4x4 homogenous matrix."""
```

---

## HDF5 Serialisation (`farms_core.io.hdf5`)

HDF5 is used to save telemetry, controller states, and simulation states efficiently. Nested dictionaries are seamlessly converted into HDF5 group structures and vice-versa.

```python
def hdf5_open(filename: str, mode: str = 'w', max_attempts: int = 10, attempt_delay: float = 0.1) -> h5py.File:
    """Open HDF5 file with delayed attempts."""
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `filename` | str | - | The path to the HDF5 file |
| `mode` | str | `'w'` | Access mode |
| `max_attempts` | int | `10` | Maximum number of retry attempts |
| `attempt_delay` | float | `0.1` | Time to wait between attempts in seconds |

```python
def dict_to_hdf5(filename: str, data: dict, mode: str = 'w', **kwargs):
    """Save a Python dictionary to an HDF5 file."""
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `filename` | str | - | The destination path |
| `data` | dict | - | The nested dictionary to save |
| `mode` | str | `'w'` | File open mode |

```python
def hdf5_to_dict(filename: str, **kwargs) -> dict:
    """Load an HDF5 file into a Python dictionary."""
```

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `filename` | str | - | The path of the file to load |

```python
def hdf5_keys(filename: str, **kwargs) -> list[str]:
    """Retrieve the top-level keys/groups from an HDF5 file."""
```

```python
def hdf5_get(filename: str, key: list[str], **kwargs) -> dict:
    """Load a specific sub-group from an HDF5 file given a path of keys."""
```

---

## YAML Configuration (`farms_core.io.yaml`)

Utilities for loading model and controller option files.

```python
def read_yaml(file_path: str) -> Any:
    """Read the yaml data from file."""
```

```python
def write_yaml(data: dict, file_path: str):
    """Method that dumps the data to yaml file."""
```

```python
def pyobject2yaml(filename: str, pyobject: Any, mode: str = 'w+'):
    """Serialise a python object directly to YAML."""
```

```python
def yaml2pyobject(filename: str) -> Any:
    """Deserialise a python object from YAML."""
```

---

## Usage Example

```python
from farms_core.io.yaml import yaml2pyobject
from farms_core.io.hdf5 import dict_to_hdf5, hdf5_to_dict

# Load configuration from YAML
options = yaml2pyobject("my_robot.yaml")

# Create some telemetry data
telemetry = {
    "time": [0.0, 0.1, 0.2],
    "joints": {
        "knee": [0.5, 0.6, 0.7]
    }
}

# Save telemetry to HDF5
dict_to_hdf5("results.h5", telemetry)

# Load data back
loaded_data = hdf5_to_dict("results.h5")
print(loaded_data["joints"]["knee"])
```

## See Also

- [Configuration Classes](./farms_core_options.md) — YAML dataclass schemas
- [Simulation Walkthrough](../walkthrough.md) — End-to-end lifecycle
