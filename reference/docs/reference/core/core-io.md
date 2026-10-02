# farms_core.io

File I/O: SDF models, HDF5 data and YAML options.

## SDF models (`farms_core.io.sdf`)

`ModelSDF.read(filename)` parses an SDF file into a list of models (one
per `<model>`), with their links (inertials, visuals, collisions) and
joints. The MJCF builder of `farms_mujoco` uses it to build the MuJoCo
model.

```python
from farms_core.io.sdf import ModelSDF

model = ModelSDF.read('examples/amphibot/models/amphibot.sdf')[0]
print(model.name, [link.name for link in model.links])
print([joint.name for joint in model.joints], model.mass())
```

The module also has helpers for poses (`get_pose_from_xml()`,
`get_homogenous_matrix_from_pose()`), inertias
(`get_inertia_tensor_from_vector()`), URDF conversion
(`ModelSDF.from_urdf()`) and fixed joint merging (`merge_fixed_joints()`).
See the [generated API](../api/farms_core/io/sdf.md).

## HDF5 (`farms_core.io.hdf5`)

Nested dictionaries are stored as HDF5 groups; lists become groups whose
names start with `FARMSLIST`. `ExperimentData.to_file()` and `from_file()`
use these functions.

```python
from farms_core.io.hdf5 import dict_to_hdf5, hdf5_to_dict

dict_to_hdf5('results.h5', {'time': [0.0, 0.1], 'joints': {'knee': [0.5, 0.6]}})
print(hdf5_to_dict('results.h5')['joints']['knee'])  # array([0.5, 0.6])
```

::: farms_core.io.hdf5
    options:
      show_root_heading: false
      heading_level: 3
      members: [dict_to_hdf5, hdf5_to_dict, hdf5_keys, hdf5_get, hdf5_open]

## YAML (`farms_core.io.yaml`)

`yaml2pyobject()` loads a YAML file into plain Python objects,
`pyobject2yaml(filename, pyobject)` writes them. `Options.load()` and
`Options.save()` use them.

::: farms_core.io.yaml
    options:
      show_root_heading: false
      heading_level: 3
      members: [yaml2pyobject, pyobject2yaml, read_yaml, write_yaml]

## See Also

- [Options classes](core-options.md)
- [Data Flow and Data Model](../../explanation/data-flow.md)
