# Contributing: development guide and coding standards

How to set up a development environment, where the code lives, the coding
conventions, how to test a change, and how the documentation is built and
kept up to date.

---

## The repositories

FARMS is made of four Python packages, each in its own repository, checked
out as submodules in the `farms/` folder of `farms_zbot`:

| Package | Purpose |
|---------|---------|
| `farms_core` | Options, data arrays, sensors, extension base classes, I/O |
| `farms_mujoco` | MuJoCo engine: MJCF builder, simulation loop, fluid forces, viewer extensions |
| `farms_sim` | Entry point: command line and simulation setup |
| `farms_amphibious` | CPG networks, controllers, amphibious options and data |

The documentation is a separate repository, `farmsim_docs`.

---

## Development environment

### Installing

The packages need Python 3.11 or newer, a C compiler and Cython. From the
root of `farms_zbot`:

```bash
git submodule update --init --recursive
cd farms
python setup_farms.py
```

`setup_farms.py` installs the dependencies of each package, then installs
the four packages in editable mode, in dependency order (`farms_core`
first: the other packages `cimport` its `.pxd` files). The Docker setup of
[Install and Run](../tutorials/install-and-run.md) does this for you.

Changes to `.py` files take effect immediately. Changes to `.pyx` or
`.pxd` files need a rebuild of the package:

```bash
cd farms/farms_mujoco
python setup.py build_ext --inplace
```

### The DEBUG flag

Each `setup.py` has `DEBUG = False` at the top. With `DEBUG = True`, the
Cython modules are compiled with `boundscheck`, `nonecheck`,
`initializedcheck`, `overflowcheck` and line tracing, which catches index
errors at the cost of speed. Set it back to `False` before benchmarking or
committing.

### Checking the installation

```bash
cd experiments/zbot_bout_glide
python run_sim.py --experiment_config experiment_config.yaml
```

Set `runtime.headless: true` in `simulation_config.yaml` to run without the
viewer.

---

## Where the code lives

```text
farms_core/farms_core/
├── array/            # Typed Cython arrays
├── sensors/          # Sensor arrays (data.py, data_cy.pyx) and column conventions (sensor_convention.pxd)
├── model/            # AnimatOptions (options.py), AnimatData (data.py), AnimatController (control.py), AnimatExtension (extensions.py)
├── experiment/       # ExperimentOptions, ExperimentData
├── simulation/       # SimulationOptions, TaskExtension and the loggers (extensions.py)
├── io/               # SDF, HDF5 and YAML
└── units.py          # SimulationUnitScaling

farms_mujoco/farms_mujoco/
├── simulation/       # Simulation, ExperimentTask, MJCF builder, viewer extensions
├── sensors/          # Sensor copy kernels, CameraRecording
└── swimming/         # Fluid forces (see farms_mujoco.swimming)

farms_amphibious/farms_amphibious/
├── control/          # AmphibiousController, network (NetworkODE), ode.pyx, joint equations, drives
├── data/             # AmphibiousData, network data
└── model/            # AmphibiousOptions, AmphibiousConvention

farms_sim/farms_sim/
├── farmsim.py        # main(), profile_simulation()
├── simulation.py     # setup_from_clargs(), simulation_setup(), run_simulation()
└── utils/parse_args.py
```

---

## Coding conventions

### Python

- PEP 8, type hints on public functions, a docstring on every public class
  and function (the [API reference](../reference/api/farms_core/index.md)
  is generated from them).
- Configuration options go in the `Options` class of their file, with a
  `ChildDoc` entry in its `doc()` method: the
  [configuration reference](../reference/env/configuration-reference.md) is
  generated from it.
- YAML keys are `snake_case`, and values in SI units unless stated.

### Cython

- Declare C types for loop variables and arrays, and use typed
  memoryviews (`double[::1]`) in hot paths.
- Mark functions that do not touch Python objects `noexcept nogil`.
- Do not raise exceptions inside `cdef` functions: validate the inputs in
  Python before the loop.
- Add a `.pxd` file for modules that other Cython modules `cimport`.
- Do not allocate in the simulation loop: allocate when the object is
  created, as `SwimmingHandler` does.

### Commit messages

Use [Conventional Commits](https://www.conventionalcommits.org/):
`feat(swimming): ...`, `fix(control): ...`, `perf(cob): ...`,
`docs(tutorial): ...`.

---

## Extending FARMS

| To add | See |
|--------|-----|
| A robot model | [The Zbot Model](../tutorials/zbot-model.md) and [Configure an Experiment YAML](configure-yaml.md) |
| A controller | [Write a Custom Controller](../tutorials/custom-controller.md) and [Write a Controller](write-controller.md) |
| A force or other per-step behaviour | [Write an AnimatExtension](write-extension.md) |
| A fluid model feature | [Hydrodynamics Internals](../internals/hydrodynamics-internals.md#how-to-extend) |
| A CPG topology | [Configure CPG Network Parameters](configure-cpg-network.md) |
| A logged quantity | [Save, Load, and Inspect Data](save-load-data.md) |

When several extensions apply forces, accumulate into
`physics.data.xfrc_applied` (`+=`) rather than overwrite it, unless the
extension is the only one writing to those bodies (`SwimmingExtension`
writes the fluid wrench of its links).

---

## Testing

### Unit tests

`farms_core/tests` and `farms_mujoco/tests` contain pytest tests (for
example the centre of buoyancy kernels, lookup tables and ellipsoid model
in `farms_mujoco/tests`):

```bash
cd farms/farms_mujoco
python -m pytest tests
```

If a ROS installation sets `PYTHONPATH`, run
`env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests`.

### Simulation check

Run a reference experiment headless and check the output for NaN values:

```python
import numpy as np
from farms_core.experiment.data import ExperimentData
from farms_core.sensors.sensor_convention import sc

data = ExperimentData.from_file('Output/simulation.hdf5')
joints = data.animats[0].sensors.joints
positions = np.asarray(joints.array)[:, :, sc.joint_position]
assert np.all(np.isfinite(positions)), 'Non-finite joint positions'
print(f'Max joint position: {np.max(np.abs(positions)):.4f} rad')
```

`farms_mujoco/benchmarks/bench_fluid.py` measures the cost of the fluid
forces and `inspect_buoyancy.py` the buoyancy budget of a model.

---

## Documentation

The documentation is a MkDocs Material site in `farmsim_docs/reference`.

### What is generated

These pages are generated from the installed code at every build (by
`tools/gen_pages.py`, a `mkdocs-gen-files` script) and must not be edited
by hand:

| Page | Source |
|------|--------|
| `reference/api/**` | The docstrings of every public module (`mkdocstrings`) |
| [Configuration Parameter Reference](../reference/env/configuration-reference.md) | The `doc()` methods of the options classes, and `FluidOptions` |
| [CLI Reference](../reference/env/cli.md) | The `farms_sim` argument parser |

Hand-written pages can include generated signatures with a
`::: module.Class` block.

### The drift guard

`tools/check_docs.py` checks the hand-written pages against the installed
code, and fails on:

1. a dotted reference (`farms_mujoco.swimming.extension.SwimmingExtension`)
   that does not import;
2. a source path (`farms_mujoco/swimming/cob.pyx`) that does not exist;
3. a command line option, on a `run_sim.py` or `farmsim` command line,
   that the parser does not have;
4. an unknown key in a YAML example (a block whose first line is
   `# check-docs: skip` is not checked);
5. an em dash.

### Which code is documented

`reference/farms-packages.yaml` lists the four FARMS packages with the
public repository and the ref (branch, tag or commit) to document:

```yaml
# check-docs: skip
packages:
  - name: farms_core
    repo: https://github.com/podalanga/farms_core.git
    ref: fluid-fast
  # ... farms_mujoco, farms_sim, farms_amphibious
```

`tools/install_farms.py` clones them at these refs and installs them in
dependency order. The documentation needs nothing else: no experiment
files or private repository.

### Building locally

In an environment where the FARMS packages are installed (for example
the `farms_zbot` environment), or after installing them with
`make install`:

```bash
cd farmsim_docs/reference
pip install -r requirements.txt
make install  # Optional: clone and install the packages of farms-packages.yaml
make check    # Drift guard
make build    # Drift guard, then mkdocs build --strict
make serve    # Live preview on http://127.0.0.1:8000
```

### Automatic updates

The workflow `.github/workflows/deploy-docs.yml` of `farmsim_docs`
installs the packages of `farms-packages.yaml`, runs the drift guard and
`mkdocs build --strict`, and deploys to GitHub Pages. It runs:

- on a push to `main` (and checks pull requests without deploying);
- every night, so that new commits on the documented branches are picked
  up;
- manually (`workflow_dispatch`), optionally with other refs
  (`farms_refs: farms_mujoco=my-branch`);
- when a FARMS repository sends a `code-updated` event.

To send that event on every push of a FARMS repository, copy
`templates/notify-docs.yml` to its `.github/workflows/`, set its branch to
the one documented, and add a repository secret `DOCS_DISPATCH_TOKEN`: a
fine-grained personal access token with "Contents: read and write" access
to `farmsim_docs`.

When a code change renames an option, a class or a flag, the generated
pages follow automatically, and the drift guard fails the build until the
hand-written pages are updated. To document another version of a package,
change its ref in `farms-packages.yaml`.

### Writing pages

- Specify the language of code blocks (`python`, `yaml`, `bash`).
- Use admonitions (`!!! note`, `!!! warning`) for callouts, and a
  `## See also` section at the end.
- Do not use em dashes.
- Add new pages to the `nav` of `mkdocs.yml`. When a page is moved or
  removed, add a redirect in the `redirects` plugin.

```yaml
# check-docs: skip
nav:
  - How-to Guides:
    - My New Page: how-to/my-new-page.md
```

---

## Checklist

- [ ] Docstrings on new public classes and functions, `ChildDoc` entries for new options
- [ ] Cython hot paths typed and allocation free, `DEBUG = False`
- [ ] Unit tests pass, a reference simulation runs without NaN
- [ ] `make build` passes in `farmsim_docs`
- [ ] Commit messages follow Conventional Commits

## See also

- [System Architecture](../explanation/architecture.md)
- [Extension and Controller Design](../explanation/extension-design.md)
- [Configuration Parameter Reference](../reference/env/configuration-reference.md)
