# Contributing — Development guide and coding standards

This guide covers everything you need to know to contribute to the FARMS framework — from setting up your development environment and understanding the build system, to coding conventions, adding new features, and submitting your changes.

!!! note "Source Files"
    - `farms_core/setup.py` — Cython build configuration
    - `farms_mujoco/setup.py` — Cython build configuration
    - `farms_amphibious/setup.py` — Cython build configuration
    - `farms_sim/setup.py` — Python package setup

---

## Before You Start

FARMS is a research framework developed at the [BioRob Lab, EPFL](https://www.epfl.ch/labs/biorob/). It is composed of four separate Git repositories, each an independent Python package:

| Repository | Purpose |
|------------|---------|
| `farms_core` | Data structures, base interfaces, I/O utilities |
| `farms_mujoco` | MuJoCo physics backend and simulation loop |
| `farms_sim` | CLI entry point and simulation orchestrator |
| `farms_amphibious` | CPG networks, controllers, and domain-specific physics |

All four repositories live as submodules inside the `farms/` directory of the `farms_zbot` project.

---

## Part 1 — Development Environment Setup

### Requirements

You must work **inside the Docker container** to have all dependencies available. See the [Installation Guide](./installation.md) for setup. Inside the container, the following are pre-installed:

- Python 3.12+
- Cython (with a GCC/Clang C compiler)
- NumPy, SciPy, h5py, PyYAML, matplotlib, dm-control, MuJoCo

### Installing in Development (Editable) Mode

The packages must be installed in dependency order. `farms_amphibious` depends on `farms_core`, so `farms_core` must be installed first.

```bash
# Inside the container, from /app
pip install -e ./farms/farms_core
pip install -e ./farms/farms_mujoco
pip install -e ./farms/farms_sim
pip install -e ./farms/farms_amphibious
```

The `-e` (editable) flag installs the package in-place. Changes to **pure Python** (`.py`) files take effect immediately without reinstalling. Changes to **Cython** (`.pyx`) files require a rebuild (see below).

### Rebuilding Cython Extensions

FARMS uses Cython for performance-critical inner loops. After editing any `.pyx` file, you must recompile the extension:

```bash
# Rebuild a single package
pip install -e ./farms/farms_core --no-build-isolation

# Or rebuild all packages
pip install -e ./farms/farms_core --no-build-isolation && \
pip install -e ./farms/farms_amphibious --no-build-isolation
```

!!! tip "Fast Iteration on Cython Code"
    Set `DEBUG = True` at the top of the relevant `setup.py` to enable Cython's `boundscheck`, `nonecheck`, and `wraparound` guards. This catches array index errors and None-dereferences at the cost of performance. Switch back to `DEBUG = False` before benchmarking.

### Verifying Your Installation

Run a reference experiment to confirm everything works end-to-end:

```bash
cd /app/experiments/zbot_swimming
farmsim --experiment_config experiment_config.yaml --headless
```

If the simulation runs to completion without error, your environment is correctly set up.

---

## Part 2 — Repository Structure

Understanding where code lives is essential before making any changes.

### `farms_core`

```
farms_core/
├── array/          # Low-level C-typed Cython array wrappers (NDARRAY types)
├── sensors/        # Sensor data containers (positions, velocities, forces)
├── model/
│   ├── control.py  # AnimatController, TaskExtension, ControlType — base interfaces
│   ├── data.py     # AnimatData, ExperimentData — telemetry containers
│   └── options.py  # AnimatOptions, SimulationOptions, etc. — YAML schemas
├── simulation/
│   ├── options.py  # RuntimeSimulationOptions, PhysicsSimulationOptions
│   └── extensions.py # ExperimentLogger, ExperimentOptionsLogger
├── io/
│   ├── hdf5.py     # HDF5 read/write utilities
│   └── yaml.py     # YAML read/write utilities
└── utils/          # Geometry, rotation, and general math utilities
```

### `farms_amphibious`

```
farms_amphibious/
├── control/
│   ├── amphibious.py          # AmphibiousController — main CPG controller
│   ├── ode.pyx                # CPG phase/amplitude ODEs (Cython, hot path)
│   ├── ekeberg.pyx            # Ekeberg muscle model (Cython, hot path)
│   ├── joints_control_cy.pyx  # Joint actuation logic (Cython)
│   ├── network.py             # NetworkODE — scipy ODE integrator wrapper
│   └── drive.py               # Drive signal computation
├── data/
│   └── data.pyx               # AmphibiousData telemetry (Cython arrays)
└── model/
    └── options.py             # AmphibiousOptions, OscillatorOptions, etc.
```

### `farms_mujoco`

```
farms_mujoco/
├── simulation/
│   ├── simulation.py   # MuJoCoSimulation — main physics class
│   ├── task.py         # ExperimentTask — dm_control Task implementation
│   └── extensions.py   # MjcfSaver, CameraFollower, etc.
└── swimming/
    ├── drag.pyx         # Hydrodynamic drag/buoyancy (Cython, hot path)
    └── extension.py     # SwimmingExtension — injects hydrodynamic forces
```

---

## Part 3 — Coding Conventions

### Python Style

- Follow **PEP 8** for all Python files.
- Use **type hints** for all public function signatures.
- All public classes and functions must have **docstrings**.
- Maximum line length: **99 characters**.

```python
# Good
def compute_drag_force(velocity: np.ndarray, coefficients: list[float]) -> np.ndarray:
    """Compute translational drag force given a relative velocity vector.

    Args:
        velocity: Relative velocity in the URDF frame [x, y, z], shape (3,).
        coefficients: Drag coefficients [cx, cy, cz].

    Returns:
        Drag force vector in the URDF frame, shape (3,).
    """
    ...

# Bad — no type hints, no docstring
def drag(v, c):
    ...
```

### Cython (`.pyx`) Style

Cython files are used for the innermost loops that run thousands of times per simulation second. Follow these rules:

- **Always declare C types** for all loop variables and arrays. Untyped variables default to Python objects and are slow.
- **Use memoryviews** (`DTYPE_t[::1]`) instead of NumPy arrays as function arguments in hot paths.
- **Declare `nogil`** for functions that do not touch Python objects — this is mandatory for any future parallelism.
- **Never** use Python exceptions (`raise`) inside `cdef` functions — use C-style return codes or pre-validate in Python.

```cython
# Good — typed, uses memoryviews
cdef void step_oscillators(
    double[::1] phases,
    double[::1] frequencies,
    double dt,
    int n_osc,
) nogil:
    cdef int i
    for i in range(n_osc):
        phases[i] += frequencies[i] * dt

# Bad — untyped, slow
def step_oscillators(phases, frequencies, dt):
    for i in range(len(phases)):
        phases[i] += frequencies[i] * dt
```

- Add a `.pxd` declaration file for any `.pyx` module that will be `cimport`ed by other Cython modules. Without `.pxd` files, cross-module C-level calls fall back to Python-level calls and lose all performance gains.

### YAML Configuration

- All new configurable parameters must be added to the relevant `Options` class in Python **and** documented in the [Configuration Reference](./configuration.md).
- YAML keys must use `snake_case`.
- Units must be SI. Document units explicitly in the `Options` class docstring.

### Commit Messages

Follow the [Conventional Commits](https://www.conventionalcommits.org/) specification:

```
<type>(<scope>): <short description>

[optional body]
```

| Type | When to use |
|------|-------------|
| `feat` | New feature or behaviour |
| `fix` | Bug fix |
| `perf` | Performance improvement (e.g., Cython optimization) |
| `refactor` | Code restructuring with no behaviour change |
| `docs` | Documentation only |
| `test` | Adding or updating tests |
| `chore` | Build scripts, CI changes, dependency updates |

**Examples:**

```
feat(control): add Tegotae sensory feedback to CPG phase ODE
fix(swimming): prevent negative buoyancy for fully submerged links
perf(ode): replace Python loop with Cython nogil inner loop in phase stepping
docs(tutorial): add section on custom controller creation
```

---

## Part 4 — Extension Recipes

FARMS is built around a plugin architecture. You rarely need to modify core files — instead, you extend the framework through well-defined interfaces.

### Recipe A — Adding a New Robot Model

To simulate a new robot, you only need a model file and YAML configuration. No Python code is required for the physics.

**Step 1**: Create your robot model as an SDF or MJCF file:

```xml
<!-- models/my_robot/sdf/my_robot.sdf -->
<sdf version="1.6">
  <model name="my_robot">
    <link name="base">
      <inertial>...</inertial>
      <visual>...</visual>
      <collision>...</collision>
    </link>
    <link name="segment_1">...</link>
    <joint name="joint_1" type="revolute">
      <parent>base</parent>
      <child>segment_1</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>
  </model>
</sdf>
```

!!! important
    Joint and link names in the SDF **must exactly match** the names used in `animat_config.yaml`. A mismatch will cause a `KeyError` at simulation startup.

**Step 2**: Create `animat_config.yaml` using the Zbot config as a template. Key sections to modify:

```yaml
sdf: ../../models/my_robot/sdf/my_robot.sdf
morphology:
  links:
    - name: base
      collisions: true
      fluid_interaction: false      # Set true if submerged
      density: 1200.0              # kg/m³
      drag_coefficients:
        - [0, 0, 0]                # Translational drag [x, y, z]
        - [0, 0, 0]                # Rotational drag
  joints:
    - name: joint_1
      initial: [0, 0]              # [position (rad), velocity (rad/s)]
      stiffness: 0
      damping: 0.01
control:
  controller_loader: my_package.MyController
  motors:
    - joint_name: joint_1
      control_types: [position]
      limits_torque: [-5.0, 5.0]
      gains: [2.0, 0.005, 0]       # [Kp, Kd, Ki]
```

### Recipe B — Adding a New Controller

Inherit from `AnimatController` and implement the actuation methods. See the [Zbot tutorial](./zbot/index.md) for a full worked example.

```python
from farms_core.model.control import AnimatController, ControlType
import numpy as np

class MyController(AnimatController):

    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
        """Required factory method — called by farms_sim at startup."""
        return cls(
            animat_i=animat_i,
            joints_names=animat_data.joints.names,
            muscles_names=(),
            max_torques=animat_data.joints.max_torques,
        )

    def positions(self, iteration: int, time: float, timestep: float) -> dict:
        """Return {joint_name: target_position_rad} for position-controlled joints."""
        return {
            joint: 0.3 * np.sin(2 * np.pi * time)
            for joint in self.joints_names[ControlType.POSITION]
        }

    def torques(self, iteration: int, time: float, timestep: float) -> dict:
        """Return {joint_name: torque_Nm} for torque-controlled joints."""
        return {}
```

Point `controller_loader` in `animat_config.yaml` to your class:

```yaml
control:
  controller_loader: my_package.my_module.MyController
```

### Recipe C — Adding a New Physics Extension (Force Model)

If you want to inject custom forces (aerodynamics, tethers, magnetic fields), subclass `TaskExtension`.

```python
from farms_core.simulation.extensions import TaskExtension
import numpy as np

class WindForceExtension(TaskExtension):
    """Applies a constant wind force to all body links of animat 0."""

    def __init__(self, wind_vector: list[float], link_indices: list[int]):
        super().__init__(substep=True)
        self.wind_vector = np.array(wind_vector, dtype=float)
        self.link_indices = link_indices

    @classmethod
    def from_options(cls, config: dict, experiment_options) -> 'WindForceExtension':
        return cls(
            wind_vector=config.get('wind_vector', [0.5, 0.0, 0.0]),
            link_indices=config.get('link_indices', [0]),
        )

    def initialize_episode(self, task, physics):
        """Called once at simulation start. Cache MuJoCo body IDs here."""
        # Pre-cache body IDs for efficient per-step access
        self._body_ids = [
            physics.model.body(name).id
            for name in task.data.animats[0].sensors.links.names
        ]

    def before_step(self, task, action, physics):
        """Called every substep. Apply forces to physics.data.xfrc_applied."""
        force_torque = np.concatenate([self.wind_vector, [0, 0, 0]])  # [Fx,Fy,Fz,Tx,Ty,Tz]
        for link_idx in self.link_indices:
            # IMPORTANT: Use += to accumulate with other active extensions
            physics.data.xfrc_applied[self._body_ids[link_idx]] += force_torque
```

Register it in `simulation_config.yaml`:

```yaml
extensions:
  - loader: my_package.WindForceExtension
    config:
      wind_vector: [0.5, 0.0, 0.0]
      link_indices: [0, 1, 2]
```

!!! warning "Extension Accumulation"
    Always use `+=` when writing to `physics.data.xfrc_applied`, never `=`. Overwriting with `=` will erase forces applied by other extensions (e.g., `SwimmingExtension`).

### Recipe D — Adding a New Logged Data Channel

If you need to record a custom quantity (e.g., muscle power, joint work, custom state), you must extend the data structures.

**Step 1**: Create a new data class in your package:

```python
# my_package/data.py
import numpy as np
from farms_core.model.data import Data

class PowerData(Data):
    """Records instantaneous mechanical power per joint."""

    def __init__(self, n_joints: int, n_iterations: int):
        super().__init__()
        # Allocate a ring buffer: shape = (n_iterations, n_joints)
        self.power = np.zeros((n_iterations, n_joints), dtype=float)

    def update(self, iteration: int, torques: np.ndarray, velocities: np.ndarray):
        idx = iteration % self.power.shape[0]
        self.power[idx] = torques * velocities  # P = τ · ω
```

**Step 2**: Write to it from a controller or extension during `before_step()`:

```python
def before_step(self, task, action, physics):
    idx = task.iteration % self.buffer_size
    torques = task.data.animats[0].sensors.joints.active_torques[idx]
    velocities = task.data.animats[0].sensors.joints.velocities[idx]
    self.power_data.update(task.iteration, torques, velocities)
```

**Step 3**: Save it manually after the simulation:

```python
from farms_core.io.hdf5 import dict_to_hdf5

# After sim.run() completes
dict_to_hdf5('Output/power.hdf5', {'power': my_power_data.power})
```

### Recipe E — Adding a New CPG Network Topology

Extend the CPG network structure by modifying the `osc2osc` coupling list in `animat_config.yaml`. Add bidirectional connections between oscillators:

```yaml
network:
  osc2osc:
    # Anti-phase coupling between left and right oscillators (undulation)
    - in: osc_body_0_R
      out: osc_body_0_L
      type: OSC2OSC
      weight: 30.0
      phase_bias: 3.14159   # π = anti-phase

    # Forward coupling between adjacent body segments (travelling wave)
    - in: osc_body_1_L
      out: osc_body_0_L
      type: OSC2OSC
      weight: 30.0
      phase_bias: 1.0472    # π/3 = 60° inter-segment phase lag
```

The `phase_bias` value controls the spatial wavelength of the travelling wave. For a Zbot with 6 joints, a bias of `π/3` creates one full wavelength across the body.

---

## Part 5 — Cython Development Guide

Cython is the most performance-sensitive part of FARMS. This section gives practical guidance for working with `.pyx` files safely.

### File Types

| Extension | Purpose |
|-----------|---------|
| `.pyx` | Cython source — compiled to C then to a `.so`/`.pyd` shared library |
| `.pxd` | Cython declaration file — like a C header, allows `cimport` |
| `.c` | Generated C code — **do not edit manually** |
| `.pyd` / `.so` | Compiled binary extension — **do not commit to Git** |

### The `DEBUG` Flag

Each `setup.py` has a `DEBUG = False` constant at the top. Set it to `True` when developing:

```python
# setup.py
DEBUG = True   # Enable during development
```

This activates the following Cython compiler directives:

| Directive | Effect |
|-----------|--------|
| `boundscheck` | Raises `IndexError` on out-of-bounds array access |
| `nonecheck` | Raises `UnboundLocalError` on access to None typed variables |
| `wraparound` | Raises `IndexError` on negative index access |
| `initializedcheck` | Raises `RuntimeError` if C++ objects are not initialized |

Always set `DEBUG = False` before submitting a merge request. Production builds use `-O3` optimisation and disabled bounds checking for maximum speed.

### Understanding the `.pxd` Import Chain

`farms_amphibious` depends on Cython type definitions from `farms_core` at compile time. This is resolved through `cimport` in `.pyx` files and `.pxd` declaration files:

```cython
# farms_amphibious/control/ekeberg.pyx
from farms_core.array.array cimport DoubleArray2D  # cimport = C-level import
```

This is why `farms_core` **must be installed before** `farms_amphibious` can be compiled. The `setup.py` for `farms_amphibious` calls `from farms_core import get_include_paths()` to locate the installed `.pxd` files.

If you add a new `.pxd` file to `farms_core`, you must add its folder to the `package_data` list in `farms_core/setup.py`:

```python
package_data={'farms_core': [
    f'{folder}*.pxd'
    for folder in ['', 'array/', 'sensors/', 'model/', 'utils/', 'my_new_folder/']
]},
```

---

## Part 6 — Testing and Validation

### Current Test Status

There is currently no automated `pytest` suite. All validation is done by running reference simulations manually and inspecting outputs.

### Manual Validation Procedure

After making any change, run the following:

**1. Run the reference simulation:**

```bash
cd /app/experiments/zbot_swimming
farmsim --experiment_config experiment_config.yaml --headless
```

Expected outcome: simulation completes all 5001 iterations without error or numerical divergence (NaN/Inf).

**2. Check the output data:**

```python
import h5py
import numpy as np

with h5py.File('Output/simulation.hdf5', 'r') as f:
    positions = f['animats/0/joints/positions'][:]
    assert not np.any(np.isnan(positions)), "NaN detected in joint positions!"
    assert not np.any(np.isinf(positions)), "Inf detected in joint positions!"
    print(f"Max joint position: {np.max(np.abs(positions)):.4f} rad")
```

**3. Run the analysis script:**

```bash
python analysis.py
```

Inspect the generated plots for physically plausible joint trajectories, stable oscillation amplitudes, and smooth undulation patterns. Any sudden jumps, flat lines, or diverging amplitudes indicate a bug.

### Adding a Simple Regression Test

Until a formal test suite exists, you can add a minimal smoke test script alongside your experiment:

```python
# test_my_change.py
import sys
from farms_sim.simulation import setup_from_clargs, run_simulation
import numpy as np

sys.argv = ['farmsim', '--experiment_config', 'experiment_config.yaml']
clargs, exp_options, simulator = setup_from_clargs()

# Override to run for fewer iterations
exp_options.simulation.runtime.n_iterations = 100
exp_options.simulation.runtime.headless = True

sim = run_simulation(exp_options, simulator=simulator)

# Validate output
positions = sim.data.animats[0].sensors.joints.positions.array
assert not np.any(np.isnan(positions[:100])), "NaN in output!"
print("✓ Smoke test passed")
```

---

## Part 7 — CI/CD Pipeline

The `.gitlab-ci.yml` in each repository is currently configured **only** to build Sphinx documentation and deploy it to GitLab Pages. It runs manually (not on every push):

```yaml
rules:
  - when: manual
```

It does **not** run integration tests. When contributing changes that affect simulation behaviour, you must validate manually as described in Part 6.

Future CI work areas (contributions welcome):

- A headless smoke-test job that runs the Zbot swimming experiment for 100 iterations
- A Cython build check job to catch compilation errors early
- A numerical regression job comparing key outputs against reference HDF5 files

---

## Part 8 — Documentation

Documentation is written in **Markdown** and built with **MkDocs** using the Material theme. Math equations are rendered with MathJax.

### Building the Docs Locally

```bash
cd /app/reference
pip install mkdocs mkdocs-material
mkdocs serve
```

The site is available at `http://127.0.0.1:8000`.

### Documentation Standards

Follow these conventions when writing or editing documentation:

- **"Where" clauses**: Every equation must be followed by a `Where:` block defining each symbol. Each definition must be on its own line (blank line between items).
- **Code blocks**: Always specify the language for syntax highlighting (` ```python `, ` ```yaml `, ` ```bash `).
- **Admonitions**: Use `!!! note`, `!!! tip`, `!!! warning`, `!!! important` for callouts. Never use `!!! tip "See also"` for See Also sections — use `## See Also` instead.
- **See Also sections**: Use a `## See Also` heading at the bottom of every API page, not an admonition block.
- **File links**: Use descriptive link text, not full paths.

### Adding a New Doc Page

1. Create your `.md` file under `reference/docs/`.
2. Add it to `reference/mkdocs.yml` under the appropriate `nav` section:

```yaml
nav:
  - API Reference:
    - My New Page: api/my_new_page.md
```

---

## Summary Checklist

Before submitting a contribution, verify:

- [ ] Code follows PEP 8 and has docstrings on all public functions and classes
- [ ] Cython `.pyx` files use typed memoryviews and C types for all hot-path variables
- [ ] `DEBUG = False` in all `setup.py` files
- [ ] New YAML parameters are documented in [Configuration Reference](./configuration.md)
- [ ] Reference simulation runs to completion without NaN/Inf
- [ ] `analysis.py` plots look physically plausible
- [ ] New doc pages are added to `mkdocs.yml`
- [ ] Commit messages follow Conventional Commits format

## See Also

- [Architecture & Data Flow](./architecture.md) — understand module boundaries before making changes
- [Zbot tutorial](./zbot/index.md) — end-to-end example of the system working together
- [Configuration Reference](./configuration.md) — all YAML parameters
- [Mathematical Models](./mathematical_models.md) — CPG and muscle model equations
- [`farms_core.model.control`](api/farms_core_control.md) — controller base class API
