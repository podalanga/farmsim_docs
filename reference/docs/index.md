<div class="hero-title" markdown>
FarmSim
</div>

FarmSim, the simulation side of FARMS (Framework for Animal and Robot
Modeling and Simulation), simulates and controls animal models and
bio-inspired robots with the [MuJoCo](https://mujoco.org/) physics engine.
You describe a robot in SDF and an experiment in YAML. FarmSim builds the
MuJoCo model, runs controllers such as central pattern generators (CPGs),
adds water forces for swimming, and records everything to HDF5.

<div class="hero-buttons" markdown>
[Get started](get-started/index.md){ .md-button .md-button--primary }
[Tutorials](tutorials/index.md){ .md-button }
[Reference](reference/env/yaml-schema.md){ .md-button }
</div>

![AmphiBot, the example robot of these docs, crawls down a ramp into a pool and swims away](assets/figures/amphibot-land-to-water.gif){ .hero-figure }

## What FarmSim provides

<div class="grid cards" markdown>

-   **YAML-driven experiments**

    ---

    Robots, arenas and simulations are described in YAML files and loaded
    into typed option classes.

-   **Extensible lifecycle**

    ---

    Controllers, sensors and loggers plug in as extensions with a fixed
    lifecycle: initialize, before step, after step.

-   **CPG locomotion control**

    ---

    A network of coupled oscillators with descending drives, sensory
    feedback and several muscle models.

-   **MuJoCo integration**

    ---

    SDF to MJCF conversion, interactive viewer or headless runs, and
    offscreen rendering.

-   **Hydrodynamics**

    ---

    Exact centre of buoyancy or O(1) lookup tables, per-link drag, and an
    ellipsoid drag and added mass model, computed in C on a single core.

-   **Data recording**

    ---

    Every sensor, network state and option is saved to HDF5 and YAML, ready
    for analysis.

</div>

## The packages

| Package | Role |
|---|---|
| `farms_core` | Options, data arrays, sensors, SDF/HDF5/YAML I/O, extension and controller base classes |
| `farms_mujoco` | MuJoCo simulation: MJCF builder, experiment task, viewer, swimming and buoyancy |
| `farms_amphibious` | CPG network, muscle models, descending drives, amphibious options |
| `farms_sim` | Command line entry point that ties the packages together |

## Quick start

Both paths install the four packages and run the
[AmphiBot example](tutorials/first-simulation.md). Docker gives a
ready-made environment; a virtual environment installs FarmSim directly on
the host.

<div class="grid quickstart" markdown>

<div class="card" markdown>

**Docker** (recommended)

---

```bash
git clone \
 https://github.com/podalanga/farmsim_docs.git
cd farmsim_docs/docker

# Linux: allow X11 windows
xhost +local:docker

# Build, start and enter
docker compose up --build -d
docker exec -it farmsim bash

# Inside the container
cd examples/amphibot
python run_sim.py --experiment_config \
  experiment_config.yaml
```

On Windows, see [Installation](get-started/installation.md#docker) for
the display setup.

</div>

<div class="card" markdown>

**Virtual environment**

---

```bash
git clone \
 https://github.com/podalanga/farmsim_docs.git
cd farmsim_docs

# Python >= 3.11
python3 -m venv .venv
source .venv/bin/activate
pip install pyyaml
python reference/tools/install_farms.py

# Run the AmphiBot example
cd examples/amphibot
python run_sim.py --experiment_config \
  experiment_config.yaml
```

Needs a C compiler and OpenGL on the host.

</div>

</div>

See [Installation](get-started/installation.md) for the details and
[Supported platforms](get-started/platforms.md) for what is tested.

## Where to go next

| You want to | Read |
|---|---|
| Run something in five minutes | [Your first simulation](tutorials/first-simulation.md) |
| Learn FarmSim step by step | [Tutorials](tutorials/index.md) |
| Do a specific task | [How-to guides](how-to/configure-yaml.md) |
| Look up an option, class or command | [Reference](reference/env/yaml-schema.md) |
| Understand the design | [Explanation](explanation/architecture.md) |
| See FarmSim in a real project | [Projects](projects/index.md) |
| Fix a problem | [Troubleshooting](help/troubleshooting.md) and [FAQ](help/faq.md) |

## About these docs

The documentation follows the [Diataxis](https://diataxis.fr/) framework:
tutorials teach, how-to guides solve tasks, the reference describes, and
explanations give the reasons. The API, configuration and CLI references
are generated from the code at every build.
