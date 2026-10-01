<div class="hero-title" markdown>
FARMS
</div>

FARMS (Framework for Animal and Robot Modeling and Simulation) is a Python
framework for simulating and controlling animal models and robots. This site
documents it as used in the zbot project: undulatory swimming robots simulated
with the [MuJoCo](https://mujoco.org/) physics engine, driven by a CPG (Central
Pattern Generator) locomotion controller and a fast hydrodynamics module.

<div class="hero-buttons" markdown>
[Get started](tutorials/install-and-run.md){ .md-button .md-button--primary }
[Browse the reference](reference/env/yaml-schema.md){ .md-button }
</div>

## What FARMS provides

<div class="grid cards" markdown>

-   :material-file-cog-outline: **YAML-driven configuration**

    ---

    Define robots, arenas, and simulations through hierarchical YAML files
    loaded via dotted Python paths.

-   :material-puzzle-outline: **Extensible architecture**

    ---

    Plug in custom controllers, sensors, and simulation extensions through a
    lifecycle-based extension system.

-   :material-wave: **CPG locomotion control**

    ---

    Built-in oscillator network model with drives, sensory feedback, and
    multiple muscle equations (phase, Ekeberg, passive).

-   :material-cube-outline: **MuJoCo integration**

    ---

    Automatic SDF-to-MJCF conversion and interactive or headless simulation
    modes.

-   :material-waves: **Hydrodynamics**

    ---

    Exact centre of buoyancy (closed forms and meshes) or O(1) lookup
    tables, per-link drag, and an ellipsoid drag and added mass model, all
    computed in C on a single core.

-   :material-database-outline: **Data persistence**

    ---

    HDF5-based recording of all sensor data, network states, and simulation
    parameters.

</div>

## Documentation structure

This documentation follows the [Diátaxis](https://diataxis.fr/) framework:

<div class="grid cards" markdown>

-   :material-school-outline: **[Tutorials](tutorials/install-and-run.md)**

    ---

    Learn FARMS step by step, from installation through writing your first
    controller.

-   :material-hammer-wrench: **[How-to Guides](how-to/configure-yaml.md)**

    ---

    Task-oriented recipes for common configuration, extension, and
    integration work.

-   :material-book-open-variant: **[Reference](reference/env/yaml-schema.md)**

    ---

    Technical descriptions of modules, classes, YAML schemas, and CLI
    options. The API, configuration and CLI references are generated from
    the code at every build.

-   :material-lightbulb-on-outline: **[Explanation](explanation/architecture.md)**

    ---

    Architecture rationale and design decisions.

</div>

## Quick start

Both paths start from the same clone. Pick Docker for a ready-made
environment, or a virtual environment to work on the FARMS sources
directly on the host.

<div class="grid quickstart" markdown>

<div class="card" markdown>

:material-docker: **Docker** (recommended)

---

```bash
# Clone (the build fetches the rest)
git clone \
 git@github.com:podalanga/farms_zbot.git
cd farms_zbot

# Linux: allow X11 windows
xhost +local:docker

# Build, start and enter
cd docker_config/linux
docker compose up --build -d
docker exec -it zbot_farms_linux bash

# Inside the container
cd experiments/zbot_bout_glide
python run_sim.py --experiment_config \
  experiment_config.yaml
```

On Windows, run compose from `docker_config/windows` and enter the
`zbot_farms_windows` container.

</div>

<div class="card" markdown>

:material-language-python: **Virtual environment**

---

```bash
# Clone with submodules and meshes
git clone \
 git@github.com:podalanga/farms_zbot.git
cd farms_zbot
git lfs pull
git submodule update \
  --init --recursive

# Install FARMS (Python >= 3.11)
python3 -m venv .venv
source .venv/bin/activate
cd farms
python setup_farms.py

# Run the bout-and-glide experiment
cd ../experiments/zbot_bout_glide
python run_sim.py --experiment_config \
  experiment_config.yaml
```

Needs a C compiler, Git LFS and OpenGL on the host.

</div>

</div>

See the [installation guide](tutorials/install-and-run.md) for full
details (Docker and native, side by side).
