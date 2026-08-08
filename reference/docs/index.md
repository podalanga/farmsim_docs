<div class="hero-title" markdown>
FARMS
</div>

Framework for Amphibious Robot Modeling and Simulation — a Python-based robotics
simulation framework for modelling and controlling undulatory swimming robots.
It integrates with the [MuJoCo](https://mujoco.org/) physics engine and provides
a CPG (Central Pattern Generator) based locomotion control system.

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

    Automatic SDF-to-MJCF conversion, fluid force computation, and
    interactive/headless simulation modes.

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
    options.

-   :material-lightbulb-on-outline: **[Explanation](explanation/architecture.md)**

    ---

    Architecture rationale and design decisions.

</div>

## Quick start

```bash
# Clone, then fetch submodules and LFS-tracked meshes
git clone git@github.com:farmsim/farms_zbot.git
cd farms_zbot
git lfs pull
git submodule update --init --recursive

# Install FARMS packages into an active virtual environment
cd farms
python setup_farms.py

# Run the zbot bout-glide experiment
cd ../experiments/zbot_bout_glide
python run_sim.py --experiment_config experiment_config.yaml
```

See the [installation guide](tutorials/install-and-run.md) for full
details (Docker and native, side by side).
