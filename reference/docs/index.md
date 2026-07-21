# FARMS Documentation

**FARMS** (Framework for Animat and Robot Morphologies in Simulation) is a modular simulation framework for biorobotics and neuro-mechanical locomotion research. It wraps the MuJoCo physics engine via `dm_control`, and provides CPG-based neural controllers, hydrodynamic models, and a plugin system for extending both the physics and the control layers.

---

## Package Overview

| Package | Repository | Purpose |
|---------|-----------|---------|
| [`farms_core`](farms_core.md) | `farms_core` | Data schemas, sensor arrays, Options/Data classes, extension interfaces, I/O |
| [`farms_sim`](farms_sim.md) | `farms_sim` | CLI entry point, simulation lifecycle orchestration, backend dispatch |
| [`farms_mujoco`](farms_mujoco.md) | `farms_mujoco` | MuJoCo physics backend, MJCF generation, task lifecycle, hydrodynamics |
| [`farms_amphibious`](farms_amphibious.md) | `farms_amphibious` | CPG networks, descending drive, Ekeberg muscle model, joint actuators |

---

## Quick Start

```bash
# Clone the repository and fetch submodules
git clone git@github.com:farmsim/farms_zbot.git
cd farms_zbot
git submodule update --init --recursive

# Initialize and activate virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows use: .venv\Scripts\Activate.ps1

# Install all FARMS packages
cd farms
python setup_farms.py

# Run a sample simulation
cd ../experiments/zbot_swimming
python -m farms_sim.farmsim --experiment_config experiment_config.yaml
```

See [Installation](installation.md) for full setup instructions including Cython compilation.

---

## Documentation Map

### Getting Started
- [Installation](installation.md) - prerequisites, native install, Docker
- [Simulation Walkthrough](walkthrough.md) - end-to-end lifecycle narrative

### Module Guides
- [farms_core](farms_core.md) - Options/Data architecture, extension system, built-in loggers
- [farms_sim](farms_sim.md) - CLI flags, function reference, RL integration
- [farms_mujoco](farms_mujoco.md) - Simulation class, viewer modes, applying external forces
- [farms_amphibious](farms_amphibious.md) - CPG math, muscle models, descending drive

### API Reference
- [Controller & Extension Interfaces](api/farms_core_control.md)
- [Sensor Data Arrays](api/farms_core_sensors.md)
- [Options Hierarchy](api/farms_core_options.md)
- [I/O Utilities](api/farms_core_io.md)
- [MuJoCo Simulation & Extensions](api/farms_mujoco_simulation.md)
- [Hydrodynamic Swimming Model](api/farms_mujoco_swimming.md)
- [CPG Oscillator Network](api/cpg_oscillators.md)
- [Ekeberg Muscle Model](api/ekeberg_muscle.md)
- [Joint Controllers](api/joint_controllers.md)
- [Amphibious Controller](api/farms_amphibious_controller.md)
- [Network ODE Integrator](api/farms_amphibious_network.md)
- [Descending Drive](api/farms_amphibious_drive.md)
- [Amphibious Data Classes](api/farms_amphibious_data.md)
- [Amphibious Options](api/farms_amphibious_options.md)

### Reference
- [System Architecture](architecture.md) - data flow diagrams, class hierarchies, execution loop
- [Mathematical Models](mathematical_models.md) - CPG ODEs, drag equations, Ekeberg model
- [Configuration Reference](configuration.md) - all YAML parameter tables
- [Core Interfaces](core_interfaces.md) - abstract base class contracts
- [Glossary](glossary.md)
- [Contributing](contributing.md)

---

## User Paths

**Running experiments** - configure animats via YAML, use the CLI, post-process HDF5 output:
1. [Installation](installation.md)
2. [Walkthrough](walkthrough.md)
3. [Configuration Reference](configuration.md)

**Extending the framework** - new controllers, physics models, or animat morphologies:
1. [Core Interfaces](abstraction_audit.md)
2. [System Architecture](architecture.md)
3. [Contributing](contributing.md)
