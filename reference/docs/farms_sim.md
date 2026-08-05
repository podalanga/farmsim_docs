# `farms_sim` — Simulation Orchestrator

`farms_sim` is the top-level entry point for the FARMS framework. It parses CLI arguments, loads experiment configuration, instantiates the appropriate physics backend, and drives the simulation loop. It defines no domain-specific controllers or physics models — those live in `farms_mujoco` and `farms_amphibious`.

**Source**: `farms_sim/farms_sim/simulation.py`, `farms_sim/farms_sim/farmsim.py`

---

## Overview

`farms_sim` operates in a purely functional style — it has no class hierarchy of its own. The module contains five functions that cover the full simulation lifecycle: argument parsing, backend setup, execution, post-processing, and interactive prompts.

Backend selection is determined at import time: `farms_sim` attempts to import `farms_mujoco` and `farms_bullet` and flags each as available. A `ModuleNotFoundError` is raised at startup if neither is installed.

---

## Entry Point

```bash
python -m farms_sim.farmsim --experiment_config <path/to/config.yaml> [--simulator MUJOCO|PYBULLET] [--profile] [--headless]
```

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--experiment_config` | `str` | *(required)* | Path to the experiment YAML configuration file |
| `--simulator` | `str` | `MUJOCO` | Physics backend: `MUJOCO` or `PYBULLET` |
| `--profile` | flag | off | Run with Python profiler; outputs call graph to stdout |
| `--log_path` | `str` | `''` | Directory for HDF5 data and YAML output |
| `--prompt` | flag | off | Enable interactive post-processing prompt after simulation |
| `--verify_save` | flag | off | Prompt user to confirm before overwriting saved data |

**Tip**: Set `headless: true` in your YAML for batch jobs or cluster runs. With `play: true`, an interactive MuJoCo viewer is launched.

---

## Function Reference

### `setup_from_clargs`

```python
def setup_from_clargs(clargs=None, **kwargs) -> tuple[Namespace, ExperimentOptions, Simulator]
```

Parses CLI arguments and loads the experiment configuration.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `clargs` | `Namespace \| None` | `None` | Pre-parsed argument namespace; if `None`, calls `sim_parse_args()` |
| `experiment_options_loader` | `type` | `ExperimentOptions` | Class used to deserialise the YAML config (override for custom options classes) |

**Returns**: `(clargs, experiment_options, simulator)` where `simulator` is a `Simulator` enum value.

---

### `simulation_setup`

```python
def simulation_setup(experiment_options: ExperimentOptions, **kwargs) -> MuJoCoSimulation | PybulletSimulation
```

Instantiates the physics backend without running it.

| Keyword | Type | Default | Description |
|---------|------|---------|-------------|
| `simulator` | `Simulator` | `Simulator.MUJOCO` | Backend selection |
| `experiment_data` | `ExperimentData \| None` | auto-constructed | Pre-built data container; constructed from options if not provided |
| `experiment_data_class` | `type` | `ExperimentData` | Class for data construction when `experiment_data` is not supplied |
| `extensions` | `list` | `[]` | Additional `TaskExtension` instances to inject (MuJoCo only) |
| `save_mjcf` | `bool` | `False` | Write the generated MJCF XML to disk before simulation |
| `handle_exceptions` | `bool` | `False` | Catch and log exceptions within the task loop rather than raising |

For MuJoCo: calls `MuJoCoSimulation.from_experiment(...)`.
For PyBullet: calls the legacy `sim_loader(...)` interface.

---

### `run_simulation`

```python
def run_simulation(experiment_options: ExperimentOptions, **kwargs) -> MuJoCoSimulation | PybulletSimulation
```

Calls `simulation_setup` then runs the simulation loop to completion. Accepts all `simulation_setup` keyword arguments.

- **MuJoCo**: calls `sim.run()` — blocks until `target_time` is reached.
- **PyBullet**: iterates `sim.iterator(show_progress=...)` manually.

Returns the completed simulation object (data available via `sim.data`).

---

### `simulation_post`

```python
def simulation_post(sim, log_path='', plot=False, video='')
```

Invokes post-processing on a completed simulation object.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sim` | `MuJoCoSimulation` | *(required)* | Completed simulation object |
| `log_path` | `str` | `''` | Directory to write HDF5 data and YAML configs |
| `plot` | `bool` | `False` | Generate and display analysis plots |
| `video` | `str` | `''` | Output video path; ignored if `sim.options.headless` is `True` |

---

### `postprocessing_from_clargs`

```python
def postprocessing_from_clargs(sim, clargs=None, **kwargs)
```

Interactive post-processing prompt, driven by CLI arguments. Calls `prompt_postprocessing`.

---

## Usage Example

```python
from farms_sim.simulation import setup_from_clargs, run_simulation, simulation_post

# Standard single-run pattern
clargs, exp_options, simulator = setup_from_clargs()
sim = run_simulation(exp_options, simulator=simulator)
simulation_post(sim, log_path=clargs.log_path, plot=True)
```

**RL integration** — bypass the blocking loop and step manually:

```python
from farms_sim.simulation import setup_from_clargs, simulation_setup

clargs, exp_options, _ = setup_from_clargs()
sim = simulation_setup(exp_options)

# sim.env is a dm_control Environment
timestep = sim.env.reset()
while not timestep.last():
    action = my_policy(timestep.observation)
    timestep = sim.env.step(action)
```

---

## See Also

- [farms_core.simulation.extensions](api/farms_core_control.md) — `TaskExtension` base class
- [farms_mujoco.simulation](api/farms_mujoco_simulation.md) — physics backend lifecycle
- [Simulation walkthrough](./walkthrough.md) — end-to-end narrative
- **Source**: `farms_sim/farms_sim/simulation.py`
