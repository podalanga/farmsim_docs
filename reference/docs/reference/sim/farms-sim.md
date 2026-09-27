# farms_sim Reference

`farms_sim` is the entry point of a simulation: it parses the command line,
loads the options and data, creates the simulation of the chosen engine
(`farms_mujoco`, or `farms_bullet` when installed) and runs it.

```text
farms_sim/
├── _bootstrap.py         # main(): macOS mjpython handling, then profile_simulation()
├── farmsim.py            # main(), profile_simulation()
├── simulation.py         # setup_from_clargs, simulation_setup, run_simulation, ...
└── utils/
    ├── parse_args.py     # sim_argument_parser, sim_parse_args
    └── prompt.py         # prompt_postprocessing (interactive, legacy)
```

The command line options are listed in the generated
[CLI Reference](../env/cli.md).

## Entry points

### `run_sim.py`

Each experiment folder has a `run_sim.py`:

```python
import os
import sys

current_dir = os.path.abspath(os.path.dirname(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)  # Makes controller/ importable

from farms_sim._bootstrap import main

for i, arg in enumerate(sys.argv):  # Accept --experiment-config too
    if arg == '--experiment-config':
        sys.argv[i] = '--experiment_config'

if __name__ == '__main__':
    sys.exit(main())
```

```bash
python run_sim.py --experiment_config experiment_config.yaml
```

The `farmsim` console command calls the same `_bootstrap.main()`, but does
not add the experiment folder to `sys.path` (set `PYTHONPATH=.` for local
controllers).

### `_bootstrap.main()`

Takes no arguments. On macOS, the MuJoCo viewer needs `mjpython`: if it is
on `PATH`, the process re-executes itself as
`mjpython -m farms_sim.farmsim ...` (the `FARMS_UNDER_MJPYTHON` variable
prevents loops), otherwise it prints a warning and continues. It then
calls `farms_sim.farmsim.profile_simulation()`.

### `farmsim.profile_simulation()` and `farmsim.main()`

`profile_simulation()` reads `--profile` and runs `main()` through
`farms_core.utils.profile.profile()`, which saves a profile only when a
file name is given. `main()`:

1. `setup_from_clargs()`: parses the arguments and loads the
   `ExperimentOptions` of `--experiment_config`;
2. allocates the data with the class of `loaders.experiment_data`;
3. `run_simulation()`: creates and runs the simulation.

## `farms_sim.simulation`

### Engines

At import, `farms_mujoco` and `farms_bullet` are imported if installed
(`ENGINE_MUJOCO`, `ENGINE_BULLET`). A package that is installed but fails
to import raises an `ImportError`, and the import fails if neither engine
is installed.

### `setup_from_clargs(clargs=None, **kwargs)`

Returns `(clargs, experiment_options, simulator)`. `experiment_options_loader`
(default `ExperimentOptions`) can be given as a keyword.

### `simulation_setup(experiment_options, **kwargs)`

Creates the simulation without running it. For MuJoCo:
`Simulation.from_experiment(experiment_options, data=experiment_data, extensions=..., handle_exceptions=..., save_mjcf=..., buffer_size=runtime.buffer_size)`.
Keywords: `simulator`, `experiment_data`, `experiment_data_class`,
`handle_exceptions`, and for MuJoCo `extensions` (extra extension
instances) and `save_mjcf`.

### `run_simulation(experiment_options, **kwargs)`

Calls `simulation_setup()`, then `sim.run()` for MuJoCo (the whole loop
runs in `farms_mujoco`), or iterates `sim.iterator()` and calls `sim.end()`
for PyBullet. Returns the simulation.

To run a simulation from Python, for example in a notebook or an RL loop:

```python
from farms_core.experiment.options import ExperimentOptions
from farms_core.experiment.data import ExperimentData
from farms_core.simulation.options import Simulator
from farms_sim.simulation import run_simulation

options = ExperimentOptions.load('experiment_config.yaml')
options.simulation.runtime.headless = True
data = ExperimentData.from_options(options)
sim = run_simulation(
    experiment_options=options,
    experiment_data=data,
    simulator=Simulator.MUJOCO,
)
# run() closes the simulation at the end: read the results from data
links = data.animats[0].sensors.links
print(links.global_com_position(iteration=options.simulation.runtime.n_iterations - 1))
```

Run it from the experiment folder, with the folder on `sys.path` if the
animat uses a local controller.

### Post-processing functions

`simulation_post()`, `postprocessing_from_clargs()` and
`utils/prompt.py:prompt_postprocessing()` (the `--prompt` and
`--verify_save` options) are not called by `farmsim.main()`, and they call
`Simulation.postprocess()`, which now raises a `DeprecationWarning` with
MuJoCo. Save the data with the `ExperimentLogger` extension instead (see
[Save, Load, and Inspect Data](../../how-to/save-load-data.md)).

## Command line

`sim_parse_args()` uses `parse_known_args()`: **unknown arguments are
ignored silently**. A mistyped or non-existent option (for example a
headless flag, which does not exist) has no effect and raises no error. The viewer, speed and
duration are set in `simulation_config.yaml` (`runtime.headless`,
`runtime.fast`, `runtime.n_iterations`).

| Option | Used by `farmsim.main()` |
|--------|-------------------------|
| `--experiment_config` | Yes, required |
| `--simulator` | Yes (`MUJOCO` or `PYBULLET`) |
| `--profile` | Yes, profile file name |
| `--log_path`, `--prompt`, `--verify_save` | No (post-processing functions only) |
| `--test_configs` | Raises a `NameError`: the code uses an undefined `animat_options_loader` |

## See also

- [CLI Reference](../env/cli.md) (generated)
- [Simulation Lifecycle](../../explanation/simulation-lifecycle.md)
- [Install and Run](../../tutorials/install-and-run.md)
