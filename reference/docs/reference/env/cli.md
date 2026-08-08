# CLI Reference

Command-line interface for running FARMS simulations.

!!! warning "Rewritten — the previous version of this page was fabricated"
    The previous draft listed `--plot`, `--video`, `--fast` flags and a
    `./simulations` default `--log_path` that do not exist in
    `farms_sim/utils/parse_args.py`, and showed `_bootstrap.main(__file__)`
    and `farms_sim.simulation.main()`, neither of which exists. See
    `reference/farms-sim.md` for the fully-verified walkthrough this page
    summarizes; every claim below is drawn from that page.

## Usage

```bash
python run_sim.py --experiment_config <path> [options]
```

## Arguments

Defined in `farms_sim/utils/parse_args.py` via `sim_argument_parser()` /
`sim_parse_args()`. Parsing uses `parse_known_args()`, so unrecognized
flags are silently ignored rather than raising an error.

| Argument | Type | Required | Default | Description |
|----------|------|----------|---------|-------------|
| `--experiment_config` | str | Yes (asserted) | `None` | Path to the experiment YAML |
| `--simulator` | str, choices `MUJOCO`/`PYBULLET` | No | `'MUJOCO'` | Simulator backend — **upper-case** values |
| `--profile` | str | No | `''` | If non-empty, save a profile to this filename |
| `--log_path` | str | No | `''` | Output directory for simulation data |
| `--test_configs` | flag | No | `False` | Round-trip options through save/load (has a known `NameError` bug — see `reference/farms-sim.md`) |
| `--prompt` | flag | No | `False` | Interactively prompt at the end of the run to save data / show plots |
| `--verify_save` | flag | No | `False` | Reload saved data after saving, as a sanity check |

There is **no `--plot`, `--video`, or `--fast` flag**. Plotting and video
output are controlled programmatically (`simulation_post(..., plot=,
video=)`) or interactively through `--prompt`, not via dedicated CLI
switches.

## Examples

### Basic simulation

```bash
python run_sim.py --experiment_config experiment_config.yaml
```

### Logging data, with interactive save/plot prompt

```bash
python run_sim.py \
    --experiment_config experiment_config.yaml \
    --log_path ./output/run_001 \
    --prompt
```

### Profiling a run

```bash
python run_sim.py \
    --experiment_config experiment_config.yaml \
    --profile ./profile_output.prof
```

## Entry point flow

```
run_sim.py                          # adds its own dir to sys.path
  → farms_sim._bootstrap.main()     # NO ARGS; macOS/mjpython re-exec check
    → farms_sim.farmsim.profile_simulation()
      → farms_sim.farmsim.main()
        → simulation.setup_from_clargs()   # parse args, load ExperimentOptions
        → import_item(loaders.experiment_data).from_options(...)  # allocate data
        → simulation.run_simulation()
          → simulation.simulation_setup()  # build MuJoCoSimulation / PybulletSimulation
          → sim.run()                      # MuJoCo: hands off to farms_mujoco's loop
```

See `reference/farms-sim.md` for the fully-annotated version of this
chain, including the `run_sim.py` `sys.path`/hyphen-fixup convention and
the double CLI-parsing quirk in `profile_simulation()`.

## Programmatic usage

`farms_sim.farmsim.main()` (not `farms_sim.simulation.main` — that
function doesn't exist) still reads from `sys.argv` internally via
`sim_parse_args()`, so the equivalent of a CLI call from Python is:

```python
import sys
from farms_sim.farmsim import main

sys.argv = [
    'run_sim.py',
    '--experiment_config', 'experiment_config.yaml',
    '--log_path', './output',
]
main()
```

For finer control without going through `sys.argv` at all, call the
lower-level functions directly:

```python
from farms_core.experiment.options import ExperimentOptions
from farms_core.simulation.options import Simulator
from farms_sim.simulation import run_simulation

experiment_options = ExperimentOptions.load('experiment_config.yaml')
sim = run_simulation(
    experiment_options=experiment_options,
    simulator=Simulator.MUJOCO,
)
```

This bypasses `--test_configs`/`--prompt`/CLI parsing entirely and is the
pattern to use from a notebook or a custom script.
