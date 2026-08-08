# farms_sim Reference

API reference for `farms_sim` — the simulation entry point and CLI handling.
`farms_sim` is a thin orchestration layer: it parses CLI args, loads
`ExperimentOptions`/`ExperimentData`, picks a simulator backend
(`farms_mujoco` or `farms_bullet`, whichever is installed), and runs it.

## Module structure

```
farms_sim/
├── _bootstrap.py         # OS-aware entry point: main() [no args]
├── farmsim.py            # main() / profile_simulation() — the real CLI entry
├── simulation.py         # setup_from_clargs, simulation_setup, run_simulation,
│                          # simulation_post, postprocessing_from_clargs
└── utils/
    ├── parse_args.py     # sim_argument_parser / sim_parse_args
    └── prompt.py         # Interactive post-run prompt (save/plot data)
```

!!! warning "Previous version of this page described a function that does not exist"
    An earlier draft documented `farms_sim._bootstrap.main(__file__)` — a
    function taking an experiment path and resolving `sys.path` from it.
    **No such function exists.** `_bootstrap.main()` takes **no arguments**.
    Adding the experiment directory to `sys.path` is done by the
    per-experiment `run_sim.py` script itself, *before* it imports
    `farms_sim`, not by `_bootstrap.main`. See "Entry point" and
    "`run_sim.py` pattern" below for the verified behaviour.

## Entry point

**Source:** `farms_sim/_bootstrap.py` (33 lines)

```python
def main():
    """Console bootstrap to handle different OS

    - MacOS: if available, re-exec under `mjpython` + run `farms_sim.farmsim`.
    - Else: import and run the real CLI directly.
    """
```

`main()` takes **no arguments**. It does exactly two things:

1. **macOS / `mjpython` handling.** MuJoCo's viewer requires running under
   Apple's `mjpython` wrapper on macOS (not plain `python`). If the current
   platform is `Darwin` and the process is not already re-executed
   (guarded by the `FARMS_UNDER_MJPYTHON` environment variable, to prevent
   infinite re-exec loops) and `mjpython` is found on `PATH`, the process
   replaces itself via `os.execvpe(mj, [mj, "-m", "farms_sim.farmsim",
   *sys.argv[1:]], env)` with `FARMS_UNDER_MJPYTHON=1` set. If `mjpython`
   isn't found, it prints a warning to stderr and continues on plain
   Python (MuJoCo may then fail to load a GUI viewer).
2. **Delegate.** On any other OS, or after the macOS check falls through,
   it imports `farms_sim.farmsim.profile_simulation` and calls it. This
   import is deliberately deferred to *after* the macOS branch, so that
   the re-exec (if it happens) occurs before any MuJoCo/dm_control modules
   are loaded in the original process.

## Real CLI entry point

**Source:** `farms_sim/farmsim.py` (45 lines)

```python
def main():
    """Main"""
    _, exp_options, simulator = setup_from_clargs()
    experiment_data_loader = import_item(exp_options.loaders.experiment_data)
    experiment_data = experiment_data_loader.from_options(exp_options)
    run_simulation(
        experiment_data=experiment_data,
        experiment_options=exp_options,
        simulator=simulator,
    )
```

This — not `_bootstrap.main` — is where argument parsing, options loading,
and data allocation actually happen:

1. `setup_from_clargs()` (see below) parses CLI args and loads
   `ExperimentOptions` from the YAML given by `--experiment_config`.
2. The experiment-data loader class is resolved dynamically via
   `import_item(exp_options.loaders.experiment_data)` — the same
   dotted-path `loaders:` block documented in
   `internals/options-yaml-internals.md` for `ExperimentOptions.load()`,
   reused here for the data container.
3. `run_simulation()` builds the simulator and runs it (see below).

```python
def profile_simulation():
    """Profile simulation"""
    tic = time.time()
    clargs = sim_parse_args()
    profile(function=main, profile_filename=clargs.profile)
    pylog.info('Total simulation time: %s [s]', time.time() - tic)
```

`profile_simulation()` re-parses args (a second, independent
`sim_parse_args()` call — cheap, but note `main()` above also parses args
internally via `setup_from_clargs()`, so CLI args are parsed twice per
run) purely to read `clargs.profile`, then wraps `main` in
`farms_core.utils.profile.profile()`, which no-ops unless
`--profile <path>` was given.

## Simulation orchestration

**Source:** `farms_sim/simulation.py` (201 lines)

### Backend availability

At import time, `farms_mujoco` and `farms_bullet` are each imported in a
`try`/`except ModuleNotFoundError` block, setting `ENGINE_MUJOCO` /
`ENGINE_BULLET` booleans. A `ModuleNotFoundError` is treated as "package
not installed" and silently tolerated; any other `ImportError` (e.g. the
package is installed but one of *its* imports is broken) is re-raised
with added context rather than swallowed. If neither engine imported
successfully, module import fails immediately with `ModuleNotFoundError:
'Neither MuJoCo nor Bullet packages are installed'`.

### `setup_from_clargs(clargs=None, **kwargs)`

```python
def setup_from_clargs(clargs=None, **kwargs):
    if clargs is None:
        clargs = sim_parse_args()
    assert clargs.experiment_config, 'No experiment config provided'
    exp_loader = kwargs.pop('experiment_options_loader', ExperimentOptions)
    experiment_options = exp_loader.load(clargs.experiment_config)
    simulator = {'MUJOCO': Simulator.MUJOCO, 'PYBULLET': Simulator.PYBULLET}[clargs.simulator]
    ...
    return clargs, experiment_options, simulator
```

Returns a **3-tuple** `(clargs, experiment_options, simulator)` — not the
`(simulator, experiment_options, experiment_data)` order previously
claimed here. `experiment_data` is not created by this function at all;
it's created separately by the caller (`farmsim.main`, above).

If `--test_configs` is passed, this function also round-trips
`AnimatOptions`/`SimulationOptions` through `.save()`/`.load()` as a
config-serialization smoke test — note the source references an
`animat_options_loader` name that is never assigned in this branch
(only `exp_loader` is defined above it), so `--test_configs` as currently
written will raise a `NameError` if exercised. This looks like a latent
bug in `farms_sim`, not a documentation error — flagged here rather than
silently "corrected" in the example.

### `simulation_setup(experiment_options, **kwargs)`

Builds — but does not run — the simulator object for either backend:

- **MuJoCo**: `MuJoCoSimulation.from_experiment(experiment_options=...,
  data=experiment_data, restart=False, extensions=..., 
  handle_exceptions=..., save_mjcf=..., buffer_size=sim_options.runtime.buffer_size)`.
- **PyBullet**: `PybulletSimulation(simulation_options=..., animat=...,
  arena_options=...)` (via a pluggable `sim_loader`, default
  `PybulletSimulation`).

`experiment_data` defaults to `experiment_data_class.from_options(experiment_options)`
if not passed in, where `experiment_data_class` defaults to `ExperimentData`
(the base `farms_core` class) — note `farmsim.main()` above instead builds
`experiment_data` via `exp_options.loaders.experiment_data` (which for the
`zbot` experiments resolves to a package-specific subclass), and passes
that pre-built object in through `run_simulation(experiment_data=...)`. So
in normal CLI use, `simulation_setup`'s own default never actually fires —
it exists for callers that invoke `simulation_setup`/`run_simulation`
directly (e.g. from a notebook or a custom script) without going through
`farmsim.main`.

### `run_simulation(experiment_options, **kwargs)`

Calls `simulation_setup()`, then:

- **MuJoCo**: `sim.run()` — the whole simulation loop runs internally to
  `farms_mujoco`'s `Simulation.run()` (see
  `internals/experiment-task-internals.md` / `reference/farms-mujoco.md`
  for what happens per step). Returns once the run completes.
- **PyBullet**: iterates `sim.iterator(show_progress=...)` manually in a
  Python `for` loop, then calls `sim.end()`.

Returns the `sim` object either way, for `simulation_post` /
`postprocessing_from_clargs` to use afterward.

### `simulation_post(sim, log_path='', plot=False, video='')`

Thin wrapper calling `sim.postprocess(iteration=sim.iteration, log_path=...,
plot=..., video=video if not sim.options.headless else '')`. Note this
function is **not** wired to any CLI flag (see "CLI arguments" below) —
it's a library-level convenience for scripts that call `run_simulation`
directly and want a one-liner for logging/plotting, bypassing the
interactive prompt path.

### `postprocessing_from_clargs(sim, clargs=None, **kwargs)`

The actual post-run path used by `farmsim.main` → `run_sim.py` in
practice: re-parses clargs if not given, then delegates to
`prompt_postprocessing()` (`utils/prompt.py`), which interactively asks
(via `input()`) whether to save data and/or show plots — see below.

## CLI arguments

**Source:** `farms_sim/utils/parse_args.py` (73 lines)

### `sim_parse_args()`

```python
def sim_parse_args() -> Namespace:
    parser = sim_argument_parser()
    args, _ = parser.parse_known_args()
    return args
```

Uses `parse_known_args()`, not `parse_args()` — **unrecognized arguments
are silently ignored** rather than causing an error. This matters in
practice: the `run_sim.py` example distributed with experiments patches
`sys.argv` before calling `main()` to fix a `--experiment-config` (hyphen)
vs `--experiment_config` (underscore) typo, relying on `parse_known_args`
to not choke on any other stray args in the meantime.

### Actual argument list

| Argument | Type | Default | Description |
|---|---|---|---|
| `--simulator` | str, choices `MUJOCO`/`PYBULLET` | `'MUJOCO'` | Simulator backend (values are upper-case) |
| `--experiment_config` | str | `None` | Path to the experiment YAML (required — asserted non-empty in `setup_from_clargs`) |
| `--profile` | str | `''` | If non-empty, save a cProfile-style profile to this filename |
| `--log_path` | str | `''` | Folder to log simulation data to |
| `--test_configs` | flag | `False` | Round-trip options through save/load (see the `NameError` note above) |
| `--prompt` | flag | `False` | Prompt interactively at the end of the run (save data? show plots?) |
| `--verify_save` | flag | `False` | After saving, reload the saved file to check it deserializes |

!!! danger "There is no `--plot` or `--video` CLI flag, and no `./simulations` default"
    A previous version of this page claimed `sim_parse_args()` returns
    `plot` (default `False`) and `video` (default `''`) attributes, and
    that `--log_path` defaults to `./simulations`. None of this is in the
    source: `log_path` defaults to `''` (current working directory
    behaviour is then up to whatever consumes it), and there is no
    `plot`/`video` argument at all — plotting and video are controlled
    programmatically via `simulation_post(..., plot=, video=)` or
    interactively via the `--prompt` flow (`utils/prompt.py`), never via a
    dedicated CLI switch.

## Interactive post-processing prompt

**Source:** `farms_sim/utils/prompt.py` (87 lines)

`prompt_postprocessing(sim, query=True, log_path='', verify=False,
extension='pdf', simulator=Simulator.MUJOCO, animat_data_loader=AnimatData)`
is what actually runs when `--prompt` is passed (or when called directly,
`query` defaults `True`):

1. Asks `Save data [y/N]:` via `input()` (skipped, defaulting to save-if-`log_path`,
   when `query=False`).
2. Asks `Show plots [y/N]:` similarly.
3. Reads the current iteration count — **note the branch**:
   `sim.iteration` for PyBullet vs. `sim.task.iteration` for MuJoCo (the
   MuJoCo `Simulation` object itself has no top-level `.iteration`; it
   lives on the `ExperimentTask`, see `experiment-task-internals.md`).
4. Calls `sim.postprocess(iteration=..., log_path=..., plot=...)`.
5. If data was saved and `--verify_save` was passed, reloads it via
   `data_loader.from_file(os.path.join(log_path, 'simulation.hdf5'))` as a
   sanity check.
6. If MuJoCo and data was saved, also calls
   `sim.save_mjcf_xml(os.path.join(log_path, 'sim_mjcf.xml'))` — the
   resolved MJCF (after all SDF→MJCF conversion, unit scaling, etc. from
   `farms_mujoco/simulation/mjcf.py`) is written out for inspection.
7. If plots were shown, optionally prompts to save each open matplotlib
   figure (`fig.canvas.get_window_title()` used as filename — spaces
   replaced with underscores) and/or show "connectivity plots" (asked
   about but not otherwise implemented differently in this function).

`strtobool()` is reimplemented locally in this file (accepts
`y/yes/t/true/on/1` and `n/no/f/false/off/0`, case-insensitive) rather than
imported from `distutils.util`, which was removed in Python 3.12 — this is
a deliberate compatibility shim, not an oversight.

## `run_sim.py` pattern

Every experiment directory (e.g. `experiments/zbot_bout_glide/`) has its
own `run_sim.py`. The **verified** real-world version does this:

```python
import sys
import os

# Explicitly add the experiment directory to sys.path *before* importing
# farms_sim, so that local modules (e.g. controller.zbot_controller) are
# importable.
current_dir = os.path.abspath(os.path.dirname(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from farms_sim._bootstrap import main

# Work around a --experiment-config / --experiment_config typo.
for i, arg in enumerate(sys.argv):
    if arg == '--experiment-config':
        sys.argv[i] = '--experiment_config'

if __name__ == '__main__':
    sys.exit(main())
```

Key points, all directly contradicting the previous draft of this page:

- `main` is called with **no arguments**: `main()`, not `main(__file__)`.
- The `sys.path` manipulation is done **in `run_sim.py` itself**, using
  `run_sim.py`'s own `__file__` — `_bootstrap.main` never sees a path and
  has no code to resolve one.
- The `--experiment-config`/`--experiment_config` hyphen fix-up is a
  per-experiment convention living in `run_sim.py`, not something
  `farms_sim` does centrally. If you write a new experiment's
  `run_sim.py` from scratch, this patch is not automatic — copy it in, or
  just always use the underscore form and skip it.
- `sys.exit(main())` — `main()`'s return value (implicitly `None`, since
  neither `_bootstrap.main` nor `farmsim.main` returns anything) is passed
  to `sys.exit`, which treats `None` as success (`exit code 0`).
