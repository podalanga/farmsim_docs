# Trace a Simulation Step

This tutorial follows the execution path from the YAML configuration files to a
single MuJoCo physics step. It shows where your own code (controllers and
extensions) plugs in.

## The entry point

Simulations start from the `run_sim.py` script of an experiment folder:

```python
# experiments/zbot_bout_glide/run_sim.py (simplified)
import os
import sys

current_dir = os.path.abspath(os.path.dirname(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from farms_sim._bootstrap import main

if __name__ == '__main__':
    sys.exit(main())
```

The script adds the experiment folder to `sys.path`, so that local modules such
as `controller.zbot_controller` can be imported, and calls
`farms_sim._bootstrap.main()`. The `farmsim` console command installed by
`farms_sim` calls the same function.

`_bootstrap.main()` takes no arguments. On macOS it re-executes the process
under `mjpython` when available (the MuJoCo viewer requires it there). It then
calls `farms_sim.farmsim.profile_simulation()`.

## Phase 1: Argument parsing

`profile_simulation()` parses the command line with
`farms_sim.utils.parse_args.sim_parse_args()` and runs
`farms_sim.farmsim.main()` through `farms_core.utils.profile.profile()`, which
only profiles when `--profile <file>` is given.

The options are listed in the [CLI reference](../reference/env/cli.md), which
is generated from the parser. The only one needed is `--experiment_config`.

## Phase 2: Loading options

`main()` calls `farms_sim.simulation.setup_from_clargs()`, which loads the
experiment file:

```python
experiment_options = ExperimentOptions.load(clargs.experiment_config)
```

The experiment file lists the other files and the Python classes (loaders)
used to read them:

```yaml
# experiment_config.yaml
simulation: simulation_config.yaml
animats:
  - animat_config.yaml
arenas:
  - arena_config.yaml
loaders:
  simulation_options: farms_core.simulation.options.SimulationOptions
  animats_options:
    - farms_amphibious.model.options.AmphibiousOptions
  arenas_options:
    - farms_amphibious.model.options.AmphibiousArenaOptions
  experiment_data: farms_core.experiment.data.ExperimentData
  animats_data:
    - farms_core.model.data.AnimatData
```

`ExperimentOptions.load()` (`farms_core/experiment/options.py`) first builds
an `ExperimentLoadOptions` from `loaders:`. Then, for `simulation` and for
every entry of `animats` and `arenas` that is a filename, it imports the
corresponding loader class (`farms_core.extensions.extensions.import_item`)
and calls its `.load(filename)`. `animats` and `arenas` must have as many
entries as `loaders.animats_options` and `loaders.arenas_options`.

!!! note "Extensions use a different mechanism"
    Extensions, listed in the `extensions:` lists of the simulation and animat
    files, each have their own `loader:` (a dotted class path) and `config:`
    (a free-form dictionary passed to the class's `from_options()`). See
    [Options and YAML Design](../explanation/options-yaml-design.md).

## Phase 3: Creating experiment data

```python
experiment_data = ExperimentData.from_options(experiment_options)
```

`ExperimentData` pre-allocates NumPy arrays for all sensors, for
`runtime.buffer_size` iterations. It holds one `AnimatData` per animat,
created with the classes listed in `loaders.animats_data`. Each `AnimatData`
contains a `SensorsData` (links, joints, contacts, xfrc, muscles, adhesions,
visuals and rays).

## Phase 4: Simulation setup

`farms_sim.simulation.run_simulation()` calls `simulation_setup()`, which, for
MuJoCo, builds a `farms_mujoco.simulation.simulation.Simulation` with
`Simulation.from_experiment()`:

1. **MJCF construction**: `setup_mjcf_xml()` converts the animat and arena SDF
   files into a MuJoCo model, applying the options (morphology, sensors,
   actuators). See [MJCF Builder Internals](../internals/mjcf-builder-internals.md).
2. **Task creation**: an `ExperimentTask` (a dm_control `Task`) is created. It
   instantiates every extension with its `from_options()` class method: first
   those of `simulation.extensions`, then those of each animat's
   `extensions`, in order.
3. **Environment creation**: a dm_control `Environment` wraps the task and the
   physics.

!!! note "Controllers are extensions"
    A controller only runs if it is listed in the animat's `extensions:`
    (for example `loader: controller.zbot_controller.ZbotCPGController`).
    The `control.controller_loader` option is still parsed but is not used to
    create controllers.

## Phase 5: The simulation loop

`sim.run()` opens the viewer, or runs headless when `runtime.headless` is
true, and repeatedly calls `env.step()`.

Time is split in three levels, set in `simulation_config.yaml`:

- An **iteration** (`physics.timestep`) is the logging and control period.
  Sensors are stored once per iteration, and `runtime.n_iterations` iterations
  are simulated.
- Each iteration is made of `physics.cb_sub_steps` **environment steps**.
- Each environment step advances MuJoCo by `physics.num_sub_steps` physics
  steps of `timestep / (cb_sub_steps*num_sub_steps)`.

dm_control calls the task methods below around every environment step.

### `ExperimentTask.initialize_episode()` (once, at time 0)

- Builds the name to index maps of links, joints, actuators and sensors.
- Creates the sensor maps between MuJoCo and `AnimatData`.
- Collects the controllers (extensions that are `AnimatController`
  instances) and maps their joints to MuJoCo actuators.
- Resets the physics to keyframe 0.
- Calls `initialize_episode(task, physics)` on every extension.

### `ExperimentTask.before_step()` (every environment step)

```python
# Simplified from farms_mujoco/simulation/task.py
full_step = first environment step of an iteration
if full_step or any extension runs on substeps:
    self.update_sensors(physics, links_only=not full_step)
for extension in self.extensions:
    if full_step or extension.substep:
        extension.before_step(task=self, action=action, physics=physics)
        if isinstance(extension, AnimatController):
            # Write controller outputs to MuJoCo, for the control types
            # the controller has joints (or muscles) for
            positions = extension.positions(iteration, time, timestep)
            velocities = extension.velocities(iteration, time, timestep)
            torques = extension.torques(iteration, time, timestep)  # + spring/damping
            excitations = extension.excitations(iteration, time, timestep)
```

A controller's `before_step()` advances its internal dynamics (for example
the CPG integration), then `positions()`, `velocities()`, `torques()` (and
the spring and damping references) or `excitations()` (muscles) give the
targets written to `physics.data.ctrl` and the model.

Extensions created with `substep=True`, such as the `SwimmingExtension`,
run on every environment step, so the fluid forces follow the body between
iterations.

### `ExperimentTask.after_step()` (every environment step)

At the end of each iteration, the iteration counter is incremented and
`after_step(task, physics)` is called on every extension. Logging extensions
use `end_episode()` instead: `ExperimentLogger` writes the HDF5 file when the
episode ends.

## Where your code plugs in

| What you want to do | Where to plug in |
|---|---|
| Add per-iteration behaviour | `AnimatExtension.before_step()` / `after_step()` |
| Add per-substep behaviour | an extension created with `substep=True` |
| Control joints | `AnimatController.positions()` / `velocities()` / `torques()` |
| Read sensor data | `AnimatData.sensors` (for example `task.data.animats[i].sensors`) |
| Apply an external force | `before_step()`, writing `physics.data.xfrc_applied` |
| Save data at the end | `end_episode()` |

## Next steps

- [Write a Custom Controller](custom-controller.md): implement your own
  `AnimatController`
- [Write an AnimatExtension](../how-to/write-extension.md): add custom
  per-step behaviour
- [Extension and Controller Design](../explanation/extension-design.md): the
  lifecycle in depth
