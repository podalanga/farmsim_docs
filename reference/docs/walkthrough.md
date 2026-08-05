# Codebase Walkthrough: The FARMS Lifecycle

This guide walks through the exact order of operations in the FARMS framework, from script invocation through the physics loop and to output writing. If you're a new contributor looking to understand how the system is wired together, start here.

## 1. Invocation and Parsing

The standard entry point to run a FARMS simulation is `farms_sim.farmsim` (`python -m farms_sim.farmsim` or via custom runner scripts).

**Relevant Files:**
- `farms_sim/farms_sim/farmsim.py`
- `farms_sim/farms_sim/simulation.py`

When `main()` is executed:
1. `setup_from_clargs()` parses command-line arguments (looking particularly for an `--experiment_config` path).
2. It uses `ExperimentOptions.load(config_path)` to deserialize the YAML configuration file into a rich tree of `farms_core` Option classes. This config contains the arena settings, the animat options, the chosen simulation backend, and a list of extensions (controllers/hydrodynamics).
3. The script then creates the memory structures that will store the simulation states via `ExperimentData.from_options()`.

## 2. Physics Backend Setup

With the options loaded, control is handed to `run_simulation()` which dispatches to the chosen physics backend (e.g., `farms_mujoco.Simulation`).

**Relevant Files:**
- `farms_mujoco/farms_mujoco/simulation/simulation.py`

The system invokes `Simulation.from_experiment()`, which does three critical things:
1. **MJCF Generation:** It dynamically generates a complete MuJoCo XML (MJCF) model by combining the arena and the animats.
2. **Task Creation:** It creates an `ExperimentTask`, a subclass of `dm_control`'s `Task`. This object manages the lifecycle of an episode.
3. **Environment Creation:** It compiles the MJCF into a `dm_control` `Physics` instance, wraps it with the `ExperimentTask` in a `dm_control` `Environment`, and prepares for execution.

## 3. Episode Initialization

Before the clock starts ticking, the physics environment must be initialized.

**Relevant Files:**
- `farms_mujoco/farms_mujoco/simulation/task.py` (see `initialize_episode`)

When `initialize_episode()` is called:
1. **Memory Maps:** It discovers the memory addresses of all joints, links, and actuators in the compiled MuJoCo model, storing these indices in `self.maps`. This is how abstract names like "tail_joint" are resolved to fast array indices during runtime.
2. **Extensions:** It instantiates the list of `TaskExtension` objects. For an amphibious animal, this typically includes:
   - `AmphibiousController` (the CPG neural network)
   - `SwimmingExtension` (the hydrodynamic force calculator)
3. Each extension has its own `initialize_episode()` called, allowing them to pre-allocate memory or initialize solvers.

## 4. The Simulation Loop

The `run()` method in `farms_mujoco`'s `Simulation` is a `while` loop that iterates until the target time is reached. Within this loop, the system calls `dm_control`'s `env.step()`, which in turn triggers:

### 4.1. Pre-Step (Sensor reading and Extension calculations)

**Relevant File:** `farms_mujoco/farms_mujoco/simulation/task.py` (see `before_step`)

1. **Update Sensors:** `update_sensors()` copies the current physical state (`physics.data.qpos`, `xpos`, etc.) into the `AnimatData.sensors` structures using the maps we built during initialization.
2. **Run Extensions:** The loop iterates over all configured extensions:
   - **Controllers:** Calls `extension.torques()` or `velocities()`. The CPG steps its Ordinary Differential Equations forward by one timestep and computes desired torques. The Task writes these directly into the MuJoCo control array (`physics.data.ctrl`).
   - **Physics Extensions:** For example, the `SwimmingExtension` calculates translational and rotational drag and buoyancy based on link velocities relative to the water, and injects these forces into `physics.data.xfrc_applied`. Added mass and lift are not implemented.

*(Danger Zone: The order these extensions run is derived from the YAML config. If the swimming extension ran before the controller updated the joints, it would calculate hydrodynamics using the old joint states!)*

### 4.2. Physics Step

MuJoCo takes the populated `physics.data.ctrl` and `physics.data.xfrc_applied` arrays, solves the contact constraints, and performs a numerical integration step forward in time.

### 4.3. Post-Step

The Task's `after_step()` is called. Extensions can perform cleanup, logging, or reward calculation here.

## 5. Output and Post-Processing

Once the target number of iterations is reached, the simulation loop exits.

If logging was enabled, the `ExperimentData` structure (which has been quietly buffering the simulation state in numpy arrays) is written to disk as an HDF5 file alongside a copy of the configuration YAMLs used. This produces a perfectly reproducible artifact of the run.
