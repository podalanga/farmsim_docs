# Core Interfaces — Base class contracts and abstract modules

This document outlines the purpose, override requirements, and implicit contracts for key base classes and interfaces across the FARMS framework modules.

!!! note "Source Files"
    - `farms_core/farms_core/simulation/extensions.py` — `TaskExtension` base class
    - `farms_core/farms_core/model/extensions.py` — `AnimatExtension` base class
    - `farms_core/farms_core/model/control.py` — `AnimatController` base class
    - `farms_amphibious/farms_amphibious/control/network.py` — `AnimatNetwork` base class
    - `farms_amphibious/farms_amphibious/control/drive.py` — `DescendingDrive` and `PotentialMap`

## 1. `farms_core` Base Classes

### `TaskExtension` (`farms_core.simulation.extensions`)
* **Purpose**: Extends the core simulation environment/task. Defines hooks for logging, overriding behaviors, or injecting arbitrary logic during the physics loop.
* **Virtual Methods**: `from_options`, `initialize_episode`, `before_step`, `after_step`, `action_spec`, `step_spec`, `get_observation`, `get_reward`, `get_termination`, `observation_spec`, `end_episode`.
* **Adding a Model**: Inherit `TaskExtension`, implement `@classmethod from_options(cls, config, experiment_options)` to instantiate the extension, and implement whichever callback hooks are necessary.
* **Memory Rules / Implicit Contracts**: 
  * `from_options` acts as an enforced factory method.
  * Lifecycle calls have a strict guaranteed order: `initialize_episode` (called at iteration 0) -> `before_step` -> [physics] -> `after_step` -> `end_episode`.
  * Memory ownership relies on the generic `Task` providing simulation `data` structures. 

### `AnimatExtension` (`farms_core.model.extensions`)
* **Purpose**: Extends `TaskExtension` specifically to inject animat-related logic (e.g., sensor processing, swimming hydrodynamics) into the simulation.
* **Virtual Methods**: Inherits all from `TaskExtension`. Defines its own version of `from_options`.
* **Adding a Model**: Implement `@classmethod from_options(cls, config, experiment_options, animat_i, animat_data, animat_options)`.
* **Memory Rules / Implicit Contracts**: 
  * Assumes the extension is tied to a specific animat. 
  * It has access to, and frequently mutates, `animat_data` during the `before_step` or `after_step` hooks. 

### `AnimatController` (`farms_core.model.control`)
* **Purpose**: Base class for defining controllers for animats, managing various control signal streams (positions, velocities, torques).
* **Virtual Methods**: `positions`, `velocities`, `torques`, `springrefs`, `springcoefs`, `dampingcoefs`, `excitations` (alongside inherited lifecycle hooks).
* **Adding a Controller**: Inherit `AnimatController`, instantiate via `from_options`, and override signal generation methods (e.g., `positions`, `velocities`) to return a dictionary mapping joint/muscle names to control floats for each requested control type.
* **Memory Rules / Implicit Contracts**: 
  * The `joints_names` parameter passed to the constructor must strictly match the `ControlType` enum length (7 lists of joint names).
  * Control methods must correctly accept `(iteration, time, timestep)` and return dictionaries fully populated with keys identical to the joint names configured for that specific control type.

## 2. `farms_mujoco` Physics Backend Interfaces

### `ExperimentTask` (`farms_mujoco.simulation.task`) & `Simulation`
* **Purpose**: Acts as the adapter between `farms_core` abstraction definitions and the `dm_control` (MuJoCo) physics backend. `ExperimentTask` inherits from `dm_control.rl.control.Task`. 
* **Virtual Methods**: Overrides dm_control's lifecycle methods (`initialize_episode`, `before_step`, `after_step`), delegating them directly to registered `TaskExtension`s.
* **Adding a Force Model**: Users typically do not override the physics backend interface directly. To add a new force model (e.g., fluid dynamics), a user creates an `AnimatExtension` (like `SwimmingExtension`) and calculates/applies forces in the `before_step` hook.
* **Memory Rules / Implicit Contracts**:
  * Applying forces relies on directly mutating underlying MuJoCo C-struct arrays (e.g., `physics.data.xfrc_applied`) at specific body indices retrieved via `physics.model.body(name).id` (cached during `initialize_episode`).
  * Memory allocation is fully owned by MuJoCo; extensions must mutate arrays in place and ensure correct coordinate transforms (e.g., local to global frames).

## 3. `farms_amphibious` Control and Network Base Classes

### `AnimatNetwork` (`farms_amphibious.control.network`)
* **Purpose**: Defines the abstract base for Central Pattern Generators (CPGs) or oscillator networks that compute neural state variables.
* **Virtual Methods**: `step(iteration, time, timestep, **kwargs)`
* **Adding a Network**: Subclass `AnimatNetwork` and implement the `step` method to update the network state forward in time.
* **Memory Rules / Implicit Contracts**: Implementations (like `NetworkODE`) are expected to directly mutate pre-allocated NumPy arrays (`self.data.state.array`) in place using the `iteration` index.

### `DescendingDrive` (`farms_amphibious.control.drive`)
* **Purpose**: Manages top-down drive commands routing from higher brain centers to spinal networks.
* **Virtual Methods**: `step(iteration, time, timestep)`
* **Adding a Drive**: Inherit `DescendingDrive` and implement `step` to determine the drive logic for the current iteration.
* **Memory Rules / Implicit Contracts**: Implementations must use helper mutators (`self.set_left_drives` / `set_right_drives`) to populate `self.drives.array` in place. This strongly depends on pre-configured indices (e.g., `spine_left_indices`).

### `PotentialMap` (`farms_amphibious.control.drive`)
* **Purpose**: Defines mathematical potential fields (directions) for goal-directed locomotion drives.
* **Virtual Methods**: `heading(pos)`
* **Adding a Potential Map**: Implement `heading(pos)` to return the desired orientation angle for any 2D Cartesian point.
