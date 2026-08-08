# ExperimentTask Internals

This page documents the `ExperimentTask` class (`farms_mujoco/simulation/task.py`, 624 lines) in detail, covering the full simulation lifecycle: initialization, sensor updates, control dispatch, and step execution. This is the central orchestrator of every FARMS MuJoCo simulation.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_mujoco/simulation/task.py` | 624 | `ExperimentTask`, `duration2nit` |
| `farms_mujoco/simulation/physics.py` | 561 | `get_sensor_maps`, `get_physics2data_maps`, `physics2data` (called by task) |
| `farms_mujoco/simulation/mjcf.py` | 1732 | `get_prefix` (used for multi-animat naming) |
| `farms_core/simulation/extensions.py` | 192 | `TaskExtension` base class |
| `farms_core/model/control.py` | — | `AnimatController`, `ControlType` |

## Call graph / entry points

```
Simulation.run() / Simulation.interactive()
  └─ env = dm_control.Environment(task, physics, ...)
       └─ env.step(action)
            ├─ task.before_step(action, physics)     [Control dispatch]
            │    ├─ task.update_sensors(physics)      [Read physics state → data arrays]
            │    ├─ extension.before_step() for each extension
            │    ├─ task.step_joints_control_position()  [Position commands → ctrl]
            │    ├─ task.step_joints_control_velocity()  [Velocity commands → ctrl]
            │    ├─ task.step_joints_control_torque()    [Torque + spring + damping → ctrl/model]
            │    └─ muscles excitations → ctrl
            ├─ physics.step()                          [MuJoCo integration]
            └─ task.after_step(physics)               [Iteration increment, extension calls]
```

## Class hierarchy

```
dm_control.rl.control.Task
  └─ ExperimentTask
```

`ExperimentTask` extends dm_control's `Task` base class. The `Task` interface requires implementing `initialize_episode`, `before_step`, `after_step`, and `action_spec`. dm_control's `Environment.step()` calls these methods in a specific order.

## `duration2nit(duration, timestep)`

```python
def duration2nit(duration: float, timestep: float) -> int:
    """Number of iterations from duration"""
    return int(duration / timestep)
```

Simple utility: converts a duration in seconds to an iteration count. Uses `int()` truncation, not rounding.

## Constructor: `__init__`

```python
def __init__(
    self,
    base_links: list[str],
    n_iterations: int,
    timestep: float,
    **kwargs,
):
```

### Parameters

| Parameter | Type | Required | Default | Description |
|---|---|---|---|---|
| `base_links` | list[str] | Yes | — | Base link names (marked as TODO: Unused) |
| `n_iterations` | int | Yes | — | Total simulation iterations |
| `timestep` | float | Yes | — | Physics timestep [s] |
| `data` | ExperimentData | No | None | Pre-allocated experiment data |
| `viewer` | Any | No | None | Viewer object |
| `mjcf` | Any | No | None | MJCF model |
| `experiment_options` | ExperimentOptions | Yes | — | Full experiment configuration |
| `restart` | bool | No | True | Whether simulation can restart |
| `extensions` | list[TaskExtension] | No | [] | Additional extensions beyond those from options |
| `hfield` | dict | No | None | Heightfield data and asset |
| `units` | SimulationUnits | No | SimulationUnits() | Unit scaling |
| `buffer_size` | int | No | 1 | Rolling buffer size for data arrays |
| `substeps` | int | No | 1 | Physics substeps per control step |

**Strict kwargs**: `assert not kwargs, kwargs` rejects unknown parameters.

### Key attributes set in constructor

| Attribute | Description |
|---|---|
| `self.iteration` | Current control iteration (incremented every full step) |
| `self.sim_iteration` | Current physics substep (incremented every substep) |
| `self.sim_iterations` | `n_iterations * cb_sub_steps` (total physics substeps) |
| `self.sim_timestep` | `timestep / cb_sub_steps` (physics substep size) |
| `self.extensions` | All extensions (sim + animat + extra) |
| `self.maps` | Per-animat mapping dicts (sensors, ctrl, etc.) |
| `self.buffer_size` | Rolling buffer size (max 1) |
| `self.cb_sub_steps` | Number of physics substeps per control step |
| `self.substeps_links` | Whether any extension needs per-substep link updates |
| `self.initialized` | Whether `initialize_episode` has run |

### Substep mechanism

When `substeps > 1`, the physics engine runs multiple substeps per control step:
- `sim_timestep = timestep / substeps` (smaller physics steps for stability)
- `sim_iterations = n_iterations * substeps` (more total physics steps)
- `before_step` is called once per control step (every `substeps` physics steps)
- `after_step` increments `sim_iteration` every physics step, `iteration` only on full steps

## `extract_extensions(experiment_options, experiment_data)` (staticmethod)

```python
@staticmethod
def extract_extensions(experiment_options, experiment_data):
    # Simulation extensions
    simulation_extentions_loaders = [
        import_item(extension['loader'])
        for extension in experiment_options.simulation.extensions
    ]
    sim_extensions = [
        loader.from_options(config=extension['config'], experiment_options=experiment_options)
        for loader, extension in zip(simulation_extentions_loaders, experiment_options.simulation.extensions)
    ]

    # Animat extensions
    animat_extensions = [
        import_item(extension.loader).from_options(
            config=extension.config, experiment_options=experiment_options,
            animat_i=animat_i, animat_data=animat_data, animat_options=animat_options,
        )
        for animat_i, (animat_data, animat_options) in enumerate(zip(
            experiment_data.animats, experiment_options.animats))
        for extension in animat_options.extensions
    ]

    return sim_extensions + animat_extensions
```

### How extensions are loaded

1. **Simulation extensions**: Each entry in `experiment_options.simulation.extensions` is a dict with `'loader'` (a dotted Python path string) and `'config'`. `import_item()` resolves the path to a class, which is then called with `.from_options()`.

2. **Animat extensions**: Each animat's `extensions` list contains `ExtensionOptions` with `.loader`, `.config`. For each animat, all its extensions are loaded with the animat's data and options.

3. **Ordering**: Simulation extensions come first, then animat extensions (in animat order). This order matters for `before_step` and `after_step` calls.

## `initialize_episode(physics, viewer=None)`

```python
def initialize_episode(self, physics: Physics, viewer=None):
```

### Walkthrough

**Step 1: Re-initialization check** (lines 146–154)

```python
if self.initialized:
    pylog.warning('Simulation was already initialized, ...')
    self.iteration = 0
    self.sim_iteration = 0
    return
```

If already initialized, only resets iteration counters. For full re-initialization, set `self.initialized = False` before calling.

**Step 2: Restart check** (lines 155–158)

```python
if self._restart:
    assert self._app is not None, 'Simulation can not be restarted without application interface'
```

If restart is enabled, an application interface must be set.

**Step 3: Viewer setup** (lines 161–163)

```python
if viewer is not None:
    scn = viewer.user_scn
    scn.ngeom = 0
```

Clears user scene geometry.

**Step 4: Link masses** (lines 166–174)

```python
links_row = physics.named.model.body_mass.axes.row
for animat_i, animat in enumerate(self.data.animats):
    prefix = get_prefix(animat_i)
    animat.sensors.links.masses = np.array([
        physics.model.body_mass[links_row.convert_key_item(prefix + link_name)]
        for link_name in animat.sensors.links.names
    ], dtype=float) / self.units.kilograms
```

Reads body masses from the MuJoCo model for each animat's links. Uses `get_prefix(animat_i)` to handle multi-animat naming (each animat gets a prefix like `animat_0_`).

**Step 5: Iteration reset** (lines 177–178)

```python
self.iteration = 0
self.sim_iteration = 0
```

**Step 6: Heightfield setup** (lines 181–199)

```python
if self._extras['hfield'] is not None:
    data = self._extras['hfield']['data']
    hfield = self._extras['hfield']['asset']
    nrow = physics.bind(hfield).nrow
    ncol = physics.bind(hfield).ncol
    idx0 = physics.bind(hfield).adr
    size = nrow * ncol
    physics.model.hfield_data[idx0:idx0+size] = 2 * (data.flatten() - 0.5)
```

Loads heightfield terrain data into the MuJoCo model. The data is normalized from [0, 1] to [-1, 1] via `2 * (data - 0.5)`.

**Step 7: Maps, data, sensors** (lines 201–206)

```python
self.initialize_maps(physics)
if self.data is None:
    self.data = self.initialize_data()
self.initialize_sensors(physics)
```

- `initialize_maps`: Reads MuJoCo named array row names (xpos, qpos, xfrc, geoms, muscles).
- `initialize_data`: If no pre-allocated data, creates `AnimatData` from sensor names.
- `initialize_sensors`: Calls `get_sensor_maps(physics)` and `get_physics2data_maps(physics, ...)` for each animat.

**Step 8: Control initialization** (lines 208–215)

```python
self._controllers = [
    extension for extension in self.extensions
    if isinstance(extension, AnimatController)
]
if self._controllers:
    self.initialize_control(physics)
```

Filters extensions that are `AnimatController` instances, then calls `initialize_control()`.

**Step 9: Keyframe reset** (line 218)

```python
physics.reset(keyframe_id=0)
```

Resets the physics state to the first keyframe (initial pose).

**Step 10: Camera setup** (lines 220–236)

If viewer or app is present, positions the camera to look at the animat.

**Step 11: Extension initialization** (lines 238–239)

```python
for extension in self.extensions:
    extension.initialize_episode(task=self, physics=physics)
```

Each extension gets its own `initialize_episode` call. This is where controllers, drives, and other extensions set up their internal state.

**Step 12: MuJoCo muscle callbacks** (lines 242–244)

```python
if rt_muscle:
    set_callback("mjcb_act_gain", rt_muscle.mjcb_muscle_gain)
    set_callback("mjcb_act_bias", rt_muscle.mjcb_muscle_bias)
```

Sets MuJoCo callbacks for muscle actuator computation (if `farms_muscle` is installed).

**Step 13: Mark initialized** (line 247)

```python
self.initialized = True
```

## `update_sensors(physics, links_only=False)`

```python
def update_sensors(self, physics: Physics, links_only=False):
    index = self.iteration % self.buffer_size
    self.data.times[index] = physics.time() / self.units.seconds
    sim_data = self.data.simulation
    sim_data.ncon[index] = physics.data.ncon
    sim_data.niter[index] = physics.data.solver_niter[0]
    sim_data.energy[index, :] = physics.data.energy
    for animat_i, animat_data in enumerate(self.data.animats):
        physics2data(
            physics=physics, iteration=index, data=animat_data,
            maps=self.maps[animat_i], units=self.units, links_only=links_only,
        )
```

### Rolling buffer mechanism

`index = self.iteration % self.buffer_size` — when `buffer_size > 1`, data wraps around. This is used for memory-constrained scenarios where you only need the last N steps of data. When `buffer_size = 1` (default), only the current step's data is stored.

### What gets recorded

| Data | Source | Description |
|---|---|---|
| `times[index]` | `physics.time()` | Current simulation time |
| `ncon[index]` | `physics.data.ncon` | Number of contacts |
| `niter[index]` | `physics.data.solver_niter[0]` | Solver iterations |
| `energy[index, :]` | `physics.data.energy` | Energy stats |

Then `physics2data()` copies all link, joint, contact, and muscle data for each animat.

## `before_step(action, physics)`

The main control dispatch function, called once per control step (or substep).

### Walkthrough

**Step 1: Auto-initialization** (lines 269–274)

```python
if physics.time() / self.units.seconds < 1e-6 * self.sim_timestep:
    self.initialize_episode(physics, self.viewer)
```

If this is the very first step (time ≈ 0), initialize the episode.

**Step 2: Full step vs substep** (lines 276–278)

```python
full_step = (
    not self.sim_iteration  # First iteration
    or not (self.sim_iteration + 1) % self.cb_sub_steps  # Last substep of a control step
)
```

`full_step` is True on the first substep and the last substep of each control step. When `cb_sub_steps = 1`, every step is a full step.

**Step 3: Iteration check** (lines 282–283)

```python
if self.n_iterations > 0:
    assert self.iteration < self.n_iterations
```

Safety check: don't exceed the planned iteration count.

**Step 4: Sensor update** (lines 286–287)

```python
if full_step or self.substeps_links:
    self.update_sensors(physics=physics, links_only=not full_step)
```

Sensors are updated on full steps. If any extension needs per-substep link data (`substeps_links = True`), links are also updated on substeps with `links_only=True`.

**Step 5: Extension before_step** (lines 290–321)

```python
for extension in self.extensions:
    if full_step or extension.substep:
        extension.before_step(task=self, action=action, physics=physics)
        if isinstance(extension, AnimatController):
            # Position control
            if extension.joints_names[ControlType.POSITION]:
                self.step_joints_control_position(controller=extension, physics=physics, time=current_time)
            # Velocity control
            if extension.joints_names[ControlType.VELOCITY]:
                self.step_joints_control_velocity(controller=extension, physics=physics, time=current_time)
            # Torque control
            if extension.joints_names[ControlType.TORQUE]:
                self.step_joints_control_torque(controller=extension, physics=physics, time=current_time)
            # Muscle excitations
            if extension.muscles_names:
                muscles_excitations = extension.excitations(iteration=index, time=current_time, timestep=self.timestep)
                muscle_indices = self.maps[extension.animat_i]['ctrl']['mus']
                physics.data.ctrl[muscle_indices] = muscles_excitations
```

Extensions are called in order. For `AnimatController` extensions, the three control types are dispatched:
1. **Position**: Calls `controller.positions()` → writes to `physics.data.ctrl[pos_map]`
2. **Velocity**: Calls `controller.velocities()` → writes to `physics.data.ctrl[vel_map]`
3. **Torque**: Calls `controller.torques()`, `controller.springrefs()`, `controller.springcoefs()`, `controller.dampingcoefs()` → writes to `physics.data.ctrl`, `physics.model.qpos_spring`, `physics.model.jnt_stiffness`, `physics.model.dof_damping`

## `step_joints_control_torque()` walkthrough

```python
def step_joints_control_torque(self, controller, physics, time):
    index = self.iteration % self.buffer_size
    # ...

    # 1. Joint torques → physics.data.ctrl
    joints_torques = controller.torques(iteration=index, time=time, timestep=self.timestep)
    for joint, value in joints_torques.items():
        ctrl[ctrl_trq_map[prefix+joint]] = value * torques

    # 2. Spring references → physics.model.qpos_spring
    springrefs = controller.springrefs(iteration=index, time=time, timestep=self.timestep)
    for joint, value in springrefs.items():
        qpos_spring[springref_map[prefix+joint]] = value  # Radians, no unit conversion

    # 3. Spring coefficients → physics.model.jnt_stiffness
    springcoefs = controller.springcoefs(iteration=index, time=time, timestep=self.timestep)
    for joint, value in springcoefs.items():
        jnt_stiffness[jnt_stiffness_map[prefix+joint]] = value * ang_stiffness

    # 4. Damping coefficients → physics.model.dof_damping
    dampingcoefs = controller.dampingcoefs(iteration=index, time=time, timestep=self.timestep)
    for joint, value in dampingcoefs.items():
        dof_damping[dof_damping_map[prefix+joint]] = value * ang_damping
```

This is the most complex control dispatch. The Ekeberg muscle model writes FOUR things per joint: torque command, spring reference, spring coefficient, and damping coefficient. These are written to different MuJoCo model arrays.

## `initialize_control(physics)`

Builds control maps that map joint names to actuator indices in `physics.data.ctrl`.

### Actuator naming convention

```
actuator_{control_type}_{prefix}{joint_name}
```

For example: `actuator_position_animat_0_joint_body_3`, `actuator_torque_animat_0_joint_leg_0_L_1`.

### Control maps built

| Map key | Description |
|---|---|
| `ctrl['pos']` | Joint name → position actuator index |
| `ctrl['vel']` | Joint name → velocity actuator index |
| `ctrl['trq']` | Joint name → torque actuator index |
| `ctrl['mus']` | List of muscle actuator indices |
| `ctrl['springref']` | Joint name → qpos_spring index |
| `ctrl['jnt_stiffness']` | Joint name → jnt_stiffness index |
| `ctrl['dof_damping']` | Joint name → dof_damping index |

### Force limiting for non-position joints (lines 442–456)

```python
for mtr_opts in animat_options.control.motors:
    if 'position' not in mtr_opts.control_types:
        for act_type in ('pos', 'vel'):
            if act_type in jntname2actid[prefix+jnt_name]:
                physics.named.model.actuator_forcelimited[...] = True
                physics.named.model.actuator_forcerange[...] = [0, 0]
```

For joints that are NOT position-controlled, the position and velocity actuators are force-limited to [0, 0] (effectively disabled). This ensures that only the torque actuator drives these joints.

## `after_step(physics)`

```python
def after_step(self, physics: Physics):
    self.sim_iteration += 1
    fullstep = not (self.sim_iteration + 1) % self.cb_sub_steps
    if fullstep:
        self.iteration += 1
    if self.n_iterations > 0:
        assert self.iteration <= self.n_iterations
    if (self.n_iterations > 0) and (self.iteration == self.n_iterations):
        pylog.info('Simulation complete')
        # Close app or allow restart
    if fullstep:
        for extension in self.extensions:
            extension.after_step(task=self, physics=physics)
```

Increments `sim_iteration` every physics step, `iteration` only on full steps. Extension `after_step` is called only on full steps. On the last iteration, the simulation is marked complete.

## How to integrate: adding a new extension

```python
from farms_core.simulation.extensions import TaskExtension

class MyExtension(TaskExtension):
    def __init__(self, config, experiment_options):
        super().__init__()
        self.config = config

    @classmethod
    def from_options(cls, config, experiment_options):
        return cls(config=config, experiment_options=experiment_options)

    def initialize_episode(self, task, physics):
        """Called once during episode initialization."""
        pass

    def before_step(self, task, action, physics):
        """Called before each physics step (or substep if self.substep=True)."""
        pass

    def after_step(self, task, physics):
        """Called after each full physics step."""
        pass
```

Register it in the experiment YAML:

```yaml
simulation:
  extensions:
    - loader: my_package.my_extension.MyExtension
      config:
        my_param: 42
```

## How to integrate: adding a new control type

To add a new control type (e.g., `ControlType.SPRING`):

1. Add the enum value to `ControlType` in `farms_core/model/control.py`.
2. In `ExperimentTask.before_step()`, add a new dispatch block.
3. In `ExperimentTask.initialize_control()`, add the actuator map for the new type.
4. Implement the corresponding method in the controller (e.g., `controller.springrefs()`).

## Common failure modes

### 1. Extension ordering

Extensions are called in order: simulation extensions first, then animat extensions. If an animat extension depends on a simulation extension being initialized first, the ordering must be correct. The YAML determines the order within each category.

### 2. Buffer overflow

When `buffer_size = 1`, data is overwritten every step. If an extension tries to read data from a previous iteration that was already overwritten, it will get the current iteration's data instead. This is usually fine but can cause subtle bugs in data-dependent control logic.

### 3. Sensor map mismatches

If the SDF model is modified (links/joints added or removed) but the FARMS data containers are not re-allocated, `physics2data()` will fail with array shape mismatches. Always call `ExperimentData.from_options()` after model changes.

### 4. Actuator name not found

`initialize_control()` asserts that every joint's actuator name exists in `ctrl_names`. If the MJCF builder doesn't create an actuator for a joint (e.g., due to a naming mismatch), the assertion fails with a descriptive error.

### 5. `farms_muscle` not installed

If `farms_muscle` is not installed, the try/except at lines 23–27 catches the `ImportError` and logs a warning. The simulation will run but without rigid tendon muscle callbacks. This may cause incorrect muscle dynamics.

## What NOT to assume

1. **`before_step` is NOT called once per physics step.** When `substeps > 1`, it's called once per control step, but the physics engine runs multiple substeps. Extensions with `substep=True` get called on every substep.

2. **`iteration` and `sim_iteration` are different.** `iteration` increments on full control steps, `sim_iteration` on every physics substep. Use `task.iteration % task.buffer_size` for data array indexing.

3. **The `base_links` parameter is unused.** The constructor accepts it but the TODO comment says "Unused?". Do not rely on it.

4. **`initialize_episode` only runs once.** Subsequent calls just reset iteration counters. Set `self.initialized = False` for full re-initialization.

5. **Actuator naming is strict.** The format `actuator_{type}_{prefix}{joint}` is enforced by assertions. Any deviation will cause `initialize_control` to fail.

6. **Force limiting is applied to non-position joints.** If a joint is torque-controlled, its position and velocity actuators are force-limited to zero. This is a design choice to prevent conflicting control inputs.
