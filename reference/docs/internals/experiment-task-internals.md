# ExperimentTask Internals

This page documents the `ExperimentTask` class (`farms_mujoco/simulation/task.py`) in detail, covering the full simulation lifecycle: initialization, sensor updates, control dispatch, and step execution. This is the central orchestrator of every FARMS MuJoCo simulation.

## Source files covered

| File | Purpose |
|---|---|
| `farms_mujoco/simulation/task.py` | `ExperimentTask`, `duration2nit` |
| `farms_mujoco/simulation/physics.py` | `get_sensor_maps`, `get_physics2data_maps`, `physics2data` (called by task) |
| `farms_mujoco/simulation/mjcf.py` | `get_prefix` (used for multi-animat naming) |
| `farms_core/simulation/extensions.py` | `TaskExtension` base class |
| `farms_core/model/control.py` | `AnimatController`, `ControlType` |

## Call graph / entry points

```
Simulation.run() / Simulation.iterator()
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
| `base_links` | list[str] | Yes | n/a | Base link names (marked as TODO: Unused) |
| `n_iterations` | int | Yes | n/a | Total simulation iterations |
| `timestep` | float | Yes | n/a | Iteration duration (`physics.timestep`) [s] |
| `data` | ExperimentData | No | None | Pre-allocated experiment data |
| `viewer` | Any | No | None | Viewer object |
| `mjcf` | Any | No | None | MJCF model |
| `experiment_options` | ExperimentOptions | Yes | n/a | Full experiment configuration |
| `restart` | bool | No | True | Whether simulation can restart |
| `extensions` | list[TaskExtension] | No | [] | Additional extensions beyond those from options |
| `hfield` | dict | No | None | Heightfield data and asset |
| `units` | SimulationUnits | No | SimulationUnits() | Unit scaling |
| `buffer_size` | int | No | 1 | Rolling buffer size for data arrays |
| `substeps` | int | No | 1 | Environment steps per iteration (`physics.cb_sub_steps`) |

**Strict kwargs**: `assert not kwargs, kwargs` rejects unknown parameters.

### Key attributes set in constructor

| Attribute | Description |
|---|---|
| `self.iteration` | Current iteration (incremented at the end of each iteration) |
| `self.sim_iteration` | Environment steps done (incremented every environment step) |
| `self.physics_iterations` | `n_iterations*cb_sub_steps*num_sub_steps`: total MuJoCo steps |
| `self.physics_timestep` | `timestep/(cb_sub_steps*num_sub_steps)`: the MuJoCo timestep [s] |
| `self.extensions` | All extensions (simulation, animats, then extra ones) |
| `self.maps` | Per-animat mapping dicts (sensors, ctrl, etc.) |
| `self.buffer_size` | Number of iterations held by the data arrays (at least 1) |
| `self.cb_sub_steps` | Environment steps per iteration |
| `self.substeps_links` | Whether any extension runs on substeps (then links are updated at every environment step) |
| `self.initialized` | Whether `initialize_episode` has run |

### Substep mechanism

An iteration lasts `timestep` and is made of `cb_sub_steps` environment
steps (`env.step()`), each of `num_sub_steps` MuJoCo steps (the
environment's `n_sub_steps`):

- `before_step()` runs at every environment step. It updates the sensors
  on the first step of an iteration (all of them) and, when
  `substeps_links`, on the other steps (links only). Extensions run on the
  first step, or on every step when created with `substep=True`.
- `after_step()` increments `sim_iteration` at every environment step, and
  `iteration` (plus the extensions' `after_step()`) at the end of each
  iteration.

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

**Step 1: Re-initialization check**

```python
if self.initialized:
    pylog.warning('Simulation was already initialized, ...')
    self.iteration = 0
    self.sim_iteration = 0
    return
```

If already initialized, only resets iteration counters. For full re-initialization, set `self.initialized = False` before calling.

**Step 2: Restart check**

```python
if self._restart:
    assert self._app is not None, 'Simulation can not be restarted without application interface'
```

If restart is enabled, an application interface must be set.

**Step 3: Viewer setup**

```python
if viewer is not None:
    scn = viewer.user_scn
    scn.ngeom = 0
```

Clears user scene geometry.

**Step 4: Link masses**

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

**Step 5: Iteration reset**

```python
self.iteration = 0
self.sim_iteration = 0
```

**Step 6: Heightfield setup**

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

**Step 7: Maps, data, sensors**

```python
self.initialize_maps(physics)
if self.data is None:
    self.data = self.initialize_data()
self.initialize_sensors(physics)
```

- `initialize_maps`: Reads MuJoCo named array row names (xpos, qpos, xfrc, geoms, muscles).
- `initialize_data`: If no pre-allocated data, creates `AnimatData` from sensor names.
- `initialize_sensors`: Calls `get_sensor_maps(physics)` and `get_physics2data_maps(physics, ...)` for each animat.

**Step 8: Control initialization**

```python
self._controllers = [
    extension for extension in self.extensions
    if isinstance(extension, AnimatController)
]
if self._controllers:
    self.initialize_control(physics)
```

Filters extensions that are `AnimatController` instances, then calls `initialize_control()`.

**Step 9: Keyframe reset**

```python
physics.reset(keyframe_id=0)
```

Resets the physics state to the first keyframe (initial pose).

**Step 10: Camera setup**

If viewer or app is present, positions the camera to look at the animat.

**Step 11: Extension initialization**

```python
for extension in self.extensions:
    extension.initialize_episode(task=self, physics=physics)
```

Each extension gets its own `initialize_episode` call. This is where controllers, drives, and other extensions set up their internal state.

**Step 12: MuJoCo muscle callbacks**

```python
if rt_muscle:
    set_callback("mjcb_act_gain", rt_muscle.mjcb_muscle_gain)
    set_callback("mjcb_act_bias", rt_muscle.mjcb_muscle_bias)
```

Sets MuJoCo callbacks for muscle actuator computation (if `farms_muscle` is installed).

**Step 13: Mark initialized**

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

`index = self.iteration % self.buffer_size`: when `buffer_size` is smaller than `n_iterations`, the data wraps around, which keeps only the last `buffer_size` iterations in memory. `Simulation` passes `runtime.buffer_size`, which defaults to `n_iterations` (the task's own default, 1, is only used when it is created directly).

### What gets recorded

| Data | Source | Description |
|---|---|---|
| `times[index]` | `physics.time()` | Current simulation time |
| `ncon[index]` | `physics.data.ncon` | Number of contacts |
| `niter[index]` | `physics.data.solver_niter[0]` | Solver iterations |
| `energy[index, :]` | `physics.data.energy` | Energy stats |

Then `physics2data()` copies all link, joint, contact, and muscle data for each animat.

## `before_step(action, physics)`

The main control dispatch function, called by dm_control before every environment step.

### Walkthrough

**Step 1: Auto-initialization**

```python
if physics.time() / self.units.seconds < 1e-6 * self.physics_timestep:
    self.initialize_episode(physics, self.viewer)
```

If this is the very first step (time ≈ 0), initialize the episode.

**Step 2: Full step vs substep**

```python
full_step = (
    not self.sim_iteration  # First iteration
    or not (self.sim_iteration + 1) % self.cb_sub_steps  # Last substep of a control step
)
```

With the counter updates of `after_step()` (`iteration` is incremented
when `(sim_iteration + 1) % cb_sub_steps == 0` after incrementing
`sim_iteration`), `full_step` is true on the first environment step of each
iteration. Iteration 0 is special: it has a single environment step, then
every following iteration has `cb_sub_steps` steps. When `cb_sub_steps = 1`,
every step is a full step.

**Step 3: Iteration check**

```python
if self.n_iterations > 0:
    assert self.iteration < self.n_iterations
```

Safety check: don't exceed the planned iteration count.

**Step 4: Sensor update**

```python
if full_step or self.substeps_links:
    self.update_sensors(physics=physics, links_only=not full_step)
```

Sensors are updated on full steps. If any extension needs per-substep link data (`substeps_links = True`), links are also updated on substeps with `links_only=True`.

**Step 5: Extension before_step**

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

### Force limiting for non-position joints

```python
for mtr_opts in animat_options.control.motors:
    if 'position' not in mtr_opts.control_types:
        for act_type in ('pos', 'vel'):
            if act_type in jntname2actid[prefix+jnt_name]:
                physics.named.model.actuator_forcelimited[...] = True
                physics.named.model.actuator_forcerange[...] = [0, 0]
```

For joints that are not position-controlled, the position and velocity actuators are force-limited to [0, 0] (effectively disabled). This ensures that only the torque actuator drives these joints.

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

Increments `sim_iteration` at every environment step, and `iteration` at the end of each iteration, when the extensions' `after_step()` is also called. On the last iteration, the simulation is marked complete.

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

To add a new control type (for example a hypothetical `ACCELERATION` member of `ControlType`):

1. Add the enum value to `ControlType` in `farms_core/model/control.py`.
2. In `ExperimentTask.before_step()`, add a new dispatch block.
3. In `ExperimentTask.initialize_control()`, add the actuator map for the new type.
4. Implement the corresponding method in the controller (e.g., `controller.springrefs()`).

## Troubleshooting

### Extension ordering

Extensions are called in order: simulation extensions first, then animat extensions. If an animat extension depends on a simulation extension being initialized first, the ordering must be correct. The YAML determines the order within each category.

### Buffer overflow

When `buffer_size = 1`, data is overwritten every step. If an extension tries to read data from a previous iteration that was already overwritten, it will get the current iteration's data instead. This is usually fine but can cause subtle bugs in data-dependent control logic.

### Sensor map mismatches

If the SDF model is modified (links/joints added or removed) but the FARMS data containers are not re-allocated, `physics2data()` will fail with array shape mismatches. Always call `ExperimentData.from_options()` after model changes.

### Actuator name not found

`initialize_control()` asserts that every joint's actuator name exists in `ctrl_names`. If the MJCF builder doesn't create an actuator for a joint (e.g., due to a naming mismatch), the assertion fails with a descriptive error.

### `farms_muscle` not installed

If `farms_muscle` is not installed, the try/except at import catches the `ImportError` and logs a warning. The simulation will run but without rigid tendon muscle callbacks. This may cause incorrect muscle dynamics.

## Caveats

- `before_step` is not called once per physics step. When `substeps > 1`, it's called once per control step, but the physics engine runs multiple substeps. Extensions with `substep=True` get called on every substep.

- `iteration` and `sim_iteration` are different. `iteration` increments on full control steps, `sim_iteration` on every physics substep. Use `task.iteration % task.buffer_size` for data array indexing.

- The `base_links` parameter is unused. The constructor accepts it but the TODO comment says "Unused?". Do not rely on it.

- `initialize_episode` only runs once. Subsequent calls just reset iteration counters. Set `self.initialized = False` for full re-initialization.

- Actuator naming is strict. The format `actuator_{type}_{prefix}{joint}` is enforced by assertions. Any deviation will cause `initialize_control` to fail.

- Force limiting is applied to non-position joints. If a joint is torque-controlled, its position and velocity actuators are force-limited to zero. This is a design choice to prevent conflicting control inputs.
