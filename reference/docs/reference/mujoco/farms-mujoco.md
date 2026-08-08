# farms_mujoco Reference

API reference for `farms_mujoco` — the MuJoCo physics engine integration layer.

## Module structure

```
farms_mujoco/
├── simulation/
│   ├── simulation.py    # MuJoCoSimulation class
│   ├── task.py          # ExperimentTask (dm_control Task)
│   ├── mjcf.py          # SDF → MJCF conversion
│   └── extensions.py    # MjcfSaver, CameraFollower, visualization extensions
├── swimming/
│   ├── extension.py     # SwimmingExtension (drag + buoyancy)
│   ├── buoyancy.py      # Buoyancy computation
│   └── ...
└── sensors/
    └── camera.py        # Camera sensor utilities
```

## MuJoCoSimulation

**Source:** `farms_mujoco/simulation/simulation.py`

```python
class Simulation:
    """MuJoCo simulation wrapper."""

    @classmethod
    def from_experiment(cls, experiment_options, experiment_data):
        """Create simulation from experiment configuration."""
```

### from_experiment() flow

1. **`setup_mjcf_xml()`** — builds MuJoCo XML from:
   - Animat SDF files + `AnimatOptions` (morphology, sensors)
   - Arena SDF file + `ArenaOptions` (water, ground)
   - Simulation options (timestep, gravity, extensions)
2. **Task creation** — creates `ExperimentTask` as a dm_control `Task`
3. **Environment creation** — wraps task and physics in
   `dm_control.rl.control.Environment`

### run()

```python
def run(self, headless=True, **kwargs):
    """Run the simulation loop."""
```

- **Interactive mode** (display available): uses `mujoco.viewer.launch_passive()`
  with keyboard callbacks (Space=pause, Q/Esc=quit)
- **Headless mode**: iterates with `tqdm` progress bar, calling
  `env.step(action=None)` for each iteration

## ExperimentTask

**Source:** `farms_mujoco/simulation/task.py`

Extends dm_control's `Task` class. This is the core per-step controller.

### initialize_episode()

```python
def initialize_episode(self, physics):
    """Called once at the start of an episode."""
```

- Builds name-to-index maps for joints, links, sensors
- Creates controller instances from `animat_options.control.controller_loader`
- Instantiates all extensions via their `from_options()` class methods
- Initializes sensor data arrays and maps

### before_step()

```python
def before_step(self, physics):
    """Called before each physics step."""
```

Execution order:

1. `self.update_sensors(iteration, physics)` — reads MuJoCo state into
   `AnimatData.sensors` arrays
2. For each extension (in YAML order): `extension.before_step(self, None, physics)`
3. Collect controller outputs:
   - `controller.positions(iteration, time, timestep)`
   - `controller.velocities(iteration, time, timestep)`
   - `controller.torques(iteration, time, timestep)`
4. Map outputs to `physics.data.ctrl`

### after_step()

```python
def after_step(self, physics):
    """Called after each physics step."""
```

1. Increments `self.iteration`
2. For each extension: `extension.after_step(self, None, physics)`

### update_sensors()

Reads from MuJoCo physics state into pre-allocated arrays:

| Sensor | MuJoCo source |
|--------|---------------|
| Joint positions | `physics.data.qpos` |
| Joint velocities | `physics.data.qvel` |
| Joint torques | `physics.data.actuator_force` |
| Link positions | `physics.data.xpos` |
| Link orientations | `physics.data.xquat` |
| Link velocities | `physics.data.cvel` |
| Contact forces | `physics.data.collision` |
| External forces | `physics.data.xfrc_applied` |

### Key attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `iteration` | int | Current iteration counter |
| `buffer_size` | int | Data buffer size |
| `units` | `SimulationUnitScaling` | Unit conversion factors |
| `extensions` | list | All registered extensions |
| `animat_controllers` | list | Controller instances |

## MJCF builder

**Source:** `farms_mujoco/simulation/mjcf.py`

Handles conversion from SDF (Simulation Description Format) to MJCF (MuJoCo XML
format). Key operations:

- Parse SDF model files for links, joints, visuals, collisions
- Create MJCF body/joint/geom elements
- Apply link options (density, drag coefficients, friction, collisions)
- Apply joint options (limits, stiffness, damping, spring reference)
- Add sensors (contact, force, position) as MJCF sensor elements
- Merge arena SDF (ground plane, water visual)

## SwimmingExtension

**Source:** `farms_mujoco/swimming/extension.py`

```python
class SwimmingExtension(AnimatExtension):
    """Computes hydrodynamic forces and applies them to MuJoCo."""
```

### from_options()

```python
@classmethod
def from_options(cls, config, experiment_options, animat_i,
                animat_data, animat_options):
    """Reads water options from experiment_options.arenas[0]."""
```

No `config` dict is needed — water parameters are read from the arena's
`WaterOptions`:

- `water.drag` — drag coefficients per link
- `water.buoyancy` — enable/disable buoyancy
- `water.height` — water surface height
- `water.density` — fluid density
- `water.velocity` — fluid velocity

### before_step()

Calls `SwimmingHandler.step()` which:

1. Computes drag forces from link linear/angular velocities and drag coefficients
2. Computes buoyancy forces from link volume, water height, and fluid density
3. Applies forces to `physics.data.xfrc_applied`

## Built-in extensions

**Source:** `farms_mujoco/simulation/extensions.py`

### MjcfSaver

```python
class MjcfSaver(AnimatExtension):
    """Saves the compiled MJCF XML model."""
```

Config: `path` (str) — output directory.

### CameraFollower

```python
class CameraFollower(AnimatExtension):
    """Camera follows an animat during interactive simulation."""
```

Config: `animat_id`, `azimuth`, `distance`, `elevation`, `angular_velocity`.

### Visualization extensions

All extend `AnimatExtension` and render markers/trails in the MuJoCo viewer:

| Class | Config | Visual |
|-------|--------|--------|
| `CoMViewer` | `animat_id` | Center of mass sphere |
| `TrailCoMViewer` | `animat_id` | CoM trajectory trail |
| `TrailLinkViewer` | `animat_id`, `link_name` | Link trajectory trail |
| `ArrowViewer` | `animat_id` | Force/momentum arrows |
