# Bring your own robot

How to simulate your own robot in FarmSim: prepare its SDF model, generate
its animat configuration, and set up an experiment folder. The AmphiBot
example (`examples/amphibot/`) is the template.

## 1. Prepare the SDF model

FarmSim reads SDF (`farms_core.io.sdf.ModelSDF`). It uses the links (pose,
inertial, collision and visual geometry, material colours) and the joints
(type, parent, child, axis, limits, dynamics). Simulator-specific blocks,
such as Gazebo `<plugin>` elements, are ignored.

Check these points, which differ from Gazebo:

| Requirement | Why |
|---|---|
| Unique names for every link, joint, collision and visual of the model | FarmSim converts the model to MJCF, where names share one namespace. Prefix geometry names with the link name (`head_collision`) |
| In the arena SDF, the model name differs from its link names | Each becomes a MuJoCo body |
| Do not name an arena body `water` | FarmSim creates a `water` body for the water surface |
| Collision geometry for every link that touches the ground or the water | Contacts and buoyancy use the collision geoms; visual shapes are ignored by the physics |
| No anisotropic friction (`mu2`, `fdir1`) | MuJoCo's friction is isotropic per geom. Use wheels or skids for snake-like crawling, as AmphiBot does |

Meshes (`.stl`, `.obj`, `.dae`) are supported. Keep their paths relative to
the SDF file.

## 2. Generate the animat configuration

For a robot with a spine (and optionally legs), `farms_amphibious` builds
the whole animat configuration, CPG included, from a few parameters with
`AmphibiousOptions.from_options()`. Copy
`examples/amphibot/generate_config.py` and adapt:

```python
options = AmphibiousOptions.from_options({
    'sdf_path': 'models/my_robot.sdf',
    'n_joints_body': 10,          # Spine joints
    'n_dof_legs': 0,              # Joints per leg
    'n_legs': 0,
    'n_joints_passive': 0,        # Passive joints (wheels, springs)
    'links_names': [...],         # Body links, then leg links, then others
    'joints_names': [...],        # Body joints, then legs, then passive
    'spawn_position': [0, 0, 0.1],
    'drag_coefficients': [...],   # One [[x, y, z], [rx, ry, rz]] per link
    'drives_init': [4.0, 4.0],
    'body_freq_gain': 1.57,       # [rad/s] per drive unit
    'body_osc_gain': 0.15,
    'body_walk_amplitude': 1.0,
    'weight_osc_body_side': 30.0,
    'weight_osc_body_down': 30.0,
    'motor_gains': [...],         # One [kp, kd, 0] per joint
    'extensions': [...],          # Controller and SwimmingExtension
})
options.save('animat_config.yaml')
```

The order of `links_names` and `joints_names` matters: the body comes
first, from head to tail, so that the convention can map oscillators to
joints (see [Naming and indexing convention](../internals/amphibious-convention.md)).
The full list of parameters is in the source of
`farms_amphibious.model.options`, and every generated key is documented in
the [configuration reference](../reference/env/configuration-reference.md).

For a robot without a spine, or with your own controller only, write the
animat file directly with the keys of the
[YAML configuration schema](../reference/env/yaml-schema.md), or start
from a generated file and edit it.

## 3. Set up the experiment folder

Copy the AmphiBot folder and replace the models:

```text
my_robot/
├── experiment_config.yaml    # Unchanged
├── simulation_config.yaml    # Adjust n_iterations, the timestep, the camera
├── arena_config.yaml         # Your arena SDF, the water height
├── animat_config.yaml        # Generated
├── generate_config.py
├── models/
└── run_sim.py                # Unchanged
```

## 4. Check the model

1. Run headless for a short time (`n_iterations: 1001`) and check that
   `Output/simulation_mjcf.xml` is written: the model converts.
2. Open it in MuJoCo's viewer, `python -m mujoco.viewer --mjcf Output/simulation_mjcf.xml`,
   and check the geometry, the joint axes (render the joints) and the
   masses.
3. Run with the viewer and a zero drive (`drives_init: [0, 0]`): the robot
   should rest without vibrating. If it vibrates, lower the motor gains or
   the timestep.
4. Increase the drive until it moves.

## See also

- [Understand the experiment files](../tutorials/experiment-files.md)
- [Write a custom controller](../tutorials/custom-controller.md)
- [Troubleshooting](../help/troubleshooting.md#model-loading)
