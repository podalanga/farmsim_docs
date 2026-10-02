# FAQ

## General

**What is the difference between FARMS and FarmSim?**
: FARMS (Framework for Animal and Robot Modeling and Simulation) is the
  whole framework. FarmSim is its simulation part, the four packages
  documented here: `farms_core`, `farms_mujoco`, `farms_sim` and
  `farms_amphibious`.

**Why MuJoCo and not Gazebo or PyBullet?**
: MuJoCo is fast, accurate for contacts and articulated bodies, and easy to
  drive from Python, which suits locomotion studies and parameter sweeps.
  `farms_core` keeps the options independent of the engine; the MuJoCo
  backend is the maintained one.

**Can I use my own robot?**
: Yes, from an SDF file. See [Bring your own robot](../how-to/own-robot.md).
  A URDF can be converted to SDF with Gazebo's `gz sdf -p robot.urdf`.

**Do I need to use the CPG?**
: No. The built-in CPG (`AmphibiousController`) is one controller; any
  `AnimatController` works, see
  [Write a custom controller](../tutorials/custom-controller.md).

## Running simulations

**How do I run many simulations in parallel?**
: Run separate processes, each with its own experiment folder or output
  path: each simulation uses one core. Set `runtime.headless: true` and
  write the results to different `log_path` folders.

**How do I record a video?**
: Replay the log offscreen: load `Output/simulation_mjcf.xml` in MuJoCo, set
  the robot's pose from the logged link and joint sensors at each frame,
  and render with `mujoco.Renderer`. `reference/tools/make_figures.py` in
  the docs repository does this for the figures of these pages and is a
  starting point. On a server, set `MUJOCO_GL=egl` or `osmesa`.

**Can I change parameters during a run?**
: Yes, from a controller or an extension: `before_step()` can change the
  drives, the targets or apply forces (`physics.data.xfrc_applied`). See
  [Write an AnimatExtension](../how-to/write-extension.md).

**Are the simulations deterministic?**
: For a given machine, package versions and configuration, MuJoCo steps
  are deterministic. The initial CPG phases are written in the animat file
  (`initial_phase`); the generator draws them with a fixed seed.

## Units and conventions

**What units does FarmSim use?**
: SI by default: metres, seconds, kilograms, radians. The `units` block of
  `simulation_config.yaml` scales them internally for numerical
  conditioning; inputs and outputs stay in SI.

**What is the axis convention?**
: z up, gravity along -z. Each link's drag coefficients are in the link
  frame. For AmphiBot, x points along the body toward the head.

**What is the difference between `n_iterations` and `buffer_size`?**
: `n_iterations` is the number of steps simulated; `buffer_size` the
  number of steps kept in memory and saved. Keep them equal to log the
  whole run.

## Water

**Where is the water?**
: Everything below `water.height` in the arena file is water, everywhere in
  the world. The water SDF is only a visual. To limit the water to a pool,
  shape the ground (as the AmphiBot arena does) so the robot can only be
  below the surface inside the pool.

**Does the `density` link option affect buoyancy?**
: Not with the default `cob_method: exact` or `lut`: buoyancy comes from
  the submerged volume of the collision geoms. `density` is used by the
  legacy `cob_method: ramp`.

## Docs

**Why do the tutorials use AmphiBot and not a real research robot?**
: AmphiBot is small, open (MIT), built from primitive shapes, and exercises
  land and water locomotion. Real projects are documented as
  [case studies](../projects/index.md).

**The docs are wrong or unclear. How do I fix them?**
: Use the edit button at the top of each page, or open an issue. The
  [contributing guide](contributing.md) describes the docs checks.
