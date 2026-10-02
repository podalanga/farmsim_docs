# Troubleshooting

Problems by symptom. If yours is not here, search the
[issues](https://github.com/podalanga/farmsim_docs/issues) or open one with
the full error message, your platform and the package refs.

## Installation

**`ModuleNotFoundError: No module named 'tomllib'` when running the installer**
: Python is older than 3.11. Create the virtual environment with Python
  3.11 or 3.12 (`python3.12 -m venv .venv`).

**The installer stops at a `git fetch` with a network error**
: The installer retries each fetch three times. If it still fails, check
  your connection or proxy and run it again: packages already cloned are
  only updated.

**Cython compilation fails: `error: Microsoft Visual C++ 14.0 or greater is required`**
: Windows without the compiler on `PATH`. Install Visual Studio Build Tools
  with **Desktop development with C++**, and run the installer from a
  **Developer PowerShell**.

**Cython compilation fails: `gcc: command not found` or `Python.h: No such file`**
: Linux without the build tools or Python headers:
  `sudo apt-get install build-essential python3-dev`.

**`ModuleNotFoundError: No module named 'farms_core'` (or another package)**
: The virtual environment is not activated, or the installer failed
  partway. Activate the environment and rerun the installer; its output
  names the failing package.

**`ModuleNotFoundError` for a third-party module at run time**
: A package dependency is missing from its declared requirements. Install
  it with `pip install <module>` and report it, so it can be added.

## Viewer and rendering

**The viewer does not open in Docker on Linux**
: Run `xhost +local:docker` on the host (once per session) and check that
  `DISPLAY` is set in the container (`echo $DISPLAY`).

**The viewer does not open in Docker on Windows**
: Check, in order: VcXsrv is running; **Native OpenGL** is unchecked in
  XLaunch; **Disable access control** is checked; `DISPLAY` is
  `host.docker.internal:0` in the container.

**`GLFW error` or `Failed to initialize OpenGL` on a server**
: There is no display. Run headless (`runtime.headless: true`), or set
  `MUJOCO_GL=egl` (GPU) or `MUJOCO_GL=osmesa` (CPU) for offscreen rendering.

**The viewer is very slow**
: MuJoCo is using software rendering. In Docker, enable the GPU (see
  [GPU and rendering](../get-started/installation.md#gpu-and-rendering)).
  Speed up playback with `+`, or run headless.

**macOS: the viewer crashes or does not open**
: The MuJoCo viewer needs `mjpython` on macOS. `farms_sim` switches to it
  automatically when it is installed (it comes with the `mujoco` package);
  check that `mjpython` is on your `PATH`.

## Model loading

**`ValueError: Duplicated identifier '...' in namespace <geom>` (or `<body>`)**
: Two elements have the same name. Give every link, joint, collision and
  visual a unique name in the SDF, and do not name an arena body `water`
  (see [Bring your own robot](../how-to/own-robot.md)).

**`AttributeError: 'NoneType' object has no attribute 'text'` while reading an SDF**
: The SDF parser expects an element that is missing. Run with the latest
  `farms_core` (joint `<dynamics>` blocks with partial fields used to fail)
  and check the element named in the traceback.

**`Unknown kwargs` or `KeyError` when loading an animat file**
: The YAML file has a key the option class does not know, or misses one it
  requires: usually a file written for another version. Regenerate it, or
  compare it with the [configuration reference](../reference/env/configuration-reference.md).

**The robot falls through the ground**
: The arena is not where you think: check `ground_height` in the arena
  file (it offsets the arena model) and that the arena links have
  collision geometry.

## Simulation behaviour

**The simulation explodes (bodies fly away, `nan` values)**
: The physics is unstable. Lower `physics.timestep` (for example to
  0.0005), lower the motor gains, or check the link masses and inertias
  (very light links with large forces are unstable). With the ellipsoid
  fluid model, prefer `added_mass: implicit` to `explicit`.

**The robot vibrates at rest**
: Motor gains too high for the link inertia, or interpenetrating
  collision geoms. Lower the gains in the animat file, and check the
  geometry in `Output/simulation_mjcf.xml`.

**The robot sinks, or floats too high**
: Buoyancy uses the submerged volume of the collision geoms, not the
  `density` option. Compare the robot's mass with the water mass its
  collision geoms displace; overlapping geoms are counted twice with
  `cob_method: exact` unless `cob_overlap: scale` (or use
  `cob_method: lut`, which uses their union). See
  [Swimming and buoyancy](../tutorials/swimming.md#step-2-buoyancy).

**The robot does not move forward in water**
: Check that the links have `fluid_interaction: true`, that `water.drag`
  is true, and that the drag is anisotropic (larger sideways than along
  the body), see [drag coefficients](../tutorials/swimming.md#step-4-drag-coefficients).

**A snake-like robot does not move on land**
: MuJoCo friction is isotropic, so a lateral undulation slides without
  propulsion. Add passive wheels or skids, as AmphiBot does.

**The simulation is slow**
: Run headless and check `runtime.show_progress`. Fluid forces cost a few
  microseconds per link; `cob_method: lut` makes buoyancy O(1) per link.
  Run parallel simulations as separate processes.

## Docker

**Warning `CoB LUT cache ... is not writable`**
: The lookup tables could not be saved next to the script (read-only
  volume). They are saved in `~/.cache/farms_mujoco/cob_lut` or the
  temporary directory instead. Set `FARMS_COB_LUT_CACHE` to a writable,
  mounted directory to keep them between containers.

**Files in `Output/` belong to another user**
: The container user's UID differs from yours. Rebuild with your IDs:
  `docker compose build --build-arg UID=$(id -u) --build-arg GID=$(id -g)`.
