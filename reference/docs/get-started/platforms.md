# Supported platforms

What FarmSim runs on, and how well each combination is tested. "Tested"
means the documentation CI or the maintainers run it regularly;
"expected to work" means it is used but not checked automatically.

## Operating systems and Python

| Platform | Python | Status | Notes |
|---|---|---|---|
| Ubuntu 22.04 and 24.04, native | 3.12 | Tested | The documentation CI installs and imports the packages on Ubuntu with Python 3.12 |
| Docker image (`python:3.12-slim`) on Linux | 3.12 | Tested | See [Installation](installation.md#docker) |
| Docker on Windows 10/11 (WSL 2) | 3.12 | Expected to work | Viewer through VcXsrv |
| Windows 10/11, native | 3.11, 3.12 | Expected to work | Needs Visual Studio Build Tools |
| macOS (Apple silicon and Intel), native | 3.11, 3.12 | Expected to work | The MuJoCo viewer needs `mjpython`, which `farms_sim` uses automatically when it is installed |
| Python 3.13 | 3.13 | Untested | Depends on the availability of wheels for MuJoCo and dm_control |
| Python 3.10 and older | | Not supported | The installer needs `tomllib` |

## Rendering

| Backend | `MUJOCO_GL` | Use |
|---|---|---|
| GLFW (default) | unset | Interactive viewer with a display |
| EGL | `egl` | Offscreen rendering on a GPU, for videos and figures on servers |
| OSMesa | `osmesa` | Offscreen rendering on the CPU, when there is no GPU (slow) |

Headless simulations (`runtime.headless: true`) do not render at all and
run on any machine.

## Hardware

- One CPU core per simulation: the physics and the fluid model are single
  threaded, so run parallel simulations as separate processes.
- A GPU only matters for the viewer and for rendering videos.
- Memory: the logged data grows with `runtime.buffer_size` times the number
  of sensors. A 10 s AmphiBot run at 1 ms per step writes a few tens of MB.

## Next steps

- [Installation](installation.md)
- [Troubleshooting](../help/troubleshooting.md)
