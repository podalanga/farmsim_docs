# Installation

FarmSim is four Python packages with Cython extensions: `farms_core`,
`farms_mujoco`, `farms_sim` and `farms_amphibious`. Install them in a
Docker container (recommended, nothing to set up on the host besides
Docker) or in a Python virtual environment.

Both methods use the [farmsim_docs](https://github.com/podalanga/farmsim_docs)
repository, which holds the example robot and the installer
`reference/tools/install_farms.py`. The installer clones each package from
its public repository at the ref listed in `reference/farms-packages.yaml`,
installs its dependencies, then installs it in editable mode, so you can
edit the FARMS sources afterwards.

!!! note "Source files"
    - `docker/Dockerfile`, `docker/docker-compose.yml`
    - `reference/tools/install_farms.py`, `reference/farms-packages.yaml`
    - `examples/amphibot/`: the example used to check the installation

## Docker

### Prerequisites

| Requirement | Linux | Windows | macOS |
|---|---|---|---|
| [Docker Desktop](https://www.docker.com/products/docker-desktop/) or Docker Engine with BuildKit | Required | Required (WSL 2 backend) | Required |
| X server for the viewer | Built in | [VcXsrv](https://sourceforge.net/projects/vcxsrv/) | [XQuartz](https://www.xquartz.org/) |
| [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) | Optional, for an NVIDIA GPU | Optional | Not available |

Headless runs (`runtime.headless: true` in `simulation_config.yaml`) need
no X server.

### Step 1: Clone the repository

```bash
git clone https://github.com/podalanga/farmsim_docs.git
cd farmsim_docs
```

### Step 2: Allow the container to open windows

=== "Linux"

    ```bash
    xhost +local:docker
    ```

    Run once per login session.

=== "Windows"

    Start **VcXsrv** with **XLaunch** and these settings:

    1. Display settings: **Multiple windows**, display number `0`
    2. Client startup: **Start no client**
    3. Extra settings: **Native OpenGL** unchecked, **Disable access
       control** checked
    4. Click **Finish**

    The compose file defaults `DISPLAY` to `host.docker.internal:0`, which
    reaches VcXsrv on the host. Check that WSL 2 is the Docker backend
    with `wsl --status` (`Default Version: 2`).

=== "macOS"

    Install XQuartz, enable **Allow connections from network clients** in
    its settings, restart it, then run `xhost +localhost` and start the
    container with `DISPLAY=host.docker.internal:0`. Rendering is software
    only, so prefer headless runs for long simulations.

### Step 3: Build and start the container

```bash
cd docker
docker compose up --build -d
```

The build installs the system libraries, creates a virtual environment and
runs the installer. The first build takes a few minutes. To build other
versions of the packages, pass refs:

```bash
docker compose build --build-arg FARMS_REFS="farms_mujoco=my-branch"
```

### Step 4: Enter the container and run the example

```bash
docker exec -it farmsim bash
cd examples/amphibot
python run_sim.py --experiment_config experiment_config.yaml
```

The `examples/` folder of the repository is mounted in the container, so
your edits and the `Output/` folders stay on the host.

### GPU and rendering

The compose file maps `/dev/dri` for integrated GPUs (Intel, AMD). For an
NVIDIA GPU, install the NVIDIA Container Toolkit and uncomment the
`runtime: nvidia` lines of `docker/docker-compose.yml`. Without a GPU,
MuJoCo falls back to Mesa software rendering, which works everywhere but
makes the interactive viewer slower. For offscreen rendering without a
display, set `MUJOCO_GL=egl` (GPU) or `MUJOCO_GL=osmesa` (CPU).

## Virtual environment

### Prerequisites

| Requirement | Notes |
|---|---|
| Python 3.11 or newer | The installer uses `tomllib` (Python 3.11+). 3.12 is the tested version |
| Git | To clone the packages |
| C compiler | Linux: `sudo apt-get install build-essential`. macOS: `xcode-select --install`. Windows: [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/) with **Desktop development with C++**, then run the commands from a **Developer PowerShell** |
| OpenGL | Linux: `sudo apt-get install libgl1 libegl1` for the viewer and offscreen rendering |

### Step 1: Clone and create a virtual environment

```bash
git clone https://github.com/podalanga/farmsim_docs.git
cd farmsim_docs
python3 -m venv .venv
```

=== "Linux and macOS"

    ```bash
    source .venv/bin/activate
    ```

=== "Windows (PowerShell)"

    ```powershell
    .venv\Scripts\Activate.ps1
    ```

Always use a virtual environment: installing FARMS into the system Python
is not supported.

### Step 2: Install the packages

```bash
pip install pyyaml
python reference/tools/install_farms.py
```

The installer clones the packages into `farms-src/`, in dependency order,
and for each one installs its dependencies, then the package itself with
`pip install --no-build-isolation -e`. This compiles the Cython
extensions. Options:

- `--dest DIR`: where to clone the packages (default: `farms-src/`).
- `FARMS_REFS="farms_mujoco=my-branch,farms_core=abc123"`: install other
  branches, tags or commits than those of `reference/farms-packages.yaml`.

The clones are shallow (latest commit only). To work on a package, fetch
its history with `git -C farms-src/farms_mujoco fetch --unshallow`.

### Step 3: Run the example

```bash
cd examples/amphibot
python run_sim.py --experiment_config experiment_config.yaml
```

## Verify the installation

The four packages import:

```bash
python -c "import farms_core, farms_mujoco, farms_sim, farms_amphibious; print('OK')"
```

The example runs: the MuJoCo viewer opens on AmphiBot, which crawls toward
the pool, and `examples/amphibot/Output/` holds `simulation.hdf5` at the
end. On a machine without a display, set `runtime.headless: true` in
`simulation_config.yaml` first. [Your first simulation](../tutorials/first-simulation.md)
explains what you see.

## Update

=== "Docker"

    ```bash
    git pull
    cd docker
    docker compose up --build -d
    ```

=== "Virtual environment"

    ```bash
    git pull
    python reference/tools/install_farms.py
    ```

    The installer fetches the refs again and rebuilds the packages.

## Uninstall

- Docker: `docker compose down` in `docker/`, then
  `docker image rm farmsim:latest`.
- Virtual environment: delete the `.venv/` and `farms-src/` folders.

## Next steps

- [Supported platforms](platforms.md)
- [Your first simulation](../tutorials/first-simulation.md)
- If something fails: [Troubleshooting](../help/troubleshooting.md#installation)
