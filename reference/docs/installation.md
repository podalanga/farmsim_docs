# Installation — Docker and native setup

FARMS requires SSH access to GitHub for cloning the repository and its submodules. Before proceeding, ensure your SSH key is configured - see [SSH Key Setup](ssh_setup.md) if you have not done this yet.

Two installation methods are available. **Docker is the recommended approach** - it handles all system dependencies, Cython compilation, and GPU configuration automatically.

---

## Method 1: Docker (Recommended)

Docker encapsulates the entire FARMS environment including system libraries, OpenGL drivers, Python packages, and Cython extensions. Python packages are installed automatically during the image build. No manual dependency management is required.

### Prerequisites

| Requirement | Linux | Windows |
|-------------|-------|---------|
| [Docker Desktop](https://www.docker.com/products/docker-desktop/) or Docker Engine | ✓ | ✓ |
| [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) | ✓ (for GPU) | ✓ (for GPU) |
| WSL 2 (Windows Subsystem for Linux) | - | ✓ |
| [VcXsrv](https://sourceforge.net/projects/vcxsrv/) X server | - | ✓ (for GUI viewer) |
| SSH key registered on GitHub | ✓ | ✓ |

---

### Step 1: Clone the repository

The Dockerfile uses SSH agent forwarding to fetch submodules during build. You must clone the repository via SSH (not HTTPS).

```bash
git clone git@github.com:farmsim/farms_zbot.git
cd farms_zbot
```

---

### Step 2 (Linux only): Allow X11 forwarding from Docker

```bash
xhost +local:docker
```

This grants the container permission to open windows on your display. Run this once per session.

---

### Step 2 (Windows only): Configure VcXsrv

MuJoCo's viewer requires an X display server on Windows. Install and launch **VcXsrv** using **XLaunch** with these exact settings:

1. **Display settings**: Multiple windows, Display number `0`
2. **Client startup**: Start no client
3. **Extra settings**:
   - ☑ Clipboard
   - ☑ Primary Selection
   - ☐ **Native OpenGL** - **must be unchecked**
   - ☑ **Disable access control** - **must be checked**
4. Click **Finish** - VcXsrv will start in the system tray

!!! warning
    If **Native OpenGL** is enabled or **Disable access control** is not checked, the MuJoCo viewer will fail to connect to the display. These two settings are mandatory.

**WSL 2 requirement**: Windows Docker with GPU passthrough requires WSL 2. Verify it is the active backend:

```powershell
wsl --status
# Must show: Default Version: 2
```

If not set: `wsl --set-default-version 2`

---

### Step 3: Start the SSH agent and add your key

The Docker build uses SSH agent forwarding to pull submodules from private GitHub repositories. Your SSH agent must be running with your key loaded **before** running the build.

**Linux / macOS:**
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519     # or id_rsa if using RSA
```

**Windows (PowerShell as Administrator, one-time setup):**
```powershell
Set-Service ssh-agent -StartupType Automatic
Start-Service ssh-agent
ssh-add $env:USERPROFILE\.ssh\id_ed25519
```

Verify the agent has the key loaded:
```bash
ssh-add -l
# Should list your key fingerprint
```

---

### Step 4: Build and start the container

From the repository root, run the appropriate compose file for your platform.

**Linux:**
```bash
docker compose -f docker_config/linux/docker-compose.yml up --build -d
```

**Windows (PowerShell):**
```powershell
docker compose -f docker_config/windows/docker-compose.yml up --build -d
```

The build will:
1. Install all system dependencies (build tools, OpenGL/GLVND, git-lfs)
2. Create a non-root user (`farmsuser`, UID 1000) with GPU group membership
3. Pull Git LFS objects and initialise all submodules via SSH forwarding
4. Create a Python virtual environment at `/home/farmsuser/venv`
5. Run `farms/setup_farms.py` - installs all four FARMS packages and their `requirements.txt` in editable mode using `uv`

Build time is typically 5–15 minutes on first run depending on network and hardware.

---

### Step 5: Enter the container

```bash
# For Linux:
docker exec -it zbot_farms_linux bash

# For Windows:
docker exec -it zbot_farms_windows bash
```

The container starts with `/app` as the working directory. The `experiments/` directory is bind-mounted from the host so simulation output is written directly to your local filesystem.

---

### Running a simulation inside Docker

```bash
cd /app/experiments/zbot_swimming
farmsim --experiment_config experiment_config.yaml
```

The FARMS virtual environment is on `PATH` inside the container - no activation step is needed.

---

### Docker compose reference

**Linux** (`docker_config/linux/docker-compose.yml`):

| Setting | Value | Purpose |
|---------|-------|---------|
| `DISPLAY` | `${DISPLAY}` | Inherits host X11 display |
| `NVIDIA_VISIBLE_DEVICES` | `all` | GPU passthrough |
| `/tmp/.X11-unix` volume | mapped | X11 socket forwarding |
| `~/.ssh` volume | `:ro` | SSH keys available inside container |
| `/dev/dri` device | mapped | DRM render node fallback |

**Windows** (`docker_config/windows/docker-compose.yml`):

| Setting | Value | Purpose |
|---------|-------|---------|
| `DISPLAY` | `host.docker.internal:0` | Routes X11 to VcXsrv on host |
| `NVIDIA_VISIBLE_DEVICES` | `all` | GPU passthrough via WSL 2 |
| `__GLX_VENDOR_LIBRARY_NAME` | `nvidia` | Forces NVIDIA GLX over mesa |
| `${USERPROFILE}/.ssh` volume | `:ro` | SSH keys for container use |

!!! warning "Software Rendering Fallback"
    If you are not using an NVIDIA graphics card, or if the NVIDIA configuration fails to work, you can switch to a software rendering fallback. This can be toggled in the docker-compose YAML file by uncommenting `LIBGL_ALWAYS_SOFTWARE=1` and removing the `deploy.resources` section. **Note that using software rendering will result in a performance degradation.**

---

## Method 2: Native Installation

Use this method if Docker is not available, or you need to modify FARMS source code and run it directly on the host.

### Prerequisites

| Requirement | Notes |
|-------------|-------|
| Python ≥ 3.10 | [python.org](https://www.python.org/) |
| Git | [git-scm.com](https://git-scm.com/install/) |
| Git LFS | `git lfs install` |
| C/C++ compiler | See platform notes below |
| SSH key on GitHub | [SSH Key Setup](ssh_setup.md) |

**Windows - Visual C++ Build Tools:**
Cython extensions require the MSVC compiler. Install **Visual C++ Build Tools 2017 or later**:

1. Download [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
2. In the installer, select **Desktop development with C++**
3. Ensure **MSVC v141 (VS 2017 C++ x64/x86 build tools)** or later is checked
4. Restart your terminal after installation

**Linux:**
```bash
sudo apt-get install build-essential git git-lfs
```

---

### Step 1: Clone the repository

```bash
git clone git@github.com:farmsim/farms_zbot.git
cd farms_zbot
```

---

### Step 2: Initialise git submodules

The FARMS packages (`farms_core`, `farms_sim`, `farms_mujoco`, `farms_amphibious`) are git submodules inside the `farms/` directory. They must be fetched before installation so that `setup_farms.py` can locate each package's `requirements.txt`.

```bash
git submodule update --init --recursive
```

This fetches all four FARMS packages and their nested submodules. Requires SSH access to GitHub.

---

### Step 3: Create a virtual environment

```bash
python -m venv .venv

# Activate - Linux/macOS:
source .venv/bin/activate

# Activate - Windows (PowerShell):
.venv\Scripts\Activate.ps1
```

Always use a virtual environment. Installing FARMS into the system Python is not supported.

---

### Step 4: Install FARMS

From the `farms/` directory, run the setup script:

```bash
cd farms
python setup_farms.py
```

`setup_farms.py` does the following for each package (`farms_core`, `farms_mujoco`, `farms_sim`, `farms_amphibious`) in order:

1. Installs `uv` (fast pip replacement) if not already present
2. Reads `<package>/requirements.txt` and installs dependencies via `uv pip install`
3. Installs the package itself in editable mode: `uv pip install -e <package>`
4. Cython `.pyx` files are compiled to C and then to `.pyd`/`.so` during this step

Installation order matters: `farms_core` must complete before `farms_mujoco`, which must complete before `farms_sim` and `farms_amphibious`.

!!! note
    The `--no-build-isolation` flag is passed internally by `setup_farms.py`. This means the build uses the packages already in your virtual environment rather than a clean temporary environment. Ensure `setuptools`, `wheel`, and Cython are present if you encounter build errors:
    ```bash
    pip install setuptools wheel Cython
    ```

---

### Step 5: Verify the installation

```bash
python -c "import farms_core; import farms_mujoco; import farms_amphibious; print('OK')"
```

---

### Running a simulation

```bash
cd experiments/zbot_swimming
farmsim --experiment_config experiment_config.yaml
```

---

### Updating after a pull

When the upstream repository changes, update both the repo and its submodules:

```bash
git pull
git submodule update --recursive
```

If any FARMS package changed (check `farms/` for modified files), rerun the setup script to recompile Cython extensions:

```bash
cd farms
python setup_farms.py
```

---

## Troubleshooting

### Docker: `ssh-agent` not running during build

```
error: Could not resolve host: github.com
```

The build requires the SSH agent to be running with your key loaded (Step 3). Verify with `ssh-add -l` before running `docker compose up --build`.

### Native: Cython compilation fails on Windows

```
error: Microsoft Visual C++ 14.0 or greater is required
```

The C++ Build Tools are not installed or not on `PATH`. Install Visual Studio Build Tools (2017+) and open a **Developer Command Prompt** or **Developer PowerShell** before running `setup_farms.py`.

### Docker on Windows: viewer does not open

The MuJoCo viewer sends OpenGL commands to VcXsrv via X11. Check:
1. VcXsrv is running (tray icon present)
2. **Native OpenGL** is unchecked in XLaunch
3. **Disable access control** is checked in XLaunch
4. `DISPLAY=host.docker.internal:0` is set in the compose file

### `ModuleNotFoundError: No module named farms_core`

The virtual environment is not activated, or `setup_farms.py` was not run. Activate the venv and rerun the script.

---

## See Also

- [SSH Key Setup](ssh_setup.md) - generating and registering SSH keys with GitHub
- [Simulation Walkthrough](walkthrough.md) - running your first experiment
- [Configuration Reference](configuration.md) - YAML parameter reference
