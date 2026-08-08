# Install and Run a Simulation

This is the single installation guide for FARMS, covering both the Docker
and native paths side by side, followed by running your first simulation
with the Zbot undulatory swimming robot. Earlier drafts split Docker into
a separate how-to page; that split has been undone here and every step
below has been re-verified directly against `docker_config/{linux,windows}/
{Dockerfile,docker-compose.yml}`, `.gitmodules`, `.gitattributes`, and
`farms/setup_farms.py`.

!!! note "Source files"
    - `docker_config/linux/{Dockerfile,docker-compose.yml}`
    - `docker_config/windows/{Dockerfile,docker-compose.yml}`
    - `.gitmodules` — submodule definitions for the four FARMS packages
    - `.gitattributes` — Git LFS patterns for meshes and images
    - `farms/setup_farms.py` — FARMS package installation script
    - `experiments/zbot_bout_glide/run_sim.py` — experiment entry point

FARMS requires SSH access to GitHub to clone the repository and its
submodules — set up an SSH key registered on GitHub before starting
either path below.

### Step 0: Set up an SSH key (skip if you already have one)

Check whether you already have a key pair:

```bash
ls ~/.ssh
```

If you see `id_ed25519` + `id_ed25519.pub` (or `id_rsa` + `id_rsa.pub`),
skip ahead to registering it on GitHub below. Otherwise generate one —
**ed25519** is recommended (smaller, faster, widely supported):

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

Accept the default file location (`~/.ssh/id_ed25519`) and set a
passphrase (recommended) or press Enter for none. This creates a
**private key** (`id_ed25519`, never share it) and a **public key**
(`id_ed25519.pub`, this one goes to GitHub). On older systems without
ed25519 support, use `ssh-keygen -t rsa -b 4096 -C "your_email@example.com"`
instead, which produces `id_rsa` / `id_rsa.pub` — if you go this route,
remember Step 3 below (Windows) and `docker_config/windows/docker-compose.yml`
default to `id_ed25519` and need editing to point at `id_rsa`.

Register the public key on GitHub:

```bash
# Linux
cat ~/.ssh/id_ed25519.pub | xclip -selection clipboard
# macOS
cat ~/.ssh/id_ed25519.pub | pbcopy
# Windows (PowerShell)
Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub | Set-Clipboard
```

Then go to **GitHub → Settings → SSH and GPG keys** ([direct
link](https://github.com/settings/keys)) → **New SSH key**, give it a
label for this machine, and paste the key (starts with `ssh-ed25519
AAAA...`).

Test the connection:

```bash
ssh -T git@github.com
# Expect: "Hi <username>! You've successfully authenticated, but GitHub
# does not provide shell access."
```

If you instead get `Permission denied (publickey)`:

1. Confirm the key file permissions: `chmod 600 ~/.ssh/id_ed25519`,
   `chmod 644 ~/.ssh/id_ed25519.pub`, `chmod 700 ~/.ssh`.
2. Confirm the public key is actually saved under **Settings → SSH keys**
   on GitHub.
3. Run `ssh -vT git@github.com 2>&1 | grep "Offering\|Authenticated"` to
   see which key is actually being tried.

!!! tip "Multiple GitHub accounts"
    If you use more than one GitHub account, add per-host aliases to
    `~/.ssh/config` instead of relying on the default `github.com` host:
    ```
    Host github-personal
        HostName github.com
        User git
        IdentityFile ~/.ssh/id_ed25519_personal
    Host github-work
        HostName github.com
        User git
        IdentityFile ~/.ssh/id_ed25519_work
    ```
    then clone using the alias, e.g. `git clone git@github-personal:farmsim/farms_zbot.git`.

This key is used both for the `git clone` below and for the Docker build's
SSH forwarding (Method 1, Step 3) — no separate setup is needed for Docker
beyond what Step 3 covers for your OS.

**Docker is the recommended approach.** It handles the compiler, OpenGL
drivers, and GPU configuration for you. Use the native path if you don't
want a container, or if you need to iterate on FARMS source code directly
on the host.

---

## Method 1: Docker

Docker encapsulates the whole FARMS environment — system libraries,
OpenGL drivers, Python packages, and Cython extensions — inside a
`python:3.12-slim`-based image. No manual dependency management is
required.

### Prerequisites

| Requirement | Linux | Windows |
|---|---|---|
| [Docker Desktop](https://www.docker.com/products/docker-desktop/) or Docker Engine (with BuildKit) | ✓ | ✓ |
| [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) | for GPU | for GPU |
| WSL 2 as the default Docker backend | — | ✓ |
| [VcXsrv](https://sourceforge.net/projects/vcxsrv/) X server | — | for the GUI viewer |
| [SSH key registered on GitHub](#step-0-set-up-an-ssh-key-skip-if-you-already-have-one) | ✓ | ✓ |

### Step 1: Clone the repository

```bash
git clone git@github.com:farmsim/farms_zbot.git
cd farms_zbot
```

You do **not** need to run `git lfs pull` or `git submodule update`
yourself — the container build does both for you in Step 4, using SSH
forwarding rather than your host's already-checked-out state.

### Step 2 (Linux only): Allow X11 forwarding from Docker

```bash
xhost +local:docker
```

Grants the container permission to open windows on your display. Run once
per session.

### Step 2 (Windows only): Configure VcXsrv

MuJoCo's viewer needs an X display server on Windows. Install and launch
**VcXsrv** via **XLaunch** with these exact settings:

1. **Display settings**: Multiple windows, Display number `0`
2. **Client startup**: Start no client
3. **Extra settings**:
   - ☑ Clipboard
   - ☑ Primary Selection
   - ☐ **Native OpenGL** — **must be unchecked**
   - ☑ **Disable access control** — **must be checked**
4. Click **Finish** — VcXsrv starts in the system tray

!!! warning
    If **Native OpenGL** is enabled, or **Disable access control** is left
    unchecked, the MuJoCo viewer will fail to connect to the display. The
    Windows compose file sets `DISPLAY=host.docker.internal:0` to route
    X11 to VcXsrv on the host — this only works with both settings
    correct.

Docker on Windows with GPU passthrough requires WSL 2 as the active
backend:

```powershell
wsl --status
# Must show: Default Version: 2
```

If not: `wsl --set-default-version 2`

### Step 3: Make your SSH key available to the build

The build fetches the private FARMS submodules over SSH using BuildKit's
`--mount=type=ssh`. **Linux and Windows are configured differently here —
this is not symmetric, and getting it wrong on Windows is a common
failure point:**

**Linux** (`docker_config/linux/docker-compose.yml` has `ssh: - default`,
i.e. it forwards your running SSH agent):

```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519     # or your key's path
ssh-add -l                    # verify: should list your key's fingerprint
```

**Windows** (`docker_config/windows/docker-compose.yml` does **not** rely
on agent forwarding — it explicitly maps a key *file* instead:
`ssh: - default=${USERPROFILE}/.ssh/id_ed25519`, with an inline comment in
the compose file itself explaining that Windows `ssh-agent` forwarding
into BuildKit is unreliable):

```powershell
# Just make sure this file exists — no agent needs to be running:
Test-Path $env:USERPROFILE\.ssh\id_ed25519
```

If your key is RSA rather than ed25519, edit the `ssh:` line in
`docker_config/windows/docker-compose.yml` to point at
`${USERPROFILE}/.ssh/id_rsa` before building — the file path is hardcoded
in that compose file, unlike the Linux side which just forwards whatever
your agent already has loaded.

### Step 4: Build and start the container

From the repository root, run the compose file for your platform:

```bash
# Linux
docker compose -f docker_config/linux/docker-compose.yml up --build -d

# Windows (PowerShell)
docker compose -f docker_config/windows/docker-compose.yml up --build -d
```

The build (see the Dockerfiles for the exact layer order):

1. Installs system dependencies: `build-essential`, `git`, `git-lfs`,
   `openssh-client`, GLVND/Mesa OpenGL libraries, `python3-tk`, a LaTeX
   toolchain (for matplotlib figure rendering), and `ffmpeg`.
2. Creates a non-root user `farmsuser` (UID 1000, group `video`, and
   `render` if the group exists) so it has GPU device access, and
   preconfigures two bash aliases (see Step 5).
3. Adds `github.com` to `known_hosts` so SSH doesn't prompt interactively
   during the build.
4. Creates a Python virtual environment at `/home/farmsuser/venv` (put on
   `PATH` for the rest of the build and the running container) and
   installs `setuptools`, `wheel`, `uv`, and `Cython` into it. Cython is
   installed explicitly here — not left to `pyproject.toml`'s
   `build-system.requires` — because `setup_farms.py` passes
   `--no-build-isolation`, which skips `uv`'s normal step of installing
   each package's declared build dependencies into an isolated
   environment; it assumes they're already in the venv.
5. Copies the repository into `/app`.
6. **Using the SSH mount from Step 3**: configures git to rewrite
   `https://github.com/` URLs to `git@github.com:` (so submodule URLs
   resolve over SSH inside the container), runs `git lfs install`,
   `git lfs pull`, and `git submodule update --init --recursive`. An LFS
   object cache (`--mount=type=cache`) persists downloaded LFS objects
   across rebuilds.
7. Runs `python setup_farms.py` from `/app/farms`, inside the venv, with
   a `uv` package cache mounted so unchanged dependencies aren't
   re-fetched on rebuild. See "What `setup_farms.py` actually does"
   below — it's identical on both paths.

First build is typically 5–15 minutes depending on network and hardware.
Rebuilds after a source change are faster: the cache mounts mean only
changed packages are rebuilt.

### Step 5: Enter the container

```bash
# Linux
docker exec -it zbot_farms_linux bash

# Windows
docker exec -it zbot_farms_windows bash
```

The container starts in `/app` with the venv already on `PATH` — no
activation step needed. `experiments/`, `models/`, and
`farms/farms_mujoco/` are bind-mounted from the host (see the
`volumes:` block in each compose file), so edits to those directories on
your machine are reflected inside the container immediately, and
simulation output written inside the container lands directly on your
local filesystem. Other `farms/*` packages are **not** bind-mounted — they
are baked into the image from the build, so changes to `farms_core`,
`farms_sim`, or `farms_amphibious` require a rebuild.

A convenience alias is preconfigured in the container's `.bashrc`:

```bash
hommie    # cd /app/experiments/zbot_bout_glide
farming   # python3 run_sim.py --experiment_config experiment_config.yaml
```

### GPU and rendering configuration

Both compose files request GPU passthrough via `runtime: nvidia` and a
`deploy.resources.reservations.devices` block, and set
`NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display`,
`__GLX_VENDOR_LIBRARY_NAME=nvidia`, and `__NV_PRIME_RENDER_OFFLOAD=1` so
GLVND resolves to the NVIDIA driver rather than falling back to Mesa on
systems with both integrated and discrete GPUs. `/dev/dri` is mapped as a
fallback device path for integrated graphics.

!!! warning "Software rendering fallback"
    If you don't have an NVIDIA GPU, or the NVIDIA configuration doesn't
    work, edit the relevant `docker-compose.yml`: uncomment
    `LIBGL_ALWAYS_SOFTWARE=1` and remove the `deploy.resources` block. This
    works everywhere but is noticeably slower for the interactive viewer.

---

## Method 2: Native installation

Use this if Docker isn't available, or you need to modify FARMS source
directly and iterate on the host.

### Prerequisites

| Requirement | Notes |
|---|---|
| Python ≥ 3.10 | [python.org](https://www.python.org/) |
| Git | [git-scm.com](https://git-scm.com/install/) |
| Git LFS | `git lfs install` — the Zbot meshes (`.stl`, `.dae`, `.obj`, `.mtl`) and reference images (`.jpg`, `.png`) are LFS-tracked per `.gitattributes` |
| C/C++ compiler | Needed to build the Cython extensions in `farms_amphibious` and `farms_mujoco` |
| SSH key on GitHub | Required for the submodule clone |

**Windows — Visual C++ Build Tools:** Cython extensions require the MSVC
compiler.

1. Download [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
2. Select **Desktop development with C++**
3. Ensure **MSVC v141 (VS 2017 C++ x64/x86 build tools)** or later is checked
4. Run `setup_farms.py` from a **Developer PowerShell** afterward, so the
   compiler is on `PATH`

**Linux:**
```bash
sudo apt-get install build-essential git git-lfs
```

**macOS:** Xcode Command Line Tools (`xcode-select --install`)

### Step 1: Clone the repository

```bash
git clone git@github.com:farmsim/farms_zbot.git
cd farms_zbot
```

### Step 2: Fetch Git LFS files and initialise submodules

The repository holds four Python packages as **git submodules** under
`farms/` — `farms_core`, `farms_mujoco`, `farms_sim`, `farms_amphibious`
— plus a `farmsim_docs` submodule. A plain clone leaves those directories
empty and mesh/image files as small LFS pointer files.

```bash
git lfs install
git lfs pull
git submodule update --init --recursive
```

!!! warning "Don't skip this step"
    `setup_farms.py` (Step 4) reads each package's `pyproject.toml` from
    inside `farms/<package>/`. If submodules aren't initialised, those
    directories are empty and the install fails with a confusing "no such
    file" error rather than an obvious submodule warning. If mesh files
    later look wrong or a `.stl` is a few hundred bytes, it's almost
    always this step being skipped, not `git submodule` — `git lfs pull`
    is a separate command and easy to forget.

### Step 3: Create and activate a virtual environment

```bash
python -m venv .venv

# Linux / macOS
source .venv/bin/activate

# Windows (PowerShell)
.venv\Scripts\Activate.ps1
```

Always use a virtual environment — installing FARMS into the system
Python is not supported.

### Step 4: Install the FARMS packages

```bash
cd farms
python setup_farms.py
```

#### What `setup_farms.py` actually does

!!! danger "Reads `pyproject.toml`, not `requirements.txt`"
    An earlier draft of this guide said `setup_farms.py` reads each
    package's `requirements.txt`. **That file is never read.**
    `setup_farms.py` opens `<package>/pyproject.toml` with `tomllib` and
    reads `[project.dependencies]` directly — if a package has no
    `pyproject.toml`, it's silently skipped for that pass rather than
    erroring.

For each package, in order — `farms_core` → `farms_mujoco` →
`farms_sim` → `farms_amphibious` (this order matters: `farms_mujoco`
imports `farms_core`, and so on) — the script runs in **two separate
passes across all four packages**, not one pass per package:

1. **Dependency pass** (all four packages): for each package, reads
   `[project.dependencies]` from its `pyproject.toml` via `tomllib` and
   installs them with plain `uv pip install <deps>` — no `-e`, no local
   build. This lets `uv` resolve and cache the dependency graph without
   triggering a build of the (fast-changing) FARMS source itself.
2. **Editable-install pass** (all four packages): runs
   `uv pip install --no-build-isolation --config-settings
   editable_mode=compat -e <package> -v` for each package in turn. This is
   where the Cython `.pyx` files actually get compiled to `.pyd`/`.so`.
   Keeping this separate from pass 1 means a source change in one package
   doesn't force `uv` to redo dependency resolution for the others.

If `uv` isn't already importable, `setup_farms.py` installs it via `pip`
before doing anything else.

!!! note "`--no-build-isolation` needs build tools already in your venv"
    Because pass 2 uses `--no-build-isolation`, `uv` does **not**
    automatically install each package's declared build dependencies into
    an isolated environment first — it assumes `setuptools`, `wheel`, and
    Cython are already importable in your active venv. If you hit a build
    error mentioning a missing build backend:
    ```bash
    pip install setuptools wheel Cython
    ```

=== "Manual install (equivalent, without the two-pass separation)"
    ```bash
    pip install setuptools wheel Cython
    pip install -e farms/farms_core
    pip install -e farms/farms_mujoco
    pip install -e farms/farms_sim
    pip install -e farms/farms_amphibious
    ```

### Step 5: Verify the installation

```bash
python -c "import farms_core; import farms_mujoco; import farms_sim; import farms_amphibious; print('OK')"
```

---

## Running the Zbot experiment

This applies identically whether you followed the Docker or native path
(inside the container for Docker; inside your activated venv for native).

```bash
cd experiments/zbot_bout_glide
python run_sim.py --experiment_config experiment_config.yaml
```

This launches a MuJoCo simulation of the Zbot — an undulatory swimming
robot with a CPG-based bout-and-glide locomotion controller. The
equivalent `farmsim` console command (installed by `farms_sim`) works the
same way:

```bash
farmsim --experiment_config experiment_config.yaml
```

### What happens when you run this command

1. `run_sim.py` adds the experiment directory to `sys.path` itself (so
   `controller.zbot_controller` can be imported), then calls
   `farms_sim._bootstrap.main()` — which takes **no arguments**. See
   `reference/farms-sim.md` for the fully-verified entry-point chain if
   you're debugging this path; an earlier draft of this page (and several
   others) described a `main(__file__)` signature that never existed.
2. `_bootstrap.main()` checks for macOS/`mjpython`, then delegates to
   `farms_sim.farmsim.main()`, which parses CLI arguments via
   `sim_parse_args()` and loads `ExperimentOptions` from
   `experiment_config.yaml` — which in turn loads `simulation_config.yaml`,
   `animat_config.yaml`, and `arena_config.yaml` via the `loaders:` block
   (see `internals/options-yaml-internals.md`).
3. `ExperimentData` is allocated via the loader named in
   `exp_options.loaders.experiment_data`.
4. `MuJoCoSimulation.from_experiment()` builds the MuJoCo model from SDF
   files and options (see `internals/mjcf-builder-internals.md`).
5. `sim.run()` starts the physics loop — interactive viewer if a display
   is available, otherwise headless with a progress bar.

See [Trace a Simulation Step](simulation-workflow.md) for the full call
graph beyond this point.

### Interactive controls

When running with a display, the MuJoCo viewer accepts keyboard input:

- `Space` — pause/resume
- `Q` / `Esc` — quit

## Examine the output

After the simulation completes (or is interrupted), the output directory
contains:

| File | Contents |
|---|---|
| `simulation.hdf5` | All sensor data, network states, and timing — see [Save, Load, and Inspect Data](../how-to/save-load-data.md) |
| `options/` | Saved YAML copies of all configuration files |
| `model.mjb` / `scene.xml` | Compiled MuJoCo model (if `MjcfSaver` extension is enabled) |

## Updating after a pull

Both paths need the same two things kept in sync after upstream changes:

```bash
git pull
git submodule update --init --recursive
```

**Native**: if any FARMS package changed (check `farms/` for modified
files), rerun `python farms/setup_farms.py` to reinstall dependencies and
recompile Cython extensions.

**Docker**: rerun `docker compose -f docker_config/<platform>/docker-compose.yml
up --build -d` — the cache mounts mean only what changed actually
rebuilds.

## Troubleshooting

**Docker: build fails with `Could not resolve host: github.com` or a submodule fetch error**
: SSH isn't reaching the build. On Linux, confirm your agent has the key
  loaded (`ssh-add -l`) *before* running `docker compose up --build`. On
  Windows, confirm the key file path in `docker_config/windows/
  docker-compose.yml`'s `ssh:` line actually matches your key
  (`id_ed25519` by default) — Windows does not use the agent for this.

**Docker on Windows: the MuJoCo viewer window never appears**
: Check, in order: VcXsrv is running (tray icon present); **Native
  OpenGL** is unchecked in XLaunch; **Disable access control** is checked;
  `DISPLAY=host.docker.internal:0` is present in the compose file's
  `environment:` block.

**Native: Cython compilation fails on Windows**
: `error: Microsoft Visual C++ 14.0 or greater is required` — the C++
  Build Tools aren't installed or aren't on `PATH`. Install Visual Studio
  Build Tools (2017+) and rerun `setup_farms.py` from a **Developer
  PowerShell**.

**`setup_farms.py` fails immediately with a missing `pyproject.toml`**
: Git submodules weren't initialised — repeat Step 2 (native) / rebuild
  after fixing SSH access (Docker).

**Mesh files fail to load, or `.stl` files are a few hundred bytes**
: Git LFS objects weren't fetched. Native: `git lfs install && git lfs
  pull` from the repository root. Docker: this is done for you during the
  build — if it's still happening, the build likely failed at the LFS
  step; check the build log.

**`ModuleNotFoundError: No module named 'farms_core'`**
: Native: the virtual environment isn't activated, or `setup_farms.py`
  wasn't run (or failed partway) — activate the venv and rerun the
  script. Docker: `setup_farms.py` likely failed partway through the
  image build — check the `docker compose up --build` output for the
  failing package and rebuild after fixing it.

## Next steps

- [Trace a Simulation Step](simulation-workflow.md) — understand the code
  flow from YAML config to physics stepping
- [Write a Custom Controller](custom-controller.md) — implement your own
  locomotion controller
- [Configure an Experiment YAML](../how-to/configure-yaml.md) — customize
  simulation parameters
- [Contributing](../how-to/contributing.md) — the recommended development
  workflow, which assumes the Docker container from this guide
