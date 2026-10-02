# syntax=docker/dockerfile:1
# FarmSim image: the four FARMS packages (reference/farms-packages.yaml)
# installed in a virtual environment, with OpenGL for the MuJoCo viewer.
FROM python:3.12-slim

# Compiler for the Cython extensions, git for the package checkouts, and
# OpenGL (GLVND for NVIDIA, Mesa and OSMesa as fallbacks)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgl1-mesa-dri \
    libosmesa6 \
    mesa-utils \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Non-root user with UID 1000, usually the host user, so files written to
# the mounted examples belong to you. 'video' and 'render' give GPU access.
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} farms && \
    useradd -u ${UID} -m -g farms -G video farms && \
    (getent group render && usermod -aG render farms || true)
USER farms

RUN python -m venv /home/farms/venv
ENV PATH="/home/farms/venv/bin:${PATH}"
RUN pip install --no-cache-dir --upgrade pip pyyaml

# Install the FARMS packages (public repositories, no SSH key needed).
# FARMS_REFS overrides package refs, e.g. farms_mujoco=my-branch.
ARG FARMS_REFS=""
COPY --chown=farms:farms reference/farms-packages.yaml reference/tools/install_farms.py /home/farms/install/
RUN --mount=type=cache,target=/home/farms/.cache/pip,uid=${UID},gid=${GID} \
    FARMS_REFS="${FARMS_REFS}" python /home/farms/install/install_farms.py \
    --config /home/farms/install/farms-packages.yaml \
    --dest /home/farms/farms-src

# Examples (overridden by the mounted folder in docker-compose.yml)
WORKDIR /home/farms/workspace
COPY --chown=farms:farms examples examples

CMD ["bash"]
