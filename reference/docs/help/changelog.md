# What's new

Notable changes to the FarmSim packages and these docs, newest first. The
docs always describe the refs of `reference/farms-packages.yaml`; the
commit history of each package has the details.

## October 2026

### Packages

- **farms_core**: SDF joints with a `<dynamics>` block load correctly
  (damping, friction and springs were read from the wrong element and the
  missing fields raised an error).
- **farms_amphibious**: `AmphibiousOptions.from_options()` builds the
  animat configuration of a new robot again (it had drifted from the
  option classes: drives, saturations, passive joints), with tests. Passive
  joints are listed once in the joint sensors.
- **farms_mujoco**: the centre-of-buoyancy lookup tables are cached in
  `cob_lut_cache/` next to the running script instead of `~/.cache`, with
  the `cob_lut_cache` water option and the `FARMS_COB_LUT_CACHE`
  environment variable to choose another directory, and fallbacks for
  read-only locations such as Docker volumes.

### Docs

- New structure: FarmSim documentation independent of any project, with
  [Projects](../projects/index.md) as case studies (Zbot).
- New example robot, AmphiBot, and six tutorials built on it.
- New Get started section ([Installation](../get-started/installation.md)
  with Docker and a virtual environment, without any project repository),
  [Supported platforms](../get-started/platforms.md), and Help pages:
  [Troubleshooting](troubleshooting.md), [FAQ](faq.md),
  [How to cite](cite.md), [License](license.md).
- Figures generated from simulations (`reference/tools/make_figures.py`).
- Edit links, last-updated dates, sitemap and a styled 404 page.

## September 2026

### Packages

- **farms_mujoco**: new fluid engine. Exact centre-of-buoyancy kernels in
  C (spheres, ellipsoids, cylinders, capsules, boxes and meshes), O(1)
  lookup tables, a single C loop for buoyancy and drag, and an ellipsoid
  drag and added mass model. See
  [Swimming and buoyancy](../tutorials/swimming.md).
- **farms_core**, **farms_amphibious**: fluid model options in the water
  options, buoyancy centre per link, ray casting sensors, attractor
  options.

### Docs

- API, configuration and CLI references generated from the code at every
  build, and a check of the hand-written pages against the code.
