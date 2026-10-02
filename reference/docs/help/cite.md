# How to cite

If FarmSim contributes to published work, please cite the FARMS paper:

> Arreguit, J., Tata Ramalingasetty, S., Danner, S. M., and Ijspeert, A.
> (2023). FARMS: Framework for Animal and Robot Modeling and Simulation.
> bioRxiv. [doi:10.1101/2023.09.25.559130](https://doi.org/10.1101/2023.09.25.559130)

```bibtex
@article{arreguit2023farms,
  title     = {{FARMS}: Framework for Animal and Robot Modeling and Simulation},
  author    = {Arreguit, Jonathan and Tata Ramalingasetty, Shravan and
               Danner, Simon M. and Ijspeert, Auke},
  journal   = {bioRxiv},
  year      = {2023},
  doi       = {10.1101/2023.09.25.559130},
  url       = {https://doi.org/10.1101/2023.09.25.559130},
}
```

## The AmphiBot example

The example robot of these docs is inspired by AmphiBot I. If you use it,
cite the original robot:

> Crespi, A., Badertscher, A., Guignard, A., and Ijspeert, A. J. (2005).
> AmphiBot I: an amphibious snake-like robot. *Robotics and Autonomous
> Systems*, 50(4), 163-175.
> [doi:10.1016/j.robot.2004.09.015](https://doi.org/10.1016/j.robot.2004.09.015)

```bibtex
@article{crespi2005amphibot,
  title     = {{AmphiBot I}: an amphibious snake-like robot},
  author    = {Crespi, Alessandro and Badertscher, Andr{\'e} and
               Guignard, Andr{\'e} and Ijspeert, Auke Jan},
  journal   = {Robotics and Autonomous Systems},
  volume    = {50},
  number    = {4},
  pages     = {163--175},
  year      = {2005},
  doi       = {10.1016/j.robot.2004.09.015},
}
```

## Software versions

For reproducibility, report the versions (branch, tag or commit) of the
FARMS packages you used. With the installer, they are the refs of
`reference/farms-packages.yaml`, and each installed package prints its
commit during the installation. In a checkout:

```bash
git -C farms-src/farms_mujoco log -1 --format=%H
```

Projects documented here may ask for their own citation, see their
[project pages](../projects/index.md).
