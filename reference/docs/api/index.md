# Zbot in FARMS

The **Zbot** is a bio-inspired, eel-like underwater robot developed for research in swimming locomotion and neural control. It consists of a rigid **Head** module followed by six serially-connected **body segments** (`Segment1`–`Segment6`) and a **TailSegment**, connected by six revolute joints (`joint_1`–`joint_6`). Sinusoidal undulation of these joints generates the travelling wave that propels the robot forward.

## Contents

| Page | Description |
|------|-------------|
| [Zbot Model](./model.md) | SDF geometry and physical properties |
| [Swimming Experiment](./experiment.md) | Full YAML config walkthrough |
| [Custom CPG Controller](./cpg_controller.md) | Step-by-step controller implementation guide |

## Quick Start

```bash
cd experiments/zbot_swimming
python -m farms_sim.farmsim --experiment_config experiment_config.yaml
```

## See Also

- [Installation](../installation.md)
- [Configuration Reference](../configuration.md)
- [System Architecture](../architecture.md)
