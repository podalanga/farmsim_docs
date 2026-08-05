# Swimming Experiment — Full YAML Config Walkthrough

This document walks through the complete YAML configuration for the Zbot swimming experiment located in `experiments/zbot_swimming/`.

## Configuration Files

| File | Purpose |
|------|---------|
| `experiment_config.yaml` | Top-level experiment configuration |
| `simulation_config.yaml` | Physics engine and simulation parameters |
| `animat_config.yaml` | Robot morphology, sensors, motors, CPG network |
| `arena_config.yaml` | Arena and water environment |

## experiment_config.yaml

Defines the top-level experiment structure:

```yaml
simulation: simulation_config.yaml
animats:
  - animat_config.yaml
arenas:
  - arena_config.yaml
```

## simulation_config.yaml

Configures the physics engine and simulation runtime:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `n_iterations` | 10000 | Total simulation steps |
| `buffer_size` | 10000 | Data buffer size |
| `timestep` | 0.001 | Physics timestep in seconds |
| `gravity` | [0, 0, -9.81] | Gravity vector |
| `cb_sub_steps` | 10 | Controller callback sub-steps |

## animat_config.yaml

The most complex configuration file. Defines the robot's morphology, sensors, motors, and CPG network.

### Morphology

Defines the 8 links and 6 joints of the Zbot body. See [Zbot Model](./model.md) for details.

### Sensors

Configured to track all 8 links, all 6 joints, and external forces on all links:

```yaml
sensors:
  links: [Head, Segment1, ..., Segment6, TailSegment]
  joints: [joint_1, joint_2, ..., joint_6]
  xfrc: [Head, Segment1, ..., Segment6, TailSegment]
```

### CPG Network

The CPG network is configured with 12 oscillators (2 per body segment — one for each side), coupled to produce a travelling wave:

```yaml
network:
  oscillators:
    - name: osc_body_0_L
      initial_phase: 0.0
      initial_amplitude: 1.0
      frequency_gain: 2.0
      frequency_bias: 0.0
      # ...
  osc2osc:
    - in: osc_body_0_L
      out: osc_body_1_L
      type: OSC2OSC
      weight: 10.0
      phase_bias: 0.5
    # ...
```

### Extensions

Registers the controller and hydrodynamics:

```yaml
extensions:
  - loader: farms_amphibious.control.amphibious.AmphibiousController
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
```

## arena_config.yaml

Defines the water environment:

```yaml
water:
  sdf: models/arena_water_v0.sdf
  drag: true
  buoyancy: true
  height: 0.0
  velocity: [0, 0, 0]
  viscosity: 0.001
  density: 1000.0
```

## See Also

- [Zbot Model](./model.md)
- [Custom CPG Controller](./cpg_controller.md)
- [Configuration Reference](../configuration.md)
