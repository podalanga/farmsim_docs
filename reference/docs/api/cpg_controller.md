# Custom CPG Controller — Step-by-Step Guide

This guide shows how to implement a custom CPG controller for the Zbot by extending `AnimatController`.

## Overview

Custom controllers in FARMS extend `AnimatController` from `farms_core.model.control`. The controller receives the current simulation state and returns desired joint commands.

## Basic Structure

```python
from farms_core.model.control import AnimatController
import numpy as np

class MyZbotController(AnimatController):
    def __init__(self, animat_i, joints_names, muscles_names, max_torques, substep=True):
        super().__init__(animat_i, joints_names, muscles_names, max_torques, substep)
        # Initialize CPG parameters here

    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
        # Build joints_names tuple (7 lists, one per ControlType)
        joints_names = cls.joints_from_control_types(
            animat_options.control.joints_names(),
            {m.joint_name: [0] for m in animat_options.control.motors}  # POSITION=0
        )
        max_torques = cls.max_torques_from_control_types(
            animat_options.control.joints_names(),
            {m.joint_name: m.limits_torque[1] for m in animat_options.control.motors},
            {m.joint_name: [0] for m in animat_options.control.motors}
        )
        return cls(animat_i, joints_names, (), max_torques)

    def positions(self, iteration, time, timestep):
        # Return desired joint positions
        return {
            joint: 0.1 * np.sin(2 * np.pi * time + i * 0.5)
            for i, joint in enumerate(self.joints_names[0])  # ControlType.POSITION
        }
```

## Registration

Register the controller in `animat_config.yaml`:

```yaml
extensions:
  - loader: my_module.MyZbotController
```

## The `positions()` Method

The `positions()` method is called every `cb_sub_steps` physics steps. It should return a dictionary mapping joint names to desired positions in radians.

The key insight is that `self.joints_names` is a 7-tuple where index 0 corresponds to `ControlType.POSITION`. The joints at this index are the ones that will receive position commands.

## Using the Built-in AmphibiousController

For most use cases, you don't need to write a custom controller. The built-in `AmphibiousController` handles CPG-based locomotion:

```yaml
extensions:
  - loader: farms_amphibious.control.amphibious.AmphibiousController
```

This controller:
1. Steps the CPG network ODEs
2. Converts oscillator outputs to joint commands via the configured equation (e.g., `position_muscle`)
3. Applies the commands to `physics.data.ctrl`

## See Also

- [Zbot Model](./model.md)
- [Swimming Experiment](./experiment.md)
- [Amphibious Controller API](../api/farms_amphibious_controller.md)
- [Controller & Extension Interfaces](../api/farms_core_control.md)
