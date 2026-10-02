# Plug into FARMS Modules

This guide helps you decide which FARMS integration point to use for your
specific need, with concrete examples for each.

## Decision table

| What you need | Where to plug in | Base class / mechanism |
|---------------|------------------|------------------------|
| Per-step runtime behavior (forces, logging) | `AnimatExtension` or `TaskExtension` | `farms_core.model.extensions.AnimatExtension` |
| Joint position/velocity/torque targets | `AnimatController` subclass | `farms_core.model.control.AnimatController` |
| YAML-configured custom object | `loader` dotted path + `Options` subclass | `farms_core.options.Options` |
| MuJoCo-specific physics behavior | `farms_mujoco` task/extension layer | `farms_mujoco.simulation.task.ExperimentTask` |
| CPG-based locomotion control | `farms_amphibious` network/controller | `farms_amphibious.control.amphibious.AmphibiousController` |
| Custom fluid force model | Swimming extension subclass | `farms_mujoco.swimming.extension.SwimmingExtension` |
| Custom sensor data | Extension with pre-allocated array | `AnimatExtension` |
| Custom options/parameters | `Options` subclass + `from_options()` | `farms_core.options.Options` |

## Integration pattern 1: Custom AnimatExtension

**When to use:** You need per-step code that runs alongside the simulation
(applying forces, logging data, modifying state).

**How:** Subclass `AnimatExtension`, implement `from_options()` and the
lifecycle methods, register in `animat_config.yaml`.

See [Write an AnimatExtension](write-extension.md) for full details.

## Integration pattern 2: Custom controller

**When to use:** You need to compute joint targets (positions, velocities, or
torques) at each step.

**How:** Subclass `AnimatController`, implement `from_options()`,
`before_step()`, and `positions()` / `velocities()` / `torques()`. List it
in the `extensions:` of `animat_config.yaml` (`control.controller_loader`
is not used).

See [Write a Controller](write-controller.md) for full details.

## Integration pattern 3: Custom options class

**When to use:** You need to add custom configuration parameters to YAML.

**How:** Subclass the options class of the file and register it in the
`loaders:` block of `experiment_config.yaml`. An animat options class must
keep everything the engine reads (`sdf`, `spawn`, `morphology`,
`control`, `extensions`), so subclass `AnimatOptions` (or
`AmphibiousOptions`) and pop your extra keys before calling the parent:

```python
from farms_amphibious.model.options import AmphibiousOptions


class MyRobotOptions(AmphibiousOptions):
    """Animat options with an extra parameter"""

    def __init__(self, **kwargs):
        custom_param = kwargs.pop('custom_param', 42)
        super().__init__(**kwargs)
        self.custom_param = custom_param
```

`animats` stays a list of file names, and `loaders.animats_options` (same
index) names the class that reads each one:

```yaml
animats:
  - my_robot_config.yaml
loaders:
  animats_options:
    - my_package.options.MyRobotOptions
  # ...plus simulation_options, arenas_options, experiment_data, animats_data
```

Extensions and controllers read the value from `animat_options.custom_param`.
A parameter used by only one extension is simpler to put in its
`config:`.

!!! note "Don't confuse this with extension `loader:`/`config:` pairs"
    This top-level `loaders:` mechanism (resolved by
    `ExperimentOptions.load()`) is separate from the inline
    `loader:`/`config:` pair used inside an `extensions:` list (imported
    with `import_item()` when the task creates the extensions). See
    [Options and YAML Design](../explanation/options-yaml-design.md) for
    both, side by side.

The `Options` base class is a `dict` subclass with `load(filename)` and
`save(filename)` methods that use `yaml2pyobject()` / `pyobject2yaml()` for
serialization. See [Options and YAML Design](../explanation/options-yaml-design.md)
for details.

## Integration pattern 4: Using the farms_amphibious CPG

**When to use:** You want CPG-based locomotion with the built-in oscillator
network.

**How:** Use `AmphibiousOptions` as the animat options loader, configure
`control.network` and `control.muscles`, and list `AmphibiousController`
in the animat's `extensions:`, as the AmphiBot example does:

```yaml
# experiment_config.yaml
loaders:
  animats_options:
    - farms_amphibious.model.options.AmphibiousOptions
```

```yaml
# animat_config.yaml
control:
  motors:
    - joint_name: joint_1
      control_types: [position]
      equation: position_muscle
      # ...
  network:
    drive_loader: ''
    drive_config: ''
    drives: [...]
    oscillators: [...]
    osc2osc: [...]
    drive2osc: [...]
  muscles: [...]
extensions:
  - loader: farms_amphibious.control.amphibious.AmphibiousController
    config: {}
  - loader: farms_mujoco.swimming.extension.SwimmingExtension
    config: {}
```

`AmphibiousController` is the concrete controller. It extends
`JointMuscleController`, the shared base, which is not meant to be used
directly. See [Configure CPG Network Parameters](configure-cpg-network.md).

## Integration pattern 5: Custom fluid dynamics

**When to use:** You want a different hydrodynamic force model.

**How:** First check the options of the built-in model: exact or lookup
table buoyancy, legacy or ellipsoid drag, added mass (see
[farms_mujoco.swimming](../reference/mujoco/mujoco-swimming.md)). For
another model, either:

1. write an `AnimatExtension` listed after `SwimmingExtension` that adds
   its forces to `physics.data.xfrc_applied` (see
   [Write an AnimatExtension](write-extension.md));
2. or extend the C loop of `SwimmingHandler`
   (`farms_mujoco/swimming/hydrodynamics.pyx`), see
   [Hydrodynamics Internals](../internals/hydrodynamics-internals.md#how-to-extend).

Option 1 is simpler; option 2 keeps everything in one allocation-free C
loop, which matters when many robots run in parallel.

## Integration pattern 6: Custom MuJoCo task behavior

**When to use:** You need to modify how the MuJoCo task initializes or steps
(sensor updates, controller application, etc.).

**How:** This is the most invasive integration. `ExperimentTask` (in
`farms_mujoco/simulation/task.py`) extends dm_control's `Task` class. You would
need to subclass it and override methods like `update_sensors()`,
`before_step()`, or `after_step()`. This is not recommended unless you have a
specific need that cannot be met through extensions.

## Package-level integration

If you are building a new FARMS package (e.g., `farms_myrobot`), follow the
pattern of the existing packages:

1. Create a `pyproject.toml` with dependencies on `farms_core`
2. Implement your options as `Options` subclasses
3. Implement your controller as an `AnimatController` subclass
4. Implement your data as `AnimatData` subclass (if needed)
5. Register your options loader in `experiment_config.yaml`

The `setup_farms.py` installer installs packages in order:
`farms_core` → `farms_mujoco` → `farms_sim` → `farms_amphibious`. Add your
package to the `PACKAGES` list to include it in the installer.

## See also

- [Write an AnimatExtension](write-extension.md)
- [Write a Controller](write-controller.md)
- [Configure an Experiment YAML](configure-yaml.md)
- [Extension and Controller Design](../explanation/extension-design.md)
