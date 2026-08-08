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
| CPG-based locomotion control | `farms_amphibious` network/controller | `farms_amphibious.control.amphibious.JointMuscleController` |
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
`before_step()`, and `positions()` / `velocities()` / `torques()`. Register
via `controller_loader` in `animat_config.yaml`.

See [Write a Controller](write-controller.md) for full details.

## Integration pattern 3: Custom options class

**When to use:** You need to add custom configuration parameters to YAML.

**How:** Subclass `Options` and register it as an animat loader in the
`loaders:` block of `experiment_config.yaml`.

```python
from farms_core.options import Options

class MyRobotOptions(Options):
    """Custom options for my robot."""

    def __init__(self, **kwargs):
        super().__init__()
        self.sdf = kwargs.pop('sdf')
        self.custom_param = kwargs.pop('custom_param', 42)
        assert not kwargs, f'Unknown kwargs: {kwargs}'
```

Register in `experiment_config.yaml`: `animats` stays a list of filenames,
and `loaders.animats_options` (same index) names the class that parses each
one.

```yaml
animats:
  - my_robot_config.yaml
loaders:
  animats_options:
    - my_package.options.MyRobotOptions
  # ...plus simulation_options, arenas_options, experiment_data, animats_data
```

!!! note "Don't confuse this with extension `loader:`/`config:` pairs"
    This top-level `loaders:` mechanism (resolved by
    `ExperimentOptions.load()`) is separate from the inline
    `loader:`/`config:` pair used inside an `extensions:` list (resolved by
    `ExtensionOptions`/`import_item()` when the task builds extensions). See
    [Options and YAML Design](../explanation/options-yaml-design.md) for
    both, side by side.

The `Options` base class is a `dict` subclass with `load(filename)` and
`save(filename)` methods that use `yaml2pyobject()` / `pyobject2yaml()` for
serialization. See [Options and YAML Design](../explanation/options-yaml-design.md)
for details.

## Integration pattern 4: Using the farms_amphibious CPG

**When to use:** You want CPG-based locomotion with the built-in oscillator
network.

**How:** Use `AmphibiousOptions` as your animat options loader and configure
the `control.network` section. The default controller
(`JointMuscleController`) is loaded automatically.

```yaml
# experiment_config.yaml
animats:
  - animat_config.yaml
loaders:
  animats_options:
    - farms_amphibious.model.options.AmphibiousOptions
```

```yaml
# animat_config.yaml
control:
  controller_loader: farms_amphibious.control.amphibious.JointMuscleController
  network:
    oscillators:
      - name: osc_0
        initial_phase: 0.0
        initial_amplitude: 0.0
        # ... (see Configure CPG Network Parameters)
    drives:
      - name: drv_0
        initial_value: 1.0
        kind: spine_left
        contacts: []
    # ...
```

See [Configure CPG Network Parameters](configure-cpg-network.md) for the full
network schema.

## Integration pattern 5: Custom fluid dynamics

**When to use:** You want a different hydrodynamic force model.

**How:** The `SwimmingExtension` computes drag and buoyancy and applies them
via `physics.data.xfrc_applied`. You can either:

1. Write a new `AnimatExtension` that computes and applies forces differently
2. Subclass `SwimmingExtension` and override the force computation

```python
from farms_mujoco.swimming.extension import SwimmingExtension

class CustomFluidModel(SwimmingExtension):
    @classmethod
    def from_options(cls, config, experiment_options, animat_i,
                    animat_data, animat_options):
        extension = super().from_options(
            config, experiment_options, animat_i,
            animat_data, animat_options,
        )
        # Add custom parameters
        extension.custom_drag_coeff = config.get('custom_drag', 0.5)
        return extension

    def before_step(self, task, action, physics):
        # Call parent for standard forces, then add custom forces
        super().before_step(task, action, physics)
        # Apply additional custom forces...
```

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
