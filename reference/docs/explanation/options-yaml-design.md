# Options and YAML Design

This document explains the design of FARMS' configuration system — how YAML
files are loaded into Python objects, how the `Options` base class works, and
why dotted-path loaders are used.

## The Options base class

`Options` (`farms_core/options.py`) is a `dict` subclass:

```python
class Options(dict):
    @classmethod
    def load(cls, filename):
        """Load from YAML using yaml2pyobject()."""

    def save(self, filename):
        """Save to YAML using pyobject2yaml()."""
```

All configuration classes extend `Options` — `SimulationOptions`,
`AnimatOptions`, `ArenaOptions`, `ExperimentOptions`, and their amphibious
subclasses. This provides:

- **Serialization** — any options object can be saved to and loaded from YAML
- **Dict compatibility** — options behave as dicts, allowing flexible
  programmatic access
- **Type safety** — `__init__` methods use `kwargs.pop()` with assertions to
  catch unknown keys early

## YAML loading mechanism

`yaml2pyobject()` (`farms_core/io/yaml.py`) is **not** a smart, `loader`-aware
deserializer — it's a one-line wrapper around `yaml.load()`. It returns a
plain nested dict/list/scalar tree with no knowledge of `Options` classes at
all:

```python
def yaml2pyobject(filename: str) -> Any:
    with open(filename, 'r', encoding='utf-8') as yaml_file:
        options = yaml.load(yaml_file, Loader=YamlLoader)
    return options
```

`Options.load()` (`farms_core/options.py`) does the actual object
construction, by handing that dict straight to the class's own `__init__` as
kwargs — no dotted-path resolution happens here either:

```python
@classmethod
def load(cls, filename: str, strict: bool = True):
    kwargs = {'strict': False} if not strict else {}
    return cls(**yaml2pyobject(filename), **kwargs)
```

So the dotted-path resolution ("which class parses this sub-config") lives
**per call site**, not in a generic mechanism, and there are two distinct
places that do it:

**1. `ExperimentOptions.load()`** (`farms_core/experiment/options.py`)
overrides `Options.load()`. It reads a separate top-level `loaders:` block
that names the classes, and treats `simulation` / each entry of `animats` /
`arenas` as a plain filename string until it resolves and replaces it:

```yaml
# experiment_config.yaml
simulation: simulation_config.yaml
animats:
  - animat_config.yaml
arenas:
  - arena_config.yaml
loaders:
  simulation_options: farms_core.simulation.options.SimulationOptions
  animats_options:
    - farms_amphibious.model.options.AmphibiousOptions
  arenas_options:
    - farms_amphibious.model.options.AmphibiousArenaOptions
  experiment_data: farms_core.experiment.data.ExperimentData
  animats_data:
    - farms_core.model.data.AnimatData
```

For each of `simulation`, `animats[i]`, `arenas[i]` that is still a `str`
after loading, `ExperimentOptions.load()` calls `import_item()` on the
matching `loaders.*` entry and invokes that class's `.load(filename,
strict=strict)`, replacing the string in place. `animats`/`arenas` must have
exactly as many entries as `loaders.animats_options`/`loaders.arenas_options`
— a length mismatch raises an assertion naming the offending config file.

**2. `ExtensionOptions`** (`farms_core/extensions/extensions.py`) is a
smaller, unrelated mechanism used for the `extensions:` lists inside
`simulation_config.yaml` and `animat_config.yaml`. Each entry is an inline
`{loader, config}` pair, and it's the *caller* (e.g. `ExperimentTask`) that
calls `import_item(extension.loader)` and then `.from_options(config=
extension.config, ...)` — `ExtensionOptions.__init__` itself just stores the
two fields:

```yaml
extensions:
  - loader: farms_core.simulation.extensions.ExperimentLogger
    config:
      log_path: Output
```

These two mechanisms look similar (`loader:` appears in both) but are
independent code paths with different call signatures — don't assume one
generic "loader resolution" pass handles all of it.

## The from_options() pattern

Many option classes have a `from_options(cls, kwargs)` classmethod in addition
to `__init__`. This serves a different purpose:

- **`__init__`** — constructs from explicit kwargs (used when YAML has full
  nested structure)
- **`from_options()`** — constructs from a flat kwargs dict, applying defaults
  and convention-based logic

For example, `AmphibiousOptions.from_options()` takes a flat kwargs dict and:

1. Extracts `sdf_path` → maps to `sdf`
2. Builds `AmphibiousMorphologyOptions.from_options(kwargs)` if not provided
3. Determines control type (kinematics vs CPG) based on `kinematics_file`
4. Applies convention-based defaults via `defaults_from_convention()`

This dual-path design allows both fully-specified YAML configs and shorthand
configs that rely on convention-based defaults.

## Convention-based defaults

`AmphibiousConvention` (`farms_amphibious/model/convention.py`) computes
naming and indexing based on morphology parameters:

- `n_joints_body`, `n_legs`, `n_dof_legs` → oscillator count, names, connectivity
- Body oscillators: `osc_body_L_0`, `osc_body_R_0`, `osc_body_L_1`, ...
- Leg oscillators: `osc_leg_LF_0`, `osc_leg_RF_0`, ...
- Drive names: derived from oscillator names (`osc` → `drv`)

The `defaults_from_convention()` methods on options classes use this convention
to auto-generate:

- Oscillator names, initial phases, frequencies, amplitudes
- Drive names, initial values, kinds
- Connectivity (osc2osc, joint2osc, contact2osc, xfrc2osc)
- Sensor name lists (links, joints, contacts, xfrc)

This allows a minimal YAML config to specify only the morphology parameters
and get a fully configured CPG network.

## The assertion pattern

All `__init__` methods end with:

```python
assert not kwargs, f'Unknown kwargs: {kwargs}'
```

This is a design choice for fail-fast validation. If a YAML file contains an
unknown key, the error message shows exactly which key was unexpected. This
catches typos and config errors at load time rather than at runtime.

## Serialization fidelity

`pyobject2yaml()` serializes `Options` objects back to YAML. The round-trip
(YAML → object → YAML) is designed to be lossless for fully-specified configs.
However, convention-based defaults are expanded during `from_options()`, so a
minimal config loaded and then saved will produce a larger file with all
defaults explicitly written.

## See also

- [YAML Configuration Schema](../reference/env/yaml-schema.md) — complete key reference
- [Configure an Experiment YAML](../how-to/configure-yaml.md) — practical guide
- [farms_core Reference](../reference/core/farms-core.md) — Options class API
