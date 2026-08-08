# Options & YAML Internals

This page documents the options system and YAML parsing pipeline in detail. The `Options` class is a dict subclass with attribute-style access, and the YAML pipeline uses PyYAML (with CLoader/CDumper when available) for serialization.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_core/options.py` | 53 | `Options` — base dict subclass with attribute access |
| `farms_core/io/yaml.py` | 83 | `read_yaml`, `write_yaml`, `pyobject2yaml`, `yaml2pyobject` |
| `farms_core/extensions/extensions.py` | 51 | `ExtensionOptions`, `import_item`, `import_module_item` |
| `farms_core/model/options.py` | — | `AnimatOptions`, `MorphologyOptions`, `ControlOptions`, etc. |
| `farms_core/experiment/options.py` | — | `ExperimentOptions` — top-level options container |

## `Options` class (farms_core/options.py)

```python
class Options(dict):
    """Options"""

    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__

    def __getstate__(self):
        return self

    def __setstate__(self, value):
        for item in value:
            self[item] = value[item]

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name)

    def to_dict(self) -> dict:
        return {
            key: (
                value.to_dict() if isinstance(value, Options)
                else int(value) if isinstance(value, IntEnum)
                else [
                    val.to_dict() if isinstance(val, Options) else val
                    for val in value
                ] if isinstance(value, list)
                else value
            )
            for key, value in self.items()
        }

    @classmethod
    def load(cls, filename: str, strict: bool = True):
        kwargs = {'strict': False} if not strict else {}
        return cls(**yaml2pyobject(filename), **kwargs)

    def save(self, filename: str):
        pyobject2yaml(filename, self.to_dict())
```

### Design

`Options` is a `dict` subclass that allows attribute-style access (`options.key` instead of `options['key']`). This makes YAML configuration feel like native Python objects.

### Attribute access

```python
__getattr__ = dict.__getitem__
__setattr__ = dict.__setitem__
```

These two lines redirect attribute access to dictionary operations. `options.key = value` is equivalent to `options['key'] = value`.

The explicit `__getattr__` method (lines 22–27) is a fallback that converts `KeyError` to `AttributeError` for missing keys. This is important for `hasattr()` checks and pickling.

### `to_dict()`

Recursively converts the `Options` tree to a plain dict:

| Value type | Conversion |
|---|---|
| `Options` instance | `value.to_dict()` (recursive) |
| `IntEnum` | `int(value)` |
| `list` of `Options` | `[val.to_dict() if isinstance(val, Options) else val for val in value]` |
| Other | `value` (as-is) |

Note: `IntEnum` values are converted to integers, not their enum names. This means the YAML file will contain numeric values, not enum names like `'position'` or `'torque'`.

### `load(filename, strict=True)`

```python
@classmethod
def load(cls, filename, strict=True):
    kwargs = {'strict': False} if not strict else {}
    return cls(**yaml2pyobject(filename), **kwargs)
```

Loads a YAML file and constructs an `Options` instance. The `strict` parameter is passed as a kwarg to the constructor — if `strict=False`, the constructor should accept unknown keys without raising.

### `save(filename)`

```python
def save(self, filename):
    pyobject2yaml(filename, self.to_dict())
```

Converts to dict and writes to YAML.

### Pickling support

```python
def __getstate__(self):
    return self

def __setstate__(self, value):
    for item in value:
        self[item] = value[item]
```

`__getstate__` returns the dict itself. `__setstate__` copies items from the saved state. This enables pickling of `Options` instances.

## YAML I/O (farms_core/io/yaml.py)

### Loader/Dumper selection

```python
try:
    from yaml import CLoader as YamlLoader, CDumper as YamlDumper
except ImportError:
    from yaml import Loader as YamlLoader, Dumper as YamlDumper
    pylog.warning('YAML CLoader and CDumper not available, switching to Python implementation')
```

Uses the C-accelerated YAML loader/dumper when available (requires `libyaml`). Falls back to pure Python with a warning.

### `read_yaml(file_path)`

```python
def read_yaml(file_path):
    with open(file_path, 'r', encoding='utf-8') as stream:
        data = yaml.load(stream, Loader=YamlLoader)
    return data
```

Generic YAML reader. Returns a plain dict (or whatever YAML structure the file contains).

### `write_yaml(data, file_path)`

```python
def write_yaml(data, file_path):
    with open(file_path, 'w', encoding='utf-8') as stream:
        to_write = yaml.dump(
            data,
            default_flow_style=False,
            explicit_start=True,
            indent=2,
            width=80,
            sort_keys=False,
            Dumper=YamlDumper,
        )
        stream.write(to_write)
```

Generic YAML writer. Key formatting options:
- `default_flow_style=False`: Use block style (not flow style)
- `explicit_start=True`: Add `---` document start marker
- `indent=2`: 2-space indentation
- `width=80`: Line width 80 characters
- `sort_keys=False`: Preserve key order (important for human-readable configs)

### `pyobject2yaml(filename, pyobject, mode='w+')`

```python
def pyobject2yaml(filename, pyobject, mode='w+'):
    with open(filename, mode, encoding='utf-8') as yaml_file:
        yaml.dump(pyobject, yaml_file,
            default_flow_style=False,
            sort_keys=False,
            Dumper=YamlDumper)
```

Simpler than `write_yaml` — no `explicit_start`, `indent`, or `width` settings. Used by `Options.save()`.

### `yaml2pyobject(filename)`

```python
def yaml2pyobject(filename):
    with open(filename, 'r', encoding='utf-8') as yaml_file:
        options = yaml.load(yaml_file, Loader=YamlLoader)
    return options
```

Simpler than `read_yaml` — same functionality but used by `Options.load()`.

### `defaultdict` representer

```python
yaml.add_representer(
    data_type=collections.defaultdict,
    representer=yaml.representer.Representer,
)
```

Registers a representer for `defaultdict` so it serializes as a regular dict.

## `ExtensionOptions` (farms_core/extensions/extensions.py)

```python
class ExtensionOptions(Options):
    def __init__(self, **kwargs):
        super().__init__()
        self.loader: str = kwargs.pop('loader')
        self.config: list[str] = kwargs.pop('config')
        if kwargs.pop('strict', True) and kwargs:
            raise Exception(f'Unknown kwargs: {kwargs}')
```

### Fields

| Field | Type | Required | Description |
|---|---|---|---|
| `loader` | str | Yes | Dotted Python path to the extension class (e.g., `'farms_mujoco.swimming.extension.SwimmingExtension'`) |
| `config` | dict | Yes | Extension-specific configuration passed to `from_options()` |
| `strict` | bool | No (True) | If True, raise on unknown kwargs |

### `import_item(item_path)`

```python
def import_item(item_path):
    module_path, _, item_name = item_path.rpartition('.')
    return import_module_item(module_path, item_name)
```

Resolves a dotted path like `'farms_mujoco.swimming.extension.SwimmingExtension'`:
1. `rpartition('.')` splits into `('farms_mujoco.swimming.extension', '.', 'SwimmingExtension')`
2. `import_module_item` imports the module and gets the attribute

### `import_module_item(module_path, item_name)`

```python
def import_module_item(module_path, item_name):
    module = importlib.import_module(module_path)
    return getattr(module, item_name)
```

Standard `importlib` dynamic import.

## Options hierarchy

```
ExperimentOptions (Options)         # loaded by ExperimentOptions.load(), see below
  ├─ simulation: SimulationOptions
  │    ├─ physics: PhysicsSimulationOptions
  │    │    ├─ timestep, gravity, num_sub_steps, cb_sub_steps, n_solver_iters
  │    ├─ runtime: RuntimeSimulationOptions
  │    │    ├─ n_iterations, buffer_size, play, rtl, fast, headless, show_progress
  │    ├─ units: SimulationUnitScaling
  │    │    ├─ meters, seconds, kilograms
  │    ├─ mujoco: MuJoCoSimulationOptions
  │    │    ├─ integrator, solver, cone, ...
  │    ├─ pybullet: PybulletSimulationOptions
  │    └─ extensions: list[SimulationExtensionOptions]  (simulation-level extensions)
  │         └─ {loader: str, config: dict}
  ├─ arenas: list[ArenaOptions]
  │    ├─ sdf: str (path to arena SDF)
  │    ├─ spawn: SpawnOptions
  │    ├─ water: WaterOptions
  │    │    ├─ height, density, viscosity, velocity, drag, buoyancy
  │    │    └─ maps: list[str] (PNG velocity field paths)
  │    └─ (farms_core's own ArenaOptions is a thin ModelOptions subclass;
  │        the amphibious build uses AmphibiousArenaOptions instead — see
  │        the farms_amphibious internals for the fields actually used)
  ├─ animats: list[AnimatOptions]
  │    ├─ sdf: str (path to animat SDF)
  │    ├─ spawn: SpawnOptions
  │    │    ├─ loader: SpawnLoader (int enum, 0=FARMS/1=PyBullet — NOT a dotted path)
  │    │    ├─ mode: SpawnMode (default 'free')
  │    │    ├─ pose: [x, y, z, rx, ry, rz]
  │    │    ├─ velocity: [vx, vy, vz, wx, wy, wz]
  │    │    └─ extras: dict
  │    ├─ morphology: MorphologyOptions
  │    │    ├─ links: list[LinkOptions]
  │    │    │    ├─ name, collisions, friction, fluid_interaction, density
  │    │    │    ├─ drag_coefficients: [[Vx,Vy,Vz],[Wx,Wy,Wz]]
  │    │    │    └─ sites, solref, solimp, extras
  │    │    └─ joints: list[JointOptions]
  │    │         ├─ name, initial, stiffness, damping, springref, extras
  │    │         └─ limits: [[pos_min,pos_max],[vel_min,vel_max]]
  │    ├─ control: ControlOptions (farms_amphibious uses AmphibiousControlOptions)
  │    │    ├─ controller_loader: str (dotted path to an AnimatController)
  │    │    ├─ motors: list[MotorOptions]
  │    │    │    └─ joint_name, control_types, gains, limits_torque
  │    │    │       (farms_amphibious's motors add transform/offsets/passive/equation —
  │    │    │        see the farms_amphibious internals)
  │    │    ├─ sensors: SensorsOptions
  │    │    │    └─ links, joints, contacts, xfrc, muscles, adhesions, visuals
  │    │    ├─ hill_muscles: list[MuscleOptions]
  │    │    └─ (if AmphibiousControlOptions) network: AmphibiousNetworkOptions — see
  │    │         the farms_amphibious internals for oscillators/connections/drives
  │    └─ extensions: list[AnimatExtensionOptions]
  │         └─ {loader: str, config: dict}
  └─ loaders: ExperimentLoadOptions   # NOT nested under simulation/animats/arenas
       ├─ simulation_options: str
       ├─ animats_options: list[str]
       ├─ arenas_options: list[str]
       ├─ experiment_data: str
       └─ animats_data: list[str]
```

Every box above marked `{loader: str, config: dict}` is an `ExtensionOptions`
instance — a small, generic, inline mechanism. `loaders: ExperimentLoadOptions`
at the bottom is a **different**, top-level-only mechanism: it's not nested
inside `simulation`/`animats`/`arenas`, and its four/five fields point at
whole `Options` subclasses (and data classes) rather than extensions. See
"How options are loaded" below for exactly how each is resolved.

## How options are loaded

### Extensions: inline `{loader, config}`, resolved by the caller

`ExtensionOptions` (`farms_core/extensions/extensions.py`) only stores its
two fields — it does **not** resolve the dotted path itself:

```python
class ExtensionOptions(Options):
    def __init__(self, **kwargs):
        super().__init__()
        self.loader: str = kwargs.pop('loader')
        self.config: list[str] = kwargs.pop('config')  # type hint is wrong; actually a dict
        if kwargs.pop('strict', True) and kwargs:
            raise Exception(f'Unknown kwargs: {kwargs}')
```

Whatever code builds the extensions (e.g. `ExperimentTask.extract_extensions()`
in `farms_mujoco`) calls `import_item(extension.loader)` and then
`ExtensionClass.from_options(config=extension.config, ...)` itself, at the
point the extensions list is consumed — not inside `ExtensionOptions`.

### Top-level: `ExperimentOptions.load(filename)`, resolved by the class itself

```python
experiment_options = ExperimentOptions.load('experiment_config.yaml')
```

Unlike the plain `Options.load()` inherited by most classes,
`ExperimentOptions.load()` (`farms_core/experiment/options.py`) is
overridden to do the `loaders:`-driven resolution:

1. `super().load(filename)` — `yaml2pyobject(filename)` → plain dict, then
   `ExperimentOptions(**dict)`, which at this point still holds `simulation`,
   `animats[i]`, `arenas[i]` as plain **filename strings** (the constructor
   doesn't try to parse them).
2. `options.loaders = ExperimentLoadOptions(**options['loaders'])`.
3. If `options.simulation` is a `str`: resolve the path relative to
   `filename`, `import_item(options.loaders.simulation_options)`, and call
   `simulation_class.load(filename=path, strict=strict)` to replace it with
   the parsed `SimulationOptions`.
4. Same for each entry of `options.animats` (using
   `options.loaders.animats_options[i]`) and `options.arenas` (using
   `options.loaders.arenas_options[i]`), after asserting the list lengths
   match.

### Constructor chain

`ExperimentOptions.__init__` itself is much simpler than the loading logic
above — it just assigns whatever it's handed (already-parsed options
objects, by the time `.load()` calls it a second time, or already-`Options`
instances if constructed directly in Python):

```python
class ExperimentOptions(Options):
    def __init__(self, simulation, animats, arenas, loaders, **kwargs):
        super().__init__()
        self.simulation = simulation
        self.animats = animats
        self.arenas = arenas
        self.loaders = loaders
        if kwargs.pop('strict', True):
            assert not kwargs, kwargs
```

Sub-options classes (`SimulationOptions`, `AnimatOptions`, etc.) each follow
the more typical pattern of popping their own fields from `**kwargs` and
constructing their own nested `Options` objects — see, e.g.,
`SimulationOptions.__init__` building `RuntimeSimulationOptions`,
`PhysicsSimulationOptions`, `MuJoCoSimulationOptions`, and
`PybulletSimulationOptions` from the corresponding sub-dicts.

### Strict mode

Most constructors accept a `strict` parameter (default `True`). When `strict=True`, unknown kwargs raise an exception. When `strict=False` (used by `Options.load(filename, strict=False)`), unknown kwargs are silently ignored.

This is important for backward compatibility: if a YAML file has extra fields that the current code doesn't recognize, loading with `strict=False` will succeed.

### `from_options` pattern

Many options classes have a `from_options` classmethod that creates derived data from options:

```python
@classmethod
def from_options(cls, animat_options, simulation_options):
    # Create data from options
    ...
```

This is used by data classes (`AnimatData`, `AmphibiousData`), controller classes, and extensions.

## How to integrate: adding a new option to an existing class

1. **Add the field to the constructor**:

```python
class MyOptions(Options):
    def __init__(self, **kwargs):
        super().__init__()
        self.existing_field = kwargs.pop('existing_field')
        self.new_field = kwargs.pop('new_field', default_value)  # Add this
        if kwargs.pop('strict', True) and kwargs:
            raise Exception(f'Unknown kwargs: {kwargs}')
```

2. **Add to `to_dict()` if needed**: The base `Options.to_dict()` handles most cases automatically. If the new field is a custom type, you may need to add conversion logic.

3. **Add to YAML schema documentation**: Update the YAML schema reference docs.

4. **Use in code**: Access via `options.new_field`.

## How to integrate: adding a new options class

```python
class MyControlOptions(Options):
    def __init__(self, **kwargs):
        super().__init__()
        self.param1: str = kwargs.pop('param1')
        self.param2: float = kwargs.pop('param2', 0.0)
        self.items: list[MyItemOptions] = [
            MyItemOptions(**item) for item in kwargs.pop('items', [])
        ]
        if kwargs.pop('strict', True) and kwargs:
            raise Exception(f'Unknown kwargs: {kwargs}')
```

Use in YAML:

```yaml
control:
  param1: value1
  param2: 3.14
  items:
    - name: item1
      value: 42
```

## How to integrate: adding a new extension type

1. **Create the extension class**:

```python
class MyExtension(AnimatExtension):
    @classmethod
    def from_options(cls, config, experiment_options, animat_i, animat_data, animat_options):
        return cls(config=config, ...)
```

2. **Register in YAML** — inside the *animat's own config file*
   (`animat_config.yaml`, referenced by filename from `experiment_config.yaml`),
   not inline in `experiment_config.yaml` itself:

```yaml
# animat_config.yaml
extensions:
  - loader: my_package.my_extension.MyExtension
    config:
      my_param: 42
```

3. **How it's loaded**: `ExperimentTask.extract_extensions()` calls `import_item(extension.loader)` to resolve the class, then calls `.from_options(config=extension.config, ...)`.

## Common failure modes

### 1. Unknown kwargs with strict=True

If a YAML file contains a key that the options class doesn't recognize, and `strict=True` (default), the constructor raises `Exception(f'Unknown kwargs: {kwargs}')`. This is the most common error when upgrading FARMS or using custom YAML files.

**Fix**: Either add the field to the options class, or load with `strict=False`.

### 2. IntEnum serialization

`IntEnum` values are serialized as integers, not as their names. This means a YAML file will contain `0` instead of `'position'` for `ControlType.POSITION`. When reading the YAML back, you get an integer, not an enum value.

**Fix**: Convert explicitly: `ControlType(value)` to get the enum from an integer.

### 3. List of Options serialization

When a list contains `Options` instances, `to_dict()` recursively converts each element. But when loading, the constructor must explicitly iterate over the list and create `Options` instances from each dict:

```python
self.items = [MyItemOptions(**item) for item in kwargs.pop('items', [])]
```

If you forget this, `self.items` will be a list of plain dicts, not `Options` instances, and attribute access will fail.

### 4. `rpartition('.')` for import paths

`import_item` uses `rpartition('.')` which splits at the LAST dot. This means `'farms_mujoco.swimming.extension.SwimmingExtension'` correctly splits into module `farms_mujoco.swimming.extension` and item `SwimmingExtension`. But if the class name contains a dot (which is invalid in Python), it will fail.

### 5. `defaultdict` representer

The `defaultdict` representer is registered globally. If you use `defaultdict` in your options, it will be serialized as a regular dict. But when loading, you get a regular dict, not a `defaultdict`.

## What NOT to assume

1. **`Options` is a dict, not a dataclass.** It inherits from `dict` and uses `__getitem__`/`__setitem__`. This means `isinstance(options, dict)` is `True`, and all dict methods work.

2. **`strict=True` is the default.** Most constructors raise on unknown kwargs. Use `strict=False` for backward-compatible loading.

3. **`IntEnum` values are serialized as integers.** The YAML file will contain `0`, not `'POSITION'`. This is by design (for MuJoCo compatibility) but can be confusing.

4. **The `loader` field is a dotted Python path, not a file path.** `import_item` uses `importlib.import_module`, which requires a Python module path, not a filesystem path.

5. **`config` in `ExtensionOptions` is typed as `list[str]` in the code but used as `dict` in practice.** The type annotation in `ExtensionOptions.__init__` says `self.config: list[str] = kwargs.pop('config')`, but the actual YAML contains a dict. This is a type annotation error in the source code.

6. **YAML key order is preserved.** `sort_keys=False` in `write_yaml` and `pyobject2yaml` ensures the YAML output matches the insertion order. This is important for human-readable configuration files.
