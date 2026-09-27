"""Helpers shared by the documentation generator (gen_pages.py) and the
documentation drift guard (check_docs.py).

Everything here reads the installed FARMS packages, so the generated pages
and the checks always describe the code that is actually installed.
"""

import argparse
import dataclasses
import importlib
import inspect
import pkgutil
import re
import warnings
from pathlib import Path

PACKAGES = ['farms_core', 'farms_mujoco', 'farms_sim', 'farms_amphibious']


def quiet_logs():
    """Silence FARMS import-time log messages"""
    try:
        from farms_core import pylog
        pylog.set_level('error')
    except ImportError:
        pass


quiet_logs()

# Modules that are internal, optional or not meant for the API reference
SKIP_MODULES = (
    'farms_amphibious.bullet',       # Requires pybullet (optional engine)
    'farms_core.pylog.log',
    'farms_amphibious.callbacks',    # Legacy: imports a module that no longer exists
)

# Option classes documented in the configuration reference, by file
CONFIG_SECTIONS = [
    ('Experiment file', 'farms_core.experiment.options', 'ExperimentOptions'),
    ('Simulation file', 'farms_core.simulation.options', 'SimulationOptions'),
    ('Animat file', 'farms_amphibious.model.options', 'AmphibiousOptions'),
    ('Arena file', 'farms_amphibious.model.options', 'AmphibiousArenaOptions'),
]


# ---------------------------------------------------------------------------
# Modules
# ---------------------------------------------------------------------------

def iter_modules(package):
    """Public submodules of a package, including compiled Cython modules"""
    root = importlib.import_module(package)
    names = [package]
    for info in pkgutil.walk_packages(root.__path__, prefix=f'{package}.'):
        names.append(info.name)
    for name in sorted(set(names)):
        parts = name.split('.')
        if any(part.startswith('_') for part in parts[1:]):
            continue
        if name.startswith(SKIP_MODULES) or 'test' in parts[-1]:
            continue
        yield name


def import_module(name):
    """Import a module, returning None (with the error) when it fails"""
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            return importlib.import_module(name), None
    except Exception as error:  # pylint: disable=broad-except
        return None, error


def resolve(reference):
    """Resolve a dotted reference (module, class, function or attribute)"""
    parts = reference.split('.')
    for i in range(len(parts), 0, -1):
        module, _ = import_module('.'.join(parts[:i]))
        if module is None:
            continue
        obj = module
        for part in parts[i:]:
            if not hasattr(obj, part):
                return False
            obj = getattr(obj, part)
        return True
    return False


# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------

def type_name(value):
    """Readable name of a ClassDoc type"""
    if isinstance(value, str):
        return value
    return getattr(value, '__name__', str(value))


def option_classes():
    """All option classes (with a doc() method) reachable from the
    configuration sections, in a stable order"""
    ordered = {}

    def walk(cls):
        if not hasattr(cls, 'doc') or cls in ordered:
            return
        try:
            doc = cls.doc()
        except Exception:  # pylint: disable=broad-except
            return
        ordered[cls] = doc
        for child in doc.children:
            for candidate in (child.class_link, child.class_type):
                if inspect.isclass(candidate):
                    walk(candidate)

    for _, module, name in CONFIG_SECTIONS:
        walk(getattr(importlib.import_module(module), name))
    return ordered


def fluid_options_fields():
    """(name, type, default, description) of the fluid model options"""
    from farms_mujoco.swimming.fluid_options import FluidOptions
    descriptions = parse_yaml_comments(inspect.getmodule(FluidOptions).__doc__)
    rows = []
    for field in dataclasses.fields(FluidOptions):
        default = (
            field.default_factory() if field.default_factory
            is not dataclasses.MISSING else field.default
        )
        rows.append((
            field.name, type_name(field.type), default,
            descriptions.get(field.name, ''),
        ))
    return rows


def parse_yaml_comments(text):
    """`key: value  # comment` lines of a docstring -> {key: comment}"""
    result = {}
    for line in (text or '').splitlines():
        match = re.match(r'\s*([a-z_]+):[^#]*#\s*(.+)', line)
        if match:
            result[match.group(1)] = match.group(2).strip()
    return result


# Modules reading YAML dictionaries by subscript (connection['in'], ...)
SUBSCRIPT_KEY_MODULES = ('options.py', 'data.py', 'network.py')


def code_option_keys():
    """Keys read by the options classes: kwargs.pop/get('key') in code, and
    dictionary subscripts (entry['key']) in the options and network modules"""
    keys = set()
    pattern = re.compile(r"""kwargs\.(?:pop|get)\(\s*['"]([A-Za-z_0-9]+)['"]""")
    subscript = re.compile(r"""\w\[['"]([A-Za-z_][A-Za-z_0-9]*)['"]\]""")
    for package in PACKAGES:
        root = Path(importlib.import_module(package).__file__).parent
        for path in root.rglob('*.py'):
            text = path.read_text(encoding='utf-8', errors='ignore')
            keys.update(pattern.findall(text))
            if path.name in SUBSCRIPT_KEY_MODULES:
                keys.update(subscript.findall(text))
    return keys


def signature_option_keys():
    """Named __init__ parameters of every Options subclass"""
    from farms_core.options import Options
    keys = set()
    for package in PACKAGES:
        for name in iter_modules(package):
            module, _ = import_module(name)
            if module is None:
                continue
            for _, cls in inspect.getmembers(module, inspect.isclass):
                if not issubclass(cls, Options) or cls.__init__ is Options.__init__:
                    continue
                try:
                    parameters = inspect.signature(cls.__init__).parameters
                except (TypeError, ValueError):
                    continue
                keys.update(
                    name for name, parameter in parameters.items()
                    if name != 'self' and parameter.kind in (
                        parameter.POSITIONAL_OR_KEYWORD, parameter.KEYWORD_ONLY,
                    )
                )
    return keys


def known_option_keys(extra_yaml_files=()):
    """Every key that is valid somewhere in the configuration files"""
    import yaml
    keys = set(code_option_keys()) | signature_option_keys()
    for doc in option_classes().values():
        keys.update(child.name for child in doc.children)
    keys.update(name for name, *_ in fluid_options_fields())
    keys.update(['cob'])
    for path in extra_yaml_files:
        keys.update(yaml_keys(yaml.load(Path(path).read_text(), Loader=_yaml_loader())))
    return keys


def _yaml_loader():
    import yaml

    class Loader(yaml.SafeLoader):  # pylint: disable=too-many-ancestors
        """Safe loader ignoring Python tags"""

    Loader.add_multi_constructor('', lambda loader, suffix, node: None)
    return Loader


def load_yaml(text):
    """Load YAML text ignoring Python tags"""
    import yaml
    return yaml.load(text, Loader=_yaml_loader())


def yaml_keys(data, free_form=('config', 'extras', 'drive_config')):
    """All mapping keys of a loaded YAML document, skipping free-form
    sub-dictionaries (extension configs, extras)"""
    keys = set()
    if isinstance(data, dict):
        for key, value in data.items():
            keys.add(key)
            if key not in free_form:
                keys.update(yaml_keys(value, free_form))
    elif isinstance(data, list):
        for value in data:
            keys.update(yaml_keys(value, free_form))
    return {key for key in keys if isinstance(key, str)}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def cli_parser():
    """The farms_sim command line parser"""
    from farms_sim.utils.parse_args import sim_argument_parser
    return sim_argument_parser()


def cli_flags():
    """All long flags of the farms_sim command line"""
    return {
        option
        for action in cli_parser()._actions  # pylint: disable=protected-access
        for option in action.option_strings
    }


def cli_rows():
    """(flags, default, choices, help) for every CLI option"""
    rows = []
    for action in cli_parser()._actions:  # pylint: disable=protected-access
        if isinstance(action, argparse._HelpAction):  # pylint: disable=protected-access
            continue
        rows.append((
            ', '.join(action.option_strings) or action.dest,
            action.default,
            action.choices,
            (action.help or '').replace('\n', ' '),
        ))
    return rows
