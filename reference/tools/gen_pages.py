"""Pages generated from the installed FARMS code at every build
(mkdocs-gen-files entry point).

- reference/api/**: API reference of every public module (mkdocstrings)
- reference/env/configuration-reference.md: every YAML option, from the
  options' own doc() descriptions and the fluid model options
- reference/env/cli.md: the farms_sim command line, from its parser

Nothing here is written by hand: when the code changes, the next build
(triggered by CI) updates these pages.
"""

import sys
from pathlib import Path

import mkdocs_gen_files

sys.path.insert(0, str(Path(__file__).parent))
import docgen  # noqa: E402  pylint: disable=wrong-import-position

GENERATED_NOTE = (
    '!!! info "Generated page"\n'
    '    This page is generated from the source code at every documentation'
    ' build. Edit the code (docstrings, option `doc()` descriptions or'
    ' argument help), not this page.\n\n'
)


def md_cell(value):
    """Escape a value for a Markdown table cell"""
    return str(value).replace('|', '\\|').replace('\n', ' ').strip()


# ---------------------------------------------------------------------------
# API reference
# ---------------------------------------------------------------------------

def generate_api():
    """One page per module, plus a navigation summary"""
    nav = mkdocs_gen_files.Nav()
    for package in docgen.PACKAGES:
        for name in docgen.iter_modules(package):
            module, error = docgen.import_module(name)
            parts = name.split('.')
            path = Path('reference', 'api', *parts[:-1], f'{parts[-1]}.md')
            if name == package:
                path = Path('reference', 'api', package, 'index.md')
            nav[tuple(parts)] = path.relative_to('reference/api').as_posix()
            with mkdocs_gen_files.open(path, 'w') as page:
                page.write(f'# `{name}`\n\n')
                if module is None:
                    page.write(
                        '!!! warning "Not importable"\n'
                        f'    This module could not be imported when the'
                        f' documentation was built: `{error!r}`\n'
                    )
                    continue
                page.write(GENERATED_NOTE)
                page.write(
                    f'::: {name}\n'
                    '    options:\n'
                    '      show_submodules: false\n'
                )
    with mkdocs_gen_files.open('reference/api/SUMMARY.md', 'w') as summary:
        summary.writelines(nav.build_literate_nav())


# ---------------------------------------------------------------------------
# Configuration reference
# ---------------------------------------------------------------------------

def generate_config():
    """Every configuration option, grouped by option class"""
    classes = docgen.option_classes()
    names = {cls: cls.__name__ for cls in classes}
    text = '# Configuration Parameter Reference\n\n' + GENERATED_NOTE
    text += (
        'An experiment is described by YAML files: an experiment file that'
        ' lists a simulation file, one file per animat and one per arena,'
        ' plus the loaders (Python classes) used to read them. The tables'
        ' below list the options of each file, following the nesting of the'
        ' YAML files. Links point to the option class describing a nested'
        ' block.\n\n'
    )
    text += '## Files\n\n'
    for title, module, name in docs_sections():
        text += f'- **{title}**: [`{name}`](#{name.lower()}) (`{module}`)\n'
    text += '\n'
    for cls, doc in classes.items():
        text += f'## {names[cls]}\n\n'
        text += f'`{cls.__module__}.{cls.__name__}`\n\n'
        text += f'{md_cell(doc.description)}\n\n'
        if not doc.children:
            continue
        is_enum = hasattr(cls, '__members__')
        header = '| Value | Description |\n|---|---|\n' if is_enum else (
            '| Key | Type | Description |\n|---|---|---|\n'
        )
        text += header
        for child in doc.children:
            if is_enum:
                text += f'| `{child.name}` | {md_cell(child.description)} |\n'
                continue
            type_text = f'`{md_cell(docgen.type_name(child.class_type))}`'
            if child.class_link in classes:
                link = names[child.class_link]
                type_text += f' ([{link}](#{link.lower()}))'
            text += (
                f'| `{child.name}` | {type_text} |'
                f' {md_cell(child.description)} |\n'
            )
        text += '\n'
    text += (
        '## Fluid model options\n\n'
        'Optional keys of the arena `water` block read by the swimming'
        ' module (`farms_mujoco.swimming.fluid_options.FluidOptions`). They'
        ' can also be grouped in a nested `cob` block (`cob_method` becomes'
        ' `cob.method`, and so on).\n\n'
        '| Key | Type | Default | Description |\n|---|---|---|---|\n'
    )
    for name, kind, default, description in docgen.fluid_options_fields():
        text += (
            f'| `{name}` | `{md_cell(kind)}` | `{md_cell(default)}` |'
            f' {md_cell(description)} |\n'
        )
    with mkdocs_gen_files.open('reference/env/configuration-reference.md', 'w') as page:
        page.write(text)


def docs_sections():
    """Configuration sections (title, module, class name)"""
    return docgen.CONFIG_SECTIONS


# ---------------------------------------------------------------------------
# CLI reference
# ---------------------------------------------------------------------------

def generate_cli():
    """farms_sim command line options"""
    parser = docgen.cli_parser()
    text = '# CLI Reference\n\n' + GENERATED_NOTE
    text += (
        'Experiments are started with the `run_sim.py` script of an'
        ' experiment folder, which calls `farms_sim._bootstrap.main` and'
        ' parses the options below (`farms_sim.utils.parse_args`):\n\n'
        '```bash\npython run_sim.py --experiment_config experiment_config.yaml\n```\n\n'
        f'{md_cell(parser.description or "")}\n\n'
        '| Option | Default | Choices | Description |\n|---|---|---|---|\n'
    )
    for flags, default, choices, help_text in docgen.cli_rows():
        choices_text = ', '.join(f'`{c}`' for c in choices) if choices else ''
        text += (
            f'| `{md_cell(flags)}` | `{md_cell(default)}` | {choices_text} |'
            f' {md_cell(help_text)} |\n'
        )
    with mkdocs_gen_files.open('reference/env/cli.md', 'w') as page:
        page.write(text)


generate_api()
generate_config()
generate_cli()
