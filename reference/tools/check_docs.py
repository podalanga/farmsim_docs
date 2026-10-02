#!/usr/bin/env python3
"""Documentation drift guard: checks the hand-written pages against the
installed FARMS code, and fails (exit code 1) on any mismatch.

Checks, for every Markdown page under docs/:

1. Dotted references (`farms_mujoco.swimming.extension.SwimmingExtension`)
   import and resolve.
2. Source paths (`farms_mujoco/swimming/cob.pyx`) exist.
3. Command line flags exist in the parser: flags on run_sim.py / farmsim
   command lines, and flags written alone in a code span (`--log_path`),
   except the flags of other tools listed in OTHER_TOOLS_FLAGS.
4. Keys of YAML examples are known configuration options (see
   docgen.known_option_keys). A block whose first line is
   `# check-docs: skip` is not checked.
5. No em dashes, emojis, emoji-like symbols (check marks) or icon
   shortcodes (:material-...:).

Usage: python tools/check_docs.py [--docs docs] [--experiments DIR]
"""

import argparse
import importlib
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import docgen  # noqa: E402  pylint: disable=wrong-import-position

REFERENCE = re.compile(
    r'(?<![/\w-])farms_(?:core|mujoco|sim|amphibious)(?:\.[A-Za-z_][A-Za-z0-9_]*)+(?!\.git\b)'
)
SOURCE_PATH = re.compile(
    r'\b(farms_(?:core|mujoco|sim|amphibious)/[A-Za-z0-9_/.]+?\.(?:pyx|pxd|py))\b'
)
FLAG = re.compile(r'(?<![\w-])(--[a-z][a-z0-9_-]*)')
FLAG_SPAN = re.compile(r'`(--[a-z][a-z0-9_-]*)`')
# Flags of other tools (pip, uv, ...) or aliases handled outside the parser
OTHER_TOOLS_FLAGS = {
    '--no-build-isolation',  # pip / uv
    '--experiment-config',   # Rewritten to --experiment_config by run_sim.py
}
CLI_LINE = re.compile(r'run_sim\.py|farms_sim|farmsim\b')
FENCE = re.compile(r'^(\s*)```(\w*)')
EM_DASH = '\u2014'
# Emojis, dingbats and symbols (check marks, stars), and icon shortcodes
EMOJI = re.compile(
    '[\u2600-\u27bf\u2b50\u2b55\U0001f000-\U0001faff\ufe0f]'
    '|:(?:material|fontawesome|octicons|simple)-[a-z0-9-]+:'
)


def iter_blocks(lines):
    """Yield (language, start_line, block_lines) of fenced code blocks"""
    i = 0
    while i < len(lines):
        match = FENCE.match(lines[i])
        if match:
            language, start, block = match.group(2), i + 1, []
            i += 1
            while i < len(lines) and not lines[i].strip().startswith('```'):
                block.append(lines[i])
                i += 1
            yield language, start, block
        i += 1


def source_roots():
    """Directories containing the FARMS package sources"""
    roots = []
    for package in docgen.PACKAGES:
        module = importlib.import_module(package)
        root = Path(module.__file__).parent.parent
        # Paths may be relative to the repository (farms_core/...) or to
        # the folder containing the repositories (farms_core/farms_core/...)
        roots += [root, root.parent]
    return roots


def check_page(path, docs_dir, context):
    """List of (line, message) problems of a page"""
    problems = []
    text = path.read_text(encoding='utf-8')
    lines = text.splitlines()
    for number, line in enumerate(lines, 1):
        if EM_DASH in line:
            problems.append((number, 'em dash (use a comma, colon or parentheses)'))
        if EMOJI.search(line):
            problems.append((number, f'emoji or icon {EMOJI.search(line)[0]!r} (use words)'))
        for reference in set(REFERENCE.findall(line)):
            reference = reference.rstrip('.')
            if reference not in context['resolved']:
                context['resolved'][reference] = docgen.resolve(reference)
            if not context['resolved'][reference]:
                problems.append((number, f'unresolved reference `{reference}`'))
        for source in set(SOURCE_PATH.findall(line)):
            if not any((root / source).exists() for root in context['roots']):
                problems.append((number, f'missing source file `{source}`'))
        flags = set(FLAG_SPAN.findall(line))
        if CLI_LINE.search(line):
            flags.update(FLAG.findall(line))
        for flag in sorted(flags - OTHER_TOOLS_FLAGS):
            if flag not in context['flags']:
                problems.append((number, f'unknown CLI flag `{flag}`'))
    for language, start, block in iter_blocks(lines):
        if language not in ('yaml', 'yml') or not block:
            continue
        if block[0].strip().startswith('# check-docs: skip'):
            continue
        try:
            data = docgen.load_yaml('\n'.join(block))
        except Exception as error:  # pylint: disable=broad-except
            problems.append((start, f'invalid YAML example: {error}'))
            continue
        unknown = sorted(docgen.yaml_keys(data) - context['keys'])
        if unknown:
            problems.append((start, f'unknown configuration keys {unknown}'))
    return [
        (f'{path.relative_to(docs_dir)}:{line}', message)
        for line, message in problems
    ]


def main():
    """Main"""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    here = Path(__file__).resolve().parent
    parser.add_argument('--docs', default=str(here.parent / 'docs'))
    parser.add_argument(
        '--experiments', default=None,
        help='Folder of working experiment configurations whose keys are'
        ' accepted (default: the examples/ folder of farmsim_docs)',
    )
    args = parser.parse_args()
    docs_dir = Path(args.docs)
    experiments = (
        Path(args.experiments) if args.experiments
        else here.parents[1] / 'examples'
    )
    yaml_files = sorted(experiments.rglob('*config.yaml')) if experiments.is_dir() else []
    context = {
        'resolved': {},
        'roots': source_roots(),
        'flags': docgen.cli_flags(),
        'keys': docgen.known_option_keys(yaml_files),
    }
    problems = []
    for path in sorted(docs_dir.rglob('*.md')):
        problems += check_page(path, docs_dir, context)
    for location, message in problems:
        print(f'{location}: {message}')
    print(f'{len(problems)} problem(s) in {docs_dir}')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
