#!/usr/bin/env python3
"""Check out and install the FARMS packages listed in farms-packages.yaml.

Used by the documentation CI, and usable locally in a fresh environment:

    python tools/install_farms.py --dest ../farms-src

Each package is cloned from its public repository at its `ref`, its
declared dependencies are installed, then the package itself is installed
in editable mode without build isolation (the Cython modules of a package
cimport the .pxd files of the packages installed before it).

FARMS_REFS overrides refs, e.g. FARMS_REFS="farms_mujoco=my-branch,farms_core=abc123".
"""

import argparse
import os
import re
import subprocess
import sys
import tomllib
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REF = re.compile(r'[A-Za-z0-9_][A-Za-z0-9._/-]*')  # Branch, tag or commit, no option


def run(*command, cwd=None):
    """Run a command, echoing it"""
    print('+', ' '.join(str(part) for part in command), flush=True)
    subprocess.run(command, cwd=cwd, check=True)


def overrides():
    """{package: ref} from the FARMS_REFS environment variable"""
    refs = {}
    for item in filter(None, os.environ.get('FARMS_REFS', '').split(',')):
        name, _, ref = item.strip().partition('=')
        if not ref:
            raise ValueError(f'FARMS_REFS entry "{item}" is not package=ref')
        refs[name.strip()] = ref.strip()
    return refs


def checkout(repo, ref, path):
    """Shallow clone of repo at ref (branch, tag or commit)"""
    if not REF.fullmatch(ref) or '..' in ref:
        raise ValueError(f'Invalid ref "{ref}"')
    if path.exists():
        run('git', 'fetch', '--depth', '1', 'origin', ref, cwd=path)
    else:
        run('git', 'init', '-q', path)
        run('git', 'remote', 'add', 'origin', repo, cwd=path)
        run('git', 'fetch', '--depth', '1', 'origin', ref, cwd=path)
    run('git', 'checkout', '-q', 'FETCH_HEAD', cwd=path)
    run('git', 'log', '--oneline', '-1', cwd=path)


def dependencies(path):
    """Dependencies of a package ([project.dependencies] of pyproject.toml
    and requirements.txt), without the FARMS packages"""
    deps = []
    pyproject = path / 'pyproject.toml'
    if pyproject.is_file():
        data = tomllib.loads(pyproject.read_text())
        deps += data.get('project', {}).get('dependencies', [])
    requirements = path / 'requirements.txt'
    if requirements.is_file():
        deps += [
            line.split('#')[0].strip()
            for line in requirements.read_text().splitlines()
        ]
    return sorted({
        dep for dep in deps
        if dep and not dep.lower().replace('-', '_').startswith('farms_')
    })


def main():
    """Main"""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=str(HERE.parent / 'farms-packages.yaml'))
    parser.add_argument('--dest', default=str(HERE.parent.parent / 'farms-src'))
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text())
    refs = overrides()
    dest = Path(args.dest).resolve()
    dest.mkdir(parents=True, exist_ok=True)
    pip = [sys.executable, '-m', 'pip', 'install']
    run(*pip, 'setuptools', 'wheel', 'Cython', 'numpy')
    for package in config['packages']:
        name = package['name']
        ref = refs.pop(name, package['ref'])
        path = dest / name
        print(f'\n=== {name} @ {ref}', flush=True)
        checkout(package['repo'], ref, path)
        deps = dependencies(path)
        if deps:
            run(*pip, *deps)
        run(*pip, '--no-build-isolation', '--no-deps', '-e', path)
    if refs:
        raise ValueError(f'FARMS_REFS names unknown packages: {sorted(refs)}')


if __name__ == '__main__':
    main()
