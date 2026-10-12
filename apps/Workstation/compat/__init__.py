"""Lazy, finite compatibility for former root-level Python module paths.

Aliases share the canonical module object, so monkeypatches and old pickle
lookups do not create a second state owner. No GUI/runtime is imported here.
"""
from importlib import import_module
from importlib.abc import Loader, MetaPathFinder
from importlib.util import find_spec, spec_from_loader
import sys

from .module_aliases import MODULE_ALIASES


class _LegacyModuleLoader(Loader):
    def __init__(self, legacy, canonical):
        self.legacy = legacy
        self.canonical = canonical

    def create_module(self, spec):
        return None

    def exec_module(self, module):
        # Populate a temporary alias, then substitute the actual owner. Passing
        # that owner through create_module would overwrite its canonical spec.
        sys.modules[self.legacy] = import_module(self.canonical)

    def get_code(self, fullname):
        # Preserve supported `python -m Workstation.qualify_*` entrypoints.
        if fullname != self.legacy or not fullname.rsplit('.', 1)[-1].startswith('qualify_'):
            raise ImportError(f"Legacy module is not an executable entrypoint: {fullname}")
        spec = find_spec(self.canonical)
        return spec.loader.get_code(self.canonical)

    def get_filename(self, fullname):
        return find_spec(self.canonical).origin


class _LegacyModuleFinder(MetaPathFinder):
    _workstation_legacy_finder = True

    def find_spec(self, fullname, path=None, target=None):
        canonical = MODULE_ALIASES.get(fullname)
        if canonical is None:
            return None
        return spec_from_loader(fullname, _LegacyModuleLoader(fullname, canonical))


def install_legacy_module_aliases():
    if not any(getattr(finder, '_workstation_legacy_finder', False) for finder in sys.meta_path):
        # Existing policy/import guards and normal files take precedence.
        sys.meta_path.append(_LegacyModuleFinder())


def legacy_module(name):
    fullname = 'Workstation.' + name
    if fullname not in MODULE_ALIASES:
        raise AttributeError(name)
    return import_module(fullname)
