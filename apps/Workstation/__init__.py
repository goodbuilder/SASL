"""Standalone Workstation UI shell and runtime owner.

This package imports only the Workstation-owned GUI, backend, and core
modules.  Its settings, recordings, lifecycle, and launcher are rooted in
the repository-level Workstation runtime.
"""

from .compat import install_legacy_module_aliases, legacy_module

install_legacy_module_aliases()

__all__ = ["WorkstationMainWindow"]


def __getattr__(name: str):
    """Load the Qt shell only when a caller explicitly requests it."""

    if name == "WorkstationMainWindow":
        from Workstation.application.main_window import WorkstationMainWindow

        return WorkstationMainWindow
    return legacy_module(name)
