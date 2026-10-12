"""navigation command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_navigation(registry, *, select_context) -> None:
    registry.register(
        CommandSpec(
            "advanced.open",
            "Advanced",
            "Open Workstation advanced diagnostics and tools.",
            "view",
            lambda: select_context("tasks.advanced"),
        )
    )
