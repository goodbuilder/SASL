"""settings command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_settings(registry, *, settings_apply, settings_save) -> None:
    for command_id, text, tooltip, handler in (
        (
            "settings.apply",
            "Apply settings",
            "Apply the complete Workstation settings payload through the existing panel owner.",
            settings_apply,
        ),
        (
            "settings.save",
            "Save settings",
            "Persist the complete Workstation settings payload without changing data or coordinate schemas.",
            settings_save,
        ),
    ):
        registry.register(CommandSpec(command_id, text, tooltip, "settings", handler))
