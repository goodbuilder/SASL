"""time sync command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_time_sync(registry, *, time_apply_settings, time_check_sync, time_save_settings, time_set_pc) -> None:
    for command_id, text, tooltip, handler in (
        (
            "time.set_pc",
            "Set camera clock",
            "Set the selected WIRIS camera clock through the existing owner.",
            time_set_pc,
        ),
        (
            "time.check_sync",
            "Check sync",
            "Refresh RTCP/NTP stream timing evidence through the existing owner.",
            time_check_sync,
        ),
        (
            "time.apply_settings",
            "Apply sync limits",
            "Apply Workstation camera synchronization limits through the existing owner.",
            time_apply_settings,
        ),
        (
            "time.save_settings",
            "Save sync limits",
            "Persist Workstation camera synchronization limits for the next run.",
            time_save_settings,
        ),
    ):
        registry.register(CommandSpec(command_id, text, tooltip, "time", handler))
