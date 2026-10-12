"""recording command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_recording_primary(registry, *, toggle_recording) -> None:
    registry.register(
        CommandSpec(
            "recording.toggle",
            "Manual record",
            "Advanced/manual fallback; normal Measurement runs record automatically.",
            "recording",
            toggle_recording,
        )
    )
