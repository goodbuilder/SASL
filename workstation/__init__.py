"""Standalone Workstation UI shell and runtime owner for StereoWIRIS.

The Workstation imports its copied GUI/backend pair.  Shared measurement,
streaming, telemetry, calibration, and recording modules remain explicit
dependencies of that copied owner, while the Legacy launcher continues to
load ``wiris_stereo_distance.legacy_gui`` independently.  The old ``gui.py``
import surface remains a compatibility alias.
"""

__all__ = ["WorkstationMainWindow"]


def __getattr__(name: str):
    """Load the Qt shell only when a caller explicitly requests it."""

    if name == "WorkstationMainWindow":
        from .main_window import WorkstationMainWindow

        return WorkstationMainWindow
    raise AttributeError(name)
