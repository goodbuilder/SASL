"""Filesystem ownership for the standalone Workstation runtime.

The directory is deliberately repository-relative so a checkout can be
continued on another machine.  It is runtime state and is ignored by Git;
no sibling application data directory is consulted.
"""

from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSTATION_DATA_ROOT = REPO_ROOT / ".local" / "workstation_data"
WORKSTATION_SETTINGS_PATH = WORKSTATION_DATA_ROOT / "stereo_distance_settings.json"
WORKSTATION_RECORDINGS_ROOT = WORKSTATION_DATA_ROOT / "recordings"

__all__ = [
    "REPO_ROOT",
    "WORKSTATION_DATA_ROOT",
    "WORKSTATION_SETTINGS_PATH",
    "WORKSTATION_RECORDINGS_ROOT",
]
