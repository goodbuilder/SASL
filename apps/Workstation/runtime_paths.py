"""Filesystem ownership for the standalone Workstation runtime.

The default directory is deliberately repository-relative so a checkout can
be continued on another machine.  Operators may place runtime state on a
different volume with ``SASL_WORKSTATION_DATA_ROOT``; relative overrides are
still anchored to the repository instead of the launcher's current directory.
"""

from __future__ import annotations

import os
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def _workstation_data_root() -> Path:
    configured = os.environ.get("SASL_WORKSTATION_DATA_ROOT", "").strip()
    if not configured:
        return REPO_ROOT / ".local" / "workstation_data"
    root = Path(configured).expanduser()
    if not root.is_absolute():
        root = REPO_ROOT / root
    return root.resolve(strict=False)


WORKSTATION_DATA_ROOT = _workstation_data_root()
WORKSTATION_SETTINGS_PATH = WORKSTATION_DATA_ROOT / "stereo_distance_settings.json"
WORKSTATION_RECORDINGS_ROOT = WORKSTATION_DATA_ROOT / "recordings"
WORKSTATION_CALIBRATION_ROOT = WORKSTATION_DATA_ROOT / "calibration"
WORKSTATION_MODELS_ROOT = WORKSTATION_DATA_ROOT / "models"


def resolve_workstation_data_path(value: str | os.PathLike[str]) -> Path:
    """Resolve runtime-relative paths against the canonical data root, not CWD."""

    path = Path(value).expanduser()
    return path if path.is_absolute() else (WORKSTATION_DATA_ROOT / path).resolve(strict=False)

__all__ = [
    "REPO_ROOT",
    "WORKSTATION_DATA_ROOT",
    "WORKSTATION_SETTINGS_PATH",
    "WORKSTATION_RECORDINGS_ROOT",
    "WORKSTATION_CALIBRATION_ROOT",
    "WORKSTATION_MODELS_ROOT",
    "resolve_workstation_data_path",
]
