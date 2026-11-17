"""Path resolution shared by v1.2 preflight and campaign execution."""

from __future__ import annotations

import os
from pathlib import Path
import sys


def root_python_path(sim_root: Path, *, environ=None, fallback: str | None = None) -> Path:
    """Prefer the app-owned runtime, while preserving explicit overrides."""
    env = os.environ if environ is None else environ
    override = str(env.get("STEREO_V1_1_ROOT_PYTHON", "")).strip()
    if override:
        path = Path(override).expanduser()
        return path if path.is_absolute() else sim_root / path

    app_runtime = sim_root / ".venv" / "bin" / "python"
    if app_runtime.is_file():
        return app_runtime
    return Path(sys.executable if fallback is None else fallback).expanduser()
