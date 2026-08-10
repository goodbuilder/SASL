"""Pytest path setup for Workstation tests moved to the repository root."""

from __future__ import annotations

import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
STEREO_WIRIS_ROOT = REPOSITORY_ROOT / "StereoWIRIS"

for path in (REPOSITORY_ROOT, STEREO_WIRIS_ROOT):
    path_text = str(path)
    if path_text not in sys.path:
        sys.path.insert(0, path_text)
