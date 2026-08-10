"""Repository-wide import paths for source-checkout tests."""

from __future__ import annotations

import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parent
SOURCE_ROOTS = (
    REPOSITORY_ROOT,
    REPOSITORY_ROOT / "common" / "python",
    REPOSITORY_ROOT / "apps",
    REPOSITORY_ROOT / "apps" / "StereoWIRIS",
    REPOSITORY_ROOT / "apps" / "Simulation" / "stereo_ground_sim",
)

for source_root in SOURCE_ROOTS:
    source_text = str(source_root)
    if source_text not in sys.path:
        sys.path.insert(0, source_text)
