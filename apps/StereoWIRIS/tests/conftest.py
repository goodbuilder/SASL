from __future__ import annotations

import sys
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = APP_ROOT.parents[1]
for path in (APP_ROOT, REPOSITORY_ROOT / "apps", REPOSITORY_ROOT / "common" / "python"):
    path_text = str(path)
    if path_text not in sys.path:
        sys.path.insert(0, path_text)
