from __future__ import annotations

import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
COMMON_PYTHON = REPOSITORY_ROOT / "common" / "python"
if str(COMMON_PYTHON) not in sys.path:
    sys.path.insert(0, str(COMMON_PYTHON))

from sasl_common.operator_runtime import check_official_interpreter, render_report

interpreter = check_official_interpreter(root=REPOSITORY_ROOT)
if interpreter.exit_code:
    print(render_report(interpreter, title="Calibration runtime blocked"), file=sys.stderr)
    raise SystemExit(interpreter.exit_code)

try:
    import cv2
    import numpy as np
except Exception as exc:  # noqa: BLE001 - report native OpenCV/NumPy loader failures.
    print(f"OpenCV calibration dependencies unavailable: {exc}", file=sys.stderr)
    raise SystemExit(1)

if int(np.__version__.split('.')[0]) >= 2:
    print(
        f"NumPy {np.__version__} is installed, but OpenCV calibration runtime expects NumPy < 2.",
        file=sys.stderr,
    )
    raise SystemExit(1)

print(f"OpenCV calibration dependencies OK: OpenCV {cv2.__version__}, NumPy {np.__version__}")
