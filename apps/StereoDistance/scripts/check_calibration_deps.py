from __future__ import annotations

import sys

try:
    import cv2
    import numpy as np
except Exception as exc:
    print(f"OpenCV calibration dependencies unavailable: {exc}", file=sys.stderr)
    raise SystemExit(1)

if int(np.__version__.split('.')[0]) >= 2:
    print(
        f"NumPy {np.__version__} is installed, but OpenCV calibration runtime expects NumPy < 2.",
        file=sys.stderr,
    )
    raise SystemExit(1)

print(f"OpenCV calibration dependencies OK: OpenCV {cv2.__version__}, NumPy {np.__version__}")
