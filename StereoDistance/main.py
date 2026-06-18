from pathlib import Path
import sys

SASL_ROOT = Path(__file__).resolve().parents[1]
if str(SASL_ROOT) not in sys.path:
    sys.path.insert(0, str(SASL_ROOT))

from wiris_stereo_distance.gui import main

if __name__ == "__main__":
    raise SystemExit(main())
