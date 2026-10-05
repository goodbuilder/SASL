from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.pair_measurement_orchestrator import PairMeasurementOrchestrator


def _dependencies(mode="bbox_yolo"):
    calls = []
    detection = ({"cx": 10.0, "cy": 20.0}, {"cx": 8.0, "cy": 20.0})
    deps = {
        "camera_snapshot": lambda: "camera-snapshot",
        "operational_snapshot": lambda pair_id: {"pair_id": pair_id},
        "observation_mode": lambda: mode,
        "alignment": lambda _pair_id: SimpleNamespace(position_world_enu_m=(1, 2, 3)),
        "ideal_detection": lambda *_args: detection,
        "decode": lambda _message: np.zeros((4, 6, 3), dtype=np.uint8),
        "device": lambda: "cpu",
        "detect": lambda *_args: detection,
        "nominal": lambda *args: calls.append(("nominal", args)) or {"ray": "nominal"},
        "optical": lambda *args: calls.append(("optical", args)) or {"ray": "optical"},
        "gps": lambda *args: calls.append(("gps", args)) or {"gps": {}},
        "diagnostics": lambda *args: calls.append(("diagnostics", args)) or {"attitude": {}},
    }
    return deps, calls


def test_bbox_observation_feeds_identical_pixels_dimensions_and_snapshot_to_all_rays():
    deps, calls = _dependencies()
    pair = SimpleNamespace(
        pair_id=7, pair_midpoint_stamp_ns=99,
        left_message=object(), right_message=object(),
    )
    result = PairMeasurementOrchestrator(deps).compute(pair, "approach")
    assert result["raw"] == {"ray": "nominal"}
    assert calls[0] == ("nominal", (10.0, 20.0, 8.0, 20.0, 6, 4))
    assert calls[1][0] == "optical"
    assert calls[1][1][-1] == "camera-snapshot"
    assert calls[-1][1][-2:] == ({"pair_id": 7}, 99)


def test_ideal_mode_rejects_invalid_dimensions_without_invoking_ray_pipeline():
    deps, calls = _dependencies("ideal_reference_point")
    message = SimpleNamespace(width=0, height=4)
    pair = SimpleNamespace(
        pair_id=1, pair_midpoint_stamp_ns=2,
        left_message=message, right_message=message,
    )
    result = PairMeasurementOrchestrator(deps).compute(pair, "approach")
    assert result["ideal_error"] == "invalid_image_dimensions"
    assert calls == []
