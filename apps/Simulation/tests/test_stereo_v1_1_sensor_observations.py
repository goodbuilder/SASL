"""Synthetic camera RTK/IMU inputs stay fixed and separate from truth."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.generate_sensor_observations import _observation


def test_camera_sensor_observations_share_trial_and_do_not_expose_truth():
    spec = json.loads((SIM_ROOT / "campaigns/stereo_v1_1/protocol/03_experiment_spec.json").read_text(encoding="utf-8-sig"))
    truth = {
        "left": {"observed_optical_position_world_enu_m": [-12.5, 0.0, 0.1]},
        "right": {"observed_optical_position_world_enu_m": [12.5, 0.0, 0.1]},
    }
    first = _observation("G0", 2, 0, truth, spec)
    repeated = _observation("G0", 2, 0, truth, spec)
    another_trial = _observation("G0", 2, 1, truth, spec)
    assert first == repeated
    assert first["cameras"] != another_trial["cameras"]
    assert first["synthetic"] and not first["runtime_sensor_claim"]
    assert "truth" not in json.dumps(first["cameras"])
    for side in ("left", "right"):
        camera = first["cameras"][side]
        assert camera["physical_antenna_to_optical_center_lever_arm_m"] is None
        assert camera["optical_center_rtk_covariance_world_enu_m2"] == [
            [0.0004, 0.0, 0.0], [0.0, 0.0004, 0.0], [0.0, 0.0, 0.0016]
        ]
        angle_variance = math.radians(0.02) ** 2
        assert camera["camera_imu_roll_pitch_covariance_optical_rad2"] == [
            [angle_variance, 0.0], [0.0, angle_variance]
        ]
