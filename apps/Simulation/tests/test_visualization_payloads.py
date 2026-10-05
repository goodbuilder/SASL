from pathlib import Path
import math
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.visualization_payloads import plot_payloads
from stereo_ground_sim import runtime_control_coordinator


def test_runtime_control_coordinator_imports_plot_payload_builder():
    assert runtime_control_coordinator.plot_payloads is plot_payloads


def test_plot_payload_preserves_public_frd_axis_order():
    metrics = {
        "estimated_forward_m": 1,
        "estimated_right_m": 2,
        "estimated_down_m": 3,
        "estimated_range_m": 4,
        "actual_distance_m": 5,
        "distance_m": 6,
        "distance_error_m": 1,
        "disparity_px": 7,
    }
    payload = plot_payloads(metrics)
    assert payload["nominal_pose_frd_range"] == [1.0, 2.0, 3.0, 4.0]
    assert payload["comparison"] == [5.0, 6.0, 1.0, 7.0]


def test_missing_values_are_nan_without_mutating_metrics():
    metrics = {}
    payload = plot_payloads(metrics, actual_fallback=9)
    assert payload["comparison"][0] == 9.0
    assert all(math.isnan(value) for value in payload["nominal_pose_frd_range"])
    assert metrics == {}
