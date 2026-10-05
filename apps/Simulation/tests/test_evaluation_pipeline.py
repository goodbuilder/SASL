from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.evaluation_pipeline import evaluation_metadata, vector_error


def _alignment():
    return SimpleNamespace(
        gt_alignment_mode="interpolated",
        gt_before_stamp_ns=10,
        gt_after_stamp_ns=20,
        gt_interpolation_alpha=0.5,
        gt_nearest_delta_ns=5,
    )


def test_frd_component_and_norm_errors_preserve_axis_order_and_sign():
    metrics = {
        "estimated_forward_m": 11.0,
        "estimated_right_m": -1.0,
        "estimated_down_m": 5.0,
        "estimated_range_m": 13.0,
    }
    gt = {"forward_m": 10.0, "right_m": 2.0, "down_m": 1.0, "range_m": 12.0}

    result = evaluation_metadata(SimpleNamespace(pair_id=4), _alignment(), gt, metrics)

    assert result["raw_forward_error_m"] == 1.0
    assert result["raw_right_error_m"] == -3.0
    assert result["raw_down_error_m"] == 4.0
    assert result["raw_position_error_m"] == pytest.approx((1 + 9 + 16) ** 0.5)
    assert result["raw_range_error_m"] == 1.0


def test_missing_ground_truth_produces_explicitly_unavailable_errors():
    result = evaluation_metadata(SimpleNamespace(pair_id=1), _alignment(), None, {})
    assert result["gt_forward_m"] is None
    assert result["raw_position_error_m"] is None
    assert result["attitude_gt_valid"] is False


def test_vector_error_rejects_non_finite_estimate():
    metrics = {
        "x_forward_m": float("nan"),
        "x_right_m": 0.0,
        "x_down_m": 1.0,
    }
    gt = {"forward_m": 0.0, "right_m": 0.0, "down_m": 0.0}
    assert vector_error(metrics, "x_", gt) == (None, None)
