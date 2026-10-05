from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME
from stereo_ground_sim.research_record_policy import sample_eligible


def _record():
    record = {
        "left_u_px": 1, "left_v_px": 2, "right_u_px": 3, "right_v_px": 4,
        "nominal_pose_computation_status": "computed",
        "corrected_computation_status": "computed",
        "output_frame": BASELINE_CENTER_FRD_FRAME,
    }
    for prefix in ("nominal_pose", "corrected", "gt"):
        for axis in ("forward", "right", "down", "range"):
            record[f"{prefix}_{axis}_m"] = 1.0
    return record


def test_only_finite_evaluated_frd_record_is_eligible():
    assert sample_eligible(
        _record(), terminal_status="evaluated",
        measurement_status="processed", evaluation_status="evaluated",
    ) is True


def test_frame_mismatch_and_ground_truth_unavailable_are_ineligible():
    record = _record()
    record["output_frame"] = "world_enu"
    assert sample_eligible(
        record, terminal_status="evaluated",
        measurement_status="processed", evaluation_status="evaluated",
    ) is False
    record = _record()
    assert sample_eligible(
        record, terminal_status="gt_unavailable",
        measurement_status="processed", evaluation_status="unavailable",
    ) is False
