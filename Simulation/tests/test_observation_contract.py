from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME
from stereo_ground_sim.observation_contract import (
    observation_metadata,
    observation_record_fields,
)


def test_bbox_record_contract_preserves_opencv_uv_and_bbox_order():
    left = {"cx": 12.5, "cy": 34.5, "confidence": 0.9, "bbox": (1, 2, 21, 42)}
    fields = observation_record_fields("bbox_yolo", left, None)

    assert (fields["left_u_px"], fields["left_v_px"]) == (12.5, 34.5)
    assert fields["left_bbox_xmin_px"] == 1.0
    assert fields["left_bbox_ymin_px"] == 2.0
    assert fields["left_bbox_xmax_px"] == 21.0
    assert fields["left_bbox_ymax_px"] == 42.0
    assert fields["right_observation_valid"] is False


def test_ideal_reference_pixels_never_claim_a_detector_bbox():
    ideal = {"cx": 10.0, "cy": 20.0, "confidence": 1.0, "bbox": (1, 2, 3, 4)}
    fields = observation_record_fields("ideal_reference_point", ideal, ideal)

    assert fields["left_observation_source"] == "gazebo_reference_point_projection"
    assert fields["left_bbox_xmin_px"] is None
    assert fields["right_bbox_ymax_px"] is None


def test_metadata_declares_baseline_center_frd_output_contract():
    metadata = observation_metadata("bbox_yolo")
    assert metadata["output_frame"] == BASELINE_CENTER_FRD_FRAME
    assert metadata["measurement_gt_independent"] is True
