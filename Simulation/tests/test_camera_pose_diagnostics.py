from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.camera_pose_diagnostics import compare_camera_pose_payload
from stereo_ground_sim.gazebo_ground_truth_adapter import PosePayloadError


def _payload(*, optical_nsec: int = 3) -> dict:
    return {
        "header": {"stamp": {"sec": 2, "nsec": 3}},
        "pose": [
            {"name": "left_stereo_camera", "id": 10, "position": {"x": 10.0}, "orientation": {"w": 1.0}},
            {"name": "left_stereo_camera::base_link", "id": 11, "position": {"x": 1.0}, "orientation": {"w": 1.0}},
            {"name": "left_stereo_camera::optical_link", "id": 12, "position": {"x": 2.0}, "orientation": {"w": 1.0}},
        ],
    }


def test_camera_diagnostic_reports_desired_and_link_to_optical_deltas() -> None:
    comparison = compare_camera_pose_payload(
        _payload(),
        camera_label="left",
        parent_model_name="left_stereo_camera",
        desired_optical_position_world_enu_m=(11.5, 0.0, 0.0),
        desired_optical_rpy_world_rad=(0.0, 0.0, 0.0),
        expected_base_entity_id=11,
        expected_optical_entity_id=12,
    )

    assert comparison.gazebo_stamp_ns == 2_000_000_003
    assert comparison.observed_base_link_position_world_enu_m == (11.0, 0.0, 0.0)
    assert comparison.observed_optical_position_world_enu_m == (12.0, 0.0, 0.0)
    assert comparison.desired_to_observed_optical_delta_world_enu_m == (0.5, 0.0, 0.0)
    assert comparison.desired_to_observed_optical_delta_norm_m == pytest.approx(0.5)
    assert comparison.observed_optical_minus_base_link_world_enu_m == (1.0, 0.0, 0.0)
    assert comparison.observed_optical_minus_base_link_norm_m == pytest.approx(1.0)
    assert comparison.desired_to_observed_optical_orientation_delta_deg == pytest.approx(0.0)
    assert comparison.observed_optical_minus_base_link_orientation_delta_deg == pytest.approx(0.0)


def test_camera_diagnostic_rejects_identity_or_stamp_mismatch() -> None:
    with pytest.raises(PosePayloadError, match="entity id changed"):
        compare_camera_pose_payload(
            _payload(),
            camera_label="left",
            parent_model_name="left_stereo_camera",
            desired_optical_position_world_enu_m=(0.0, 0.0, 0.0),
            desired_optical_rpy_world_rad=(0.0, 0.0, 0.0),
            expected_base_entity_id=99,
        )


def test_camera_diagnostic_serialization_preserves_observed_orientations_xyzw() -> None:
    payload = _payload()
    payload["pose"][1]["orientation"] = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    payload["pose"][2]["orientation"] = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}

    comparison = compare_camera_pose_payload(
        payload,
        camera_label="left",
        parent_model_name="left_stereo_camera",
        desired_optical_position_world_enu_m=(11.5, 0.0, 0.0),
        desired_optical_rpy_world_rad=(0.0, 0.0, 0.0),
    )

    serialized = json.loads(json.dumps(asdict(comparison)))
    assert serialized["observed_base_link_orientation_world_from_entity_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert serialized["observed_optical_orientation_world_from_entity_xyzw"] == [0.0, 0.0, 0.0, 1.0]
