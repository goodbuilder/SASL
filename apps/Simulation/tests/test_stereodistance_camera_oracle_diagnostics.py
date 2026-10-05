from __future__ import annotations

import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import project_world_point_to_pixel
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


SDF_SHA256 = "a" * 64
NOMINAL_ORIENTATION_XYZW = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))


def _node() -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    for prefix in ("cam_l_optical", "cam_r_optical", "cam_l_px4", "cam_r_px4"):
        setattr(node, f"{prefix}_roll_deg", 0.0)
        setattr(node, f"{prefix}_pitch_deg", 0.0)
        setattr(node, f"{prefix}_yaw_deg", 0.0)
    for prefix in ("common_optical", "relative_optical"):
        setattr(node, f"{prefix}_roll_error_deg", 0.0)
        setattr(node, f"{prefix}_pitch_error_deg", 0.0)
        setattr(node, f"{prefix}_yaw_error_deg", 0.0)
    node._ensure_camera_frames()
    node.camera_oracle_generated_sdf_sha256 = SDF_SHA256
    node.observed_left_camera = None
    node.observed_right_camera = None
    node.camera_oracle_rejection_count = 0
    node.last_camera_oracle_rejection_reason = "unavailable"
    node.camera_oracle_max_age_s = 2.5
    node.simulation_epoch = SimpleNamespace(epoch=0)
    node.get_logger = lambda: SimpleNamespace(warning=lambda _: None)
    return node


def _diagnostic(camera_label: str, *, stamp_ns: int, **changes: object) -> SimpleNamespace:
    center_x = -12.5 if camera_label == "left" else 12.5
    payload = {
        "schema": "camera_pose_comparison/v1",
        "camera_label": camera_label,
        "gazebo_stamp_ns": stamp_ns,
        "generated_sdf_sha256": SDF_SHA256,
        "observed_base_link_entity_name": f"{camera_label}_stereo_camera::base_link",
        "observed_base_link_entity_id": -1,
        "observed_optical_entity_name": f"{camera_label}_stereo_camera::optical_link",
        "observed_optical_entity_id": -1,
        "observed_base_link_position_world_enu_m": [center_x, 0.0, 5.0],
        "observed_optical_position_world_enu_m": [center_x, 0.0, 5.0],
        "observed_base_link_orientation_world_from_entity_xyzw": list(NOMINAL_ORIENTATION_XYZW),
        "observed_optical_orientation_world_from_entity_xyzw": list(NOMINAL_ORIENTATION_XYZW),
    }
    payload.update(changes)
    return SimpleNamespace(data=json.dumps(payload))


def _oracle_result(node: StereoDistanceNode):
    point_world = node.baseline_center_frd.point_to_world((100.0, 2.0, 1.0))
    intrinsics = node._camera_intrinsics(1280, 720)
    left_pixel = project_world_point_to_pixel(
        point_world,
        node.observed_left_camera.pose,
        fx_px=intrinsics.fx_px,
        fy_px=intrinsics.fy_px,
        cx_px=intrinsics.cx_px,
        cy_px=intrinsics.cy_px,
    )
    right_pixel = project_world_point_to_pixel(
        point_world,
        node.observed_right_camera.pose,
        fx_px=intrinsics.fx_px,
        fy_px=intrinsics.fy_px,
        cx_px=intrinsics.cx_px,
        cy_px=intrinsics.cy_px,
    )
    assert left_pixel is not None and right_pixel is not None
    return node._triangulate_pixel_centers_oracle(
        left_pixel[0], left_pixel[1], right_pixel[0], right_pixel[1], 1280, 720
    )


def test_oracle_returns_none_until_both_observed_camera_diagnostics_are_accepted() -> None:
    node = _node()

    assert node._triangulate_pixel_centers_oracle(640.0, 360.0, 630.0, 360.0, 1280, 720) is None

    node._camera_pose_diagnostic_callback("left", _diagnostic("left", stamp_ns=100))
    assert node._triangulate_pixel_centers_oracle(640.0, 360.0, 630.0, 360.0, 1280, 720) is None


def test_oracle_accepts_valid_diagnostics_with_distinct_gazebo_stamps() -> None:
    node = _node()
    node._camera_pose_diagnostic_callback("left", _diagnostic("left", stamp_ns=100))
    node._camera_pose_diagnostic_callback("right", _diagnostic("right", stamp_ns=200))

    result = _oracle_result(node)

    assert result is not None
    assert node._camera_oracle_record_fields()["camera_oracle_source"] == "gazebo_observed_optical"


def test_oracle_rejects_diagnostic_with_mismatched_sdf_provenance() -> None:
    node = _node()

    node._camera_pose_diagnostic_callback(
        "left",
        _diagnostic("left", stamp_ns=100, generated_sdf_sha256="b" * 64),
    )

    assert node.observed_left_camera is None
    assert node.camera_oracle_rejection_count == 1


def test_oracle_rejects_diagnostic_with_missing_sdf_provenance() -> None:
    node = _node()
    message = _diagnostic("left", stamp_ns=100)
    payload = json.loads(message.data)
    del payload["generated_sdf_sha256"]

    node._camera_pose_diagnostic_callback("left", SimpleNamespace(data=json.dumps(payload)))

    assert node.observed_left_camera is None
    assert node.camera_oracle_rejection_count == 1


def test_oracle_rejects_diagnostic_with_mismatched_camera_identity() -> None:
    node = _node()

    node._camera_pose_diagnostic_callback("left", _diagnostic("right", stamp_ns=100))

    assert node.observed_left_camera is None
    assert node.camera_oracle_rejection_count == 1


def test_oracle_rejects_diagnostic_with_zero_length_orientation() -> None:
    node = _node()

    node._camera_pose_diagnostic_callback(
        "left",
        _diagnostic(
            "left",
            stamp_ns=100,
            observed_optical_orientation_world_from_entity_xyzw=[0.0, 0.0, 0.0, 0.0],
        ),
    )

    assert node.observed_left_camera is None
    assert node.camera_oracle_rejection_count == 1


def test_invalid_diagnostic_revokes_a_previously_accepted_camera_observation() -> None:
    node = _node()
    node._camera_pose_diagnostic_callback("left", _diagnostic("left", stamp_ns=100))

    node._camera_pose_diagnostic_callback(
        "left",
        _diagnostic("left", stamp_ns=101, generated_sdf_sha256="b" * 64),
    )

    assert node.observed_left_camera is None


def test_oracle_rejects_observations_from_a_previous_simulation_epoch() -> None:
    node = _node()
    node._camera_pose_diagnostic_callback("left", _diagnostic("left", stamp_ns=100))
    node._camera_pose_diagnostic_callback("right", _diagnostic("right", stamp_ns=200))

    node.simulation_epoch.epoch += 1

    assert node._camera_oracle_snapshot() is None


def test_oracle_rejects_stale_observed_camera_diagnostics(monkeypatch) -> None:
    node = _node()
    monotonic_s = 100.0
    monkeypatch.setattr("stereo_ground_sim.stereo_distance_node.time.monotonic", lambda: monotonic_s)
    node._camera_pose_diagnostic_callback("left", _diagnostic("left", stamp_ns=100))
    node._camera_pose_diagnostic_callback("right", _diagnostic("right", stamp_ns=200))

    monotonic_s = 103.0

    assert node._camera_oracle_snapshot() is None
