from __future__ import annotations

import math
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from sasl_common.stereo_geometry import (  # noqa: E402
    BASELINE_CENTER_FRD_AXIS_CONTRACT,
    BASELINE_CENTER_FRD_FRAME,
    BASELINE_CENTER_FRD_VECTOR_CONTRACT,
)
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode  # noqa: E402


def _node_with_camera_error(left_yaw_deg: float = 0.0, right_yaw_deg: float = 0.0):
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    node.synthetic_detection_bbox_size_px = 32.0
    node.left_camera_error_rpy_rad = (0.0, 0.0, math.radians(left_yaw_deg))
    node.right_camera_error_rpy_rad = (0.0, 0.0, math.radians(right_yaw_deg))
    node._ensure_camera_frames()
    return node


def _position_error(actual, expected):
    return math.sqrt(sum((float(actual[index]) - float(expected[index])) ** 2 for index in range(3)))


def test_nonzero_attitude_observation_uses_actual_but_raw_uses_nominal() -> None:
    node = _node_with_camera_error(left_yaw_deg=-0.005, right_yaw_deg=0.005)
    expected = (1000.0, 0.0, 0.0)
    world_point = node.baseline_center_frd.point_to_world(expected)

    left, right = node._detection_pair_from_world(
        world_point,
        1280,
        720,
        projection_camera_mode="actual",
    )
    raw = node._triangulate_pixel_centers(
        left["cx"],
        left["cy"],
        right["cx"],
        right["cy"],
        1280,
        720,
    )
    oracle = node._triangulate_pixel_centers_oracle(
        left["cx"],
        left["cy"],
        right["cx"],
        right["cy"],
        1280,
        720,
    )

    raw_error = _position_error(
        (raw["forward_m"], raw["right_m"], raw["down_m"]),
        expected,
    )
    oracle_error = _position_error(
        (oracle["forward_m"], oracle["right_m"], oracle["down_m"]),
        expected,
    )
    assert raw["output_frame"] == BASELINE_CENTER_FRD_FRAME
    assert raw_error > 1.0e-3
    assert oracle_error < 1.0e-6
    assert raw_error != oracle_error


def test_canonical_raw_result_reports_baseline_frame_provenance() -> None:
    node = _node_with_camera_error()
    left, right = node._synthetic_detection_pair_from_rdf(
        right_m=2.0,
        down_m=1.0,
        forward_m=100.0,
        width=1280,
        height=720,
    )
    result = node._triangulate_pixel_centers(
        left["cx"],
        left["cy"],
        right["cx"],
        right["cy"],
        1280,
        720,
    )
    assert result["point_frame"] == BASELINE_CENTER_FRD_FRAME
    assert result["output_frame"] == BASELINE_CENTER_FRD_FRAME
    assert result["axis_contract"] == BASELINE_CENTER_FRD_AXIS_CONTRACT
    assert result["vector_contract"] == BASELINE_CENTER_FRD_VECTOR_CONTRACT
