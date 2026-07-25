from __future__ import annotations

import math
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from camera_oracle_test_support import seed_observed_camera_oracle


def _node(common: float, relative: float):
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    node.synthetic_detection_bbox_size_px = 32.0
    node.common_optical_yaw_error_deg = common
    node.relative_optical_yaw_error_deg = relative
    node._ensure_camera_frames()
    seed_observed_camera_oracle(node)
    return node


def _error(result, expected):
    return math.sqrt(sum((result[name] - expected[index]) ** 2 for index, name in enumerate(("forward_m", "right_m", "down_m"))))


def _observe_and_reconstruct(node, expected):
    point_world = node.baseline_center_frd.point_to_world(expected)
    left, right = node._detection_pair_from_world(
        point_world, 1280, 720, projection_camera_mode="gazebo_observed_optical"
    )
    pixels = (left["cx"], left["cy"], right["cx"], right["cy"], 1280, 720)
    return node._triangulate_pixel_centers(*pixels), node._triangulate_pixel_centers_oracle(*pixels)


def test_zero_and_relative_yaw_raw_oracle_contract() -> None:
    expected = (1852.0, 0.0, 0.0)
    raw_zero, corrected_zero = _observe_and_reconstruct(_node(0.0, 0.0), expected)
    assert _error(raw_zero, expected) < 1e-6
    assert _error(corrected_zero, expected) < 1e-6
    raw, corrected = _observe_and_reconstruct(_node(0.0, 0.005), expected)
    assert _error(raw, expected) > 0.1
    assert _error(corrected, expected) < 1e-6

def test_common_yaw_distinguishes_raw_from_oracle() -> None:
    expected = (1000.0, 4.0, -2.0)
    raw, corrected = _observe_and_reconstruct(_node(0.1, 0.0), expected)
    assert _error(raw, expected) > 1e-3
    assert _error(corrected, expected) < 1e-6
