from __future__ import annotations

from collections import deque
import math
from pathlib import Path
import sys

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


def _node_for_metrics_test() -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    node.synthetic_detection_bbox_size_px = 32.0
    node.left_camera_error_rpy_rad = (0.0, 0.0, 0.0)
    node.right_camera_error_rpy_rad = (0.0, 0.0, 0.0)
    node._ensure_camera_frames()
    node.bbox_center_adjustment = None
    node.disparity_residual_profile = None
    node.disparity_residual_feature_selector = None
    node.disparity_residual_feature_selector_window = deque()
    node.yolo_class_name = "drone"
    node.yolo_load_error = ""
    node._adjust_bbox_centers = lambda left, right: None
    node._disparity_residual_feature_summary = lambda **kwargs: ({}, "disabled")
    node._apply_disparity_residual_profile = lambda corrected, raw, profile=None: (corrected, "disabled")
    return node


def test_observation_fusion_cannot_overwrite_canonical_raw() -> None:
    node = _node_for_metrics_test()
    left, right = node._synthetic_detection_pair_from_rdf(
        right_m=3.0,
        down_m=1.0,
        forward_m=100.0,
        width=1280,
        height=720,
    )
    canonical = node._triangulate_center_pixels(
        left["cx"], left["cy"], right["cx"], right["cy"], 1280, 720
    )
    fused = {
        "right_m": 77.0,
        "down_m": -11.0,
        "forward_m": 999.0,
        "range_m": math.sqrt(77.0**2 + (-11.0) ** 2 + 999.0**2),
        "distance_m": 999.0,
        "disparity_px": 0.25,
    }
    node._apply_observation_fusion = lambda left, right, width, height: (fused, "mocked_fusion")

    metrics = node._calculate_metrics(
        np.zeros((720, 1280, 3), dtype=np.uint8),
        left,
        right,
        None,
        "test",
    )

    assert metrics["observation_fusion_status"] == "mocked_fusion"
    assert metrics["estimated_forward_m"] == canonical["forward_m"]
    assert metrics["estimated_right_m"] == canonical["right_m"]
    assert metrics["estimated_down_m"] == canonical["down_m"]
    assert metrics["legacy_fused_raw"] is fused
    assert metrics["corrected_forward_m"] == fused["forward_m"]
    assert metrics["estimated_forward_m"] != fused["forward_m"]
