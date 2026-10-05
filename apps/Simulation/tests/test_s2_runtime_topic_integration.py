from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from types import MethodType

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.coordinate_frames import build_baseline_center_frd, nominal_camera_poses
from stereo_ground_sim.gps_baseline import RawGpsFix
from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import (
    BoundedIdHistory,
    GroundTruthAlignment,
    StereoImagePair,
)


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _node_harness():
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.nominal_pose_position_pub = CapturePublisher()
    node.attitude_corrected_position_pub = CapturePublisher()
    node.canonical_ground_truth_position_pub = CapturePublisher()
    node.gps_baseline_position_pub = CapturePublisher()
    node.global_baseline_position_pub = CapturePublisher()
    node.gps_baseline_plot_pub = CapturePublisher()
    node.global_baseline_plot_pub = CapturePublisher()
    node.uam_raw_gps_reference_position_pub = CapturePublisher()
    node.uam_raw_gps_reference_plot_pub = CapturePublisher()
    node.last_raw_published_pair_id = 0
    node.last_corrected_published_pair_id = 0
    node.last_gps_baseline_published_pair_id = 0
    node.last_gps_baseline_published_pair_id_by_source = {
        "gps_baseline": 0,
        "global_baseline": 0,
    }
    node.last_uam_raw_gps_reference_published_pair_id = 0
    node.gt_published_pair_ids = BoundedIdHistory(128)
    node.raw_computed_count = 0
    node.corrected_computed_count = 0
    node.evaluated_pair_ids = BoundedIdHistory(128)
    node.measurement_results_by_pair_id = {}
    node.gt_alignments_by_pair_id = {}
    node.pairs_by_pair_id = {}
    node.gt_alignment_counts = {
        "exact": 0,
        "interpolated": 0,
        "nearest_before": 0,
        "nearest_after": 0,
        "unavailable": 0,
    }
    node.last_pair_result = None
    node._ground_truth_alignment_to_baseline = MethodType(
        lambda self, alignment: None
        if alignment.position_world_enu_m is None
        else {
            "forward_m": alignment.position_world_enu_m[0],
            "right_m": alignment.position_world_enu_m[1],
            "down_m": alignment.position_world_enu_m[2],
            "range_m": sum(value * value for value in alignment.position_world_enu_m) ** 0.5,
        },
        node,
    )
    return node


def test_raw_and_corrected_publish_before_late_gt_on_same_pair_stamp() -> None:
    node = _node_harness()
    pair = StereoImagePair(5, 100, 102, 101, 2, "nearest", object(), object(), 0)
    metrics = {
        "pair_id": 5,
        "pair_midpoint_stamp_ns": 101,
        "computation_status": "computed",
        "estimated_forward_m": 10.0,
        "estimated_right_m": 2.0,
        "estimated_down_m": -1.0,
        "attitude_corrected_computation_status": "computed",
        "attitude_corrected_forward_m": 9.5,
        "attitude_corrected_right_m": 2.0,
        "attitude_corrected_down_m": -1.0,
    }
    node._publish_canonical_pair_positions(metrics)
    node._publish_canonical_pair_positions(metrics)
    assert len(node.nominal_pose_position_pub.messages) == 1
    assert len(node.attitude_corrected_position_pub.messages) == 1
    assert node.canonical_ground_truth_position_pub.messages == []

    node.pairs_by_pair_id[5] = pair
    node.measurement_results_by_pair_id[5] = metrics
    node.gt_alignments_by_pair_id[5] = GroundTruthAlignment(
        (9.0, 2.0, -1.0),
        None,
        "interpolated",
        50,
        152,
        0.5,
        None,
    )
    assert node._try_evaluate_pair_locked(5)
    assert not node._try_evaluate_pair_locked(5)
    assert len(node.canonical_ground_truth_position_pub.messages) == 1
    messages = [
            node.nominal_pose_position_pub.messages[0],
        node.attitude_corrected_position_pub.messages[0],
        node.canonical_ground_truth_position_pub.messages[0],
    ]
    assert [(msg.header.stamp.sec, msg.header.stamp.nanosec) for msg in messages] == [(0, 101)] * 3
    assert {msg.header.frame_id for msg in messages} == {"baseline_center_frd"}
    assert (messages[0].point.x, messages[0].point.y, messages[0].point.z) == (10.0, 2.0, -1.0)
    assert node.last_pair_result["raw_position_error_m"] > 0.0
    assert node.last_pair_result["corrected_position_error_m"] > 0.0


def test_gps_baseline_plot_arrays_preserve_public_frd_order_without_header_time() -> None:
    node = _node_harness()
    metrics = {
        "pair_id": 6,
        "pair_midpoint_stamp_ns": 200,
        "gps_baseline_computation_status": "unavailable",
        "gps_baseline_computation_status": "computed",
        "gps_baseline_forward_m": 1002.0,
        "gps_baseline_right_m": 6.0,
        "gps_baseline_down_m": 5.0,
        "global_baseline_computation_status": "computed",
        "global_baseline_forward_m": 1000.0,
        "global_baseline_right_m": 5.5,
        "global_baseline_down_m": 4.5,
        "uam_raw_gps_reference_computation_status": "computed",
        "uam_raw_gps_reference_forward_m": 1001.0,
        "uam_raw_gps_reference_right_m": 5.0,
        "uam_raw_gps_reference_down_m": 4.0,
    }

    node._publish_canonical_pair_positions(metrics)
    node._publish_canonical_pair_positions(metrics)

    assert len(node.gps_baseline_position_pub.messages) == 1
    assert len(node.global_baseline_position_pub.messages) == 1
    assert len(node.uam_raw_gps_reference_position_pub.messages) == 1
    # ROS-generated Float32MultiArray uses array('f'); fallback test messages
    # may use list. Normalize only the container while preserving FRD values.
    assert list(node.gps_baseline_plot_pub.messages[0].data) == [1002.0, 6.0, 5.0]
    assert list(node.uam_raw_gps_reference_plot_pub.messages[0].data) == [1001.0, 5.0, 4.0]
    assert list(node.global_baseline_plot_pub.messages[0].data) == [1000.0, 5.5, 4.5]


def test_uam_raw_gps_reference_uses_left_local_enu_then_public_frd() -> None:
    node = _node_harness()
    left, right = nominal_camera_poses(25.0, 10.0)
    node.enable_gps_baseline_triangulation = True
    node.lock = threading.RLock()
    node.gps_baseline_max_age_s = 2.5
    node.nominal_left_camera = left
    node.baseline_center_frd = build_baseline_center_frd(
        left.center_world_enu_m,
        right.center_world_enu_m,
        left.forward_world_enu,
        right.forward_world_enu,
    )
    left_fix = RawGpsFix(1, 47.3979712, 8.5459985, 10.0, 0.9, 1.8)
    uam_fix = RawGpsFix(1, 47.3979712, 8.5463293, 22.0, 0.9, 1.8)
    node.gps_baseline_fixes = {
        "left": (left_fix, time.monotonic()),
        "uam": (uam_fix, time.monotonic()),
    }

    reference = node._uam_raw_gps_reference()

    assert reference["computation_status"] == "computed"
    assert abs(reference["forward_m"]) < 0.02
    assert abs(reference["right_m"] - 12.47) < 0.2
    assert abs(reference["down_m"] + 12.0) < 0.02
