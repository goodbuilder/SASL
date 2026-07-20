from __future__ import annotations

import threading
import sys
from collections import deque
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import (
    BoundedIdHistory,
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    SimulationEpochTracker,
    StereoImagePairer,
)
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.coordinate_frames import build_baseline_center_frd


class Logger:
    def warning(self, _message):
        pass


def _pose(stamp: int):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="gazebo_world_enu",
            stamp=SimpleNamespace(sec=0, nanosec=stamp),
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=10.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _node():
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.lock = threading.Lock()
    node.yolo_condition = threading.Condition(node.lock)
    node.accepting_input = True
    node.simulation_epoch = SimulationEpochTracker()
    node.image_pairer = StereoImagePairer()
    node.gt_position_buffer = GroundTruthPositionBuffer()
    node.pending_gt_aligner = PendingPairGroundTruthAligner(node.gt_position_buffer)
    node.measurement_pair_queue = deque()
    node.gt_alignment_ready_queue = deque()
    node.measurement_results_by_pair_id = {}
    node.gt_alignments_by_pair_id = {}
    node.pairs_by_pair_id = {}
    node.evaluated_pair_ids = BoundedIdHistory(8)
    node.gt_published_pair_ids = BoundedIdHistory(8)
    node.future_gt_pending = deque()
    node.future_gt_capacity = 8
    node.gt_reorder_lead_limit_ns = 4
    node.future_gt_overflow_count = 0
    node.stale_pre_reset_gt_count = 0
    node.inconsistent_clock_count = 0
    node.old_epoch_frame_count = 0
    node.accepted_pair_discard_count = 0
    node.cached_ground_truth_position = None
    node.cached_actual_distance = None
    node.ground_truth_stamp_ns = None
    node.actual_distance_sample_seq = 0
    node.get_logger = lambda: Logger()
    node._ensure_camera_frames = lambda: None
    node.baseline_center_frd = build_baseline_center_frd(
        (-1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    return node


def test_future_gt_reorders_but_stale_pre_reset_gt_is_rejected() -> None:
    node = _node()
    node.simulation_epoch.observe_clock(100)
    node._ground_truth_pose_cb(_pose(104))
    assert len(node.future_gt_pending) == 1
    with node.yolo_condition:
        node._drain_future_ground_truth_locked(104)
    assert len(node.future_gt_pending) == 0
    assert node.gt_position_buffer.latest_stamp_ns == 104

    with node.yolo_condition:
        event = node.simulation_epoch.observe_clock(1)
        node._reset_for_simulation_epoch_locked(event.epoch)
    node._ground_truth_pose_cb(_pose(100))
    assert node.stale_pre_reset_gt_count == 1
    assert node.gt_position_buffer.latest_stamp_ns is None
    node._ground_truth_pose_cb(_pose(1))
    assert node.gt_position_buffer.latest_stamp_ns == 1
    with node.yolo_condition:
        node.simulation_epoch.observe_clock(2)
    node._ground_truth_pose_cb(_pose(2))
    assert node.gt_position_buffer.latest_stamp_ns == 2
    assert node.simulation_epoch.reset_count == node.gt_position_buffer.reset_count == 1
