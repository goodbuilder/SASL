from __future__ import annotations

import sys
import threading
from collections import deque
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import build_baseline_center_frd
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.time_alignment import (
    BoundedIdHistory,
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    SimulationEpochTracker,
    StereoImagePairer,
)


class Logger:
    def warning(self, _message):
        pass


def _pose(stamp_ns: int):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="gazebo_world_enu",
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=10.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _node() -> StereoDistanceNode:
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
    node.gt_reorder_lead_limit_ns = 200_000_000
    node.future_gt_overflow_count = 0
    node.stale_pre_reset_gt_count = 0
    node.gt_reset_quarantine_duration_ns = 200_000_000
    node.gt_quarantine_required_contiguous_samples = 2
    node.gt_quarantine_active = False
    node.gt_quarantine_floor_stamp_ns = None
    node.gt_quarantine_candidates = deque()
    node.gt_quarantine_last_stamp_ns = None
    node.gt_quarantine_rejected_count = 0
    node.gt_quarantine_release_count = 0
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


def test_normal_future_gt_keeps_receive_epoch_and_is_accepted() -> None:
    node = _node()
    node.simulation_epoch.observe_clock(100_000_000)
    node._ground_truth_pose_cb(_pose(120_000_000))
    assert len(node.future_gt_pending) == 1
    assert node.future_gt_pending[0].epoch == 0
    with node.yolo_condition:
        node.simulation_epoch.observe_clock(120_000_000)
        node._drain_future_ground_truth_locked(120_000_000)
    assert node.gt_position_buffer.latest_stamp_ns == 120_000_000


def test_reset_discards_pre_reset_pending_gt() -> None:
    node = _node()
    node.simulation_epoch.observe_clock(100_000_000)
    node._ground_truth_pose_cb(_pose(150_000_000))
    assert len(node.future_gt_pending) == 1
    with node.yolo_condition:
        event = node.simulation_epoch.observe_clock(0)
        node._reset_for_simulation_epoch_locked(event.epoch, event.stamp_ns)
    assert node.future_gt_pending == deque()
    assert node.gt_position_buffer.latest_stamp_ns is None
    assert node.stale_pre_reset_gt_count == 1


def test_reset_quarantine_rejects_old_gt_within_window_then_recovers_alignment() -> None:
    node = _node()
    node.simulation_epoch.observe_clock(1_000_000_000)
    with node.yolo_condition:
        event = node.simulation_epoch.observe_clock(0)
        node._reset_for_simulation_epoch_locked(event.epoch, event.stamp_ns)

    node._ground_truth_pose_cb(_pose(100_000_000))
    assert node.gt_position_buffer.latest_stamp_ns is None
    assert node.gt_quarantine_rejected_count == 1

    with node.yolo_condition:
        node.simulation_epoch.observe_clock(200_000_000)
    node._ground_truth_pose_cb(_pose(220_000_000))
    node._ground_truth_pose_cb(_pose(240_000_000))
    assert not node.gt_quarantine_active
    assert node.gt_quarantine_release_count == 1
    with node.yolo_condition:
        node.simulation_epoch.observe_clock(240_000_000)
        node._drain_future_ground_truth_locked(240_000_000)

    assert node.gt_position_buffer.latest_stamp_ns == 240_000_000
    alignment = node.gt_position_buffer.align(230_000_000, epoch=1)
    assert alignment.gt_alignment_mode == "interpolated"
    assert alignment.gt_interpolation_alpha == 0.5
    assert node.stale_pre_reset_gt_count == 1
