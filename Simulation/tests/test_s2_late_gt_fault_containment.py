from __future__ import annotations

import sys
import threading
import time
from collections import deque
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import build_baseline_center_frd
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.time_alignment import (
    BoundedIdHistory,
    GroundTruthAlignment,
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    SimulationEpochTracker,
    StereoImagePair,
    StereoImagePairer,
)


class Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FailFirstPublisher(CapturePublisher):
    def __init__(self):
        super().__init__()
        self.calls = 0

    def publish(self, message):
        self.calls += 1
        if self.calls == 1:
            raise RuntimeError("deterministic canonical GT publish failure")
        super().publish(message)


def _node() -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.lock = threading.Lock()
    node.yolo_condition = threading.Condition(node.lock)
    node.shutdown_event = threading.Event()
    node.accepting_input = True
    node.drain_requested = False
    node.drain_complete = False
    node.simulation_epoch = SimulationEpochTracker()
    node.image_pairer = StereoImagePairer()
    node.gt_position_buffer = GroundTruthPositionBuffer()
    node.pending_gt_aligner = PendingPairGroundTruthAligner(node.gt_position_buffer)
    node.measurement_pair_queue = deque()
    node.gt_alignment_ready_queue = deque()
    node.measurement_results_by_pair_id = {}
    node.gt_alignments_by_pair_id = {}
    node.pairs_by_pair_id = {}
    node.max_pair_result_history = 16
    node.evaluated_pair_ids = BoundedIdHistory(16)
    node.gt_evaluation_failed_pair_ids = BoundedIdHistory(16)
    node.gt_published_pair_ids = BoundedIdHistory(16)
    node.measurement_accepted_count = 0
    node.measurement_processed_count = 0
    node.accepted_pair_discard_count = 0
    node.measurement_processing_error_count = 0
    node.gt_evaluation_error_count = 0
    node.gt_evaluation_success_count = 0
    node.last_gt_evaluation_error_type = ""
    node.last_gt_evaluation_error_message = ""
    node.last_gt_evaluation_error_trigger = ""
    node.inflight_pair_id = None
    node.inflight_pair_epoch = None
    node.yolo_inference_in_progress = False
    node.discard_measurement_commits = False
    node.shutdown_drain_status = "not_requested"
    node.shutdown_drain_duration_s = 0.0
    node.shutdown_remaining_measurement_count = 0
    node.shutdown_remaining_gt_alignment_count = 0
    node.shutdown_inflight_pair_id = None
    node.shutdown_initial_measurement_count = 0
    node.shutdown_initial_gt_alignment_count = 0
    node.shutdown_initial_inflight_pair_id = None
    node.shutdown_gt_evaluation_error_count = 0
    node.dropped_on_shutdown_count = 0
    node.dropped_on_shutdown_pair_id_min = None
    node.dropped_on_shutdown_pair_id_max = None
    node.raw_position_pub = CapturePublisher()
    node.attitude_corrected_position_pub = CapturePublisher()
    node.canonical_ground_truth_position_pub = FailFirstPublisher()
    node.raw_computed_count = 0
    node.corrected_computed_count = 0
    node.last_raw_published_pair_id = 0
    node.last_corrected_published_pair_id = 0
    node.gt_alignment_counts = {
        name: 0
        for name in (
            "exact",
            "interpolated",
            "nearest_before",
            "nearest_after",
            "unavailable",
        )
    }
    node.baseline_center_frd = build_baseline_center_frd(
        (-1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    node.last_pair_result = None
    node.get_logger = lambda: Logger()
    node._warm_yolo_model = lambda: None
    return node


def _pair(pair_id: int, stamp: int) -> StereoImagePair:
    return StereoImagePair(
        pair_id,
        stamp,
        stamp,
        stamp,
        0,
        "exact",
        object(),
        object(),
        0,
    )


def _metrics(pair: StereoImagePair) -> dict:
    return {
        "pair_id": pair.pair_id,
        "pair_midpoint_stamp_ns": pair.pair_midpoint_stamp_ns,
        "computation_status": "computed",
        "estimated_forward_m": 10.0,
        "estimated_right_m": 0.0,
        "estimated_down_m": 0.0,
        "attitude_corrected_computation_status": "computed",
        "attitude_corrected_forward_m": 10.0,
        "attitude_corrected_right_m": 0.0,
        "attitude_corrected_down_m": 0.0,
    }


def _alignment(stamp: int) -> GroundTruthAlignment:
    return GroundTruthAlignment(
        (0.0, 10.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        "exact",
        stamp,
        stamp,
        0.0,
        0,
    )


def test_late_gt_publish_exception_is_contained_and_next_pair_evaluates() -> None:
    node = _node()
    pairs = (_pair(1, 10), _pair(2, 20))
    with node.yolo_condition:
        for pair in pairs:
            node.measurement_accepted_count += 1
            node.pairs_by_pair_id[pair.pair_id] = pair
            metrics = _metrics(pair)
            node._publish_canonical_pair_positions(metrics)
            node._complete_pair_measurement_locked(pair.pair_id, metrics)
            node.gt_alignment_ready_queue.append((pair, _alignment(pair.pair_midpoint_stamp_ns)))

    node.yolo_thread = threading.Thread(target=node._yolo_worker_loop)
    node.yolo_thread.start()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        with node.yolo_condition:
            if node.gt_evaluation_error_count == 1 and node.gt_evaluation_success_count == 1:
                break
        time.sleep(0.01)

    assert node.yolo_thread.is_alive()
    assert len(node.raw_position_pub.messages) == 2
    assert len(node.attitude_corrected_position_pub.messages) == 2
    assert node.measurement_processed_count == 2
    assert node.accepted_pair_discard_count == 0
    assert node.measurement_processing_error_count == 0
    assert node.gt_evaluation_error_count == 1
    assert node.gt_evaluation_success_count == 1
    assert 1 in node.gt_evaluation_failed_pair_ids
    assert 2 in node.evaluated_pair_ids
    assert len(node.canonical_ground_truth_position_pub.messages) == 1
    assert node.measurement_results_by_pair_id == {}
    assert node.gt_alignments_by_pair_id == {}
    assert node.pairs_by_pair_id == {}

    node.shutdown()
    assert node.shutdown_drain_status == "INCOMPLETE"
    assert node.shutdown_gt_evaluation_error_count == 1
    assert node.measurement_pair_queue == deque()
    assert node.gt_alignment_ready_queue == deque()
    assert node.inflight_pair_id is None
    assert not node.yolo_thread.is_alive()


def test_gt_first_then_production_commit_contains_evaluation_failure() -> None:
    node = _node()
    pair = _pair(1, 10)
    node.measurement_accepted_count = 1
    node.pairs_by_pair_id[pair.pair_id] = pair
    with node.yolo_condition:
        node._store_gt_alignment_locked(pair, _alignment(10))

    node.detections_pub = CapturePublisher()
    node.distance_pub = CapturePublisher()
    node.error_pub = CapturePublisher()
    node.disparity_pub = CapturePublisher()
    node.yolo_inference_ms_pub = CapturePublisher()
    node.yolo_update_hz_pub = CapturePublisher()
    node.last_yolo_completion = None
    node.inflight_pair_id = pair.pair_id
    node.inflight_pair_epoch = pair.epoch
    node.yolo_inference_in_progress = True
    node.common_optical_yaw_error_deg = 0.0
    node.relative_optical_yaw_error_deg = 0.0
    node.effective_left_optical_yaw_error_deg = 0.0
    node.effective_right_optical_yaw_error_deg = 0.0
    node._calculate_metrics = lambda *_args: {
        "distance_m": None,
        "distance_error_m": None,
        "disparity_px": None,
    }
    node._detections_to_msg = lambda *_args: object()
    node._publish_estimated_position = lambda *_args: None
    raw = {
        "computation_status": "computed",
        "forward_m": 10.0,
        "right_m": 0.0,
        "down_m": 0.0,
        "range_m": 10.0,
    }
    computed = {
        "left_image": object(),
        "right_image": object(),
        "left": None,
        "right": None,
        "raw": raw,
        "corrected": raw,
        "phase": "test",
        "started": 1.0,
        "finished": 1.1,
    }

    with node.yolo_condition:
        result = node._commit_pair_measurement_if_current(pair, computed)

    assert result is not None
    assert node.measurement_processed_count == 1
    assert node.accepted_pair_discard_count == 0
    assert node.measurement_processing_error_count == 0
    assert node.gt_evaluation_error_count == 1
    assert node.gt_evaluation_success_count == 0
    assert len(node.raw_position_pub.messages) == 1
    assert len(node.attitude_corrected_position_pub.messages) == 1
    assert pair.pair_id in node.gt_evaluation_failed_pair_ids
    assert node.last_pair_result["measurement_status"] == "processed"
    assert node.last_pair_result["evaluation_status"] == "evaluation_error"


def test_evicted_terminal_id_cannot_resurrect_late_alignment_state() -> None:
    node = _node()
    node.gt_evaluation_failed_pair_ids = BoundedIdHistory(2)
    with node.yolo_condition:
        for pair_id in (1, 2, 3):
            node._finalize_gt_evaluation_failure_locked(
                pair_id,
                RuntimeError(f"failure {pair_id}"),
                trigger="test",
            )
        assert 1 not in node.gt_evaluation_failed_pair_ids
        error_count = node.gt_evaluation_error_count
        node._store_gt_alignment_locked(_pair(1, 10), _alignment(10))

    assert node.gt_evaluation_error_count == error_count
    assert node.pairs_by_pair_id == {}
    assert node.gt_alignments_by_pair_id == {}
    assert node.measurement_results_by_pair_id == {}
