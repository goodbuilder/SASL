from __future__ import annotations

import sys
import threading
from collections import deque
from pathlib import Path
from types import MethodType, SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.time_alignment import (
    BoundedIdHistory,
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    SimulationEpochTracker,
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
    node.evaluated_pair_ids = BoundedIdHistory(16)
    node.gt_evaluation_failed_pair_ids = BoundedIdHistory(16)
    node.gt_published_pair_ids = BoundedIdHistory(16)
    node.measurement_rejected_pair_ids = BoundedIdHistory(16)
    node.measurement_queue_capacity = 8
    node.measurement_queue_overflow_count = 0
    node.measurement_overrun_invalid = False
    node.measurement_accepted_count = 0
    node.measurement_processed_count = 0
    node.accepted_pair_discard_count = 0
    node.measurement_processing_error_count = 0
    node.last_measurement_error_type = ""
    node.last_measurement_error_message = ""
    node.gt_evaluation_error_count = 0
    node.gt_evaluation_success_count = 0
    node.last_gt_evaluation_error_type = ""
    node.last_gt_evaluation_error_message = ""
    node.last_gt_evaluation_error_trigger = ""
    node.measurement_queue_depth_sample_count = 0
    node.measurement_queue_depth_sum = 0
    node.measurement_queue_depth_min = None
    node.measurement_queue_depth_max = 0
    node.inflight_pair_id = None
    node.inflight_pair_epoch = None
    node.old_epoch_result_discard_count = 0
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
    node.yolo_target_hz = 1_000_000.0
    node.yolo_inference_in_progress = False
    node.last_yolo_update = 0.0
    node.raw_position_pub = CapturePublisher()
    node.attitude_corrected_position_pub = CapturePublisher()
    node.raw_computed_count = 0
    node.corrected_computed_count = 0
    node.last_raw_published_pair_id = 0
    node.last_corrected_published_pair_id = 0
    node.get_logger = lambda: Logger()
    node._warm_yolo_model = lambda: None
    node._mission_state = lambda: {"phase": "test"}
    return node


def test_compute_failure_is_contained_and_same_worker_processes_next_pair() -> None:
    node = _node()
    second_committed = threading.Event()

    def compute(self, pair, _phase, device=None):
        del self, device
        if pair.pair_id == 1:
            raise RuntimeError("deterministic compute failure")
        return {"pair_id": pair.pair_id}

    def commit(self, pair, _computed):
        metrics = {
            "pair_id": pair.pair_id,
            "pair_midpoint_stamp_ns": pair.pair_midpoint_stamp_ns,
            "computation_status": "computed",
            "estimated_forward_m": 1.0,
            "estimated_right_m": 0.0,
            "estimated_down_m": 0.0,
            "attitude_corrected_computation_status": "computed",
            "attitude_corrected_forward_m": 1.0,
            "attitude_corrected_right_m": 0.0,
            "attitude_corrected_down_m": 0.0,
        }
        self._publish_canonical_pair_positions(metrics)
        self.measurement_processed_count += 1
        self.measurement_results_by_pair_id[pair.pair_id] = metrics
        self.pending_gt_aligner.remove_pair(pair.pair_id)
        self.pairs_by_pair_id.pop(pair.pair_id, None)
        self.inflight_pair_id = None
        self.inflight_pair_epoch = None
        self.yolo_inference_in_progress = False
        second_committed.set()
        self.yolo_condition.notify_all()
        return metrics

    node._compute_pair_measurement = MethodType(compute, node)
    node._commit_pair_measurement_if_current = MethodType(commit, node)
    pairs = []
    for stamp in (10, 20):
        node.image_pairer.add("left", stamp, SimpleNamespace(side="left"))
        pairs.extend(node.image_pairer.add("right", stamp, SimpleNamespace(side="right")))
    with node.yolo_condition:
        node._register_measurement_pairs_locked(pairs)

    node.yolo_thread = threading.Thread(target=node._yolo_worker_loop)
    node.yolo_thread.start()
    assert second_committed.wait(timeout=2.0)
    assert node.yolo_thread.is_alive()
    assert node.measurement_processing_error_count == 1
    assert node.accepted_pair_discard_count == 1
    assert node.measurement_processed_count == 1
    assert node.inflight_pair_id is None
    assert node.inflight_pair_epoch is None
    assert not node.yolo_inference_in_progress
    assert 1 not in node.pairs_by_pair_id
    assert 1 not in node.measurement_results_by_pair_id
    assert len(node.raw_position_pub.messages) == 1
    assert len(node.attitude_corrected_position_pub.messages) == 1
    assert node.last_measurement_error_type == "RuntimeError"

    node.shutdown()
    assert node.shutdown_drain_status == "INCOMPLETE"
    assert node.measurement_pair_queue == deque()
    assert node.inflight_pair_id is None
    assert not node.yolo_thread.is_alive()


def test_commit_evaluation_failure_preserves_measurement_and_worker_continues() -> None:
    node = _node()
    second_committed = threading.Event()

    def compute(self, pair, _phase, device=None):
        del self, device
        return {"pair_id": pair.pair_id}

    def evaluate(self, pair_id):
        if pair_id == 1:
            raise RuntimeError("deterministic evaluation failure")
        return False

    def commit(self, pair, _computed):
        self.inflight_pair_id = None
        self.inflight_pair_epoch = None
        self.yolo_inference_in_progress = False
        metrics = {
            "pair_id": pair.pair_id,
            "pair_midpoint_stamp_ns": pair.pair_midpoint_stamp_ns,
            "computation_status": "computed",
            "estimated_forward_m": 1.0,
            "estimated_right_m": 0.0,
            "estimated_down_m": 0.0,
            "attitude_corrected_computation_status": "computed",
            "attitude_corrected_forward_m": 1.0,
            "attitude_corrected_right_m": 0.0,
            "attitude_corrected_down_m": 0.0,
        }
        self._publish_canonical_pair_positions(metrics)
        self._complete_pair_measurement_locked(pair.pair_id, metrics)
        self._evaluate_pair_with_fault_containment_locked(
            pair.pair_id,
            trigger="measurement_complete",
        )
        self.pending_gt_aligner.remove_pair(pair.pair_id)
        self.pairs_by_pair_id.pop(pair.pair_id, None)
        second_committed.set()
        self.yolo_condition.notify_all()
        return metrics

    node._compute_pair_measurement = MethodType(compute, node)
    node._try_evaluate_pair_locked = MethodType(evaluate, node)
    node._commit_pair_measurement_if_current = MethodType(commit, node)
    pairs = []
    for stamp in (10, 20):
        node.image_pairer.add("left", stamp, SimpleNamespace(side="left"))
        pairs.extend(node.image_pairer.add("right", stamp, SimpleNamespace(side="right")))
    with node.yolo_condition:
        node._register_measurement_pairs_locked(pairs)

    node.yolo_thread = threading.Thread(target=node._yolo_worker_loop)
    node.yolo_thread.start()
    assert second_committed.wait(timeout=2.0)
    assert node.yolo_thread.is_alive()
    assert node.measurement_accepted_count == 2
    assert node.measurement_processed_count == 2
    assert node.accepted_pair_discard_count == 0
    assert node.measurement_processing_error_count == 0
    assert node.gt_evaluation_error_count == 1
    assert node.measurement_processed_count + node.accepted_pair_discard_count == 2
    assert 1 not in node.measurement_results_by_pair_id
    assert node.inflight_pair_id is None
    assert node.inflight_pair_epoch is None
    assert not node.yolo_inference_in_progress

    node.shutdown()
    assert node.shutdown_drain_status == "INCOMPLETE"
    assert not node.yolo_thread.is_alive()
