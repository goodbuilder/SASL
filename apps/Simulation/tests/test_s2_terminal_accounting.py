from __future__ import annotations

import sys
import threading
from collections import deque
from pathlib import Path

import pytest

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
    def __init__(self, *, fail=False):
        self.fail = fail
        self.messages = []

    def publish(self, message):
        if self.fail:
            raise RuntimeError("deterministic GT evaluation failure")
        self.messages.append(message)


def _node(*, fail_gt_publish=False) -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.lock = threading.Lock()
    node.yolo_condition = threading.Condition(node.lock)
    node.shutdown_event = threading.Event()
    node.accepting_input = True
    node.drain_requested = False
    node.drain_complete = False
    node.yolo_thread = None
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
    node.measurement_accepted_count = 1
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
    node.canonical_ground_truth_position_pub = CapturePublisher(
        fail=fail_gt_publish
    )
    node.baseline_center_frd = build_baseline_center_frd(
        (-1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    node.last_pair_result = None
    node.get_logger = lambda: Logger()
    return node


def _pair() -> StereoImagePair:
    return StereoImagePair(1, 10, 10, 10, 0, "exact", object(), object(), 0)


def _metrics() -> dict:
    return {
        "pair_id": 1,
        "pair_midpoint_stamp_ns": 10,
        "computation_status": "computed",
        "estimated_forward_m": 10.0,
        "estimated_right_m": 0.0,
        "estimated_down_m": 0.0,
        "attitude_corrected_computation_status": "computed",
        "attitude_corrected_forward_m": 10.0,
        "attitude_corrected_right_m": 0.0,
        "attitude_corrected_down_m": 0.0,
    }


def _alignment() -> GroundTruthAlignment:
    return GroundTruthAlignment(
        (0.0, 10.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        "exact",
        10,
        10,
        0.0,
        0,
    )


def _assert_accounting(
    node,
    *,
    processed,
    discarded,
    measurement_errors,
    evaluation_successes,
    evaluation_errors,
    shutdown_status,
):
    assert node.measurement_accepted_count == 1
    assert node.measurement_processed_count == processed
    assert node.accepted_pair_discard_count == discarded
    assert node.measurement_processing_error_count == measurement_errors
    assert node.gt_evaluation_success_count == evaluation_successes
    assert node.gt_evaluation_error_count == evaluation_errors
    assert processed + discarded == node.measurement_accepted_count
    node.shutdown()
    assert node.shutdown_drain_status == shutdown_status


def test_measurement_success_without_gt_is_terminally_accounted() -> None:
    node = _node()
    with node.yolo_condition:
        node._complete_pair_measurement_locked(1, _metrics())

    _assert_accounting(
        node,
        processed=1,
        discarded=0,
        measurement_errors=0,
        evaluation_successes=0,
        evaluation_errors=0,
        shutdown_status="SUCCESS",
    )


def test_measurement_failure_does_not_increment_evaluation_error() -> None:
    node = _node()
    pair = _pair()
    node.pairs_by_pair_id[pair.pair_id] = pair
    node.inflight_pair_id = pair.pair_id
    node.inflight_pair_epoch = pair.epoch
    node.yolo_inference_in_progress = True
    node.pending_gt_aligner.add_pair(pair)
    with node.yolo_condition:
        node._finalize_pair_failure_locked(pair, RuntimeError("compute failed"))

    _assert_accounting(
        node,
        processed=0,
        discarded=1,
        measurement_errors=1,
        evaluation_successes=0,
        evaluation_errors=0,
        shutdown_status="INCOMPLETE",
    )


@pytest.mark.parametrize(
    ("fail_gt_publish", "evaluation_successes", "evaluation_errors", "status"),
    (
        (False, 1, 0, "SUCCESS"),
        (True, 0, 1, "INCOMPLETE"),
    ),
)
def test_measurement_success_and_evaluation_terminal_are_separate(
    fail_gt_publish,
    evaluation_successes,
    evaluation_errors,
    status,
) -> None:
    node = _node(fail_gt_publish=fail_gt_publish)
    pair = _pair()
    node.pairs_by_pair_id[pair.pair_id] = pair
    with node.yolo_condition:
        node._complete_pair_measurement_locked(pair.pair_id, _metrics())
        node._store_gt_alignment_locked(pair, _alignment())
        node._evaluate_pair_with_fault_containment_locked(
            pair.pair_id,
            trigger="duplicate_retry",
        )

    _assert_accounting(
        node,
        processed=1,
        discarded=0,
        measurement_errors=0,
        evaluation_successes=evaluation_successes,
        evaluation_errors=evaluation_errors,
        shutdown_status=status,
    )
