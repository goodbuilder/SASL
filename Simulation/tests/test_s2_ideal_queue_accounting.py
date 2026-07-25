from collections import deque
from pathlib import Path
import sys
import threading

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

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


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _node():
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.observation_mode = "ideal_reference_point"
    node.simulation_epoch = SimulationEpochTracker()
    node.image_pairer = StereoImagePairer()
    node.measurement_pair_queue = deque()
    node.gt_alignment_ready_queue = deque()
    node.measurement_queue_capacity = 1
    node.measurement_queue_overflow_count = 0
    node.measurement_overrun_invalid = False
    node.measurement_rejected_after_overrun_count = 0
    node.measurement_rejected_pair_ids = BoundedIdHistory(16)
    node.last_measurement_rejection_reason = ""
    node.measurement_accepted_count = 0
    node.measurement_processed_count = 0
    node.accepted_pair_discard_count = 0
    node.measurement_processing_error_count = 0
    node.measurement_results_by_pair_id = {}
    node.gt_alignments_by_pair_id = {}
    node.pairs_by_pair_id = {}
    node.pending_gt_aligner = PendingPairGroundTruthAligner(
        GroundTruthPositionBuffer()
    )
    node.evaluated_pair_ids = BoundedIdHistory(16)
    node.gt_evaluation_failed_pair_ids = BoundedIdHistory(16)
    node.gt_published_pair_ids = BoundedIdHistory(16)
    node.measurement_queue_depth_sample_count = 0
    node.measurement_queue_depth_sum = 0
    node.measurement_queue_depth_min = None
    node.measurement_queue_depth_max = 0
    node.old_epoch_frame_count = 0
    node.inflight_pair_id = None
    node.inflight_pair_epoch = None
    node.yolo_inference_in_progress = False
    node.yolo_condition = threading.Condition()
    node._evaluate_pair_with_fault_containment_locked = lambda *args, **kwargs: False
    node.shutdown_event = threading.Event()
    node.accepting_input = True
    node.drain_requested = False
    node.drain_complete = False
    node.yolo_thread = None
    node.discard_measurement_commits = False
    node.gt_evaluation_error_count = 0
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
    node.get_logger = lambda: _Logger()
    node.raw_position_pub = _CapturePublisher()
    node.attitude_corrected_position_pub = _CapturePublisher()
    node.canonical_ground_truth_position_pub = _CapturePublisher()
    node.estimated_position_pub = _CapturePublisher()
    node.detections_pub = _CapturePublisher()
    node.distance_pub = _CapturePublisher()
    node.disparity_pub = _CapturePublisher()
    node.error_pub = _CapturePublisher()
    return node


def _pair(pair_id):
    return StereoImagePair(
        pair_id,
        pair_id,
        pair_id,
        pair_id,
        0,
        "exact",
        object(),
        object(),
        0,
    )


def _alignment(pair_id):
    return GroundTruthAlignment(
        (0.0, float(pair_id), 0.0),
        (0.0, 0.0, 0.0, 1.0),
        "exact",
        pair_id,
        pair_id,
        0.0,
        0,
    )


def _deliver_alignment(node, pair):
    assert node.pending_gt_aligner.remove_pair(pair.pair_id)
    node._store_gt_alignment_locked(pair, _alignment(pair.pair_id))


def _process_next_pair(node, expected_pair_id):
    pair = node.measurement_pair_queue.popleft()
    assert pair.pair_id == expected_pair_id
    node.inflight_pair_id = pair.pair_id
    node.inflight_pair_epoch = pair.epoch
    node.yolo_inference_in_progress = True
    node._complete_pair_measurement_locked(pair.pair_id, {"pair_id": pair.pair_id})
    node._drop_pair_state_locked(pair.pair_id, "test_processed_terminal")
    node.inflight_pair_id = None
    node.inflight_pair_epoch = None
    node.yolo_inference_in_progress = False


def test_ideal_overflow_drains_accepted_pair_and_rejects_new_admission():
    node = _node()
    pair_a, pair_b, pair_c = (_pair(pair_id) for pair_id in (1, 2, 3))

    with node.yolo_condition:
        assert node._register_measurement_pairs_locked([pair_a, pair_b]) == 2
        _deliver_alignment(node, pair_a)
        assert [pair.pair_id for pair in node.measurement_pair_queue] == [1]

        _deliver_alignment(node, pair_b)
        overflow_snapshot = (
            node.measurement_queue_overflow_count,
            node.accepted_pair_discard_count,
        )
        node._store_gt_alignment_locked(pair_b, _alignment(pair_b.pair_id))
        assert (
            node.measurement_queue_overflow_count,
            node.accepted_pair_discard_count,
        ) == overflow_snapshot

        _process_next_pair(node, expected_pair_id=1)
        publish_counts_before_pair_c = {
            name: len(getattr(node, name).messages)
            for name in (
                "raw_position_pub",
                "attitude_corrected_position_pub",
                "canonical_ground_truth_position_pub",
                "estimated_position_pub",
                "detections_pub",
                "distance_pub",
                "disparity_pub",
                "error_pub",
            )
        }
        assert node._register_measurement_pairs_locked([pair_c]) == 0
        publish_counts_after_pair_c = {
            name: len(getattr(node, name).messages)
            for name in publish_counts_before_pair_c
        }

    assert node.measurement_accepted_count == 2
    assert node.measurement_processed_count == 1
    assert node.accepted_pair_discard_count == 1
    assert (
        node.measurement_processed_count + node.accepted_pair_discard_count
        == node.measurement_accepted_count
    )
    assert node.measurement_queue_overflow_count == 1
    assert node.measurement_overrun_invalid is True
    assert node.measurement_rejected_after_overrun_count == 1
    assert node.last_measurement_rejection_reason == (
        "measurement_run_invalid_after_queue_overrun"
    )
    assert 2 in node.measurement_rejected_pair_ids
    assert 3 in node.measurement_rejected_pair_ids
    assert 2 not in node.pairs_by_pair_id
    assert 2 not in node.gt_alignments_by_pair_id
    assert 2 not in node.measurement_results_by_pair_id
    assert 2 not in [pair.pair_id for pair in node.measurement_pair_queue]
    assert 3 not in node.pairs_by_pair_id
    assert 3 not in node.gt_alignments_by_pair_id
    assert 3 not in node.measurement_results_by_pair_id
    assert not any(pair.pair_id == 3 for pair in node.pending_gt_aligner._pending)
    assert publish_counts_after_pair_c == publish_counts_before_pair_c
    assert node.measurement_pair_queue == deque()
    assert node.gt_alignment_ready_queue == deque()
    assert node.inflight_pair_id is None
    assert node.inflight_pair_epoch is None
    assert node.yolo_inference_in_progress is False
    assert node.shutdown_event.is_set() is False

    node.shutdown()

    assert node.drain_complete is True
    assert node.shutdown_drain_status == "SUCCESS"
    assert node.shutdown_remaining_measurement_count == 0
    assert node.shutdown_remaining_gt_alignment_count == 0
    assert node.shutdown_inflight_pair_id is None
    assert node.measurement_overrun_invalid is True


def test_accepted_gt_pending_pair_drains_after_run_is_invalidated():
    node = _node()
    pair_a, pair_b, pair_pending = (_pair(pair_id) for pair_id in (1, 2, 4))

    with node.yolo_condition:
        assert node._register_measurement_pairs_locked(
            [pair_a, pair_b, pair_pending]
        ) == 3
        _deliver_alignment(node, pair_a)
        _deliver_alignment(node, pair_b)
        assert node.measurement_overrun_invalid is True

        _process_next_pair(node, expected_pair_id=1)
        _deliver_alignment(node, pair_pending)
        _process_next_pair(node, expected_pair_id=4)

    assert node.measurement_accepted_count == 3
    assert node.measurement_processed_count == 2
    assert node.accepted_pair_discard_count == 1
    assert node.measurement_rejected_after_overrun_count == 0
    assert (
        node.measurement_processed_count + node.accepted_pair_discard_count
        == node.measurement_accepted_count
    )
    assert node.measurement_pair_queue == deque()
    assert node.gt_alignment_ready_queue == deque()
    assert node.pairs_by_pair_id == {}
    assert node.gt_alignments_by_pair_id == {}
    assert node.inflight_pair_id is None

    node.shutdown()

    assert node.shutdown_drain_status == "SUCCESS"
    assert node.measurement_overrun_invalid is True
