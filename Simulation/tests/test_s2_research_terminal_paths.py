from __future__ import annotations

import math
import sys
import threading
from collections import deque
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import build_baseline_center_frd
from stereo_ground_sim.research_recording import validate_pair_record
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


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class CaptureRecorder:
    def __init__(self):
        self.records = []
        self.queue_depth = 0
        self.queue_depth_max = 0
        self.queue_capacity = 16
        self.overrun_invalid = False
        self.error_count = 0
        self.first_error_type = ""
        self.first_error_message = ""
        self.closed_state = None

    def submit(self, record):
        prepared = dict(record)
        prepared["record_sequence"] = len(self.records) + 1
        self.records.append(validate_pair_record(prepared))
        return True

    def close(self, state):
        self.closed_state = dict(state)
        return {"run_valid": False, "finalization_status": "INVALID"}


def _pair(pair_id=1, *, epoch=0, stamp=100):
    return StereoImagePair(
        pair_id,
        stamp,
        stamp + 2,
        stamp + 1,
        2,
        "nearest",
        object(),
        object(),
        epoch,
    )


def _alignment(*, unavailable=False):
    return GroundTruthAlignment(
        None if unavailable else (0.0, 10.0, 0.0),
        None if unavailable else (0.0, 0.0, 0.0, 1.0),
        "unavailable" if unavailable else "exact",
        None if unavailable else 100,
        None if unavailable else 102,
        None if unavailable else 0.5,
        None if unavailable else 1,
    )


def _metrics(pair):
    return {
        "pair_id": pair.pair_id,
        "epoch": pair.epoch,
        "left_stamp_ns": pair.left_stamp_ns,
        "right_stamp_ns": pair.right_stamp_ns,
        "pair_midpoint_stamp_ns": pair.pair_midpoint_stamp_ns,
        "pair_delta_ns": pair.pair_delta_ns,
        "pairing_mode": pair.pairing_mode,
        "left_u_px": 650.25,
        "left_v_px": 350.5,
        "right_u_px": 620.25,
        "right_v_px": 350.5,
        "left_confidence": None,
        "right_confidence": None,
        "observation_mode": "ideal_reference_point",
        "observation_source": "gazebo_reference_point_projection",
        "measurement_gt_independent": False,
        "reference_source": "vehicle_reference_link",
        "projection_camera_pose_source": "actual",
        "computation_status": "computed",
        "raw_computation_status": "computed",
        "estimated_forward_m": 9.0,
        "estimated_right_m": 1.0,
        "estimated_down_m": 2.0,
        "estimated_range_m": math.sqrt(86.0),
        "raw_forward_m": 9.0,
        "raw_right_m": 1.0,
        "raw_down_m": 2.0,
        "raw_range_m": math.sqrt(86.0),
        "raw_ray_gap_m": 0.01,
        "raw_intersection_angle_deg": 1.0,
        "raw_left_depth_m": 9.0,
        "raw_right_depth_m": 9.0,
        "geometry_rank": 2,
        "condition_number": 4.0,
        "attitude_corrected_computation_status": "computed",
        "attitude_corrected_forward_m": 10.0,
        "attitude_corrected_right_m": 0.0,
        "attitude_corrected_down_m": 0.0,
        "attitude_corrected_range_m": 10.0,
        "attitude_corrected_ray_gap_m": 0.0,
        "attitude_corrected_intersection_angle_deg": 1.1,
        "attitude_corrected_left_depth_m": 10.0,
        "attitude_corrected_right_depth_m": 10.0,
        "attitude_corrected_geometry_rank": 2,
        "attitude_corrected_condition_number": 3.0,
        "measurement_status": "processed",
        "evaluation_status": "pending",
    }


def _node():
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.lock = threading.Lock()
    node.yolo_condition = threading.Condition(node.lock)
    node.shutdown_event = threading.Event()
    node.accepting_input = True
    node.drain_requested = False
    node.drain_complete = False
    node.yolo_thread = None
    node.simulation_epoch = SimulationEpochTracker()
    node.simulation_epoch.observe_clock(200)
    node.image_pairer = StereoImagePairer()
    node.gt_position_buffer = GroundTruthPositionBuffer()
    node.pending_gt_aligner = PendingPairGroundTruthAligner(node.gt_position_buffer)
    node.measurement_pair_queue = deque()
    node.gt_alignment_ready_queue = deque()
    node.measurement_results_by_pair_id = {}
    node.gt_alignments_by_pair_id = {}
    node.pairs_by_pair_id = {}
    node.max_pair_result_history = 32
    node.evaluated_pair_ids = BoundedIdHistory(32)
    node.gt_evaluation_failed_pair_ids = BoundedIdHistory(32)
    node.gt_published_pair_ids = BoundedIdHistory(32)
    node.measurement_queue_capacity = 1
    node.measurement_queue_depth_max = 0
    node.measurement_queue_depth_sample_count = 0
    node.measurement_queue_depth_sum = 0
    node.measurement_queue_depth_min = None
    node.measurement_queue_overflow_count = 0
    node.measurement_overrun_invalid = False
    node.measurement_rejected_after_overrun_count = 0
    node.measurement_rejected_pair_ids = BoundedIdHistory(32)
    node.last_measurement_rejection_reason = ""
    node.measurement_accepted_count = 0
    node.measurement_processed_count = 0
    node.accepted_pair_discard_count = 0
    node.measurement_processing_error_count = 0
    node.last_measurement_error_type = ""
    node.last_measurement_error_message = ""
    node.gt_evaluation_success_count = 0
    node.gt_evaluation_error_count = 0
    node.last_gt_evaluation_error_type = ""
    node.last_gt_evaluation_error_message = ""
    node.last_gt_evaluation_error_trigger = ""
    node.old_epoch_result_discard_count = 0
    node.old_epoch_frame_count = 0
    node.stale_pre_reset_gt_count = 0
    node.future_gt_pending = deque()
    node.gt_quarantine_candidates = deque()
    node.gt_quarantine_last_stamp_ns = None
    node.gt_quarantine_active = False
    node.gt_quarantine_floor_stamp_ns = None
    node.gt_reset_quarantine_duration_ns = 200_000_000
    node.inflight_pair_id = None
    node.inflight_pair_epoch = None
    node.yolo_inference_in_progress = False
    node.discard_measurement_commits = False
    node.ideal_projection_rejected_count = 0
    node.last_ideal_projection_error = ""
    node.last_ideal_projection_pair_id = None
    node.gt_alignment_counts = {
        name: 0 for name in ("exact", "interpolated", "nearest_before", "nearest_after", "unavailable")
    }
    node.baseline_center_frd = build_baseline_center_frd(
        (-1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    node.canonical_ground_truth_position_pub = Publisher()
    node.observation_mode = "ideal_reference_point"
    node.research_run_id = "run"
    node.research_case_id = "case"
    node.research_record_queue_capacity = 16
    node.research_recorder = CaptureRecorder()
    node.research_terminal_pair_keys = set()
    node.research_terminal_record_count = 0
    node.research_recording_overrun_invalid = False
    node.research_recording_error_count = 0
    node.research_recording_first_error_type = ""
    node.research_recording_first_error_message = ""
    node.input_rejected_after_shutdown_count = 0
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
    node.cached_ground_truth_position = None
    node.cached_actual_distance = None
    node.ground_truth_stamp_ns = None
    node.get_logger = lambda: Logger()
    node._ensure_attitude_error_state()
    return node


def test_successful_evaluation_is_one_atomic_eligible_record() -> None:
    node = _node()
    pair = _pair()
    node.measurement_accepted_count = 1
    node.pairs_by_pair_id[pair.pair_id] = pair
    metrics = _metrics(pair)
    alignment = _alignment()
    with node.yolo_condition:
        node._complete_pair_measurement_locked(pair.pair_id, metrics)
        node.gt_alignments_by_pair_id[pair.pair_id] = alignment
        assert node._try_evaluate_pair_locked(pair.pair_id)

    assert len(node.research_recorder.records) == 1
    record = node.research_recorder.records[0]
    assert record["terminal_status"] == "evaluated"
    assert record["sample_eligible"] is True
    assert record["pair_midpoint_stamp_ns"] == (
        record["left_stamp_ns"] + record["right_stamp_ns"]
    ) // 2
    assert record["raw_forward_error_m"] == -1.0
    assert record["raw_right_error_m"] == 1.0
    assert record["raw_down_error_m"] == 2.0
    assert abs(record["raw_position_error_m"] - math.sqrt(6.0)) <= 1e-12
    assert abs(record["raw_range_error_m"] - (math.sqrt(86.0) - 10.0)) <= 1e-12
    assert record["corrected_position_error_m"] == 0.0
    assert node.measurement_accepted_count == node.measurement_processed_count


def test_gt_unavailable_is_recorded_once_and_is_not_sample_eligible() -> None:
    node = _node()
    pair = _pair()
    node.measurement_accepted_count = 1
    node.pairs_by_pair_id[pair.pair_id] = pair
    with node.yolo_condition:
        node._complete_pair_measurement_locked(pair.pair_id, _metrics(pair))
        node.gt_alignments_by_pair_id[pair.pair_id] = _alignment(unavailable=True)
        assert node._try_evaluate_pair_locked(pair.pair_id)
    record = node.research_recorder.records[0]
    assert record["terminal_status"] == "gt_unavailable"
    assert record["sample_eligible"] is False
    assert record["gt_forward_m"] is None
    assert record["raw_position_error_m"] is None


def test_ideal_rejection_and_measurement_exception_are_terminal_once() -> None:
    rejected = _node()
    pair = _pair()
    rejected.measurement_accepted_count = 1
    rejected.pairs_by_pair_id[pair.pair_id] = pair
    rejected.gt_alignments_by_pair_id[pair.pair_id] = _alignment()
    computed = {
        "left_image": np.zeros((8, 8, 3), dtype=np.uint8),
        "right_image": np.zeros((8, 8, 3), dtype=np.uint8),
        "left": None,
        "right": None,
        "ideal_error": "projection_invalid_or_out_of_fov",
    }
    with rejected.yolo_condition:
        assert rejected._commit_pair_measurement_if_current(pair, computed) is None
    assert rejected.accepted_pair_discard_count == 1
    assert rejected.research_recorder.records[0]["terminal_status"] == "ideal_projection_rejected"

    failed = _node()
    failed.measurement_accepted_count = 1
    failed.pairs_by_pair_id[pair.pair_id] = pair
    with failed.yolo_condition:
        failed._finalize_pair_failure_locked(pair, RuntimeError("compute failed"))
    assert failed.accepted_pair_discard_count == 1
    assert failed.measurement_processing_error_count == 1
    assert failed.research_recorder.records[0]["terminal_status"] == "measurement_processing_error"


def test_accepted_queue_overflow_duplicate_alignment_records_once() -> None:
    node = _node()
    pair = _pair(pair_id=2)
    node.measurement_accepted_count = 1
    node.pairs_by_pair_id[pair.pair_id] = pair
    node.gt_alignments_by_pair_id[pair.pair_id] = _alignment()
    with node.yolo_condition:
        assert node._finalize_measurement_queue_overflow_locked(pair.pair_id)
        assert not node._finalize_measurement_queue_overflow_locked(pair.pair_id)
    assert len(node.research_recorder.records) == 1
    assert node.research_recorder.records[0]["terminal_status"] == "measurement_queue_overflow"
    assert node.accepted_pair_discard_count == 1


def test_old_epoch_and_reset_pending_pairs_do_not_duplicate_terminal_records() -> None:
    node = _node()
    pending = _pair(pair_id=3, epoch=0)
    processed = _pair(pair_id=4, epoch=0)
    node.measurement_accepted_count = 2
    node.measurement_processed_count = 1
    node.pairs_by_pair_id = {3: pending, 4: processed}
    node.measurement_results_by_pair_id[4] = _metrics(processed)
    with node.yolo_condition:
        node._reset_for_simulation_epoch_locked(1, 1)
        node.simulation_epoch.epoch = 1
        node._commit_pair_measurement_if_current(pending, {"finished": 0.0})
    assert len(node.research_recorder.records) == 2
    assert {record["terminal_status"] for record in node.research_recorder.records} == {
        "simulation_reset_discarded"
    }
    assert node.accepted_pair_discard_count == 1
    assert node.measurement_processed_count + node.accepted_pair_discard_count == 2


def test_shutdown_timeout_records_every_remaining_accepted_pair(monkeypatch) -> None:
    node = _node()
    pair = _pair(pair_id=8)
    node.measurement_accepted_count = 1
    node.measurement_pair_queue.append(pair)
    node.pairs_by_pair_id[pair.pair_id] = pair
    ticks = iter((0.0, 6.0, 6.0, 6.0, 6.0))
    monkeypatch.setattr("time.monotonic", lambda: next(ticks, 6.0))
    node.shutdown()
    assert node.shutdown_drain_status == "INCOMPLETE"
    assert node.accepted_pair_discard_count == 1
    assert node.research_recorder.records[0]["terminal_status"] == "shutdown_discarded"
    assert node.research_recorder.closed_state["accepted"] == 1


def test_post_overrun_input_is_not_accepted_or_recorded() -> None:
    node = _node()
    node.measurement_overrun_invalid = True
    pair = _pair(pair_id=9)
    with node.yolo_condition:
        assert node._register_measurement_pairs_locked([pair]) == 0
    assert node.measurement_accepted_count == 0
    assert node.measurement_rejected_after_overrun_count == 1
    assert node.research_recorder.records == []
    assert node.pairs_by_pair_id == {}
