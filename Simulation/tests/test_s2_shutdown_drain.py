from __future__ import annotations

import sys
import threading
from collections import deque
from pathlib import Path
from types import MethodType, SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import (
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    SimulationEpochTracker,
    StereoImagePairer,
)


def test_shutdown_flush_routes_exact_nearest_and_unavailable_pairs() -> None:
    pairer = StereoImagePairer()
    aligner = PendingPairGroundTruthAligner(GroundTruthPositionBuffer())
    measurement = []
    pairer.add("left", 10, "exact-left")
    measurement.extend(pairer.add("right", 10, "exact-right"))
    pairer.add("left", 20, "nearest-left")
    pairer.add("right", 22, "nearest-right")
    measurement.extend(pairer.flush())
    for pair in measurement:
        aligner.add_pair(pair)
    evaluations = aligner.flush()
    assert len(measurement) == len(evaluations) == 2
    assert {pair.pair_id for pair in measurement} == {pair.pair_id for pair, _ in evaluations}
    assert all(alignment.gt_alignment_mode == "unavailable" for _, alignment in evaluations)


class Logger:
    def warning(self, _message):
        pass


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_production_worker_drains_ten_gt_unavailable_measurements() -> None:
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
    node.evaluated_pair_ids = set()
    node.raw_published_pair_ids = set()
    node.corrected_published_pair_ids = set()
    node.gt_published_pair_ids = set()
    node.raw_position_pub = CapturePublisher()
    node.attitude_corrected_position_pub = CapturePublisher()
    node.canonical_ground_truth_position_pub = CapturePublisher()
    node.raw_computed_count = 0
    node.corrected_computed_count = 0
    node.max_pair_result_history = 128
    node.gt_alignment_counts = {name: 0 for name in ("exact", "interpolated", "nearest_before", "nearest_after", "unavailable")}
    node.old_epoch_frame_count = 0
    node.cached_ground_truth_position = None
    node.cached_actual_distance = None
    node.ground_truth_stamp_ns = None
    node.yolo_target_hz = 1_000_000.0
    node.yolo_inference_in_progress = False
    node.last_yolo_update = 0.0
    processed = []
    node._warm_yolo_model = lambda: None
    node._mission_state = lambda: {"phase": "test"}
    node.get_logger = lambda: Logger()
    node._ground_truth_alignment_to_baseline = lambda alignment: None

    def update(self, _left, _right, _actual, _phase, **kwargs):
        pair = kwargs["pair"]
        processed.append(pair.pair_id)
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
        return metrics

    node._update_metrics_from_messages = MethodType(update, node)
    pairs = []
    for stamp in range(10):
        node.image_pairer.add("left", stamp, SimpleNamespace(side="left"))
        pairs.extend(node.image_pairer.add("right", stamp, SimpleNamespace(side="right")))
    with node.yolo_condition:
        node._register_measurement_pairs_locked(pairs)
    node.yolo_thread = threading.Thread(target=node._yolo_worker_loop)
    node.yolo_thread.start()
    node.shutdown()
    assert node.drain_complete
    assert sorted(processed) == list(range(1, 11))
    assert node.gt_alignment_counts["unavailable"] == 10
    assert len(node.raw_position_pub.messages) == 10
    assert len(node.attitude_corrected_position_pub.messages) == 10
    assert node.canonical_ground_truth_position_pub.messages == []
    assert node.measurement_pair_queue == deque()
    assert node.gt_alignment_ready_queue == deque()
