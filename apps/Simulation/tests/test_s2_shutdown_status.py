from __future__ import annotations

import threading
import sys
from collections import deque
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import (
    BoundedIdHistory,
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    StereoImagePair,
    StereoImagePairer,
)
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


class Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


def test_shutdown_timeout_reports_incomplete_and_remaining_pair(monkeypatch) -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.lock = threading.Lock()
    node.yolo_condition = threading.Condition(node.lock)
    node.shutdown_event = threading.Event()
    node.accepting_input = True
    node.drain_requested = False
    node.drain_complete = False
    node.image_pairer = StereoImagePairer()
    node.gt_position_buffer = GroundTruthPositionBuffer()
    node.pending_gt_aligner = PendingPairGroundTruthAligner(node.gt_position_buffer)
    pair = StereoImagePair(3, 1, 1, 1, 0, "exact", object(), object(), 0)
    node.measurement_pair_queue = deque([pair])
    node.gt_alignment_ready_queue = deque()
    node.pairs_by_pair_id = {3: pair}
    node.measurement_results_by_pair_id = {}
    node.gt_alignments_by_pair_id = {}
    node.evaluated_pair_ids = BoundedIdHistory(8)
    node.gt_published_pair_ids = BoundedIdHistory(8)
    node.inflight_pair_id = None
    node.inflight_pair_epoch = None
    node.discard_measurement_commits = False
    node.measurement_accepted_count = 1
    node.measurement_processed_count = 0
    node.measurement_processing_error_count = 0
    node.accepted_pair_discard_count = 0
    node.gt_evaluation_error_count = 0
    node.shutdown_gt_evaluation_error_count = 0
    node.shutdown_drain_status = "not_requested"
    node.shutdown_drain_duration_s = 0.0
    node.shutdown_remaining_measurement_count = 0
    node.shutdown_remaining_gt_alignment_count = 0
    node.shutdown_inflight_pair_id = None
    node.dropped_on_shutdown_count = 0
    node.dropped_on_shutdown_pair_id_min = None
    node.dropped_on_shutdown_pair_id_max = None
    node.get_logger = lambda: Logger()
    node.yolo_thread = None

    ticks = iter((0.0, 6.0, 6.0, 6.0, 6.0))
    monkeypatch.setattr("time.monotonic", lambda: next(ticks, 6.0))
    node.shutdown()
    assert node.shutdown_drain_status == "INCOMPLETE"
    assert node.shutdown_remaining_measurement_count == 1
    assert node.dropped_on_shutdown_count == 1
    assert node.dropped_on_shutdown_pair_id_min == 3
    assert node.dropped_on_shutdown_pair_id_max == 3
    assert node.discard_measurement_commits
