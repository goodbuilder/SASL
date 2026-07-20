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
    SimulationEpochTracker,
    StereoImagePair,
)
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


class Logger:
    def error(self, _message):
        pass


def test_strict_research_queue_overrun_is_bounded_and_observable() -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.lock = threading.Lock()
    node.yolo_condition = threading.Condition(node.lock)
    node.simulation_epoch = SimulationEpochTracker()
    node.measurement_queue_capacity = 2
    node.measurement_pair_queue = deque()
    node.gt_alignment_ready_queue = deque()
    node.gt_position_buffer = GroundTruthPositionBuffer()
    node.pending_gt_aligner = PendingPairGroundTruthAligner(node.gt_position_buffer)
    node.pairs_by_pair_id = {}
    node.measurement_results_by_pair_id = {}
    node.gt_alignments_by_pair_id = {}
    node.evaluated_pair_ids = BoundedIdHistory(8)
    node.gt_published_pair_ids = BoundedIdHistory(8)
    node.measurement_rejected_pair_ids = BoundedIdHistory(8)
    node.measurement_queue_overflow_count = 0
    node.measurement_overrun_invalid = False
    node.measurement_accepted_count = 0
    node.measurement_queue_depth_sample_count = 0
    node.measurement_queue_depth_sum = 0
    node.measurement_queue_depth_min = None
    node.measurement_queue_depth_max = 0
    node.old_epoch_frame_count = 0
    node.get_logger = lambda: Logger()
    pairs = [
        StereoImagePair(i, i, i, i, 0, "exact", object(), object(), 0)
        for i in range(1, 5)
    ]
    with node.yolo_condition:
        accepted = node._register_measurement_pairs_locked(pairs)
    assert accepted == 2
    assert [pair.pair_id for pair in node.measurement_pair_queue] == [1, 2]
    assert node.measurement_queue_depth_max == 2
    assert node.measurement_queue_overflow_count == 2
    assert node.measurement_overrun_invalid
    assert all(pair_id in node.measurement_rejected_pair_ids for pair_id in (3, 4))
    assert not any(pair.pair_id in (3, 4) for pair in node.pending_gt_aligner._pending)
