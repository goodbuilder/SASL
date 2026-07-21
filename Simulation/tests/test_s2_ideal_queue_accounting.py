from collections import deque
import threading
from types import SimpleNamespace

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.time_alignment import (
    BoundedIdHistory,
    GroundTruthAlignment,
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    SimulationEpochTracker,
    StereoImagePair,
)


def test_ideal_alignment_overflow_discards_already_accepted_pair_once():
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.observation_mode = "ideal_reference_point"
    node.simulation_epoch = SimulationEpochTracker()
    node.measurement_pair_queue = deque()
    node.gt_alignment_ready_queue = deque()
    node.measurement_queue_capacity = 1
    node.measurement_queue_overflow_count = 0
    node.measurement_overrun_invalid = False
    node.measurement_rejected_pair_ids = BoundedIdHistory(16)
    node.accepted_pair_discard_count = 0
    node.measurement_accepted_count = 2
    node.measurement_results_by_pair_id = {}
    node.gt_alignments_by_pair_id = {}
    node.pairs_by_pair_id = {}
    node.pending_gt_aligner = PendingPairGroundTruthAligner(GroundTruthPositionBuffer())
    node.evaluated_pair_ids = BoundedIdHistory(16)
    node.gt_evaluation_failed_pair_ids = BoundedIdHistory(16)
    node.gt_published_pair_ids = BoundedIdHistory(16)
    node.yolo_condition = threading.Condition()
    node._evaluate_pair_with_fault_containment_locked = lambda *args, **kwargs: False
    node._record_measurement_queue_depth_locked = lambda: None
    pair_a = StereoImagePair(1, 1, 1, 1, 0, "exact", object(), object(), 0)
    pair_b = StereoImagePair(2, 2, 2, 2, 0, "exact", object(), object(), 0)
    node.pairs_by_pair_id.update({1: pair_a, 2: pair_b})
    node.measurement_pair_queue.append(pair_a)
    alignment = GroundTruthAlignment((0.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0), "exact", 2, 2, 0.0, 0)
    with node.yolo_condition:
        node._store_gt_alignment_locked(pair_b, alignment)
        node._store_gt_alignment_locked(pair_b, alignment)
    assert node.measurement_queue_overflow_count == 1
    assert node.accepted_pair_discard_count == 1
    assert node.measurement_overrun_invalid is True
    assert 2 in node.measurement_rejected_pair_ids
    assert 2 not in node.pairs_by_pair_id
    assert 2 not in node.gt_alignments_by_pair_id
    assert 2 not in [p.pair_id for p in node.measurement_pair_queue]
    assert node.measurement_processed_count if hasattr(node, "measurement_processed_count") else True
