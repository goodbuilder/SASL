from __future__ import annotations

from collections import deque

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import (
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    StereoImagePairer,
)


def test_pair_enters_measurement_queue_without_ground_truth() -> None:
    pairer = StereoImagePairer()
    aligner = PendingPairGroundTruthAligner(GroundTruthPositionBuffer())
    measurement_queue = deque()
    assert pairer.add("left", 100, "left") == []
    pairs = pairer.add("right", 100, "right")
    for pair in pairs:
        measurement_queue.append(pair)
        assert aligner.add_pair(pair) == []
    assert len(measurement_queue) == 1
    assert measurement_queue[0].pair_midpoint_stamp_ns == 100
    assert aligner.flush()[0][1].gt_alignment_mode == "unavailable"


def test_ten_pairs_do_not_wait_for_gt_or_overflow() -> None:
    pairer = StereoImagePairer()
    measurement_queue = deque()
    for stamp in range(10):
        pairer.add("left", stamp, f"left-{stamp}")
        measurement_queue.extend(pairer.add("right", stamp, f"right-{stamp}"))
    assert len(measurement_queue) == 10
    assert pairer.paired_count == 10
