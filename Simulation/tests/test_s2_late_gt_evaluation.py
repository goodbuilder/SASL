from __future__ import annotations

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import (
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    StereoImagePair,
)


Q = (0.0, 0.0, 0.0, 1.0)


def test_late_gt_interpolates_same_pair_once() -> None:
    gt = GroundTruthPositionBuffer()
    aligner = PendingPairGroundTruthAligner(gt)
    pair = StereoImagePair(7, 149, 151, 150, 2, "nearest", object(), object(), 0)
    assert aligner.add_pair(pair) == []
    gt.add(100, (1.0, 2.0, 3.0), Q)
    assert aligner.pop_watermarked() == []
    gt.add(200, (3.0, 6.0, 9.0), Q)
    ready = aligner.pop_watermarked()
    assert len(ready) == 1
    ready_pair, alignment = ready[0]
    assert ready_pair.pair_id == 7
    assert alignment.gt_alignment_mode == "interpolated"
    assert alignment.gt_interpolation_alpha == 0.5
    assert alignment.position_world_enu_m == (2.0, 4.0, 6.0)
    assert aligner.pop_watermarked() == []
