from __future__ import annotations

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import (
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    SimulationEpochTracker,
    StereoImagePair,
    StereoImagePairer,
)
from Simulation.stereo_ground_sim.stereo_ground_sim.gazebo_ground_truth_adapter import PoseVState
from pathlib import Path
import pytest


Q = (0.0, 0.0, 0.0, 1.0)


def test_clock_reset_clears_pending_and_never_cross_pairs_epochs() -> None:
    tracker = SimulationEpochTracker()
    pairer = StereoImagePairer()
    tracker.observe_clock(100)
    pairer.add("left", 100, "old-left", epoch=tracker.epoch)
    event = tracker.observe_clock(10)
    pairer.reset_for_epoch(event.epoch)
    pairer.add("left", 10, "new-left", epoch=tracker.epoch)
    pairs = pairer.add("right", 10, "new-right", epoch=tracker.epoch)
    pairs.extend(pairer.flush())
    assert pairer.image_reset_count == 1
    assert pairer.unmatched_left_count == 1
    assert len(pairs) == 1
    assert pairs[0].epoch == 1
    assert pairs[0].left_message == "new-left"


def test_gt_reset_is_externally_driven_and_rejects_old_pair_epoch() -> None:
    buffer = GroundTruthPositionBuffer()
    buffer.add(100, (1.0, 0.0, 0.0), Q)
    with pytest.raises(ValueError, match="without a /clock reset"):
        buffer.add(10, (2.0, 0.0, 0.0), Q)
    buffer.reset_for_epoch(1)
    buffer.add(10, (2.0, 0.0, 0.0), Q, epoch=1)
    assert buffer.reset_count == 1
    assert buffer.align(10, 0).gt_alignment_mode == "unavailable"
    assert buffer.align(10, 1).position_world_enu_m == (2.0, 0.0, 0.0)


def test_image_reset_can_clear_pending_gt_pairs() -> None:
    buffer = GroundTruthPositionBuffer()
    aligner = PendingPairGroundTruthAligner(buffer)
    pair = StereoImagePair(1, 100, 100, 100, 0, "exact", object(), object(), 0)
    assert aligner.add_pair(pair) == []
    assert aligner.clear() == 1
    assert aligner.flush() == []


def test_ground_truth_source_link_is_parameterized_and_launch_uses_target_model() -> None:
    state = PoseVState("vehicle_base")
    payload = {
        "header": {"stamp": {"sec": 1}},
        "pose": [
            {
                "name": "vehicle_base",
                "id": 42,
                "position": {},
                "orientation": {"w": 1.0},
            }
        ],
    }
    assert state.consume(payload).entity_id == 42
    launch = (
        Path(__file__).resolve().parents[1]
        / "stereo_ground_sim"
        / "launch"
        / "stereo_ground_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert '"parent_model_name": LaunchConfiguration("target_model_name")' in launch
    assert '"source_link_name": "base_link"' in launch
