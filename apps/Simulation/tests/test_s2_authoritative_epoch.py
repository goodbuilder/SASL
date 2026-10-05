from __future__ import annotations

from pathlib import Path

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import (
    GroundTruthPositionBuffer,
    SimulationEpochTracker,
    StereoImagePairer,
)


Q = (0.0, 0.0, 0.0, 1.0)


def test_single_clock_reset_owns_image_and_gt_epoch() -> None:
    clock = SimulationEpochTracker()
    pairer = StereoImagePairer()
    gt = GroundTruthPositionBuffer()
    assert not clock.observe_clock(100).reset
    pairer.add("left", 100, "old-left", epoch=0)
    assert clock.observe_clock(100).epoch == 0
    event = clock.observe_clock(1)
    assert event.reset and event.epoch == 1 and clock.reset_count == 1
    pairer.reset_for_epoch(event.epoch)
    gt.reset_for_epoch(event.epoch)
    assert not clock.accepts_stamp(100)
    pairer.add("left", 1, "new-left", epoch=1)
    pairs = pairer.add("right", 1, "new-right", epoch=1)
    gt.add(1, (1.0, 2.0, 3.0), Q, epoch=1)
    assert len(pairs) == 1
    assert pairs[0].epoch == gt.epoch == clock.epoch == 1
    assert pairs[0].right_message == "new-right"
    assert gt.align(1, 1).gt_alignment_mode == "exact"
    assert pairer.image_reset_count == 1


def test_opposite_callback_order_does_not_create_second_epoch() -> None:
    clock = SimulationEpochTracker()
    pairer = StereoImagePairer()
    clock.observe_clock(100)
    pairer.add("right", 100, "old-right", epoch=0)
    event = clock.observe_clock(1)
    pairer.reset_for_epoch(event.epoch)
    pairer.add("right", 1, "new-right", epoch=1)
    assert not clock.accepts_stamp(100)
    pairs = pairer.add("left", 1, "new-left", epoch=1)
    assert len(pairs) == 1
    assert pairs[0].left_message == "new-left"
    assert clock.reset_count == pairer.image_reset_count == 1


def test_launch_bridges_world_scoped_clock_to_ros_clock() -> None:
    launch = (
        Path(__file__).resolve().parents[1]
        / "stereo_ground_sim"
        / "launch"
        / "stereo_ground_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert '"/world/stereo_ground_drone/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"' in launch
    assert '"/world/stereo_ground_drone/clock:=/clock"' in launch
