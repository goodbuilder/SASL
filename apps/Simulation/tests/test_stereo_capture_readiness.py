"""Readiness uses unique source time, not callback counts or stale latest values."""
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1.capture_readiness import CaptureReadiness


def feed(gate, stamps, host_offset=10_000_000_000):
    for t in stamps:
        gate.clock(t, host_offset+t)
        gate.ground_truth(t, "gazebo_world_enu", host_offset+t+1)
    return gate.report(host_offset+t+2)


def test_slow_warmup_then_stable_source_window():
    gate = CaptureReadiness(50, .04)
    assert feed(gate, range(0, 2_000_000_001, 40_000_000))["status"] == "WAITING"
    assert feed(gate, range(2_020_000_000, 4_020_000_001, 20_000_000))["status"] == "READY"


def test_duplicates_cannot_create_rate_or_duration():
    gate = CaptureReadiness(50, .04)
    gate.clock(0, 0)
    for i in range(200):
        gate.ground_truth(0, "gazebo_world_enu", i+1)
    report = gate.report(201)
    assert report["status"] == "WAITING"
    assert report["unique_gt_samples"] == 1
    assert report["duplicate_gt_count"] == 199


def test_stale_clock_or_stale_source_never_ready():
    gate = CaptureReadiness(50, .04)
    assert feed(gate, range(0, 2_000_000_001, 20_000_000))["status"] == "READY"
    assert gate.report(13_000_000_000)["status"] == "WAITING"
    gate.clock(3_000_000_000, 13_000_000_001)
    gate.ground_truth(2_000_000_000, "gazebo_world_enu", 13_000_000_002)
    assert gate.report(13_000_000_003)["unique_gt_samples"] == 0


def test_clock_epoch_reset_requires_new_stable_window():
    gate = CaptureReadiness(50, .04)
    assert feed(gate, range(0, 2_000_000_001, 20_000_000))["status"] == "READY"
    gate.clock(0, 13_000_000_000)
    assert gate.report(13_000_000_001)["status"] == "WAITING"
    assert gate.epoch == 1
    assert feed(gate, range(0, 2_020_000_001, 20_000_000), 14_000_000_000)["status"] == "READY"


def test_source_outage_restarts_stable_window():
    gate = CaptureReadiness(50, .04)
    feed(gate, range(0, 2_000_000_001, 20_000_000))
    assert feed(gate, [2_100_000_000])["status"] == "WAITING"
    assert feed(gate, range(2_120_000_000, 4_100_000_001, 20_000_000))["status"] == "READY"


def test_wrong_frame_and_backwards_host_are_rejected():
    gate = CaptureReadiness(50, .04)
    gate.clock(0, 10)
    with pytest.raises(ValueError, match="world_enu"):
        gate.ground_truth(0, "ned", 11)
    gate = CaptureReadiness(50, .04)
    gate.clock(0, 10)
    with pytest.raises(ValueError, match="monotonic"):
        gate.clock(1, 9)
