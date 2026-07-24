from dataclasses import dataclass

from wiris_stereo_distance.sync import (
    TimedSampleBuffer,
    classify_attitude_timing,
    classify_sync_quality,
    match_nearest_pair,
)


@dataclass(frozen=True)
class Sample:
    timestamp_s: float
    name: str


def test_match_nearest_pair_prefers_tightest_delta_within_recent_candidates() -> None:
    left = [Sample(10.00, "old-left"), Sample(20.00, "new-left")]
    right = [Sample(10.00, "old-right"), Sample(20.04, "new-right")]

    pair = match_nearest_pair(left, right, max_delta_s=0.05)

    assert pair is not None
    assert pair.left.name == "new-left"
    assert pair.right.name == "new-right"
    assert round(pair.delta_s, 2) == 0.04


def test_match_nearest_pair_uses_tightest_delta_inside_recent_window() -> None:
    left = [Sample(20.00, "new-left-loose"), Sample(20.03, "new-left-tight")]
    right = [Sample(20.04, "new-right"), Sample(20.031, "new-right-tight")]

    pair = match_nearest_pair(left, right, max_delta_s=0.05)

    assert pair is not None
    assert pair.left.name == "new-left-tight"
    assert pair.right.name == "new-right-tight"
    assert round(pair.delta_s, 3) == 0.001


def test_match_nearest_pair_rejects_pairs_outside_gate() -> None:
    pair = match_nearest_pair([Sample(1.0, "left")], [Sample(1.2, "right")], max_delta_s=0.05)

    assert pair is None


def test_timed_sample_buffer_nearest_reports_age_and_applies_max_age() -> None:
    buffer = TimedSampleBuffer[Sample](maxlen=3)
    buffer.add(Sample(1.0, "a"))
    buffer.add(Sample(2.0, "b"))
    buffer.add(Sample(3.0, "c"))

    nearest = buffer.nearest(2.2)

    assert nearest is not None
    assert nearest.sample.name == "b"
    assert round(nearest.age_s, 1) == 0.2
    assert buffer.nearest(2.2, max_age_s=0.1) is None


def test_sync_quality_marks_missing_attitude_as_degraded_without_attitude_gate() -> None:
    quality, warnings = classify_sync_quality(
        camera_delta_s=0.02,
        max_camera_delta_s=0.05,
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        max_attitude_age_s=0.1,
        attitude_pair_delta_s=None,
        max_attitude_pair_delta_s=0.05,
        require_attitude=False,
    )

    assert quality == "DEGRADED"
    assert "attitude waiting" in warnings


def test_sync_quality_blocks_when_attitude_gate_requires_missing_attitude() -> None:
    quality, warnings = classify_sync_quality(
        camera_delta_s=0.02,
        max_camera_delta_s=0.05,
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        max_attitude_age_s=0.1,
        attitude_pair_delta_s=None,
        max_attitude_pair_delta_s=0.05,
        require_attitude=True,
    )

    assert quality == "BAD"
    assert "attitude waiting" in warnings


def test_attitude_timing_status_uses_nonblocking_display_thresholds() -> None:
    assert classify_attitude_timing(
        uses_pixhawk_attitude=True,
        left_age_s=2.0,
        right_age_s=1.0,
        pair_delta_s=1.0,
    ) == "FRESH"
    assert classify_attitude_timing(
        uses_pixhawk_attitude=True,
        left_age_s=2.001,
        right_age_s=1.0,
        pair_delta_s=0.0,
    ) == "STALE"
    assert classify_attitude_timing(
        uses_pixhawk_attitude=True,
        left_age_s=1.0,
        right_age_s=1.0,
        pair_delta_s=1.001,
    ) == "STALE"
    assert classify_attitude_timing(
        uses_pixhawk_attitude=True,
        left_age_s=10.001,
        right_age_s=1.0,
        pair_delta_s=0.0,
    ) == "VERY_STALE"
    assert classify_attitude_timing(
        uses_pixhawk_attitude=True,
        left_age_s=1.0,
        right_age_s=1.0,
        pair_delta_s=5.001,
    ) == "VERY_STALE"


def test_attitude_timing_status_marks_unused_and_missing_inputs() -> None:
    assert classify_attitude_timing(
        uses_pixhawk_attitude=False,
        left_age_s=None,
        right_age_s=None,
        pair_delta_s=None,
    ) == "NOT_USED"
    assert classify_attitude_timing(
        uses_pixhawk_attitude=True,
        left_age_s=None,
        right_age_s=0.1,
        pair_delta_s=0.0,
    ) == "WAITING"
