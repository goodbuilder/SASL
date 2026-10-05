"""Signed horizontal P1->P2 baseline/yaw presentation contract."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from Workstation.evidence import EvidenceAdapter
from Workstation.workstation_core.pixhawk_attitude import CameraExtrinsicState
from Workstation.workstation_core.physical_alignment import (
    calculate_baseline_yaw_alignment,
)
from Workstation.workstation_core.rtk_geometry import GpsPositionState, RtkBaselineEstimate


@pytest.mark.parametrize(
    ("baseline_ned_m", "expected_azimuth_deg"),
    (
        ((1.0, 0.0, 0.0), 0.0),
        ((0.0, 1.0, 0.0), 90.0),
        ((-1.0, 0.0, 0.0), 180.0),
        ((0.0, -1.0, 0.0), 270.0),
    ),
)
def test_baseline_yaw_alignment_uses_clockwise_ned_compass_bearing(
    baseline_ned_m: tuple[float, float, float],
    expected_azimuth_deg: float,
) -> None:
    result = calculate_baseline_yaw_alignment(baseline_ned_m)

    assert result is not None
    assert result.baseline_azimuth_deg == pytest.approx(expected_azimuth_deg)


def test_baseline_p1_and_baseline_p2_degrees_are_independent_signed_values() -> None:
    result = calculate_baseline_yaw_alignment(
        (0.0, 3.0, 9.0),
        p1_yaw_deg=350.0,
        p2_yaw_deg=100.0,
    )

    assert result is not None
    assert result.baseline_azimuth_deg == pytest.approx(90.0)
    assert result.p1_yaw_deg == pytest.approx(350.0)
    assert result.p2_yaw_deg == pytest.approx(100.0)
    assert result.baseline_p1_deg == pytest.approx(100.0)
    assert result.baseline_p2_deg == pytest.approx(-10.0)
    assert result.p2_minus_p1_deg == pytest.approx(110.0)


def test_baseline_yaw_alignment_wraps_compass_and_signed_boundaries() -> None:
    north = calculate_baseline_yaw_alignment(
        (1.0, 0.0, -50.0),
        p1_yaw_deg=359.0,
        p2_yaw_deg=180.0,
    )

    assert north is not None
    assert north.baseline_p1_deg == pytest.approx(1.0)
    assert north.baseline_p2_deg == pytest.approx(-180.0)
    assert north.p2_minus_p1_deg == pytest.approx(-179.0)


def test_baseline_yaw_alignment_keeps_one_available_pixhawk() -> None:
    result = calculate_baseline_yaw_alignment(
        (1.0, 1.0, 2.0),
        p1_yaw_deg=10.0,
    )

    assert result is not None
    assert result.baseline_p1_deg == pytest.approx(35.0)
    assert result.p2_yaw_deg is None
    assert result.baseline_p2_deg is None
    assert result.p2_minus_p1_deg is None


def test_baseline_yaw_alignment_has_no_direction_for_vertical_or_zero_baseline() -> None:
    assert calculate_baseline_yaw_alignment((0.0, 0.0, 5.0), p1_yaw_deg=20.0) is None
    assert calculate_baseline_yaw_alignment((0.0, 0.0, 0.0), p2_yaw_deg=20.0) is None


@pytest.mark.parametrize(
    ("baseline_ned_m", "p1_yaw_deg", "p2_yaw_deg"),
    (
        ((math.nan, 1.0, 0.0), 0.0, 0.0),
        ((1.0, 0.0, 0.0), math.inf, 0.0),
        ((1.0, 0.0, 0.0), 0.0, math.nan),
    ),
)
def test_baseline_yaw_alignment_rejects_nonfinite_inputs(
    baseline_ned_m: tuple[float, float, float],
    p1_yaw_deg: float,
    p2_yaw_deg: float,
) -> None:
    with pytest.raises(ValueError):
        calculate_baseline_yaw_alignment(
            baseline_ned_m,
            p1_yaw_deg=p1_yaw_deg,
            p2_yaw_deg=p2_yaw_deg,
        )


def _attitude(side: str, *, yaw_deg: float, timestamp_s: float) -> CameraExtrinsicState:
    half_yaw = math.radians(yaw_deg) / 2.0
    return CameraExtrinsicState(
        camera=side,
        namespace=side,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=yaw_deg,
        timestamp_s=timestamp_s,
        topic="mavlink/ATTITUDE",
        q=(math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)),
    )


def _gps(side: str, *, timestamp_s: float, longitude_deg: float) -> GpsPositionState:
    return GpsPositionState(
        camera=side,
        namespace=side,
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=37.0,
        longitude_deg=longitude_deg,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
    )


def _evidence_panel(*, now_s: float = 100.0) -> SimpleNamespace:
    left_gps = _gps("left", timestamp_s=now_s - 0.2, longitude_deg=127.0)
    right_gps = _gps("right", timestamp_s=now_s - 0.1, longitude_deg=127.00001)
    return SimpleNamespace(
        latest_rtk_baseline=RtkBaselineEstimate(
            baseline_m=1.0,
            north_m=0.0,
            east_m=1.0,
            down_m=0.0,
            left_age_s=0.2,
            right_age_s=0.1,
            fix_quality="GOOD",
            gps_pair_delta_s=0.1,
        ),
        latest_p1_body_frd_baseline=None,
        latest_baseline_quality=None,
        latest_gps={"left": left_gps, "right": right_gps},
        latest_extrinsics={
            "left": _attitude("left", yaw_deg=0.0, timestamp_s=now_s - 0.05),
            "right": _attitude("right", yaw_deg=100.0, timestamp_s=now_s - 0.1),
        },
    )


def test_baseline_yaw_evidence_projects_separate_current_p1_and_p2_values() -> None:
    snapshot = EvidenceAdapter(_evidence_panel()).baseline_yaw_alignment(now_s=100.0)

    assert snapshot["state"] == "CURRENT"
    assert snapshot["baseline_state"] == "CURRENT"
    assert snapshot["p1_state"] == "CURRENT"
    assert snapshot["p2_state"] == "CURRENT"
    assert snapshot["baseline_azimuth_deg"] == pytest.approx(90.0)
    assert snapshot["baseline_p1_deg"] == pytest.approx(90.0)
    assert snapshot["baseline_p2_deg"] == pytest.approx(-10.0)
    assert snapshot["p2_minus_p1_deg"] == pytest.approx(100.0)
    assert snapshot["gps_pair_delta_ms"] == pytest.approx(100.0)
    assert snapshot["p1_fix_type"] == 6
    assert snapshot["p2_fix_type"] == 6


def test_baseline_yaw_evidence_keeps_current_p1_when_p2_attitude_is_missing() -> None:
    panel = _evidence_panel()
    panel.latest_extrinsics.pop("right")

    snapshot = EvidenceAdapter(panel).baseline_yaw_alignment(now_s=100.0)

    assert snapshot["state"] == "PARTIAL"
    assert snapshot["p1_state"] == "CURRENT"
    assert snapshot["p2_state"] == "WAITING"
    assert snapshot["baseline_p1_deg"] == pytest.approx(90.0)
    assert snapshot["baseline_p2_deg"] is None
    assert "P2 ATTITUDE waiting" in snapshot["reason"]


def test_baseline_yaw_evidence_retains_but_marks_one_stale_attitude() -> None:
    panel = _evidence_panel()
    panel.latest_extrinsics["right"] = _attitude(
        "right",
        yaw_deg=100.0,
        timestamp_s=97.0,
    )

    snapshot = EvidenceAdapter(panel).baseline_yaw_alignment(now_s=100.0)

    assert snapshot["state"] == "PARTIAL"
    assert snapshot["p1_state"] == "CURRENT"
    assert snapshot["p2_state"] == "STALE"
    assert snapshot["baseline_p2_deg"] == pytest.approx(-10.0)
    assert snapshot["p2_attitude_age_s"] == pytest.approx(3.0)


def test_baseline_yaw_evidence_marks_selected_gnss_baseline_stale() -> None:
    panel = _evidence_panel()
    panel.latest_gps["left"] = _gps("left", timestamp_s=98.0, longitude_deg=127.0)
    panel.latest_gps["right"] = _gps("right", timestamp_s=98.1, longitude_deg=127.00001)

    snapshot = EvidenceAdapter(panel).baseline_yaw_alignment(now_s=100.0)

    assert snapshot["state"] == "STALE"
    assert snapshot["baseline_state"] == "STALE"
    assert snapshot["baseline_p1_deg"] == pytest.approx(90.0)
    assert "GNSS baseline is stale" in snapshot["reason"]


def test_baseline_yaw_evidence_uses_explicit_gps2_selection() -> None:
    panel = _evidence_panel()
    panel.latest_gps["left_GPS2"] = panel.latest_gps["left"]
    panel.latest_gps["right_GPS2"] = panel.latest_gps["right"]
    panel._pixhawk_gnss_source = lambda _side: "GPS2"

    snapshot = EvidenceAdapter(panel).baseline_yaw_alignment(now_s=100.0)

    assert snapshot["state"] == "CURRENT"
    assert snapshot["p1_gps_source"] == "GPS2"
    assert snapshot["p2_gps_source"] == "GPS2"


def test_baseline_yaw_evidence_waits_when_horizontal_direction_is_undefined() -> None:
    panel = _evidence_panel()
    panel.latest_rtk_baseline = RtkBaselineEstimate(
        baseline_m=1.0,
        north_m=0.0,
        east_m=0.0,
        down_m=1.0,
        left_age_s=0.2,
        right_age_s=0.1,
        fix_quality="GOOD",
    )

    snapshot = EvidenceAdapter(panel).baseline_yaw_alignment(now_s=100.0)

    assert snapshot["state"] == "WAITING"
    assert snapshot["baseline_azimuth_deg"] is None
    assert snapshot["baseline_p1_deg"] is None
    assert snapshot["p2_minus_p1_deg"] == pytest.approx(100.0)
    assert "horizontal baseline direction is undefined" in snapshot["reason"]
