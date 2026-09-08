from __future__ import annotations

import math
from dataclasses import replace
from types import SimpleNamespace

import pytest

from Workstation.evidence import EvidenceAdapter
from Workstation.workstation_core.pixhawk_attitude import CameraExtrinsicState
from Workstation.workstation_core.rtk_geometry import EARTH_RADIUS_M, GpsPositionState


def _gps(
    side: str,
    *,
    north_m: float,
    east_m: float,
    down_m: float,
    timestamp_s: float = 99.9,
) -> GpsPositionState:
    reference_latitude_deg = 37.0
    return GpsPositionState(
        camera=side,
        namespace=side,
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=reference_latitude_deg + math.degrees(north_m / EARTH_RADIUS_M),
        longitude_deg=127.0
        + math.degrees(
            east_m
            / (EARTH_RADIUS_M * math.cos(math.radians(reference_latitude_deg)))
        ),
        altitude_m=100.0 - down_m,
        fix_type=6,
        satellites_used=20,
    )


def _attitude(side: str, *, timestamp_s: float = 99.9) -> CameraExtrinsicState:
    return CameraExtrinsicState(
        camera=side,
        namespace=side,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=timestamp_s,
        topic="mavlink/ATTITUDE",
        q=(1.0, 0.0, 0.0, 0.0),
    )


def _panel(*, include_p2_attitude: bool = True) -> SimpleNamespace:
    p1 = _gps("left", north_m=0.0, east_m=0.0, down_m=0.0)
    p2 = _gps("right", north_m=10.0, east_m=0.0, down_m=2.0)
    drone = _gps("drone", north_m=0.0, east_m=10.0, down_m=-3.0)
    selected = {"left": p1, "right": p2, "drone": drone}
    attitudes = {
        "left": _attitude("left"),
    }
    if include_p2_attitude:
        attitudes["right"] = _attitude("right")
    return SimpleNamespace(
        latest_gps={"left": p1, "right": p2, "drone": drone},
        latest_extrinsics=attitudes,
        _selected_rover_gps_state=lambda side: selected.get(side),
        _pixhawk_gnss_source=lambda _side: "GPS1",
    )


def test_evidence_projects_three_vectors_and_both_horizontal_angles() -> None:
    snapshot = EvidenceAdapter(_panel()).drone_pixhawk_geometry(now_s=100.0)

    assert snapshot["state"] == "CURRENT"
    vectors = {row["key"]: row for row in snapshot["vectors"]}
    assert vectors["baseline_center"]["frame"] == "gnss_baseline_center_local_level_frd"
    assert vectors["baseline_center"]["forward_m"] == pytest.approx(-10.0)
    assert vectors["baseline_center"]["right_m"] == pytest.approx(-5.0)
    assert vectors["baseline_center"]["down_m"] == pytest.approx(-4.0)
    assert vectors["baseline_center"]["range_m"] == pytest.approx(math.sqrt(141.0), abs=1e-5)
    assert vectors["p1"]["frame"] == "px4_frd (P1 body)"
    assert vectors["p1"]["forward_m"] == pytest.approx(0.0, abs=1e-5)
    assert vectors["p1"]["right_m"] == pytest.approx(10.0, abs=1e-5)
    assert vectors["p1"]["down_m"] == pytest.approx(-3.0, abs=1e-5)
    assert vectors["p2"]["frame"] == "px4_frd (P2 body)"
    assert vectors["p2"]["forward_m"] == pytest.approx(-10.0, abs=1e-5)
    assert vectors["p2"]["right_m"] == pytest.approx(10.0, abs=1e-5)
    assert vectors["p2"]["down_m"] == pytest.approx(-5.0, abs=1e-5)

    angles = {row["key"]: row for row in snapshot["angles"]}
    assert angles["dr_p1_p2"]["label"] == "∠Dr-P1-P2"
    assert angles["dr_p1_p2"]["angle_deg"] == pytest.approx(90.0, abs=1e-5)
    assert angles["dr_p2_p1"]["label"] == "∠Dr-P2-P1"
    assert angles["dr_p2_p1"]["angle_deg"] == pytest.approx(45.0, abs=1e-3)
    assert "North/East only" in snapshot["angle_contract"]


def test_evidence_keeps_midpoint_and_p1_when_p2_attitude_is_missing() -> None:
    snapshot = EvidenceAdapter(_panel(include_p2_attitude=False)).drone_pixhawk_geometry(
        now_s=100.0
    )

    assert snapshot["state"] == "PARTIAL"
    vectors = {row["key"]: row for row in snapshot["vectors"]}
    assert vectors["baseline_center"]["state"] == "CURRENT"
    assert vectors["p1"]["state"] == "CURRENT"
    assert vectors["p2"]["state"] == "WAITING"
    assert vectors["p2"]["forward_m"] is None
    assert vectors["p1"]["forward_m"] is not None


def test_evidence_marks_all_geometry_stale_without_discarding_finite_values() -> None:
    panel = _panel()
    panel.latest_gps = {
        side: replace(state, timestamp_s=97.0)
        for side, state in panel.latest_gps.items()
    }
    panel._selected_rover_gps_state = lambda side: panel.latest_gps.get(side)
    panel.latest_extrinsics["left"] = _attitude("left", timestamp_s=97.0)
    panel.latest_extrinsics["right"] = _attitude("right", timestamp_s=97.0)

    snapshot = EvidenceAdapter(panel).drone_pixhawk_geometry(now_s=100.0)

    assert snapshot["state"] == "STALE"
    assert all(row["state"] == "STALE" for row in snapshot["vectors"])
    assert all(row["state"] == "STALE" for row in snapshot["angles"])
    assert all(row["forward_m"] is not None for row in snapshot["vectors"])
    assert snapshot["angles"][0]["angle_deg"] == pytest.approx(90.0, abs=1e-5)


def test_frozen_drone_gnss_marks_geometry_and_receiver_row() -> None:
    """A drone whose GNSS repeats one position must not present as CURRENT.

    Field log 2026-09-07: 165 consecutive identical drone fixes with UTC
    advancing; age kept resetting so the triangle looked live. The operator
    must see this in the table, not only in the log.
    """

    panel = _panel()
    panel._gnss_position_frozen = {"drone": {"packets": 165, "utc_frozen": False, "since_s": 50.0}}
    evidence = EvidenceAdapter(panel)

    snapshot = evidence.drone_pixhawk_geometry(now_s=100.0)
    assert snapshot["state"] == "FROZEN"
    assert snapshot["drone_position_frozen"] is True
    assert snapshot["reason"].startswith("Drone GNSS 위치 고정 ×165 (수신기 위치 출력 정지)")
    # The numbers are still shown; only the state says they are untrustworthy.
    angles = {row["key"]: row for row in snapshot["angles"]}
    assert angles["dr_p1_p2"]["angle_deg"] == pytest.approx(90.0, abs=1e-5)

    row = evidence.gps("drone", now_s=100.0)
    assert row["position_frozen"] is True
    assert row["position_frozen_packets"] == 165
    assert "receiver position output frozen" in row["position_frozen_cause"]

    panel._gnss_position_frozen = {"drone": {"packets": 12, "utc_frozen": True}}
    snapshot = evidence.drone_pixhawk_geometry(now_s=100.0)
    assert "autopilot이 같은 fix 재전송" in snapshot["reason"]

    panel._gnss_position_frozen = {}
    assert evidence.drone_pixhawk_geometry(now_s=100.0)["state"] == "CURRENT"
    assert evidence.gps("drone", now_s=100.0)["position_frozen"] is False
