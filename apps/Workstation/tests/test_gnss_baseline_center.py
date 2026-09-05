from __future__ import annotations

from dataclasses import replace
import json
import math

import pytest


def _gps(
    side: str,
    *,
    north_m: float,
    east_m: float,
    down_m: float,
    timestamp_s: float = 100.0,
    fix_type: int = 6,
):
    from Workstation.workstation_core.rtk_geometry import EARTH_RADIUS_M, GpsPositionState

    latitude_origin_deg = 37.0
    longitude_origin_deg = 127.0
    latitude_deg = latitude_origin_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude_deg = longitude_origin_deg + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(latitude_origin_deg)))
    )
    return GpsPositionState(
        camera=side,
        namespace=side,
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=100.0 - down_m,
        fix_type=fix_type,
    )


def test_ned_sentinel_proves_forward_right_down_and_range_independently() -> None:
    from Workstation.workstation_core.coordinate_contract import MEASUREMENT_FRAME
    from Workstation.workstation_core.gnss_baseline_center import (
        GNSS_BASELINE_CENTER_FRD_FRAME,
        gnss_baseline_center_frd_from_ned,
    )

    geometry = gnss_baseline_center_frd_from_ned(
        (0.0, 2.0, 0.0),
        (10.0, 3.0, 4.0),
    )

    assert geometry.forward_m == pytest.approx(10.0)
    assert geometry.right_m == pytest.approx(2.0)
    assert geometry.down_m == pytest.approx(4.0)
    assert geometry.range_m == pytest.approx(math.sqrt(120.0))
    assert geometry.baseline_drone_angle_deg == pytest.approx(math.degrees(math.atan2(10.0, 2.0)))
    assert geometry.baseline_drone_angle_error_deg == pytest.approx(
        abs(90.0 - math.degrees(math.atan2(10.0, 2.0)))
    )
    assert geometry.midpoint_to_drone_ned_m == pytest.approx((10.0, 2.0, 4.0))
    assert geometry.frame == GNSS_BASELINE_CENTER_FRD_FRAME
    assert geometry.frame != MEASUREMENT_FRAME


def test_p1_p2_side_reversal_changes_signed_horizontal_axes() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        gnss_baseline_center_frd_from_ned,
    )

    forward = gnss_baseline_center_frd_from_ned(
        (0.0, 2.0, 0.0),
        (10.0, 3.0, 4.0),
    )
    reversed_sides = gnss_baseline_center_frd_from_ned(
        (0.0, -2.0, 0.0),
        (10.0, 1.0, 4.0),
    )

    assert reversed_sides.forward_m == pytest.approx(-forward.forward_m)
    assert reversed_sides.right_m == pytest.approx(-forward.right_m)
    assert reversed_sides.down_m == pytest.approx(forward.down_m)
    assert reversed_sides.range_m == pytest.approx(forward.range_m)


@pytest.mark.parametrize(
    "baseline",
    ((0.0, 0.0, 0.0), (0.0, 0.0, 3.0)),
)
def test_zero_or_vertical_only_baseline_cannot_define_right_axis(baseline) -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        gnss_baseline_center_frd_from_ned,
    )

    with pytest.raises(ValueError, match="horizontal P1->P2"):
        gnss_baseline_center_frd_from_ned(baseline, (1.0, 2.0, 3.0))


def test_gps_adapter_preserves_current_midpoint_range_contract() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        gnss_baseline_center_frd_from_gps,
    )
    from Workstation.workstation_core.ideal_disparity import target_distance_from_gps

    p1 = _gps("left", north_m=0.0, east_m=0.0, down_m=0.0)
    p2 = _gps("right", north_m=0.0, east_m=2.0, down_m=0.0)
    drone = _gps("drone", north_m=10.0, east_m=3.0, down_m=4.0)
    geometry = gnss_baseline_center_frd_from_gps(p1, p2, drone)

    assert geometry is not None
    assert geometry.forward_m == pytest.approx(10.0, abs=2.0e-5)
    assert geometry.right_m == pytest.approx(2.0, abs=2.0e-5)
    assert geometry.down_m == pytest.approx(4.0, abs=1.0e-9)
    assert geometry.range_m == pytest.approx(10.95445115029011, abs=1.0e-12)
    assert geometry.range_m == pytest.approx(target_distance_from_gps(p1, p2, drone))
    assert geometry.baseline_drone_angle_deg == pytest.approx(math.degrees(math.atan2(10.0, 2.0)), abs=2.0e-5)


def test_gnss_baseline_center_angle_is_90_when_drone_is_on_perpendicular_bisector() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        gnss_baseline_center_frd_from_ned,
    )

    geometry = gnss_baseline_center_frd_from_ned(
        (0.0, 2.0, 0.0),
        (8.0, 1.0, 0.0),
    )

    assert geometry.right_m == pytest.approx(-0.0)
    assert geometry.forward_m == pytest.approx(8.0)
    assert geometry.baseline_drone_angle_deg == pytest.approx(90.0)
    assert geometry.baseline_drone_angle_error_deg == pytest.approx(0.0)


def test_gnss_baseline_center_angle_is_acute_and_ignores_down() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        gnss_baseline_center_frd_from_ned,
    )

    geometry = gnss_baseline_center_frd_from_ned(
        (0.0, 2.0, 0.0),
        (0.0, 3.0, 100.0),
    )

    assert geometry.baseline_drone_angle_deg == pytest.approx(0.0)
    assert geometry.baseline_drone_angle_error_deg == pytest.approx(90.0)


def test_gnss_baseline_center_angle_is_missing_for_zero_horizontal_drone_vector() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        estimate_gnss_baseline_center_frd,
        gnss_baseline_center_frd_from_ned,
    )

    geometry = gnss_baseline_center_frd_from_ned(
        (0.0, 2.0, 0.0),
        (0.0, 1.0, 4.0),
    )

    assert geometry.baseline_drone_angle_deg is None
    assert geometry.baseline_drone_angle_error_deg is None

    estimate = estimate_gnss_baseline_center_frd(
        _gps("left", north_m=0.0, east_m=0.0, down_m=0.0),
        _gps("right", north_m=0.0, east_m=2.0, down_m=0.0),
        _gps("drone", north_m=0.0, east_m=1.0, down_m=-4.0),
        now_s=100.0,
    )
    assert estimate.state == "WAITING"
    assert "horizontal GNSS vector is undefined" in estimate.reason


def test_readiness_uses_two_second_age_and_one_second_spread_boundaries() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        estimate_gnss_baseline_center_frd,
    )

    p1 = _gps("left", north_m=0.0, east_m=0.0, down_m=0.0, timestamp_s=100.0)
    p2 = _gps("right", north_m=0.0, east_m=2.0, down_m=0.0, timestamp_s=100.5)
    drone = _gps("drone", north_m=10.0, east_m=3.0, down_m=4.0, timestamp_s=101.0)
    boundary = estimate_gnss_baseline_center_frd(p1, p2, drone, now_s=102.0)
    assert boundary.state == "CURRENT"
    assert boundary.maximum_age_s == pytest.approx(2.0)
    assert boundary.timestamp_spread_s == pytest.approx(1.0)

    over_spread = estimate_gnss_baseline_center_frd(
        p1,
        p2,
        _gps("drone", north_m=10.0, east_m=3.0, down_m=4.0, timestamp_s=101.001),
        now_s=102.0,
    )
    assert over_spread.state == "STALE"
    assert over_spread.geometry is not None
    assert "spread" in over_spread.reason

    over_age = estimate_gnss_baseline_center_frd(p1, p2, drone, now_s=102.001)
    assert over_age.state == "STALE"
    assert over_age.geometry is not None
    assert "age" in over_age.reason


def test_future_timestamp_is_stale_and_low_fix_is_evidence_not_gate() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        estimate_gnss_baseline_center_frd,
    )

    p1 = _gps(
        "left",
        north_m=0.0,
        east_m=0.0,
        down_m=0.0,
        timestamp_s=100.1,
        fix_type=3,
    )
    p2 = _gps(
        "right",
        north_m=0.0,
        east_m=2.0,
        down_m=0.0,
        timestamp_s=100.0,
        fix_type=5,
    )
    drone = _gps(
        "drone",
        north_m=10.0,
        east_m=3.0,
        down_m=4.0,
        timestamp_s=100.0,
        fix_type=6,
    )
    future = estimate_gnss_baseline_center_frd(p1, p2, drone, now_s=100.0)
    assert future.state == "STALE"
    assert future.geometry is not None
    assert future.p1_fix_label == "3D_FIX"
    assert future.p2_fix_label == "RTK_FLOAT"
    assert future.drone_fix_label == "RTK_FIXED"

    current = estimate_gnss_baseline_center_frd(p1, p2, drone, now_s=100.1)
    assert current.state == "CURRENT"


def test_missing_position_waits_without_fabricating_zero() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        estimate_gnss_baseline_center_frd,
    )

    estimate = estimate_gnss_baseline_center_frd(None, None, None, now_s=1.0)
    mapping = estimate.to_mapping()
    assert estimate.state == "WAITING"
    assert mapping["forward_m"] is None
    assert mapping["right_m"] is None
    assert mapping["down_m"] is None
    assert mapping["range_m"] is None

    partial = estimate_gnss_baseline_center_frd(
        _gps("left", north_m=0.0, east_m=0.0, down_m=0.0, fix_type=5),
        _gps("right", north_m=0.0, east_m=2.0, down_m=0.0, fix_type=6),
        None,
        now_s=1.0,
    ).to_mapping()
    assert partial["fix_labels"] == {
        "p1": "RTK_FLOAT",
        "p2": "RTK_FIXED",
        "drone": "UNKNOWN",
    }

    malformed = estimate_gnss_baseline_center_frd(  # type: ignore[arg-type]
        {"latitude_deg": 37.0},
        _gps("right", north_m=0.0, east_m=2.0, down_m=0.0),
        _gps("drone", north_m=1.0, east_m=3.0, down_m=0.0),
        now_s=1.0,
    )
    assert malformed.state == "WAITING"
    assert "invalid P1" in malformed.reason


def test_record_builder_is_self_contained_json_safe_and_below_four_kib() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        build_gnss_baseline_center_record,
        estimate_gnss_baseline_center_frd,
    )

    p1 = _gps("left", north_m=0.0, east_m=0.0, down_m=0.0, timestamp_s=100.0)
    p2 = _gps("right", north_m=0.0, east_m=2.0, down_m=0.0, timestamp_s=100.5)
    drone = _gps("drone", north_m=10.0, east_m=3.0, down_m=4.0, timestamp_s=101.0)
    estimate = estimate_gnss_baseline_center_frd(p1, p2, drone, now_s=102.0)
    row = build_gnss_baseline_center_record(
        estimate,
        p1=p1,
        p2=p2,
        drone=drone,
        p1_selection="GPS1",
        p2_selection="GPS2",
        sequence=7,
    )
    encoded = json.dumps(row, allow_nan=False, sort_keys=True).encode("utf-8")

    assert len(encoded) <= 4096
    assert row["sequence"] == 7
    assert row["geometry_state"] == "CURRENT"
    assert row["coordinate_frame"] == "gnss_baseline_center_local_level_frd"
    assert row["gnss_baseline_center_forward_m"] == pytest.approx(10.0, abs=2.0e-5)
    assert row["gnss_baseline_center_right_m"] == pytest.approx(2.0, abs=2.0e-5)
    assert row["gnss_baseline_center_down_m"] == pytest.approx(4.0)
    assert "baseline_drone_angle_deg" not in row
    assert row["three_source_timestamp_spread_ms"] == pytest.approx(1000.0)
    assert row["p1_source_selection"] == "GPS1"
    assert row["p2_source_selection"] == "GPS2"
    assert row["drone_source_selection"] == "GPS1_PRIMARY"
    assert row["p1_app_receive_monotonic_ns"] == 100_000_000_000
    assert row["p2_app_receive_monotonic_ns"] == 100_500_000_000
    assert row["drone_app_receive_monotonic_ns"] == 101_000_000_000


def test_waiting_record_keeps_partial_sources_and_null_geometry() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        build_gnss_baseline_center_record,
        estimate_gnss_baseline_center_frd,
    )

    drone = _gps("drone", north_m=1.0, east_m=2.0, down_m=3.0, timestamp_s=10.0)
    estimate = estimate_gnss_baseline_center_frd(None, None, drone, now_s=10.1)
    row = build_gnss_baseline_center_record(
        estimate,
        p1=None,
        p2=None,
        drone=drone,
        p1_selection="GPS1",
        p2_selection="GPS1",
        sequence=0,
    )

    assert row["geometry_state"] == "WAITING"
    assert row["gnss_baseline_center_forward_m"] is None
    assert row["gnss_baseline_center_range_m"] is None
    assert row["p1_position_available"] is False
    assert row["p2_position_available"] is False
    assert row["drone_position_available"] is True


def test_nonfinite_waiting_source_remains_strict_json_recordable() -> None:
    from Workstation.workstation_core.gnss_baseline_center import (
        build_gnss_baseline_center_record,
        estimate_gnss_baseline_center_frd,
    )

    p1 = replace(
        _gps("left", north_m=0.0, east_m=0.0, down_m=0.0),
        latitude_deg=float("nan"),
        eph_m=float("inf"),
    )
    p2 = _gps("right", north_m=0.0, east_m=2.0, down_m=0.0)
    drone = _gps("drone", north_m=10.0, east_m=3.0, down_m=4.0)
    estimate = estimate_gnss_baseline_center_frd(p1, p2, drone, now_s=100.1)

    row = build_gnss_baseline_center_record(
        estimate,
        p1=p1,
        p2=p2,
        drone=drone,
        p1_selection="GPS1",
        p2_selection="GPS1",
        sequence=0,
    )

    assert estimate.state == "WAITING"
    assert row["p1_position_available"] is True
    assert row["p1_latitude_deg"] is None
    assert row["p1_eph_m"] is None
    assert row["gnss_baseline_center_range_m"] is None
    json.dumps(row, allow_nan=False)
