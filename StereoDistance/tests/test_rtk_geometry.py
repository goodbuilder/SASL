from types import SimpleNamespace

from wiris_stereo_distance.rtk_geometry import (
    GPS_SOURCE_GLOBAL_POSITION,
    GpsPositionState,
    estimate_rtk_baseline,
    global_position_topic,
    gps_state_from_message,
    gps_topic,
    ned_delta_m,
)


def test_gps_topic_maps_namespace() -> None:
    assert gps_topic("/uav_0/") == "/uav_0/fmu/out/vehicle_gps_position"
    assert gps_topic("") == "/fmu/out/vehicle_gps_position"
    assert global_position_topic("/uav_0/") == "/uav_0/fmu/out/vehicle_global_position"


def test_gps_state_from_px4_scaled_message() -> None:
    message = SimpleNamespace(
        timestamp=1_234_000,
        lat=37_1234567,
        lon=127_7654321,
        alt=123_450,
        fix_type=6,
        satellites_used=22,
        eph=0.03,
        epv=0.04,
        heading=1.57079632679,
    )

    state = gps_state_from_message(camera="left", namespace="uav_0", message=message, timestamp_s=10.0)

    assert state.latitude_deg == 37.1234567
    assert state.longitude_deg == 127.7654321
    assert state.altitude_m == 123.45
    assert state.rtk_label == "RTK_FIXED"
    assert round(state.heading_deg or 0, 1) == 90.0
    assert state.px4_timestamp_s == 1.234


def test_global_position_state_preserves_source_and_sample_timestamp() -> None:
    message = SimpleNamespace(
        timestamp=2_000_000,
        timestamp_sample=1_900_000,
        lat=37.0,
        lon=127.0,
        alt=10.5,
        lat_lon_valid=True,
        alt_valid=True,
        eph=0.4,
        epv=0.7,
    )

    state = gps_state_from_message(
        camera="left",
        namespace="uav_0",
        message=message,
        timestamp_s=20.0,
        source=GPS_SOURCE_GLOBAL_POSITION,
        topic=global_position_topic("uav_0"),
    )

    assert state.source == GPS_SOURCE_GLOBAL_POSITION
    assert state.topic.endswith("vehicle_global_position")
    assert state.px4_timestamp_s == 2.0
    assert state.px4_sample_timestamp_s == 1.9
    assert state.has_position


def test_ned_delta_and_baseline_between_two_fixed_rtk_positions() -> None:
    left = GpsPositionState(
        camera="left",
        namespace="uav_0",
        timestamp_s=100.0,
        topic="/uav_0/fmu/out/vehicle_gps_position",
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=10.0,
        fix_type=6,
        eph_m=0.02,
    )
    right = GpsPositionState(
        camera="right",
        namespace="uav_1",
        timestamp_s=100.1,
        topic="/uav_1/fmu/out/vehicle_gps_position",
        latitude_deg=37.0,
        longitude_deg=127.000001,
        altitude_m=10.0,
        fix_type=6,
        eph_m=0.02,
    )

    delta = ned_delta_m(left, right)
    baseline = estimate_rtk_baseline(left, right, now_s=100.2, require_fixed=True)

    assert delta is not None
    assert round(delta[1], 3) == 0.089
    assert baseline is not None
    assert round(baseline.baseline_m, 3) == 0.089
    assert baseline.fix_quality == "GOOD"
    assert baseline.warnings == ()


def test_rtk_baseline_blocks_when_fixed_required_but_float() -> None:
    left = GpsPositionState("left", "uav_0", 1.0, "topic", 37.0, 127.0, 10.0, fix_type=5)
    right = GpsPositionState("right", "uav_1", 1.0, "topic", 37.0, 127.000001, 10.0, fix_type=6)

    baseline = estimate_rtk_baseline(left, right, now_s=1.1, require_fixed=True)

    assert baseline is not None
    assert baseline.fix_quality == "BAD"
    assert "left RTK_FLOAT" in baseline.warnings
