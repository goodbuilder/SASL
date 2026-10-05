from __future__ import annotations

import pytest


def test_alignment_profile_manager_payload_contains_required_contracts() -> None:
    from wiris_stereo_distance.alignment_managers import (
        AlignmentProfileManager,
        CoordinateSystemManager,
        GNSSMonitor,
        BaselineManager,
    )
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    gnss = GNSSMonitor()
    gnss.update_position("left", GpsPositionState("left", "/uav_0", 1.0, "gps", 37.0, 127.0, 10.0, fix_type=6))
    gnss.update_position("right", GpsPositionState("right", "/uav_1", 1.01, "gps", 37.0, 127.000003, 10.0, fix_type=6))
    baseline = BaselineManager(gnss).current()
    assert baseline is not None

    coordinates = CoordinateSystemManager()
    profile = AlignmentProfileManager()
    payload = profile.build_payload(
        baseline=baseline[0],
        baseline_quality=None,
        left_attitude=None,
        right_attitude=None,
        coordinate_settings=coordinates.settings,
        gps_mapping={"left_camera": "GPS1", "right_camera": "GPS1"},
        magnetometer_mapping={"left_can1": "CAN1", "right_can1": "CAN1"},
    )
    loaded = profile.load(profile.dump(payload))

    assert loaded["alignment_profile"]["baseline"]["frame"] == "ENU"
    assert loaded["alignment_profile"]["coordinate_system"]["camera_frame"] == "RDF"
    assert loaded["alignment_profile"]["gps_source_mapping"]["left_camera"] == "GPS1"


def test_alignment_validator_reports_fail_for_large_profile_delta() -> None:
    from wiris_stereo_distance.alignment_managers import AlignmentValidator
    from wiris_stereo_distance.baseline_alignment import BaselineVector

    current = BaselineVector(
        east_m=0.20,
        north_m=0.0,
        up_m=0.0,
        length_m=0.20,
        azimuth_deg=90.0,
        elevation_deg=0.0,
        altitude_delta_m=0.0,
        timestamp_delta_ms=10.0,
        left_timestamp_s=1.0,
        right_timestamp_s=1.01,
    )

    result = AlignmentValidator().validate(
        stored_baseline_enu_m=(0.0, 0.0, 0.0),
        current_baseline=current,
        stored_rpy_deg=(0.0, 0.0, 0.0),
        current_rpy_delta_deg=(0.0, 0.0, 0.0),
        distance_m=1852.0,
    )

    assert result.baseline_vector_difference_m == 0.20
    assert result.status == "FAIL"


def test_magnetometer_difference_normalizes_milligauss_but_preserves_display_unit() -> None:
    from wiris_stereo_distance.alignment_managers import MagnetometerMonitor
    from wiris_stereo_distance.pixhawk_attitude import MagSensorState

    monitor = MagnetometerMonitor()
    for slot, magnitude, topic in (
        ("CAN1", 100.0, "mavlink/SCALED_IMU"),
        ("CAN2", 110.0, "mavlink/SCALED_IMU2"),
    ):
        monitor.update(
            "left",
            MagSensorState(
                camera="left",
                namespace="uav_0",
                sensor_slot=slot,
                timestamp_s=1.0,
                topic=topic,
                magnitude=magnitude,
            ),
        )

    assert monitor.can_difference("left") == pytest.approx(0.01)
    assert monitor.can_difference_display("left") == (10.0, "mG")
