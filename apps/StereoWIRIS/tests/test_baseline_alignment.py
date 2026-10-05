from __future__ import annotations


def test_baseline_from_gps_outputs_enu_and_ned_contract() -> None:
    from wiris_stereo_distance.baseline_alignment import baseline_from_gps
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    left = GpsPositionState("left", "uav_0", 10.0, "gps", 37.0, 127.0, 10.0, fix_type=6)
    right = GpsPositionState("right", "uav_1", 10.01, "gps", 37.000001, 127.000002, 11.0, fix_type=6)

    baseline = baseline_from_gps(left, right)

    assert baseline is not None
    assert baseline.enu_m == (baseline.east_m, baseline.north_m, baseline.up_m)
    assert baseline.ned_m == (baseline.north_m, baseline.east_m, -baseline.up_m)
    assert baseline.altitude_delta_m == 1.0
    assert round(baseline.timestamp_delta_ms, 1) == 10.0


def test_baseline_quality_and_profile_yaml() -> None:
    from wiris_stereo_distance.baseline_alignment import (
        baseline_from_gps,
        baseline_profile_payload,
        dump_baseline_profile,
        evaluate_baseline_quality,
        load_baseline_profile,
    )
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    samples = []
    for index in range(5):
        left = GpsPositionState(
            "left",
            "uav_0",
            float(index),
            "gps",
            37.0,
            127.0,
            10.0,
            fix_type=6,
            eph_m=0.01,
            epv_m=0.02,
        )
        right = GpsPositionState(
            "right",
            "uav_1",
            float(index) + 0.01,
            "gps",
            37.0,
            127.000003,
            10.0,
            fix_type=6,
            eph_m=0.01,
            epv_m=0.02,
        )
        baseline = baseline_from_gps(left, right)
        assert baseline is not None
        samples.append((baseline, left, right))

    quality = evaluate_baseline_quality(samples, duration_s=5.0)

    assert quality is not None
    assert quality.grade == "GOOD"
    payload = baseline_profile_payload(
        left_gps1_topic="/left/gps1",
        left_gps2_topic="/left/gps2",
        right_gps1_topic="/right/gps1",
        right_gps2_topic="/right/gps2",
        baseline=samples[-1][0],
        quality=quality,
    )
    loaded = load_baseline_profile(dump_baseline_profile(payload))

    assert loaded["baseline_profile"]["baseline"]["frame"] == "ENU"
    assert loaded["baseline_profile"]["quality"]["grade"] == "GOOD"
