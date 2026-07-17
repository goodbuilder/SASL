from wiris_stereo_distance.uam_telemetry import (
    UamAlignmentProfile,
    UamTelemetrySample,
    compare_measurement_rows,
    comparison_summary,
    estimate_alignment_profile_from_rows,
    estimate_time_offset_s,
    load_ulog_local_position_samples,
    sample_at_time,
    sample_at_utc_time,
)


def test_sample_at_time_interpolates_bracketed_local_position() -> None:
    samples = [
        UamTelemetrySample(timestamp_s=10.0, x_m=0.0, y_m=0.0, z_m=0.0, source="ulog"),
        UamTelemetrySample(timestamp_s=12.0, x_m=2.0, y_m=4.0, z_m=-2.0, source="ulog"),
    ]

    matched = sample_at_time(samples, 11.0, max_dt_s=1.1)

    assert matched is not None
    sample, dt_s = matched
    assert dt_s == 0.0
    assert sample.x_m == 1.0
    assert sample.y_m == 2.0
    assert sample.z_m == -1.0


def test_sample_at_utc_time_interpolates_bracketed_local_position() -> None:
    samples = [
        UamTelemetrySample(
            timestamp_s=100.0,
            x_m=0.0,
            y_m=0.0,
            z_m=0.0,
            source="ulog",
            utc_unix_s=1_800_000_000.0,
        ),
        UamTelemetrySample(
            timestamp_s=102.0,
            x_m=2.0,
            y_m=4.0,
            z_m=-2.0,
            source="ulog",
            utc_unix_s=1_800_000_002.0,
        ),
    ]

    matched = sample_at_utc_time(samples, 1_800_000_001.0, max_dt_s=1.1)

    assert matched is not None
    sample, dt_s = matched
    assert dt_s == 0.0
    assert sample.timestamp_s == 101.0
    assert sample.utc_unix_s == 1_800_000_001.0
    assert sample.x_m == 1.0
    assert sample.source == "ulog:utc-interp"


def test_load_ulog_local_position_samples_maps_gps_utc_and_prefers_timestamp_sample(monkeypatch) -> None:
    import sys
    from types import SimpleNamespace

    class FakeULog:
        def __init__(self, _path: str) -> None:
            self.data_list = [
                SimpleNamespace(
                    name="vehicle_local_position",
                    data={
                        "timestamp": [10_600_000, 11_600_000],
                        "timestamp_sample": [10_500_000, 11_500_000],
                        "x": [1.0, 2.0],
                        "y": [3.0, 4.0],
                        "z": [-1.0, -2.0],
                    },
                ),
                SimpleNamespace(
                    name="vehicle_gps_position",
                    data={
                        "timestamp": [10_000_000, 12_000_000],
                        "timestamp_sample": [0, 0],
                        "time_utc_usec": [1_800_000_010_000_000, 1_800_000_012_000_000],
                    },
                ),
            ]

    monkeypatch.setitem(sys.modules, "pyulog", SimpleNamespace(ULog=FakeULog))

    samples, topic_name = load_ulog_local_position_samples("fake.ulg")

    assert topic_name == "vehicle_local_position"
    assert len(samples) == 2
    assert samples[0].timestamp_s == 10.5
    assert samples[0].timestamp_reference == "px4-timestamp-sample"
    assert samples[0].utc_unix_s == 1_800_000_010.5
    assert samples[1].utc_unix_s == 1_800_000_011.5


def test_load_ulog_local_position_samples_falls_back_to_gps_lla_to_ned(monkeypatch) -> None:
    import sys
    from types import SimpleNamespace

    class FakeULog:
        def __init__(self, _path: str) -> None:
            self.data_list = [
                SimpleNamespace(
                    name="vehicle_gps_position",
                    data={
                        "timestamp": [10_000_000, 11_000_000],
                        "lat": [37.0 * 1e7, 37.00001 * 1e7],
                        "lon": [127.0 * 1e7, 127.00002 * 1e7],
                        "alt": [100_000.0, 103_000.0],
                        "time_utc_usec": [1_800_000_010_000_000, 1_800_000_011_000_000],
                    },
                ),
            ]

    monkeypatch.setitem(sys.modules, "pyulog", SimpleNamespace(ULog=FakeULog))

    samples, topic_name = load_ulog_local_position_samples("fake-gps-only.ulg")

    assert topic_name == "vehicle_gps_position:gps_lla_to_ned:first_valid_gps_sample"
    assert len(samples) == 2
    assert samples[0].source == "ulog:vehicle_gps_position:gps_lla_to_ned:first_valid_gps_sample"
    assert samples[0].x_m == 0.0
    assert samples[0].y_m == 0.0
    assert samples[0].z_m == 0.0
    assert samples[0].gps_reference_kind == "first_valid_gps_sample"
    assert samples[0].gps_reference_lat_deg == 37.0
    assert samples[0].gps_reference_lon_deg == 127.0
    assert samples[0].gps_reference_alt_m == 100.0
    assert samples[1].x_m > 1.0
    assert samples[1].y_m > 1.0
    assert samples[1].z_m == -3.0
    assert samples[1].utc_unix_s == 1_800_000_011.0


def test_load_ulog_gps_fallback_prefers_px4_local_reference(monkeypatch) -> None:
    import sys
    from types import SimpleNamespace

    class FakeULog:
        def __init__(self, _path: str) -> None:
            self.data_list = [
                SimpleNamespace(
                    name="vehicle_local_position",
                    data={
                        "timestamp": [10_000_000],
                        "ref_lat": [37.0],
                        "ref_lon": [127.0],
                        "ref_alt": [100.0],
                    },
                ),
                SimpleNamespace(
                    name="vehicle_gps_position",
                    data={
                        "timestamp": [10_000_000],
                        "lat": [37.00001],
                        "lon": [127.00002],
                        "alt": [103.0],
                    },
                ),
            ]

    monkeypatch.setitem(sys.modules, "pyulog", SimpleNamespace(ULog=FakeULog))

    samples, topic_name = load_ulog_local_position_samples("fake-gps-ref.ulg")

    assert topic_name == "vehicle_gps_position:gps_lla_to_ned:vehicle_local_position_ref"
    assert len(samples) == 1
    assert samples[0].gps_reference_kind == "vehicle_local_position_ref"
    assert samples[0].x_m > 1.0
    assert samples[0].y_m > 1.0
    assert samples[0].z_m == -3.0


def test_load_ulog_local_position_samples_falls_back_to_vehicle_global_position(monkeypatch) -> None:
    import sys
    from types import SimpleNamespace

    class FakeULog:
        def __init__(self, _path: str) -> None:
            self.data_list = [
                SimpleNamespace(
                    name="vehicle_global_position",
                    data={
                        "timestamp": [10_000_000, 11_000_000],
                        "lat": [37.0, 37.00001],
                        "lon": [127.0, 127.00002],
                        "alt": [100.0, 103.0],
                    },
                ),
            ]

    monkeypatch.setitem(sys.modules, "pyulog", SimpleNamespace(ULog=FakeULog))

    samples, topic_name = load_ulog_local_position_samples("fake-global-only.ulg")

    assert topic_name == "vehicle_global_position:gps_lla_to_ned:first_valid_global_sample"
    assert samples[0].source_frame == "global_lla"
    assert samples[0].source_axis_contract == "lat_deg,lon_deg,alt_m"
    assert samples[0].raw_lat_deg == 37.0
    assert samples[1].x_m > 1.0
    assert samples[1].y_m > 1.0
    assert samples[1].z_m == -3.0


def test_load_ulog_local_position_samples_can_select_global_position(monkeypatch) -> None:
    import sys
    from types import SimpleNamespace

    class FakeULog:
        def __init__(self, _path: str) -> None:
            self.data_list = [
                SimpleNamespace(
                    name="vehicle_local_position",
                    data={
                        "timestamp": [10_000_000],
                        "x": [100.0],
                        "y": [100.0],
                        "z": [100.0],
                    },
                ),
                SimpleNamespace(
                    name="vehicle_global_position",
                    data={
                        "timestamp": [10_000_000, 11_000_000],
                        "lat": [37.0, 37.00001],
                        "lon": [127.0, 127.00002],
                        "alt": [100.0, 103.0],
                    },
                ),
            ]

    monkeypatch.setitem(sys.modules, "pyulog", SimpleNamespace(ULog=FakeULog))

    samples, topic_name = load_ulog_local_position_samples("fake-mixed.ulg", truth_source_type="global_position")

    assert topic_name == "vehicle_global_position:gps_lla_to_ned:first_valid_global_sample"
    assert len(samples) == 2
    assert samples[0].source_frame == "global_lla"
    assert samples[0].source_axis_contract == "lat_deg,lon_deg,alt_m"
    assert samples[0].raw_lat_deg == 37.0
    assert samples[0].x_m == 0.0
    assert samples[1].x_m > 1.0


def test_load_ulog_local_position_samples_can_select_gps_position(monkeypatch) -> None:
    import sys
    from types import SimpleNamespace

    class FakeULog:
        def __init__(self, _path: str) -> None:
            self.data_list = [
                SimpleNamespace(
                    name="vehicle_local_position",
                    data={
                        "timestamp": [10_000_000],
                        "x": [100.0],
                        "y": [100.0],
                        "z": [100.0],
                        "ref_lat": [37.0],
                        "ref_lon": [127.0],
                        "ref_alt": [100.0],
                    },
                ),
                SimpleNamespace(
                    name="vehicle_gps_position",
                    data={
                        "timestamp": [10_000_000],
                        "lat": [37.00001],
                        "lon": [127.00002],
                        "alt": [103.0],
                    },
                ),
            ]

    monkeypatch.setitem(sys.modules, "pyulog", SimpleNamespace(ULog=FakeULog))

    samples, topic_name = load_ulog_local_position_samples("fake-mixed.ulg", truth_source_type="gps_position")

    assert topic_name == "vehicle_gps_position:gps_lla_to_ned:vehicle_local_position_ref"
    assert len(samples) == 1
    assert samples[0].source == "ulog:vehicle_gps_position:gps_lla_to_ned:vehicle_local_position_ref"
    assert samples[0].x_m > 1.0
    assert samples[0].y_m > 1.0
    assert samples[0].z_m == -3.0


def test_compare_measurement_rows_uses_ulog_truth_first() -> None:
    rows = [
        {
            "timestamp_s": "11.0",
            "x_m": "1.5",
            "y_m": "2.0",
            "z_m": "-1.0",
            "distance_m": "2.692582",
            "drone_x_m": "100.0",
            "drone_y_m": "100.0",
            "drone_z_m": "100.0",
        }
    ]
    samples = [
        UamTelemetrySample(timestamp_s=10.0, x_m=0.0, y_m=0.0, z_m=0.0, source="ulog"),
        UamTelemetrySample(timestamp_s=12.0, x_m=2.0, y_m=4.0, z_m=-2.0, source="ulog"),
    ]

    comparisons = compare_measurement_rows(rows, samples, truth_source="ulog", max_dt_s=1.1)

    assert len(comparisons) == 1
    assert comparisons[0]["truth_source"] == "ulog:interp"
    assert comparisons[0]["truth_time_basis"] == "px4-boot"
    assert comparisons[0]["truth_x_m"] == "1.000000"
    assert comparisons[0]["truth_north_m"] == "1.000000"
    assert comparisons[0]["measurement_frame"] == "stereo_center_opencv_optical"
    assert comparisons[0]["comparison_frame"] == "mixed-frame"
    assert comparisons[0]["error_x_m"] == "0.500000"
    assert comparisons[0]["camera_time_sync_status"] == "unrecorded"
    assert not any(key.endswith("_eligible") or key.endswith("_blocker") for key in comparisons[0])


def test_compare_measurement_rows_preserves_camera_time_sync_status() -> None:
    rows = [
        {
            "timestamp_s": "1.0",
            "x_m": "1.0",
            "y_m": "2.0",
            "z_m": "3.0",
            "distance_m": "3.741657",
            "camera_time_sync_status": "pc-offset-failed",
            "camera_time_sync_basis": "rtcp-sender-report-ntp",
            "camera_time_sync_warning": "RTCP/NTP와 PC UTC 차이 초과",
            "rtcp_pair_delta_s": "0.020000",
            "rtcp_pc_offset_max_ms": "8600.000",
        }
    ]
    samples = [UamTelemetrySample(timestamp_s=1.0, x_m=1.0, y_m=2.0, z_m=3.0, source="ulog")]

    comparisons = compare_measurement_rows(rows, samples, truth_source="ulog", max_dt_s=0.01)

    assert len(comparisons) == 1
    assert comparisons[0]["camera_time_sync_status"] == "pc-offset-failed"
    assert comparisons[0]["camera_time_sync_basis"] == "rtcp-sender-report-ntp"
    assert comparisons[0]["rtcp_pair_delta_s"] == "0.020000"
    assert comparisons[0]["rtcp_pc_offset_max_ms"] == "8600.000"


def test_compare_measurement_rows_prefers_raw_stereo_center_over_live_bias() -> None:
    rows = [
        {
            "timestamp_s": "1.0",
            "x_m": "9.0",
            "y_m": "0.0",
            "z_m": "0.0",
            "distance_m": "9.0",
            "raw_x_m": "10.0",
            "raw_y_m": "0.0",
            "raw_z_m": "0.0",
            "raw_distance_m": "10.0",
            "bias_profile_id": "operational_bias",
        }
    ]
    samples = [UamTelemetrySample(timestamp_s=1.0, x_m=10.0, y_m=0.0, z_m=0.0, source="ulog")]

    comparisons = compare_measurement_rows(rows, samples, truth_source="ulog", max_dt_s=0.01)

    assert len(comparisons) == 1
    assert comparisons[0]["measurement_value_kind"] == "raw-stereo-center"
    assert comparisons[0]["measurement_right_m"] == "10.000000"
    assert comparisons[0]["measurement_distance_m"] == "10.000000"
    assert comparisons[0]["estimated_range_m"] == "10.000000"
    assert "estimated_baseline_center_distance_m" not in comparisons[0]
    assert comparisons[0]["measurement_bias_policy"] == "recorded live bias ignored; raw stereo-center measurement used"


def test_compare_measurement_rows_prefers_utc_truth_when_available() -> None:
    rows = [
        {
            "timestamp_s": "10.0",
            "bbox_mid_utc_unix_s": "1800000000.500000",
            "x_m": "1.5",
            "y_m": "2.0",
            "z_m": "-1.0",
            "distance_m": "2.692582",
        }
    ]
    samples = [
        UamTelemetrySample(
            timestamp_s=100.0,
            x_m=0.0,
            y_m=0.0,
            z_m=0.0,
            source="ulog",
            utc_unix_s=1_800_000_000.0,
        ),
        UamTelemetrySample(
            timestamp_s=101.0,
            x_m=2.0,
            y_m=4.0,
            z_m=-2.0,
            source="ulog",
            utc_unix_s=1_800_000_001.0,
        ),
    ]

    comparisons = compare_measurement_rows(rows, samples, truth_source="ulog", max_dt_s=0.6)

    assert len(comparisons) == 1
    assert comparisons[0]["measurement_time_basis"] == "bbox_mid_utc_unix_s"
    assert comparisons[0]["truth_time_basis"] == "utc-unix"
    assert comparisons[0]["truth_source"] == "ulog:utc-interp"
    assert comparisons[0]["truth_timestamp_s"] == "100.500000"
    assert comparisons[0]["truth_utc_unix_s"] == "1800000000.500000"
    assert comparisons[0]["truth_x_m"] == "1.000000"
    assert comparisons[0]["truth_north_m"] == "1.000000"


def test_estimate_alignment_profile_maps_camera_points_to_truth_ned() -> None:
    def camera_to_ned(point: tuple[float, float, float]) -> tuple[float, float, float]:
        right, down, forward = point
        return (10.0 + forward, 20.0 + right, 30.0 + down)

    camera_points = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (2.0, 1.0, 3.0),
    ]
    rows = [
        {
            "timestamp_s": f"{index + 1}.0",
            "x_m": f"{point[0]}",
            "y_m": f"{point[1]}",
            "z_m": f"{point[2]}",
            "distance_m": "1.0",
        }
        for index, point in enumerate(camera_points)
    ]
    samples = [
        UamTelemetrySample(
            timestamp_s=float(index + 1),
            x_m=camera_to_ned(point)[0],
            y_m=camera_to_ned(point)[1],
            z_m=camera_to_ned(point)[2],
            source="ulog:test",
        )
        for index, point in enumerate(camera_points)
    ]

    raw_comparisons = compare_measurement_rows(rows, samples, truth_source="ulog", max_dt_s=0.01)
    profile = estimate_alignment_profile_from_rows(raw_comparisons, profile_id="test_profile")
    aligned = compare_measurement_rows(
        rows,
        samples,
        truth_source="ulog",
        max_dt_s=0.01,
        alignment_profile=profile,
    )

    assert profile.profile_id == "test_profile"
    assert profile.sample_count == 4
    assert profile.rmse_m < 1e-9
    assert aligned[0]["comparison_frame"] == "px4_local_ned"
    assert not any(key.endswith("_eligible") or key.endswith("_blocker") for key in aligned[0])
    assert aligned[0]["alignment_profile_id"] == "test_profile"
    assert aligned[0]["estimated_north_m"] == "10.000000"
    assert aligned[0]["estimated_east_m"] == "21.000000"
    assert aligned[0]["estimated_down_m"] == "30.000000"
    assert aligned[0]["truth_rdf_right_m"] == "1.000000"
    assert aligned[0]["truth_rdf_down_m"] == "0.000000"
    assert aligned[0]["truth_rdf_forward_m"] == "0.000000"
    assert aligned[0]["truth_rdf_range_m"] == "1.000000"
    assert aligned[0]["measurement_frd_forward_m"] == "0.000000"
    assert aligned[0]["measurement_frd_right_m"] == "1.000000"
    assert aligned[0]["measurement_frd_down_m"] == "0.000000"
    assert aligned[0]["truth_frd_forward_m"] == "0.000000"
    assert aligned[0]["truth_frd_right_m"] == "1.000000"
    assert aligned[0]["truth_frd_down_m"] == "0.000000"
    assert aligned[0]["truth_frd_range_m"] == "1.000000"
    assert aligned[0]["baseline_center_north_m"] == "10.000000"
    assert aligned[0]["baseline_center_east_m"] == "20.000000"
    assert aligned[0]["baseline_center_down_m"] == "30.000000"
    assert aligned[0]["estimated_range_m"] == "1.000000"
    assert aligned[0]["truth_range_m"] == "1.000000"
    assert "estimated_baseline_center_distance_m" not in aligned[0]
    assert "truth_baseline_center_distance_m" not in aligned[0]
    assert aligned[0]["range_error_m"] == "0.000000"
    assert aligned[0]["range_error_basis"] == "baseline_center"
    assert aligned[0]["error_north_m"] == "0.000000"
    assert aligned[0]["error_east_m"] == "0.000000"
    assert aligned[0]["error_down_m"] == "0.000000"
    assert aligned[0]["error_rdf_right_m"] == "0.000000"
    assert aligned[0]["error_rdf_down_m"] == "0.000000"
    assert aligned[0]["error_rdf_forward_m"] == "0.000000"
    assert aligned[0]["error_frd_forward_m"] == "0.000000"
    assert aligned[0]["error_frd_right_m"] == "0.000000"
    assert aligned[0]["error_frd_down_m"] == "0.000000"
    assert aligned[0]["error_distance_kind"] == "position_error_m"


def test_alignment_profile_records_truth_source_contract() -> None:
    rows = [
        {
            "measurement_right_m": str(index),
            "measurement_down_m": str(index % 2),
            "measurement_forward_m": str(index + 1),
            "truth_north_m": str(index + 10),
            "truth_east_m": str(index % 2 + 20),
            "truth_down_m": str(index + 31),
            "truth_source_type": "gps_position",
            "truth_source_frame": "global_lla",
            "truth_source_axis_contract": "lat_deg,lon_deg,alt_m",
            "truth_time_basis": "utc-unix",
            "truth_gps_reference_kind": "vehicle_local_position_ref",
        }
        for index in range(4)
    ]

    profile = estimate_alignment_profile_from_rows(rows, profile_id="gps_profile")
    payload = profile.to_dict()
    restored = UamAlignmentProfile.from_dict(payload)

    assert profile.truth_source_type == "gps_position"
    assert profile.truth_source_frame == "global_lla"
    assert profile.truth_axis_contract == "lat_deg,lon_deg,alt_m"
    assert profile.truth_time_basis == "utc-unix"
    assert profile.truth_origin_kind == "vehicle_local_position_ref"
    assert restored == profile


def test_alignment_profile_rejects_mixed_truth_source_types() -> None:
    rows = [
        {
            "measurement_right_m": "0",
            "measurement_down_m": "0",
            "measurement_forward_m": "1",
            "truth_north_m": "10",
            "truth_east_m": "20",
            "truth_down_m": "31",
            "truth_source_type": "local_ned",
            "truth_source_frame": "px4_local_ned",
            "truth_source_axis_contract": "n/north,e/east,d/down",
        },
        {
            "measurement_right_m": "1",
            "measurement_down_m": "0",
            "measurement_forward_m": "2",
            "truth_north_m": "11",
            "truth_east_m": "20",
            "truth_down_m": "32",
            "truth_source_type": "gps_position",
            "truth_source_frame": "global_lla",
            "truth_source_axis_contract": "lat_deg,lon_deg,alt_m",
        },
        {
            "measurement_right_m": "0",
            "measurement_down_m": "1",
            "measurement_forward_m": "3",
            "truth_north_m": "10",
            "truth_east_m": "21",
            "truth_down_m": "33",
            "truth_source_type": "local_ned",
            "truth_source_frame": "px4_local_ned",
            "truth_source_axis_contract": "n/north,e/east,d/down",
        },
    ]

    try:
        estimate_alignment_profile_from_rows(rows)
    except Exception as exc:
        assert "mix multiple truth source types" in str(exc)
    else:
        raise AssertionError("mixed truth sources must not produce one alignment profile")


def test_uam_alignment_profile_round_trips_json_payload() -> None:
    profile = UamAlignmentProfile(
        profile_id="roundtrip",
        rotation_matrix=((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        origin_offset_m=(10.0, 20.0, 30.0),
        sample_count=5,
        rmse_m=0.01,
        max_error_m=0.02,
        created_unix_s=1_800_000_000.0,
    )

    restored = UamAlignmentProfile.from_dict(profile.to_dict())

    assert restored == profile
    assert restored.apply((1.0, 2.0, 3.0)) == (13.0, 21.0, 32.0)


def test_compare_measurement_rows_falls_back_to_recorded_drone_position() -> None:
    rows = [
        {
            "bbox_mid_timestamp_s": "20.0",
            "corrected_x_m": "9.0",
            "corrected_y_m": "1.0",
            "corrected_z_m": "2.0",
            "corrected_distance_m": "9.273618",
            "drone_x_m": "8.0",
            "drone_y_m": "1.5",
            "drone_z_m": "2.0",
            "drone_age_s": "0.25",
        }
    ]

    comparisons = compare_measurement_rows(
        rows,
        [],
        truth_source="embedded-recorded-row",
        max_dt_s=0.25,
    )

    assert len(comparisons) == 1
    assert comparisons[0]["truth_source"] == "live-recorded-row"
    assert comparisons[0]["truth_time_basis"] == "app-receive"
    assert comparisons[0]["truth_timestamp_s"] == "19.750000"
    assert comparisons[0]["truth_east_m"] == "1.500000"
    assert comparisons[0]["error_y_m"] == "-0.500000"


def test_compare_measurement_rows_prefers_recorded_truth_ned_aliases() -> None:
    rows = [
        {
            "bbox_mid_timestamp_s": "20.0",
            "corrected_x_m": "1.0",
            "corrected_y_m": "2.0",
            "corrected_z_m": "3.0",
            "drone_north_m": "0.0",
            "drone_east_m": "-1.0",
            "drone_down_m": "2.0",
            "drone_x_m": "99.0",
            "drone_y_m": "99.0",
            "drone_z_m": "99.0",
            "drone_timestamp_s": "20.0",
        }
    ]

    comparisons = compare_measurement_rows(
        rows,
        [],
        truth_source="embedded-recorded-row",
        max_dt_s=0.25,
    )

    assert len(comparisons) == 1
    assert comparisons[0]["truth_north_m"] == "0.000000"
    assert comparisons[0]["truth_east_m"] == "-1.000000"
    assert comparisons[0]["truth_down_m"] == "2.000000"
    assert comparisons[0]["error_x_m"] == "1.000000"


def test_compare_measurement_rows_can_fall_back_to_recorded_drone_utc() -> None:
    rows = [
        {
            "bbox_mid_timestamp_s": "20.0",
            "bbox_mid_utc_unix_s": "1800000020.000000",
            "corrected_x_m": "9.0",
            "corrected_y_m": "1.0",
            "corrected_z_m": "2.0",
            "corrected_distance_m": "9.273618",
            "drone_x_m": "8.0",
            "drone_y_m": "1.5",
            "drone_z_m": "2.0",
            "drone_timestamp_s": "19.9",
            "drone_utc_unix_s": "1800000019.950000",
        }
    ]

    comparisons = compare_measurement_rows(
        rows,
        [],
        truth_source="embedded-recorded-row",
        max_dt_s=0.25,
    )

    assert len(comparisons) == 1
    assert comparisons[0]["truth_time_basis"] == "embedded-utc-unix"
    assert comparisons[0]["truth_dt_s"] == "0.050000"
    assert comparisons[0]["truth_utc_unix_s"] == "1800000019.950000"


def test_comparison_summary_reports_distance_error_metrics() -> None:
    summary = comparison_summary(
        [
            {"error_distance_m": "-1.0"},
            {"error_distance_m": "2.0"},
            {"error_distance_m": "3.0"},
        ]
    )

    assert summary["count"] == 3.0
    assert summary["median_abs_m"] == 2.0
    assert summary["p95_abs_m"] == 3.0
    assert round(summary["rmse_m"], 6) == 2.160247


def test_estimate_time_offset_matches_shifted_ulog_samples() -> None:
    rows = [
        {"timestamp_s": "1.0", "x_m": "0.0", "y_m": "0.0", "z_m": "0.0", "distance_m": "0.0"},
        {"timestamp_s": "2.0", "x_m": "1.0", "y_m": "0.0", "z_m": "0.0", "distance_m": "1.0"},
        {"timestamp_s": "3.0", "x_m": "2.0", "y_m": "0.0", "z_m": "0.0", "distance_m": "2.0"},
    ]
    samples = [
        UamTelemetrySample(timestamp_s=11.0, x_m=0.0, y_m=0.0, z_m=0.0, source="ulog"),
        UamTelemetrySample(timestamp_s=12.0, x_m=1.0, y_m=0.0, z_m=0.0, source="ulog"),
        UamTelemetrySample(timestamp_s=13.0, x_m=2.0, y_m=0.0, z_m=0.0, source="ulog"),
    ]

    estimate = estimate_time_offset_s(rows, samples, max_dt_s=0.1)

    assert estimate is not None
    assert estimate["time_offset_s"] == 10.0
    assert estimate["matched_count"] == 3.0
    assert estimate["mean_abs_dt_s"] == 0.0


def test_estimate_time_offset_uses_utc_basis_when_available() -> None:
    rows = [
        {
            "timestamp_s": "1.0",
            "bbox_mid_utc_unix_s": "1800000100.5",
            "x_m": "0.0",
            "y_m": "0.0",
            "z_m": "0.0",
            "distance_m": "0.0",
        },
        {
            "timestamp_s": "2.0",
            "bbox_mid_utc_unix_s": "1800000101.5",
            "x_m": "1.0",
            "y_m": "0.0",
            "z_m": "0.0",
            "distance_m": "1.0",
        },
        {
            "timestamp_s": "3.0",
            "bbox_mid_utc_unix_s": "1800000102.5",
            "x_m": "2.0",
            "y_m": "0.0",
            "z_m": "0.0",
            "distance_m": "2.0",
        },
    ]
    samples = [
        UamTelemetrySample(
            timestamp_s=101.0,
            x_m=0.0,
            y_m=0.0,
            z_m=0.0,
            source="ulog",
            utc_unix_s=1_800_000_100.0,
        ),
        UamTelemetrySample(
            timestamp_s=102.0,
            x_m=1.0,
            y_m=0.0,
            z_m=0.0,
            source="ulog",
            utc_unix_s=1_800_000_101.0,
        ),
        UamTelemetrySample(
            timestamp_s=103.0,
            x_m=2.0,
            y_m=0.0,
            z_m=0.0,
            source="ulog",
            utc_unix_s=1_800_000_102.0,
        ),
    ]

    estimate = estimate_time_offset_s(rows, samples, max_dt_s=0.1)

    assert estimate is not None
    assert estimate["time_offset_s"] == -0.5
    assert estimate["matched_count"] == 3.0
    assert estimate["mean_abs_dt_s"] == 0.0
