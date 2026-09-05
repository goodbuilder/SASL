from __future__ import annotations

import json
from types import SimpleNamespace


def _gps(side: str, *, timestamp_s: float, east_offset: float = 0.0):
    from Workstation.workstation_core.rtk_geometry import GpsPositionState

    return GpsPositionState(
        camera=side,
        namespace=side,
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=37.0,
        longitude_deg=127.0 + east_offset,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
        gps_utc_us=1_700_000_000_000_000 + int(timestamp_s * 1_000_000),
        mavlink_source_system={"left": 1, "right": 2, "drone": 3}[side],
        mavlink_source_component=1,
    )


def _panel(*, active: bool = True, admitted: bool = True):
    p1 = _gps("left", timestamp_s=100.0)
    p2 = _gps("right", timestamp_s=100.5, east_offset=0.00001)
    calls: list[tuple[str, dict[str, object], bool]] = []
    panel = SimpleNamespace(
        distance_recording_enabled=active,
        latest_gps={"left": p1, "right": p2},
        _selected_rover_gps_state=lambda side: p1 if side == "left" else p2,
        _pixhawk_gnss_source=lambda side: "GPS2" if side == "right" else "GPS1",
        _gnss_baseline_center_recording_sequence=0,
        _raw_timeline_row=lambda **kwargs: {
            "measurement_id": "m-live",
            "side": kwargs["side"],
            "sequence": kwargs["sequence"],
            "sample_monotonic_ns": int(kwargs["sample_monotonic_s"] * 1_000_000_000),
            "sample_monotonic_s": f"{kwargs['sample_monotonic_s']:.9f}",
            "session_elapsed_s": "1.000000000",
            "session_utc_ns": 1_700_000_101_000_000_000,
            "session_utc_s": "1700000101.000000000",
            "session_utc_method": "host_unix_monotonic_session_anchor",
            "session_utc_anchor_uncertainty_ns": 12345,
            "host_receive_monotonic_ns": kwargs["host_receive_monotonic_ns"],
            "host_receive_utc_ns": kwargs["host_receive_utc_ns"],
            "source_utc_ns": kwargs["source_utc_ns"],
            "source_utc_s": "1700000101.000000000",
            "source_utc_valid": "true",
            "source_utc_method": kwargs["source_utc_method"],
            "sample_type": kwargs["sample_type"],
            "row_contract": kwargs["row_contract"],
        },
        append_workstation_data=lambda stream, row, priority=False: calls.append(
            (stream, dict(row), bool(priority))
        )
        or admitted,
    )
    return panel, calls


def test_recording_owner_writes_one_primary_drone_row_with_exact_sources(monkeypatch) -> None:
    from Workstation import recording_controller as module

    panel, calls = _panel()
    monkeypatch.setattr(module.time, "monotonic", lambda: 101.0)
    monkeypatch.setattr(module.time, "monotonic_ns", lambda: 101_000_000_123)
    monkeypatch.setattr(module.time, "time_ns", lambda: 1_700_000_101_000_000_456)
    drone = _gps("drone", timestamp_s=101.0, east_offset=0.00010)

    assert module.RecordingController(panel)._record_gnss_baseline_center("drone", drone)
    assert len(calls) == 1
    stream, row, priority = calls[0]
    assert stream == "gnss.baseline.center"
    assert priority is False
    assert row["measurement_id"] == "m-live"
    assert row["geometry_state"] == "CURRENT"
    assert row["p1_source_selection"] == "GPS1"
    assert row["p2_source_selection"] == "GPS2"
    assert row["drone_app_receive_monotonic_ns"] == 101_000_000_000
    assert row["source_utc_ns"] == drone.gps_utc_us * 1000
    persisted_shape = {
        **row,
        "schema": "workstation-measurement-data-record-v1",
        "recorded_utc_ns": 1_700_000_101_000_000_456,
        "recorded_monotonic_ns": 101_000_000_123,
    }
    assert len(json.dumps(persisted_shape, allow_nan=False).encode("utf-8")) <= 4096
    assert panel._gnss_baseline_center_recording_sequence == 1


def test_recording_gate_gps2_and_writer_rejection_do_not_claim_rows(monkeypatch) -> None:
    from Workstation import recording_controller as module

    drone = _gps("drone", timestamp_s=101.0, east_offset=0.00010)
    inactive, inactive_calls = _panel(active=False)
    assert module.RecordingController(inactive)._record_gnss_baseline_center("drone", drone) is False
    assert inactive_calls == []

    active, active_calls = _panel(admitted=True)
    assert module.RecordingController(active)._record_gnss_baseline_center("drone_GPS2", drone) is False
    assert active_calls == []

    rejected, rejected_calls = _panel(admitted=False)
    monkeypatch.setattr(module.time, "monotonic", lambda: 101.0)
    assert module.RecordingController(rejected)._record_gnss_baseline_center("drone", drone) is False
    assert len(rejected_calls) == 1
    assert rejected._gnss_baseline_center_recording_sequence == 1


def test_missing_selected_p1_p2_still_records_waiting_evidence(monkeypatch) -> None:
    from Workstation import recording_controller as module

    panel, calls = _panel()
    panel._selected_rover_gps_state = lambda _side: None
    monkeypatch.setattr(module.time, "monotonic", lambda: 101.0)
    drone = _gps("drone", timestamp_s=101.0, east_offset=0.00010)

    assert module.RecordingController(panel)._record_gnss_baseline_center("drone", drone)
    [(_stream, row, _priority)] = calls
    assert row["geometry_state"] == "WAITING"
    assert row["gnss_baseline_center_forward_m"] is None
    assert row["p1_position_available"] is False
    assert row["p2_position_available"] is False
    assert row["drone_position_available"] is True


def test_sensor_handoff_occurs_after_raw_primary_drone_row_and_excludes_gps2() -> None:
    from Workstation.sensor_presentation_controller import SensorPresentationController

    calls: list[tuple[str, str]] = []
    panel = SimpleNamespace(
        latest_gps={},
        gnss_monitor=SimpleNamespace(update_position=lambda *_args: None),
        baseline_gps_rate_history={},
        _fmt_record_float=lambda value, _digits: value,
        _record_external_timeline_row=lambda camera, _sample_type, _values: calls.append(
            ("raw", camera)
        ),
        _record_gnss_baseline_center=lambda camera, _gps_state: calls.append(
            ("derived", camera)
        ),
        _telemetry_source_label=lambda topic: topic,
        _refresh_rtk_baseline=lambda: None,
        _refresh_baseline_alignment_ui=lambda: None,
        _refresh_gnss_base_tab=lambda: None,
        _refresh_gnss_operation_readiness=lambda: None,
        _append_baseline_sample_if_active=lambda: None,
        _refresh_distance=lambda: None,
    )
    controller = SensorPresentationController(panel)
    primary = _gps("drone", timestamp_s=10.0)
    controller._update_gps("drone", primary)
    assert calls == [("raw", "drone"), ("derived", "drone")]
    assert panel.latest_gps["drone"] is primary

    calls.clear()
    secondary = _gps("drone", timestamp_s=11.0)
    controller._update_gps("drone_GPS2", secondary)
    assert calls == [("raw", "drone_GPS2")]
    assert panel.latest_gps["drone_GPS2"] is secondary


def test_recording_owner_to_real_store_publishes_exact_jsonl(tmp_path, monkeypatch) -> None:
    from Workstation import recording_controller as module
    from Workstation.workstation_core.measurement_data_store import MeasurementDataStore

    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    panel, _calls = _panel()
    panel.append_workstation_data = (
        lambda stream, row, priority=False: store.enqueue(
            stream,
            row,
            priority=priority,
        )
    )
    monkeypatch.setattr(module.time, "monotonic", lambda: 101.0)
    drone = _gps("drone", timestamp_s=101.0, east_offset=0.00010)

    assert module.RecordingController(panel)._record_gnss_baseline_center("drone", drone)
    assert store.close(timeout_s=2.0)

    path = tmp_path / "gnss_baseline_center.jsonl"
    [row] = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert row["measurement_id"] == "m-live"
    assert row["geometry_state"] == "CURRENT"
    assert row["coordinate_frame"] == "gnss_baseline_center_local_level_frd"
    assert row["p1_source_selection"] == "GPS1"
    assert row["p2_source_selection"] == "GPS2"
    assert row["schema"] == "workstation-measurement-data-record-v1"
    assert store.snapshot()["streams"]["gnss.baseline.center"]["written_records"] == 1
