from __future__ import annotations

from types import SimpleNamespace

from PyQt5 import QtWidgets

from Workstation.measurement_run_controller import MeasurementRunController


def _panel(*, storage_accepts: bool = True) -> SimpleNamespace:
    events: list[tuple[str, dict[str, object]]] = []
    exports: list[tuple[tuple[object, ...], dict[str, object]]] = []

    def append(stream: str, payload: dict[str, object], **_kwargs: object) -> bool:
        events.append((stream, dict(payload)))
        return storage_accepts

    return SimpleNamespace(
        # Historical state deliberately looks unrecoverable. The transient
        # controller must never inspect it for admission.
        _measurement_session_state="RECOVERABLE",
        _measurement_session_id="old-open-session",
        recording_session_id="old-open-session",
        _measurement_view_controller_owner=None,
        _measurement_view_controller=lambda: SimpleNamespace(
            measurement_method_config_for_new_measurement=lambda: {
                "measurement_preferred_method": "pixhawk_rtk_live",
            }
        ),
        retry_data_storage=lambda: {"state": "UNAVAILABLE"},
        append_workstation_data=append,
        flush_workstation_data=lambda: True,
        data_storage_snapshot=lambda: {
            "state": "UNAVAILABLE" if not storage_accepts else "OK",
            "streams": {},
        },
        _refresh_operation_header=lambda: None,
        _events=events,
        _exports=exports,
    )


def test_old_recoverable_session_and_storage_failure_do_not_block_measurement() -> None:
    panel = _panel(storage_accepts=False)
    controller = MeasurementRunController(panel)

    assert controller.begin_measurement_run() is True
    assert controller.state == "RUNNING"
    assert panel._measurement_id.startswith("run-")
    assert controller.confirm_measurement_running(()) is True
    assert controller.confirm_measurement_running(("left", "right")) is True
    assert controller.state == "RUNNING"
    assert controller.request_measurement_stop() is True
    assert controller.state == "IDLE"
    assert controller.finish_measurement_run() is True
    assert controller.state == "IDLE"
    assert panel._measurement_id == ""


def test_sequential_runs_receive_new_transient_generations_without_package_state() -> None:
    panel = _panel()
    controller = MeasurementRunController(panel)

    assert controller.begin_measurement_run()
    first = panel._measurement_run_generation
    assert controller.confirm_measurement_running(("left",))
    assert controller.request_measurement_stop()
    assert controller.state == "IDLE"
    assert controller.finish_measurement_run()
    assert controller.begin_measurement_run()
    second = panel._measurement_run_generation
    assert first != second
    assert panel._measurement_session_state == "RECOVERABLE"
    assert [payload["event_type"] for _stream, payload in panel._events] == [
        "measurement_start_requested",
        "measurement_source_transition",
        "measurement_stopped",
        "measurement_start_requested",
    ]


def test_each_measurement_freezes_current_timing_preference_at_its_boundary() -> None:
    panel = _panel()
    selected = {"basis": "app_receive_monotonic"}
    boundaries: list[str] = []
    owner = SimpleNamespace(
        begin_pairing_generation=lambda: boundaries.append(selected["basis"]),
        measurement_method_config_for_new_measurement=lambda: {
            "measurement_preferred_method": "pixhawk_rtk_live",
            "measurement_pairing_time_basis_requested": selected["basis"],
        },
    )
    panel._measurement_view_controller = lambda: owner
    controller = MeasurementRunController(panel)

    assert controller.begin_measurement_run()
    assert panel._measurement_run_metadata[
        "measurement_pairing_time_basis_requested"
    ] == "app_receive_monotonic"
    selected["basis"] = "camera_rtcp"
    assert panel._measurement_run_metadata[
        "measurement_pairing_time_basis_requested"
    ] == "app_receive_monotonic"

    assert controller.request_measurement_stop()
    assert controller.finish_measurement_run()
    assert controller.begin_measurement_run()
    assert panel._measurement_run_metadata[
        "measurement_pairing_time_basis_requested"
    ] == "camera_rtcp"
    assert boundaries == ["app_receive_monotonic", "camera_rtcp"]


def test_measurement_start_records_verified_pixhawk_mag_selection() -> None:
    panel = _panel()
    panel._last_measurement_mag_gate = {
        "schema": "pixhawk-mag-measurement-gate-v1",
        "required": True,
        "required_sides": ["left", "right"],
        "ready": True,
        "blockers": [],
        "sides": {
            "left": {"state": "ACTIVE VERIFIED", "actual_device_id": 10},
            "right": {"state": "ACTIVE VERIFIED", "actual_device_id": 20},
        },
    }
    controller = MeasurementRunController(panel)

    assert controller.begin_measurement_run()
    assert panel._measurement_run_metadata["pixhawk_mag_selection"]["ready"] is True
    assert panel._measurement_run_metadata["pixhawk_mag_selection"]["sides"]["right"]["actual_device_id"] == 20


def test_busy_start_is_queued_and_duplicate_live_start_is_rejected() -> None:
    panel = _panel()
    controller = MeasurementRunController(panel)
    assert controller.queue_measurement_start("camera time sync")
    assert controller.state == "QUEUED"
    assert controller.begin_measurement_run()
    assert controller.state == "RUNNING"
    assert controller.begin_measurement_run() is False


def test_failed_camera_start_keeps_observation_generation_running() -> None:
    panel = _panel()
    controller = MeasurementRunController(panel)
    assert controller.begin_measurement_run()
    generation = panel._measurement_run_generation

    assert controller.fail_measurement_start("no current-generation frame")

    snapshot = controller.measurement_run_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert snapshot["generation"] == generation
    assert snapshot["error"] == ""
    assert snapshot["sources"]["camera.left"]["state"] == "UNAVAILABLE"
    assert snapshot["sources"]["camera.right"]["state"] == "UNAVAILABLE"
    assert [payload["event_type"] for _stream, payload in panel._events] == [
        "measurement_start_requested",
        "measurement_source_transition",
        "measurement_source_transition",
        "measurement_source_transition",
        "measurement_source_transition",
    ]
    assert controller.begin_measurement_run() is False


def test_sources_join_and_leave_without_owning_run_state() -> None:
    panel = _panel(storage_accepts=False)
    controller = MeasurementRunController(panel)
    assert controller.begin_measurement_run()

    assert controller.measurement_source_transition(
        "pixhawk.left", "LIVE", "fresh expected-system packet"
    )
    event_count = len(panel._events)
    assert controller.measurement_source_transition(
        "pixhawk.left", "LIVE", "fresh expected-system packet"
    )
    assert len(panel._events) == event_count
    assert controller.measurement_source_transition(
        "camera.left", "LIVE", "current-generation frame"
    )
    assert controller.measurement_source_transition(
        "camera.left", "UNAVAILABLE", "frame timeout"
    )

    snapshot = controller.measurement_run_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert snapshot["sources"]["pixhawk.left"]["state"] == "LIVE"
    assert snapshot["sources"]["camera.left"]["state"] == "UNAVAILABLE"
    assert snapshot["sources"]["camera.left"]["joined_monotonic_ns"] is not None
    assert snapshot["sources"]["camera.left"]["left_monotonic_ns"] is not None


def test_stop_boundary_immediately_admits_a_new_run_before_old_teardown() -> None:
    panel = _panel(storage_accepts=False)
    controller = MeasurementRunController(panel)
    assert controller.begin_measurement_run()
    first = panel._measurement_run_generation

    assert controller.request_measurement_stop()
    assert controller.state == "IDLE"
    assert panel._measurement_run_last_generation == first
    assert controller.begin_measurement_run()
    assert controller.state == "RUNNING"
    assert panel._measurement_run_generation != first

    # A late old-owner completion is now a harmless no-op and cannot clear
    # the current observation generation.
    current = panel._measurement_run_generation
    assert controller.finish_measurement_run(first) is True
    assert panel._measurement_run_generation == current


def test_start_and_stop_open_and_close_the_measurement_record_after_state_release() -> None:
    panel = _panel()
    observed_states: list[str] = []
    calls: list[tuple[str, str, dict[str, object]]] = []

    class _Record:
        file_prefix = "20260906_143005"
        counters = {"pixhawk_rtk_live": {"VALID": 0}}

        def add_alignment_row(self, row):
            calls.append(("alignment", "", dict(row)))

    class _Registry:
        def begin(self, measurement_id, **kwargs):
            observed_states.append(str(panel._measurement_run_state))
            calls.append(("begin", measurement_id, dict(kwargs)))
            return _Record()

        def stop(self, measurement_id, **kwargs):
            observed_states.append(str(panel._measurement_run_state))
            calls.append(("stop", measurement_id, dict(kwargs)))

    panel.measurement_registry = lambda: _Registry()
    panel._measurement_view_controller_owner = SimpleNamespace(
        measurement_calibration_snapshot=lambda: {"profile_sha256": "c" * 64},
        measurement_alignment_rows=lambda: [{"method_id": "checkerboard_stereo"}],
    )
    controller = MeasurementRunController(panel)
    assert controller.begin_measurement_run()
    generation = panel._measurement_run_generation
    started_utc_ns = panel._measurement_run_started_utc_ns
    assert panel._measurement_file_prefix == "20260906_143005"
    assert panel._measurement_method_result_counters is _Record.counters

    assert controller.request_measurement_stop()

    # Start reserves admission before optional registry callbacks; Stop releases
    # it before optional persistence. A reentrant Start cannot create two runs.
    assert observed_states == ["STARTING", "IDLE"]
    assert calls[0][0] == "begin" and calls[0][1] == generation
    assert calls[0][2]["started_utc_ns"] == started_utc_ns
    assert calls[0][2]["calibration"] == {"profile_sha256": "c" * 64}
    assert calls[1] == ("alignment", "", {"method_id": "checkerboard_stereo"})
    assert calls[2][0] == "stop" and calls[2][1] == generation
    assert calls[2][2]["stopped_utc_ns"] >= started_utc_ns
    assert panel._measurement_file_prefix == ""


def test_start_never_retires_previous_measurement_workers() -> None:
    panel = _panel()
    closed: list[str] = []
    panel._measurement_view_controller_owner = SimpleNamespace(
        close_method_evaluation_workers=lambda: closed.append("closed") or True,
    )
    controller = MeasurementRunController(panel)

    assert controller.begin_measurement_run()
    assert controller.request_measurement_stop()
    assert controller.begin_measurement_run()

    assert closed == []


def test_registry_callback_cannot_reenter_start_or_create_a_second_run() -> None:
    panel = _panel()
    controller = MeasurementRunController(panel)
    attempts = []

    def begin(*_args, **_kwargs):
        attempts.append(controller.begin_measurement_run())
        return SimpleNamespace(file_prefix="one", counters={}, add_alignment_row=lambda row: None)

    panel.measurement_registry = lambda: SimpleNamespace(begin=begin, stop=lambda *a, **k: None)
    assert controller.begin_measurement_run()
    assert attempts == [False]
    assert controller.state == "RUNNING"


def test_panel_projection_is_not_the_run_state_owner_and_adapters_share_service() -> None:
    panel = _panel()
    controller = MeasurementRunController(panel)
    assert controller.begin_measurement_run()
    generation = controller.measurement_run_snapshot()["generation"]
    panel._measurement_run_state = "IDLE"
    panel._measurement_run_generation = "foreign"
    assert controller.state == "RUNNING"
    assert controller.measurement_run_snapshot()["generation"] == generation
    assert not controller.begin_measurement_run()
    second_adapter = MeasurementRunController(panel)
    assert second_adapter.run is controller.run
    assert panel._measurement_run_generation == generation
    assert second_adapter.request_measurement_stop()
    assert controller.state == "IDLE"


def test_persistence_exceptions_never_gate_start_stop_or_next_run() -> None:
    panel = _panel()

    def fail_append(*_args: object, **_kwargs: object) -> bool:
        raise OSError("storage offline")

    def fail_flush() -> bool:
        raise OSError("flush offline")

    panel.append_workstation_data = fail_append
    panel.flush_workstation_data = fail_flush
    panel.measurement_registry = lambda: SimpleNamespace(
        begin=lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("workbook offline")),
        stop=lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("workbook offline")),
    )
    panel._measurement_view_controller_owner = SimpleNamespace(
        measurement_calibration_snapshot=lambda: (_ for _ in ()).throw(
            RuntimeError("calibration unavailable")
        )
    )
    panel._measurement_view_controller = lambda: (_ for _ in ()).throw(
        RuntimeError("method settings unavailable")
    )
    controller = MeasurementRunController(panel)

    assert controller.begin_measurement_run()
    first = panel._measurement_run_generation
    assert controller.state == "RUNNING"
    assert controller.request_measurement_stop()
    assert controller.state == "IDLE"
    assert panel._measurement_run_last_generation == first
    assert panel._measurement_run_generation == ""
    assert controller.begin_measurement_run()
    assert controller.state == "RUNNING"
    assert panel._measurement_run_generation != first


def test_real_panel_ignores_old_session_and_prepares_type_owned_paths(
    monkeypatch,
    tmp_path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_initial_status",
        lambda self: None,
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda self: None,
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel._measurement_session_state = "RECOVERABLE"
    panel._measurement_session_id = "20260823_194605_395747300"
    panel.recording_session_id = "20260823_194605_395747300"
    starts = []
    monkeypatch.setattr(
        panel,
        "_start_camera_streams",
        lambda **kwargs: starts.append(kwargs),
    )

    assert panel.start_yolo() is None
    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    assert starts == [{"enable_detection": True, "switch_to_depth": True}]
    assert panel.data_storage_snapshot()["root"] == str(tmp_path / "recordings")
    # Every Measurement owns its start-stamped directory and media paths.
    prefix = panel._measurement_file_prefix
    assert prefix
    assert {path.name for path in panel.video_recording_paths.values()} == {
        f"{prefix}_left.mp4",
        f"{prefix}_right.mp4",
    }
    assert all(
        path.parent == tmp_path / "recordings" / prefix / "media"
        for path in panel.video_recording_paths.values()
    )
    assert panel.measurement_workbook_snapshot()["current_measurement_id"] == panel._measurement_id
    assert not (tmp_path / "recordings" / "session.json").exists()

    assert panel.close_workstation_data(timeout_s=5.0)
    # Shutdown finalises the running Measurement into its own workbook.
    workbooks = sorted((tmp_path / "recordings").glob("*/*_measurement.xlsx"))
    assert [path.name for path in workbooks] == [f"{prefix}_measurement.xlsx"]
    from Workstation.workstation_core.measurement_xlsx_read import read_workbook, table_rows

    sheets = read_workbook(workbooks[0])
    assert "10_Pixhawk_RTK" in sheets and "99_Definitions" in sheets
    events = {row["event_type"] for row in table_rows(sheets["50_Events"])}
    assert {"MEASUREMENT_STARTED", "APPLICATION_SHUTDOWN"} <= events
    panel._shutdown_coordinator = SimpleNamespace(run=lambda: True)
    panel.close()
    panel.deleteLater()
    app.processEvents()
