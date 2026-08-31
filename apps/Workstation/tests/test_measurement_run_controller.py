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
        request_latest_measurement_xlsx=lambda *args, **kwargs: exports.append(
            (args, dict(kwargs))
        )
        or True,
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


def test_stop_requests_latest_xlsx_after_releasing_measurement_state() -> None:
    panel = _panel()
    observed_states: list[str] = []
    requests: list[tuple[tuple[object, ...], dict[str, object]]] = []

    def request(*args: object, **kwargs: object) -> bool:
        observed_states.append(str(panel._measurement_run_state))
        requests.append((args, dict(kwargs)))
        return True

    panel.request_latest_measurement_xlsx = request
    controller = MeasurementRunController(panel)
    assert controller.begin_measurement_run()
    generation = panel._measurement_run_generation
    started_utc_ns = panel._measurement_run_started_utc_ns
    assert controller.measurement_source_transition(
        "camera.left", "LIVE", "fixture source"
    )

    assert controller.request_measurement_stop()

    assert observed_states == ["IDLE"]
    assert requests[0][0] == (generation,)
    assert requests[0][1]["started_utc_ns"] == started_utc_ns
    assert requests[0][1]["stopped_utc_ns"] >= started_utc_ns
    assert requests[0][1]["source_states"]["camera.left"]["state"] == "LIVE"
    assert requests[0][1]["automatic"] is True


def test_persistence_exceptions_never_gate_start_stop_or_next_run() -> None:
    panel = _panel()

    def fail_append(*_args: object, **_kwargs: object) -> bool:
        raise OSError("storage offline")

    def fail_flush() -> bool:
        raise OSError("flush offline")

    panel.append_workstation_data = fail_append
    panel.flush_workstation_data = fail_flush
    panel.request_latest_measurement_xlsx = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        OSError("xlsx offline")
    )
    panel._measurement_view_controller_owner = SimpleNamespace(
        close_method_evaluation_workers=lambda: (_ for _ in ()).throw(
            RuntimeError("old worker teardown failed")
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
    assert all(
        path.parent == tmp_path / "recordings" / "videos"
        for path in panel.video_recording_paths.values()
    )
    assert not (tmp_path / "recordings" / "session.json").exists()

    assert panel.close_workstation_data(timeout_s=2.0)
    panel._shutdown_coordinator = SimpleNamespace(run=lambda: True)
    panel.close()
    panel.deleteLater()
    app.processEvents()
