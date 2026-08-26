from __future__ import annotations

from types import SimpleNamespace

from PyQt5 import QtWidgets

from Workstation.measurement_run_controller import MeasurementRunController


def _panel(*, storage_accepts: bool = True) -> SimpleNamespace:
    events: list[tuple[str, dict[str, object]]] = []

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
    )


def test_old_recoverable_session_and_storage_failure_do_not_block_measurement() -> None:
    panel = _panel(storage_accepts=False)
    controller = MeasurementRunController(panel)

    assert controller.begin_measurement_run() is True
    assert controller.state == "STARTING"
    assert panel._measurement_id.startswith("run-")
    assert controller.confirm_measurement_running(("left", "right")) is True
    assert controller.state == "RUNNING"
    assert controller.request_measurement_stop() is True
    assert controller.state == "STOPPING"
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
    assert controller.finish_measurement_run()
    assert controller.begin_measurement_run()
    second = panel._measurement_run_generation
    assert first != second
    assert panel._measurement_session_state == "RECOVERABLE"
    assert [payload["event_type"] for _stream, payload in panel._events] == [
        "measurement_start_requested",
        "measurement_stopped",
        "measurement_start_requested",
    ]


def test_busy_start_is_queued_and_duplicate_live_start_is_rejected() -> None:
    panel = _panel()
    controller = MeasurementRunController(panel)
    assert controller.queue_measurement_start("camera time sync")
    assert controller.state == "QUEUED"
    assert controller.begin_measurement_run()
    assert controller.state == "STARTING"
    assert controller.begin_measurement_run() is False


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
    assert panel.measurement_run_snapshot()["state"] == "STARTING"
    assert starts == [{"enable_detection": True, "switch_to_depth": True}]
    assert panel.data_storage_snapshot()["root"] == str(tmp_path / "measurement_data")
    assert all(path.parent == tmp_path / "measurement_data" / "videos" for path in panel.video_recording_paths.values())
    assert not (tmp_path / "measurement_data" / "session.json").exists()

    assert panel.close_workstation_data(timeout_s=2.0)
    panel._shutdown_coordinator = SimpleNamespace(run=lambda: True)
    panel.close()
    panel.deleteLater()
    app.processEvents()
