"""Workstation camera regression tests."""

from __future__ import annotations

from Workstation.tests.workstation_test_support import (
    _FakeBackend,
    json,
    LifecycleCoordinator,
    Path,
    pytest,
    QtCore,
    QtWidgets,
    SimpleNamespace,
    StandalonePanelBackend,
    threading,
    time,
    WorkstationMainWindow,
)


def test_visible_encoder_profile_settings_round_trip_and_clamp(tmp_path: Path) -> None:
    from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings

    path = tmp_path / "settings.json"
    save_settings(
        AppSettings(
            wiris_visible_encoder_bitrate=5,
            wiris_visible_encoder_qp=17,
            wiris_visible_encoder_gop=30,
            wiris_visible_encoder_idr=1,
            rtsp_reconnect_policy="automatic",
        ),
        path,
    )
    loaded = load_settings(path)
    assert (
        loaded.wiris_visible_encoder_bitrate,
        loaded.wiris_visible_encoder_qp,
        loaded.wiris_visible_encoder_gop,
        loaded.wiris_visible_encoder_idr,
    ) == (5, 17, 30, 1)
    assert loaded.rtsp_reconnect_policy == "automatic"
    payload = json.loads(path.read_text())
    payload.update(
        wiris_visible_encoder_bitrate=-1,
        wiris_visible_encoder_qp=99,
        wiris_visible_encoder_gop=99999,
        wiris_visible_encoder_idr=-2,
    )
    path.write_text(json.dumps(payload))
    clamped = load_settings(path)
    assert (
        clamped.wiris_visible_encoder_bitrate,
        clamped.wiris_visible_encoder_qp,
        clamped.wiris_visible_encoder_gop,
        clamped.wiris_visible_encoder_idr,
    ) == (0, 51, 32767, 0)


def test_encoded_queue_mode_settings_round_trip_and_invalid_value_defaults_to_latest(
    tmp_path: Path,
) -> None:
    from Workstation.workstation_core.settings import (
        AppSettings,
        load_settings,
        save_settings,
    )

    path = tmp_path / "settings.json"
    save_settings(AppSettings(rtsp_encoded_queue_mode="reference_safe"), path)
    assert load_settings(path).rtsp_encoded_queue_mode == "reference_safe"

    payload = json.loads(path.read_text())
    payload["rtsp_encoded_queue_mode"] = "invalid-mode"
    path.write_text(json.dumps(payload))
    assert load_settings(path).rtsp_encoded_queue_mode == "latest"


def test_default_visible_encoder_profile_matches_field_validated_wiris_profile() -> None:
    from Workstation.workstation_core.settings import AppSettings

    settings = AppSettings()
    assert (
        settings.wiris_visible_encoder_bitrate,
        settings.wiris_visible_encoder_qp,
        settings.wiris_visible_encoder_gop,
        settings.wiris_visible_encoder_idr,
    ) == (0, 1, 90, 0)

def test_workstation_camera_sync_readiness_uses_imported_finite_check() -> None:
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    now_monotonic_s = time.monotonic()
    def observations(side: str) -> list[dict[str, object]]:
        ssrc = 101 if side == "left" else 202
        return [
            {
                "observer_generation": 4,
                "ssrc": ssrc,
                "ntp_raw": index,
                "received_monotonic_s": now_monotonic_s - (3 - index) * 0.1,
                "arrival_basis": "rtcp-pad-probe",
                "sr_pc_offset_s": 0.020 if side == "left" else 0.025,
                "ntp_rate": 1.0001,
                "media_rate": 0.9999,
            }
            for index in (1, 2, 3)
        ]

    panel = SimpleNamespace(
        _rtcp_clock_observer_generation=4,
        _rtcp_clock_observation_history={
            "left": observations("left"),
            "right": observations("right"),
        },
        _camera_sync_last_sr_counts={},
        _camera_name_ko=lambda camera: camera,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_sync_receive_pair_max_delta_s=lambda: 0.35,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
    )

    ready, reason = CameraTimeSyncController(panel)._camera_sync_readiness()

    assert ready is True
    assert "검증 완료" in reason

def test_stus_restarts_only_rtcp_observer_and_preserves_video_workers() -> None:
    from Workstation.camera_time_sync_controller import (
        CAMERA_SYNC_STATUS_RECONNECTING,
        CameraTimeSyncController,
    )

    class Timer:
        def __init__(self) -> None:
            self.started = 0

        def start(self) -> None:
            self.started += 1

    states: list[tuple[str, str]] = []
    observer_restarts: list[str] = []
    timer = Timer()
    panel = SimpleNamespace(
        _camera_sync_stus_sr_baseline={},
        _camera_sync_deadline_monotonic_s=None,
        _camera_sync_transport_override="tcp",
        _camera_sync_timer=timer,
        _camera_sync_warmup_timeout_s=lambda: 30.0,
        _camera_sync_stus_operation={
            "active": True,
            "operation_id": "stus-test",
            "attempts_started": 1,
        },
        _restart_rtcp_clock_observer=lambda *, operation_id: observer_restarts.append(operation_id) or True,
        _set_camera_sync_state=lambda status, reason: states.append((status, reason)),
    )

    CameraTimeSyncController(panel)._begin_camera_sync_after_stus(
        target="both",
        result="success",
    )

    assert observer_restarts == ["stus-test"]
    assert panel._camera_sync_stus_operation["phase"] == "observer-restarting-attempt-1"
    assert panel._camera_sync_stus_sr_baseline == {"left": 0, "right": 0}
    assert panel._camera_sync_transport_override is None
    assert timer.started == 1
    assert states[-1][0] == CAMERA_SYNC_STATUS_RECONNECTING
    assert "영상/YOLO/MP4는 유지" in states[-1][1]

def test_workstation_launcher_bypasses_stereowiris_runtime() -> None:
    from Workstation import standalone_gui

    repository = Path(__file__).resolve().parents[3]
    launcher = (repository / "apps" / "Workstation" / "run.sh").read_text(encoding="utf-8")
    entrypoint = (repository / "apps" / "Workstation" / "__main__.py").read_text(encoding="utf-8")
    runtime_requirements = (repository / "requirements" / "runtime.txt").read_text(encoding="utf-8")

    assert "StereoWIRIS" not in launcher
    assert "Simulation" not in launcher
    assert "-m Workstation" in launcher
    assert "--start" not in launcher
    assert "--ui" not in launcher
    assert "WorkstationMainWindow" in entrypoint
    assert "--no-start" in entrypoint
    assert "legacy" not in entrypoint.lower()
    assert "PyQt5" in runtime_requirements
    assert "PyQt6" not in runtime_requirements
    assert standalone_gui._parse_ui_mode(["workstation"])[0] == "workstation"
    assert standalone_gui._parse_ui_mode(["workstation", "--ui", "workstation"])[1] == ["workstation"]
    with pytest.raises(SystemExit):
        standalone_gui._parse_ui_mode(["workstation", "--ui", "legacy"])
    assert not hasattr(standalone_gui, "WirisStereoDistanceGui")
    assert not hasattr(standalone_gui, "IntegratedWirisPixhawkConsole")

def test_workstation_entrypoint_keeps_cameras_stopped_by_default_and_supports_explicit_start() -> None:
    from Workstation.__main__ import _parser

    assert _parser().parse_args([]).start is False
    assert _parser().parse_args(["--no-start"]).start is False
    assert _parser().parse_args(["--start"]).start is True

def test_workstation_camera_defaults_prefer_reliable_receive_profile() -> None:
    from Workstation.workstation_core.settings import AppSettings

    settings = AppSettings()

    assert settings.preview_width == 1280
    assert settings.rtsp_latency_ms == 100
    assert settings.rtsp_decoder == "auto"
    assert settings.stream_backend == "gstreamer"
    assert settings.rtsp_transport == "tcp"

def test_class_gate_settings_are_forwarded_to_the_yolo_worker(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.yolo_gate_class_checkbox.setChecked(True)
    panel.yolo_gate_class_edit.setText("person")

    kwargs = panel.yolo_kwargs()

    assert kwargs["class_gate_enabled"] is True
    assert kwargs["class_gate"] == "person"
    assert "rtsp_pipeline_profile" not in kwargs
    assert kwargs["rtsp_reconnect_policy"] == "manual"

    panel.apply_workstation_system_settings(
        {
            "rtsp_reconnect_policy": "automatic",
            "rtsp_encoded_queue_mode": "reference_safe",
        }
    )
    overridden = panel.yolo_kwargs()
    assert "rtsp_pipeline_profile" not in overridden
    assert overridden["rtsp_reconnect_policy"] == "automatic"
    assert overridden["rtsp_encoded_queue_mode"] == "reference_safe"
    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_gstreamer_rtsp_worker_retries_transient_sdp_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    from Workstation.workstation_core import yolo_stream

    attempts: list[int] = []
    messages: list[object] = []

    class OutputQueue:
        def put(self, message: object) -> None:
            messages.append(message)

    def fake_run_once(*_args: object) -> bool:
        attempts.append(len(attempts) + 1)
        return len(attempts) >= 2

    monkeypatch.setattr(yolo_stream, "_run_gstreamer_stream_once", fake_run_once)
    monkeypatch.setattr(yolo_stream, "GST_RTSP_RECONNECT_DELAYS_S", (0.001,))

    yolo_stream._run_gstreamer_stream(
        {},
        OutputQueue(),
        threading.Event(),
        None,
        None,
        False,
    )

    assert attempts == [1, 2]
    assert any("RTSP 재연결 대기" in str(message[1]) for message in messages if isinstance(message, tuple))

@pytest.mark.parametrize("full_control_scan", (True, False))
def test_camera_auto_detect_result_records_full_control_scan_mode(
    monkeypatch: pytest.MonkeyPatch,
    full_control_scan: bool,
) -> None:
    from Workstation import standalone_gui

    emitted: list[dict[str, object]] = []
    monkeypatch.setattr(standalone_gui, "discover_host_interface_candidates", lambda: [])
    worker = standalone_gui.CameraAutoDetectWorker(
        mode="auto_connect",
        full_control_scan=full_control_scan,
    )
    worker._discover_wiris_control = lambda _reports: ([], [])
    worker.detected.connect(emitted.append)

    worker.run()

    assert emitted and emitted[0]["full_control_scan"] is full_control_scan

def test_auto_start_uses_wiris_activation_sequence_before_stream_start(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    initial_status = standalone_gui.WirisStereoDistancePanel._initial_status
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.auto_start = True
    monkeypatch.setattr(panel, "_start_interface_status_worker", lambda: None)
    monkeypatch.setattr(panel, "_recover_retryable_recording_session", lambda: None)
    startup_calls: list[str] = []
    monkeypatch.setattr(panel, "_start_startup_camera_sequence", lambda: startup_calls.append("auto-connect"))

    initial_status(panel)

    assert startup_calls == ["auto-connect"]
    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_startup_camera_sequence_uses_configured_hosts_before_full_discovery(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    calls: list[dict[str, object]] = []
    monkeypatch.setattr(panel, "_session_recording_active", lambda: False)
    monkeypatch.setattr(
        panel,
        "_start_rtsp_enable_worker",
        lambda **kwargs: calls.append(dict(kwargs)),
    )
    monkeypatch.setattr(panel, "_start_camera_detection", lambda *_args, **_kwargs: pytest.fail("full discovery ran"))

    panel._start_startup_camera_sequence()

    assert calls == [{"target_labels": ("left", "right"), "reactivate": False}]
    assert panel.auto_wiris_connect_pending is True
    assert panel._startup_camera_sequence_pending is True
    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_startup_camera_failure_does_not_escalate_to_full_network_scan(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.auto_wiris_connect_pending = True
    panel._startup_camera_sequence_pending = True
    monkeypatch.setattr(panel, "_start_camera_detection", lambda *_args, **_kwargs: pytest.fail("full discovery ran"))

    panel._schedule_auto_wiris_connect_retry("configured host timeout")

    assert panel.auto_wiris_connect_pending is False
    assert panel._startup_camera_sequence_pending is False
    assert "connect cameras" in panel.camera_connection_label.text().lower()
    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_manual_camera_connect_does_not_retry_the_same_endpoint_after_failure(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.auto_wiris_connect_pending = True
    panel._startup_camera_sequence_pending = False
    panel._camera_auto_connect_attempt = 1
    scheduled: list[tuple[object, ...]] = []
    monkeypatch.setattr(
        standalone_gui.QtCore.QTimer,
        "singleShot",
        lambda *args: scheduled.append(args),
    )

    panel._schedule_auto_wiris_connect_retry("configured WIRIS IP unavailable")

    assert panel.auto_wiris_connect_pending is False
    assert panel._startup_camera_sequence_pending is False
    assert scheduled == []
    assert "camera service unavailable" in panel.camera_connection_label.text().lower()
    panel.close()
    panel.deleteLater()
    app.processEvents()


@pytest.mark.parametrize(
    ("startup_sequence", "expected_call"),
    ((True, "start_yolo"), (False, "connect_cameras")),
)
def test_auto_wiris_completion_preserves_manual_preview_mode(
    startup_sequence: bool,
    expected_call: str,
) -> None:
    from Workstation import standalone_gui

    calls: list[str] = []
    panel = SimpleNamespace(
        _startup_camera_sequence_pending=startup_sequence,
        _cancel_auto_wiris_connect_retry=lambda: calls.append("cancel"),
        _log=lambda _message: None,
        start_yolo=lambda: calls.append("start_yolo"),
        connect_cameras=lambda: calls.append("connect_cameras"),
    )

    standalone_gui.WirisStereoDistancePanel._finish_auto_wiris_connect(panel)

    assert calls == ["cancel", expected_call]

def test_manual_record_stop_does_not_delay_measurement_observation_start(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.distance_recording_enabled = True
    panel._manual_recording_active = True
    panel.camera_connected = True
    starts: list[tuple[bool, bool]] = []
    stops: list[bool] = []
    monkeypatch.setattr(
        panel,
        "_start_camera_streams",
        lambda *, enable_detection, switch_to_depth: starts.append((enable_detection, switch_to_depth)),
    )
    monkeypatch.setattr(
        panel,
        "_stop_session_recording",
        lambda *, restart_streams=False: stops.append(restart_streams),
    )

    assert panel.toggle_yolo_detection() is None
    assert starts == []
    assert stops == [False]
    snapshot = panel.measurement_run_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert snapshot["sources"]["camera.left"]["state"] == "WAITING_RESOURCE"

    panel._manual_recording_active = False
    panel.distance_recording_enabled = False
    assert panel._resume_queued_measurement_start() is True
    app.processEvents()
    assert starts == [(True, True)]

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_measurement_decoder_preflight_rejection_preserves_live_preview() -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    class Button:
        def __init__(self) -> None:
            self.enabled: list[bool] = []

        def setEnabled(self, enabled: bool) -> None:  # noqa: N802 - Qt-compatible fake.
            self.enabled.append(bool(enabled))

    class Label:
        def __init__(self) -> None:
            self.text = "camera LIVE: left/right decoded current frame verified"

        def setText(self, text: str) -> None:  # noqa: N802 - Qt-compatible fake.
            self.text = str(text)

    events: list[object] = []
    left_worker = object()
    right_worker = object()
    panel = SimpleNamespace(
        _shutdown_in_progress=False,
        _manual_recording_active=False,
        _automatic_session_recording=False,
        camera_connected=True,
        yolo_enabled=False,
        left_pane=SimpleNamespace(worker=left_worker),
        right_pane=SimpleNamespace(worker=right_worker),
        start_button=Button(),
        camera_connect_button=Button(),
        camera_connection_label=Label(),
        _session_recording_active=lambda: False,
        _start_camera_streams=lambda **kwargs: events.append(
            ("camera.start.rejected", dict(kwargs))
        )
        or False,
        _stop_camera_panes=lambda **kwargs: events.append(
            ("camera.stop", dict(kwargs))
        )
        or True,
        fail_measurement_start=lambda reason: events.append(
            ("measurement.camera.unavailable", str(reason))
        )
        or True,
        _log=lambda message: events.append(("log", str(message))),
        _refresh_session_toggle_buttons=lambda: None,
        _refresh_operation_header=lambda: None,
    )

    def start_recording() -> bool:
        panel._automatic_session_recording = True
        events.append(("recording.start",))
        return True

    def stop_recording(*, restart_streams: bool = False) -> bool:
        panel._automatic_session_recording = False
        events.append(("recording.stop", bool(restart_streams)))
        return True

    panel.start_automatic_session_recording = start_recording
    panel.stop_automatic_session_recording = stop_recording
    controller = CameraStreamLifecycleController(panel)
    controller._reserve_package_detection = lambda: True  # type: ignore[method-assign]
    controller._detection_inputs_valid = lambda: True  # type: ignore[method-assign]
    controller._camera_busy_reason = lambda: ""  # type: ignore[method-assign]

    assert controller.start_yolo() is None

    assert not any(event[0] == "camera.stop" for event in events)
    assert panel.camera_connected is True
    assert panel.yolo_enabled is False
    assert panel.left_pane.worker is left_worker
    assert panel.right_pane.worker is right_worker
    assert ("recording.start",) in events
    assert ("recording.stop", False) in events
    assert any(event[0] == "measurement.camera.unavailable" for event in events)
    assert "Preview 유지" in panel.camera_connection_label.text


def test_measurement_start_is_blocked_when_live_pixhawk_mag_is_unverified() -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    events: list[str] = []
    panel = SimpleNamespace(
        _shutdown_in_progress=False,
        measurement_mag_start_gate=lambda: False,
        _log=lambda message: events.append(str(message)),
    )

    assert CameraStreamLifecycleController(panel).start_yolo() is False
    assert any("MAG active-selection verification" in message for message in events)


def test_camera_stream_preflight_rejection_does_not_cancel_current_owner() -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    events: list[str] = []
    controller = CameraStreamLifecycleController(SimpleNamespace())
    controller._apply_decoder_start_policy = (  # type: ignore[method-assign]
        lambda: events.append("decoder.preflight") or False
    )
    controller._cancel_camera_stop_watchdog = (  # type: ignore[method-assign]
        lambda: events.append("stop-watchdog.cancel")
    )
    controller._cancel_camera_start_readiness = (  # type: ignore[method-assign]
        lambda: events.append("start-readiness.cancel")
    )
    controller._cancel_video_start_readiness = (  # type: ignore[method-assign]
        lambda **_kwargs: events.append("video-readiness.cancel")
    )

    assert controller._start_camera_streams(
        enable_detection=True,
        switch_to_depth=True,
    ) is False
    assert events == ["decoder.preflight"]


@pytest.mark.parametrize(
    "busy_attribute",
    (
        "rtsp_enable_thread",
        "camera_auto_detect_thread",
        "calibration_wiris_oz_capture_thread",
        "optical_pose_wiris_ftp_thread",
    ),
)
def test_measurement_start_queues_while_camera_resource_is_busy(
    monkeypatch: pytest.MonkeyPatch,
    busy_attribute: str,
) -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    queued: list[str] = []
    timers: list[object] = []
    panel = SimpleNamespace(
        _manual_recording_active=False,
        queue_measurement_start=lambda reason: queued.append(str(reason)),
    )
    setattr(panel, busy_attribute, object())
    monkeypatch.setattr(
        QtCore.QTimer,
        "singleShot",
        lambda _delay, callback: timers.append(callback),
    )

    assert CameraStreamLifecycleController(panel).start_yolo() is None
    assert queued
    assert panel._pending_measurement_run_start is True
    assert len(timers) == 1


def test_measurement_start_does_not_treat_stus_observer_as_camera_resource_busy() -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    panel = SimpleNamespace(
        _camera_sync_stus_operation={
            "active": True,
            "attempts_started": 1,
            "max_attempts": 2,
        },
        _rtsp_reactivation_side=None,
        auto_wiris_connect_pending=False,
    )

    reason = CameraStreamLifecycleController(panel)._camera_busy_reason()

    assert reason == ""


def test_queued_stus_measurement_retry_cannot_restart_during_shutdown(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    callbacks: list[object] = []
    starts: list[str] = []
    panel = SimpleNamespace(
        _pending_measurement_run_start=False,
        _shutdown_in_progress=False,
        _manual_recording_active=False,
        _camera_sync_stus_operation={
            "active": True,
            "attempts_started": 1,
            "max_attempts": 2,
        },
        _rtsp_reactivation_side=None,
        auto_wiris_connect_pending=False,
        queue_measurement_start=lambda _reason: True,
        start_yolo=lambda: starts.append("started"),
    )
    monkeypatch.setattr(
        QtCore.QTimer,
        "singleShot",
        lambda _delay, callback: callbacks.append(callback),
    )
    controller = CameraStreamLifecycleController(panel)

    controller._queue_measurement_start("STUS closed-loop")
    panel._shutdown_in_progress = True
    panel._camera_sync_stus_operation["active"] = False
    callbacks.pop(0)()

    assert panel._pending_measurement_run_start is False
    assert starts == []


def test_recording_owner_rejects_camera_swap_but_allows_no_restart_clock_sync(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.distance_recording_enabled = True
    panel._restart_rtcp_clock_observer = lambda **_kwargs: True
    left_url = panel.left_url_edit.text()
    right_url = panel.right_url_edit.text()

    assert panel.swap_cameras() is False
    assert panel.left_url_edit.text() == left_url
    assert panel.right_url_edit.text() == right_url
    assert panel.set_selected_wiris_time_from_pc() is None
    assert panel._camera_sync_stus_operation["phase"] == "observer-preflight"
    panel._finish_wiris_time_sync_operation("cancelled", "test cleanup")
    assert panel.wiris_time_sync_thread is None
    assert panel.swap_pixhawks() is False
    assert panel.toggle_mavlink_connection() is False
    assert panel.auto_connect_wiris() is False
    assert panel.auto_wiris_connect_pending is False
    panel._camera_candidate_rows = [{"kind": "RTSP", "address": "rtsp://candidate", "host": "10.0.0.50"}]
    assert panel.apply_camera_candidate(0, "left") is False
    panel.distance_recording_enabled = False
    panel.auto_wiris_connect_pending = True
    original_left_url = panel.left_url_edit.text()
    retry_reasons: list[str] = []
    monkeypatch.setattr(panel, "_schedule_auto_wiris_connect_retry", retry_reasons.append)
    panel._apply_camera_detection({"mode": "auto_connect", "rtsp": [], "control": [], "verified_control": []})
    assert panel.auto_wiris_connect_pending is True
    assert retry_reasons == ["WIRIS 제어 포트가 아직 열리지 않음"]
    assert panel.left_url_edit.text() == original_left_url
    panel.distance_recording_enabled = True
    panel._pixhawk_candidate_rows_by_source["mavlink"] = [
        {"kind": "MAVLink", "value": "udpin:0.0.0.0:14550"}
    ]
    assert panel.apply_pixhawk_candidate("mavlink", 0, "left") is False

    starts: list[tuple[bool, bool]] = []
    monkeypatch.setattr(
        panel,
        "_start_camera_streams",
        lambda *, enable_detection, switch_to_depth: starts.append((enable_detection, switch_to_depth)),
    )
    panel.camera_connected = True
    panel._begin_camera_sync_after_stus(target="both", result="success")
    assert starts == []
    assert panel.camera_sync_status == standalone_gui.CAMERA_SYNC_STATUS_RECONNECTING
    panel.camera_connected = False

    panel.distance_recording_enabled = False

    panel._camera_sync_stus_operation = {
        "active": True,
        "attempts_started": 1,
        "max_attempts": 2,
    }
    assert panel._start_session_recording(start_camera_streams=False) is not False
    assert panel.distance_recording_enabled is True
    panel._stop_session_recording()
    panel._camera_sync_stus_operation = None

    panel.pixhawk_connection_thread = object()
    assert panel._start_session_recording() is False
    assert panel.recording_session_id == ""
    panel.pixhawk_connection_thread = None

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_camera_workspace_uses_hidden_diagnostics_height_without_cropping_preview_contract() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.resize(1600, 980)
    window.diagnostics_dock.show()
    window.show()
    window.navigation.select_context("devices.cameras")
    app.processEvents()
    app.processEvents()

    shown_height = window.camera.preview_splitter.height()
    assert window.camera.workspace_splitter.orientation() == QtCore.Qt.Vertical
    assert window.camera.preview_splitter.orientation() == QtCore.Qt.Horizontal

    window.diagnostics_dock.hide()
    app.processEvents()
    app.processEvents()

    assert window.camera.workspace_splitter.orientation() == QtCore.Qt.Horizontal
    assert window.camera.preview_splitter.orientation() == QtCore.Qt.Vertical
    assert window.camera.preview_splitter.height() > shown_height
    image = window.camera.previews["left"].image
    assert image.width() > 0
    assert image.height() > 0
    assert image.width() / image.height() == pytest.approx(16.0 / 9.0, abs=0.05)

    window.diagnostics_dock.show()
    app.processEvents()
    app.processEvents()
    assert window.camera.workspace_splitter.orientation() == QtCore.Qt.Vertical
    assert window.camera.preview_splitter.orientation() == QtCore.Qt.Horizontal

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_devices_camera_status_shows_active_decoder_per_side_only() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.active_stream_decoders = {"left": "nvh264dec", "right": "nvh264dec"}
    backend.panel.decoder_fallback_reasons = {"left": "", "right": ""}
    backend.panel.decoder_combo = SimpleNamespace(currentData=lambda: "auto")
    backend.panel._runtime_decoder_override = None
    backend.panel.decoder_selection_status = {
        "requested": "auto",
        "effective": "nvh264dec",
        "candidates": ("nvh264dec", "avdec_h264"),
        "allowed": True,
        "degraded": False,
        "reason": "",
        "nvidia_available": True,
    }
    backend.panel.nvidia_runtime_identity = {
        "model": "NVIDIA GeForce RTX 4050 Laptop GPU",
        "driver_version": "595.84",
        "gpu_uuid": "GPU-sentinel",
        "bus_location": "0000:01:00.0",
    }
    backend.panel.camera_stream_stats = {
        "left": {
            "rtp_packets_hz": 3722.4,
            "rtp_missing_hz": 3300.1,
            "rtp_loss_pct": 47.0,
            "h264_au_hz": 18.5,
            "decoded_hz": 2.1,
            "rtp_metrics_window_s": 5.0,
            "rtp_packets_received_total": 12_345,
            "rtp_missing_packets_total": 6_789,
        },
        "right": {
            "rtp_packets_hz": 4500.0,
            "rtp_missing_hz": 500.0,
            "rtp_loss_pct": 10.0,
            "h264_au_hz": 19.0,
            "decoded_hz": 7.4,
            "rtp_metrics_window_s": 5.0,
            "rtp_packets_received_total": 23_456,
            "rtp_missing_packets_total": 1_234,
        },
    }
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.show()
    window.navigation.select_context("devices.cameras")
    window.camera.refresh(window.status_adapter.model.snapshot())
    app.processEvents()

    assert window.camera.decoder_values["left"].value.text() == "GPU · nvh264dec"
    assert window.camera.decoder_values["right"].value.text() == "GPU · nvh264dec"
    assert "RTP 3722.4 pkt/s" in window.camera.stream_metric_labels["left"].text()
    assert "missing 3300.1 pkt/s" in window.camera.stream_metric_labels["left"].text()
    assert "loss 47.0%" in window.camera.stream_metric_labels["left"].text()
    assert "H.264 AU 18.5 /s" in window.camera.stream_metric_labels["left"].text()
    assert "decoded 2.1 Hz" in window.camera.stream_metric_labels["left"].text()
    preview_button = window.camera.operation_buttons["camera.toggle"]
    preview_button.setFocus()
    app.processEvents()
    gpu_geometry = (
        window.camera.decoder_values["left"].geometry(),
        window.camera.decoder_values["right"].geometry(),
        window.camera.stream_metric_labels["left"].geometry(),
        window.camera.stream_metric_labels["right"].geometry(),
        tuple(window.camera.workspace_splitter.sizes()),
    )

    backend.panel.camera_stream_stats["left"].update(
        {
            "rtp_packets_hz": 100.0,
            "rtp_missing_hz": 0.0,
            "rtp_loss_pct": 0.0,
            "h264_au_hz": 20.0,
            "decoded_hz": 19.8,
        }
    )
    window.camera.refresh(window.status_adapter.model.snapshot())
    assert "RTP 100.0 pkt/s" in window.camera.stream_metric_labels["left"].text()
    assert "decoded 19.8 Hz" in window.camera.stream_metric_labels["left"].text()
    app.processEvents()
    updated_metrics_geometry = (
        window.camera.decoder_values["left"].geometry(),
        window.camera.decoder_values["right"].geometry(),
        window.camera.stream_metric_labels["left"].geometry(),
        window.camera.stream_metric_labels["right"].geometry(),
        tuple(window.camera.workspace_splitter.sizes()),
    )
    assert updated_metrics_geometry == gpu_geometry

    backend.panel.decoder_combo = SimpleNamespace(currentData=lambda: "software")
    backend.panel.active_stream_decoders = {"left": "avdec_h264", "right": "avdec_h264"}
    window.camera.refresh(window.status_adapter.model.snapshot())
    assert window.camera.decoder_values["left"].value.text() == "CPU · avdec_h264 (selected)"
    assert window.camera.decoder_values["right"].value.text() == "CPU · avdec_h264 (selected)"

    backend.panel.decoder_combo = SimpleNamespace(currentData=lambda: "auto")
    backend.panel._runtime_decoder_override = "software"
    backend.panel.decoder_fallback_reasons = {
        "left": "왼쪽: first-frame timeout",
        "right": "왼쪽: first-frame timeout",
    }
    window.camera.refresh(window.status_adapter.model.snapshot())
    assert window.camera.decoder_values["left"].value.text() == "CPU fallback · avdec_h264"
    assert "first-frame timeout" in window.camera.decoder_values["left"].toolTip()
    app.processEvents()
    fallback_geometry = (
        window.camera.decoder_values["left"].geometry(),
        window.camera.decoder_values["right"].geometry(),
        window.camera.stream_metric_labels["left"].geometry(),
        window.camera.stream_metric_labels["right"].geometry(),
        tuple(window.camera.workspace_splitter.sizes()),
    )
    assert fallback_geometry == gpu_geometry
    assert QtWidgets.QApplication.focusWidget() is preview_button

    window.diagnostics._refresh_performance(window.status_adapter.model.snapshot())
    performance_text = window.diagnostics.performance_view.toPlainText()
    assert "NVIDIA GeForce RTX 4050 Laptop GPU" in performance_text
    assert "Driver: 595.84" in performance_text
    assert "NVDEC available: True" in performance_text
    assert "CPU fallback · avdec_h264" in performance_text

    backend.panel.active_stream_decoders = {"left": None, "right": None}
    window.camera.refresh(window.status_adapter.model.snapshot())
    assert window.camera.decoder_values["left"].value.text() == "--"
    assert window.camera.decoder_values["right"].value.text() == "--"

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_camera_status_tooltip_warns_on_auto_decoder_fallback() -> None:
    from Workstation.status_model import BackendStatusAdapter, StatusSeverity

    panel = SimpleNamespace(
        left_pane=SimpleNamespace(current_status="playing"),
        camera_connected=True,
        camera_frame_times={"left": [99.9]},
        decoder_combo=SimpleNamespace(currentData=lambda: "auto"),
        active_stream_decoders={"left": "avdec_h264"},
        decoder_fallback_reasons={"left": "nvh264dec first-frame timeout"},
        _runtime_decoder_override="software",
    )
    adapter = BackendStatusAdapter(SimpleNamespace(panel=panel))

    adapter._refresh_camera(panel, "left", "CAM L", now_s=100.0)

    status = adapter.model.record("camera.left")
    assert status is not None
    assert status.state == "LIVE"
    assert status.severity == StatusSeverity.WARN
    assert "decoder=CPU fallback · avdec_h264" in status.detail
    assert "first-frame timeout" in status.detail


def test_camera_status_keeps_non_idr_experiment_visible_and_non_green() -> None:
    from Workstation.status_model import BackendStatusAdapter, StatusSeverity

    panel = SimpleNamespace(
        left_pane=SimpleNamespace(current_status="playing"),
        camera_connected=True,
        camera_frame_times={"left": [99.9]},
        camera_frame_metadata={
            "left": {
                "media_recovery_basis": "wiris_non_idr_intra_experimental",
                "media_random_access_guaranteed": False,
            }
        },
        decoder_combo=SimpleNamespace(currentData=lambda: "auto"),
        active_stream_decoders={"left": "nvh264dec"},
        decoder_fallback_reasons={},
        _runtime_decoder_override=None,
    )
    adapter = BackendStatusAdapter(SimpleNamespace(panel=panel))

    adapter._refresh_camera(panel, "left", "CAM L", now_s=100.0)

    status = adapter.model.record("camera.left")
    assert status is not None
    assert status.state == "LIVE"
    assert status.severity == StatusSeverity.WARN
    assert "EXPERIMENTAL NON-IDR INTRA" in status.detail
    assert "IDR/keyframe guarantee=no" in status.detail

def test_wiris_control_probe_worker_keeps_socket_checks_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui

    calls: list[tuple[str, int, float]] = []

    def fake_check(host: str, port: int, *, timeout: float) -> bool:
        calls.append((host, port, timeout))
        return host == "10.0.0.11"

    monkeypatch.setattr(standalone_gui, "check_tcp_port", fake_check)
    results: list[object] = []
    worker = standalone_gui.WirisControlCheckWorker(
        (("left", "10.0.0.11", 5000), ("right", "", 5001)),
        timeout_s=2.5,
    )
    worker.checked.connect(results.append)
    worker.run()

    assert calls == [("10.0.0.11", 5000, 2.5)]
    assert results == [[
        {"label": "left", "host": "10.0.0.11", "port": 5000, "state": "control-open"},
        {"label": "right", "host": "", "port": 5001, "state": "missing-host"},
    ]]

def test_wiris_rtsp_reactivation_skips_redundant_seth_for_enabled_camera(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    sent: list[tuple[str, str]] = []

    def fake_send(command) -> str:
        sent.append((command.host, command.command))
        return "TRUE" if command.command == "GETH" else "OK"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    monkeypatch.setattr(wiris_runtime_workers, "check_tcp_port", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(wiris_runtime_workers, "RTSP_ENABLE_SETTLE_DELAY_MS", 0)
    monkeypatch.setattr(wiris_runtime_workers, "ENCODER_PROFILE_COMMAND_SETTLE_MS", 0)

    results: list[object] = []
    worker = wiris_runtime_workers.WirisRtspEnableWorker(
        hosts={"left": "10.0.0.11", "right": "10.0.0.12"},
        candidate_ports={"left": [2240], "right": [2240]},
        urls={
            "left": "rtsp://10.0.0.11:8554/visible",
            "right": "rtsp://10.0.0.12:8554/visible",
        },
        timeout_s=1.0,
        target_labels=("left",),
        reactivate=True,
    )
    worker.enabled.connect(results.append)
    worker.run()

    assert sent == [("10.0.0.11", "GETH")]
    assert results[0]["mode"] == "reactivate"
    assert results[0]["target_labels"] == ("left",)
    assert results[0]["statuses"] == ["left=reactivated-sent"]
    assert results[0]["should_check"] is True
    assert "SETH TRUE 생략" in "\n".join(results[0]["logs"])


def test_wiris_rtsp_enable_ignores_stale_persisted_encoder_profile(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    sent: list[str] = []

    def fake_send(command) -> str:
        sent.append(command.command)
        return {"GETH": "TRUE"}[command.command]

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    monkeypatch.setattr(wiris_runtime_workers, "check_tcp_port", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(wiris_runtime_workers, "RTSP_ENABLE_SETTLE_DELAY_MS", 0)
    monkeypatch.setattr(wiris_runtime_workers, "ENCODER_PROFILE_COMMAND_SETTLE_MS", 0)
    results: list[object] = []
    worker = wiris_runtime_workers.WirisRtspEnableWorker(
        hosts={"left": "10.0.0.11"},
        candidate_ports={"left": [2240]},
        urls={"left": "rtsp://10.0.0.11:8554/visible"},
        timeout_s=1.0,
        target_labels=("left",),
        visible_encoder_profile={"bitrate": 0, "qp": 1, "gop": 90, "idr": 1},
    )
    worker.enabled.connect(results.append)
    worker.run()

    assert sent == ["GETH"]
    assert results[0]["statuses"] == ["left=enable-sent"]
    assert results[0]["should_check"] is True
    logs = "\n".join(results[0]["logs"])
    assert "SETH TRUE 생략" in logs
    assert "자동 적용하지 않음" in logs
    assert "SEBR/SEQZ/SEGS/SEIR" in logs


@pytest.mark.parametrize(
    ("reactivate", "expected_status"),
    ((False, "left=enable-sent"), (True, "left=reactivated-sent")),
)
def test_wiris_rtsp_enable_sends_seth_only_when_disabled_without_encoder_mutation(
    monkeypatch,
    reactivate: bool,
    expected_status: str,
) -> None:
    from Workstation import wiris_runtime_workers

    sent: list[str] = []

    def fake_send(command) -> str:
        sent.append(command.command)
        return "FALSE" if command.command == "GETH" else "OK"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    monkeypatch.setattr(wiris_runtime_workers, "check_tcp_port", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(wiris_runtime_workers, "RTSP_ENABLE_SETTLE_DELAY_MS", 0)
    results: list[object] = []
    worker = wiris_runtime_workers.WirisRtspEnableWorker(
        hosts={"left": "10.0.0.11"},
        candidate_ports={"left": [2240]},
        urls={"left": "rtsp://10.0.0.11:8554/visible"},
        timeout_s=1.0,
        target_labels=("left",),
        reactivate=reactivate,
        visible_encoder_profile={"bitrate": 0, "qp": 1, "gop": 90, "idr": 1},
    )
    worker.enabled.connect(results.append)
    worker.run()

    assert sent == ["GETH", "SETH TRUE"]
    assert results[0]["statuses"] == [expected_status]
    assert results[0]["should_check"] is True


def test_explicit_encoder_profile_helper_applies_only_changed_fields(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    sent: list[str] = []
    readbacks = {
        "GEBR VISIBLE": iter(("4", "5")),
        "GEQZ VISIBLE": iter(("17",)),
        "GEGS VISIBLE": iter(("30",)),
        "GEIR VISIBLE": iter(("1",)),
    }

    def fake_send(command) -> str:
        sent.append(command.command)
        if command.command == "SEBR VISIBLE 5":
            return "OK"
        return next(readbacks[command.command])

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    monkeypatch.setattr(wiris_runtime_workers, "ENCODER_PROFILE_COMMAND_SETTLE_MS", 0)
    worker = wiris_runtime_workers.WirisRtspEnableWorker(
        hosts={"left": "10.0.0.11"},
        candidate_ports={"left": [2240]},
        urls={"left": "rtsp://10.0.0.11:8554/visible"},
        timeout_s=1.0,
        target_labels=("left",),
        visible_encoder_profile={"bitrate": 5, "qp": 17, "gop": 30, "idr": 1},
    )
    logs: list[str] = []
    error = worker._apply_visible_encoder_profile(
        "10.0.0.11",
        2240,
        logs,
        "왼쪽",
    )

    assert sent == [
        "GEBR VISIBLE",
        "GEQZ VISIBLE",
        "GEGS VISIBLE",
        "GEIR VISIBLE",
        "SEBR VISIBLE 5",
        "GEBR VISIBLE",
    ]
    assert error == ""
    assert "changed=bitrate" in "\n".join(logs)


def test_explicit_encoder_profile_helper_retries_transient_initial_readback_reset(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    sent: list[str] = []
    bitrate_attempts = iter((ConnectionResetError("reset"), "5"))

    def fake_send(command) -> str:
        sent.append(command.command)
        if command.command == "GEBR VISIBLE":
            result = next(bitrate_attempts)
            if isinstance(result, Exception):
                raise result
            return result
        return {
            "GEQZ VISIBLE": "17",
            "GEGS VISIBLE": "30",
            "GEIR VISIBLE": "1",
        }[command.command]

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    monkeypatch.setattr(wiris_runtime_workers, "ENCODER_PROFILE_COMMAND_SETTLE_MS", 0)
    monkeypatch.setattr(wiris_runtime_workers, "ENCODER_PROFILE_READBACK_RETRY_INTERVAL_MS", 0)
    monkeypatch.setattr(wiris_runtime_workers, "ENCODER_PROFILE_READBACK_MAX_ATTEMPTS", 1)
    worker = wiris_runtime_workers.WirisRtspEnableWorker(
        hosts={"left": "10.0.0.11"},
        candidate_ports={"left": [2240]},
        urls={"left": "rtsp://10.0.0.11:8554/visible"},
        timeout_s=1.0,
        target_labels=("left",),
        visible_encoder_profile={"bitrate": 5, "qp": 17, "gop": 30, "idr": 1},
    )
    logs: list[str] = []
    error = worker._apply_visible_encoder_profile(
        "10.0.0.11",
        2240,
        logs,
        "왼쪽",
    )

    assert sent.count("GEBR VISIBLE") == 2
    assert error == ""
    assert "changed=none" in "\n".join(logs)

def test_explicit_encoder_profile_helper_fails_closed_on_readback_mismatch(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    def fake_send(command) -> str:
        if command.command.startswith(("SEBR", "SEQZ", "SEGS", "SEIR")):
            return "OK"
        expected = {
            "GEBR VISIBLE": "4",
            "GEQZ VISIBLE": "17",
            "GEGS VISIBLE": "30",
            "GEIR VISIBLE": "1",
        }
        return expected[command.command]

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    monkeypatch.setattr(wiris_runtime_workers, "ENCODER_PROFILE_COMMAND_SETTLE_MS", 0)
    monkeypatch.setattr(wiris_runtime_workers, "ENCODER_PROFILE_READBACK_MAX_ATTEMPTS", 0)
    monkeypatch.setattr(wiris_runtime_workers, "ENCODER_PROFILE_READBACK_RETRY_INTERVAL_MS", 0)
    worker = wiris_runtime_workers.WirisRtspEnableWorker(
        hosts={"left": "10.0.0.11"},
        candidate_ports={"left": [2240]},
        urls={"left": "rtsp://10.0.0.11:8554/visible"},
        timeout_s=1.0,
        target_labels=("left",),
        visible_encoder_profile={"bitrate": 5, "qp": 17, "gop": 30, "idr": 1},
    )
    error = worker._apply_visible_encoder_profile(
        "10.0.0.11",
        2240,
        [],
        "왼쪽",
    )

    assert "readback=4 expected=5" in error

def test_wiris_control_sessions_are_serialized_per_camera(monkeypatch) -> None:
    from Workstation.workstation_core import wiris_zoom

    first_connection_entered = threading.Event()
    release_first_connection = threading.Event()
    connection_count = 0
    count_lock = threading.Lock()

    class FakeSocket:
        def __init__(self) -> None:
            self._responses = iter((b"OK\n", b""))

        def __enter__(self):
            return self

        def __exit__(self, *_args) -> None:
            return None

        def settimeout(self, _timeout: float) -> None:
            return None

        def sendall(self, _payload: bytes) -> None:
            return None

        def recv(self, _size: int) -> bytes:
            return next(self._responses, b"")

    def fake_create_connection(_endpoint, *, timeout):
        nonlocal connection_count
        with count_lock:
            connection_count += 1
            current = connection_count
        if current == 1:
            first_connection_entered.set()
            assert release_first_connection.wait(1.0)
        return FakeSocket()

    monkeypatch.setattr(wiris_zoom.socket, "create_connection", fake_create_connection)
    command = wiris_zoom.build_wiris_command(
        host="10.0.0.231",
        port=2240,
        command="HIWS",
    )
    responses: list[str] = []
    first = threading.Thread(target=lambda: responses.append(wiris_zoom.send_tcp_line(command)))
    second = threading.Thread(target=lambda: responses.append(wiris_zoom.send_tcp_line(command)))
    first.start()
    assert first_connection_entered.wait(1.0)
    second.start()
    time.sleep(0.05)
    assert connection_count == 1
    release_first_connection.set()
    first.join(1.0)
    second.join(1.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert connection_count == 2
    assert responses == ["OK", "OK"]


def test_wiris_control_response_is_assembled_across_tcp_fragments(monkeypatch) -> None:
    from Workstation.workstation_core import wiris_zoom

    class FakeSocket:
        def __init__(self) -> None:
            self.responses = iter((b"GZ", b"VV 7", b" 8.0\n"))

        def __enter__(self):
            return self

        def __exit__(self, *_args) -> None:
            return None

        def settimeout(self, _timeout: float) -> None:
            return None

        def sendall(self, _payload: bytes) -> None:
            return None

        def recv(self, _size: int) -> bytes:
            return next(self.responses, b"")

    monkeypatch.setattr(
        wiris_zoom.socket,
        "create_connection",
        lambda *_args, **_kwargs: FakeSocket(),
    )
    command = wiris_zoom.build_wiris_command(
        host="10.0.0.249",
        port=2240,
        command="GZVV",
        timeout_s=0.5,
    )

    assert wiris_zoom.send_tcp_line(command) == "GZVV 7 8.0"


def test_wiris_zoom_reports_control_contention_as_busy_not_camera_failure(
    monkeypatch,
) -> None:
    from Workstation import wiris_runtime_workers
    from Workstation.workstation_core.wiris_zoom import WirisControlBusyError

    monkeypatch.setattr(
        wiris_runtime_workers,
        "send_tcp_line",
        lambda _command: (_ for _ in ()).throw(WirisControlBusyError("busy sentinel")),
    )
    results: list[object] = []
    worker = wiris_runtime_workers.WirisZoomWorker(
        (("left", "10.0.0.231", 2240),),
        template="GZVV",
        level=None,
        timeout_s=0.1,
    )
    worker.completed.connect(results.append)
    worker.run()

    assert results == [[{"label": "left", "state": "busy", "detail": "busy sentinel"}]]


def test_wiris_both_command_continues_healthy_side_when_peer_fails_before_barrier(
    monkeypatch,
) -> None:
    from Workstation import wiris_runtime_workers

    sent: list[str] = []
    monkeypatch.setattr(
        wiris_runtime_workers,
        "send_tcp_line",
        lambda command: sent.append(command.host) or "OK",
    )
    results: list[object] = []
    worker = wiris_runtime_workers.WirisCommandWorker(
        (
            ("left", "10.0.0.231", 2240, 0.1),
            ("right", "", 2240, 0.1),
        ),
        command_text="HIWS",
        terminator="\n",
    )
    worker.completed.connect(results.append)
    started_s = time.monotonic()
    worker.run()

    assert time.monotonic() - started_s < 0.5
    assert sent == ["10.0.0.231"]
    assert results[0]["statuses"]["left"] == "OK"
    assert results[0]["statuses"]["right"].startswith("실패:")

def test_camera_network_discovery_worker_keeps_nm_queries_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    interface = SimpleNamespace(name="enx-camera", address="10.0.0.1/24")
    candidate = SimpleNamespace(interface=interface, neighbor_hosts=("10.0.0.11",))
    monkeypatch.setattr(
        device_runtime_workers,
        "discover_host_interface_candidates",
        lambda **_kwargs: [candidate],
    )
    monkeypatch.setattr(
        device_runtime_workers,
        "resolve_nm_interface_address",
        lambda _name, _hints: "10.0.0.1/24",
    )
    monkeypatch.setattr(device_runtime_workers, "nm_device_connection_name", lambda _name: "camera-profile")
    monkeypatch.setattr(device_runtime_workers, "nm_connection_ipv4_addresses", lambda _name: ("10.0.0.1/24",))

    results: list[object] = []
    worker = standalone_gui.CameraNetworkDiscoveryWorker(
        host_address="10.0.0.1/24",
        host_hints=("10.0.0.11",),
    )
    worker.discovered.connect(results.append)
    worker.run()

    assert results[0]["candidates"] == (candidate,)
    assert results[0]["resolved_address"] == "10.0.0.1/24"
    assert results[0]["connection_name"] == "camera-profile"


def test_camera_network_discovery_projects_host_interface_into_candidate_rows() -> None:
    from Workstation import standalone_gui
    from Workstation.device_runtime_controller import DeviceRuntimeController

    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    interface = SimpleNamespace(name="enx-new", address="10.0.0.10/24")
    candidate = SimpleNamespace(
        interface=interface,
        neighbor_hosts=("10.0.0.231", "10.0.0.232"),
    )
    panel = SimpleNamespace(
        _camera_candidate_rows=[
            {"kind": "WIRIS", "address": "stale", "status": "stale"}
        ],
        interface_edit=QtWidgets.QLineEdit(),
        host_address_edit=QtWidgets.QLineEdit(),
        camera_connection_label=QtWidgets.QLabel(),
        workstation_camera_discovery_status="SEARCHING",
        _log=lambda _message: None,
        _refresh_operation_header=lambda: None,
    )

    def apply_host_interface_candidates(candidates: list[object]) -> None:
        selected = candidates[0].interface
        panel.interface_edit.setText(selected.name)
        panel.host_address_edit.setText(str(selected.address))

    panel._apply_host_interface_candidates = apply_host_interface_candidates
    panel._set_camera_candidate_table_rows = lambda rows: setattr(
        panel, "_camera_candidate_rows", list(rows)
    )
    panel._set_camera_candidate_rows = lambda *args: (
        standalone_gui.WirisStereoDistancePanel._set_camera_candidate_rows(panel, *args)
    )

    DeviceRuntimeController(panel)._apply_camera_network_discovery(
        {
            "candidates": (candidate,),
            "resolved_address": "10.0.0.10/24",
            "connection_name": "camera-profile",
            "addresses": ("10.0.0.10/24",),
        }
    )

    assert panel.workstation_camera_discovery_status == "FOUND 1 HOST INTERFACES"
    assert panel._camera_candidate_rows == [
        {
            "kind": "NIC",
            "address": "enx-new",
            "port": "",
            "status": "neighbors=10.0.0.231, 10.0.0.232",
            "interface": interface,
            "host_ip": "10.0.0.10/24",
            "candidate": candidate,
        }
    ]


def test_camera_network_discovery_zero_result_clears_stale_candidate_rows() -> None:
    from Workstation import standalone_gui
    from Workstation.device_runtime_controller import DeviceRuntimeController

    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = SimpleNamespace(
        _camera_candidate_rows=[
            {"kind": "WIRIS", "address": "stale", "status": "stale"}
        ],
        interface_edit=QtWidgets.QLineEdit("enx-old"),
        host_address_edit=QtWidgets.QLineEdit("10.0.0.11/32"),
        camera_connection_label=QtWidgets.QLabel(),
        workstation_camera_discovery_status="SEARCHING",
        _log=lambda _message: None,
        _refresh_operation_header=lambda: None,
        _apply_host_interface_candidates=lambda _candidates: None,
    )
    panel._set_camera_candidate_table_rows = lambda rows: setattr(
        panel, "_camera_candidate_rows", list(rows)
    )
    panel._set_camera_candidate_rows = lambda *args: (
        standalone_gui.WirisStereoDistancePanel._set_camera_candidate_rows(panel, *args)
    )

    DeviceRuntimeController(panel)._apply_camera_network_discovery(
        {
            "candidates": (),
            "resolved_address": None,
            "connection_name": None,
            "addresses": (),
        }
    )

    assert panel.workstation_camera_discovery_status == "FOUND 0 HOST INTERFACES"
    assert panel._camera_candidate_rows == []


def test_wiris_clock_worker_keeps_stus_and_gdti_io_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import wiris_runtime_workers

    sent: list[str] = []

    def fake_send(command) -> str:
        sent.append(str(command.command))
        return "OK" if command.command.startswith("STUS ") else "2026/08/09 12:00:00"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    monkeypatch.setattr(
        standalone_gui.WirisClockCommandWorker,
        "_measure_control_rtt",
        lambda self, **_kwargs: {
            "command": "HIWS",
            "samples_s": [0.009, 0.010, 0.011],
            "median_s": 0.010,
            "errors": [],
        },
    )
    target_infos = (
        ("left", "10.0.0.11", 5000, 1.0),
        ("right", "10.0.0.12", 5001, 1.0),
    )
    stus_results: list[object] = []
    stus_worker = standalone_gui.WirisClockCommandWorker(
        target_infos,
        command_text="STUS",
        shared_utc=True,
    )
    stus_worker.completed.connect(stus_results.append)
    stus_worker.run()

    gdti_results: list[object] = []
    gdti_worker = standalone_gui.WirisClockCommandWorker(target_infos, command_text="GDTI")
    gdti_worker.completed.connect(gdti_results.append)
    gdti_worker.run()

    assert len(sent) == 4
    assert all(command.startswith("STUS ") for command in sent[:2])
    assert stus_results[0]["results"]["left"]["response"] == "OK"
    assert stus_results[0]["results"]["right"]["response"] == "OK"
    assert stus_results[0]["compensation_method"] == "control-rtt-half-plus-parse-margin"
    assert stus_results[0]["results"]["left"]["stus_rtt_median_s"] == pytest.approx(0.010)
    assert stus_results[0]["results"]["left"]["stus_one_way_delay_s"] == pytest.approx(0.005)
    assert stus_results[0]["results"]["left"]["stus_compensation_s"] == pytest.approx(0.007)
    assert stus_results[0]["requested_utc_by_camera"] == stus_results[0]["command_times"]
    assert gdti_results[0]["results"]["left"]["response"].startswith("2026/")

def test_wiris_stus_compensation_rejects_invalid_rtt() -> None:
    from Workstation import standalone_gui

    assert standalone_gui.WirisClockCommandWorker._stus_one_way_delay_s(0.012) == pytest.approx(0.006)
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(0.012) == pytest.approx(0.008)
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(0.515) == pytest.approx(0.2595)
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(None) == 0.0
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(float("nan")) == 0.0
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(2.001) == pytest.approx(1.0025)
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(3.901) == 0.0


def test_runtime_pair_status_shows_requested_effective_and_distinct_deltas() -> None:
    from Workstation import standalone_gui

    panel = SimpleNamespace(
        selected_runtime_frame_pair_delta_s=0.020,
        selected_runtime_frame_pair_basis="APP RECEIVE",
        selected_runtime_frame_pair_camera_rtc_delta_s=32_400.0,
        selected_runtime_frame_metadata={
            "left": {
                "timestamp_s": 10.0,
                "rtcp_frame_ntp_unix_s": 100.0,
            },
            "right": {
                "timestamp_s": 10.02,
                "rtcp_frame_ntp_unix_s": 32_500.0,
            },
        },
    )

    text, tooltip = standalone_gui.WirisStereoDistancePanel._format_runtime_pair_status(
        panel
    )

    assert "Pair: requested=APP RECEIVE" in text
    assert "effective=APP RECEIVE" in text
    assert "selected Δ 20.0 ms" in text
    assert "app Δ 20.0 ms" in text
    assert "NTP Δ 32400000.0 ms (record-only)" in text
    assert "left pair 전체 시각" in tooltip


def test_runtime_pair_status_shows_healthy_rtcp_and_app_fallback_reason() -> None:
    from Workstation import standalone_gui

    base = {
        "selected_runtime_frame_pair_delta_s": 0.010,
        "selected_runtime_frame_pair_basis": "camera_rtcp",
        "selected_runtime_frame_pair_requested_basis": "camera_rtcp",
        "selected_runtime_frame_pair_fallback_reason": "",
        "selected_runtime_frame_pair_camera_rtc_delta_s": 0.010,
        "selected_runtime_frame_metadata": {
            "left": {
                "timestamp_s": 10.0,
                "app_receive_monotonic_ns": 10_000_000_000,
                "rtcp_frame_ntp_unix_s": 100.0,
            },
            "right": {
                "timestamp_s": 10.09,
                "app_receive_monotonic_ns": 10_090_000_000,
                "rtcp_frame_ntp_unix_s": 100.01,
            },
        },
    }
    healthy_text, _ = standalone_gui.WirisStereoDistancePanel._format_runtime_pair_status(
        SimpleNamespace(**base)
    )

    assert "requested=RTCP/NTP" in healthy_text
    assert "effective=RTCP/NTP" in healthy_text
    assert "selected Δ 10.0 ms" in healthy_text
    assert "app Δ 90.0 ms" in healthy_text
    assert "NTP Δ 10.0 ms (selected)" in healthy_text

    fallback_text, _ = standalone_gui.WirisStereoDistancePanel._format_runtime_pair_status(
        SimpleNamespace(
            **{
                **base,
                "selected_runtime_frame_pair_delta_s": 0.090,
                "selected_runtime_frame_pair_basis": "app_receive_monotonic",
                "selected_runtime_frame_pair_fallback_reason": "CLOCK_NOT_VERIFIED",
            }
        )
    )

    assert "requested=RTCP/NTP" in fallback_text
    assert "effective=APP RECEIVE" in fallback_text
    assert "fallback=CLOCK_NOT_VERIFIED" in fallback_text
    assert "NTP Δ 10.0 ms (record-only)" in fallback_text


def test_noncoordinator_app_pair_path_hard_rejects_100ms_plus_one_ns() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel
    from Workstation.workstation_core.stereo import (
        DetectionObservation,
        detection_timestamp_for_basis,
    )

    def observation(side: str, timestamp_s: float, exact_ns: int) -> DetectionObservation:
        return DetectionObservation(
            camera=side,
            timestamp_s=timestamp_s,
            width=1280,
            height=720,
            x1=100.0,
            y1=100.0,
            x2=200.0,
            y2=200.0,
            confidence=0.9,
            class_id=0,
            timestamp_metadata={"app_receive_monotonic_ns": exact_ns},
            candidate_class_ids=(0,),
        )

    samples = {
        "left": [observation("left", 10.0, 10_000_000_000)],
        "right": [observation("right", 10.100000001, 10_100_000_001)],
    }
    panel = SimpleNamespace(
        _detection_pair_coordinator_active=False,
        _recent_detection_observations=lambda side, **_kwargs: samples[side],
        _yolo_pair_max_delta_s=lambda: 0.500,
        _yolo_match_timestamp_s=lambda item: detection_timestamp_for_basis(
            item,
            "app_receive_monotonic",
        ),
        _validated_correspondence_pair=lambda pair, _left, _right: pair,
        _yolo_vertical_max_delta_px=lambda: float("inf"),
        _runtime_pair_uses_app_fallback=True,
        _last_pair_advisories=(),
        _last_correspondence_rejection_reason="",
    )

    pair = WirisStereoDistancePanel._matched_detection_pair(panel)

    assert pair is None
    assert panel._last_correspondence_rejection_reason == "APP_RECEIVE_DELTA_EXCEEDED"
    assert panel._runtime_pair_uses_app_fallback is False


def test_raw_stus_sdk_command_is_blocked_in_favor_of_interlocked_workflow() -> None:
    from Workstation.wiris_control_controller import WirisControlController

    statuses: list[str] = []
    mutations: list[object] = []
    panel = SimpleNamespace(
        wiris_command_edit=SimpleNamespace(text=lambda: "STUS 123 456"),
        wiris_command_status_label=SimpleNamespace(setText=statuses.append),
        _start_wiris_command_worker=lambda **payload: mutations.append(payload),
    )

    WirisControlController(panel).send_selected_wiris_command()

    assert mutations == []
    assert statuses and "Raw STUS is blocked" in statuses[-1]


def test_stus_and_generic_camera_control_reject_shutdown_or_active_action() -> None:
    from Workstation.wiris_control_controller import WirisControlController

    statuses: list[str] = []
    logs: list[str] = []
    panel = SimpleNamespace(
        _shutdown_in_progress=True,
        _camera_sync_stus_operation=None,
        wiris_command_status_label=SimpleNamespace(setText=statuses.append),
        _log=logs.append,
    )
    controller = WirisControlController(panel)

    assert controller.set_selected_wiris_time_from_pc("both") is False
    assert getattr(panel, "wiris_time_sync_thread", None) is None
    assert panel._camera_sync_stus_operation is None

    panel._shutdown_in_progress = False
    panel._camera_sync_stus_operation = {
        "active": True,
        "attempts_started": 1,
        "max_attempts": 2,
    }
    panel.wiris_command_edit = SimpleNamespace(text=lambda: "HIWS")
    controller.send_selected_wiris_command()

    assert "STUS closed-loop owns" in statuses[-1]

def test_camera_network_address_worker_keeps_interface_apply_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    calls: list[tuple[str, str]] = []
    monkeypatch.setattr(
        device_runtime_workers,
        "apply_interface_address",
        lambda interface_name, address_text: calls.append((interface_name, address_text)) or (True, "applied"),
    )
    monkeypatch.setattr(device_runtime_workers, "interface_addresses", lambda interface_name: f"{interface_name} UP")
    results: list[object] = []
    worker = standalone_gui.CameraNetworkAddressApplyWorker(
        interface_name="enx-camera",
        address_text="10.0.0.1/24",
    )
    worker.applied.connect(results.append)
    worker.run()

    assert calls == [("enx-camera", "10.0.0.1/24")]
    assert results == [
        {
            "ok": True,
            "message": "applied",
            "interface_name": "enx-camera",
            "address_text": "10.0.0.1/24",
            "addresses": "enx-camera UP",
        }
    ]


def test_camera_network_pkexec_timeout_returns_finite_failure(monkeypatch) -> None:
    import subprocess

    from Workstation.workstation_core import network

    calls: list[tuple[list[str], object]] = []

    def fake_run(command, **kwargs):
        calls.append((list(command), kwargs.get("timeout")))
        if command[0] == "ip":
            return SimpleNamespace(
                returncode=1,
                stdout="",
                stderr="RTNETLINK answers: Operation not permitted",
            )
        raise subprocess.TimeoutExpired(command, kwargs["timeout"])

    monkeypatch.setattr(network, "interface_has_address", lambda *_args: False)
    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network.shutil, "which", lambda _name: "/usr/bin/pkexec")
    monkeypatch.setenv("DISPLAY", ":1")
    monkeypatch.setenv("QT_QPA_PLATFORM", "xcb")

    ok, message = network.apply_interface_address("enx-camera", "10.0.0.10/24")

    assert ok is False
    assert "timed out" in message
    assert calls[-1][1] == network.PKEXEC_TIMEOUT_SECONDS

def test_wiris_stip_worker_keeps_interface_and_tcp_io_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import wiris_runtime_workers

    interface = SimpleNamespace(name="enx-camera", address="10.0.0.1/24")
    candidate = SimpleNamespace(interface=interface, neighbor_hosts=())
    sent: list[str] = []
    monkeypatch.setattr(wiris_runtime_workers, "interface_link_exists", lambda _name: False)
    monkeypatch.setattr(wiris_runtime_workers, "discover_host_interface_candidates", lambda **_kwargs: [candidate])
    monkeypatch.setattr(wiris_runtime_workers, "resolve_nm_interface_address", lambda _name, _hints: None)
    monkeypatch.setattr(wiris_runtime_workers, "interface_has_address", lambda _name, _address: True)

    def fake_send(command) -> str:
        sent.append(str(command.command))
        return "OK"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    results: list[object] = []
    worker = standalone_gui.WirisStipWorker(
        interface_name="enx-camera",
        host_address="10.0.0.1/24",
        host_hints=("10.0.0.11",),
        current_host="10.0.0.11",
        new_ip="10.0.0.21",
        mask="255.255.255.0",
        port=5000,
        reboot=False,
        timeout_s=1.0,
    )
    worker.completed.connect(results.append)
    worker.run()

    assert sent == [
        "STIP 10.0.0.21",
        "STMS 255.255.255.0",
    ]
    assert results[0]["state"] == "sent"
    assert results[0]["interface_name"] == "enx-camera"

def test_wiris_command_worker_reports_partial_timeout_without_ui_join(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import wiris_runtime_workers

    def fake_send(command) -> str:
        if command.host.endswith("12"):
            raise TimeoutError("right camera timeout")
        return "OK"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    results: list[object] = []
    worker = standalone_gui.WirisCommandWorker(
        (
            ("left", "10.0.0.11", 5000, 1.0),
            ("right", "10.0.0.12", 5001, 1.0),
        ),
        command_text="HIWS",
        terminator=standalone_gui.LINE_TERMINATOR,
    )
    worker.completed.connect(results.append)
    worker.run()

    assert results[0]["statuses"]["left"] == "OK"
    assert results[0]["statuses"]["right"] == "실패: right camera timeout"

def test_workstation_uses_explicit_camera_coordinate_label() -> None:
    source = (Path(__file__).resolve().parents[2] / "Workstation" / "standalone_gui.py").read_text(
        encoding="utf-8"
    )
    assert "OpenCV optical RDF" in source
    assert "ROS Optical Frame" not in source

def test_camera_mode_switch_restarts_after_workers_stop_without_blocking(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    callbacks: list[object] = []
    monkeypatch.setattr(
        standalone_gui.QtCore.QTimer,
        "singleShot",
        lambda _delay_ms, callback: callbacks.append(callback),
    )
    stopped = {"value": False}
    starts: list[tuple[str, bool]] = []
    panel.left_pane.stop = lambda *, wait=False: stopped["value"]
    panel.right_pane.stop = lambda *, wait=False: stopped["value"]
    panel.left_pane.start_stream = (
        lambda _url, *, enable_detection, **_kwargs: starts.append(("left", enable_detection)) or True
    )
    panel.right_pane.start_stream = (
        lambda _url, *, enable_detection, **_kwargs: starts.append(("right", enable_detection)) or True
    )

    panel._start_camera_streams(enable_detection=True, switch_to_depth=True)
    assert panel._pending_camera_start == (True, True)
    assert starts == []
    assert len(callbacks) == 1

    stopped["value"] = True
    callbacks.pop(0)()
    assert panel._pending_camera_start is None
    assert starts == [("left", True), ("right", True)]
    assert panel.camera_connected is False
    assert panel.yolo_enabled is False
    from PyQt5 import QtGui

    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    panel._camera_stream_frame_ready(
        "left",
        image,
        {"stream_generation": panel.left_pane.stream_generation},
    )
    panel._camera_stream_frame_ready(
        "right",
        image,
        {"stream_generation": panel.right_pane.stream_generation},
    )
    assert panel.camera_connected is True
    assert panel.yolo_enabled is True

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_camera_disconnect_watchdog_retries_wedged_stop_until_it_exits(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    callbacks: list[object] = []
    monkeypatch.setattr(
        standalone_gui.QtCore.QTimer,
        "singleShot",
        lambda _delay_ms, callback: callbacks.append(callback),
    )
    stopped = {"value": False}
    panel.left_pane.stop = lambda *, wait=False: stopped["value"]
    panel.right_pane.stop = lambda *, wait=False: stopped["value"]
    panel._last_wiris_pc_utc_sync = {
        "command": "STUS",
        "result": "success",
    }

    # A worker that does not exit on the first cooperative stop must not be
    # abandoned in "종료 대기 중" forever; a watchdog keeps retrying it.
    assert panel.stop_yolo_streams(wait=False) is False
    assert panel._last_wiris_pc_utc_sync["result"] == "success"
    assert panel.frame_timing_status == "not-verified"
    assert len(callbacks) == 1

    callbacks.pop(0)()
    assert len(callbacks) == 1

    stopped["value"] = True
    callbacks.pop(0)()
    assert callbacks == []

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_camera_start_cancels_stale_disconnect_watchdog(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    callbacks: list[object] = []
    monkeypatch.setattr(
        standalone_gui.QtCore.QTimer,
        "singleShot",
        lambda _delay_ms, callback: callbacks.append(callback),
    )
    panel.left_pane.stop = lambda *, wait=False: False
    panel.right_pane.stop = lambda *, wait=False: False

    assert panel.stop_yolo_streams(wait=False) is False
    pending_watchdog = callbacks.pop(0)

    # A fresh connect takes over teardown of the stale worker (via its own
    # _stop_camera_panes/_schedule_camera_start_retry); the leftover
    # watchdog tick from the earlier disconnect must not call stop() again
    # on the pane it no longer owns.
    panel.left_url_edit.setText("rtsp://left")
    panel.right_url_edit.setText("rtsp://right")
    post_start_stop_calls: list[str] = []
    panel.left_pane.stop = lambda *, wait=False: post_start_stop_calls.append("left") or True
    panel.right_pane.stop = lambda *, wait=False: post_start_stop_calls.append("right") or True
    panel.left_pane.start_stream = lambda _url, **_kwargs: True
    panel.right_pane.start_stream = lambda _url, **_kwargs: True
    panel._start_camera_streams(enable_detection=False, switch_to_depth=False)
    calls_from_start = list(post_start_stop_calls)

    pending_watchdog()
    assert post_start_stop_calls == calls_from_start

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_camera_start_keeps_healthy_side_when_other_side_start_fails(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.left_pane.stop = lambda *, wait=False: True
    panel.right_pane.stop = lambda *, wait=False: True
    starts: list[str] = []
    panel.left_pane.start_stream = lambda _url, **_kwargs: starts.append("left") or True
    panel.right_pane.start_stream = lambda _url, **_kwargs: starts.append("right") or False
    retries: list[tuple[bool, bool]] = []
    monkeypatch.setattr(
        panel,
        "_schedule_camera_start_retry",
        lambda enable_detection, switch_to_depth: retries.append((enable_detection, switch_to_depth)),
    )

    panel._start_camera_streams(enable_detection=False, switch_to_depth=False)

    assert starts == ["left", "right"]
    assert retries == []
    assert panel._pending_camera_start is None
    assert panel.camera_connected is False
    from PyQt5 import QtGui

    panel._camera_stream_frame_ready(
        "left",
        QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888),
        {"stream_generation": panel.left_pane.stream_generation},
    )
    assert panel.camera_connected is True
    assert panel.camera_connection_label.text() == "camera partial: left LIVE; right unavailable"
    assert "current-generation frame 확인" in panel.wiris_log.toPlainText()
    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    app.processEvents()
    panel.deleteLater()
    app.processEvents()

def test_system_yolo_input_size_applies_to_runtime_owner() -> None:
    from Workstation.settings_controller import WorkstationSettingsController

    input_size = QtWidgets.QSpinBox()
    input_size.setRange(160, 1920)
    input_size.setValue(416)
    panel = SimpleNamespace(
        _workstation_setting_overrides={},
        input_size_spin=input_size,
        manual_focal_checkbox=QtWidgets.QCheckBox(),
        _refresh_yolo_device_toggle=lambda: None,
        _refresh_pixhawk_ethernet_mode_controls=lambda: None,
        _set_focal_override_enabled=lambda _enabled: None,
        _refresh_operation_header=lambda: None,
        _log=lambda _message: None,
    )

    WorkstationSettingsController(panel).apply_workstation_system_settings(
        {"yolo_input_size": 960}
    )

    assert input_size.value() == 960
    assert panel._workstation_setting_overrides["yolo_input_size"] == 960
    input_size.deleteLater()
    panel.manual_focal_checkbox.deleteLater()

def test_wiris_command_catalog_covers_official_workswell_sdk_opcodes() -> None:
    from Workstation import standalone_gui
    from Workstation.wiris_sdk_catalog import (
        WIRIS_COMMAND_CATALOG,
        WIRIS_SDK_MANUAL_FIRMWARE,
        WIRIS_SDK_MANUAL_REVISION,
        WIRIS_SDK_MANUAL_URL,
    )

    # Workstation only operates WIRIS Enterprise units, so opcodes the manual
    # documents as exclusive to WIRIS AGRO (5.8, CWSI/crop stress), WIRIS
    # Security (5.7), GIS 320 (5.11), or AGRO's Colourmap (5.13) are
    # deliberately absent from the catalog and therefore from this set too.
    expected_opcodes = set(
        """
        ACTV IACT SETH GETH SSDK SDLM GDLM HIWS GSRN GATN GFWV GTCU GTIC GTIN GFPW
        MOVE GGPS STUT GTUT SRMD GRMD SRMM GRMM SRWC GRWC SREN GREN GREL
        SLAY STTY SMCA GMCA
        SGPI GGPI SZIN SZOT SZSM GZSM SZTN GZTV GZTL SZVN GZVV GZVL
        GPTE SPTE SPTI GPTL GPLV CPTR ICPT RCRS
        RCRF IRCR SPIC GPIC GGFT SGFT SBFL GBFL SEFL GEFL SCSB GCSB SALM GALM SALV
        GALV SALC GALC GTEX GROT GOPT CUPD IUPD STEM GTEM STRT GTRT STAT GTAT STII
        GTII STSC GTSC STCP GTCP GTSL GTSN IMCR SLEN GLEN GLEL SIRJ GIRJ SIRT GIRT
        SICT GICT SISR GISR SISS GISS SIVI GIVI SIHR GIHR SVTH GVTH SVTE GVTE SVVI
        GVVI SVHR GVHR SILC GILC SHRR GHRR SLRF GLRF GLRD SDTI GDTI STUS GMST GMSI
        GMFR GMCP GTRC GVRC GEST CPST CPSS GTRE SEBR GEBR SEGS GEGS SEIR GEIR SEQZ
        GEQZ STRG GTRG STIP STMS GTMS STGW GTGW SMGW GMGW SDST SHDW REBT
        """.split()
    )
    actual_opcodes = {
        record[2].split()[0]
        for record in WIRIS_COMMAND_CATALOG
        if record[2]
    }

    assert actual_opcodes == expected_opcodes
    assert standalone_gui.WIRIS_COMMAND_CATALOG is WIRIS_COMMAND_CATALOG
    assert WIRIS_SDK_MANUAL_REVISION == "241106EN"
    assert WIRIS_SDK_MANUAL_FIRMWARE == "2.10.60+"
    assert WIRIS_SDK_MANUAL_URL.startswith("https://my.workswell.eu/")

def test_advanced_camera_control_displays_wiris_sdk_responses() -> None:
    from Workstation.status_model import StatusModel
    from Workstation.workspaces import AdvancedWorkspace

    class ResponseBus(QtCore.QObject):
        response = QtCore.pyqtSignal(object)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    bus = ResponseBus()
    backend.panel.wiris_command_response = bus.response
    advanced = AdvancedWorkspace(backend, StatusModel())
    assert "Status" not in {
        group.title() for group in advanced.findChildren(QtWidgets.QGroupBox)
    }
    assert advanced.wiris_command_edit.text() == "HIWS"
    assert advanced.wiris_status_label.text() == "Connection check"

    bus.response.emit(
        {
            "phase": "completed",
            "target": "both",
            "command": "HIWS",
            "status_text": "왼쪽: OK | 오른쪽: 실패: timeout",
            "target_infos": (
                ("left", "192.168.0.101", 2240, 1.0),
                ("right", "192.168.0.102", 2240, 1.0),
            ),
            "statuses": {"left": "OK", "right": "실패: timeout"},
            "command_logs": {
                "left": "> left 192.168.0.101:2240 HIWS\nOK\n",
            },
        }
    )
    app.processEvents()

    response_text = advanced.wiris_response_log.toPlainText()
    assert advanced.wiris_response_log.isReadOnly()
    assert "target=both command=HIWS" in response_text
    assert "> left 192.168.0.101:2240 HIWS" in response_text
    assert "right: 실패: timeout" in response_text
    assert advanced.wiris_status_label.text() == "왼쪽: OK | 오른쪽: 실패: timeout"

    advanced.wiris_response_clear_button.click()
    assert advanced.wiris_response_log.toPlainText() == ""
    advanced.deleteLater()
    bus.deleteLater()
    app.processEvents()

def test_advanced_camera_control_sends_through_actual_panel_owner(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    sent: list[tuple[str, int, str]] = []

    def fake_send(command) -> str:
        sent.append((command.host, command.port, command.command))
        return "OK"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow()
    try:
        window._select_context("tasks.advanced")
        # Any startup/alignment LRF polling belongs to the window lifecycle,
        # not to this raw WIRIS command assertion.
        sent.clear()
        window.advanced.wiris_command_edit.setText("HIWS")
        window.advanced.wiris_send_button.click()
        deadline = time.monotonic() + 2.0
        while window.backend.panel.wiris_command_thread is not None and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.01)

        assert window.backend.panel.wiris_command_thread is None
        response_text = window.advanced.wiris_response_log.toPlainText()
        assert len(sent) == 2
        assert {item[2] for item in sent} == {"HIWS"}
        assert "target=both command=HIWS" in response_text
        assert response_text.count("OK") == 2
        assert window.advanced.wiris_status_label.text().count("OK") == 2
    finally:
        window.backend.shutdown_for_close()
        window.deleteLater()
        app.processEvents()

def test_wiris_controller_publishes_completed_command_payload() -> None:
    from Workstation.wiris_control_controller import WirisControlController

    class ResponseBus(QtCore.QObject):
        response = QtCore.pyqtSignal(object)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    bus = ResponseBus()
    published: list[dict[str, object]] = []
    bus.response.connect(published.append)
    panel = SimpleNamespace(
        wiris_command_response=bus.response,
        wiris_command_status_label=QtWidgets.QLabel(),
        _append_wiris_command_log=lambda _text: None,
        _camera_name_ko=lambda label: {"left": "왼쪽", "right": "오른쪽"}.get(label, label),
        _log=lambda _text: None,
    )
    controller = WirisControlController(panel)

    controller._apply_wiris_command_result(
        "both",
        "HIWS",
        {
            "statuses": {"left": "OK", "right": "OK"},
            "command_logs": {"left": "left OK", "right": "right OK"},
            "send_times": {"left": 1.0, "right": 1.001},
            "target_infos": (
                ("left", "192.168.0.101", 2240, 1.0),
                ("right", "192.168.0.102", 2240, 1.0),
            ),
        },
    )
    app.processEvents()

    assert published[-1]["phase"] == "completed"
    assert published[-1]["target"] == "both"
    assert published[-1]["command"] == "HIWS"
    assert published[-1]["statuses"] == {"left": "OK", "right": "OK"}
    assert "전송 시각 차이 왼쪽→오른쪽: 1.000 ms" in str(published[-1]["status_text"])
    bus.deleteLater()
    panel.wiris_command_status_label.deleteLater()
    app.processEvents()

def test_diagnostics_ftp_tab_surfaces_the_ftp_lifecycle_log() -> None:
    from Workstation.status_model import StatusModel
    from Workstation.workspaces import DiagnosticsContent

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    diagnostics = DiagnosticsContent(_FakeBackend(), StatusModel())

    assert diagnostics.tabs.tabText(5) == "FTP"
    diagnostics.tabs.setCurrentIndex(5)
    diagnostics.refresh({})

    assert diagnostics.ftp_view.toPlainText() == "[fake] FTP source-backed log"
    diagnostics.deleteLater()
    app.processEvents()

def test_standalone_backend_ftp_log_includes_the_current_ftp_status() -> None:
    from Workstation.standalone_backend import StandalonePanelBackend

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = SimpleNamespace(
        ftp_log_lines=["[캘리브레이션] WIRIS FTP Capture calib-1 left: ftp_downloaded=True"],
        calibration_status_label=QtWidgets.QLabel("WIRIS FTP Capture calib-1: 좌/우 저장 완료"),
    )
    backend = StandalonePanelBackend(panel)

    text = backend.ftp_diagnostic_text()
    assert "ftp_downloaded=True" in text
    assert "[status] WIRIS FTP Capture calib-1: 좌/우 저장 완료" in text

def test_workstation_camera_pane_shutdown_is_cooperative_and_retryable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.workstation_core import camera_pane
    from Workstation.workstation_core.camera_pane import CameraPane

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    calls: list[str] = []

    class FakeThread:
        def __init__(self) -> None:
            self.running = True
            self.wait_calls: list[int] = []

        def wait(self, timeout: int) -> bool:
            self.wait_calls.append(timeout)
            return not self.running

    thread = FakeThread()

    class FakeWorker:
        def request_stop(self) -> None:
            calls.append("request_stop")

        def stop(self) -> None:
            calls.append("stop")
            thread.running = False

        def terminate_child_process(self) -> None:
            calls.append("terminate")

        def kill_child_process(self) -> None:
            calls.append("kill")

    worker = FakeWorker()
    pane.worker = worker
    pane.thread = thread

    assert pane.stop(wait=True) is True
    assert calls == ["request_stop", "stop"]
    assert thread.wait_calls == [30000, 1000]
    assert pane.worker is None
    assert pane.thread is None

    pane.worker = worker
    pane.thread = thread
    thread.running = True
    calls.clear()
    assert pane.stop(wait=False) is False
    assert calls == ["request_stop"]
    assert pane.worker is worker
    assert pane.thread is thread

    pane._stop_requested_monotonic_s = 10.0
    monkeypatch.setattr(camera_pane.time, "monotonic", lambda: 40.1)
    assert pane.stop(wait=False) is False
    assert calls[-2:] == ["request_stop", "terminate"]
    monkeypatch.setattr(camera_pane.time, "monotonic", lambda: 50.1)
    assert pane.stop(wait=False) is False
    assert calls[-2:] == ["request_stop", "kill"]

    pane._clear_thread_if_current(1, thread)
    assert pane.worker is None
    assert pane.thread is None
    pane.deleteLater()
    app.processEvents()


def test_retired_native_ring_is_swept_after_pinned_lease_releases() -> None:
    from Workstation.workstation_core.camera_pane import CameraPane

    class Ring:
        active_leases = 1

        def __init__(self) -> None:
            self.closed = False
            self.unlinked = False

        def metrics(self) -> dict[str, int]:
            return {"active_leases": self.active_leases}

        def close(self) -> None:
            self.closed = True

        def unlink(self) -> None:
            self.unlinked = True

    ring = Ring()
    pane = SimpleNamespace(_retired_native_frame_rings=[])

    CameraPane._retire_native_frame_ring(pane, ring)
    assert pane._retired_native_frame_rings == [ring]
    assert ring.closed is False
    assert ring.unlinked is False

    ring.active_leases = 0
    assert CameraPane.sweep_retired_native_frame_rings(pane) is True
    assert pane._retired_native_frame_rings == []
    assert ring.closed is True
    assert ring.unlinked is True


def test_recording_decoder_fallback_isolated_to_failed_side() -> None:
    from types import SimpleNamespace

    from Workstation.standalone_gui import WirisStereoDistancePanel

    events = []
    panel = SimpleNamespace(
        _runtime_decoder_override=None,
        _runtime_decoder_overrides={"left": None, "right": None},
        decoder_fallback_reasons={"left": "", "right": ""},
        decoder_combo=SimpleNamespace(currentData=lambda: "auto"),
        active_stream_decoders={"left": "nvh264dec", "right": "nvh264dec"},
        _measurement_run_state=lambda: "RUNNING",
        _camera_stream_start_status=lambda side, state: events.append(
            ("start-status", side, state)
        ),
        _camera_name_ko=lambda _camera: "왼쪽",
        _log=lambda text: events.append(("log", text)),
        stop_yolo_streams=lambda: events.append(("stop",)),
        _start_camera_streams=lambda **kwargs: events.append(("start", kwargs)),
    )

    WirisStereoDistancePanel._handle_runtime_decoder_fallback(
        panel,
        "left",
        "DECODER_FALLBACK_REQUIRED: nvh264dec first-frame timeout",
    )

    assert panel._runtime_decoder_override is None
    assert panel._runtime_decoder_overrides == {"left": "software", "right": None}
    assert "first-frame timeout" in panel.decoder_fallback_reasons["left"]
    assert panel.decoder_fallback_reasons["right"] == ""
    assert panel.active_stream_decoders == {"left": None, "right": "nvh264dec"}
    assert not any(event[0] in {"stop", "start"} for event in events)
    assert any(event[0] == "start-status" and event[1] == "left" for event in events)


def test_decoder_start_policy_records_requested_effective_and_capability(monkeypatch) -> None:
    from Workstation import camera_stream_controller
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    def selection(requested):
        if requested == "auto":
            return SimpleNamespace(
                requested="auto",
                effective="nvh264dec",
                candidates=("nvh264dec", "avdec_h264"),
                allowed=True,
                degraded=False,
                reason="",
            )
        raise AssertionError(requested)

    monkeypatch.setattr(camera_stream_controller, "current_decoder_selection", selection)
    panel = SimpleNamespace(
        stream_backend_combo=SimpleNamespace(currentData=lambda: "gstreamer"),
        decoder_combo=SimpleNamespace(currentData=lambda: "auto"),
        _runtime_decoder_override=None,
        decoder_fallback_reasons={"left": "old", "right": "old"},
        _log=lambda _text: None,
        camera_connection_label=SimpleNamespace(setText=lambda _text: None),
    )

    assert CameraStreamLifecycleController(panel)._apply_decoder_start_policy() is True
    assert panel.decoder_selection_status == {
        "requested": "auto",
        "effective": "nvh264dec",
        "candidates": ("nvh264dec", "avdec_h264"),
        "allowed": True,
        "degraded": False,
        "reason": "",
        "nvidia_available": True,
    }
    assert panel.decoder_fallback_reasons == {"left": "", "right": ""}


def test_decoder_start_policy_preserves_runtime_fallback_reason(monkeypatch) -> None:
    from Workstation import camera_stream_controller
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    auto = SimpleNamespace(
        requested="auto",
        effective="nvh264dec",
        candidates=("nvh264dec", "avdec_h264"),
        allowed=True,
        degraded=False,
        reason="",
    )
    monkeypatch.setattr(camera_stream_controller, "current_decoder_selection", lambda _requested: auto)
    panel = SimpleNamespace(
        stream_backend_combo=SimpleNamespace(currentData=lambda: "gstreamer"),
        decoder_combo=SimpleNamespace(currentData=lambda: "auto"),
        _runtime_decoder_override="software",
        decoder_fallback_reasons={
            "left": "왼쪽: nvh264dec first-frame timeout",
            "right": "왼쪽: nvh264dec first-frame timeout",
        },
    )

    assert CameraStreamLifecycleController(panel)._apply_decoder_start_policy() is True
    assert panel.decoder_selection_status["effective"] == "avdec_h264"
    assert panel.decoder_selection_status["degraded"] is True
    assert "first-frame timeout" in panel.decoder_selection_status["reason"]
    assert panel.decoder_selection_status["nvidia_available"] is True


def test_decoder_mode_change_clears_process_local_fallback_evidence() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    panel = SimpleNamespace(
        _runtime_decoder_override="software",
        decoder_fallback_reasons={"left": "sentinel", "right": "sentinel"},
        decoder_combo=SimpleNamespace(currentData=lambda: "auto"),
        decoder_selection_status={"nvidia_available": True},
    )

    WirisStereoDistancePanel._handle_decoder_mode_changed(panel, 0)

    assert panel._runtime_decoder_override is None
    assert panel.decoder_fallback_reasons == {"left": "", "right": ""}
    assert panel.decoder_selection_status["requested"] == "auto"
    assert panel.decoder_selection_status["effective"] is None
