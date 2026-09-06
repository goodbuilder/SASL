"""Workstation settings regression tests."""

from __future__ import annotations

from Workstation.tests.workstation_test_support import (
    EvidenceAdapter,
    _FakeBackend,
    os,
    Path,
    pytest,
    QtWidgets,
    SimpleNamespace,
    WorkstationMainWindow,
)


def test_default_camera_host_address_preserves_camera_pixhawk_nic_isolation() -> None:
    from Workstation.workstation_core.settings import AppSettings

    settings = AppSettings()

    assert settings.camera_host_address == "10.0.0.11/32"
    assert settings.pixhawk_host_address == "10.0.0.10"
    assert settings.camera_sync_rtcp_pair_max_delta_s == pytest.approx(0.100)
    assert settings.pixhawk_transport_schema_version == 2
    assert (
        settings.left_pixhawk_transport,
        settings.right_pixhawk_transport,
        settings.drone_pixhawk_transport,
    ) == ("radio", "radio", "radio")
    assert settings.left_pixhawk_radio_device == ""
    assert settings.right_pixhawk_radio_device == ""
    assert settings.drone_pixhawk_radio_device == ""


def test_legacy_pixhawk_settings_migrate_once_to_radio_without_reusing_drone_serial(
    tmp_path: Path,
) -> None:
    import json

    from Workstation.workstation_core.settings import load_settings

    path = tmp_path / "legacy-pixhawk.json"
    legacy = {
        "left_mavlink_url": "udpin:0.0.0.0:14550",
        "right_mavlink_url": "udpin:0.0.0.0:14551",
        "drone_telemetry_url": "/dev/serial/by-id/p2-ground-radio",
        "left_mavlink_interface": "enx-left",
        "right_mavlink_interface": "enx-right",
        "mavlink_configure_streams": True,
    }
    path.write_text(json.dumps(legacy), encoding="utf-8")

    migrated = load_settings(path)
    persisted_once = path.read_bytes()
    reloaded = load_settings(path)

    assert migrated.pixhawk_transport_schema_version == 2
    assert migrated.left_pixhawk_transport == "radio"
    assert migrated.right_pixhawk_transport == "radio"
    assert migrated.drone_pixhawk_transport == "radio"
    assert migrated.left_mavlink_url == legacy["left_mavlink_url"]
    assert migrated.right_mavlink_url == legacy["right_mavlink_url"]
    assert migrated.drone_telemetry_url == legacy["drone_telemetry_url"]
    assert migrated.drone_pixhawk_radio_device == ""
    assert migrated.left_mavlink_interface == "enx-left"
    assert migrated.right_mavlink_interface == "enx-right"
    assert reloaded == migrated
    assert path.read_bytes() == persisted_once


def test_invalid_pixhawk_transport_fails_closed_to_radio(tmp_path: Path) -> None:
    from Workstation.workstation_core.settings import load_settings

    path = tmp_path / "invalid-pixhawk-transport.json"
    path.write_text(
        '{"pixhawk_transport_schema_version":1,"left_pixhawk_transport":"magic"}',
        encoding="utf-8",
    )

    assert load_settings(path).left_pixhawk_transport == "radio"


def test_stus_schedule_settings_migrate_to_manual_and_bound_interval(tmp_path: Path) -> None:
    from Workstation.workstation_core.settings import load_settings

    legacy = tmp_path / "legacy.json"
    legacy.write_text("{}", encoding="utf-8")
    migrated = load_settings(legacy)
    assert migrated.camera_sync_mode == "manual"
    assert migrated.camera_sync_interval_s == pytest.approx(1800.0)

    configured = tmp_path / "configured.json"
    configured.write_text(
        '{"camera_sync_mode":"AUTO","camera_sync_interval_s":1}',
        encoding="utf-8",
    )
    loaded = load_settings(configured)
    assert loaded.camera_sync_mode == "auto"
    assert loaded.camera_sync_interval_s == pytest.approx(300.0)


def test_legacy_live_time_basis_migrates_to_rtcp_preferred_with_app_fallback(
    tmp_path: Path,
) -> None:
    from Workstation.workstation_core.settings import load_settings

    path = tmp_path / "legacy-time-basis.json"
    path.write_text(
        '{"yolo_gate_matching_enabled":false,'
        '"yolo_gate_time_delta_enabled":false,'
        '"yolo_gate_max_time_delta_s":0.5,'
        '"yolo_gate_time_basis":"camera_rtc_required"}',
        encoding="utf-8",
    )

    settings = load_settings(path)

    # yolo_gate_matching_enabled/yolo_gate_time_delta_enabled are independent
    # persisted preferences, unrelated to the legacy time-basis migration
    # below -- the stored `false` values must round-trip unchanged.
    assert settings.yolo_gate_matching_enabled is False
    assert settings.yolo_gate_time_delta_enabled is False
    assert settings.yolo_gate_max_time_delta_s == pytest.approx(0.100)
    assert settings.yolo_gate_time_basis == "camera_rtcp"


def test_missing_or_invalid_live_time_basis_uses_app_receive_default(
    tmp_path: Path,
) -> None:
    from Workstation.workstation_core.settings import load_settings

    missing = tmp_path / "missing-time-basis.json"
    missing.write_text("{}", encoding="utf-8")
    invalid = tmp_path / "invalid-time-basis.json"
    invalid.write_text('{"yolo_gate_time_basis":"unknown"}', encoding="utf-8")

    assert load_settings(missing).yolo_gate_time_basis == "app_receive_monotonic"
    assert load_settings(invalid).yolo_gate_time_basis == "app_receive_monotonic"


def test_rtcp_preference_round_trips_through_persisted_app_settings(
    tmp_path: Path,
) -> None:
    from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings

    path = tmp_path / "settings.json"
    save_settings(AppSettings(yolo_gate_time_basis="camera_rtcp"), path)

    assert load_settings(path).yolo_gate_time_basis == "camera_rtcp"


def test_save_settings_failure_leaves_existing_settings_file_intact(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # save_settings() writes via a temp file + os.replace(); if the replace
    # step itself fails partway (disk full, killed process), the previously
    # saved settings file must stay exactly as it was -- never truncated or
    # half-written, since load_settings() has no error handling around
    # json.loads() and runs unconditionally at app startup.
    from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings

    path = tmp_path / "settings.json"
    save_settings(AppSettings(calibration_zoom_level=11), path)
    original_bytes = path.read_bytes()

    monkeypatch.setattr(os, "replace", lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("disk full")))

    with pytest.raises(OSError):
        save_settings(AppSettings(calibration_zoom_level=99), path)

    assert path.read_bytes() == original_bytes
    assert list(tmp_path.glob(".*.tmp")) == []
    assert load_settings(path).calibration_zoom_level == 11

def test_standalone_settings_wrappers_use_workstation_settings_path(monkeypatch: pytest.MonkeyPatch) -> None:
    from Workstation import standalone_gui
    from Workstation.runtime_paths import WORKSTATION_SETTINGS_PATH

    captured: dict[str, object] = {}
    sentinel = object()
    monkeypatch.setattr(
        standalone_gui,
        "_load_shared_settings",
        lambda path: captured.__setitem__("load", path) or sentinel,
    )
    monkeypatch.setattr(
        standalone_gui,
        "_save_shared_settings",
        lambda settings, path: captured.update(save=(settings, path)),
    )

    assert standalone_gui.load_settings() is sentinel
    standalone_gui.save_settings(sentinel)
    assert captured["load"] == WORKSTATION_SETTINGS_PATH
    assert captured["save"] == (sentinel, WORKSTATION_SETTINGS_PATH)

def test_create_session_folder_does_not_apply_recording_run_setup() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    package: dict[str, object] = {
        "state": "IDLE",
        "session_id": "",
        "measurement_id": "",
        "artifacts": [],
        "evidence_completeness": {"status": "PENDING", "complete": False},
    }
    session: dict[str, object] = {
        "state": "IDLE",
        "session_id": "",
        "measurements": [],
    }
    run_setup: dict[str, object] = {
        "state": "PENDING",
        "locked": False,
        "source": "no setup",
        "condition": {},
    }

    backend.measurement_package_snapshot = lambda: dict(package)
    backend.measurement_session_snapshot = lambda: dict(session)
    backend.run_setup_snapshot = lambda: dict(run_setup)

    def begin() -> bool:
        backend.calls.append("session.begin")
        session.update(state="OPEN", session_id="session-generated", measurements=[])
        package.update(session_id="session-generated", session_state="OPEN")
        run_setup.clear()
        run_setup.update(
            state="APPLIED",
            locked=False,
            source="automatic fallback",
            experiment_id="unassigned",
            run_id="session-generated",
            condition_id="unspecified",
            mode="drone",
            condition={"identity_source": "automatic_session_identity"},
        )
        return True

    def apply_run_setup(payload: dict[str, object]) -> dict[str, object]:
        values = dict(payload)
        backend.calls.append(("run_setup.apply", values))
        run_setup.clear()
        run_setup.update(
            **values,
            state="APPLIED",
            locked=False,
            source="operator draft accepted after session allocation",
        )
        return dict(run_setup)

    backend.begin_measurement_session = begin
    backend.apply_run_setup = apply_run_setup
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    backend.calls.clear()

    recording = window.recording
    assert not hasattr(recording, "run_setup_widgets")
    window.session.start_session_button.click()
    assert backend.calls == ["session.begin"]
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
    return
    recording.run_setup_widgets["experiment_id"].setText("EXP-START")
    recording.run_setup_widgets["run_id"].setText("RUN-START")
    recording.run_setup_widgets["condition_id"].setText("COND-START")
    recording.run_platform_combo.setCurrentIndex(
        recording.run_platform_combo.findData("uam")
    )
    recording.run_camera_serial_edits["left"].setText("CAM-L-001")
    recording.run_camera_serial_edits["right"].setText("CAM-R-002")
    recording.run_camera_stabilization_combos["left"].setCurrentIndex(
        recording.run_camera_stabilization_combos["left"].findData("disabled")
    )
    recording.run_camera_stabilization_combos["right"].setCurrentIndex(
        recording.run_camera_stabilization_combos["right"].findData("enabled")
    )

    assert recording.run_setup_widgets["experiment_id"].isEnabled()
    assert window.session.start_session_button.isEnabled()
    window.session.start_session_button.click()

    expected = {
        "experiment_id": "EXP-START",
        "run_id": "RUN-START",
        "condition_id": "COND-START",
        "target_platform": "uam",
        "condition": {
            "left_serial": "CAM-L-001",
            "right_serial": "CAM-R-002",
            "left_stabilization_state": "disabled",
            "right_stabilization_state": "enabled",
            "camera_identity_source": "operator_verified_run_setup",
        },
    }
    assert backend.calls == ["session.begin"]
    assert recording.run_setup_payload() == expected
    assert recording.run_setup_widgets["experiment_id"].isEnabled()

    run_setup.update(state="LOCKED", locked=True, source="first evidence")
    recording.refresh_run_setup_from_owner()
    assert not recording.run_setup_widgets["experiment_id"].isEnabled()

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_start_session_allows_partial_run_setup_without_blocking_session() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    calls: list[str] = []
    backend.measurement_package_snapshot = lambda: {
        "state": "IDLE",
        "artifacts": [],
        "evidence_completeness": {"status": "PENDING", "complete": False},
    }
    backend.measurement_session_snapshot = lambda: {
        "state": "IDLE",
        "session_id": "",
        "measurements": [],
    }
    backend.run_setup_snapshot = lambda: {
        "state": "PENDING",
        "locked": False,
        "condition": {},
    }
    backend.begin_measurement_session = lambda: calls.append("begin") or True
    backend.apply_run_setup = lambda _payload: calls.append("apply") or True
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    assert not hasattr(window.recording, "run_setup_widgets")
    window.session.start_session_button.click()
    assert calls == ["begin"]
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
    return
    window.recording.run_setup_widgets["experiment_id"].setText("EXP-only")

    window.session.start_session_button.click()

    assert calls == ["begin"]
    assert "ERROR" not in window.session.session_status_label.text()

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_workstation_shutdown_orders_runtime_stop_before_settings_save(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui, "APP_ROOT", tmp_path)
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
    panel = standalone_gui.WirisStereoDistancePanel()
    assert panel.app_root == tmp_path.resolve()
    assert panel.settings_path == (tmp_path / "stereo_distance_settings.json").resolve()
    assert tuple(step.name for step in panel._shutdown_coordinator.steps) == (
        "stop-target-drone",
        "stop-gnss-network",
        "stop-gnss-base-serial",
        "flush-stop-rtcm-pipeline",
        "stop-drone-telemetry",
        "stop-runtime-workers",
        "stop-auxiliary-workers",
        "wait-session-save",
        "verify-resource-cleanup",
        "save-settings",
        "close-measurement-data-writer",
    )
    panel.close()
    app.processEvents()
    assert panel.settings_path.is_file()
    panel.deleteLater()
    app.processEvents()

def test_workstation_exposes_complete_system_settings_surface() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.navigation.select_context("tasks.settings")
    app.processEvents()
    settings = window.settings_workspace
    assert window.workspace_host.currentWidget().widget() is settings
    assert settings.tabs.count() == 7
    assert settings.settings_widgets["left_pixhawk_transport"].itemText(0) == "P1 Telemetry Radio"
    assert settings.settings_widgets["right_pixhawk_transport"].itemText(0) == "P2 Telemetry Radio"
    assert settings.settings_widgets["drone_pixhawk_transport"].itemText(0) == "Drone Telemetry Radio"
    assert not any(
        "Units/frames" in label.text()
        for label in settings.findChildren(QtWidgets.QLabel)
    )
    assert {
        "interface_name",
        "left_rtsp_url",
        "yolo_weights_path",
        "yolo_input_size",
        "rtsp_transport",
        "rtsp_encoded_queue_mode",
        "pixhawk_host_address",
        "pixhawk_ethernet_mode",
        "intrinsics_path",
        "measurement_data_root",
        "camera_sync_mode",
        "camera_sync_interval_s",
        "gnss_operation_mode",
        "gnss_network_caster_host",
        "gnss_network_tls",
        "gnss_readiness_maximum_baseline_std_m",
        "gnss_fixed_coordinate_profile_path",
    } <= set(settings.settings_widgets)
    assert {
        "gnss_require_drone_rtk_fixed",
        "gnss_route_left_enabled",
        "gnss_route_right_enabled",
        "gnss_route_drone_enabled",
    } <= set(settings.settings_widgets)
    assert "gnss_route_field_base_enabled" not in settings.settings_widgets
    assert not {
        "ros_domain_id",
        "left_namespace",
        "right_namespace",
        "mavlink_configure_uxrce_agent",
        "uxrce_port",
        "clock_ftp_password",
    } & set(settings.settings_widgets)
    assert settings.settings_widgets["gnss_readiness_minimum_fixed_duration_s"].value() == pytest.approx(30.0)
    assert settings.settings_widgets["gnss_readiness_maximum_baseline_std_m"].value() == pytest.approx(0.02)
    assert settings.settings_widgets["gnss_network_caster_host"].text() == ""
    assert settings.settings_widgets["yolo_input_size"].value() == 416
    assert "next Detection start" in settings.settings_widgets["yolo_input_size"].toolTip()
    assert settings.settings_widgets["rtsp_encoded_queue_mode"].currentData() == "latest"
    measurement_root = settings.settings_widgets["measurement_data_root"]
    assert measurement_root.text() == ""
    assert "recordings" in measurement_root.placeholderText()
    assert settings.settings_payload()["measurement_data_root"] == ""
    settings.settings_widgets["rtsp_encoded_queue_mode"].setCurrentIndex(
        settings.settings_widgets["rtsp_encoded_queue_mode"].findData("reference_safe")
    )
    assert settings.settings_payload()["rtsp_encoded_queue_mode"] == "reference_safe"
    assert "hostname only" in settings.settings_widgets["gnss_network_caster_host"].placeholderText()
    assert "측정 데이터의 현장 메모" in settings.settings_widgets["gnss_site_label"].toolTip()
    assert "기록용" in settings.settings_widgets["gnss_site_address"].toolTip()

    operation_mode = settings.settings_widgets["gnss_operation_mode"]
    assert operation_mode.currentData() == "LOCAL_SURVEY_IN"
    assert settings.settings_widgets["gnss_base_serial_port"].isEnabled()
    assert settings.settings_widgets["gnss_base_survey_in_min_duration_s"].isEnabled()
    assert not settings.settings_widgets["gnss_network_caster_host"].isEnabled()
    assert not settings.settings_widgets["gnss_fixed_coordinate_profile_path"].isEnabled()

    operation_mode.setCurrentIndex(operation_mode.findData("NETWORK_RTK_DIRECT"))
    assert not settings.settings_widgets["gnss_base_serial_port"].isEnabled()
    assert not settings.settings_widgets["gnss_base_survey_in_min_duration_s"].isEnabled()
    assert settings.settings_widgets["gnss_network_caster_host"].isEnabled()
    assert not settings.settings_widgets["gnss_fixed_coordinate_profile_path"].isEnabled()

    operation_mode.setCurrentIndex(operation_mode.findData("NETWORK_ASSISTED_FIXED_BASE"))
    assert settings.settings_widgets["gnss_base_serial_port"].isEnabled()
    assert not settings.settings_widgets["gnss_base_survey_in_min_duration_s"].isEnabled()
    assert settings.settings_widgets["gnss_network_caster_host"].isEnabled()
    assert settings.settings_widgets["gnss_fixed_coordinate_profile_path"].isEnabled()

    operation_mode.setCurrentIndex(operation_mode.findData("LOCAL_SURVEY_IN"))

    settings.settings_widgets["interface_name"].setText("enx-workstation")
    settings.settings_widgets["yolo_input_size"].setValue(736)
    settings.settings_widgets["gnss_network_caster_host"].setText("caster.example.org")
    settings.settings_widgets["gnss_network_caster_port"].setValue(2201)
    settings.settings_widgets["gnss_network_mountpoint"].setText("VRS_RTCM32")
    settings.settings_widgets["gnss_network_username_env"].setText("EXAMPLE_NTRIP_USER")
    settings.settings_widgets["gnss_network_password_env"].setText("EXAMPLE_NTRIP_PASSWORD")
    settings.settings_widgets["gnss_network_tls"].setChecked(True)
    settings.settings_widgets["gnss_readiness_maximum_baseline_std_m"].setValue(0.1234)
    settings.settings_widgets["gnss_fixed_coordinate_profile_path"].setText("gnss/base.json")
    settings.settings_widgets["gnss_route_left_enabled"].setChecked(False)
    settings.settings_widgets["gnss_route_right_enabled"].setChecked(True)
    settings.settings_widgets["gnss_route_drone_enabled"].setChecked(True)
    settings.settings_widgets["gnss_require_drone_rtk_fixed"].setChecked(True)
    reconnect_policy = settings.settings_widgets["rtsp_reconnect_policy"]
    reconnect_policy.setCurrentIndex(reconnect_policy.findData("automatic"))
    payload = settings.settings_payload()
    assert payload["interface_name"] == "enx-workstation"
    assert payload["yolo_input_size"] == 736
    assert "rtsp_pipeline_profile" not in payload
    assert payload["rtsp_reconnect_policy"] == "automatic"
    assert payload["gnss_operation"]["require_drone_rtk_fixed"] is True
    assert payload["gnss_operation"]["routing"]["left_enabled"] is False
    assert payload["gnss_operation"]["routing"]["right_enabled"] is True
    assert payload["gnss_operation"]["routing"]["drone_enabled"] is True
    assert payload["gnss_operation"]["routing"]["field_base_enabled"] is False
    assert payload["gnss_base_rtcm_injection_target"] == "right"
    assert payload["gnss_operation"]["network"]["caster_host"] == "caster.example.org"
    assert payload["gnss_operation"]["network"]["caster_port"] == 2201
    assert payload["gnss_operation"]["network"]["mountpoint"] == "VRS_RTCM32"
    assert payload["gnss_operation"]["network"]["username_env"] == "EXAMPLE_NTRIP_USER"
    assert payload["gnss_operation"]["network"]["password_env"] == "EXAMPLE_NTRIP_PASSWORD"
    assert payload["gnss_operation"]["network"]["tls"] is True
    assert payload["gnss_operation"]["readiness"]["maximum_baseline_std_m"] == pytest.approx(0.1234)
    assert payload["gnss_operation"]["fixed_base"]["coordinate_profile_path"] == "gnss/base.json"

    assert window.commands.invoke("settings.apply") is True
    assert backend.calls[-1][0] == "settings_apply"
    assert window.commands.invoke("settings.save") is True
    # The untouched masked deployment fallback is presentation state, not an
    # explicit password edit.  Do not hand it to the runtime owner where it
    # would shadow a saved keyring credential after restart.
    assert not any(
        call[0] == "set_gnss_ntrip_session_password"
        for call in backend.calls
        if isinstance(call, tuple)
    )
    assert backend.calls[-2][0] == "settings_apply"
    assert backend.calls[-2][1]["interface_name"] == "enx-workstation"
    assert backend.calls[-2][1]["yolo_input_size"] == 736
    assert "rtsp_pipeline_profile" not in backend.calls[-2][1]
    assert backend.calls[-2][1]["rtsp_reconnect_policy"] == "automatic"
    assert backend.calls[-1] == "settings_save"
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_yolo_weights_file_picker_imports_external_model_into_owned_storage(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import hashlib

    from Workstation.settings_workspace import SettingsWorkspace

    data_root = tmp_path / "workstation-data"
    source = tmp_path / "external" / "custom-drone.pt"
    source.parent.mkdir()
    source_bytes = b"independent-yolo-model-sentinel"
    source.write_bytes(source_bytes)
    backend = SimpleNamespace(panel=SimpleNamespace(app_root=data_root))
    workspace = SettingsWorkspace(backend=backend)
    monkeypatch.setattr(
        QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args, **_kwargs: (str(source), "Model weights (*.pt)"),
    )

    workspace.yolo_weights_browse_button.click()

    digest = hashlib.sha256(source_bytes).hexdigest()
    expected_relative = f"models/imported/{digest}/custom-drone.pt"
    imported = data_root / expected_relative
    assert workspace.settings_widgets["yolo_weights_path"].text() == expected_relative
    assert workspace.settings_payload()["yolo_weights_path"] == expected_relative
    assert imported.read_bytes() == source_bytes
    assert "SHA-256 검증 완료" in workspace.yolo_weights_selection_status.text()

    monkeypatch.setattr(
        QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args, **_kwargs: (str(imported), "Model weights (*.pt)"),
    )
    workspace.yolo_weights_browse_button.click()
    assert workspace.settings_widgets["yolo_weights_path"].text() == expected_relative
    assert workspace.yolo_weights_selection_status.text().startswith("모델 선택 완료")

    workspace.close()
    workspace.deleteLater()
    QtWidgets.QApplication.processEvents()


def test_yolo_weights_file_picker_failure_preserves_previous_selection(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.settings_workspace import SettingsWorkspace

    backend = SimpleNamespace(panel=SimpleNamespace(app_root=tmp_path / "workstation-data"))
    workspace = SettingsWorkspace(backend=backend)
    weights_edit = workspace.settings_widgets["yolo_weights_path"]
    weights_edit.setText("models/approved.pt")
    missing = tmp_path / "missing-model.pt"
    monkeypatch.setattr(
        QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args, **_kwargs: (str(missing), "Model weights (*.pt)"),
    )

    workspace.yolo_weights_browse_button.click()

    assert weights_edit.text() == "models/approved.pt"
    assert "모델 가져오기 실패" in workspace.yolo_weights_selection_status.text()

    workspace.close()
    workspace.deleteLater()
    QtWidgets.QApplication.processEvents()


def test_visible_encoder_settings_are_labeled_manual_reference_only(tmp_path: Path) -> None:
    from Workstation.settings_workspace import SettingsWorkspace

    backend = SimpleNamespace(panel=SimpleNamespace(app_root=tmp_path / "workstation-data"))
    workspace = SettingsWorkspace(backend=backend)

    for key in (
        "wiris_visible_encoder_bitrate",
        "wiris_visible_encoder_qp",
        "wiris_visible_encoder_gop",
        "wiris_visible_encoder_idr",
    ):
        tooltip = workspace.settings_widgets[key].toolTip()
        assert "Manual-reference" in tooltip
        assert "do not apply it" in tooltip

    workspace.close()
    workspace.deleteLater()
    QtWidgets.QApplication.processEvents()


def test_workstation_every_flat_app_setting_has_an_editable_surface() -> None:
    from dataclasses import fields

    from Workstation.workstation_core.settings import AppSettings

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    system_keys = set(window.settings_workspace.settings_widgets)
    measurement_keys = set(window.measurement.settings_widgets)
    time_keys = set(window.time_workspace.settings_widgets)
    calibration_keys = set(window.calibration.setup_widgets)
    measurement_aliases = {
        "yolo_conf_threshold": "yolo_confidence",
        "yolo_iou_threshold": "yolo_iou",
        "yolo_gate_class_enabled": "class_gate_enabled",
        "yolo_gate_class_text": "class_gate",
        "yolo_gate_matching_enabled": "matching_gate_enabled",
        "yolo_gate_time_delta_enabled": "time_delta_gate_enabled",
        "yolo_gate_vertical_delta_enabled": "vertical_delta_gate_enabled",
        "yolo_gate_max_vertical_delta_px": "max_vertical_delta_px",
        "yolo_gate_max_time_delta_s": "max_pair_delta_s",
        "yolo_gate_time_basis": "time_basis",
    }
    sync_aliases = {
        "camera_sync_history_window_s": "camera_sync_history_window_s",
        "camera_sync_receive_pair_max_delta_s": "camera_sync_receive_pair_max_delta_ms",
        "camera_sync_rtcp_pair_max_delta_s": "camera_sync_rtcp_pair_max_delta_ms",
        "camera_sync_max_sr_age_s": "camera_sync_max_sr_age_s",
        "camera_sync_max_pc_offset_s": "camera_sync_max_pc_offset_ms",
        "camera_sync_max_clock_rate_error": "camera_sync_clock_rate_error_pct",
        "camera_sync_required_sr_count": "camera_sync_required_sr_count",
        "camera_sync_warmup_timeout_s": "camera_sync_warmup_timeout_s",
    }
    calibration_aliases = {
        "calibration_inner_cols": "inner_cols",
        "calibration_inner_rows": "inner_rows",
        "calibration_square_size_mm": "square_size_mm",
        "calibration_min_images": "min_images",
        "calibration_calculation_pair_count": "calculation_pair_count",
        "calibration_pair_max_delta_s": "pair_max_delta_ms",
        "calibration_auto_interval_s": "auto_interval_s",
        "calibration_root": "calibration_root",
        "calibration_validation_mode": "validation_mode",
    }
    retired_calibration_settings = {
        "calibration_zoom_level",
        "calibration_validation_root",
        "calibration_validation_json",
        "calibration_validation_runs",
        "calibration_validation_pair_count",
        "calibration_validation_seed",
    }
    compatibility_only_settings = {
        # The visible GNSS surface now exposes independent P1/P2 route
        # toggles; this flat target remains only for older settings files and
        # the owner bridge.
        "gnss_base_rtcm_injection_target",
        # The visible WIRIS FTP surface now exposes independent left/right
        # camera credentials; these shared flat targets remain only as
        # fallback defaults for older settings files.
        "wiris_ftp_port",
        "wiris_ftp_user",
        "wiris_ftp_password",
        "wiris_ftp_remote_dir",
        "wiris_ftp_timeout_s",
    }
    custom_measurement_method_settings = {
        "triangulation_extrinsics_source",
        "measurement_preferred_method",
        "measurement_method_fallback_order",
        "measurement_baseline_yaw_reverse",
        "pixhawk_left_gnss_source",
        "pixhawk_right_gnss_source",
    }
    pixhawk_workspace_owned_settings = {
        "pixhawk_transport_schema_version",
        # These machine-local values are selected by Find network + Assign
        # P1/P2 in the Pixhawk workspace.  Free-form Settings edits would
        # bypass the route evidence bound to that operator selection.
        "left_mavlink_interface",
        "right_mavlink_interface",
        "drone_mavlink_interface",
        "left_mavlink_local_address",
        "right_mavlink_local_address",
        "drone_mavlink_local_address",
        # Stable physical identity and exact app-owned NM entries are
        # produced only by the confirmed role-application workflow.
        "camera_network_stable_id",
        "left_mavlink_interface_stable_id",
        "right_mavlink_interface_stable_id",
        "drone_mavlink_interface_stable_id",
        "left_pixhawk_radio_usb_serial",
        "right_pixhawk_radio_usb_serial",
        "drone_pixhawk_radio_usb_serial",
        "network_role_owned_entries",
    }
    flat_fields = {field.name for field in fields(AppSettings)} - {"gnss_operation"}

    assert flat_fields - system_keys == (
        set(measurement_aliases)
        | set(sync_aliases)
        | set(calibration_aliases)
        | retired_calibration_settings
        | compatibility_only_settings
        | custom_measurement_method_settings
        | pixhawk_workspace_owned_settings
    )
    assert set(measurement_aliases.values()) <= measurement_keys
    assert {
        "pixhawk_left_gnss_source",
        "pixhawk_right_gnss_source",
    } <= measurement_keys
    assert set(sync_aliases.values()) <= time_keys
    assert set(calibration_aliases.values()) <= calibration_keys
    assert window.settings_workspace.measurement_fallback_list.count() == 9
    assert "zoom_level" not in calibration_keys
    assert window.advanced.zoom_level_spin.value() > 0
    assert not set(sync_aliases.values()) & measurement_keys

    window.close()
    window.deleteLater()
    app.processEvents()


def test_measurement_workspace_exposes_app_default_and_rtcp_preference(
    workstation_qapplication,
) -> None:
    window = WorkstationMainWindow(backend=_FakeBackend())
    basis = window.measurement.settings_widgets["time_basis"]
    matching = window.measurement.settings_widgets["matching_gate_enabled"]
    delta_gate = window.measurement.settings_widgets["time_delta_gate_enabled"]
    delta = window.measurement.settings_widgets["max_pair_delta_s"]

    assert basis.currentData() == "app_receive_monotonic"
    assert basis.isEnabled() is True
    assert basis.count() == 2
    assert basis.itemData(1) == "camera_rtcp"
    assert matching.isChecked() is True
    assert matching.isEnabled() is False
    assert delta_gate.isChecked() is True
    assert delta_gate.isEnabled() is False
    assert delta.maximum() == pytest.approx(0.100)
    assert "Settings preferred" in window.measurement.distance_timing_contract.text()
    assert "fallback APP RECEIVE" in window.measurement.distance_timing_contract.text()

    window.close()
    window.deleteLater()
    workstation_qapplication.processEvents()


def test_measurement_timing_status_projects_effective_fallback_without_relayout(
    workstation_qapplication,
) -> None:
    from Workstation.workstation_core.detection_pair_coordinator import PairTimingDecision

    window = WorkstationMainWindow(backend=_FakeBackend())
    panel = SimpleNamespace(
        _yolo_time_basis=lambda: "camera_rtcp",
        _coordinated_pairing_result=SimpleNamespace(
            timing=PairTimingDecision(
                requested_basis="camera_rtcp",
                effective_basis="app_receive_monotonic",
                left_timestamp_s=10.0,
                right_timestamp_s=10.02,
                signed_delta_ns=20_000_000,
                abs_delta_ns=20_000_000,
                fallback_reason="CLOCK_NOT_VERIFIED",
            )
        ),
    )

    window.measurement._refresh_distance_timing_contract(panel)

    text = window.measurement.distance_timing_contract.text()
    assert "RTCP/NTP → APP RECEIVE" in text
    assert "fallback=CLOCK_NOT_VERIFIED" in text
    assert (
        window.measurement.distance_timing_contract.sizePolicy().horizontalPolicy()
        == QtWidgets.QSizePolicy.Ignored
    )

    window.close()
    window.deleteLater()
    workstation_qapplication.processEvents()

def test_evidence_projects_measurement_settings_and_session_csv_without_conversion(tmp_path, monkeypatch) -> None:
    from Workstation.workstation_core import settings as settings_module

    panel = SimpleNamespace(
        settings=SimpleNamespace(
            manual_focal_override=False,
            triangulation_extrinsics_source="pixhawk",
            yolo_input_size=640,
            yolo_conf_threshold=0.6,
            yolo_iou_threshold=0.4,
            yolo_gate_class_enabled=True,
            yolo_gate_class_text="person",
            yolo_gate_matching_enabled=True,
            yolo_gate_time_delta_enabled=True,
            yolo_gate_vertical_delta_enabled=False,
            yolo_gate_max_time_delta_s=0.08,
            yolo_gate_time_basis="camera_rtcp",
            yolo_gate_max_vertical_delta_px=50.0,
            camera_sync_history_window_s=1.0,
            camera_sync_receive_pair_max_delta_s=0.3,
            camera_sync_rtcp_pair_max_delta_s=0.05,
            camera_sync_max_sr_age_s=7.0,
            camera_sync_max_pc_offset_s=0.1,
            camera_sync_max_clock_rate_error=0.005,
            camera_sync_required_sr_count=2,
            camera_sync_warmup_timeout_s=30.0,
        )
    )
    evidence = EvidenceAdapter(panel)
    measurement = evidence.measurement_settings()
    assert measurement["yolo_input_size_px"] == 640
    assert measurement["class_gate"] == "person"
    assert measurement["distance_mode"] == "pixhawk"
    assert measurement["time_basis"] == "camera_rtcp"
    assert measurement["time_basis_role"] == "preferred_with_app_receive_fallback"
    assert measurement["ntp_evidence_role"] == "preferred_when_selected_and_verified"
    assert measurement["matching_gate_enabled"] is True
    assert measurement["time_delta_gate_enabled"] is True
    assert measurement["camera_sync_rtcp_pair_max_delta_ms"] == 50.0
    assert "bbox_gate_enabled" not in measurement
    assert "min_bbox_width_px" not in measurement
    assert "min_bbox_height_px" not in measurement
    assert "min_bbox_area_px2" not in measurement
    assert "baseline_center_frd" in str(measurement["coordinate_contract"])

    monkeypatch.setattr(settings_module, "APP_ROOT", tmp_path)
    session_dir = tmp_path / "recordings" / "sessions" / "session-1"
    generation_dir = session_dir / ".generations" / "generation-1"
    generation_dir.mkdir(parents=True)
    csv_path = generation_dir / "session-1_distance_measurements.csv"
    csv_path.write_text("recording_time_ns,forward_m,right_m,down_m\n1,2,3,4\n", encoding="utf-8")
    (session_dir / "published").symlink_to(Path(".generations") / "generation-1")
    (session_dir / "session_manifest.json").write_text(
        '{"status":"finalized","generation_id":"generation-1",'
        '"time_axis":"recording_time_ns",'
        '"artifact_roles":{"session-1_distance_measurements.csv":"distance_measurements_csv"}}',
        encoding="utf-8",
    )
    catalog = evidence.session_catalog()
    assert catalog[0]["session_id"] == "session-1"
    assert "session-1_distance_measurements.csv" in catalog[0]["artifacts"]
    preview = evidence.session_artifact_rows("session-1", "session-1_distance_measurements.csv")
    assert preview["columns"] == ("recording_time_ns", "forward_m", "right_m", "down_m")
    assert preview["rows"][0]["forward_m"] == "2"
    assert "no NED/ENU" in str(preview["coordinate_contract"])

def test_system_measurement_and_time_save_apply_visible_payload_before_persist() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()

    window.measurement.settings_widgets["yolo_confidence"].setValue(0.731)
    backend.calls.clear()
    assert window.commands.invoke("measurement.save_settings") is True
    assert backend.calls[0][0] == "measurement_apply_settings"
    assert backend.calls[0][1]["yolo_confidence"] == pytest.approx(0.731)
    assert "focal_px" not in backend.calls[0][1]
    assert "yolo_input_size_px" not in backend.calls[0][1]
    assert backend.calls[1] == "measurement_save_settings"

    window.time_workspace.settings_widgets["camera_sync_history_window_s"].setValue(4.5)
    backend.calls.clear()
    assert window.commands.invoke("time.save_settings") is True
    assert backend.calls[0][0] == "measurement_apply_settings"
    assert backend.calls[0][1]["camera_sync_history_window_s"] == pytest.approx(4.5)
    assert backend.calls[1] == "measurement_save_settings"

    window.settings_workspace.settings_widgets["interface_name"].setText("enx-visible")
    window.settings_workspace.settings_widgets["yolo_input_size"].setValue(960)
    window.settings_workspace.gnss_network_password_edit.setText("session-only-secret")
    backend.calls.clear()
    assert window.commands.invoke("settings.save") is True
    assert backend.calls[0][:2] == (
        "set_gnss_ntrip_session_password",
        "session-only-secret",
    )
    assert backend.calls[1][0] == "settings_apply"
    applied_network = backend.calls[1][1]["gnss_operation"]["network"]
    assert backend.calls[0][2:] == (
        applied_network["caster_host"],
        applied_network["mountpoint"],
        applied_network["username"],
    )
    assert backend.calls[1][1]["interface_name"] == "enx-visible"
    assert backend.calls[1][1]["yolo_input_size"] == 960
    assert backend.calls[2] == "settings_save"

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_measurement_start_applies_selected_yolo_weights_before_detection() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    selected_weights = "models/yolov8n-military-student.pt"
    window.settings_workspace.settings_widgets["yolo_weights_path"].setText(
        selected_weights
    )
    backend.calls.clear()

    assert window.commands.invoke("detection.toggle") is True

    assert backend.calls == [
        ("settings_apply", {"yolo_weights_path": selected_weights}),
        "detection",
    ]

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_measurement_start_rejection_does_not_fall_back_to_stale_yolo_weights() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()

    def reject_settings(settings: dict[str, object]) -> bool:
        backend.calls.append(("settings_apply", dict(settings)))
        return False

    backend.apply_workstation_system_settings = reject_settings
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    selected_weights = "models/yolov8m-defence.pt"
    window.settings_workspace.settings_widgets["yolo_weights_path"].setText(
        selected_weights
    )
    backend.calls.clear()

    assert window.commands.invoke("detection.toggle") is False
    assert backend.calls == [
        ("settings_apply", {"yolo_weights_path": selected_weights})
    ]

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_measurement_stop_does_not_require_a_new_yolo_model_selection() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel._measurement_run_state = "RUNNING"
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.settings_workspace.settings_widgets["yolo_weights_path"].clear()
    backend.calls.clear()

    assert window.commands.invoke("detection.toggle") is True
    assert backend.calls == ["detection"]

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_measurement_start_forwards_selected_weights_to_real_yolo_config(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import standalone_gui
    from Workstation.standalone_backend import StandalonePanelBackend

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
    captured_configs: list[dict[str, object]] = []
    panel.toggle_yolo_detection = lambda: captured_configs.append(
        panel.yolo_kwargs()
    )
    window = WorkstationMainWindow(backend=StandalonePanelBackend(panel))
    window.status_adapter.stop()
    selected_weights = "models/yolov8m-defence.pt"
    window.settings_workspace.settings_widgets["yolo_weights_path"].setText(
        selected_weights
    )

    assert window.commands.invoke("detection.toggle") is True

    assert panel.weights_edit.text() == selected_weights
    assert len(captured_configs) == 1
    assert str(captured_configs[0]["weights_path"]).endswith(selected_weights)

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_settings_save_does_not_persist_when_visible_payload_apply_is_rejected() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    persisted: list[bool] = []
    backend.apply_measurement_settings = lambda _payload: False
    backend.save_measurement_settings = lambda: persisted.append(True)
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    failures: list[tuple[str, str]] = []
    window.commands.failed.connect(
        lambda command_id, message: failures.append((command_id, message))
    )

    assert window.commands.invoke("measurement.save_settings") is False
    assert persisted == []
    assert failures[-1] == (
        "measurement.save_settings",
        "measurement settings apply was rejected",
    )

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_settings_save_propagates_persistence_rejection() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    apply_calls: list[dict[str, object]] = []
    backend.apply_measurement_settings = lambda payload: apply_calls.append(dict(payload))
    backend.save_measurement_settings = lambda: False
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    failures: list[tuple[str, str]] = []
    window.commands.failed.connect(
        lambda command_id, message: failures.append((command_id, message))
    )

    assert window.commands.invoke("time.save_settings") is False
    assert len(apply_calls) == 1
    assert failures[-1] == ("time.save_settings", "handler rejected command")

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
