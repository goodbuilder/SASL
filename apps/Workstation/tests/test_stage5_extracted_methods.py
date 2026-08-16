"""Before/after behavior checks for the methods moved off `WirisStereoDistancePanel`
in the Stage 5 God-Object cleanup (apply_selected_pixhawk_candidate ->
DeviceRuntimeController, _recover_retryable_recording_session ->
RecordingController, apply_wiris_stip -> WirisControlController, and the
settings apply/snapshot surface -> WorkstationSettingsController).

Each test follows the codebase's existing convention of exercising the
extracted logic against a lightweight `SimpleNamespace` stand-in for the
panel, matching the pattern already used throughout `test_workstation.py`
and `test_cross_zoom_calibration.py`.
"""

from __future__ import annotations

from types import SimpleNamespace

import Workstation.recording_controller as recording_controller_module
from Workstation.device_runtime_controller import DeviceRuntimeController
from Workstation.recording_controller import RecordingController
from Workstation.settings_controller import WorkstationSettingsController
from Workstation.wiris_control_controller import WirisControlController
from Workstation.workstation_core.recording_session import RetryableSession, SpoolRows


class _ValueWidget:
    def __init__(self, value=None):
        self.value_state = value

    def value(self):
        return self.value_state

    def setValue(self, value):
        self.value_state = value

    def setChecked(self, value):
        self.value_state = bool(value)

    def setText(self, value):
        self.value_state = str(value)

    def text(self):
        return str(self.value_state)

    def isChecked(self):
        return bool(self.value_state)

    def setEnabled(self, value):
        self.value_state = bool(value)


def test_apply_selected_pixhawk_candidate_applies_pc_ip_and_saves() -> None:
    logs = []
    saved = []
    panel = SimpleNamespace(
        pixhawk_host_address_edit=_ValueWidget("old-host"),
        pixhawk_connection_label=_ValueWidget(),
        _selected_pixhawk_candidate=lambda source: {
            "kind": "PC_IP",
            "value": "10.0.0.5",
            "interface": "eth0",
        },
        _log=lambda message: logs.append(message),
        _save_current_settings=lambda: saved.append(True),
        _refresh_operation_header=lambda: None,
    )

    DeviceRuntimeController(panel).apply_selected_pixhawk_candidate("host")

    assert panel.pixhawk_host_address_edit.text() == "10.0.0.5"
    assert saved == [True]
    assert any("Pixhawk Host IP" in message for message in logs)


def test_recover_retryable_recording_session_returns_false_without_candidates(tmp_path) -> None:
    panel = SimpleNamespace(app_root=tmp_path)
    original = recording_controller_module.discover_retryable_sessions
    recording_controller_module.discover_retryable_sessions = lambda root: []
    try:
        result = RecordingController(panel)._recover_retryable_recording_session()
    finally:
        recording_controller_module.discover_retryable_sessions = original

    assert result is False


def test_recover_retryable_recording_session_restores_hr_oz_capture_records(tmp_path) -> None:
    # event_index/capture_id for the *next* HR/OZ capture are derived from
    # len(panel.pnp_photo_capture_records)/len(panel.oz_photo_capture_records);
    # if recovery leaves those empty despite hr_captures.jsonl/oz_captures.jsonl
    # already holding events on disk, the next capture restarts at event_index=1
    # and duplicates an id already present in that session's JSONL file.
    output_dir = tmp_path / "recovered-session"
    output_dir.mkdir()
    (output_dir / "hr_captures.jsonl").write_text(
        '{"capture_id": "hr-001", "event_index": 1}\n'
        '{"capture_id": "hr-002", "event_index": 2}\n',
        encoding="utf-8",
    )
    (output_dir / "oz_captures.jsonl").write_text(
        '{"capture_id": "oz-001", "event_index": 1}\n',
        encoding="utf-8",
    )
    recovered = RetryableSession(
        output_dir=output_dir,
        metadata={"session_id": "recovered-1", "session_started_unix_ns": 1_000_000_000},
        pair_rows=SpoolRows(),
        camera_rows={"left": SpoolRows(), "right": SpoolRows()},
        px4_rows={"left": SpoolRows(), "right": SpoolRows()},
        gnss_base_rows=SpoolRows(),
        extrinsics_comparison_rows=SpoolRows(),
    )
    panel = SimpleNamespace(
        app_root=tmp_path,
        record_button=_ValueWidget(),
        save_history_button=_ValueWidget(),
        recording_status_label=_ValueWidget(),
        _log=lambda message: None,
        _refresh_recording_table=lambda: None,
    )
    original_discover = recording_controller_module.discover_retryable_sessions
    original_load = recording_controller_module.load_retryable_session
    recording_controller_module.discover_retryable_sessions = lambda root: [output_dir]
    recording_controller_module.load_retryable_session = lambda candidate: recovered
    try:
        RecordingController(panel)._recover_retryable_recording_session()
    finally:
        recording_controller_module.discover_retryable_sessions = original_discover
        recording_controller_module.load_retryable_session = original_load

    assert [row["capture_id"] for row in panel.pnp_photo_capture_records] == ["hr-001", "hr-002"]
    assert [row["capture_id"] for row in panel.oz_photo_capture_records] == ["oz-001"]
    # The next capture's event_index/capture_id (len(records) + 1) now
    # continues from disk instead of colliding with "hr-001"/event_index=1.
    assert len(panel.pnp_photo_capture_records) + 1 == 3
    assert len(panel.oz_photo_capture_records) + 1 == 2


def test_recover_retryable_recording_session_restores_spool_state(tmp_path) -> None:
    output_dir = tmp_path / "recovered-session"
    output_dir.mkdir()
    recovered = RetryableSession(
        output_dir=output_dir,
        metadata={"session_id": "recovered-1", "session_started_unix_ns": 1_000_000_000},
        pair_rows=SpoolRows(),
        camera_rows={"left": SpoolRows(), "right": SpoolRows()},
        px4_rows={"left": SpoolRows(), "right": SpoolRows()},
        gnss_base_rows=SpoolRows(),
        extrinsics_comparison_rows=SpoolRows(),
    )
    logs = []
    panel = SimpleNamespace(
        app_root=tmp_path,
        record_button=_ValueWidget(),
        save_history_button=_ValueWidget(),
        recording_status_label=_ValueWidget(),
        _log=lambda message: logs.append(message),
        _refresh_recording_table=lambda: None,
    )
    original_discover = recording_controller_module.discover_retryable_sessions
    original_load = recording_controller_module.load_retryable_session
    recording_controller_module.discover_retryable_sessions = lambda root: [output_dir]
    recording_controller_module.load_retryable_session = lambda candidate: recovered
    try:
        result = RecordingController(panel)._recover_retryable_recording_session()
    finally:
        recording_controller_module.discover_retryable_sessions = original_discover
        recording_controller_module.load_retryable_session = original_load

    assert result is True
    assert panel.recording_session_id == "recovered-1"
    assert panel.distance_recording_rows is recovered.pair_rows
    assert panel._session_save_retry_required is True
    assert panel.record_button.value_state is False
    assert panel.save_history_button.value_state is True
    assert panel.pnp_photo_capture_records == []
    assert panel.oz_photo_capture_records == []


def test_apply_wiris_stip_skips_unchanged_host_without_starting_worker() -> None:
    logs = []
    panel = SimpleNamespace(
        left_zoom_host_edit=_ValueWidget("192.168.1.10"),
        left_stip_ip_edit=_ValueWidget("192.168.1.10"),
        left_url_edit=_ValueWidget("rtsp://192.168.1.10"),
        stip_mask_edit=_ValueWidget("255.255.255.0"),
        camera_connection_label=_ValueWidget(),
        _camera_name_ko=lambda camera: "좌" if camera == "left" else "우",
        _log=lambda message: logs.append(message),
    )

    WirisControlController(panel).apply_wiris_stip("left")

    assert "unchanged" in panel.camera_connection_label.text()
    assert any("건너뜀" in message for message in logs)


def test_apply_workstation_calibration_setup_applies_values_and_refreshes() -> None:
    calls = []
    panel = SimpleNamespace(
        zoom_level_spin=_ValueWidget(0),
        calib_cols_spin=_ValueWidget(0),
        calib_rows_spin=_ValueWidget(0),
        calib_square_spin=_ValueWidget(0.0),
        calib_min_images_spin=_ValueWidget(0),
        calib_calculation_pairs_spin=_ValueWidget(0),
        calibration_auto_interval_spin=_ValueWidget(0),
        calibration_auto_target_spin=_ValueWidget(0),
        calib_dir_edit=_ValueWidget(""),
        checkerboard_validation_root_edit=_ValueWidget(""),
        checkerboard_validation_json_edit=_ValueWidget(""),
        checkerboard_validation_random_runs_spin=_ValueWidget(0),
        checkerboard_validation_random_pair_count_spin=_ValueWidget(0),
        checkerboard_validation_seed_spin=_ValueWidget(0),
        checkerboard_validation_mode_combo=SimpleNamespace(current=None),
        _set_combo_data=lambda combo, value: setattr(combo, "current", value),
        _refresh_all_calibration_previews=lambda: calls.append("previews"),
        _refresh_zoom_intrinsics_tab=lambda: calls.append("zoom_tab"),
        scan_checkerboard_validation_files=lambda: calls.append("scan"),
        _refresh_operation_header=lambda: calls.append("header"),
        _log=lambda message: calls.append(("log", message)),
    )

    WorkstationSettingsController(panel).apply_workstation_calibration_setup(
        {"zoom_level": 7, "inner_cols": 9, "inner_rows": 6, "square_size_mm": 24.5}
    )

    assert panel.zoom_level_spin.value() == 7
    assert panel.calib_cols_spin.value() == 9
    assert panel.calib_rows_spin.value() == 6
    assert "previews" in calls and "zoom_tab" in calls and "scan" in calls and "header" in calls


def test_workstation_settings_snapshot_merges_in_memory_overrides() -> None:
    panel = SimpleNamespace(
        settings=SimpleNamespace(
            interface_name="eth0",
            gnss_operation=SimpleNamespace(to_mapping=lambda: {"operation_mode": "local_survey_in"}),
        ),
        _workstation_setting_overrides={"interface_name": "eth1-pending"},
    )

    snapshot = WorkstationSettingsController(panel).workstation_settings_snapshot()

    assert snapshot["interface_name"] == "eth1-pending"
    assert snapshot["gnss_operation"] == {"operation_mode": "local_survey_in"}
