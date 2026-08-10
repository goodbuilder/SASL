from __future__ import annotations

import os
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets

from wiris_stereo_distance.gui import _parse_ui_mode
from Workstation.command_registry import CommandRegistry, CommandSpec
from Workstation.evidence import EvidenceAdapter
from sasl_common.lifecycle import LifecycleCoordinator, LifecycleStep
from Workstation.main_window import WorkstationMainWindow
from Workstation.status_model import BackendStatusAdapter, StatusSeverity
from Workstation.standalone_backend import (
    StandalonePanelBackend,
    create_standalone_backend,
)


def test_p1_body_frd_baseline_projection_keeps_raw_ned_contract_explicit() -> None:
    from math import cos, radians, sin

    from Workstation.workstation_core.rtk_geometry import project_ned_baseline_to_p1_body_frd

    identity = project_ned_baseline_to_p1_body_frd(
        (1.0, 2.0, 3.0),
        (1.0, 0.0, 0.0, 0.0),
        attitude_timestamp_s=10.0,
        attitude_age_s=0.25,
    )
    assert identity.vector_m == pytest.approx((1.0, 2.0, 3.0))
    assert identity.source_frame == "ned"
    assert identity.target_frame == "px4_frd"
    assert identity.direction == "P1->P2"
    assert identity.axis_contract == "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres"
    assert identity.attitude_age_s == pytest.approx(0.25)

    yaw_90 = (cos(radians(90.0) / 2.0), 0.0, 0.0, sin(radians(90.0) / 2.0))
    rotated = project_ned_baseline_to_p1_body_frd((1.0, 2.0, 3.0), yaw_90)
    assert rotated.vector_m == pytest.approx((2.0, -1.0, 3.0))


def test_ui_mode_parser_removes_workstation_argument() -> None:
    mode, argv = _parse_ui_mode(["main.py", "--ui", "workstation", "--maximized"])

    assert mode == "workstation"
    assert argv == ["main.py", "--maximized"]
    assert _parse_ui_mode(["main.py"])[0] == "legacy"


def test_workstation_camera_sync_readiness_uses_imported_finite_check() -> None:
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    metadata = {
        "rtcp_sr_received_count": 2,
        "rtcp_sr_age_s": 0.2,
        "rtcp_camera_ntp_rate": 1.0001,
        "rtcp_camera_media_rate": 0.9999,
        "timestamp_s": 10.0,
    }
    panel = SimpleNamespace(
        camera_connected=True,
        camera_frame_metadata_history={},
        _camera_sync_gdti={
            "left": {"unix_s": 1_800_000_000.01},
            "right": {"unix_s": 1_800_000_000.02},
        },
        _camera_name_ko=lambda camera: camera,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_sync_receive_pair_max_delta_s=lambda: 0.35,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
        _clock_sync_pair_metadata=lambda **_kwargs: (dict(metadata), dict(metadata)),
        _camera_time_sync_metrics=lambda *_args, **_kwargs: {
            "pair_delta_s": 0.01,
            "receive_pair_delta_s": 0.01,
            "max_sr_pc_offset_ms": 20.0,
        },
    )

    ready, reason = CameraTimeSyncController(panel)._camera_sync_readiness()

    assert ready is True
    assert "검증 완료" in reason


def test_gui_compatibility_module_aliases_legacy_implementation() -> None:
    import wiris_stereo_distance.gui as gui
    import wiris_stereo_distance.legacy_gui as legacy_gui

    assert gui is legacy_gui
    assert gui.__file__.endswith("/legacy_gui.py")
    assert gui.WirisStereoDistancePanel is legacy_gui.WirisStereoDistancePanel


def test_standalone_entrypoint_does_not_import_legacy_gui() -> None:
    repository = Path(__file__).resolve().parents[3]
    app_root = repository / "apps" / "StereoWIRIS"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(app_root), str(repository / "apps"), str(repository / "common" / "python"))
    )
    environment["SASL_ROS_LIBRARY_PATH_READY"] = "1"
    script = """
import sys

import main

assert main._workstation_requested(["main.py", "--ui", "workstation"])
assert "wiris_stereo_distance.gui" not in sys.modules
assert "wiris_stereo_distance.legacy_gui" not in sys.modules
from Workstation import standalone_gui
assert standalone_gui.__file__.endswith("/standalone_gui.py")
assert "wiris_stereo_distance.gui" not in sys.modules
assert "wiris_stereo_distance.legacy_gui" not in sys.modules
assert not any(
    name == "wiris_stereo_distance" or name.startswith("wiris_stereo_distance.")
    for name in sys.modules
)
assert "rclpy" not in sys.modules
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=app_root,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )


def test_workstation_launcher_bypasses_stereowiris_runtime() -> None:
    from Workstation import standalone_gui

    repository = Path(__file__).resolve().parents[3]
    launcher = (repository / "apps" / "Workstation" / "run.sh").read_text(encoding="utf-8")
    entrypoint = (repository / "apps" / "Workstation" / "__main__.py").read_text(encoding="utf-8")
    runtime_requirements = (repository / "requirements" / "runtime.txt").read_text(encoding="utf-8")

    assert "StereoWIRIS" not in launcher
    assert "Simulation" not in launcher
    assert "-m Workstation" in launcher
    assert "--ui" not in launcher
    assert "WorkstationMainWindow" in entrypoint
    assert "legacy" not in entrypoint.lower()
    assert "PyQt5" in runtime_requirements
    assert "PyQt6" not in runtime_requirements
    assert standalone_gui._parse_ui_mode(["workstation"])[0] == "workstation"
    assert standalone_gui._parse_ui_mode(["workstation", "--ui", "workstation"])[1] == ["workstation"]
    with pytest.raises(SystemExit):
        standalone_gui._parse_ui_mode(["workstation", "--ui", "legacy"])
    assert not hasattr(standalone_gui, "WirisStereoDistanceGui")
    assert not hasattr(standalone_gui, "IntegratedWirisPixhawkConsole")


def test_workstation_core_has_no_legacy_ros_or_image_timesync_surface() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    core_root = repo_root / "apps" / "Workstation" / "workstation_core"
    forbidden_markers = (
        "wiris_stereo_distance",
        "rclpy",
        "rosidl",
        "ros2",
        "ros 2",
        "uxrce",
        "mavros",
        "photo_clock",
        "cptr",
        "icpt",
        "exif",
        "clock_ftp",
        "still-image",
        "still image",
    )
    for source_path in (*core_root.glob("*.py"), *(repo_root / "Workstation").glob("*.py")):
        if source_path.name == "__init__.py":
            continue
        source = source_path.read_text(encoding="utf-8").lower()
        assert not any(marker in source for marker in forbidden_markers), source_path

    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber
    from Workstation.workstation_core.settings import AppSettings

    assert not hasattr(DualPixhawkMavlinkSubscriber, "start_ros2_client")
    assert not hasattr(DualPixhawkMavlinkSubscriber, "start_px4_uxrce_client_shell")
    assert not hasattr(AppSettings, "ros_domain_id")
    assert not hasattr(AppSettings, "mavlink_configure_uxrce_agent")
    assert not hasattr(AppSettings, "clock_ftp_user")


def test_legacy_ros2_and_image_timesync_surface_remains_available() -> None:
    from wiris_stereo_distance import settings as legacy_settings
    from wiris_stereo_distance.photo_clock import compare_photo_clocks
    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    assert hasattr(legacy_settings.AppSettings, "ros_domain_id")
    assert hasattr(legacy_settings.AppSettings, "clock_ftp_user")
    assert hasattr(DualPixhawkMavlinkSubscriber, "start_ros2_client")
    assert callable(compare_photo_clocks)


def test_workstation_mavlink_packet_provenance_reaches_state_and_timeline() -> None:
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        _mavlink_optional_timeline_row,
    )
    from Workstation.workstation_core.measurement_attempt import AttitudeSnapshot, GpsSnapshot

    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    attitudes = []
    gps_states = []
    subscriber.extrinsic_changed.connect(lambda _camera, state: attitudes.append(state))
    subscriber.gps_changed.connect(lambda _camera, state: gps_states.append(state))

    attitude_message = SimpleNamespace(
        get_type=lambda: "ATTITUDE",
        get_srcSystem=lambda: 11,
        get_srcComponent=lambda: 42,
        get_seq=lambda: 7,
        get_msgId=lambda: 30,
        roll=0.1,
        pitch=0.2,
        yaw=0.3,
        rollspeed=0.0,
        pitchspeed=0.0,
        yawspeed=0.0,
        time_boot_ms=123,
    )
    subscriber._handle_mavlink_message("left", attitude_message)
    assert len(attitudes) == 1
    assert attitudes[0].mavlink_source_system == 11
    assert attitudes[0].mavlink_source_component == 42
    assert attitudes[0].mavlink_sequence == 7
    assert attitudes[0].mavlink_message_id == 30
    attitude_snapshot = AttitudeSnapshot.from_state(attitudes[0])
    assert attitude_snapshot.mavlink_sequence == 7

    gps_message = SimpleNamespace(
        get_type=lambda: "GPS_RAW_INT",
        get_srcSystem=lambda: 11,
        get_srcComponent=lambda: 42,
        get_seq=lambda: 8,
        get_msgId=lambda: 24,
        lat=370000000,
        lon=1270000000,
        alt=10000,
        fix_type=6,
        satellites_visible=18,
        eph=50,
        epv=60,
        cog=9000,
        time_usec=1_800_000_000_000_000,
        yaw=0,
        hdg_acc=0,
    )
    subscriber._handle_mavlink_message("left", gps_message)
    assert len(gps_states) == 1
    assert gps_states[0].mavlink_source_system == 11
    assert gps_states[0].mavlink_source_component == 42
    assert gps_states[0].mavlink_sequence == 8
    assert gps_states[0].mavlink_message_id == 24
    gps_snapshot = GpsSnapshot.from_state(gps_states[0])
    assert gps_snapshot.mavlink_sequence == 8

    timeline_row = _mavlink_optional_timeline_row(
        "GLOBAL_POSITION_INT",
        SimpleNamespace(
            get_srcSystem=lambda: 11,
            get_srcComponent=lambda: 42,
            get_seq=lambda: 9,
            get_msgId=lambda: 33,
            lat=370000000,
            lon=1270000000,
            alt=10000,
            relative_alt=9000,
            vx=1,
            vy=2,
            vz=3,
            hdg=9000,
            time_boot_ms=123,
        ),
        10.0,
    )
    assert timeline_row is not None
    assert timeline_row["mavlink_source_system"] == 11
    assert timeline_row["mavlink_source_component"] == 42
    assert timeline_row["mavlink_sequence"] == 9
    assert timeline_row["mavlink_message_id"] == 33


def test_experiment_protocol_readiness_is_advisory_and_mode_specific() -> None:
    from Workstation.protocol import evaluate_recording_protocol

    snapshot = {
        key: SimpleNamespace(state="LIVE" if key.startswith(("camera.", "pixhawk.")) else state)
        for key, state in (
            ("camera.left", "LIVE"),
            ("camera.right", "LIVE"),
            ("synchronization", "SYNCED"),
            ("detection", "RUNNING"),
            ("pixhawk.left", "LIVE"),
            ("pixhawk.right", "LIVE"),
        )
    }
    calibration = {
        "active": True,
        "left": {"state": "AVAILABLE"},
        "right": {"state": "AVAILABLE"},
        "extrinsics": {"state": "AVAILABLE"},
    }
    baseline = {"state": "GOOD", "length_m": 1.2}

    pixhawk_ready = evaluate_recording_protocol(
        snapshot,
        distance_mode="pixhawk",
        calibration=calibration,
        baseline=baseline,
        require_rtk_fixed=True,
    )
    assert pixhawk_ready.state == "READY"
    assert not pixhawk_ready.blocking

    checkerboard_ready = evaluate_recording_protocol(
        snapshot,
        distance_mode="checkerboard",
        calibration=calibration,
        baseline={"state": "NO DATA", "length_m": None},
        require_rtk_fixed=True,
    )
    assert checkerboard_ready.state == "READY"

    snapshot["synchronization"] = SimpleNamespace(state="UNVERIFIED")
    blocked = evaluate_recording_protocol(
        snapshot,
        distance_mode="pixhawk",
        calibration=calibration,
        baseline=baseline,
        require_rtk_fixed=True,
    )
    assert blocked.state == "BLOCKED"
    assert {check.key for check in blocked.blocking} == {"synchronization"}


def test_recording_protocol_includes_fix_alignment_and_final_sync_gates() -> None:
    from Workstation.protocol import evaluate_recording_protocol

    snapshot = {
        key: SimpleNamespace(state=state)
        for key, state in (
            ("camera.left", "LIVE"),
            ("camera.right", "LIVE"),
            ("synchronization", "SYNCED"),
            ("detection", "RUNNING"),
            ("pixhawk.left", "LIVE"),
            ("pixhawk.right", "LIVE"),
        )
    }
    calibration = {
        "active": True,
        "left": {"state": "AVAILABLE"},
        "right": {"state": "AVAILABLE"},
    }
    baseline = {
        "state": "GOOD",
        "length_m": 1.2,
        "left_fix_type": 3,
        "right_fix_type": 3,
    }
    readiness = evaluate_recording_protocol(
        snapshot,
        distance_mode="pixhawk",
        calibration=calibration,
        baseline=baseline,
        require_rtk_fixed=False,
        minimum_fix_type=3,
        physical_alignment={"state": "PASS"},
        final_sync_verified=True,
    )
    assert readiness.state == "READY"
    assert {check.key for check in readiness.checks} >= {
        "fix_type",
        "physical_alignment",
        "final_synchronization",
    }

    baseline["right_fix_type"] = 2
    blocked = evaluate_recording_protocol(
        snapshot,
        distance_mode="pixhawk",
        calibration=calibration,
        baseline=baseline,
        require_rtk_fixed=False,
        minimum_fix_type=3,
        physical_alignment={"state": "FAIL"},
        final_sync_verified=False,
    )
    assert {check.key for check in blocked.blocking} >= {
        "fix_type",
        "physical_alignment",
        "final_synchronization",
    }


def test_experiment_step_one_requires_mode_specific_base_link_and_step_ten_tracks_detection() -> None:
    from Workstation.workspaces import ExperimentWorkspace
    from Workstation.workstation_core.gnss_operation import GnssOperationConfig, GnssOperationManager

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    manager = GnssOperationManager(GnssOperationConfig())
    manager.configure(GnssOperationConfig())
    panel = SimpleNamespace(
        gnss_operation_manager=manager,
        gnss_base_serial_connected=False,
        gnss_network_connected=False,
        gnss_network_status_line="",
        gnss_base_rtcm_stats={},
        latest_gnss_base_survey_in={},
        gnss_base_survey_samples=(),
        gnss_coordinate_samples=(),
        latest_gps={
            "left": SimpleNamespace(fix_type=3, rtk_label="3D FIX", topic="mavlink/GPS_RAW_INT"),
            "right": SimpleNamespace(fix_type=3, rtk_label="3D FIX", topic="mavlink/GPS_RAW_INT"),
        },
    )
    alignment_snapshot: dict[str, object] = {
        "state": "WAITING",
        "sampling_active": False,
        "sample_count": 0,
        "iterations": (),
        "quality": {},
    }
    panel.workstation_physical_alignment_snapshot = lambda: alignment_snapshot
    workspace = ExperimentWorkspace(SimpleNamespace(panel=panel))
    assert [button.text() for button in workspace.step_action_buttons[1]] == ["Cameras", "Pixhawk", "Base"]
    assert [button.text() for button in workspace.step_action_buttons[4]] == ["Base", "Pixhawk"]
    opened: list[str] = []
    workspace.open_context.connect(opened.append)
    workspace.step_action_buttons[1][1].click()
    workspace.step_action_buttons[4][0].click()
    workspace.pre_run_calibration_button.click()
    assert opened == ["devices.pixhawk", "devices.gnss", "tasks.calibration"]
    snapshot = {
        "camera.left": SimpleNamespace(state="LIVE"),
        "camera.right": SimpleNamespace(state="LIVE"),
        "pixhawk.left": SimpleNamespace(state="LIVE"),
        "pixhawk.right": SimpleNamespace(state="LIVE"),
        "synchronization": SimpleNamespace(state="SYNCED"),
        "detection": SimpleNamespace(state="RUNNING"),
        "measurement": SimpleNamespace(state="WAITING"),
        "recording": SimpleNamespace(state="IDLE"),
    }

    workspace.refresh(snapshot)
    assert workspace.step_states[1].text() == "WAIT"
    assert "BASE=DISCONNECTED" in workspace.step_details[1].text()
    assert workspace.step_states[10].text() == "RUNNING"
    assert "detection=RUNNING" in workspace.step_details[10].text()

    panel.gnss_base_serial_connected = True
    workspace.refresh(snapshot)
    assert workspace.step_states[1].text() == "READY"
    assert "BASE=CONNECTED" in workspace.step_details[1].text()

    alignment_snapshot.update(
        {
            "state": "FAIL",
            "sample_count": 20,
            "relative_rpy_deg": (2.0, 0.0, 0.0),
            "iterations": ({"iteration": 1},),
            "quality": {
                "baseline_error_m": (0.001, -0.001, 0.0),
                "relative_rpy_error_deg": (2.0, 0.0, 0.0),
                "reasons": ("relative_rpy_out_of_tolerance",),
            },
        }
    )
    workspace.refresh(snapshot)
    assert workspace.step_states[5].text() == "READY"
    assert "FRD error=" in workspace.step_details[5].text()
    assert workspace.step_states[6].text() == "CHECK"
    assert workspace.step_states[7].text() == "CHECK"

    alignment_snapshot["state"] = "PASS"
    alignment_snapshot["iterations"] = ({"iteration": 1}, {"iteration": 2})
    alignment_snapshot["quality"] = {
        "baseline_error_m": (0.001, -0.001, 0.0),
        "relative_rpy_error_deg": (0.1, -0.1, 0.2),
        "reasons": (),
    }
    workspace.refresh(snapshot)
    assert [workspace.step_states[number].text() for number in (5, 6, 7)] == ["READY", "READY", "READY"]

    panel.recording_session_id = "unsaved-session"
    workspace.refresh(snapshot)
    assert workspace.step_states[11].text() == "SAVE"
    assert "unsaved" in workspace.step_details[11].text()
    snapshot["recording"] = SimpleNamespace(state="SAVED")
    workspace.refresh(snapshot)
    assert workspace.step_states[11].text() == "SAVED"

    workspace.deleteLater()
    app.processEvents()


def test_workstation_entrypoint_does_not_add_ros_runtime_paths() -> None:
    app_root = Path(__file__).resolve().parents[2] / "StereoWIRIS"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(app_root)
    environment["SASL_ROS_LIBRARY_PATH_READY"] = "1"
    script = """
import sys

sys.argv = ["main.py", "--ui", "workstation"]
import main

assert main.WORKSTATION_REQUESTED is True
assert not any("/opt/ros/" in path for path in sys.path)
assert not any("workspace/main/install" in path for path in sys.path)
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=app_root,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )


def test_legacy_entrypoint_does_not_import_workstation_modules() -> None:
    repository = Path(__file__).resolve().parents[3]
    app_root = repository / "apps" / "StereoWIRIS"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(app_root), str(repository / "apps"), str(repository / "common" / "python"))
    )
    environment["SASL_ROS_LIBRARY_PATH_READY"] = "1"
    script = """
import sys

import main

assert not main._workstation_requested(["main.py"])
from wiris_stereo_distance import legacy_gui
assert legacy_gui.__file__.endswith("/legacy_gui.py")
assert "Workstation" not in sys.modules
assert not any(name.startswith("Workstation.") for name in sys.modules)
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=app_root,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )


def test_workstation_runtime_paths_are_separate_from_legacy_storage() -> None:
    from wiris_stereo_distance import settings as legacy_settings
    from Workstation.runtime_paths import (
        WORKSTATION_DATA_ROOT,
        WORKSTATION_RECORDINGS_ROOT,
        WORKSTATION_SETTINGS_PATH,
    )

    assert WORKSTATION_DATA_ROOT.name == "workstation_data"
    assert WORKSTATION_DATA_ROOT.parent.name == ".local"
    assert WORKSTATION_SETTINGS_PATH == WORKSTATION_DATA_ROOT / "stereo_distance_settings.json"
    assert WORKSTATION_RECORDINGS_ROOT == WORKSTATION_DATA_ROOT / "recordings"
    assert WORKSTATION_SETTINGS_PATH != legacy_settings.DEFAULT_SETTINGS_PATH
    assert WORKSTATION_RECORDINGS_ROOT != legacy_settings.APP_ROOT / "recordings"


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


def test_workstation_evidence_uses_panel_data_root(tmp_path, monkeypatch: pytest.MonkeyPatch) -> None:
    from wiris_stereo_distance import settings as legacy_settings

    legacy_root = tmp_path / "legacy"
    workstation_root = tmp_path / "workstation"
    monkeypatch.setattr(legacy_settings, "APP_ROOT", legacy_root)
    session_dir = workstation_root / "recordings" / "sessions" / "workstation-session"
    session_dir.mkdir(parents=True)
    (session_dir / "session_manifest.json").write_text(
        '{"status":"finalized","artifact_roles":'
        '{"workstation-session_distance_measurements.csv":"distance_measurements_csv"}}',
        encoding="utf-8",
    )
    (session_dir / "workstation-session_distance_measurements.csv").write_text(
        "recording_time_ns,forward_m,right_m,down_m,range_m\n1,2,3,4,5\n",
        encoding="utf-8",
    )

    adapter = EvidenceAdapter(SimpleNamespace(app_root=workstation_root))
    catalog = adapter.session_catalog()
    rows = adapter.session_artifact_rows(
        "workstation-session",
        "workstation-session_distance_measurements.csv",
    )

    assert catalog and catalog[0]["path"] == str(session_dir)
    assert rows["rows"] == (
        {"recording_time_ns": "1", "forward_m": "2", "right_m": "3", "down_m": "4", "range_m": "5"},
    )


def test_standalone_backend_is_a_separate_copy_and_factory_owner(monkeypatch: pytest.MonkeyPatch) -> None:
    from Workstation import standalone_gui

    assert Path(standalone_gui.__file__).name == "standalone_gui.py"

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = QtWidgets.QWidget()
    monkeypatch.setattr(standalone_gui, "WirisStereoDistancePanel", lambda **_kwargs: panel)
    backend = create_standalone_backend()

    assert isinstance(backend, StandalonePanelBackend)
    assert backend.panel is panel
    assert backend.owner_widget is panel
    backend.deleteLater()
    panel.deleteLater()
    app.processEvents()


def test_standalone_backend_facades_match_the_checked_contract() -> None:
    from Workstation.protocol import missing_backend_facades

    assert missing_backend_facades(StandalonePanelBackend) == ()


def test_lifecycle_coordinator_resumes_after_pending_step() -> None:
    calls: list[str] = []
    pending = {"value": True}

    def first() -> bool:
        calls.append("first")
        if pending["value"]:
            pending["value"] = False
            return False
        return True

    coordinator = LifecycleCoordinator(
        [LifecycleStep("first", first), LifecycleStep("second", lambda: calls.append("second"))]
    )

    assert coordinator.run() is False
    assert calls == ["first"]
    assert coordinator.run() is True
    assert calls == ["first", "first", "second"]
    assert coordinator.run() is True
    assert calls == ["first", "first", "second"]


class _FakeBackend:
    def __init__(self) -> None:
        self.calls: list[object] = []
        left_bootstrap_baud = QtWidgets.QSpinBox()
        left_bootstrap_baud.setRange(1, 10_000_000)
        left_bootstrap_baud.setValue(115_200)
        right_bootstrap_baud = QtWidgets.QSpinBox()
        right_bootstrap_baud.setRange(1, 10_000_000)
        right_bootstrap_baud.setValue(115_200)
        self.panel = SimpleNamespace(
            camera_connected=False,
            yolo_enabled=False,
            camera_frame_times={"left": [], "right": []},
            camera_stream_stats={"left": {}, "right": {}},
            pixhawk_attitude_times={"left": [], "right": []},
            last_distance_quality="BAD",
            last_distance_status="거리: 좌/우 검출 매칭 대기 중",
            recording_session_id="",
            distance_history=[],
            left_url_edit=QtWidgets.QLineEdit("rtsp://left"),
            right_url_edit=QtWidgets.QLineEdit("rtsp://right"),
            _session_save_thread=None,
            _session_save_retry_required=False,
            _session_save_finalized=False,
            recording_active=False,
            camera_frame_metadata={"left": {}, "right": {}},
            camera_sync_status="not-verified",
            camera_sync_reason="STUS 동기화 후 RTCP/NTP 검증이 필요합니다.",
            selected_runtime_frame_pair_delta_s=None,
            selected_runtime_frame_pair_basis="",
            selected_runtime_frame_pair_camera_rtc_delta_s=None,
            latest_gps={},
            latest_rtk_baseline=None,
            custom_watch_specs=[],
            _camera_candidate_rows=[],
            camera_network_discovery_thread=None,
            camera_auto_detect_thread=None,
            workstation_camera_discovery_status="NOT RUN",
            _pixhawk_candidate_rows_by_source={"pc": [], "mavlink": []},
            pixhawk_usb_discovery_thread=None,
            pixhawk_host_ip_discovery_thread=None,
            workstation_pixhawk_discovery_status="NOT RUN",
            left_pixhawk_bootstrap_url_edit=QtWidgets.QLineEdit("/dev/ttyACM0"),
            right_pixhawk_bootstrap_url_edit=QtWidgets.QLineEdit("/dev/ttyACM1"),
            left_pixhawk_bootstrap_baud_spin=left_bootstrap_baud,
            right_pixhawk_bootstrap_baud_spin=right_bootstrap_baud,
            pixhawk_usb_bootstrap_thread=None,
            pixhawk_usb_bootstrap_status={
                "left": {"state": "IDLE", "detail": "대기", "updated_s": 10.0},
                "right": {"state": "IDLE", "detail": "대기", "updated_s": 10.0},
            },
        )
        self.panel._session_recording_active = lambda: bool(self.panel.recording_active)

    def toggle_camera(self) -> None:
        self.calls.append("camera")

    def toggle_detection(self) -> None:
        self.calls.append("detection")

    def toggle_recording(self) -> None:
        self.calls.append("recording")

    def save_session(self) -> None:
        self.calls.append("save")

    def apply_camera_settings(self, left: str, right: str) -> bool:
        self.calls.append(("settings", left, right))
        return True

    def camera_discover_host_ip(self) -> None:
        self.calls.append("camera_discover_host_ip")

    def camera_apply_candidate(self, row: int, target: str) -> None:
        self.calls.append(("camera_apply_candidate", row, target))

    def camera_auto_connect(self) -> None:
        self.calls.append("camera_auto_connect")

    def camera_check_rtsp(self) -> None:
        self.calls.append("camera_check_rtsp")

    def camera_check_control(self) -> None:
        self.calls.append("camera_check_control")

    def camera_enable_rtsp(self) -> None:
        self.calls.append("camera_enable_rtsp")

    def camera_swap(self) -> None:
        self.calls.append("camera_swap")

    def camera_save_settings(self) -> None:
        self.calls.append("camera_save_settings")

    def time_set_pc(self) -> None:
        self.calls.append("time_set_pc")

    def time_check_sync(self) -> None:
        self.calls.append("time_check_sync")

    def pixhawk_toggle_mavlink(self) -> None:
        self.calls.append("pixhawk_toggle_mavlink")

    def pixhawk_recover_eth_mavlink(self, camera: str) -> None:
        self.calls.append(("pixhawk_recover_eth_mavlink", camera))

    def pixhawk_discover_usb(self) -> None:
        self.calls.append("pixhawk_discover_usb")

    def pixhawk_discover_mavlink_candidates(self) -> None:
        self.calls.append("pixhawk_discover_mavlink_candidates")

    def pixhawk_discover_host_ip(self) -> None:
        self.calls.append("pixhawk_discover_host_ip")

    def pixhawk_apply_candidate(self, source: str, row: int, target: str) -> None:
        self.calls.append(("pixhawk_apply_candidate", source, row, target))

    def gnss_base_discover(self) -> None:
        self.calls.append("gnss_base_discover")

    def gnss_base_apply_candidate(self, row: int) -> None:
        self.calls.append(("gnss_base_apply_candidate", row))

    def gnss_base_connect(self) -> None:
        self.calls.append("gnss_base_connect")

    def gnss_base_disconnect(self) -> None:
        self.calls.append("gnss_base_disconnect")

    def gnss_survey_start(self) -> None:
        self.calls.append("gnss_survey_start")

    def gnss_survey_stop(self) -> None:
        self.calls.append("gnss_survey_stop")

    def gnss_ntrip_start(self) -> None:
        self.calls.append("gnss_ntrip_start")

    def gnss_ntrip_stop(self) -> None:
        self.calls.append("gnss_ntrip_stop")

    def calibration_capture_images(self) -> None:
        self.calls.append("calibration_capture_images")

    def calibration_run_intrinsic(self) -> None:
        self.calls.append("calibration_run_intrinsic")

    def calibration_save_intrinsics(self) -> None:
        self.calls.append("calibration_save_intrinsics")

    def calibration_load_intrinsics(self) -> None:
        self.calls.append("calibration_load_intrinsics")

    def calibration_load_current_zoom_intrinsics(self) -> None:
        self.calls.append("calibration_load_current_zoom_intrinsics")

    def calibration_apply_intrinsics(self) -> None:
        self.calls.append("calibration_apply_intrinsics")

    def calibration_load_validation(self) -> None:
        self.calls.append("calibration_load_validation")

    def calibration_run_validation(self) -> None:
        self.calls.append("calibration_run_validation")

    def calibration_save_validation(self) -> None:
        self.calls.append("calibration_save_validation")

    def calibration_apply_validation(self) -> None:
        self.calls.append("calibration_apply_validation")

    def apply_measurement_settings(self, settings: dict[str, object]) -> None:
        self.calls.append(("measurement_apply_settings", settings))

    def save_measurement_settings(self) -> None:
        self.calls.append("measurement_save_settings")

    def workstation_settings_snapshot(self) -> dict[str, object]:
        from Workstation.workstation_core.settings import AppSettings

        defaults = AppSettings()
        snapshot = dict(defaults.__dict__)
        snapshot["gnss_operation"] = defaults.gnss_operation.to_mapping()
        return snapshot

    def apply_workstation_system_settings(self, settings: dict[str, object]) -> None:
        self.calls.append(("settings_apply", dict(settings)))

    def save_workstation_system_settings(self) -> None:
        self.calls.append("settings_save")

    def calibration_apply_setup(self, setup: dict[str, object]) -> None:
        self.calls.append(("calibration_apply_setup", setup))

    def calibration_scan_validation(self) -> None:
        self.calls.append("calibration_scan_validation")

    def calibration_save_setup(self) -> None:
        self.calls.append("calibration_save_setup")

    def alignment_scan_sources(self) -> None:
        self.calls.append("alignment_scan_sources")

    def alignment_start_sampling(self) -> None:
        self.calls.append("alignment_start_sampling")

    def alignment_stop_sampling(self) -> None:
        self.calls.append("alignment_stop_sampling")

    def alignment_save_profile(self) -> None:
        self.calls.append("alignment_save_profile")

    def alignment_load_profile(self) -> None:
        self.calls.append("alignment_load_profile")

    def alignment_swap_pixhawks(self) -> None:
        self.calls.append("alignment_swap_pixhawks")

    def advanced_wiris_catalog(self) -> tuple[dict[str, str], ...]:
        return (
            {
                "category": "Basic",
                "label": "Check connection",
                "command": "HIWS",
                "terminator": "\n",
                "description": "Connection check",
            },
        )

    def advanced_send_wiris_command(self, target: str, command: str, terminator: str) -> None:
        self.calls.append(("advanced_wiris", target, command, terminator))

    def advanced_send_zoom(self, target: str, action: str, level: int | None = None) -> None:
        self.calls.append(("advanced_zoom", target, action, level))

    def advanced_discover_custom_mavlink_watch(self) -> None:
        self.calls.append("advanced_watch_discover_mavlink")

    def advanced_add_custom_mavlink_watch(self, name: str, message: str, field: str) -> str:
        self.calls.append(("advanced_watch_add_mavlink", name, message, field))
        return name or f"mavlink:{message}.{field}"

    def advanced_remove_custom_mavlink_watch(self, key: str) -> None:
        self.calls.append(("advanced_watch_remove", key))

    def advanced_refresh_baseline_fields(self) -> dict[str, object]:
        self.calls.append("advanced_baseline_scan")
        return {
            "mavlink_messages": (("GPS_RAW_INT", "lat, lon, alt"),),
            "axis_contract": "producer fields preserved",
        }

    def advanced_open_px4_shell(self, camera: str) -> None:
        self.calls.append(("advanced_px4_shell", camera))

    def advanced_refresh_px4_mag(self, camera: str, *, read_params: bool = True) -> None:
        self.calls.append(("advanced_px4_mag", camera, read_params))

    def advanced_refresh_px4_attitude_cal(self, camera: str, *, read_params: bool = True) -> None:
        self.calls.append(("advanced_px4_attitude", camera, read_params))

    def advanced_apply_px4_mag_priorities(self, camera: str, priorities: dict[str, int]) -> None:
        self.calls.append(("advanced_px4_apply_mag", camera, priorities))

    def advanced_save_px4_mag_params(self, camera: str) -> None:
        self.calls.append(("advanced_px4_save_mag", camera))

    def advanced_reboot_px4(self, camera: str) -> None:
        self.calls.append(("advanced_px4_reboot", camera))

    def advanced_refresh_calibration_preview(self) -> dict[str, object]:
        self.calls.append("advanced_preview_refresh")
        return {"images": {}, "axis_contract": "checkerboard pixels preserved"}

    def advanced_calibration_preview_snapshot(self) -> dict[str, object]:
        return {"images": {}, "axis_contract": "checkerboard pixels preserved"}

    def diagnostic_text(self) -> str:
        return "[fake] source-backed log"

    def shutdown_for_close(self) -> bool:
        self.calls.append("shutdown")
        return True


def test_workstation_shell_navigation_and_close() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.resize(1366, 768)
    app.processEvents()
    assert window.workspace_host.width() > 500
    root = window.navigation.model.item(0, 0)
    assert [root.child(row).text() for row in range(root.rowCount())] == [
        "Overview",
        "Protocol",
        "Pre-run Calibration",
        "Devices",
        "Alignment",
        "Measurement",
        "Recording / Sessions",
        "Advanced",
        "Settings",
        "Diagnostics",
    ]

    window.navigation.select_context("devices.cameras")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.camera
    assert [
        window.camera.operation_buttons[key].text()
        for key in (
            "camera.discover_host_ip",
            "camera.check_control",
            "camera.enable_rtsp",
            "camera.check_rtsp",
        )
    ] == [
        "Find network",
        "Check control",
        "Enable stream",
        "Check video",
    ]

    window.navigation.select_context("devices.time")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.time_workspace
    assert window.time_workspace.sync_summary.height() == 143
    assert window.commands.invoke("time.check_sync") is True
    assert backend.calls[-1] == "time_check_sync"

    window.navigation.select_context("processing.measurement")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.measurement
    assert any(
        "baseline_center_frd" in label.text()
        for label in window.measurement.findChildren(QtWidgets.QLabel)
    )
    assert [window.measurement.values[key].label.text() for key in ("range", "forward", "right", "down")] == [
        "Range",
        "Forward",
        "Right",
        "Down",
    ]
    assert window.measurement.history.model().headerData(0, 1) == "timestamp_s"
    assert set(window.measurement.previews) == {"left", "right"}
    assert not any(
        group.title() == "Frames"
        for group in window.measurement.findChildren(QtWidgets.QGroupBox)
    )
    assert window.measurement.preview_row.minimumHeight() == 280
    assert window.measurement.preview_row.maximumHeight() > 380
    assert window.measurement.previews["left"].heightForWidth(640) > 300
    assert set(window.measurement.upstream_values) >= {
        "camera.left",
        "camera.right",
        "detection",
        "synchronization",
        "gnss",
        "recording",
    }
    assert set(window.measurement.control_buttons) == {
        "camera.toggle",
        "detection.toggle",
        "recording.toggle",
        "session.save",
    }
    assert set(window.measurement.quality_labels) == {
        "status",
        "gap",
        "distance",
        "mode",
        "attitude",
        "raw_point",
    }
    backend.panel.triangulation_quality_status_label = QtWidgets.QLabel("GOOD | ray pair accepted")
    backend.panel.triangulation_quality_gap_label = QtWidgets.QLabel("gap: 0.012 m")
    backend.panel.triangulation_quality_distance_label = QtWidgets.QLabel("distance: 12.300 m")
    backend.panel.triangulation_quality_mode_label = QtWidgets.QLabel("mode: two_ray_closest_midpoint")
    backend.panel.triangulation_quality_attitude_label = QtWidgets.QLabel("Attitude timing: FRESH")
    backend.panel.triangulation_quality_raw_point_label = QtWidgets.QLabel(
        "Raw baseline_center_frd point: [12.0, 0.3, -0.1, 12.0]"
    )
    window.measurement.refresh(window.status_adapter.model.snapshot())
    assert window.measurement.quality_labels["mode"].text() == "mode: two_ray_closest_midpoint"
    assert "baseline_center_frd" in window.measurement.quality_labels["raw_point"].text()
    assert "INVALID" not in window.inspector.status_value.meta.text()
    assert "source=" not in window.inspector.status_value.meta.text()
    assert all(
        "source=" not in value.meta.text()
        for value in window.inspector.context_values.values()
    )

    window.navigation.select_context("devices.pixhawk")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.pixhawk
    assert window.pixhawk.attitude_model.horizontalHeaderItem(2).text() == "Roll°"
    assert window.pixhawk.receiver_model.horizontalHeaderItem(1).text() == "Fix number"
    assert "north_m" in window.pixhawk.baseline_values
    assert "p1_body_frd_forward_m" in window.pixhawk.baseline_values
    window.navigation.select_context("devices.gnss")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.base_gnss
    assert window.base_gnss.summary_values["mode"].label.text() == "Mode"
    assert window.base_gnss.survey_values["accuracy"].label.text() == "Mean accuracy"
    assert window.base_gnss.ntrip_values["mountpoint"].label.text() == "Mountpoint"
    assert [
        window.base_gnss.operation_buttons[key].text()
        for key in (
            "gnss.base_discover",
            "gnss.base_connect",
            "gnss.base_disconnect",
            "gnss.ntrip_start",
            "gnss.ntrip_stop",
        )
    ] == [
        "Find Base",
        "Connect Base",
        "Disconnect Base",
        "Start NTRIP",
        "Stop NTRIP",
    ]

    window.navigation.select_context("processing.alignment")
    app.processEvents()
    assert window.alignment.operation_tabs.currentIndex() == 2

    window.navigation.select_context("diagnostics.logs")
    app.processEvents()
    assert window.diagnostics_dock.isVisible()
    assert window.diagnostics.log_view.toPlainText() == "[fake] source-backed log"
    current_workspace = window.workspace_host.currentWidget()
    window.navigation.select_context("diagnostics.camera")
    app.processEvents()
    assert window.workspace_host.currentWidget() is current_workspace
    assert window.diagnostics.tabs.currentIndex() == 1
    window.navigation.select_context("diagnostics.performance")
    app.processEvents()
    assert window.diagnostics.tabs.currentIndex() == 6
    assert "fps" in window.status_widgets

    assert window.commands.invoke("camera.toggle") is True
    assert backend.calls[-1] == "camera"
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
    assert "shutdown" in backend.calls
    window.deleteLater()
    app.processEvents()


def test_base_gnss_workspace_projects_survey_in_and_ntrip_without_relabeling_frames() -> None:
    from Workstation.workspaces import BaseGnssWorkspace
    from Workstation.workstation_core.gnss_operation import (
        CorrectionSourceIdentity,
        GnssOperationConfig,
        GnssOperationManager,
        GnssOperationMode,
    )

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    manager = GnssOperationManager(GnssOperationConfig())
    manager.configure(GnssOperationConfig())
    manager.set_source_identity(
        CorrectionSourceIdentity(source_kind="LOCAL_BASE", base_device="/dev/ttyACM2")
    )
    panel = SimpleNamespace(
        gnss_operation_manager=manager,
        gnss_base_serial_connected=True,
        gnss_base_detect_thread=None,
        workstation_gnss_base_discovery_status="FOUND 1 · UBX 1",
        _gnss_base_candidate_rows=[
            {
                "port": "/dev/ttyACM2",
                "target": "u-blox F9P",
                "baud": 115200,
                "status": "UBX detected",
                "messages": ("NAV-PVT", "NAV-SVIN"),
                "ubx": True,
            }
        ],
        gnss_network_connected=False,
        gnss_network_status_line="",
        latest_gnss_base_survey_in={
            "active": True,
            "valid": False,
            "duration_s": 75,
            "mean_accuracy_m": 0.0412,
            "observations": 812,
            "mean_x_m": -3041150.0,
            "mean_y_m": 4048200.0,
            "mean_z_m": 3867400.0,
        },
        gnss_base_survey_samples=(),
        gnss_coordinate_samples=(),
        gnss_fixed_base_profile=None,
        gnss_fixed_base_reconfigure_ack=False,
        gnss_fixed_base_reconfigure_readback=False,
        gnss_fixed_base_reconfigure_rtcm=False,
        gnss_base_rtcm_stats={
            "start_s": 5.0,
            "last_frame_s": 9.0,
            "frames": 50,
            "bytes": 4000,
            "injection": {
                "left": {"frames": 20, "packets": 30, "bytes": 1600, "fail": 0, "last_delivery_s": 9.0},
                "right": {"frames": 20, "packets": 30, "bytes": 1600, "fail": 1, "last_delivery_s": 8.5},
            },
        },
    )
    adapter = EvidenceAdapter(panel)
    projected = adapter.gnss_operation(now_s=10.0)
    assert projected["mode"] == "LOCAL_SURVEY_IN"
    assert projected["survey_in"]["mean_ecef_m"] == (-3041150.0, 4048200.0, 3867400.0)
    assert "ECEF [X,Y,Z]" in projected["survey_in"]["axis_contract"]
    assert "NED" in projected["survey_in"]["axis_contract"]
    assert projected["routes"]["left"]["delivery_age_s"] == pytest.approx(1.0)
    assert projected["base_candidates"][0]["port"] == "/dev/ttyACM2"
    assert projected["base_candidates"][0]["messages"] == "NAV-PVT, NAV-SVIN"
    assert projected["base_discovery_active"] is False
    assert projected["base_discovery_status"] == "FOUND 1 · UBX 1"

    applied: list[int] = []
    backend = SimpleNamespace(
        panel=panel,
        gnss_base_apply_candidate=lambda row: applied.append(row),
    )
    workspace = BaseGnssWorkspace(backend, lambda _command: True)
    workspace.refresh({})
    assert workspace.summary_values["mode"].value.text() == "Local Survey-in"
    assert workspace.survey_values["duration"].value.text().startswith("75 /")
    assert workspace.survey_values["observations"].value.text() == "812"
    assert workspace.survey_values["ecef"].value.text() == "-3041150.000/4048200.000/3867400.000"
    assert not workspace.survey_group.isHidden()
    assert workspace.ntrip_group.isHidden()
    assert workspace.candidate_model.rowCount() == 1
    assert workspace.discovery_status.value.text() == "FOUND 1 · UBX 1"
    assert workspace.candidate_model.item(0, 0).text() == "/dev/ttyACM2"
    assert workspace.candidate_model.item(0, 4).text() == "NAV-PVT, NAV-SVIN"
    workspace.candidate_table.selectRow(0)
    assert workspace.apply_candidate_button.isEnabled()
    workspace.apply_candidate_button.click()
    assert applied == [0]
    assert workspace.candidate_feedback.text() == "Applied Base receiver row 1"

    network_config = GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT)
    manager.configure(network_config)
    manager.set_source_identity(
        CorrectionSourceIdentity(
            source_kind="NTRIP",
            caster_host="caster.example.org",
            caster_port=2101,
            mountpoint="BASE01",
        )
    )
    panel.gnss_network_connected = True
    panel.gnss_network_status_line = "ICY 200 OK"
    workspace.refresh({})
    assert workspace.summary_values["mode"].value.text() == "Network RTK Direct"
    assert workspace.ntrip_values["caster"].value.text().startswith("caster.example.org:2101")
    assert workspace.ntrip_values["mountpoint"].value.text() == "BASE01"
    assert workspace.survey_group.isHidden()
    assert not workspace.ntrip_group.isHidden()
    workspace.deleteLater()
    app.processEvents()


def test_camera_and_pixhawk_discovery_candidates_are_visible_and_assignable() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    interface = SimpleNamespace(name="enp4s0", address="192.168.0.10")
    backend.panel._camera_candidate_rows = [
        {
            "kind": "NIC",
            "address": "enp4s0",
            "port": "",
            "status": "candidate",
            "interface": interface,
            "host_ip": "192.168.0.10",
        },
        {
            "kind": "WIRIS",
            "address": "192.168.0.20",
            "host": "192.168.0.20",
            "port": "23",
            "status": "HIWS OK",
            "interface": interface,
            "host_ip": "192.168.0.10",
        },
    ]
    backend.panel._pixhawk_candidate_rows_by_source = {
        "pc": [
            {
                "kind": "PC_IP",
                "value": "192.168.1.10",
                "status": "enp5s0",
                "interface": "enp5s0",
            }
        ],
        "mavlink": [
            {
                "kind": "MAVLink",
                "value": "udpin:0.0.0.0:14550",
                "baud": 115200,
                "status": "Ethernet candidate",
            }
        ],
    }
    backend.panel.workstation_camera_discovery_status = "FOUND 2 ENDPOINTS"
    backend.panel.workstation_pixhawk_discovery_status = "FOUND 2 ENDPOINTS"
    window = WorkstationMainWindow(backend=backend)

    window.camera.refresh(window.status_adapter.model.snapshot())
    assert window.camera.candidate_model.rowCount() == 2
    assert window.camera.discovery_status.value.text() == "FOUND 2 ENDPOINTS"
    camera_inspections: list[dict[str, object]] = []
    window.camera.inspection_requested.connect(camera_inspections.append)
    window.camera.candidate_table.selectRow(0)
    window.camera.candidate_table.clicked.emit(window.camera.candidate_model.index(0, 0))
    assert camera_inspections[-1]["selection"] == "Camera endpoint · NIC"
    assert "no image pixels" in str(camera_inspections[-1]["contract"])
    assert window.camera.candidate_buttons["nic"].isEnabled()
    assert not window.camera.candidate_buttons["left"].isEnabled()
    window.camera.candidate_buttons["nic"].click()
    assert backend.calls[-1] == ("camera_apply_candidate", 0, "nic")
    assert window.camera.candidate_feedback.text() == "Applied row 1 to NIC"
    window.camera.candidate_table.selectRow(1)
    assert window.camera.candidate_buttons["left"].isEnabled()
    window.camera.candidate_buttons["left"].click()
    assert backend.calls[-1] == ("camera_apply_candidate", 1, "left")

    window.pixhawk.refresh(window.status_adapter.model.snapshot())
    assert window.pixhawk.candidate_model.rowCount() == 2
    assert window.pixhawk.discovery_status.value.text() == "FOUND 2 ENDPOINTS"
    pixhawk_inspections: list[dict[str, object]] = []
    window.pixhawk.inspection_requested.connect(pixhawk_inspections.append)
    window.pixhawk.candidate_table.selectRow(0)
    window.pixhawk.candidate_table.clicked.emit(window.pixhawk.candidate_model.index(0, 0))
    assert pixhawk_inspections[-1]["selection"] == "Pixhawk endpoint · PC_IP"
    assert "no attitude" in str(pixhawk_inspections[-1]["contract"])
    assert window.pixhawk.candidate_buttons["host"].isEnabled()
    window.pixhawk.candidate_buttons["host"].click()
    assert backend.calls[-1] == ("pixhawk_apply_candidate", "pc", 0, "host")
    assert window.pixhawk.candidate_feedback.text() == "Applied PC_IP to HOST"
    window.pixhawk.candidate_table.selectRow(1)
    assert window.pixhawk.candidate_buttons["right"].isEnabled()
    window.pixhawk.candidate_buttons["right"].click()
    assert backend.calls[-1] == ("pixhawk_apply_candidate", "mavlink", 0, "right")

    backend.camera_apply_candidate = lambda _row, _target: False
    window.camera.candidate_table.selectRow(1)
    window.camera.candidate_buttons["right"].click()
    assert window.camera.candidate_feedback.text() == "ERROR: endpoint was not applied"

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_inspector_changes_by_context_and_selected_evidence() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.navigation.select_context("devices.cameras")
    app.processEvents()
    assert not window.inspector.context_section.isHidden()
    assert not window.inspector.findChildren(QtWidgets.QLineEdit)
    assert window.inspector.output_section.isHidden()
    assert "CAM L=" in window.inspector.status_value.value.text()
    assert "CAM R=" in window.inspector.status_value.value.text()
    assert window.inspector.context_section.title() == "Camera pair"
    assert window.inspector.context_row_labels["primary"].text() == "Pair"
    assert window.inspector.context_row_labels["contract"].text() == "Image contract"
    assert window.inspector.context_row_labels["primary"].objectName() == "engineeringLabel"
    assert window.inspector.context_values["primary"].label.isHidden()

    window.navigation.select_context("devices.time")
    app.processEvents()
    assert not window.inspector.status_section.isHidden()
    assert window.inspector.context_values["contract"].value.text().startswith("WIRIS RTCP/NTP")

    window.navigation.select_context("devices.pixhawk")
    app.processEvents()
    assert not window.inspector.status_section.isHidden()
    assert "P1=" in window.inspector.status_value.value.text()
    assert "P2=" in window.inspector.status_value.value.text()
    assert window.inspector.context_section.title() == "Pixhawk pair"
    window.pixhawk.attitude_table.clicked.emit(window.pixhawk.attitude_model.index(1, 0))
    app.processEvents()
    assert window.inspector.context_section.title() == "Selection details"
    assert window.inspector.context_values["primary"].value.text() == "P2 attitude"
    assert "body FRD" in window.inspector.context_values["contract"].value.text()
    window.inspector.refresh("devices.pixhawk", window.status_adapter.model)
    assert window.inspector.context_section.title() == "Selection details"
    assert window.inspector.context_values["primary"].value.text() == "P2 attitude"

    window.navigation.select_context("devices.gnss")
    app.processEvents()
    assert window.inspector.context_section.title() == "Base operation"
    assert window.inspector.context_row_labels["secondary"].text() == "Survey / NTRIP"
    assert "Base position WGS84" in window.inspector.context_values["contract"].value.text()
    assert "P1/P2=" not in window.inspector.context_values["primary"].value.text()
    window.base_gnss.route_table.clicked.emit(window.base_gnss.route_model.index(0, 0))
    app.processEvents()
    assert window.inspector.context_values["primary"].value.text() == "RTCM route · P1"
    assert "transport" in window.inspector.context_values["contract"].value.text()

    window.navigation.select_context("processing.alignment")
    app.processEvents()
    assert "Physical adjustment only" in window.inspector.context_values["verdict"].value.text()
    window.alignment.attitude_table.clicked.emit(window.alignment.attitude_model.index(0, 0))
    app.processEvents()
    assert window.inspector.context_values["primary"].value.text() == "Alignment input · P1 attitude"
    assert "body FRD" in window.inspector.context_values["contract"].value.text()

    window.navigation.select_context("processing.measurement")
    app.processEvents()
    assert not window.inspector.output_section.isHidden()
    window.measurement.values["forward"].activated.emit()
    app.processEvents()
    assert window.inspector.context_values["primary"].value.text() == "Measurement Forward"
    assert "baseline_center_frd" in window.inspector.context_values["contract"].value.text()
    window.measurement.values["forward"].set_value("4.200", unit="m")
    window.inspector.refresh("processing.measurement", window.status_adapter.model)
    assert window.inspector.context_values["secondary"].value.text() == "4.200 m"

    window.navigation.select_context("tasks.settings")
    app.processEvents()
    assert window.inspector.status_section.isHidden()
    assert "Credentials remain environment-variable references" in window.inspector.context_values["contract"].value.text()

    window.navigation.select_context("overview")
    app.processEvents()
    assert "Overview preserves each producer contract" in window.inspector.context_values["contract"].value.text()
    protocol_button = next(
        button
        for button in window.overview.findChildren(QtWidgets.QPushButton)
        if button.text() == "Protocol"
    )
    protocol_button.click()
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.experiment

    window.navigation.select_context("diagnostics.system")
    app.processEvents()
    assert window.inspector.context_label.text() == "Diagnostics · System"
    assert not window.inspector.open_advanced_button.isHidden()
    assert "status channels" in window.inspector.context_values["primary"].value.text()
    assert "aggregates state only" in window.inspector.context_values["contract"].value.text()

    window.navigation.select_context("diagnostics.logs")
    app.processEvents()
    assert window.inspector.context_values["verdict"].value.text() == "AVAILABLE"
    assert "[fake] source-backed log" in window.inspector.context_values["secondary"].value.text()

    window.navigation.select_context("diagnostics.gnss")
    app.processEvents()
    gnss_diagnostics = window.diagnostics.gnss_view.toPlainText()
    assert "Base operation (Base receiver / Survey-in / NTRIP / RTCM)" in gnss_diagnostics
    assert "Rover receiver projection (WGS84 geodetic; not NED baseline)" in gnss_diagnostics
    assert "Baseline source projection (NED north/east/down metres)" in gnss_diagnostics

    window.navigation.select_context("diagnostics.performance")
    app.processEvents()
    assert "rates are hertz" in window.inspector.context_values["contract"].value.text()

    window.navigation.select_context("diagnostics.measurement")
    app.processEvents()
    assert not window.inspector.status_section.isHidden()
    assert window.inspector.status_value.value.text() != "UNKNOWN"

    window.navigation.select_context("tasks.advanced")
    app.processEvents()
    assert window.inspector.open_advanced_button.isHidden()

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_workstation_recording_gate_protects_preview_mapping_and_save() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    backend.panel.recording_active = True
    backend.panel.recording_session_id = "active-session"
    window.status_adapter.refresh()
    app.processEvents()

    assert not window.commands.action("camera.toggle").isEnabled()
    assert not window.commands.action("time.set_pc").isEnabled()
    assert not window.commands.action("camera.swap").isEnabled()
    assert not window.commands.action("camera.auto_connect").isEnabled()
    assert not window.commands.action("alignment.swap_pixhawks").isEnabled()
    assert not window.commands.action("pixhawk.toggle_mavlink").isEnabled()
    assert window.commands.action("detection.toggle").isEnabled()
    assert not window.commands.action("session.save").isEnabled()
    assert not window.measurement.control_buttons["camera.toggle"].isEnabled()
    assert window.measurement.control_buttons["detection.toggle"].isEnabled()
    assert not window.measurement.control_buttons["session.save"].isEnabled()
    assert not window.time_workspace.operation_buttons["time.set_pc"].isEnabled()
    assert not window.camera.operation_buttons["camera.swap"].isEnabled()
    assert not window.alignment.operation_buttons["alignment.swap_pixhawks"].isEnabled()
    calls_before_disabled_invoke = list(backend.calls)
    assert window.commands.invoke("camera.swap") is False
    assert window.commands.invoke("camera.auto_connect") is False
    assert window.commands.invoke("alignment.swap_pixhawks") is False
    assert window.commands.invoke("pixhawk.toggle_mavlink") is False
    assert backend.calls == calls_before_disabled_invoke

    backend.panel.camera_connected = True
    window.status_adapter.refresh()
    app.processEvents()
    assert not window.commands.action("detection.toggle").isEnabled()
    assert not window.measurement.control_buttons["detection.toggle"].isEnabled()

    backend.panel.recording_active = False
    backend.panel.camera_connected = False
    backend.panel._session_save_finalized = True
    window.status_adapter.refresh()
    app.processEvents()
    assert window.commands.action("camera.toggle").isEnabled()
    assert window.commands.action("time.set_pc").isEnabled()
    assert window.commands.action("session.save").isEnabled()
    assert window.measurement.control_buttons["session.save"].isEnabled()

    backend.panel.wiris_gdti_thread = object()
    window._refresh_from_status(window.status_adapter.model.snapshot())
    app.processEvents()
    assert not window.commands.action("recording.toggle").isEnabled()
    assert not window.measurement.control_buttons["recording.toggle"].isEnabled()
    backend.panel.wiris_gdti_thread = None

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_detection_mode_switch_is_rejected_while_live_camera_is_recording(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.distance_recording_enabled = True
    panel.camera_connected = True
    starts: list[tuple[bool, bool]] = []
    monkeypatch.setattr(
        panel,
        "_start_camera_streams",
        lambda *, enable_detection, switch_to_depth: starts.append((enable_detection, switch_to_depth)),
    )

    assert panel.toggle_yolo_detection() is False
    assert starts == []
    assert "세션 기록 중" in panel.camera_yolo_log.toPlainText()

    panel.camera_connected = False
    assert panel.toggle_yolo_detection() is None
    assert starts == [(True, True)]

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_recording_owner_rejects_camera_swap_and_clock_reconnect(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.distance_recording_enabled = True
    left_url = panel.left_url_edit.text()
    right_url = panel.right_url_edit.text()

    assert panel.swap_cameras() is False
    assert panel.left_url_edit.text() == left_url
    assert panel.right_url_edit.text() == right_url
    assert panel.set_selected_wiris_time_from_pc() is False
    assert panel.wiris_time_sync_thread is None
    assert panel.swap_pixhawks() is False
    assert panel.toggle_mavlink_connection() is False
    assert panel.auto_connect_wiris() is False
    assert panel.auto_wiris_connect_pending is False
    panel._camera_candidate_rows = [{"kind": "RTSP", "address": "rtsp://candidate", "host": "10.0.0.50"}]
    assert panel.apply_camera_candidate(0, "left") is False
    panel.auto_wiris_connect_pending = True
    original_left_url = panel.left_url_edit.text()
    panel._apply_camera_detection({"mode": "auto_connect", "rtsp": [], "control": [], "verified_control": []})
    assert panel.auto_wiris_connect_pending is False
    assert panel.left_url_edit.text() == original_left_url
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
    panel._apply_wiris_gdti_result({})
    assert starts == []
    assert panel.camera_sync_status == standalone_gui.CAMERA_SYNC_STATUS_NOT_VERIFIED

    panel.distance_recording_enabled = False
    panel.wiris_gdti_thread = object()
    assert panel._start_session_recording() is False
    assert panel.recording_session_id == ""
    panel.wiris_gdti_thread = None

    panel.pixhawk_connection_thread = object()
    assert panel._start_session_recording() is False
    assert panel.recording_session_id == ""
    panel.pixhawk_connection_thread = None

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_recording_preview_updates_in_place_and_key_value_rows_do_not_show_sample_detail(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    table = QtWidgets.QTableWidget()
    panel._fill_recording_sheet(table, ("a", "b"), (("1", "2"), ("3", "4")))
    table.selectRow(0)
    original_item = table.item(0, 0)

    panel._fill_recording_sheet(table, ("a", "b"), (("updated", "2"), ("3", "4")))
    assert table.item(0, 0) is original_item
    assert table.item(0, 0).text() == "updated"
    assert table.selectionModel().selectedRows()[0].row() == 0

    panel.distance_recording_rows = [{"distance_m": 12.3}]
    summary_index = panel.recording_sheet_tabs.indexOf(panel.recording_sheet_tables["Summary"])
    panel.recording_sheet_tabs.setCurrentIndex(summary_index)
    summary_table = panel.recording_sheet_tables["Summary"]
    panel._fill_recording_sheet(summary_table, ("key", "value"), (("session", "test"),))
    summary_table.selectRow(0)
    panel._refresh_recording_sample_detail()
    assert panel.recording_sample_detail.toPlainText() == ""

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    table.deleteLater()
    app.processEvents()


def test_workstation_exposes_device_discovery_and_routes_tools_to_advanced() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    assert window.camera.operation_buttons["camera.toggle"].text() == "Preview"
    assert window.camera.operation_buttons["camera.auto_connect"].text() == "Connect cameras"
    assert window.camera.operation_buttons["advanced.open"].text() == "Advanced"
    assert window.camera.preview_splitter.minimumHeight() == 320
    assert window.camera.workspace_splitter.orientation() == QtCore.Qt.Vertical
    assert window.camera.details_scroll.widgetResizable() is True
    assert window.camera.preview_pair_orientation(1600, 700) == QtCore.Qt.Horizontal
    assert window.camera.preview_pair_orientation(900, 700) == QtCore.Qt.Vertical
    assert window.measurement.preview_splitter.minimumHeight() == 280
    assert window.measurement.workspace_splitter.orientation() == QtCore.Qt.Vertical
    assert window.measurement.details_scroll.widgetResizable() is True
    assert window.measurement.preview_pair_orientation(1600, 700) == QtCore.Qt.Horizontal
    assert window.measurement.preview_pair_orientation(900, 700) == QtCore.Qt.Vertical
    for command_id in (
        "camera.auto_connect",
        "pixhawk.discover_usb",
        "pixhawk.discover_mavlink_candidates",
        "pixhawk.discover_host_ip",
    ):
        assert window.commands.invoke(command_id) is True

    assert backend.calls[-4:] == [
        "camera_auto_connect",
        "pixhawk_discover_usb",
        "pixhawk_discover_mavlink_candidates",
        "pixhawk_discover_host_ip",
    ]
    assert window.commands.invoke("advanced.open") is True
    assert window.workspace_host.currentWidget().widget() is window.advanced
    assert "legacy" not in window._workspace_indexes

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_measurement_workspace_overlays_bbox_on_connected_preview() -> None:
    from PyQt5 import QtGui

    from Workstation.status_model import StatusModel
    from Workstation.workspaces import MeasurementWorkspace
    from Workstation.workstation_core.stereo import DetectionObservation

    class FakePane(QtCore.QObject):
        frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
        detection_changed = QtCore.pyqtSignal(str, object)
        stats_changed = QtCore.pyqtSignal(str, object)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    left_pane = FakePane()
    right_pane = FakePane()
    backend = SimpleNamespace(
        panel=SimpleNamespace(
            left_pane=left_pane,
            right_pane=right_pane,
            settings=SimpleNamespace(),
        )
    )
    workspace = MeasurementWorkspace(backend, StatusModel())
    preview = workspace.previews["left"]
    preview.resize(400, 300)
    preview.show()
    image = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    left_pane.frame_changed.emit("left", image, {"timestamp_s": 1.0})
    app.processEvents()
    without_box = preview.image.pixmap().toImage().copy()

    detection = DetectionObservation(
        camera="left",
        timestamp_s=1.0,
        width=640,
        height=480,
        x1=100.0,
        y1=100.0,
        x2=300.0,
        y2=300.0,
        confidence=0.9,
        class_id=0,
        class_name="person",
    )
    left_pane.detection_changed.emit("left", detection)
    app.processEvents()
    with_box = preview.image.pixmap().toImage().copy()

    assert with_box != without_box
    assert preview._detection is detection

    left_pane.detection_changed.emit("left", None)
    app.processEvents()
    assert preview.image.pixmap().toImage().copy() == without_box

    workspace.deleteLater()
    left_pane.deleteLater()
    right_pane.deleteLater()
    app.processEvents()


def test_workstation_shell_smoke_uses_standalone_backend_without_devices() -> None:
    from Workstation.workstation_core.settings import AppSettings

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    source = _FakeBackend()
    panel = QtWidgets.QWidget()
    panel.__dict__.update(source.panel.__dict__)
    calls: list[tuple[str, tuple[object, ...]]] = []

    def record(name: str):
        def handler(*args: object, **_kwargs: object) -> None:
            calls.append((name, args))

        return handler

    for method_name in (
        "toggle_camera_connection",
        "toggle_yolo_detection",
        "toggle_session_recording",
        "save_distance_history_csv",
        "auto_connect_wiris",
        "discover_usb_pixhawk_connections",
        "discover_mavlink_url_candidates",
        "discover_pc_pixhawk_ip",
        "save_current_settings_from_ui",
        "apply_workstation_measurement_settings",
    ):
        setattr(panel, method_name, record(method_name))
    panel.workstation_settings_snapshot = lambda: {
        **dict(AppSettings().__dict__),
        "gnss_operation": AppSettings().gnss_operation.to_mapping(),
    }
    panel.workstation_wiris_command_catalog = lambda: (
        {"category": "Basic", "label": "Check", "command": "HIWS", "terminator": "\n"},
    )
    panel.shutdown_for_close = lambda: True

    backend = StandalonePanelBackend(panel)
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    assert panel.parent() is window._owner_host
    assert window._owner_host.isHidden()
    assert not hasattr(window, "legacy_page")
    assert window.commands.invoke("camera.auto_connect") is True
    assert window.commands.invoke("pixhawk.discover_usb") is True
    assert window.commands.invoke("settings.save") is True
    assert [name for name, _args in calls] == [
        "auto_connect_wiris",
        "discover_usb_pixhawk_connections",
        "save_current_settings_from_ui",
    ]

    window.navigation.select_context("tasks.recording")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.recording
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_command_registry_distinguishes_deferred_completion() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    registry = CommandRegistry()
    events: list[tuple[str, str]] = []
    registry.accepted.connect(lambda command_id, _result: events.append(("accepted", command_id)))
    registry.completed.connect(lambda command_id, _result: events.append(("completed", command_id)))
    registry.register(CommandSpec("sync", "Sync", "", "test", lambda: "done"))
    registry.register(CommandSpec("async", "Async", "", "test", lambda: None))
    disabled_calls: list[str] = []
    registry.register(
        CommandSpec("disabled", "Disabled", "", "test", lambda: disabled_calls.append("called"))
    )
    registry.set_deferred_completion("async")
    registry.set_enabled("disabled", False)

    assert registry.invoke("sync") is True
    assert registry.invoke("async") is True
    assert registry.invoke("disabled") is False
    assert disabled_calls == []
    assert events == [
        ("accepted", "sync"),
        ("completed", "sync"),
        ("accepted", "async"),
    ]
    registry.deleteLater()
    app.processEvents()


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


def test_wiris_command_and_zoom_workers_keep_tcp_io_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import wiris_runtime_workers

    sent: list[str] = []

    def fake_send(command) -> str:
        sent.append(str(command.command))
        return "OK"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    command_results: list[object] = []
    command_worker = standalone_gui.WirisCommandWorker(
        (("left", "10.0.0.11", 5000, 1.0),),
        command_text="HIWS",
        terminator="\n",
    )
    command_worker.completed.connect(command_results.append)
    command_worker.run()

    zoom_results: list[object] = []
    zoom_worker = standalone_gui.WirisZoomWorker(
        (("right", "10.0.0.12", 5001),),
        template="SZIN",
        level=None,
        timeout_s=1.0,
    )
    zoom_worker.completed.connect(zoom_results.append)
    zoom_worker.run()

    assert sent == ["HIWS", "SZIN"]
    assert command_results[0]["statuses"] == {"left": "OK"}
    assert zoom_results == [[{"label": "right", "state": "sent", "command": "SZIN", "response": "OK"}]]


def test_pixhawk_usb_discovery_worker_keeps_serial_probe_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers, "usb_mavlink_device_candidates", lambda: ["/dev/ttyACM0"])
    monkeypatch.setattr(
        device_runtime_workers,
        "probe_usb_mavlink_devices",
        lambda *, candidates, timeout_s: [(candidates[0], 115200)],
    )
    results: list[object] = []
    worker = standalone_gui.PixhawkUsbDiscoveryWorker(timeout_s=0.8)
    worker.discovered.connect(results.append)
    worker.run()

    assert results == [{"candidates": ["/dev/ttyACM0"], "found": [("/dev/ttyACM0", 115200)]}]


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


def test_camera_time_sync_gdti_thread_uses_panel_qobject_parent(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller

    class FakeWorker(QtCore.QObject):
        completed = QtCore.pyqtSignal(object)
        failed = QtCore.pyqtSignal(str)
        finished = QtCore.pyqtSignal()

        def __init__(self, *_args, **_kwargs) -> None:
            super().__init__()

        def run(self) -> None:
            self.finished.emit()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = QtWidgets.QWidget()
    panel.wiris_gdti_thread = None
    panel.wiris_gdti_worker = None
    panel.wiris_command_status_label = QtWidgets.QLabel()
    panel._log = lambda _text: None
    panel._apply_wiris_gdti_result = lambda _result: None
    panel._wiris_gdti_failed = lambda _error: None
    panel._clear_wiris_gdti_worker = lambda: None
    monkeypatch.setattr(camera_time_sync_controller, "WirisClockCommandWorker", FakeWorker)

    controller = camera_time_sync_controller.CameraTimeSyncController(panel)
    controller._start_wiris_gdti_worker((("left", "127.0.0.1", 5000, 1.0),))
    thread = panel.wiris_gdti_thread
    assert thread is not None
    assert thread.parent() is panel
    thread.quit()
    thread.wait(1000)
    panel.deleteLater()
    app.processEvents()


def test_wiris_stus_compensation_rejects_invalid_rtt() -> None:
    from Workstation import standalone_gui

    assert standalone_gui.WirisClockCommandWorker._stus_one_way_delay_s(0.012) == pytest.approx(0.006)
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(0.012) == pytest.approx(0.008)
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(None) == 0.0
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(float("nan")) == 0.0
    assert standalone_gui.WirisClockCommandWorker._stus_compensation_s(0.251) == 0.0


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


def test_interface_status_worker_keeps_startup_ip_query_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers, "interface_addresses", lambda name: f"{name} UP")
    results: list[tuple[str, str]] = []
    worker = standalone_gui.InterfaceStatusWorker("enx-camera")
    worker.ready.connect(lambda name, status: results.append((name, status)))
    worker.run()

    assert results == [("enx-camera", "enx-camera UP")]


def test_interface_status_worker_ignores_late_signal_after_qt_teardown(monkeypatch) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui, "interface_addresses", lambda name: f"{name} UP")
    worker = standalone_gui.InterfaceStatusWorker("enx-camera")
    monkeypatch.setattr(worker, "_emit_if_alive", lambda *_args: False)

    worker.run()


def test_session_save_worker_ignores_late_signal_only_after_teardown(monkeypatch, tmp_path) -> None:
    from Workstation import standalone_gui
    from Workstation import session_workers

    saved = object()
    monkeypatch.setattr(session_workers, "save_session_files", lambda *_args, **_kwargs: saved)
    worker = standalone_gui.SessionSaveWorker(
        output_dir=tmp_path,
        metadata={},
        pair_rows=(),
        camera_rows={},
        px4_rows={},
    )
    emitted: list[object] = []
    worker.saved.connect(emitted.append)
    worker.run()
    assert emitted == [saved]

    monkeypatch.setattr(worker, "_emit_if_alive", lambda *_args: False)
    worker.run()


def test_checkerboard_capture_worker_saves_and_validates_optical_pixel_images(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui
    from Workstation import calibration_workers
    from Workstation.workstation_core.calibration import CheckerboardConfig

    monkeypatch.setattr(
            calibration_workers,
        "detect_checkerboard",
        lambda _path, _config: SimpleNamespace(found=True, corner_count=48, error=""),
    )
    frames = {}
    for side in ("left", "right"):
        image = standalone_gui.QtGui.QImage(32, 24, standalone_gui.QtGui.QImage.Format_RGB888)
        image.fill(127)
        frames[side] = image
    worker = standalone_gui.CheckerboardCaptureWorker(
        root=tmp_path,
        frames=frames,
        stem="001",
        config=CheckerboardConfig(inner_cols=8, inner_rows=6, square_size_m=0.03),
        opencv_error=None,
        pair_timing_ok=True,
        pair_time_basis="rtcp-ntp",
        pair_delta_s=0.002,
        pair_timing_detail="ok",
        metadata_payload={
            "frame_contract": "OpenCV image pixels; stereo timing uses RTCP/NTP camera timestamp"
        },
    )
    completed: list[object] = []
    worker.completed.connect(completed.append)

    worker.run()

    assert len(completed) == 1
    assert completed[0]["saved_sides"] == ("left", "right")
    assert (tmp_path / "left" / "left_001.png").is_file()
    assert (tmp_path / "right" / "right_001.png").is_file()
    metadata = (tmp_path / "capture_metadata" / "capture_001.json").read_text(encoding="utf-8")
    assert "OpenCV image pixels" in metadata
    assert '"stereo_pair_saved": true' in metadata


def test_checkerboard_calibration_worker_preserves_optical_rdf_stereo_contract(monkeypatch, tmp_path) -> None:
    from Workstation import standalone_gui
    from Workstation import calibration_workers
    from Workstation.workstation_core.calibration import (
        CameraIntrinsics,
        CheckerboardConfig,
        StereoExtrinsics,
    )

    config = CheckerboardConfig(inner_cols=8, inner_rows=6, square_size_m=0.03)
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=4,
    )
    extrinsics = StereoExtrinsics(
        rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        translation_left_to_right_m=[-0.3, 0.0, 0.0],
        reprojection_error_px=0.1,
        pair_count=4,
    )
    calls: list[tuple[str, tuple[Path, ...]]] = []
    monkeypatch.setattr(
        calibration_workers,
        "calibrate_camera_from_images",
        lambda paths, _config: calls.append(("intrinsics", tuple(paths))) or intrinsics,
    )
    monkeypatch.setattr(
        calibration_workers,
        "calibrate_stereo_extrinsics_from_image_pairs",
        lambda left, right, _config, _left, _right: calls.append(
            ("stereo", (*tuple(left), *tuple(right)))
        )
        or extrinsics,
    )
    left_paths = [tmp_path / f"left_{index}.png" for index in range(4)]
    right_paths = [tmp_path / f"right_{index}.png" for index in range(4)]
    worker = standalone_gui.CheckerboardCalibrationWorker(
        mode="checkerboard",
        left_paths=left_paths,
        right_paths=right_paths,
        config=config,
    )
    completed: list[object] = []
    worker.completed.connect(completed.append)

    worker.run()

    assert len(completed) == 1
    result = completed[0]
    assert result.extrinsics is extrinsics
    assert "opencv_optical:RDF" in result.extrinsics.frame_contract
    assert result.extrinsics.translation_left_to_right_m == [-0.3, 0.0, 0.0]
    assert [kind for kind, _paths in calls] == ["intrinsics", "intrinsics", "stereo"]

    calls.clear()
    validation_worker = standalone_gui.CheckerboardRandomValidationWorker(
        mode="checkerboard",
        pairs=[(str(index), left_paths[index], right_paths[index]) for index in range(4)],
        config=config,
        runs=3,
        subset_count=3,
        seed=7,
    )
    validation_results: list[object] = []
    validation_worker.completed.connect(validation_results.append)

    validation_worker.run()

    assert len(validation_results) == 1
    assert len(validation_results[0]) == 3
    assert all(row["result"].extrinsics is extrinsics for row in validation_results[0])
    assert [kind for kind, _paths in calls].count("stereo") == 3


def test_workstation_calibration_and_validation_run_off_ui_thread(monkeypatch, tmp_path) -> None:
    from Workstation import standalone_gui
    from Workstation import calibration_workers
    from Workstation.workstation_core.calibration import CameraIntrinsics, StereoExtrinsics

    monkeypatch.setattr(standalone_gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    monkeypatch.setattr(standalone_gui, "opencv_available", lambda: (True, None))
    monkeypatch.setattr(standalone_gui, "save_settings", lambda _settings: None)
    root = tmp_path / "calibration"
    for side in ("left", "right"):
        directory = root / side
        directory.mkdir(parents=True)
        for index in range(4):
            (directory / f"{side}_{index:03d}.png").write_text("fixture", encoding="utf-8")

    def fake_intrinsics(paths, _config):
        return CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.0],
            reprojection_error_px=0.2,
            image_count=len(paths),
        )

    def fake_extrinsics(left_paths, _right_paths, _config, _left, _right):
        return StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.3, 0.0, 0.0],
            reprojection_error_px=0.1,
            pair_count=len(left_paths),
        )

    monkeypatch.setattr(calibration_workers, "calibrate_camera_from_images", fake_intrinsics)
    monkeypatch.setattr(calibration_workers, "calibrate_stereo_extrinsics_from_image_pairs", fake_extrinsics)
    monkeypatch.setattr(
        calibration_workers,
        "detect_checkerboard",
        lambda _path, _config: SimpleNamespace(found=True, corner_count=48, error=""),
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel()
    panel.calib_dir_edit.setText(str(root))
    panel.checkerboard_validation_root_edit.setText(str(root))
    panel.checkerboard_validation_mode_combo.setCurrentIndex(
        panel.checkerboard_validation_mode_combo.findData("checkerboard")
    )

    image = standalone_gui.QtGui.QImage(32, 24, standalone_gui.QtGui.QImage.Format_RGB888)
    image.fill(127)
    panel.left_pane.current_image = lambda: image
    panel.right_pane.current_image = lambda: image
    panel.camera_frame_metadata["left"] = {"rtcp_frame_ntp_unix_s": 100.000}
    panel.camera_frame_metadata["right"] = {"rtcp_frame_ntp_unix_s": 100.002}
    panel.capture_calibration_images()
    assert panel.checkerboard_calibration_thread is not None
    deadline = time.monotonic() + 2.0
    while panel.checkerboard_calibration_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        QtCore.QThread.msleep(5)
    assert panel.checkerboard_calibration_thread is None
    assert (root / "left" / "left_004.png").is_file()
    assert panel.capture_calibration_button.isEnabled()

    panel.run_intrinsic_calibration()
    assert panel.checkerboard_calibration_thread is not None
    assert EvidenceAdapter(panel).calibration()["state"] == "CALCULATING"
    deadline = time.monotonic() + 2.0
    while panel.checkerboard_calibration_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        QtCore.QThread.msleep(5)
    assert panel.checkerboard_calibration_thread is None
    assert panel.checkerboard_validation_result is not None
    assert EvidenceAdapter(panel).calibration()["state"] == "AVAILABLE"
    assert "T_left_to_right_m" in panel.calibration_result_text.toPlainText()

    panel.checkerboard_validation_random_runs_spin.setValue(2)
    panel.checkerboard_validation_random_pair_count_spin.setValue(3)
    panel.run_checkerboard_random_validation()
    assert panel.checkerboard_calibration_thread is not None
    deadline = time.monotonic() + 2.0
    while panel.checkerboard_calibration_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        QtCore.QThread.msleep(5)
    assert panel.checkerboard_calibration_thread is None
    assert "Random stereo-pair validation" in panel.calibration_result_text.toPlainText()

    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_pixhawk_host_ip_discovery_worker_keeps_interface_scan_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    interfaces = (SimpleNamespace(name="enx-pixhawk", address="10.41.10.1/24"),)
    monkeypatch.setattr(device_runtime_workers, "local_ipv4_interfaces", lambda *, include_wireless: interfaces)
    results: list[object] = []
    worker = standalone_gui.PixhawkHostIpDiscoveryWorker()
    worker.discovered.connect(results.append)
    worker.run()

    assert results == [interfaces]


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


def test_pixhawk_shell_and_parameter_workers_use_exclusive_command_path() -> None:
    from contextlib import contextmanager

    from Workstation import standalone_gui
    from Workstation.workstation_core.pixhawk_attitude import MavlinkShellCommandResult
    from Workstation.workstation_core.px4_mag_manager import MAG_PARAM_NAMES

    class FakeSubscriber:
        def __init__(self) -> None:
            self.connections = {"left": object()}
            self.params = {name: None for name in MAG_PARAM_NAMES}
            self.params.update(
                {
                    "UAVCAN_ENABLE": 2,
                    "UAVCAN_SUB_MAG": 1,
                    "CAL_MAG0_ID": standalone_gui.BOARD_INTERNAL_MAG_ID,
                    "CAL_MAG0_PRIO": 75,
                }
            )
            self.active: list[str] = []

        @contextmanager
        def exclusive_command(self, camera: str):
            self.active.append(camera)
            try:
                yield
            finally:
                self.active.pop()

        def send_shell_command_with_result(self, camera: str, command: str, *, timeout_s: float):
            assert self.active == [camera]
            return MavlinkShellCommandResult(command, "nsh> OK", "respond+exclusive")

        def read_param_int(self, camera: str, name: str, *, timeout_s: float):
            assert self.active == [camera]
            return self.params.get(name)

        def read_param(self, camera: str, name: str, *, timeout_s: float):
            assert self.active == [camera]
            return self.params.get(name)

        def set_param_int(self, camera: str, name: str, value: int, *, timeout_s: float):
            assert self.active == [camera]
            self.params[name] = int(value)
            return int(value)

        def reboot_autopilot(self, camera: str) -> None:
            assert self.active == [camera]

    subscriber = FakeSubscriber()
    shell_results: list[object] = []
    shell_worker = standalone_gui.PixhawkShellCommandWorker(
        subscriber,
        camera="left",
        command="param show",
    )
    shell_worker.completed.connect(shell_results.append)
    shell_worker.run()
    assert shell_results[0]["output"] == "nsh> OK"

    parameter_results: list[object] = []
    parameter_worker = standalone_gui.PixhawkParameterWorker(
        subscriber,
        camera="left",
        operation="mag_apply",
        priorities={"BOARD": 100},
    )
    parameter_worker.completed.connect(parameter_results.append)
    parameter_worker.run()
    assert parameter_results[0]["applied"] == ("BOARD:CAL_MAG0_PRIO=100",)
    assert subscriber.params["CAL_MAG0_PRIO"] == 100


def test_pixhawk_connection_and_stream_setup_workers_keep_transport_io_off_ui_path() -> None:
    from Workstation import standalone_gui

    class FakeSubscriber:
        def __init__(self) -> None:
            self.connections = {}
            self.start_args = None
            self.setup_args = None

        def start(self, camera, *, start_timer, apply_pending_setup):
            self.start_args = (camera, start_timer, apply_pending_setup)
            self.connections["left"] = object()

        def start_receive_timer(self):
            raise AssertionError("timer activation belongs to the UI completion handler")

        def configure_mavlink_streams(self, **kwargs):
            self.setup_args = kwargs

    subscriber = FakeSubscriber()
    connection_results: list[object] = []
    connection_worker = standalone_gui.PixhawkConnectionWorker(subscriber, "left")
    connection_worker.completed.connect(connection_results.append)
    connection_worker.run()

    setup_results: list[object] = []
    setup_worker = standalone_gui.PixhawkStreamSetupWorker(
        subscriber,
        host_ip="10.41.10.1",
        ports={"left": 14550, "right": 14551},
        sysids={"left": 1, "right": 2},
        rate=2_000_000,
        cameras=("left",),
    )
    setup_worker.completed.connect(setup_results.append)
    setup_worker.run()

    assert subscriber.start_args == ("left", False, False)
    assert subscriber.setup_args["host_ip"] == "10.41.10.1"
    assert setup_results[0]["cameras"] == ("left",)
    assert connection_results[0]["connections"] == ("left",)


def test_pixhawk_usb_bootstrap_worker_keeps_usb_shell_and_cleanup_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers
    from Workstation.workstation_core.pixhawk_attitude import MavlinkShellCommandResult

    instances: list[object] = []

    class FakeConnection:
        def wait_heartbeat(self, *, timeout):
            return object()

    class FakeSubscriber:
        def __init__(self, *args, **kwargs) -> None:
            self.connections = {}
            self.closed = False
            self.commands: list[str] = []
            instances.append(self)

        def start(self, camera, *, start_timer, apply_pending_setup):
            self.connections[camera] = FakeConnection()

        def stop(self, camera=None):
            self.closed = True
            self.connections.clear()

        @contextmanager
        def exclusive_command(self, camera):
            yield

        def send_shell_command_with_result(self, camera, command, *, timeout_s):
            self.commands.append(command)
            return MavlinkShellCommandResult(command, "OK", "respond+exclusive")

    monkeypatch.setattr(device_runtime_workers, "DualPixhawkMavlinkSubscriber", FakeSubscriber)
    results: list[object] = []
    worker = standalone_gui.PixhawkUsbBootstrapWorker(
        camera="left",
        usb_url="/dev/ttyACM0",
        baud_candidates=(115200,),
        command="ifup eth0",
        fallback_command="ifup eth0 -m onboard",
    )
    worker.completed.connect(results.append)
    worker.run()

    assert results[0]["camera"] == "left"
    assert results[0]["baud"] == 115200
    assert instances[0].commands == ["ifup eth0"]
    assert instances[0].closed is True


def test_pixhawk_persistent_config_worker_serializes_shell_and_ftp() -> None:
    from Workstation import standalone_gui
    from Workstation.workstation_core.pixhawk_attitude import MavlinkShellCommandResult

    class FakeSubscriber:
        connections = {"left": object()}

        def __init__(self) -> None:
            self.active = False
            self.calls: list[tuple[str, str]] = []

        @contextmanager
        def exclusive_command(self, camera):
            assert self.active is False
            self.active = True
            try:
                yield
            finally:
                self.active = False

        def send_shell_command_with_result(self, camera, command, *, timeout_s):
            assert self.active is True
            self.calls.append(("shell", command))
            return MavlinkShellCommandResult(command, "saved", "respond+exclusive")

        def write_remote_file_mavftp(self, camera, path, content):
            assert self.active is True
            self.calls.append((path, content))

    subscriber = FakeSubscriber()
    results: list[object] = []
    worker = standalone_gui.PixhawkPersistentConfigWorker(
        subscriber,
        camera="left",
        command="param save",
        extras_text="ifup eth0\n",
        net_cfg_text="IPADDR=10.41.10.2\n",
    )
    worker.completed.connect(results.append)
    worker.run()

    assert [name for name, _value in subscriber.calls] == [
        "shell",
        "/fs/microsd/etc/extras.txt",
        "/fs/microsd/net.cfg",
    ]
    assert results[0]["camera"] == "left"


def test_pixhawk_rtcm_worker_queues_injection_and_reports_partial_failure(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    class FakeSubscriber:
        def __init__(self) -> None:
            self.sent: list[tuple[str, bytes]] = []

        @contextmanager
        def exclusive_command(self, camera):
            yield

        def send_rtcm_data(self, camera, payload):
            self.sent.append((camera, payload))
            if camera == "right":
                raise TimeoutError("right unavailable")
            return 3

    monkeypatch.setattr(device_runtime_workers.QtCore.QTimer, "singleShot", lambda *_args: None)
    subscriber = FakeSubscriber()
    results: list[object] = []
    worker = standalone_gui.PixhawkRtcmInjectionWorker(subscriber, max_queue=2)
    worker.injected.connect(results.append)
    worker.enqueue(("left", "right"), b"RTCM")

    assert subscriber.sent == [("left", b"RTCM"), ("right", b"RTCM")]
    assert results == [
        {
            "payload_bytes": 4,
            "sent": (("left", 3),),
            "failed": (("right", "right unavailable"),),
            "dropped": 0,
        }
    ]


def test_workstation_uses_explicit_camera_coordinate_label() -> None:
    source = (Path(__file__).resolve().parents[2] / "Workstation" / "standalone_gui.py").read_text(
        encoding="utf-8"
    )
    assert "OpenCV optical RDF" in source
    assert "ROS Optical Frame" not in source


def test_workstation_shutdown_orders_runtime_stop_before_settings_save(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui, "APP_ROOT", tmp_path)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel()
    assert panel.app_root == tmp_path.resolve()
    assert panel.settings_path == (tmp_path / "stereo_distance_settings.json").resolve()
    assert tuple(step.name for step in panel._shutdown_coordinator.steps) == (
        "stop-runtime-workers",
        "stop-gnss-network",
        "stop-gnss-base-serial",
        "stop-auxiliary-workers",
        "wait-session-save",
        "save-settings",
    )
    panel.close()
    app.processEvents()
    assert panel.settings_path.is_file()
    panel.deleteLater()
    app.processEvents()


def test_workstation_shutdown_requests_workers_without_blocking_ui_thread(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    stop_calls: list[bool] = []
    panel.stop_all = lambda *, wait=False: stop_calls.append(wait) or False

    assert panel.shutdown_for_close() is False
    assert stop_calls == [False]

    thread_calls: list[str] = []

    class RunningThread:
        def quit(self) -> None:
            thread_calls.append("quit")

        def isRunning(self) -> bool:  # noqa: N802 - Qt-compatible test double.
            return True

        def wait(self, _timeout_ms: int) -> bool:
            raise AssertionError("non-blocking shutdown must not call QThread.wait")

    panel.test_shutdown_thread = RunningThread()
    assert panel._shutdown_stop_thread("test_shutdown_thread") is False
    assert thread_calls == ["quit"]

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()


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
    assert panel.camera_connected is True
    assert panel.yolo_enabled is True

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
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
    assert settings.tabs.count() == 5
    assert not any(
        "Units/frames" in label.text()
        for label in settings.findChildren(QtWidgets.QLabel)
    )
    assert {
        "interface_name",
        "left_rtsp_url",
        "yolo_weights_path",
        "rtsp_transport",
        "pixhawk_host_address",
        "pixhawk_ethernet_mode",
        "intrinsics_path",
        "gnss_operation_mode",
        "gnss_network_caster_host",
        "gnss_network_tls",
        "gnss_readiness_maximum_baseline_std_m",
        "gnss_fixed_coordinate_profile_path",
    } <= set(settings.settings_widgets)
    assert not {
        "ros_domain_id",
        "left_namespace",
        "right_namespace",
        "mavlink_configure_uxrce_agent",
        "uxrce_port",
        "clock_ftp_password",
        "gnss_require_drone_rtk_fixed",
        "gnss_route_drone_enabled",
        "gnss_route_field_base_enabled",
    } & set(settings.settings_widgets)
    assert settings.settings_widgets["gnss_readiness_minimum_fixed_duration_s"].value() == pytest.approx(30.0)
    assert settings.settings_widgets["gnss_readiness_maximum_baseline_std_m"].value() == pytest.approx(0.02)

    settings.settings_widgets["interface_name"].setText("enx-workstation")
    settings.settings_widgets["gnss_network_tls"].setChecked(True)
    settings.settings_widgets["gnss_readiness_maximum_baseline_std_m"].setValue(0.1234)
    settings.settings_widgets["gnss_fixed_coordinate_profile_path"].setText("gnss/base.json")
    payload = settings.settings_payload()
    assert payload["interface_name"] == "enx-workstation"
    assert "require_drone_rtk_fixed" not in payload["gnss_operation"]
    assert payload["gnss_operation"]["routing"]["drone_enabled"] is False
    assert payload["gnss_operation"]["routing"]["field_base_enabled"] is False
    assert payload["gnss_operation"]["network"]["tls"] is True
    assert payload["gnss_operation"]["readiness"]["maximum_baseline_std_m"] == pytest.approx(0.1234)
    assert payload["gnss_operation"]["fixed_base"]["coordinate_profile_path"] == "gnss/base.json"

    assert window.commands.invoke("settings.apply") is True
    assert backend.calls[-1][0] == "settings_apply"
    assert window.commands.invoke("settings.save") is True
    assert backend.calls[-1] == "settings_save"
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_workstation_gnss_settings_normalize_routes_to_supported_p1_p2_base_topology(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui
    from Workstation.workstation_core.settings import AppSettings

    monkeypatch.setattr(standalone_gui, "APP_ROOT", tmp_path)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel()
    operation = AppSettings().gnss_operation.to_mapping()
    operation.update(
        {
            "operation_mode": "NETWORK_RTK_DIRECT",
            "require_drone_rtk_fixed": True,
            "routing": {
                **operation["routing"],
                "drone_enabled": True,
                "field_base_enabled": True,
            },
        }
    )

    panel.apply_workstation_system_settings({"gnss_operation": operation})

    applied = panel._workstation_setting_overrides["gnss_operation"]
    assert "require_drone_rtk_fixed" not in applied
    assert applied["routing"]["drone_enabled"] is False
    assert applied["routing"]["field_base_enabled"] is False
    built = panel._build_gnss_operation_config()
    assert built.require_drone_rtk_fixed is False
    assert built.routing.drone_enabled is False
    assert built.routing.field_base_enabled is False

    panel._save_current_settings = lambda: None
    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_workstation_every_flat_app_setting_has_an_editable_surface() -> None:
    from dataclasses import fields

    from Workstation.workstation_core.settings import AppSettings

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    system_keys = set(window.settings_workspace.settings_widgets)
    measurement_keys = set(window.measurement.settings_widgets)
    time_keys = set(window.time_workspace.settings_widgets)
    measurement_aliases = {
        "triangulation_extrinsics_source": "distance_mode",
        "yolo_input_size": "yolo_input_size_px",
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
    flat_fields = {field.name for field in fields(AppSettings)} - {"gnss_operation"}

    assert flat_fields - system_keys == set(measurement_aliases) | set(sync_aliases)
    assert set(measurement_aliases.values()) <= measurement_keys
    assert set(sync_aliases.values()) <= time_keys
    assert not set(sync_aliases.values()) & measurement_keys

    window.close()
    window.deleteLater()
    app.processEvents()


def test_workstation_main_cards_hide_source_but_provenance_survives_in_inspector() -> None:
    from Workstation.components import EngineeringValue, StatusIndicator
    from Workstation.status_model import RuntimeStatus, StatusSeverity

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    status = RuntimeStatus(
        key="camera.left",
        label="CAM L",
        state="LIVE",
        severity=StatusSeverity.OK,
        detail="frame age 0.10 s",
        source="CameraPane[left]",
        clock="app_receive_monotonic",
        age_s=0.10,
    )

    main_value = EngineeringValue("Range")
    main_value.set_value("12.3", source="MeasurementAttempt/controller result", frame="baseline_center_frd")
    assert "source=" not in main_value.meta.text()
    assert "frame=" not in main_value.meta.text()
    assert "Frame: baseline_center_frd" in main_value.toolTip()
    assert "From: MeasurementAttempt/controller result" in main_value.toolTip()

    inspector_value = EngineeringValue("Evidence")
    inspector_value.set_value("frame age 0.10 s", source=status.source)
    assert "source=" not in inspector_value.meta.text()
    assert "From: CameraPane[left]" in inspector_value.toolTip()

    main_indicator = StatusIndicator("CAM L")
    main_indicator.set_status(status)
    assert main_indicator.state.text() == "LIVE"
    assert main_indicator.age.text() == "100 ms"
    assert "source=" not in main_indicator.toolTip()

    valid_value = EngineeringValue("Pair")
    valid_value.set_value("person", valid=True, age_s=0.038)
    assert "VALID" in valid_value.meta.text()
    assert "38 ms" in valid_value.meta.text()
    assert "background" not in valid_value.meta.text()

    _ = app


def test_workstation_secondary_status_text_is_compact_but_tooltip_keeps_evidence() -> None:
    from Workstation.components import StatusIndicator
    from Workstation.status_model import RuntimeStatus, StatusSeverity

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    detail = "baseline NED=+1.000/+2.000/+3.000 m; length=3.742 m; quality=GOOD; age P1/P2=0.10/0.20s; P1→P2 @ P1 body FRD F/R/D=+2.000/-1.000/+3.000 m"
    status = RuntimeStatus(
        key="gnss",
        label="RTK / GNSS",
        state="FIX",
        severity=StatusSeverity.OK,
        detail=detail,
    )

    indicator = StatusIndicator("RTK")
    indicator.set_status(status)

    assert len(indicator.detail.text()) <= 96
    assert indicator.detail.text().endswith("…")
    assert detail in indicator.toolTip()
    _ = app


def test_status_indicator_keeps_neutral_background_across_severity_changes() -> None:
    from Workstation.components import StatusIndicator
    from Workstation.status_model import RuntimeStatus, StatusSeverity
    from Workstation.workstation_core.theme import ERROR_BG, SUCCESS_BG, WARNING_BG

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    indicator = StatusIndicator("PAIR")
    neutral_style = indicator.styleSheet()
    for state, severity in (
        ("VALID", StatusSeverity.OK),
        ("STALE", StatusSeverity.WARN),
        ("INVALID", StatusSeverity.BAD),
    ):
        indicator.set_status(
            RuntimeStatus("pair", "PAIR", state, severity, age_s=0.042)
        )
        assert indicator.styleSheet() == neutral_style
        assert indicator.state.text() == state
        assert indicator.age.text() == "42 ms"
    assert all(color not in neutral_style for color in (SUCCESS_BG, WARNING_BG, ERROR_BG))
    _ = app


def test_status_adapter_distinguishes_fresh_stale_and_error_evidence() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.left_pane = SimpleNamespace(current_status="수신 중")
    backend.panel.right_pane = SimpleNamespace(current_status="중지됨")
    backend.panel.camera_connected = True
    backend.panel.camera_frame_times["left"] = [time.monotonic()]

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()
    fresh = adapter.model.record("camera.left")
    assert fresh is not None
    assert fresh.state == "LIVE"
    assert fresh.severity == StatusSeverity.OK

    backend.panel.camera_frame_times["left"] = [time.monotonic() - 3.0]
    backend.panel.left_pane.current_status = "카메라 연결 중"
    adapter.refresh()
    stale = adapter.model.record("camera.left")
    assert stale is not None
    assert stale.state == "STALE"
    assert stale.age_s is not None and stale.age_s >= 2.9

    backend.panel.camera_connected = False
    backend.panel.camera_frame_times["left"] = []
    backend.panel.left_pane.current_status = "오류: decoder"
    adapter.refresh()
    failed = adapter.model.record("camera.left")
    assert failed is not None
    assert failed.state == "ERROR"
    assert failed.severity == StatusSeverity.BAD
    _ = app


def test_status_adapter_emits_one_snapshot_per_poll() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    adapter = BackendStatusAdapter(_FakeBackend())
    snapshots: list[object] = []
    polls: list[object] = []
    adapter.model.changed.connect(snapshots.append)
    adapter.polled.connect(polls.append)

    adapter.refresh()

    assert len(snapshots) == 1
    assert len(snapshots[0]) == 11
    assert len(polls) == 1
    assert len(polls[0]) == 11
    adapter.refresh()
    assert len(snapshots) == 1
    assert len(polls) == 2
    _ = app


def test_status_adapter_exposes_sync_and_ned_baseline_evidence() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.camera_sync_status = "verified"
    backend.panel.camera_sync_reason = "RTCP/NTP pair gate verified"
    backend.panel.selected_runtime_frame_pair_delta_s = 0.012
    backend.panel.selected_runtime_frame_pair_basis = "camera_rtc"
    backend.panel.selected_runtime_frame_pair_camera_rtc_delta_s = 0.004
    backend.panel.latest_gps = {
        "left": SimpleNamespace(
            timestamp_s=time.monotonic() - 0.1,
            rtk_label="RTK_FIXED",
            fix_type=6,
            satellites_used=18,
            eph_m=0.02,
            epv_m=0.03,
        ),
        "right": SimpleNamespace(
            timestamp_s=time.monotonic() - 0.2,
            rtk_label="RTK_FIXED",
            fix_type=6,
            satellites_used=17,
            eph_m=0.03,
            epv_m=0.04,
        ),
    }
    backend.panel.latest_rtk_baseline = SimpleNamespace(
        baseline_m=1.25,
        north_m=1.0,
        east_m=0.7,
        down_m=-0.1,
        left_age_s=0.1,
        right_age_s=0.2,
        fix_quality="GOOD",
    )
    backend.panel.latest_p1_body_frd_baseline = SimpleNamespace(
        forward_m=0.7,
        right_m=-1.0,
        down_m=-0.1,
        attitude_age_s=0.05,
    )

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()
    sync = adapter.model.record("synchronization")
    gnss = adapter.model.record("gnss")
    assert sync is not None and sync.state == "SYNCED"
    assert "RTCP/NTP" in sync.detail
    assert gnss is not None and gnss.state == "FIX"
    assert "baseline NED=+1.000/+0.700/-0.100 m" in gnss.detail
    assert "P1→P2 @ P1 body FRD F/R/D=+0.700/-1.000/-0.100 m" in gnss.detail
    assert gnss.age_s is not None and gnss.age_s >= 0.19
    _ = app


def test_status_adapter_separates_base_gnss_from_rover_baseline() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.gnss_operation_manager = SimpleNamespace(
        state=SimpleNamespace(value="BASE_SURVEYING"),
        config=SimpleNamespace(
            operation_mode=SimpleNamespace(value="LOCAL_SURVEY_IN"),
            readiness=SimpleNamespace(maximum_correction_age_s=2.0),
        ),
    )
    backend.panel.gnss_base_serial_connected = True
    backend.panel.gnss_network_connected = False
    backend.panel.latest_gnss_base_survey_in = {
        "active": True,
        "valid": False,
        "duration_s": 42,
        "mean_accuracy_m": 0.12,
        "observations": 540,
    }
    backend.panel.gnss_base_rtcm_stats = {
        "frames": 0,
        "last_frame_s": None,
    }
    backend.panel.latest_rtk_baseline = SimpleNamespace(
        baseline_m=1.2,
        north_m=1.0,
        east_m=0.5,
        down_m=0.0,
        left_age_s=0.1,
        right_age_s=0.1,
        fix_quality="GOOD",
    )

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()
    base = adapter.model.record("gnss.base")
    aggregate = adapter.model.record("gnss")
    assert base is not None and base.state == "SURVEYING"
    assert "Survey-in=active(42s, 0.12m, obs=540)" in base.detail
    assert "baseline NED" not in base.detail
    assert aggregate is not None and "baseline NED" in aggregate.detail
    _ = app


def test_alignment_workspace_keeps_raw_ned_and_p1_body_frd_values_separate() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.latest_rtk_baseline = SimpleNamespace(
        baseline_m=3.741,
        north_m=1.0,
        east_m=2.0,
        down_m=3.0,
        left_age_s=0.1,
        right_age_s=0.1,
        fix_quality="GOOD",
        warnings=(),
    )
    backend.panel.latest_p1_body_frd_baseline = SimpleNamespace(
        forward_m=1.0,
        right_m=2.0,
        down_m=3.0,
        attitude_age_s=0.05,
        target_frame="px4_frd",
        axis_contract="P1->P2 in P1 body FRD: [Forward, Right, Down]; metres",
    )
    window = WorkstationMainWindow(backend=backend)
    window.alignment.refresh({})
    assert window.alignment.baseline_values["north_m"].value.text() == "1.000 m"
    assert window.alignment.p1_body_frd_values["p1_body_frd_forward_m"].value.text() == "1.000 m"
    assert "frame=" not in window.alignment.baseline_values["north_m"].meta.text()
    assert "Frame: NED north/east/down" in window.alignment.baseline_values["north_m"].toolTip()
    assert "Frame: P1 body FRD" in window.alignment.p1_body_frd_values["p1_body_frd_forward_m"].toolTip()
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_backend_command_failure_is_not_reported_as_completion() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = QtWidgets.QWidget()
    backend = StandalonePanelBackend(panel)
    errors: list[str] = []
    backend.command_error.connect(errors.append)
    with pytest.raises(RuntimeError, match="toggle_camera_connection"):
        backend.toggle_camera()
    assert errors and "unavailable" in errors[0]
    panel.deleteLater()
    backend.deleteLater()
    app.processEvents()


def test_phase_one_backend_facades_delegate_to_standalone_owner() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls: list[str] = []
    panel = SimpleNamespace()
    method_names = (
        "discover_pc_camera_ip",
        "apply_camera_candidate",
        "check_cameras",
        "check_wiris_control",
        "enable_wiris_rtsp",
        "swap_cameras",
        "save_current_settings_from_ui",
        "set_selected_wiris_time_from_pc",
        "refresh_clock_sync_status",
        "toggle_mavlink_connection",
        "apply_pixhawk_candidate",
        "discover_gnss_base_serial_ports",
        "apply_gnss_base_candidate",
        "connect_gnss_base_serial",
        "disconnect_gnss_base_serial",
        "start_gnss_base_survey_in",
        "stop_gnss_base_survey_in",
        "start_gnss_network",
        "stop_gnss_network",
        "capture_calibration_images",
        "run_intrinsic_calibration",
        "save_intrinsics",
        "load_intrinsics",
        "load_current_zoom_intrinsics",
        "apply_intrinsics_to_distance",
        "load_checkerboard_validation_json",
        "run_checkerboard_random_validation",
        "save_checkerboard_validation_result",
        "apply_checkerboard_validation_result",
    )
    for name in method_names:
        setattr(
            panel,
            name,
            lambda *_args, name=name: calls.append(name) or (False if name == "stop_gnss_network" else None),
        )
    backend = StandalonePanelBackend(panel)
    backend.camera_apply_candidate(1, "left")
    backend.pixhawk_apply_candidate("mavlink", 2, "right")
    backend.gnss_base_apply_candidate(2)
    assert calls[-1] == "apply_gnss_base_candidate"

    facades = (
        "camera_discover_host_ip",
        "camera_check_rtsp",
        "camera_check_control",
        "camera_enable_rtsp",
        "camera_swap",
        "camera_save_settings",
        "time_set_pc",
        "time_check_sync",
        "pixhawk_toggle_mavlink",
        "gnss_base_discover",
        "gnss_base_connect",
        "gnss_base_disconnect",
        "gnss_survey_start",
        "gnss_survey_stop",
        "gnss_ntrip_start",
        "gnss_ntrip_stop",
        "calibration_capture_images",
        "calibration_run_intrinsic",
        "calibration_save_intrinsics",
        "calibration_load_intrinsics",
        "calibration_load_current_zoom_intrinsics",
        "calibration_apply_intrinsics",
        "calibration_load_validation",
        "calibration_run_validation",
        "calibration_save_validation",
        "calibration_apply_validation",
    )
    for facade in facades:
        getattr(backend, facade)()

    assert calls == [
        "apply_camera_candidate",
        "apply_pixhawk_candidate",
        "apply_gnss_base_candidate",
        *[
            name
            for name in method_names
            if name not in {"apply_camera_candidate", "apply_pixhawk_candidate", "apply_gnss_base_candidate"}
        ],
    ]
    assert backend.gnss_ntrip_stop() is None
    assert calls[-1] == "stop_gnss_network"
    backend.deleteLater()
    app.processEvents()


def test_base_gnss_disconnect_requests_stop_without_waiting_on_ui_thread() -> None:
    from Workstation import standalone_gui

    calls: list[str] = []

    class Worker:
        def request_stop(self) -> None:
            calls.append("request_stop")

    class Thread:
        def wait(self, *_args: object) -> bool:
            raise AssertionError("UI disconnect must not wait for the serial thread")

    panel = SimpleNamespace(
        gnss_base_serial_worker=Worker(),
        gnss_base_serial_thread=Thread(),
    )
    standalone_gui.WirisStereoDistancePanel.disconnect_gnss_base_serial(panel)

    assert calls == ["request_stop"]


def test_session_save_shutdown_wait_is_non_blocking_and_retryable() -> None:
    from Workstation import standalone_gui

    class Thread:
        def isRunning(self) -> bool:
            return True

        def wait(self, *_args: object) -> bool:
            raise AssertionError("GUI shutdown must not block on session save")

    panel = SimpleNamespace(
        _session_save_thread=Thread(),
        _session_save_worker=object(),
        _session_save_shutdown_wait_started_s=None,
        _session_save_shutdown_timeout_logged=False,
        _log=lambda _message: None,
    )

    assert standalone_gui.WirisStereoDistancePanel._wait_for_session_save(panel) is False
    panel._session_save_thread = None
    assert standalone_gui.WirisStereoDistancePanel._wait_for_session_save(panel) is True


def test_measurement_and_calibration_setup_facades_delegate_to_owner() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls: list[tuple[str, object]] = []
    panel = SimpleNamespace(
        apply_workstation_measurement_settings=lambda payload: calls.append(("measurement", payload)),
        save_current_settings_from_ui=lambda: calls.append(("save", None)),
        apply_workstation_calibration_setup=lambda payload: calls.append(("calibration", payload)),
        scan_checkerboard_validation_files=lambda: calls.append(("scan", None)),
        refresh_baseline_field_browser=lambda: calls.append(("alignment_scan", None)),
        start_baseline_sampling=lambda: calls.append(("alignment_start", None)),
        stop_baseline_sampling=lambda: calls.append(("alignment_stop", None)),
        save_alignment_profile=lambda: calls.append(("alignment_save", None)),
        load_alignment_profile=lambda: calls.append(("alignment_load", None)),
        swap_pixhawks=lambda: calls.append(("alignment_swap", None)),
    )
    backend = StandalonePanelBackend(panel)
    measurement = {"distance_mode": "pixhawk", "focal_px": 900.0}
    setup = {"inner_cols": 13, "inner_rows": 8, "square_size_mm": 40.0}
    backend.apply_measurement_settings(measurement)
    backend.save_measurement_settings()
    backend.calibration_apply_setup(setup)
    backend.calibration_scan_validation()
    backend.calibration_save_setup()
    backend.alignment_scan_sources()
    backend.alignment_start_sampling()
    backend.alignment_stop_sampling()
    backend.alignment_save_profile()
    backend.alignment_load_profile()
    backend.alignment_swap_pixhawks()
    assert calls == [
        ("measurement", measurement),
        ("save", None),
        ("calibration", setup),
        ("scan", None),
        ("save", None),
        ("alignment_scan", None),
        ("alignment_start", None),
        ("alignment_stop", None),
        ("alignment_save", None),
        ("alignment_load", None),
        ("alignment_swap", None),
    ]
    backend.deleteLater()
    app.processEvents()


def test_advanced_backend_facades_delegate_without_creating_resource_owners() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls: list[object] = []
    panel = SimpleNamespace()
    panel.workstation_wiris_command_catalog = lambda: ({"command": "HIWS"},)
    panel.workstation_send_wiris_command = lambda target, command, terminator: calls.append(
        ("wiris", target, command, terminator)
    )
    panel.workstation_send_zoom = lambda target, action, level: calls.append(
        ("zoom", target, action, level)
    )
    panel.workstation_discover_custom_mavlink_watch = lambda: calls.append("watch_mavlink")
    panel.workstation_add_custom_mavlink_watch = lambda name, message, field: calls.append(
        ("watch_add", name, message, field)
    )
    panel.workstation_remove_custom_mavlink_watch = lambda key: calls.append(("watch_remove", key))
    panel.workstation_refresh_baseline_field_browser = lambda: calls.append("baseline_scan") or {}
    panel.workstation_open_px4_shell = lambda camera: calls.append(("shell", camera))
    panel.workstation_refresh_px4_mag = lambda camera, read_params=True: calls.append(
        ("mag", camera, read_params)
    )
    panel.workstation_refresh_px4_attitude_cal = lambda camera, read_params=True: calls.append(
        ("attitude", camera, read_params)
    )
    panel.workstation_apply_px4_mag_priorities = lambda camera, priorities: calls.append(
        ("apply_mag", camera, priorities)
    )
    panel.workstation_save_px4_mag_params = lambda camera: calls.append(("save_mag", camera))
    panel.workstation_reboot_px4 = lambda camera: calls.append(("reboot", camera))
    panel.workstation_refresh_calibration_preview = lambda: calls.append("preview") or {"images": {}}
    panel.workstation_calibration_preview_snapshot = lambda: {"images": {}}
    backend = StandalonePanelBackend(panel)

    assert backend.advanced_wiris_catalog() == ({"command": "HIWS"},)
    backend.advanced_send_wiris_command("left", "HIWS", "\n")
    backend.advanced_send_zoom("both", "set", 10)
    backend.advanced_discover_custom_mavlink_watch()
    backend.advanced_add_custom_mavlink_watch("z", "GPS_RAW_INT", "lat")
    backend.advanced_remove_custom_mavlink_watch("z")
    backend.advanced_refresh_baseline_fields()
    backend.advanced_open_px4_shell("right")
    backend.advanced_refresh_px4_mag("right")
    backend.advanced_refresh_px4_attitude_cal("right")
    backend.advanced_apply_px4_mag_priorities("right", {"CAN1": 100})
    backend.advanced_save_px4_mag_params("right")
    backend.advanced_reboot_px4("right")
    backend.advanced_refresh_calibration_preview()
    backend.advanced_calibration_preview_snapshot()

    assert calls == [
        ("wiris", "left", "HIWS", "\n"),
        ("zoom", "both", "set", 10),
        "watch_mavlink",
        ("watch_add", "z", "GPS_RAW_INT", "lat"),
        ("watch_remove", "z"),
        "baseline_scan",
        ("shell", "right"),
        ("mag", "right", True),
        ("attitude", "right", True),
        ("apply_mag", "right", {"CAN1": 100}),
        ("save_mag", "right"),
        ("reboot", "right"),
        "preview",
    ]
    backend.deleteLater()
    app.processEvents()


def test_evidence_adapter_preserves_attitude_gps_detection_and_recording_contracts() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    now = time.monotonic()
    panel = SimpleNamespace(
        latest_extrinsics={
            "left": SimpleNamespace(
                roll_deg=1.0,
                pitch_deg=2.0,
                yaw_deg=3.0,
                q=(1.0, 0.0, 0.0, 0.0),
                timestamp_s=now - 0.25,
                px4_timestamp_s=123.5,
                px4_timestamp_us=123500000,
                px4_timestamp_sample_us=123400000,
                namespace="px4_1",
                topic="/px4_1/fmu/out/vehicle_attitude",
            )
        },
        latest_gps={
            "left": SimpleNamespace(
                rtk_label="RTK_FIXED",
                fix_type=6,
                satellites_used=18,
                eph_m=0.02,
                epv_m=0.03,
                latitude_deg=37.0,
                longitude_deg=127.0,
                altitude_m=20.0,
                timestamp_s=now - 0.1,
                px4_timestamp_s=22.0,
                namespace="px4_1",
                topic="/px4_1/fmu/out/vehicle_gps_position",
                time_source="px4_ros2",
            )
        },
        latest_rtk_baseline=SimpleNamespace(
            fix_quality="GOOD",
            north_m=1.0,
            east_m=2.0,
            down_m=-0.2,
            baseline_m=2.25,
            left_age_s=0.1,
            right_age_s=0.2,
            warnings=(),
        ),
        latest_p1_body_frd_baseline=SimpleNamespace(
            forward_m=2.0,
            right_m=1.0,
            down_m=-0.2,
            attitude_age_s=0.04,
            target_frame="px4_frd",
            axis_contract="P1->P2 in P1 body FRD: [Forward, Right, Down]; metres",
        ),
        latest_left=SimpleNamespace(
            timestamp_s=now - 0.05,
            width=1920,
            height=1080,
            x1=10.0,
            y1=20.0,
            x2=110.0,
            y2=220.0,
            center=(60.0, 120.0),
            confidence=0.9,
            class_id=0,
            class_name="person",
            candidate_class_ids=(0,),
            timestamp_metadata={"source_timestamp_s": 100.0, "timestamp_source": "rtcp_ntp"},
        ),
        latest_right=None,
        camera_stream_stats={"left": {"inferred_hz": 15.0}, "right": {}},
        yolo_enabled=True,
        recording_session_id="session-1",
        distance_recording_enabled=False,
        video_recording_enabled=False,
        distance_recording_rows=[{"row": 1}],
        camera_recording_rows={},
        px4_recording_rows={},
        gnss_base_recording_rows=[],
        video_recording_paths={},
        _session_save_thread=None,
        _session_save_retry_required=False,
        _session_save_finalized=True,
        _recording_elapsed_s=lambda _now: 1.5,
        _recording_session_dir=lambda _session: "/tmp/session-1",
        recording_status_label=QtWidgets.QLabel("saved"),
    )
    evidence = EvidenceAdapter(panel)
    attitude = evidence.attitude("left", now_s=now)
    gps = evidence.gps("left", now_s=now)
    detection = evidence.detection("left", now_s=now)
    recording = evidence.recording(now_s=now)
    assert attitude["frame_contract"].startswith("PX4")
    assert attitude["px4_timestamp_s"] == 123.5
    assert gps["position_contract"].startswith("WGS84")
    assert evidence.baseline()["axis_contract"] == "NED = north / east / down; metres"
    assert evidence.baseline()["p1_body_frd_state"] == "READY"
    assert evidence.baseline()["p1_body_frd_forward_m"] == pytest.approx(2.0)
    assert evidence.baseline()["p1_body_frd_axis_contract"].startswith("P1->P2 in P1 body FRD")
    assert detection["pixel_contract"].startswith("bbox")
    assert detection["center_u_px"] == 60.0
    assert recording["state"] == "FINALIZED"
    assert recording["sample_counts"]["pairs"] == 1
    _ = app


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


def test_workstation_evidence_preserves_mavlink_link_provenance() -> None:
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    connection = SimpleNamespace(target_system=11, target_component=42)
    subscriber = SimpleNamespace(
        urls={"left": "udpin:0.0.0.0:14550"},
        connections={"left": connection},
        _heartbeat_logged={"left"},
    )
    panel = SimpleNamespace(
        mavlink_pixhawk=subscriber,
        latest_extrinsics={
            "left": CameraExtrinsicState(
                camera="left",
                namespace="",
                roll_deg=1.0,
                pitch_deg=2.0,
                yaw_deg=3.0,
                timestamp_s=10.0,
                topic="mavlink/ATTITUDE",
                q=(1.0, 0.0, 0.0, 0.0),
            )
        },
        latest_gps={
            "left": GpsPositionState(
                camera="left",
                namespace="",
                timestamp_s=10.0,
                topic="mavlink/GPS_RAW_INT",
                latitude_deg=37.0,
                longitude_deg=127.0,
                altitude_m=10.0,
                fix_type=6,
            )
        },
    )

    evidence = EvidenceAdapter(panel)
    attitude = evidence.attitude("left", now_s=10.1)
    gps = evidence.gps("left", now_s=10.1)

    for snapshot in (attitude, gps):
        assert snapshot["telemetry_transport"] == "MAVLink"
        assert snapshot["mavlink_endpoint"] == "udpin:0.0.0.0:14550"
        assert snapshot["mavlink_link_open"] is True
        assert snapshot["mavlink_heartbeat_seen"] is True
        assert snapshot["mavlink_target_system"] == 11
        assert snapshot["mavlink_target_component"] == 42


def test_workstation_px4_timeline_persists_mavlink_link_provenance(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui, "APP_ROOT", tmp_path)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel()
    now_s = time.monotonic()
    panel.recording_session_id = "provenance-session"
    panel.recording_session_started_monotonic_ns = int((now_s - 1.0) * 1_000_000_000)
    panel.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    panel.recording_session_anchor_uncertainty_ns = 100
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = False
    panel.mavlink_pixhawk = SimpleNamespace(
        urls={"left": "udpin:0.0.0.0:14550"},
        connections={"left": SimpleNamespace(target_system=11, target_component=42)},
        _heartbeat_logged={"left"},
    )

    panel._record_external_timeline_row(
        "left",
        "pixhawk_attitude",
        {
            "topic": "mavlink/ATTITUDE",
            "sample_app_timestamp_s": now_s,
        },
    )
    row = panel.px4_recording_rows["left"][0]
    assert row["mavlink_endpoint"] == "udpin:0.0.0.0:14550"
    assert row["mavlink_link_open"] is True
    assert row["mavlink_heartbeat_seen"] is True
    assert row["mavlink_target_system"] == 11
    assert row["mavlink_target_component"] == 42

    # This test verifies one timeline row, not recording finalization. Avoid
    # launching a session-save worker from closeEvent after the assertion.
    panel.distance_recording_enabled = False
    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_workstation_uses_source_backed_alignment_calibration_and_recording_pages() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    for context, workspace in (
        ("devices.pixhawk", window.pixhawk),
        ("devices.gnss", window.base_gnss),
        ("processing.alignment", window.alignment),
        ("tasks.calibration", window.calibration),
        ("tasks.recording", window.recording),
        ("tasks.advanced", window.advanced),
    ):
        window.navigation.select_context(context)
        app.processEvents()
        assert window.workspace_host.currentWidget().widget() is workspace

    assert window.alignment.attitude_model.rowCount() == 2
    assert window.alignment.gnss_model.rowCount() == 2
    assert window.calibration.intrinsic_model.rowCount() == 2
    assert window.recording.artifact_model.rowCount() >= 1
    assert window.recording.protocol_state_label.text().startswith("BLOCKED")
    assert any("Both camera streams" in label.text() for label in window.recording.protocol_check_labels.values())
    assert any("Numeric GNSS fix_type" in label.text() for label in window.recording.protocol_check_labels.values())
    assert any("Physical alignment evidence" in label.text() for label in window.recording.protocol_check_labels.values())
    assert any("Final WIRIS time-sync checkpoint" in label.text() for label in window.recording.protocol_check_labels.values())
    assert "NED" in window.alignment.findChildren(QtWidgets.QLabel)[-1].text()
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_session_catalog_scans_are_throttled_during_status_refresh(monkeypatch) -> None:
    from Workstation.status_model import StatusModel
    from Workstation.workspaces import AdvancedWorkspace, RecordingWorkspace

    calls: list[float] = []

    def catalog(_adapter: object) -> list[object]:
        calls.append(time.monotonic())
        return []

    monkeypatch.setattr(EvidenceAdapter, "session_catalog", catalog)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    status_model = StatusModel()

    recording = RecordingWorkspace(backend, status_model, lambda: None)
    assert len(calls) == 1
    for _ in range(10):
        recording.refresh({})
    assert len(calls) == 1
    recording.refresh_sessions(force=True)
    assert len(calls) == 2

    advanced = AdvancedWorkspace(backend, status_model)
    assert len(calls) == 3
    for _ in range(10):
        advanced.refresh({})
    assert len(calls) == 3
    advanced.refresh_media(force=True)
    assert len(calls) == 4

    recording.deleteLater()
    advanced.deleteLater()
    app.processEvents()


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


def test_status_refresh_updates_only_visible_workspace_and_diagnostics() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.show()
    app.processEvents()
    window.status_adapter.stop()
    calls: list[str] = []
    window.overview.refresh = lambda _snapshot: calls.append("overview")
    window.camera.refresh = lambda _snapshot: calls.append("camera")
    window.measurement.refresh = lambda _snapshot: calls.append("measurement")
    window.diagnostics.refresh = lambda _snapshot: calls.append("diagnostics")
    window.diagnostics_dock.hide()

    window._select_context("overview")
    calls.clear()
    window._refresh_evidence_from_poll({})
    assert calls == ["overview"]

    window._select_context("devices.cameras")
    calls.clear()
    window._refresh_evidence_from_poll({})
    assert calls == ["camera"]

    window.diagnostics_dock.show()
    app.processEvents()
    calls.clear()
    window._refresh_evidence_from_poll({})
    assert calls == ["camera", "diagnostics"]
    assert "measurement" not in calls

    window.close()
    window.deleteLater()
    app.processEvents()


def test_diagnostics_refreshes_only_the_visible_tab_and_skips_unchanged_text() -> None:
    from Workstation.status_model import StatusModel
    from Workstation.workspaces import DiagnosticsContent

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    diagnostics = DiagnosticsContent(_FakeBackend(), StatusModel())
    calls: list[int] = []
    refresh_names = (
        "_refresh_system",
        "_refresh_camera",
        "_refresh_pixhawk",
        "_refresh_gnss",
        "_refresh_log",
        "_refresh_measurement",
        "_refresh_performance",
    )
    for index, name in enumerate(refresh_names):
        setattr(diagnostics, name, lambda _snapshot, index=index: calls.append(index))

    for index in range(diagnostics.tabs.count()):
        diagnostics.tabs.setCurrentIndex(index)
        calls.clear()
        diagnostics.refresh({})
        assert calls == [index]

    changes: list[bool] = []
    diagnostics.camera_view.textChanged.connect(lambda: changes.append(True))
    diagnostics._set_diagnostic_text(diagnostics.camera_view, "same evidence")
    diagnostics._set_diagnostic_text(diagnostics.camera_view, "same evidence")
    assert len(changes) == 1


def test_diagnostics_log_preserves_manual_scroll_and_offers_follow_latest() -> None:
    from Workstation.status_model import StatusModel
    from Workstation.workspaces import DiagnosticsContent

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    diagnostics = DiagnosticsContent(_FakeBackend(), StatusModel())
    initial = "\n".join(f"line {index}" for index in range(80))
    diagnostics._set_log_text(initial)
    scrollbar = diagnostics.log_view.verticalScrollBar()
    scrollbar.setValue(12)
    saved_position = scrollbar.value()

    diagnostics._set_log_text(initial + "\nnew line 1\nnew line 2")
    assert scrollbar.value() == saved_position
    assert diagnostics.log_new_label.text() == "New logs: 2"
    assert not diagnostics.log_new_label.isHidden()
    assert not diagnostics.log_follow_button.isHidden()

    diagnostics.log_follow_button.click()
    assert scrollbar.value() == scrollbar.maximum()
    assert diagnostics.log_new_label.isHidden()
    assert diagnostics.log_follow_button.isHidden()

    diagnostics.deleteLater()
    app.processEvents()


def test_polled_table_updates_preserve_selected_row() -> None:
    from PyQt5 import QtGui

    from Workstation.workspaces import _sync_table_rows

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    model = QtGui.QStandardItemModel(0, 2)
    table = QtWidgets.QTableView()
    table.setModel(model)
    table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
    _sync_table_rows(model, (("P1", "WAITING"), ("P2", "WAITING")))
    table.selectRow(0)
    app.processEvents()

    _sync_table_rows(model, (("P1", "LIVE"), ("P2", "WAITING")))

    assert model.item(0, 1).text() == "LIVE"
    assert [index.row() for index in table.selectionModel().selectedRows()] == [0]
    table.deleteLater()
    app.processEvents()


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
    assert thread.wait_calls == [1500, 1500]
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
    monkeypatch.setattr(camera_pane.time, "monotonic", lambda: 11.6)
    assert pane.stop(wait=False) is False
    assert calls[-2:] == ["request_stop", "terminate"]
    monkeypatch.setattr(camera_pane.time, "monotonic", lambda: 13.1)
    assert pane.stop(wait=False) is False
    assert calls[-2:] == ["request_stop", "kill"]

    pane._clear_thread_if_current(1, thread)
    assert pane.worker is None
    assert pane.thread is None
    pane.deleteLater()
    app.processEvents()


def test_load_intrinsics_opens_file_picker_and_applies_selected_profile(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui
    from Workstation.workstation_core.calibration import (
        CameraIntrinsics,
        CheckerboardConfig,
        StereoIntrinsicsResult,
        save_intrinsics_json,
    )

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.zoom_level_spin.setValue(4)
    camera = CameraIntrinsics(
        width=1920,
        height=1080,
        camera_matrix=[[1000.0, 0.0, 960.0], [0.0, 1001.0, 540.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=12,
    )
    result = StereoIntrinsicsResult(
        left=camera,
        right=camera,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.025),
    )
    selected_path = tmp_path / "selected_intrinsics.json"
    save_intrinsics_json(selected_path, result, metadata={"zoom_level": 4})
    monkeypatch.setattr(
        standalone_gui.QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args, **_kwargs: (str(selected_path), "Intrinsics JSON (*.json)"),
    )

    panel.load_intrinsics()

    assert panel.active_intrinsics == result
    assert panel._active_intrinsics_profile_path == str(selected_path)
    assert panel.manual_focal_checkbox.isChecked() is False
    assert panel.calibration_status_label.text() == "Loaded and applied intrinsics: selected_intrinsics.json"
    panel.deleteLater()
    app.processEvents()


def test_pixhawk_eth_mav_recovery_is_visible_and_routes_to_usb_owner() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.pixhawk.refresh(window.status_adapter.model.snapshot())
    pixhawk_layout = window.pixhawk.layout()
    assert [
        pixhawk_layout.itemAt(index).widget().title()
        for index in range(2, 8)
        if pixhawk_layout.itemAt(index).widget() is not None
    ] == [
        "Connection",
        "Attitude (R/P/Y)",
        "P1/P2 GNSS receivers",
        "P1→P2 baseline",
        "Discovered endpoints",
        "ETH MAV",
    ]
    top_actions = pixhawk_layout.itemAt(1).layout()
    assert [top_actions.itemAt(index).widget().text() for index in range(5)] == [
        "Connect",
        "Find USB",
        "Find network",
        "Find host IP",
        "Swap P1/P2",
    ]
    assert window.pixhawk.operation_buttons["pixhawk.recover_eth_mavlink.p1"].text() == "P1 ETH MAV"
    assert window.pixhawk.operation_buttons["pixhawk.recover_eth_mavlink.p2"].text() == "P2 ETH MAV"
    assert window.pixhawk.recovery_values["left"]["usb"].value.text() == "/dev/ttyACM0"
    assert window.pixhawk.recovery_values["right"]["baud"].value.text() == "115200"

    backend.panel.pixhawk_usb_bootstrap_status["left"] = {
        "state": "RECOVERED",
        "detail": "PX4 Ethernet MAVLink 시작 완료; UDP telemetry 확인 중",
        "updated_s": 10.0,
    }
    window.pixhawk.refresh(window.status_adapter.model.snapshot())
    assert window.pixhawk.recovery_values["left"]["recovery"].value.text() == "RECOVERED"
    assert "HB NO LINK" in window.pixhawk.recovery_values["left"]["telemetry"].value.text()

    assert window.commands.invoke("pixhawk.recover_eth_mavlink.p1") is True
    assert backend.calls[-1] == ("pixhawk_recover_eth_mavlink", "left")

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_pixhawk_recovery_evidence_separates_usb_control_and_attitude() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    left_baud = QtWidgets.QSpinBox()
    left_baud.setRange(1, 10_000_000)
    left_baud.setValue(115_200)
    panel = SimpleNamespace(
        left_pixhawk_bootstrap_url_edit=QtWidgets.QLineEdit("/dev/ttyACM0"),
        left_pixhawk_bootstrap_baud_spin=left_baud,
        pixhawk_usb_bootstrap_thread=None,
        pixhawk_usb_bootstrap_status={
            "left": {"state": "RECOVERED", "detail": "stream started", "updated_s": 99.0}
        },
        mavlink_pixhawk=SimpleNamespace(
            urls={"left": "udpin:0.0.0.0:14550"},
            connections={"left": object()},
            _heartbeat_logged={"left"},
        ),
        latest_extrinsics={
            "left": SimpleNamespace(
                topic="mavlink/ATTITUDE",
                timestamp_s=99.5,
                roll_deg=1.0,
                pitch_deg=2.0,
                yaw_deg=3.0,
            )
        },
    )

    recovery = EvidenceAdapter(panel).pixhawk_recovery("left", now_s=100.0)

    assert recovery["usb_url"] == "/dev/ttyACM0"
    assert recovery["usb_baud"] == 115_200
    assert recovery["state"] == "RECOVERED"
    assert recovery["heartbeat_state"] == "OK"
    assert recovery["attitude_state"] == "OK"
    assert recovery["mavlink_endpoint"] == "udpin:0.0.0.0:14550"
    assert "PX4 Ethernet MAVLink" in str(recovery["contract"])

    left_baud.deleteLater()
    panel.left_pixhawk_bootstrap_url_edit.deleteLater()
    app.processEvents()


def test_workstation_exposes_phase_one_camera_pixhawk_and_gnss_commands() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    camera_commands = (
        "camera.discover_host_ip",
        "camera.check_rtsp",
        "camera.check_control",
        "camera.enable_rtsp",
        "camera.swap",
        "camera.save_settings",
    )
    alignment_commands = (
        "pixhawk.toggle_mavlink",
        "gnss.base_discover",
        "gnss.base_connect",
        "gnss.base_disconnect",
        "gnss.survey_start",
        "gnss.survey_stop",
        "gnss.ntrip_start",
        "gnss.ntrip_stop",
    )
    calibration_commands = (
        "calibration.capture_images",
        "calibration.run_intrinsic",
        "calibration.save_intrinsics",
        "calibration.load_intrinsics",
        "calibration.load_current_zoom_intrinsics",
        "calibration.apply_intrinsics",
        "calibration.load_validation",
        "calibration.run_validation",
        "calibration.save_validation",
        "calibration.apply_validation",
    )

    assert all(window.camera.operation_buttons[command].isEnabled() for command in camera_commands)
    assert all(window.alignment.operation_buttons[command].isEnabled() for command in alignment_commands)
    assert all(window.calibration.operation_buttons[command].isEnabled() for command in calibration_commands)
    for command in (*camera_commands, *alignment_commands, *calibration_commands):
        assert window.commands.invoke(command) is True

    assert backend.calls[-len(camera_commands) - len(alignment_commands) - len(calibration_commands):] == [
        "camera_discover_host_ip",
        "camera_check_rtsp",
        "camera_check_control",
        "camera_enable_rtsp",
        "camera_swap",
        "camera_save_settings",
        "pixhawk_toggle_mavlink",
        "gnss_base_discover",
        "gnss_base_connect",
        "gnss_base_disconnect",
        "gnss_survey_start",
        "gnss_survey_stop",
        "gnss_ntrip_start",
        "gnss_ntrip_stop",
        "calibration_capture_images",
        "calibration_run_intrinsic",
        "calibration_save_intrinsics",
        "calibration_load_intrinsics",
        "calibration_load_current_zoom_intrinsics",
        "calibration_apply_intrinsics",
        "calibration_load_validation",
        "calibration_run_validation",
        "calibration_save_validation",
        "calibration_apply_validation",
    ]
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_workstation_exposes_measurement_and_calibration_setup_controls() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    assert window.measurement.operation_buttons["measurement.apply_settings"].isEnabled()
    assert window.measurement.operation_buttons["measurement.save_settings"].isEnabled()
    assert window.time_workspace.operation_buttons["time.apply_settings"].isEnabled()
    assert window.time_workspace.operation_buttons["time.save_settings"].isEnabled()
    assert window.calibration.operation_buttons["calibration.apply_setup"].isEnabled()
    assert window.calibration.operation_buttons["calibration.scan_validation"].isEnabled()
    assert window.calibration.operation_buttons["calibration.save_setup"].isEnabled()
    assert window.alignment.operation_buttons["alignment.scan_sources"].isEnabled()
    assert window.alignment.operation_buttons["alignment.start_sampling"].isEnabled()
    assert window.alignment.operation_buttons["alignment.stop_sampling"].isEnabled()
    assert window.alignment.operation_buttons["alignment.save_profile"].isEnabled()
    assert window.alignment.operation_buttons["alignment.load_profile"].isEnabled()
    assert window.alignment.operation_buttons["alignment.swap_pixhawks"].isEnabled()
    measurement_payload = window.measurement.settings_payload()
    time_payload = window.time_workspace.settings_payload()
    assert measurement_payload["distance_mode"] in {"pixhawk", "checkerboard"}
    assert measurement_payload["focal_px"] > 0
    assert "camera_sync_rtcp_pair_max_delta_ms" not in measurement_payload
    assert "camera_sync_rtcp_pair_max_delta_ms" in time_payload
    calibration_payload = window.calibration.setup_payload()
    assert calibration_payload["inner_cols"] >= 2
    assert calibration_payload["square_size_mm"] > 0
    assert window.commands.invoke("measurement.apply_settings") is True
    assert window.commands.invoke("measurement.save_settings") is True
    assert window.commands.invoke("time.apply_settings") is True
    assert window.commands.invoke("time.save_settings") is True
    assert window.commands.invoke("calibration.apply_setup") is True
    assert window.commands.invoke("calibration.scan_validation") is True
    assert window.commands.invoke("calibration.save_setup") is True
    assert window.commands.invoke("alignment.scan_sources") is True
    assert window.commands.invoke("alignment.start_sampling") is True
    assert window.commands.invoke("alignment.stop_sampling") is True
    assert window.commands.invoke("alignment.save_profile") is True
    assert window.commands.invoke("alignment.load_profile") is True
    assert window.commands.invoke("alignment.swap_pixhawks") is True
    extended_calls = backend.calls[-13:]
    assert extended_calls[0][0] == "measurement_apply_settings"
    assert extended_calls[1] == "measurement_save_settings"
    assert extended_calls[2][0] == "measurement_apply_settings"
    assert extended_calls[3] == "measurement_save_settings"
    assert extended_calls[4][0] == "calibration_apply_setup"
    assert extended_calls[5] == "calibration_scan_validation"
    assert extended_calls[6] == "calibration_save_setup"
    assert extended_calls[7:] == [
        "alignment_scan_sources",
        "alignment_start_sampling",
        "alignment_stop_sampling",
        "alignment_save_profile",
        "alignment_load_profile",
        "alignment_swap_pixhawks",
    ]
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_recording_workspace_reviews_saved_csv_without_frame_conversion(tmp_path, monkeypatch) -> None:
    from Workstation.workstation_core import settings as settings_module

    monkeypatch.setattr(settings_module, "APP_ROOT", tmp_path)
    session_dir = tmp_path / "recordings" / "sessions" / "session-1"
    generation_dir = session_dir / ".generations" / "generation-1"
    generation_dir.mkdir(parents=True)
    (generation_dir / "session-1_distance_measurements.csv").write_text(
        "recording_time_ns,forward_m,right_m,down_m,range_m\n1,2,3,4,5\n",
        encoding="utf-8",
    )
    (session_dir / "published").symlink_to(Path(".generations") / "generation-1")
    (session_dir / "session_manifest.json").write_text(
        '{"status":"finalized","generation_id":"generation-1",'
        '"time_axis":"recording_time_ns",'
        '"artifact_roles":{"session-1_distance_measurements.csv":"distance_measurements_csv"}}',
        encoding="utf-8",
    )

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.show()
    app.processEvents()
    assert window.recording.session_selector.count() == 1
    assert window.recording.session_selector.currentData() == "session-1"
    assert window.recording.artifact_selector.currentData() == "session-1_distance_measurements.csv"
    assert window.recording.review_model.rowCount() == 1
    assert window.recording.review_model.item(0, 1).text() == "2"
    assert "no NED/ENU" in window.recording.review_status_label.toolTip()
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_workstation_exposes_advanced_owner_tools_and_preserves_contracts() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.navigation.select_context("tasks.advanced")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.advanced
    assert window.advanced.wiris_catalog_combo.count() == 1
    assert window.advanced.baseline_mavlink_model.rowCount() == 0
    assert not any("Safety contracts" in label.text() for label in window.advanced.findChildren(QtWidgets.QLabel))
    assert "OpenCV RDF" in window.advanced.preview_status_label.text()

    window.advanced.wiris_command_edit.setText("HIWS")
    assert window.commands.invoke("advanced.wiris.send") is True
    assert backend.calls[-1] == ("advanced_wiris", "both", "HIWS", "\n")

    window.advanced.zoom_level_spin.setValue(12)
    assert window.commands.invoke("advanced.zoom.set") is True
    assert backend.calls[-1] == ("advanced_zoom", "both", "set", 12)

    window.advanced.watch_name_edit.setText("local_z")
    window.advanced.watch_message_edit.setText("LOCAL_POSITION_NED")
    window.advanced.watch_field_edit.setText("z")
    assert window.commands.invoke("advanced.watch.add_mavlink") is True
    assert backend.calls[-1] == (
        "advanced_watch_add_mavlink",
        "local_z",
        "LOCAL_POSITION_NED",
        "z",
    )

    assert window.commands.invoke("advanced.baseline.scan") is True
    assert window.advanced.baseline_mavlink_model.rowCount() == 1
    assert "advanced.baseline.describe" not in window.commands._specs

    window.advanced.px4_camera_combo.setCurrentIndex(1)
    assert window.commands.invoke("advanced.px4.refresh_mag") is True
    assert backend.calls[-1] == ("advanced_px4_mag", "right", True)
    assert window.commands.invoke("advanced.px4.refresh_attitude") is True
    assert backend.calls[-1] == ("advanced_px4_attitude", "right", True)
    assert window.commands.invoke("advanced.px4.shell") is True
    assert backend.calls[-1] == ("advanced_px4_shell", "right")

    assert window.commands.invoke("advanced.preview.refresh") is True
    assert "checkerboard pixels" in window.advanced.preview_status_label.text()

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
