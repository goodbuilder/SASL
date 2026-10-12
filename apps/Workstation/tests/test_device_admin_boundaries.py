"""Administration ownership, captured inputs and GUI/evidence handoffs.

All workers and subscribers below are in-memory fakes. These checks must never
open USB/network transports or invoke the firmware uploader.
"""

from pathlib import Path
from dataclasses import FrozenInstanceError
from types import SimpleNamespace
import ast
import threading
import time

import pytest
from PyQt5 import QtCore, QtWidgets

import Workstation.device_runtime_controller as controller_module
from Workstation.device_runtime.admin_commands import (
    BootstrapCommand, FlashCommand, PersistentCommand, RecoveryCommand,
    StreamSetupCommand, TransferCommand,
    PersistentEthernetInputs, PersistentEthernetConfiguration,
    build_persistent_ethernet_configuration,
)
from Workstation.device_runtime.admin_jobs import ADMIN_FIELDS, PixhawkAdminJobs
from Workstation.device_runtime.discovery_jobs import (
    DISCOVERY_FIELDS, NetworkDiscoveryJobs, CameraHostDiscovery,
    PixhawkHostDiscovery, PixhawkNetworkDiscovery, CameraAddressApply,
)
from Workstation.device_runtime.parameters import interpret_parameter_result, apply_mag_manager_readback
from Workstation.device_runtime.presentation import attitude_cal_status_text, parameter_messages
from Workstation.tests.test_panel_composition import panels


class HeldWorker(QtCore.QObject):
    progress = QtCore.pyqtSignal(object)
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, subscriber=None, **inputs):
        super().__init__()
        self.subscriber, self.inputs = subscriber, inputs
        self.entered, self.release = threading.Event(), threading.Event()
        self.cancelled = False

    def cancel(self):
        self.cancelled = True

    @QtCore.pyqtSlot()
    def run(self):
        self.entered.set()
        self.release.wait(5.0)
        self.progress.emit({"camera": self.inputs.get("camera", "left"), "sent": 7, "total": 7,
                            "direction": self.inputs.get("direction", "upload")})
        self.completed.emit({"camera": self.inputs.get("camera", "left"), "remote_size": 7,
                             "remote_crc32": 0x12345678, "local_sha256": "captured-hash",
                             "remote_path": self.inputs.get("remote_path", "/file"),
                             "evidence_path": self.inputs.get("evidence_path", "")})
        self.finished.emit()


class FlashWorker(HeldWorker):
    progress = QtCore.pyqtSignal(str)

    @QtCore.pyqtSlot()
    def run(self):
        self.entered.set()
        self.release.wait(5.0)
        self.progress.emit("board verified")
        self.completed.emit({"camera": self.inputs["camera"], "state": "FLASHED/VERIFIED",
                             "detected_board_id": 53, "firmware": {"sha256": "verified-hash"}})
        self.finished.emit()


class DiscoveryWorker(HeldWorker):
    discovered = QtCore.pyqtSignal(object)
    applied = QtCore.pyqtSignal(object)

    @QtCore.pyqtSlot()
    def run(self):
        self.entered.set()
        self.release.wait(5.0)
        result = {"candidates": [], "inputs": dict(self.inputs), "ok": True}
        self.discovered.emit(result)
        self.applied.emit(result)
        self.finished.emit()


def until(predicate):
    deadline = time.monotonic() + 4.0
    while not predicate() and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert predicate()


def cleanup(owner, *, dispose=True):
    for job in list(owner._jobs.values()):
        job.worker.release.set()
    owner.stop(wait=False)
    until(lambda: not owner._jobs)
    if dispose:
        owner.deleteLater()


def command(kind="mavftp", camera="left"):
    if kind == "mavftp":
        return TransferCommand(object(), camera, "/local", "/remote", False, "/evidence", "upload", "file_upload", 7)
    if kind == "usb_flash":
        return FlashCommand(camera, 1, "/dev/test-usb", "/firmware", "/uploader")
    if kind == "usb_bootstrap":
        return BootstrapCommand(camera, "/dev/test-usb", (57600,), "primary", "fallback")
    if kind == "usb_recovery":
        return RecoveryCommand(camera, {"device": "/dev/test-usb", "system_id": 1}, 1)
    if kind == "persistent_config":
        return PersistentCommand(object(), camera, "command", "extras", "net.cfg")
    if kind == "stream_setup":
        return StreamSetupCommand(object(), "10.0.0.1", {camera: 14550}, {camera: 1}, 2_000_000, (camera,))
    raise AssertionError(kind)


def prepare(owner, cmd, events, **overrides):
    callbacks = dict(
        worker_factory=FlashWorker if cmd.kind == "usb_flash" else HeldWorker,
        completed=lambda result: events.append(("completed", result, QtCore.QThread.currentThread())),
        failed=lambda error: events.append(("failed", error, QtCore.QThread.currentThread())),
        progress=lambda value: events.append(("progress", value, QtCore.QThread.currentThread())),
    )
    callbacks.update(overrides)
    return owner.prepare(cmd, **callbacks)


@pytest.mark.parametrize("kind", ["mavftp", "usb_flash", "usb_bootstrap", "usb_recovery", "persistent_config", "stream_setup"])
def test_admin_lanes_retain_exclusion_gui_affinity_and_old_event_fencing(kind):
    owner, events = PixhawkAdminJobs(), []
    job = prepare(owner, command(kind), events)
    try:
        assert owner.start(job)
        assert job.worker.entered.wait(1.0)
        with pytest.raises(RuntimeError, match="already running"):
            prepare(owner, command(kind), events)
        owner.retire(job)
        assert owner.busy(kind, "left")
        job.worker.release.set()
        until(lambda: not owner.busy(kind, "left"))
        assert [event[0] for event in events] == ["progress", "completed"]
        assert all(event[2] is owner.thread() for event in events)
        replacement = prepare(owner, command(kind), events)
        owner.deliver(job, "completed", {"camera": "left", "stale": True})
        owner.retire(job)
        assert owner._jobs[replacement.lane] is replacement
        assert len(events) == 2
        assert owner.start(replacement)
    finally:
        cleanup(owner)


def test_distinct_transfer_roles_and_admin_kinds_keep_existing_concurrency():
    owner, events = PixhawkAdminJobs(), []
    jobs = [prepare(owner, command("mavftp", side), events) for side in ("left", "right", "drone")]
    jobs.append(prepare(owner, command("persistent_config"), events))
    try:
        for job in jobs:
            assert owner.start(job)
            assert job.worker.entered.wait(1.0)
        assert len(owner._jobs) == 4
        assert jobs[0].worker.subscriber is jobs[0].command.subscriber
    finally:
        cleanup(owner)


def test_captured_commands_are_detached_from_mutable_request_configuration():
    ports, ids, sides = {"left": 14550}, {"left": 1}, ["left"]
    subscriber = object()
    setup = StreamSetupCommand(subscriber, "10.0.0.1", ports, ids, 2_000_000, sides)
    evidence = {"device": "/dev/verified", "baud": 115200, "system_id": 1}
    recovery = RecoveryCommand("left", evidence, 1)
    ports["left"], ids["left"], sides[0], evidence["system_id"] = 9, 9, "right", 2
    args = setup.build(lambda transport, **kwargs: (transport, kwargs))
    assert args[0] is subscriber
    assert args[1] == {"host_ip": "10.0.0.1", "ports": {"left": 14550}, "sysids": {"left": 1},
                       "rate": 2_000_000, "cameras": ("left",)}
    recovered = recovery.build(lambda **kwargs: kwargs)
    assert recovered["evidence"]["system_id"] == 1
    recovered["evidence"]["device"] = "changed"
    assert recovery.evidence["device"] == "/dev/verified"


@pytest.mark.parametrize("cmd", [
    CameraHostDiscovery("10.0.0.10/24", ("10.0.0.231", "10.0.0.232")),
    PixhawkHostDiscovery(),
    PixhawkNetworkDiscovery("10.0.0.2", "10.0.0.3", "udp:1", "udp:2"),
    CameraAddressApply("test-nic", "10.0.0.10/24"),
])
def test_discovery_owns_each_real_signal_slot_and_drains_before_replacement(cmd):
    owner, events = NetworkDiscoveryJobs(), []
    job = prepare(owner, cmd, events, worker_factory=DiscoveryWorker, progress=None)
    try:
        assert owner.start(job)
        assert job.worker.entered.wait(1.0)
        with pytest.raises(RuntimeError, match="already running"):
            prepare(owner, cmd, events, worker_factory=DiscoveryWorker, progress=None)
        prefix = owner.prefixes[cmd.kind]
        setattr(owner, prefix + "_thread", None)
        setattr(owner, prefix + "_worker", None)
        assert job.thread in owner.resource_inventory().values()
        assert owner.stop(wait=False) is False
        job.worker.release.set()
        until(lambda: not owner._jobs)
        assert [event[0] for event in events] == ["completed"]
        assert events[0][2] is owner.thread()
        replacement = prepare(owner, cmd, events, worker_factory=DiscoveryWorker, progress=None)
        owner.deliver(job, "failed", "old discovery failure")
        assert owner._jobs[replacement.lane] is replacement
        assert len(events) == 1
        owner.start(replacement)
    finally:
        cleanup(owner)


def test_camera_discovery_uses_one_state_owner_and_recovers_start_failure(panels, monkeypatch):
    panel = panels()
    owner = panel.application.features["diagnostics"].state.discovery_jobs
    monkeypatch.setattr(controller_module, "CameraNetworkDiscoveryWorker", DiscoveryWorker)
    monkeypatch.setattr(QtCore.QThread, "start", lambda _self: (_ for _ in ()).throw(RuntimeError("start failed")))
    try:
        panel.discover_pc_camera_ip()
        assert not owner._jobs
        assert panel.detect_pc_ip_button.isEnabled()
        assert panel.workstation_camera_discovery_status == "ERROR · start failed"
        state = panel.application.features["diagnostics"].state
        for name in DISCOVERY_FIELDS:
            assert name not in vars(state)
            assert getattr(panel, name) is getattr(owner, name)
    finally:
        cleanup(owner, dispose=False)


def test_hidden_aliases_do_not_lose_admin_resources_and_shutdown_drains_results():
    owner, events = PixhawkAdminJobs(), []
    job = prepare(owner, command(), events)
    try:
        owner.start(job)
        assert job.worker.entered.wait(1.0)
        owner.pixhawk_mavftp_threads.clear()
        owner.pixhawk_mavftp_workers.clear()
        assert job.thread in owner.resource_inventory().values()
        assert job.worker in owner.resource_inventory().values()
        assert owner.request_shutdown() is False
        assert job.worker.cancelled
        with pytest.raises(RuntimeError, match="shutting down"):
            prepare(owner, command("usb_bootstrap"), events)
        job.worker.release.set()
        until(lambda: not owner._jobs)
        assert [event[0] for event in events] == ["progress", "completed"]
        assert owner.request_shutdown() is True
    finally:
        cleanup(owner)


def test_nonstarted_admin_failure_releases_lane_but_partial_start_stays_admitted(monkeypatch):
    owner, events = PixhawkAdminJobs(), []
    job = prepare(owner, command(), events)
    original_start = QtCore.QThread.start
    try:
        monkeypatch.setattr(job.thread, "start", lambda: (_ for _ in ()).throw(RuntimeError("not started")))
        assert owner.start(job) is False
        assert [event[0] for event in events] == ["failed"]
        assert not owner.busy("mavftp", "left")
        newer = prepare(owner, command(), events)
        def partial_start():
            original_start(newer.thread)
            assert newer.worker.entered.wait(1.0)
            raise RuntimeError("after actual launch")
        monkeypatch.setattr(newer.thread, "start", partial_start)
        assert owner.start(newer) is True
        assert owner.busy("mavftp", "left")
        newer.worker.release.set()
        until(lambda: not owner._jobs)
        assert [event[0] for event in events] == ["failed", "progress", "completed"]
    finally:
        cleanup(owner)


def test_reentrant_retirement_waits_for_terminal_effect_and_connection_guard():
    owner, events = PixhawkAdminJobs(), []
    job = prepare(owner, command(), events)
    try:
        def completed(_value):
            owner.retire(job)
            events.append(owner.busy("mavftp", "left"))
            with pytest.raises(RuntimeError, match="already running"):
                prepare(owner, command(), events)
        job.completed = completed
        owner.deliver(job, "completed", {})
        assert events == [True]
        assert not owner._jobs
        job.worker.deleteLater()
        guarded = prepare(owner, command("persistent_config"), events, accepts_result=lambda: False)
        owner.deliver(guarded, "completed", {"camera": "left"})
        assert events == [True]
        owner.retire(guarded)
        guarded.worker.deleteLater()
    finally:
        cleanup(owner)


def test_transfer_snapshot_preserves_direction_and_detaches_nested_evidence():
    owner = PixhawkAdminJobs()
    try:
        owner.transfer_started(command())
        owner.transfer_progress({"camera": "left", "direction": "download", "sent": 3})
        assert owner.transfer_snapshot()["left"]["state"] == "DOWNLOADING"
        owner.transfer_completed({"camera": "left", "remote_size": 7, "checks": {"crc": "1234"}}, category="firmware_stage")
        snapshot = owner.transfer_snapshot()
        snapshot["left"]["checks"]["crc"] = "changed"
        assert owner.transfer_snapshot()["left"] == {"camera": "left", "remote_size": 7,
                                                    "checks": {"crc": "1234"}, "state": "STAGED",
                                                    "category": "firmware_stage"}
    finally:
        owner.deleteLater()


def test_real_panel_transfer_owner_preserves_evidence_path_and_stop_result(panels, monkeypatch, tmp_path):
    panel = panels()
    old_subscriber = panel.mavlink_pixhawk
    subscriber = SimpleNamespace(connections={"left": object()})
    panel.mavlink_pixhawk = subscriber
    monkeypatch.setattr(controller_module, "PixhawkMavFtpTransferWorker", HeldWorker)
    monkeypatch.setattr(controller_module, "WORKSTATION_DATA_ROOT", tmp_path)
    source = tmp_path / "upload.bin"
    source.write_bytes(b"1234567")
    controller = panel._device_runtime_controller()
    owner = panel.application.features["diagnostics"].state.admin_jobs
    try:
        assert controller.start_pixhawk_mavftp_upload("left", str(source), "/fs/microsd/upload.bin")
        job = owner._jobs[("mavftp", "left")]
        assert job.worker.entered.wait(1.0)
        assert job.command.subscriber is subscriber
        assert Path(job.command.evidence_path).parent == tmp_path / "pixhawk_transfers" / "left"
        state = panel.application.features["diagnostics"].state
        for name in ADMIN_FIELDS:
            assert name not in vars(state)
            assert getattr(panel, name) is getattr(owner, name)
        assert panel.application.resource_inventory()["pixhawk_mavftp_threads"]["left"] is job.thread
        assert owner.stop(wait=False) is False
        controller._apply_pixhawk_mavftp_result({"camera": "left", "remote_size": 999}, category="file_upload")
        assert panel.pixhawk_transfer_status["left"]["state"] == "UPLOADING"
        job.worker.release.set()
        until(lambda: not owner._jobs)
        assert controller.pixhawk_transfer_snapshot()["left"]["remote_size"] == 7
        assert controller.pixhawk_transfer_snapshot()["left"]["evidence_path"] == job.command.evidence_path
    finally:
        cleanup(owner, dispose=False)
        panel.mavlink_pixhawk = old_subscriber


def test_flash_evidence_persistence_failure_is_not_verified_success(panels, monkeypatch, tmp_path):
    panel = panels()
    monkeypatch.setattr(controller_module, "PixhawkUsbFlashWorker", FlashWorker)
    monkeypatch.setattr(controller_module, "WORKSTATION_DATA_ROOT", tmp_path)
    monkeypatch.setattr(controller_module, "atomic_write_firmware_evidence",
                        lambda *_args: (_ for _ in ()).throw(OSError("evidence disk full")))
    firmware = tmp_path / "firmware.px4"
    firmware.write_bytes(b"fixture")
    owner = panel.application.features["diagnostics"].state.admin_jobs
    try:
        assert panel.start_pixhawk_usb_flash("left", "/dev/test-usb", str(firmware))
        job = owner._jobs[("usb_flash", "")]
        assert job.worker.entered.wait(1.0)
        job.worker.release.set()
        until(lambda: not owner._jobs)
        assert panel.pixhawk_usb_flash_status["state"] == "FAILED/UNKNOWN"
        assert "evidence disk full" in panel.pixhawk_usb_flash_status["error"]
        assert panel.mavlink_pixhawk is None
    finally:
        cleanup(owner, dispose=False)


def test_parameter_projection_keeps_ack_readback_actual_selection_separate():
    status = SimpleNamespace(actual_selected_device_id=11, expected_selected_device_id=22,
                             armed=True, params={"CAL_MAG0_ID": 11}, slots=())
    outcome = interpret_parameter_result({"camera": "right", "kind": "mag_apply", "status": status,
                                          "applied": ["CAL_MAG0_PRIO=50"], "missing": ["GPS2"]})
    manager = SimpleNamespace(actual_selected_device_id=None, armed=False)
    apply_mag_manager_readback(manager, outcome.mag_status)
    assert outcome.camera == "right"
    assert manager.actual_selected_device_id == 11
    assert manager.armed is True
    assert outcome.expected_device_id == 22
    assert outcome.verify_actual_selection is True
    assert outcome.missing_readback is None
    assert parameter_messages(outcome, "P2") == (
        "[PX4 MAG] P2 Workstation PARAM 적용: CAL_MAG0_PRIO=50", "[PX4 MAG] P2 미적용 role: GPS2")
    refresh = interpret_parameter_result({"camera": "right", "kind": "mag_refresh", "status": status})
    assert "CAL_MAG0_PRIO" in refresh.missing_readback
    assert refresh.verify_actual_selection is False


def test_parameter_legacy_role_fallback_is_finite_and_explicit_camera_wins():
    status = SimpleNamespace(actual_selected_device_id=22, params={}, slots=())
    legacy = {"kind": "mag_refresh", "status": status}
    inputs = dict(active_requests=(("left", "mag_refresh"), ("right", "mag_refresh")),
                  active_selections=(("left", 11), ("right", 22)))
    assert interpret_parameter_result(legacy, **inputs).camera == "right"
    assert interpret_parameter_result(dict(legacy, camera="drone"), **inputs).camera == "drone"
    assert interpret_parameter_result(legacy, active_requests=(("left", "mag_refresh"),)).camera == "left"


def test_attitude_presentation_preserves_asymmetric_sensor_axes_units_and_degree_offsets():
    status = SimpleNamespace(
        accel_slots=(SimpleNamespace(slot=0, device_id=101, priority=50, calibrated=True,
                                    offset_m_s2=(1.0, 2.0, 3.0), scale=(4.0, 5.0, 6.0)),),
        gyro_slots=(SimpleNamespace(slot=1, device_id=202, priority=40, calibrated=False,
                                   offset_rad_s=(7.0, 8.0, 9.0)),),
        board_rotation=SimpleNamespace(rotation_label="ROTATION_YAW_90", level_offset_deg=(10.0, None, -12.0)),
        warning_text="raw producer axes",
    )
    text = attitude_cal_status_text("P1", status, format_value=lambda value: f"{value:.1f}")
    assert text == (
        "P1 | PX4 자세 보정 상태\n"
        "ACC0: id=101 prio=50 calibrated=Y offset(m/s^2)=(1.0, 2.0, 3.0) scale=(4.0, 5.0, 6.0)\n"
        "GYRO1: id=202 prio=40 calibrated=N offset(rad/s)=(7.0, 8.0, 9.0)\n"
        "BOARD_ROT: ROTATION_YAW_90 level_offset=(10.0deg, N/A, -12.0deg)\nraw producer axes"
    )


def test_new_admin_and_result_modules_have_no_panel_or_transport_creation_dependency():
    root = Path(controller_module.__file__).parent / "device_runtime"
    for name in ("admin_commands.py", "admin_jobs.py", "qt_jobs.py", "discovery_jobs.py", "parameters.py", "presentation.py"):
        tree = ast.parse((root / name).read_text())
        assert not any(isinstance(node, ast.Attribute) and node.attr in {"panel", "mavlink_connection"}
                       for node in ast.walk(tree))
        assert not any(isinstance(node, ast.ImportFrom) and node.module and
                       ("standalone_gui" in node.module or "panel_" in node.module)
                       for node in ast.walk(tree))


def test_persistent_ethernet_builder_preserves_shell_and_file_bytes_against_golden():
    inputs = PersistentEthernetInputs(
        camera="right", eth_ip="10.8.7.3", host_ip="10.8.7.10",
        mavlink_url="udpin:0.0.0.0:15566/path", mavlink_rate=123456,
    )
    configuration = build_persistent_ethernet_configuration(inputs)
    expected_parameters = (
        "param set MAV_SYS_ID 2",
        "param set MAV_2_CONFIG 1000",
        "param set MAV_2_MODE 0",
        "param set MAV_2_BROADCAST 1",
        "param set MAV_2_REMOTE_PRT 15566",
        "param set MAV_2_UDP_PRT 15566",
        "param set MAV_2_RATE 123456",
    )
    assert configuration.param_lines == expected_parameters
    assert configuration.extras_text().encode("utf-8") == (
        b"ifup eth0\nifconfig eth0 10.8.7.3 netmask 255.255.255.0 up\n"
    )
    assert configuration.net_cfg_text().encode("utf-8") == (
        b"DEVICE=eth0\nBOOTPROTO=static\nNETMASK=255.255.255.0\nIPADDR=10.8.7.3\nROUTER=10.8.7.10\n"
    )
    assert configuration.persistent_command().encode("utf-8") == (
        b"mkdir /fs/microsd/etc\n"
        b"param set MAV_SYS_ID 2\n"
        b"param set MAV_2_CONFIG 1000\n"
        b"param set MAV_2_MODE 0\n"
        b"param set MAV_2_BROADCAST 1\n"
        b"param set MAV_2_REMOTE_PRT 15566\n"
        b"param set MAV_2_UDP_PRT 15566\n"
        b"param set MAV_2_RATE 123456\n"
        b"param save\n"
        b"ifup eth0\n"
        b"ifconfig eth0 10.8.7.3 netmask 255.255.255.0 up\n"
        b"mavlink status\n"
        b"echo 'ifup eth0' > /fs/microsd/etc/extras.txt\n"
        b"echo 'ifconfig eth0 10.8.7.3 netmask 255.255.255.0 up' >> /fs/microsd/etc/extras.txt\n"
        b"cat /fs/microsd/etc/extras.txt"
    )
    expected_bootstrap = (
        b"mkdir /fs/microsd/etc\n"
        b"param set MAV_SYS_ID 2\n"
        b"param set MAV_2_CONFIG 1000\n"
        b"param set MAV_2_MODE 0\n"
        b"param set MAV_2_BROADCAST 1\n"
        b"param set MAV_2_REMOTE_PRT 15566\n"
        b"param set MAV_2_UDP_PRT 15566\n"
        b"param set MAV_2_RATE 123456\n"
        b"param save\n"
        b"echo 'ifup eth0' > /fs/microsd/etc/extras.txt\n"
        b"echo 'ifconfig eth0 10.8.7.3 netmask 255.255.255.0 up' >> /fs/microsd/etc/extras.txt\n"
        b"echo 'DEVICE=eth0' > /fs/microsd/net.cfg\n"
        b"echo 'BOOTPROTO=static' >> /fs/microsd/net.cfg\n"
        b"echo 'NETMASK=255.255.255.0' >> /fs/microsd/net.cfg\n"
        b"echo 'IPADDR=10.8.7.3' >> /fs/microsd/net.cfg\n"
        b"echo 'ROUTER=10.8.7.10' >> /fs/microsd/net.cfg\n"
        b"reboot"
    )
    assert configuration.bootstrap_command(mode="normal").encode("utf-8") == expected_bootstrap
    assert configuration.bootstrap_command(mode="onboard").encode("utf-8") == expected_bootstrap
    assert "mavlink start" not in configuration.persistent_command()
    assert "mavlink stop" not in configuration.bootstrap_command()


@pytest.mark.parametrize("camera,expected_system,expected_port", [
    ("left", "param set MAV_SYS_ID 1", "param set MAV_2_REMOTE_PRT 14550"),
    ("right", "param set MAV_SYS_ID 2", "param set MAV_2_REMOTE_PRT 14551"),
    ("drone", "param set MAV_SYS_ID 3", "param set MAV_2_REMOTE_PRT 14552"),
])
def test_persistent_ethernet_builder_keeps_role_ids_and_legacy_port_fallback(camera, expected_system, expected_port):
    result = build_persistent_ethernet_configuration(PersistentEthernetInputs(
        camera, "10.0.0.2", "10.0.0.10", "endpoint-without-port", 2000000,
    ))
    assert result.param_lines[0] == expected_system
    assert result.param_lines[4] == expected_port
    assert result.param_lines[5] == expected_port.replace("REMOTE_PRT", "UDP_PRT")


def test_persistent_ethernet_builder_owns_immutable_result_and_detached_legacy_schema():
    inputs = PersistentEthernetInputs("left", "10.0.0.2", "10.0.0.10", "udp:14550", 2000000)
    result = build_persistent_ethernet_configuration(inputs)
    with pytest.raises(FrozenInstanceError):
        inputs.camera = "right"
    with pytest.raises(FrozenInstanceError):
        result.eth_ip = "10.0.0.3"
    legacy = result.legacy_setup()
    assert set(legacy) == {"eth_ip", "host_ip", "ifup_cmd", "ifconfig_cmd", "param_lines", "extras_lines", "net_cfg_lines"}
    assert isinstance(legacy["param_lines"], list)
    legacy["param_lines"][0] = "param set MAV_SYS_ID 99"
    legacy["extras_lines"].clear()
    assert result.param_lines[0] == "param set MAV_SYS_ID 1"
    assert result.extras_lines == ("ifup eth0", "ifconfig eth0 10.0.0.2 netmask 255.255.255.0 up")
    old_setup = {"param_lines": ["param set MAV_2_CONFIG 1000"],
                 "extras_lines": ["ifup eth0"], "net_cfg_lines": ["DEVICE=eth0"]}
    captured = PersistentEthernetConfiguration.from_legacy_setup(old_setup)
    old_setup["param_lines"].clear()
    assert captured.param_lines == ("param set MAV_2_CONFIG 1000",)


@pytest.mark.parametrize("invalid", ["", "10.0.0.2 x", "10.0.0.2\n", "10.0.0.2'", '10.0.0.2"', "10.0.0.2;", "10.0.0.2`", "10.0.0.2$"])
@pytest.mark.parametrize("field", ["eth_ip", "host_ip"])
def test_persistent_ethernet_builder_keeps_existing_shell_input_rejection(field, invalid):
    values = dict(camera="left", eth_ip="10.0.0.2", host_ip="10.0.0.10", mavlink_url="udp:14550", mavlink_rate=2000000)
    values[field] = invalid
    with pytest.raises(ValueError, match="shell 명령"):
        build_persistent_ethernet_configuration(PersistentEthernetInputs(**values))


def test_persistent_ethernet_builder_rejects_unknown_role_and_bootstrap_mode():
    with pytest.raises(ValueError, match="Unknown Pixhawk side: other"):
        build_persistent_ethernet_configuration(PersistentEthernetInputs("other", "10.0.0.2", "10.0.0.10", "udp:14550", 2000000))
    result = build_persistent_ethernet_configuration(PersistentEthernetInputs("left", "10.0.0.2", "10.0.0.10", "udp:14550", 2000000))
    with pytest.raises(ValueError, match="Unsupported Ethernet MAVLink bootstrap mode: other"):
        result.bootstrap_command(mode="other")


def test_persistent_controller_only_captures_assigned_host_and_explicit_overrides():
    panel = SimpleNamespace(
        left_pixhawk_eth_ip_edit=SimpleNamespace(text=lambda: "10.0.0.2"),
        left_mavlink_url_edit=SimpleNamespace(text=lambda: "udp:14550"),
        pixhawk_network_assignments={"left": {"local_address": "192.0.2.10/24"}},
        _pixhawk_host_ip_for_mavlink=lambda: pytest.fail("assigned host must keep priority"),
        mavlink_stream_rate_spin=SimpleNamespace(value=lambda: 987654),
    )
    setup = controller_module.DeviceRuntimeController(panel)._persistent_pixhawk_eth_setup(
        "left", eth_ip_override="192.0.2.2", mavlink_url_override="udp:15678/path",
    )
    assert setup["eth_ip"] == "192.0.2.2"
    assert setup["host_ip"] == "192.0.2.10"
    assert setup["param_lines"] == [
        "param set MAV_SYS_ID 1", "param set MAV_2_CONFIG 1000", "param set MAV_2_MODE 0",
        "param set MAV_2_BROADCAST 1", "param set MAV_2_REMOTE_PRT 15678",
        "param set MAV_2_UDP_PRT 15678", "param set MAV_2_RATE 987654",
    ]
    assert setup["extras_lines"] == ["ifup eth0", "ifconfig eth0 192.0.2.2 netmask 255.255.255.0 up"]
