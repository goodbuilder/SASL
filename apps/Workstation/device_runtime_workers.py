"""Workstation device discovery, Pixhawk I/O, and configuration workers.

The classes in this module are UI-agnostic QObject workers owned by the
workstation app. StereoWIRIS app code is intentionally not imported.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from ipaddress import IPv4Address
import threading

from PyQt5 import QtCore

from Workstation.workstation_core.network import (
    DEFAULT_HOST_ADDRESS,
    apply_interface_address,
    discover_host_interface_candidates,
    interface_addresses,
    local_ipv4_interfaces,
    nm_connection_ipv4_addresses,
    nm_device_connection_name,
    resolve_nm_interface_address,
)
from Workstation.workstation_core.pixhawk_attitude import (
    DualPixhawkMavlinkSubscriber,
    MavlinkShellCommandResult,
    probe_usb_mavlink_devices,
    usb_mavlink_device_candidates,
)
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    MAG_PARAM_NAMES,
    RM3100_BACKUP_IDS,
    RM3100_PRIMARY_IDS,
    build_mag_status,
)
from Workstation.workstation_core.px4_attitude_cal_manager import (
    ATTITUDE_CAL_PARAM_NAMES,
    build_attitude_cal_status,
)


class PixhawkUsbDiscoveryWorker(QtCore.QObject):
    """Probe USB serial candidates without blocking the owner UI."""

    discovered = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, timeout_s: float = 0.8) -> None:
        super().__init__()
        self.timeout_s = max(0.1, float(timeout_s))

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            candidates = usb_mavlink_device_candidates()
            found = probe_usb_mavlink_devices(candidates=candidates, timeout_s=self.timeout_s) if candidates else []
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.discovered.emit({"candidates": candidates, "found": found})
        finally:
            self.finished.emit()


class CameraNetworkDiscoveryWorker(QtCore.QObject):
    """Discover camera-network interfaces and NetworkManager addresses off the UI thread."""

    discovered = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, host_address: str, host_hints: Sequence[str]) -> None:
        super().__init__()
        self.host_address = str(host_address)
        self.host_hints = tuple(str(value) for value in host_hints)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            candidates = discover_host_interface_candidates(
                host_address=self.host_address or DEFAULT_HOST_ADDRESS,
                include_unconfigured=True,
            )
            if not candidates:
                result = {"candidates": (), "resolved_address": None, "connection_name": None, "addresses": ()}
            else:
                selected = candidates[0]
                interface_name = str(getattr(selected.interface, "name", ""))
                resolved_address = resolve_nm_interface_address(interface_name, self.host_hints)
                connection_name = nm_device_connection_name(interface_name)
                addresses = () if connection_name is None else nm_connection_ipv4_addresses(connection_name)
                result = {
                    "candidates": tuple(candidates),
                    "resolved_address": resolved_address,
                    "connection_name": connection_name,
                    "addresses": tuple(addresses),
                }
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.discovered.emit(result)
        finally:
            self.finished.emit()


class CameraNetworkAddressApplyWorker(QtCore.QObject):
    """Apply a selected camera-network address without blocking the UI."""

    applied = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, interface_name: str, address_text: str) -> None:
        super().__init__()
        self.interface_name = str(interface_name)
        self.address_text = str(address_text)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            ok, message = apply_interface_address(self.interface_name, self.address_text)
            addresses = interface_addresses(self.interface_name) if ok else ""
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.applied.emit(
                {
                    "ok": bool(ok),
                    "message": str(message),
                    "interface_name": self.interface_name,
                    "address_text": self.address_text,
                    "addresses": addresses,
                }
            )
        finally:
            self.finished.emit()


class InterfaceStatusWorker(QtCore.QObject):
    """Read one local interface status without blocking panel construction."""

    ready = QtCore.pyqtSignal(str, str)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, interface_name: str) -> None:
        super().__init__()
        self.interface_name = str(interface_name)

    def _emit_if_alive(self, signal_name: str, *args: object) -> bool:
        """Ignore a late result when Qt has already destroyed this startup worker."""

        try:
            getattr(self, signal_name).emit(*args)
        except RuntimeError:
            return False
        return True

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            status = interface_addresses(self.interface_name)
        except Exception as exc:
            self._emit_if_alive("failed", str(exc))
        else:
            self._emit_if_alive("ready", self.interface_name, str(status))
        finally:
            self._emit_if_alive("finished")


class PixhawkHostIpDiscoveryWorker(QtCore.QObject):
    """Enumerate local IPv4 interfaces for Pixhawk host candidates off the UI thread."""

    discovered = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            interfaces = local_ipv4_interfaces(include_wireless=True)
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.discovered.emit(tuple(interfaces))
        finally:
            self.finished.emit()


class PixhawkConnectionWorker(QtCore.QObject):
    """Open a MAVLink transport without touching the Workstation event loop."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, subscriber: DualPixhawkMavlinkSubscriber, camera: str | None) -> None:
        super().__init__()
        self.subscriber = subscriber
        self.camera = None if camera is None else str(camera)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            self.subscriber.start(
                self.camera,
                start_timer=False,
                apply_pending_setup=False,
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(
                {
                    "camera": self.camera,
                    "connections": tuple(getattr(self.subscriber, "connections", {}).keys()),
                }
            )
        finally:
            self.finished.emit()


class PixhawkStreamSetupWorker(QtCore.QObject):
    """Apply the optional PX4 MAVLink stream setup away from the UI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        *,
        host_ip: str,
        ports: dict[str, int],
        sysids: dict[str, int],
        rate: int,
        cameras: Sequence[str],
    ) -> None:
        super().__init__()
        self.subscriber = subscriber
        self.host_ip = str(host_ip)
        self.ports = {str(key): int(value) for key, value in dict(ports).items()}
        self.sysids = {str(key): int(value) for key, value in dict(sysids).items()}
        self.rate = int(rate)
        self.cameras = tuple(str(camera) for camera in cameras)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            self.subscriber.configure_mavlink_streams(
                host_ip=self.host_ip,
                ports=self.ports,
                sysids=self.sysids,
                rate=self.rate,
                cameras=self.cameras,
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(
                {
                    "host_ip": self.host_ip,
                    "ports": dict(self.ports),
                    "cameras": self.cameras,
                }
            )
        finally:
            self.finished.emit()


class PixhawkUsbBootstrapWorker(QtCore.QObject):
    """Bootstrap one Ethernet MAVLink stream over a temporary USB link."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        camera: str,
        usb_url: str,
        baud_candidates: Sequence[int],
        command: str,
        fallback_command: str,
    ) -> None:
        super().__init__()
        self.camera = str(camera)
        self.usb_url = str(usb_url)
        self.baud_candidates = tuple(int(value) for value in baud_candidates if int(value) > 0)
        self.command = str(command)
        self.fallback_command = str(fallback_command)

    @staticmethod
    def _heartbeat(subscriber: DualPixhawkMavlinkSubscriber, camera: str) -> bool:
        connection = getattr(subscriber, "connections", {}).get(camera)
        if connection is None:
            return False
        wait_heartbeat = getattr(connection, "wait_heartbeat", None)
        if wait_heartbeat is None:
            return True
        try:
            return wait_heartbeat(timeout=1.2) is not None
        except Exception:
            return False

    @QtCore.pyqtSlot()
    def run(self) -> None:
        last_error = ""
        selected_baud: int | None = None
        output = ""
        flag_label = ""
        for candidate_baud in self.baud_candidates:
            temp = DualPixhawkMavlinkSubscriber(
                left_url=self.usb_url if self.camera == "left" else "",
                right_url=self.usb_url if self.camera == "right" else "",
                left_baud=candidate_baud,
                right_baud=candidate_baud,
            )
            try:
                temp.start(self.camera, start_timer=False, apply_pending_setup=False)
                if not self._heartbeat(temp, self.camera):
                    last_error = f"{candidate_baud} baud heartbeat timeout"
                    continue
                with temp.exclusive_command(self.camera):
                    result = temp.send_shell_command_with_result(
                        self.camera,
                        self.command,
                        timeout_s=2.0,
                    )
                    if "invalid mode" in result.output.lower() and self.fallback_command:
                        result = temp.send_shell_command_with_result(
                            self.camera,
                            self.fallback_command,
                            timeout_s=2.0,
                        )
                selected_baud = candidate_baud
                output = str(result.output or "")
                flag_label = str(result.flag_label or "")
                break
            except Exception as exc:
                last_error = f"{candidate_baud} baud: {exc}"
            finally:
                try:
                    temp.stop(self.camera)
                except Exception:
                    pass
        if selected_baud is None:
            self.failed.emit(last_error or "USB MAVLink bootstrap failed")
        else:
            self.completed.emit(
                {
                    "camera": self.camera,
                    "baud": selected_baud,
                    "output": output,
                    "flag_label": flag_label,
                }
            )
        self.finished.emit()


class PixhawkPersistentConfigWorker(QtCore.QObject):
    """Apply PX4 Ethernet settings and persist their files off the UI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        *,
        camera: str,
        command: str,
        extras_text: str,
        net_cfg_text: str,
    ) -> None:
        super().__init__()
        self.subscriber = subscriber
        self.camera = str(camera)
        self.command = str(command)
        self.extras_text = str(extras_text)
        self.net_cfg_text = str(net_cfg_text)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            with self.subscriber.exclusive_command(self.camera):
                result = self.subscriber.send_shell_command_with_result(
                    self.camera,
                    self.command,
                    timeout_s=3.0,
                )
                self.subscriber.write_remote_file_mavftp(
                    self.camera,
                    "/fs/microsd/etc/extras.txt",
                    self.extras_text,
                )
                self.subscriber.write_remote_file_mavftp(
                    self.camera,
                    "/fs/microsd/net.cfg",
                    self.net_cfg_text,
                )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(
                {
                    "camera": self.camera,
                    "output": str(getattr(result, "output", "") or ""),
                    "flag_label": str(getattr(result, "flag_label", "") or ""),
                }
            )
        finally:
            self.finished.emit()


class PixhawkRtcmInjectionWorker(QtCore.QObject):
    """Queue RTCM packets so GNSS correction delivery never runs in the UI."""

    request = QtCore.pyqtSignal(object, object)
    injected = QtCore.pyqtSignal(object)
    finished = QtCore.pyqtSignal()

    def __init__(self, subscriber: DualPixhawkMavlinkSubscriber, *, max_queue: int = 96) -> None:
        super().__init__()
        self.subscriber = subscriber
        self.max_queue = max(1, int(max_queue))
        self._queue: deque[tuple[tuple[str, ...], bytes]] = deque()
        self._queue_lock = threading.Lock()
        self._draining = False

    @QtCore.pyqtSlot(object, object)
    def enqueue(self, sides: object, payload: object) -> None:
        target_sides = tuple(str(side) for side in (sides or ()))
        frame = bytes(payload or b"")
        if not target_sides or not frame:
            return
        with self._queue_lock:
            dropped = 0
            while len(self._queue) >= self.max_queue:
                self._queue.popleft()
                dropped += 1
            self._queue.append((target_sides, frame))
            if self._draining:
                return
            self._draining = True
        if dropped:
            self.injected.emit({"dropped": dropped, "payload_bytes": 0, "sent": (), "failed": ()})
        self._drain_one()

    def _drain_one(self) -> None:
        with self._queue_lock:
            if not self._queue:
                self._draining = False
                return
            sides, payload = self._queue.popleft()
        sent: list[tuple[str, int]] = []
        failed: list[tuple[str, str]] = []
        for side in sides:
            try:
                with self.subscriber.exclusive_command(side):
                    packet_count = int(self.subscriber.send_rtcm_data(side, payload) or 0)
                sent.append((side, packet_count))
            except Exception as exc:
                failed.append((side, str(exc)))
        self.injected.emit(
            {
                "payload_bytes": len(payload),
                "sent": tuple(sent),
                "failed": tuple(failed),
                "dropped": 0,
            }
        )
        QtCore.QTimer.singleShot(0, self._drain_one)

    def clear_pending(self) -> None:
        with self._queue_lock:
            self._queue.clear()


class PixhawkShellCommandWorker(QtCore.QObject):
    """Run one blocking MAVLink shell/reboot operation off the UI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        *,
        camera: str,
        command: str,
        operation: str = "shell",
        timeout_s: float = 2.0,
    ) -> None:
        super().__init__()
        self.subscriber = subscriber
        self.camera = str(camera)
        self.command = str(command)
        self.operation = str(operation)
        self.timeout_s = max(0.1, float(timeout_s))

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            with self.subscriber.exclusive_command(self.camera):
                if self.operation == "reboot":
                    self.subscriber.reboot_autopilot(self.camera)
                    result: dict[str, object] = {
                        "camera": self.camera,
                        "operation": self.operation,
                        "command": self.command,
                        "output": "",
                        "flag_label": "command-sent",
                    }
                else:
                    shell_result = self.subscriber.send_shell_command_with_result(
                        self.camera,
                        self.command,
                        timeout_s=self.timeout_s,
                    )
                    if isinstance(shell_result, MavlinkShellCommandResult):
                        result = {
                            "camera": self.camera,
                            "operation": self.operation,
                            "command": shell_result.command,
                            "output": shell_result.output,
                            "flag_label": shell_result.flag_label,
                        }
                    else:
                        result = {
                            "camera": self.camera,
                            "operation": self.operation,
                            "command": self.command,
                            "output": str(getattr(shell_result, "output", "") or ""),
                            "flag_label": str(getattr(shell_result, "flag_label", "unknown")),
                        }
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(result)
        finally:
            self.finished.emit()


class PixhawkParameterWorker(QtCore.QObject):
    """Read or update PX4 parameters without blocking the Workstation UI."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        *,
        camera: str,
        operation: str,
        actual_selected_device_id: int | None = None,
        armed: bool = False,
        priorities: dict[str, int] | None = None,
    ) -> None:
        super().__init__()
        self.subscriber = subscriber
        self.camera = str(camera)
        self.operation = str(operation)
        self.actual_selected_device_id = actual_selected_device_id
        self.armed = bool(armed)
        self.priorities = {str(key): int(value) for key, value in dict(priorities or {}).items()}

    def _read_mag_params(self) -> dict[str, int | float | None]:
        return {
            # MAG calibration includes REAL32 offset/scale/soft-iron values;
            # using the integer-only reader would destroy the QGC result.
            name: self.subscriber.read_param(self.camera, name, timeout_s=1.5)
            for name in MAG_PARAM_NAMES
        }

    def _read_attitude_params(self) -> dict[str, int | float | None]:
        return {
            name: self.subscriber.read_param(self.camera, name, timeout_s=1.5)
            for name in ATTITUDE_CAL_PARAM_NAMES
        }

    @staticmethod
    def _role_for_device_id(device_id: int | None) -> str | None:
        if device_id == BOARD_INTERNAL_MAG_ID:
            return "BOARD"
        if device_id in RM3100_PRIMARY_IDS:
            return "CAN1"
        if device_id in RM3100_BACKUP_IDS:
            return "CAN2"
        if device_id == GPS1_MAG_ID:
            return "GPS1_MAG"
        if device_id == GPS2_MAG_ID:
            return "GPS2_MAG"
        return None

    def _run_mag_apply(self) -> dict[str, object]:
        if self.armed:
            raise RuntimeError("Disarm required before changing PX4 magnetometer parameters.")
        params = self._read_mag_params()
        before = build_mag_status(
            params,
            actual_selected_device_id=self.actual_selected_device_id,
            armed=self.armed,
        )
        applied: list[str] = []
        missing = set(self.priorities)
        for slot in before.slots:
            role = self._role_for_device_id(slot.device_id)
            if role is None or role not in self.priorities:
                continue
            name = f"CAL_MAG{slot.slot}_PRIO"
            expected = int(self.priorities[role])
            actual = self.subscriber.set_param_int(self.camera, name, expected, timeout_s=2.0)
            if actual != expected:
                raise RuntimeError(f"{name} readback mismatch: expected={expected}, actual={actual}")
            applied.append(f"{role}:{name}={expected}")
            missing.discard(role)
        if not applied:
            raise RuntimeError("적용 가능한 BOARD/CAN/GPS MAG 슬롯을 찾지 못했습니다.")
        after_params = self._read_mag_params()
        status = build_mag_status(
            after_params,
            actual_selected_device_id=self.actual_selected_device_id,
            armed=self.armed,
        )
        return {
            "kind": self.operation,
            "camera": self.camera,
            "status": status,
            "applied": tuple(applied),
            "missing": tuple(sorted(missing)),
        }

    @QtCore.pyqtSlot()
    def run(self) -> None:
        # Each individual read_param/read_param_int/set_param_int call below
        # already reserves the receive path for itself via the subscriber's
        # own exclusive_command(camera). Wrapping this whole method in a
        # second, outer exclusive_command(camera) used to make every single
        # operation fail immediately: the guard treats a second reservation
        # for the same camera as another command already in progress and
        # raises, even though it is this same worker calling into itself.
        try:
            if self.operation == "mag_refresh":
                status = build_mag_status(
                    self._read_mag_params(),
                    actual_selected_device_id=self.actual_selected_device_id,
                    armed=self.armed,
                )
                result = {"kind": self.operation, "camera": self.camera, "status": status}
            elif self.operation == "attitude_refresh":
                status = build_attitude_cal_status(self._read_attitude_params())
                result = {"kind": self.operation, "camera": self.camera, "status": status}
            elif self.operation == "calibration_refresh":
                mag_status = build_mag_status(
                    self._read_mag_params(),
                    actual_selected_device_id=self.actual_selected_device_id,
                    armed=self.armed,
                )
                attitude_status = build_attitude_cal_status(self._read_attitude_params())
                result = {
                    "kind": self.operation,
                    "camera": self.camera,
                    "mag_status": mag_status,
                    "attitude_status": attitude_status,
                }
            elif self.operation == "mag_apply":
                result = self._run_mag_apply()
            else:
                raise ValueError(f"Unknown Pixhawk parameter operation: {self.operation}")
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(result)
        finally:
            self.finished.emit()
