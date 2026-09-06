"""Workstation device discovery, Pixhawk I/O, and configuration workers.

The classes in this module are UI-agnostic QObject workers owned by the
workstation app. StereoWIRIS app code is intentionally not imported.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from ipaddress import IPv4Address
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

from PyQt5 import QtCore

from Workstation.workstation_core.network import (
    DEFAULT_HOST_ADDRESS,
    NetworkRoleInterfaceRequest,
    apply_network_role_transaction,
    apply_interface_address,
    discover_pixhawk_network_candidates,
    discover_host_interface_candidates,
    interface_addresses,
    local_ipv4_interfaces,
    nm_connection_ipv4_addresses,
    nm_device_connection_name,
    pixhawk_route_evidence,
    resolve_nm_interface_address,
)
from Workstation.workstation_core.drone_network_discovery import (
    discover_drone_network_candidates,
)
from Workstation.workstation_core.pixhawk_attitude import (
    DualPixhawkMavlinkSubscriber,
    MAVLINK_GCS_SOURCE_COMPONENT,
    MAVLINK_GCS_SOURCE_SYSTEM,
    MavlinkShellCommandResult,
    UsbMavlinkIdentity,
    probe_usb_mavlink_devices,
    usb_mavlink_device_candidates,
)
from Workstation.workstation_core.pixhawk_mavftp import (
    MavFtpOffsetClient,
    atomic_write_transfer_evidence,
)
from Workstation.workstation_core.pixhawk_firmware import (
    parse_px_uploader_output,
    read_px4_firmware_identity,
    verify_pinned_uploader,
)
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    MAG_PARAM_NAMES,
    MAG_PRIORITY_PARAM_NAMES,
    RM3100_BACKUP_IDS,
    RM3100_PRIMARY_IDS,
    build_mag_status,
    mag_priority_slot_from_key,
    mag_priority_slot_key,
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
            normalized = [
                item.to_mapping()
                if isinstance(item, UsbMavlinkIdentity)
                else {
                    "device": str(item[0]),
                    "resolved_device": str(item[0]),
                    "baud": int(item[1]),
                    "system_id": None,
                    "component_id": None,
                    "observed_monotonic_s": None,
                }
                for item in found
            ]
            self.discovered.emit({"candidates": candidates, "found": normalized})
        finally:
            self.finished.emit()


class PixhawkUsbRecoveryWorker(QtCore.QObject):
    """Revalidate one USB heartbeat identity, send one reboot, and close."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        camera: str,
        evidence: dict[str, object],
        expected_system_id: int,
        timeout_s: float = 1.5,
    ) -> None:
        super().__init__()
        self.camera = str(camera)
        self.evidence = dict(evidence)
        self.expected_system_id = int(expected_system_id)
        self.timeout_s = max(0.1, float(timeout_s))

    @QtCore.pyqtSlot()
    def run(self) -> None:
        connection = None
        try:
            from pymavlink import mavutil

            device = str(self.evidence.get("device") or "")
            baud = int(self.evidence.get("baud") or 0)
            if not device or baud <= 0:
                raise RuntimeError("verified USB device/baud evidence is missing")
            connection = mavutil.mavlink_connection(
                device,
                baud=baud,
                autoreconnect=False,
                robust_parsing=True,
                source_system=MAVLINK_GCS_SOURCE_SYSTEM,
                source_component=MAVLINK_GCS_SOURCE_COMPONENT,
            )
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            heartbeat = connection.wait_heartbeat(timeout=self.timeout_s)
            if heartbeat is None:
                raise RuntimeError("USB heartbeat revalidation timed out; reboot not sent")
            system_id = int(heartbeat.get_srcSystem())
            component_id = int(heartbeat.get_srcComponent())
            if system_id != self.expected_system_id:
                raise RuntimeError(
                    f"USB system ID changed: expected {self.expected_system_id}, got {system_id}; reboot not sent"
                )
            recorded_component = self.evidence.get("component_id")
            if recorded_component is not None and int(recorded_component) != component_id:
                raise RuntimeError(
                    f"USB component ID changed: expected {int(recorded_component)}, got {component_id}; reboot not sent"
                )
            connection.mav.command_long_send(
                system_id,
                component_id,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            result = {
                "camera": self.camera,
                "device": device,
                "resolved_device": str(self.evidence.get("resolved_device") or device),
                "baud": baud,
                "system_id": system_id,
                "component_id": component_id,
                "revalidated_monotonic_s": time.monotonic(),
                "command_sent": True,
            }
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(result)
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
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


class NetworkRoleApplyWorker(QtCore.QObject):
    """Apply an approved multi-NIC role transaction off the GUI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, requests: Sequence[NetworkRoleInterfaceRequest]) -> None:
        super().__init__()
        self.requests = tuple(requests)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            results = apply_network_role_transaction(self.requests)
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(results)
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


class PixhawkNetworkDiscoveryWorker(QtCore.QObject):
    """Read wired NIC/address/route candidates without mutating host networking."""

    discovered = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        p1_peer: str,
        p2_peer: str,
        p1_endpoint: str,
        p2_endpoint: str,
    ) -> None:
        super().__init__()
        self.p1_peer = str(p1_peer)
        self.p2_peer = str(p2_peer)
        self.p1_endpoint = str(p1_endpoint)
        self.p2_endpoint = str(p2_endpoint)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            candidates = discover_pixhawk_network_candidates(
                p1_peer=self.p1_peer,
                p2_peer=self.p2_peer,
                p1_endpoint=self.p1_endpoint,
                p2_endpoint=self.p2_endpoint,
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.discovered.emit(tuple(candidate.to_mapping() for candidate in candidates))
        finally:
            self.finished.emit()


class DroneNetworkDiscoveryWorker(QtCore.QObject):
    """Passively find UDP MAVLink sources and retain their reported identity."""

    discovered = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, timeout_s: float = 3.0) -> None:
        super().__init__()
        self.timeout_s = float(timeout_s)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        def resolve_route(peer: str) -> tuple[str, str]:
            evidence = pixhawk_route_evidence(peer)
            return evidence.interface, evidence.source

        try:
            candidates = discover_drone_network_candidates(
                timeout_s=self.timeout_s,
                route_resolver=resolve_route,
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.discovered.emit(tuple(item.to_mapping() for item in candidates))
        finally:
            self.finished.emit()


class PixhawkConnectionWorker(QtCore.QObject):
    """Open a MAVLink transport without touching the Workstation event loop."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        camera: str | Sequence[str] | None,
    ) -> None:
        super().__init__()
        self.subscriber = subscriber
        if camera is None:
            self.cameras: tuple[str, ...] | None = None
            self.camera = None
        elif isinstance(camera, str):
            self.cameras = (str(camera),)
            self.camera = str(camera)
        else:
            self.cameras = tuple(str(value) for value in camera)
            self.camera = self.cameras[0] if len(self.cameras) == 1 else None

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            if self.cameras is None:
                self.subscriber.start(
                    None,
                    start_timer=False,
                    apply_pending_setup=False,
                )
            else:
                errors: list[str] = []
                for camera in self.cameras:
                    try:
                        self.subscriber.start(
                            camera,
                            start_timer=False,
                            apply_pending_setup=False,
                        )
                    except Exception as exc:
                        errors.append(f"{camera}: {exc}")
                if not any(
                    camera in getattr(self.subscriber, "connections", {})
                    for camera in self.cameras
                ):
                    raise RuntimeError("; ".join(errors) or "no requested Pixhawk connected")
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(
                {
                    "camera": self.camera,
                    "requested_cameras": self.cameras,
                    "connections": tuple(getattr(self.subscriber, "connections", {}).keys()),
                }
            )
        finally:
            self.finished.emit()


class PixhawkDisconnectWorker(QtCore.QObject):
    """Close a MAVLink transport without touching the Workstation event loop.

    ``DualPixhawkMavlinkSubscriber.stop()`` closes the underlying
    serial/socket connection, which can block for the duration of a
    concurrent blocking read held by an in-flight shell/parameter worker
    (they share ``_command_state_lock``). Running it synchronously on the
    GUI thread froze the whole app for that duration; this worker mirrors
    ``PixhawkConnectionWorker`` so disconnect no longer blocks the UI thread.
    """

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
            self.subscriber.stop(self.camera)
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


class PixhawkMavFtpTransferWorker(QtCore.QObject):
    """Run one explicit offset-aware MAVFTP upload for one Pixhawk role."""

    progress = QtCore.pyqtSignal(object)
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        *,
        camera: str,
        local_path: str,
        remote_path: str,
        resume: bool,
        evidence_path: str,
        direction: str = "upload",
    ) -> None:
        super().__init__()
        self.subscriber = subscriber
        self.camera = str(camera)
        self.local_path = str(local_path)
        self.remote_path = str(remote_path)
        self.resume = bool(resume)
        self.evidence_path = str(evidence_path)
        self.direction = str(direction)
        self._cancelled = threading.Event()

    @QtCore.pyqtSlot()
    def cancel(self) -> None:
        self._cancelled.set()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            connection = self.subscriber.connections.get(self.camera)
            if connection is None:
                raise RuntimeError(f"{self.camera} MAVLink connection is missing")
            send_lock = self.subscriber._mavlink_send_locks.setdefault(
                self.camera,
                threading.RLock(),
            )
            with self.subscriber.exclusive_command(
                self.camera,
                hold_send_lock=False,
            ):
                client = MavFtpOffsetClient(
                    connection,
                    send_lock=send_lock,
                    on_unmatched=lambda message: self.subscriber._defer_command_message(
                        self.camera,
                        message,
                    ),
                )
                progress = lambda sent, total: self.progress.emit(
                    {
                        "camera": self.camera,
                        "sent": int(sent),
                        "total": int(total),
                        "remote_path": self.remote_path,
                        "direction": self.direction,
                    }
                )
                if self.direction == "download":
                    evidence = client.download(
                        self.remote_path,
                        self.local_path,
                        resume=self.resume,
                        progress=progress,
                        cancelled=self._cancelled.is_set,
                    )
                elif self.direction == "upload":
                    evidence = client.upload(
                        self.local_path,
                        self.remote_path,
                        resume=self.resume,
                        progress=progress,
                        cancelled=self._cancelled.is_set,
                    )
                else:
                    raise RuntimeError(f"unsupported MAVFTP direction: {self.direction}")
            atomic_write_transfer_evidence(self.evidence_path, evidence)
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            result = evidence.to_mapping()
            result.update(
                camera=self.camera,
                evidence_path=self.evidence_path,
                direction=self.direction,
            )
            self.completed.emit(result)
        finally:
            self.finished.emit()


class PixhawkUsbFlashWorker(QtCore.QObject):
    """Verify direct USB identity and run the pinned PX4 bootloader uploader."""

    progress = QtCore.pyqtSignal(str)
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        camera: str,
        expected_system_id: int,
        usb_device: str,
        firmware_path: str,
        uploader_path: str,
    ) -> None:
        super().__init__()
        self.camera = str(camera)
        self.expected_system_id = int(expected_system_id)
        self.usb_device = str(usb_device)
        self.firmware_path = str(firmware_path)
        self.uploader_path = str(uploader_path)
        self._cancelled = threading.Event()
        self._process: subprocess.Popen[str] | None = None

    @QtCore.pyqtSlot()
    def cancel(self) -> None:
        self._cancelled.set()
        process = self._process
        if process is not None and process.poll() is None:
            process.terminate()

    def _probe_expected_usb(self, *, timeout_s: float) -> UsbMavlinkIdentity:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        last_seen: UsbMavlinkIdentity | None = None
        while time.monotonic() < deadline and not self._cancelled.is_set():
            identities = probe_usb_mavlink_devices(
                candidates=[self.usb_device],
                timeout_s=min(0.6, max(0.1, deadline - time.monotonic())),
            )
            if identities:
                last_seen = identities[0]
                if int(last_seen.system_id) == self.expected_system_id:
                    return last_seen
            time.sleep(0.1)
        if self._cancelled.is_set():
            raise RuntimeError("USB flash cancelled")
        if last_seen is None:
            raise RuntimeError("direct USB heartbeat was not received")
        raise RuntimeError(
            f"direct USB sysid mismatch: expected {self.expected_system_id}, got {last_seen.system_id}"
        )

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            resolved = os.path.realpath(self.usb_device)
            if not Path(resolved).name.startswith("ttyACM"):
                raise RuntimeError(
                    f"USB flash requires a direct Pixhawk CDC device, not a telemetry radio: {resolved}"
                )
            firmware = read_px4_firmware_identity(self.firmware_path)
            uploader_sha256 = verify_pinned_uploader(self.uploader_path)
            self.progress.emit("USB identity preflight")
            preflight = self._probe_expected_usb(timeout_s=3.0)
            command = [
                sys.executable,
                self.uploader_path,
                "--port",
                self.usb_device,
                self.firmware_path,
            ]
            self.progress.emit("Bootloader erase/program/verify")
            self._process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            output_lines: list[str] = []
            assert self._process.stdout is not None
            for line in self._process.stdout:
                output_lines.append(line)
                stripped = line.strip()
                if stripped:
                    self.progress.emit(stripped[-240:])
                if self._cancelled.is_set():
                    self._process.terminate()
                    break
            try:
                return_code = self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                # terminate() did not stop the child within the grace period
                # (e.g. it ignored SIGTERM). Force-kill and reap it so we
                # never leave an orphan process behind.
                self._process.kill()
                return_code = self._process.wait()
            output = "".join(output_lines)
            if self._cancelled.is_set():
                raise RuntimeError("USB flash cancelled")
            if return_code != 0:
                raise RuntimeError(f"PX4 uploader exited {return_code}: {output[-800:]}")
            uploader_result = parse_px_uploader_output(output)
            if uploader_result["detected_board_id"] != firmware.board_id:
                raise RuntimeError(
                    f"bootloader board mismatch: firmware {firmware.board_id}, "
                    f"detected {uploader_result['detected_board_id']}"
                )
            phases = tuple(uploader_result[name] for name in ("erase", "program", "verify", "reboot"))
            if phases != ("PASS", "PASS", "PASS", "PASS"):
                raise RuntimeError(f"USB uploader phase evidence is incomplete: {uploader_result}")
            self.progress.emit("Post-flash USB identity verification")
            postflight = self._probe_expected_usb(timeout_s=30.0)
            result = {
                "camera": self.camera,
                "expected_system_id": self.expected_system_id,
                "usb_device": self.usb_device,
                "resolved_usb_device": resolved,
                "uploader_sha256": uploader_sha256,
                "firmware": firmware.to_mapping(),
                "preflight": preflight.to_mapping(),
                "postflight": postflight.to_mapping(),
                **uploader_result,
                "state": "FLASHED/VERIFIED",
            }
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(result)
        finally:
            self._process = None
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

    MAX_INJECTION_AGE_S = 2.0

    request = QtCore.pyqtSignal(object, object)
    request_envelope = QtCore.pyqtSignal(object)
    injected = QtCore.pyqtSignal(object)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        *,
        max_queue: int = 96,
        max_injection_age_s: float = MAX_INJECTION_AGE_S,
    ) -> None:
        super().__init__()
        self.subscriber = subscriber
        self.max_queue = max(1, int(max_queue))
        self.max_injection_age_s = max(0.0, float(max_injection_age_s))
        self._queue: deque[
            tuple[
                tuple[str, ...],
                bytes,
                dict[str, object],
                dict[str, dict[str, object]],
            ]
        ] = deque()
        self._queue_lock = threading.Lock()
        self._draining = False

    @QtCore.pyqtSlot(object, object)
    def enqueue(self, sides: object, payload: object) -> None:
        self._enqueue(sides, payload, {}, drain_immediately=True)

    @QtCore.pyqtSlot(object)
    def enqueue_envelope(self, request: object) -> None:
        self._enqueue_envelope(request, drain_immediately=True)

    @QtCore.pyqtSlot(object)
    def submit_envelope(self, request: object) -> None:
        """Thread-safe bounded handoff used directly by the RTCM pipeline."""

        self._enqueue_envelope(request, drain_immediately=False)

    def _enqueue_envelope(self, request: object, *, drain_immediately: bool) -> None:
        values = dict(request) if isinstance(request, dict) else {}
        metadata = {
            key: values[key]
            for key in (
                "pipeline_sequence",
                "source",
                "targets",
                "payload_sha256",
                "receive_monotonic_ns",
                "message_type",
                "evidence_context_token",
                "evidence_session_id",
                "evidence_session_root",
            )
            if key in values
        }
        self._enqueue(
            values.get("targets"),
            values.get("payload"),
            metadata,
            drain_immediately=drain_immediately,
        )

    def _enqueue(
        self,
        sides: object,
        payload: object,
        metadata: dict[str, object],
        *,
        drain_immediately: bool,
    ) -> None:
        target_sides = tuple(str(side) for side in (sides or ()))
        frame = bytes(payload or b"")
        if not target_sides or not frame:
            return
        enqueued_health = self._capture_target_health(target_sides)
        with self._queue_lock:
            dropped = 0
            dropped_metadata: list[dict[str, object]] = []
            while len(self._queue) >= self.max_queue:
                _old_sides, _old_payload, old_metadata, _old_health = self._queue.popleft()
                if old_metadata:
                    dropped_metadata.append(dict(old_metadata))
                dropped += 1
            self._queue.append((target_sides, frame, dict(metadata), enqueued_health))
            should_start = not self._draining
            if should_start:
                self._draining = True
        if dropped:
            result: dict[str, object] = {
                "dropped": dropped,
                "payload_bytes": 0,
                "sent": (),
                "failed": (),
            }
            if dropped_metadata:
                result["dropped_envelopes"] = tuple(dropped_metadata)
            self.injected.emit(result)
        if should_start:
            if drain_immediately:
                self._drain_one()
            else:
                QtCore.QMetaObject.invokeMethod(
                    self,
                    "_drain_one",
                    QtCore.Qt.QueuedConnection,
                )

    @staticmethod
    def _state_text(health: object) -> str:
        state = getattr(health, "state", "")
        return str(getattr(state, "value", state) or "")

    def _health_snapshot(self, side: str) -> object | None:
        reader = getattr(self.subscriber, "mavlink_health_snapshot", None)
        if not callable(reader):
            return None
        try:
            return reader(str(side))
        except Exception:
            return None

    def _health_record(self, health: object) -> dict[str, object]:
        return {
            "eligible": bool(getattr(health, "rtcm_eligible", False)),
            "connection_generation": int(getattr(health, "connection_generation", 0) or 0),
            "state": self._state_text(health),
            "last_target_rx_age_s": getattr(health, "any_rx_age_s", None),
            "stall_reason": str(getattr(health, "stall_reason", "") or ""),
        }

    def _capture_target_health(self, sides: tuple[str, ...]) -> dict[str, dict[str, object]]:
        captured: dict[str, dict[str, object]] = {}
        for side in sides:
            health = self._health_snapshot(side)
            if health is not None:
                captured[side] = self._health_record(health)
        return captured

    def _blocked_target(
        self,
        side: str,
        *,
        health: object | None = None,
        record: dict[str, object] | None = None,
        reason: str | None = None,
    ) -> dict[str, object]:
        values = dict(record or (self._health_record(health) if health is not None else {}))
        state = str(values.get("state") or "")
        category = "stall" if state == "ETH MAVLINK RX STALLED" else "liveness"
        if reason == "connection-generation-changed":
            category = "generation"
        return {
            "side": str(side),
            "category": category,
            "reason": str(reason or values.get("stall_reason") or state or "target-mavlink-ineligible"),
            "state": state,
            "last_target_rx_age_s": values.get("last_target_rx_age_s"),
            "connection_generation": int(values.get("connection_generation") or 0),
        }

    @QtCore.pyqtSlot()
    def _drain_one(self) -> None:
        with self._queue_lock:
            if not self._queue:
                self._draining = False
                return
            sides, payload, metadata, enqueued_health = self._queue.popleft()
        sent: list[tuple[str, int]] = []
        failed: list[tuple[str, str]] = []
        stale_targets: list[str] = []
        blocked_targets: list[dict[str, object]] = []
        delivery_monotonic_s_by_target: dict[str, float] = {}
        receive_monotonic_ns = int(metadata.get("receive_monotonic_ns") or 0)
        last_age_s = 0.0
        for index, side in enumerate(sides):
            initial_health = enqueued_health.get(side)
            if initial_health is not None and not bool(initial_health.get("eligible", False)):
                blocked_targets.append(self._blocked_target(side, record=initial_health))
                continue
            try:
                if receive_monotonic_ns > 0:
                    last_age_s = max(
                        0.0,
                        (time.monotonic_ns() - receive_monotonic_ns)
                        / 1_000_000_000.0,
                    )
                if (
                    receive_monotonic_ns > 0
                    and self.max_injection_age_s > 0.0
                    and last_age_s > self.max_injection_age_s
                ):
                    stale_targets.extend(str(value) for value in sides[index:])
                    break
                guarded_sender = getattr(self.subscriber, "send_rtcm_data_if_eligible", None)
                if callable(guarded_sender):
                    expected_generation = (
                        None
                        if initial_health is None
                        else int(initial_health.get("connection_generation") or 0)
                    )
                    outcome = dict(
                        guarded_sender(
                            side,
                            payload,
                            expected_generation=expected_generation,
                        )
                        or {}
                    )
                    if not bool(outcome.get("sent", False)):
                        blocked_targets.append(
                            self._blocked_target(
                                side,
                                health=outcome.get("health"),
                                reason=str(outcome.get("block_reason") or "target-mavlink-ineligible"),
                            )
                        )
                        continue
                    packet_count = int(outcome.get("packet_count") or 0)
                else:
                    packet_count = int(self.subscriber.send_rtcm_data(side, payload) or 0)
                sent.append((side, packet_count))
                delivery_monotonic_s_by_target[side] = time.monotonic()
            except Exception as exc:
                failed.append((side, str(exc)))
        result: dict[str, object] = {
            "payload_bytes": len(payload),
            "sent": tuple(sent),
            "failed": tuple(failed),
            "dropped": 1 if stale_targets else 0,
        }
        result.update(metadata)
        if delivery_monotonic_s_by_target and int(metadata.get("pipeline_sequence") or 0) > 0:
            result["delivery_monotonic_s_by_target"] = dict(
                delivery_monotonic_s_by_target
            )
            result["last_delivery_s"] = max(
                delivery_monotonic_s_by_target.values()
            )
        if stale_targets:
            result.update(
                {
                    "drop_reason": "stale-before-send",
                    "age_s": last_age_s,
                    "dropped_targets": tuple(stale_targets),
                    "terminal_outcome": "stale" if not sent else "partial-stale",
                }
            )
        if blocked_targets:
            blocked_sides = tuple(str(item["side"]) for item in blocked_targets)
            blocked_stall = sum(1 for item in blocked_targets if item.get("category") == "stall")
            blocked_generation = sum(1 for item in blocked_targets if item.get("category") == "generation")
            target_ages = {
                str(item["side"]): item.get("last_target_rx_age_s")
                for item in blocked_targets
            }
            target_generations = {
                str(item["side"]): int(item.get("connection_generation") or 0)
                for item in blocked_targets
            }
            result.update(
                {
                    "blocked_targets": tuple(blocked_targets),
                    "blocked_stall": blocked_stall,
                    "blocked_generation": blocked_generation,
                    "target_rx_age_s": target_ages,
                    "target_connection_generations": target_generations,
                }
            )
            if not stale_targets:
                result.update(
                    {
                        "drop_reason": (
                            "target-mavlink-stalled"
                            if blocked_stall
                            else "target-mavlink-generation-changed"
                            if blocked_generation
                            else "target-mavlink-ineligible"
                        ),
                        "dropped_targets": blocked_sides,
                        "terminal_outcome": "blocked" if not sent else "partial-blocked",
                    }
                )
        self.injected.emit(result)
        QtCore.QTimer.singleShot(0, self._drain_one)

    def clear_pending(
        self,
        *,
        reason: str = "worker-retired",
    ) -> tuple[dict[str, object], ...]:
        with self._queue_lock:
            pending = tuple(self._queue)
            self._queue.clear()
        outcomes: list[dict[str, object]] = []
        for sides, payload, metadata, _enqueued_health in pending:
            outcome: dict[str, object] = {
                "payload_bytes": len(payload),
                "sent": (),
                "failed": (),
                "dropped": 1,
            }
            outcome.update(metadata)
            outcome.update(
                {
                    "drop_reason": str(reason),
                    "dropped_targets": tuple(sides),
                    "terminal_outcome": "cancelled",
                }
            )
            outcomes.append(outcome)
            self.injected.emit(outcome)
        return tuple(outcomes)


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
        command: str | tuple[str, ...],
        operation: str = "shell",
        timeout_s: float = 2.0,
    ) -> None:
        super().__init__()
        self.subscriber = subscriber
        self.camera = str(camera)
        self.command = command
        self.operation = str(operation)
        self.timeout_s = max(0.1, float(timeout_s))

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            commands = (
                tuple(str(item) for item in self.command)
                if isinstance(self.command, (tuple, list))
                else (str(self.command),)
            )
            command_text = "; ".join(commands)
            command_label = (
                "RM3100 SHELL POLL"
                if self.operation == "sensor_mag_shell_poll"
                else "PX4 SHELL"
            )
            try:
                command_context = self.subscriber.exclusive_command(
                    self.camera,
                    command_label=command_label,
                    preserve_qualification=self.operation == "sensor_mag_shell_poll",
                )
            except TypeError:
                # Keep older test doubles/compatibility adapters usable while
                # production subscribers expose the explicit command label.
                command_context = self.subscriber.exclusive_command(self.camera)
            with command_context:
                if self.operation == "reboot":
                    self.subscriber.reboot_autopilot(self.camera)
                    result: dict[str, object] = {
                        "camera": self.camera,
                        "operation": self.operation,
                        "command": command_text,
                        "output": "",
                        "flag_label": "command-sent",
                    }
                else:
                    output_parts: list[str] = []
                    flag_labels: list[str] = []
                    for command in commands:
                        shell_result = self.subscriber.send_shell_command_with_result(
                            self.camera,
                            command,
                            timeout_s=self.timeout_s,
                        )
                        if isinstance(shell_result, MavlinkShellCommandResult):
                            output_parts.append(shell_result.output)
                            flag_labels.append(shell_result.flag_label)
                        else:
                            output_parts.append(str(getattr(shell_result, "output", "") or ""))
                            flag_labels.append(str(getattr(shell_result, "flag_label", "unknown")))
                    result = {
                        "camera": self.camera,
                        "operation": self.operation,
                        "command": command_text,
                        "output": "".join(output_parts),
                        "flag_label": "+".join(dict.fromkeys(label for label in flag_labels if label)),
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

    def _read_mag_priority_params(self) -> dict[str, int | float | None]:
        """Read only the identity/priority surface needed by ``mag_apply``.

        Full correction coefficients remain available through the explicit
        MAG refresh path. Priority Apply must not spend dozens of Radio PARAM
        round trips reading unrelated REAL32 calibration fields before it can
        write the requested priority.
        """

        return {
            name: self.subscriber.read_param(self.camera, name, timeout_s=1.5)
            for name in MAG_PRIORITY_PARAM_NAMES
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
        params = self._read_mag_priority_params()
        before = build_mag_status(
            params,
            actual_selected_device_id=self.actual_selected_device_id,
            armed=self.armed,
        )
        applied: list[str] = []
        missing = set(self.priorities)
        slot_priorities: dict[int, int] = {}
        role_priorities: dict[str, int] = {}
        for key, value in self.priorities.items():
            slot = mag_priority_slot_from_key(key)
            if slot is None:
                # Backward compatibility for callers that still submit the
                # old logical-role payload. The visible UI submits CAL_MAGn.
                role_priorities[str(key)] = int(value)
            else:
                slot_priorities[slot] = int(value)
        for slot in before.slots:
            role = self._role_for_device_id(slot.device_id)
            slot_key = mag_priority_slot_key(slot.slot)
            if slot.slot in slot_priorities:
                expected = slot_priorities[slot.slot]
                missing.discard(slot_key)
            elif role is not None and role in role_priorities:
                expected = role_priorities[role]
                missing.discard(role)
            else:
                continue
            name = f"CAL_MAG{slot.slot}_PRIO"
            actual = self.subscriber.set_param_int(self.camera, name, expected, timeout_s=2.0)
            if actual != expected:
                raise RuntimeError(f"{name} readback mismatch: expected={expected}, actual={actual}")
            identity = role or f"ID={slot.device_id if slot.device_id is not None else 'N/A'}"
            applied.append(f"{slot_key}({identity}):{name}={expected}")
        if not applied:
            raise RuntimeError("적용 가능한 BOARD/CAN/GPS MAG 슬롯을 찾지 못했습니다.")
        after_params = self._read_mag_priority_params()
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
