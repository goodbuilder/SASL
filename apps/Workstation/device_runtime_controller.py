"""Workstation camera-network and Pixhawk runtime orchestration.

This controller owns workstation-specific worker lifecycles and UI state.
It does not reinterpret PX4 body-FRD, earth-NED, GPS, or camera coordinates.
"""

from __future__ import annotations

import time
from collections.abc import Sequence
from ipaddress import IPv4Address

from PyQt5 import QtCore, QtWidgets

from Workstation.device_runtime_workers import (
    CameraNetworkAddressApplyWorker,
    CameraNetworkDiscoveryWorker,
    PixhawkConnectionWorker,
    PixhawkDisconnectWorker,
    PixhawkHostIpDiscoveryWorker,
    PixhawkParameterWorker,
    PixhawkPersistentConfigWorker,
    PixhawkShellCommandWorker,
    PixhawkStreamSetupWorker,
    PixhawkUsbBootstrapWorker,
    PixhawkUsbRecoveryWorker,
)
from Workstation.workstation_core.network import DEFAULT_HOST_ADDRESS
from Workstation.workstation_core.pixhawk_attitude import (
    DualPixhawkMavlinkSubscriber,
    USB_MAVLINK_IDENTITY_MAX_AGE_S,
    camera_label_ko,
)
from Workstation.workstation_core.px4_attitude_cal_manager import (
    LEVEL_HORIZON_RESET_PARAM_NAMES,
)
from Workstation.workstation_core.px4_calibration_groups import px4_calibration_group

# Legacy fallback for test doubles/older subscriber adapters that do not yet
# publish the subscriber-owned all-RX liveness snapshot. Production decisions
# use the 10-second expected-system packet-silence state instead.
_MAVLINK_STALE_TIMEOUT_S = 5.0

DEVICE_RUNTIME_CONTROLLER_METHODS = [
    "toggle_mavlink_connection",
    "_stop_pixhawk_operation_workers",
    "reset_px4_calibration_group",
    "discover_pc_camera_ip",
    "_apply_camera_network_discovery",
    "_camera_network_discovery_failed",
    "_clear_camera_network_discovery",
    "discover_pc_pixhawk_ip",
    "_apply_pixhawk_host_ip_discovery",
    "_pixhawk_host_ip_discovery_failed",
    "_clear_pixhawk_host_ip_discovery",
    "_append_pixhawk_host_ip_candidates",
    "apply_pc_camera_ip",
    "_apply_camera_network_address_result",
    "_camera_network_apply_failed",
    "_clear_camera_network_apply",
    "_update_mavlink_optional_timeline",
    "_mavlink_connected_sides",
    "_mavlink_open_sides",
    "_refresh_mavlink_side_actions",
    "_mavlink_target_sides",
    "_sync_mavlink_subscriber_settings",
    "_ensure_mavlink_subscriber",
    "_handle_mavlink_status_changed",
    "_configured_mavlink_sides",
    "_mavlink_setup_target_sides",
    "_apply_mavlink_pixhawk_setup",
    "_start_pixhawk_stream_setup_worker",
    "_handle_mavlink_stream_setup_required",
    "_clear_pixhawk_stream_setup_worker",
    "bootstrap_ethernet_mavlink_from_usb",
    "recover_stalled_pixhawk_via_verified_usb",
    "_apply_pixhawk_usb_recovery_result",
    "_pixhawk_usb_recovery_failed",
    "_clear_pixhawk_usb_recovery_worker",
    "_apply_pixhawk_usb_bootstrap_result",
    "_clear_pixhawk_usb_bootstrap_worker",
    "_temporary_mavlink_heartbeat",
    "_ethernet_mavlink_bootstrap_command",
    "connect_mavlink_pixhawk",
    "_apply_pixhawk_connection_result",
    "_pixhawk_connection_failed",
    "_clear_pixhawk_connection_worker",
    "disconnect_mavlink_pixhawk",
    "apply_selected_pixhawk_candidate",
    "apply_persistent_pixhawk_eth",
    "_apply_pixhawk_persistent_config_result",
    "_clear_pixhawk_persistent_config_worker",
    "reboot_pixhawk_from_mavlink",
    "_persistent_pixhawk_eth_command",
    "_persistent_pixhawk_extras_text",
    "_persistent_pixhawk_net_cfg_text",
    "_persistent_pixhawk_eth_setup",
    "_start_pixhawk_shell_worker",
    "_apply_pixhawk_shell_worker_result",
    "_pixhawk_shell_worker_failed",
    "_clear_pixhawk_shell_worker",
    "reset_px4_level_calibration",
    "_start_pixhawk_parameter_worker",
    "_apply_pixhawk_parameter_worker_result",
    "_pixhawk_parameter_worker_failed",
    "_clear_pixhawk_parameter_worker"
]


class DeviceRuntimeController:
    def __init__(self, panel):
        self.panel = panel

    def toggle_mavlink_connection(self, camera: str | None = None) -> bool | None:
        if self.panel._session_recording_active():
            self.panel._log("[픽스호크] 세션 기록 중에는 Pixhawk MAVLink 연결을 변경할 수 없습니다.")
            return False
        if camera is None:
            if self.panel._mavlink_open_sides():
                self.panel.disconnect_mavlink_pixhawk()
            else:
                self.panel.connect_mavlink_pixhawk()
            return None
        if camera in self.panel._mavlink_open_sides():
            self.panel.disconnect_mavlink_pixhawk(camera)
        else:
            self.panel.connect_mavlink_pixhawk(camera)
        return None

    def reset_px4_level_calibration(self) -> bool:
        """Reset only PX4 Level Horizon fine-tuning offsets on both Pixhawks.

        The board mounting rotation and the individual accelerometer/gyro
        calibration slots are deliberately left untouched.  The PX4 shell's
        ``param reset`` command restores each named parameter to the firmware
        default and persists the change.
        """

        if self.panel._session_recording_active():
            self.panel._log("[픽스호크] 세션 기록 중에는 수평 보정 파라미터를 초기화할 수 없습니다.")
            return False
        target_sides = tuple(self.panel._mavlink_open_sides())
        if not target_sides:
            self.panel._log("[픽스호크] 수평 보정 초기화 차단: MAVLink 연결이 없습니다.")
            return False
        armed_sides = tuple(
            side
            for side in target_sides
            if bool(getattr(self.panel.px4_mag_managers.get(side), "armed", False))
        )
        if armed_sides:
            labels = ", ".join(self.panel._px4_mag_camera_label(side) for side in armed_sides)
            self.panel._log(
                f"[픽스호크] 수평 보정 초기화 차단: {labels}가 armed 상태입니다. Disarm 후 다시 시도하세요."
            )
            return False

        parameter_text = ", ".join(LEVEL_HORIZON_RESET_PARAM_NAMES)
        labels = ", ".join(self.panel._px4_mag_camera_label(side) for side in target_sides)
        result = QtWidgets.QMessageBox.question(
            self.panel,
            "Reset Level Horizon Calibration",
            (
                f"{labels}의 PX4 수평(Level Horizon) 미세 보정값을 펌웨어 기본값으로 초기화합니다.\n\n"
                f"대상: {parameter_text}\n"
                "보드 장착 방향(SENS_BOARD_ROT), 가속도계/자이로(CAL_ACC*/CAL_GYRO*)와 "
                "다른 PX4 파라미터는 변경하지 않습니다.\n\n"
                "기체를 반드시 Disarm한 상태에서 진행하세요. 계속할까요?"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if result != QtWidgets.QMessageBox.Yes:
            self.panel._log("[픽스호크] 수평 보정 파라미터 초기화를 취소했습니다.")
            return False

        command = "param reset " + " ".join(LEVEL_HORIZON_RESET_PARAM_NAMES)
        started = False
        for side in target_sides:
            started = (
                self.panel._start_pixhawk_shell_worker(
                    side,
                    command,
                    operation="reset_level_calibration",
                    timeout_s=3.0,
                    on_complete=lambda _result, camera=side: self.panel._start_pixhawk_parameter_worker(
                        camera, "attitude_refresh"
                    ),
                )
                or started
            )
        if not started:
            self.panel._log("[픽스호크] 수평 보정 파라미터 초기화 명령을 시작하지 못했습니다.")
            return False
        self.panel._log(
            f"[픽스호크] {labels} 수평 보정 초기화를 요청했습니다: {parameter_text}. "
            "PX4 PARAM readback을 갱신합니다."
        )
        return True

    def _stop_pixhawk_operation_workers(self, *, wait: bool) -> bool:
        all_stopped = True
        for attr in (
            "pixhawk_connection_thread",
            "pixhawk_disconnection_thread",
            "pixhawk_stream_setup_thread",
            "pixhawk_usb_bootstrap_thread",
            "pixhawk_usb_recovery_thread",
            "pixhawk_persistent_config_thread",
            "pixhawk_rtcm_thread",
        ):
            thread = getattr(self.panel, attr, None)
            if thread is None:
                continue
            worker_attr = attr.replace("_thread", "_worker")
            worker = getattr(self.panel, worker_attr, None)
            if attr == "pixhawk_rtcm_thread" and worker is not None:
                try:
                    worker.clear_pending()
                except Exception:
                    pass
            thread.quit()
            if thread.isRunning():
                if not wait or not thread.wait(1000):
                    all_stopped = False
                    continue
            if not thread.isRunning() and getattr(self.panel, attr, None) is thread:
                setattr(self.panel, attr, None)
                if getattr(self.panel, worker_attr, None) is worker:
                    setattr(self.panel, worker_attr, None)
        for workers in (self.panel.pixhawk_shell_threads, self.panel.pixhawk_parameter_threads):
            for camera, thread in list(workers.items()):
                try:
                    thread.quit()
                    if thread.isRunning():
                        if not wait or not thread.wait(1000):
                            all_stopped = False
                            continue
                except RuntimeError:
                    pass
                if not thread.isRunning():
                    workers.pop(camera, None)
                    if workers is self.panel.pixhawk_shell_threads:
                        self.panel.pixhawk_shell_workers.pop(camera, None)
                    else:
                        self.panel.pixhawk_parameter_workers.pop(camera, None)
        return all_stopped

    def discover_pc_camera_ip(self) -> None:
        if self.panel.camera_network_discovery_thread is not None:
            self.panel._log("[연결] PC IP 탐색이 이미 진행 중입니다.")
            return
        self.panel._log("[연결] PC IP 탐색: 카메라망 인터페이스와 PC IP를 함께 탐색합니다.")
        self.panel.workstation_camera_discovery_status = "SEARCHING"
        self.panel.detect_pc_ip_button.setEnabled(False)
        self.panel.camera_connection_label.setText("camera host IP: discovering...")
        self.panel.camera_network_discovery_thread = QtCore.QThread(self.panel)
        self.panel.camera_network_discovery_worker = CameraNetworkDiscoveryWorker(
            host_address=self.panel.host_address_edit.text().strip() or DEFAULT_HOST_ADDRESS,
            host_hints=self.panel._camera_host_hints(),
        )
        self.panel.camera_network_discovery_worker.moveToThread(self.panel.camera_network_discovery_thread)
        self.panel.camera_network_discovery_thread.started.connect(self.panel.camera_network_discovery_worker.run)
        self.panel.camera_network_discovery_worker.discovered.connect(self.panel._apply_camera_network_discovery)
        self.panel.camera_network_discovery_worker.failed.connect(self.panel._camera_network_discovery_failed)
        self.panel.camera_network_discovery_worker.finished.connect(self.panel.camera_network_discovery_thread.quit)
        self.panel.camera_network_discovery_worker.finished.connect(self.panel.camera_network_discovery_worker.deleteLater)
        self.panel.camera_network_discovery_thread.finished.connect(self.panel.camera_network_discovery_thread.deleteLater)
        self.panel.camera_network_discovery_thread.finished.connect(self.panel._clear_camera_network_discovery)
        self.panel.camera_network_discovery_thread.start()

    def _apply_camera_network_discovery(self, result: object) -> None:
        data = dict(result or {})
        candidates = list(data.get("candidates") or [])
        if not candidates:
            self.panel.workstation_camera_discovery_status = "FOUND 0 HOST INTERFACES"
            self.panel._log("[연결] PC IP 탐색: 사용할 수 있는 유선 인터페이스를 찾지 못했습니다.")
            self.panel.camera_connection_label.setText("camera host IP: interface not found")
            self.panel._refresh_operation_header()
            return
        self.panel._apply_host_interface_candidates(candidates)
        self.panel.workstation_camera_discovery_status = f"FOUND {len(candidates)} HOST INTERFACES"
        interface_name = self.panel.interface_edit.text().strip()
        resolved_address = data.get("resolved_address")
        if resolved_address is None:
            connection_name = data.get("connection_name")
            addresses = tuple(data.get("addresses") or ())
            if connection_name is None:
                self.panel._log(f"[연결] PC IP 탐색: {interface_name}에 활성 NetworkManager 연결이 없습니다.")
            else:
                if addresses:
                    address_text = ", ".join(str(address) for address in addresses)
                    self.panel._log(f"[연결] PC IP 탐색: 활성 프로필 {connection_name} 주소={address_text}")
                else:
                    self.panel._log(f"[연결] PC IP 탐색: 활성 프로필 {connection_name}에서 주소를 찾지 못했습니다.")
            self.panel.camera_connection_label.setText("camera host IP: not found")
            self.panel._refresh_operation_header()
            return
        resolved_text = str(resolved_address)
        self.panel.host_address_edit.setText(resolved_text)
        self.panel._log(
            f"[연결] PC IP 탐색: NetworkManager 활성 프로필에서 카메라망 IP를 찾았습니다: "
            f"{interface_name} {resolved_text}"
        )
        self.panel.camera_connection_label.setText(f"camera host IP: found {resolved_text}")
        self.panel._refresh_operation_header()

    def _camera_network_discovery_failed(self, error: str) -> None:
        self.panel.workstation_camera_discovery_status = f"ERROR · {error}"
        self.panel._log(f"[연결] PC IP 탐색 실패: {error}")
        self.panel.camera_connection_label.setText(f"camera host IP: failed ({error})")
        self.panel._refresh_operation_header()

    def _clear_camera_network_discovery(self) -> None:
        self.panel.camera_network_discovery_thread = None
        self.panel.camera_network_discovery_worker = None
        self.panel.detect_pc_ip_button.setEnabled(True)

    def discover_pc_pixhawk_ip(self) -> None:
        if self.panel.pixhawk_host_ip_discovery_thread is not None:
            self.panel._log("[픽스호크] PC IP 탐색이 이미 진행 중입니다.")
            return
        self.panel.detect_pixhawk_host_ip_button.setEnabled(False)
        self.panel.workstation_pixhawk_discovery_status = "SEARCHING HOST IP"
        self.panel.pixhawk_connection_label.setText("pixhawk host IP: discovering...")
        self.panel.pixhawk_host_ip_discovery_thread = QtCore.QThread(self.panel)
        self.panel.pixhawk_host_ip_discovery_worker = PixhawkHostIpDiscoveryWorker()
        self.panel.pixhawk_host_ip_discovery_worker.moveToThread(self.panel.pixhawk_host_ip_discovery_thread)
        self.panel.pixhawk_host_ip_discovery_thread.started.connect(self.panel.pixhawk_host_ip_discovery_worker.run)
        self.panel.pixhawk_host_ip_discovery_worker.discovered.connect(self.panel._apply_pixhawk_host_ip_discovery)
        self.panel.pixhawk_host_ip_discovery_worker.failed.connect(self.panel._pixhawk_host_ip_discovery_failed)
        self.panel.pixhawk_host_ip_discovery_worker.finished.connect(self.panel.pixhawk_host_ip_discovery_thread.quit)
        self.panel.pixhawk_host_ip_discovery_worker.finished.connect(self.panel.pixhawk_host_ip_discovery_worker.deleteLater)
        self.panel.pixhawk_host_ip_discovery_thread.finished.connect(self.panel.pixhawk_host_ip_discovery_thread.deleteLater)
        self.panel.pixhawk_host_ip_discovery_thread.finished.connect(self.panel._clear_pixhawk_host_ip_discovery)
        self.panel.pixhawk_host_ip_discovery_thread.start()

    def _apply_pixhawk_host_ip_discovery(self, interfaces: object) -> None:
        rows = list(interfaces or [])
        if not rows:
            self.panel.workstation_pixhawk_discovery_status = "FOUND 0 HOST INTERFACES"
            self.panel._log("[픽스호크] PC IP 탐색: 활성 IPv4 인터페이스를 찾지 못했습니다.")
            self.panel.pixhawk_connection_label.setText("pixhawk host IP: not found")
            self.panel._refresh_operation_header()
            return
        self.panel._append_pixhawk_host_ip_candidates(rows)
        self.panel.workstation_pixhawk_discovery_status = f"FOUND {len(rows)} HOST INTERFACES"
        self.panel.pixhawk_connection_label.setText("pixhawk host IP: 후보 표시됨 - 표에서 PC_IP를 선택해 적용하세요")
        self.panel._refresh_operation_header()

    def _pixhawk_host_ip_discovery_failed(self, error: str) -> None:
        self.panel.workstation_pixhawk_discovery_status = f"ERROR · {error}"
        self.panel._log(f"[픽스호크] PC IP 탐색 실패: {error}")
        self.panel.pixhawk_connection_label.setText(f"pixhawk host IP: failed ({error})")
        self.panel._refresh_operation_header()

    def _clear_pixhawk_host_ip_discovery(self) -> None:
        self.panel.pixhawk_host_ip_discovery_thread = None
        self.panel.pixhawk_host_ip_discovery_worker = None
        self.panel.detect_pixhawk_host_ip_button.setEnabled(True)

    def _append_pixhawk_host_ip_candidates(self, interfaces: Sequence[object]) -> None:
        hints: list[IPv4Address] = []
        for text in (
            self.panel.left_pixhawk_eth_ip_edit.text().strip(),
            self.panel.right_pixhawk_eth_ip_edit.text().strip(),
            *self.panel._active_mavlink_remote_hosts(),
        ):
            if not text:
                continue
            try:
                hints.append(IPv4Address(str(text).split("/", 1)[0]))
            except ValueError:
                continue
        rows: list[dict[str, object]] = []
        seen: set[tuple[str, str]] = set()
        for candidate in interfaces:
            ip_text = str(candidate.address.ip)
            interface_name = str(getattr(candidate, "name", ""))
            key = (interface_name, ip_text)
            if key in seen:
                continue
            seen.add(key)
            matching_hints = [str(hint) for hint in hints if hint in candidate.address.network]
            status_parts = [str(candidate.address.network)]
            if matching_hints:
                status_parts.append("Pixhawk subnet match: " + ", ".join(matching_hints))
            if interface_name.startswith(("wl", "wlan")):
                status_parts.append("Wi-Fi")
            else:
                status_parts.append("wired/other")
            rows.append(
                {
                    "kind": "PC_IP",
                    "value": ip_text,
                    "baud": "",
                    "status": " | ".join(status_parts),
                    "interface": interface_name,
                }
            )
        if rows:
            self.panel._append_pixhawk_candidate_rows(rows, source="pc")
            for row in rows:
                self.panel._log(
                    f"[픽스호크] PC IP 후보: {row['interface']} {row['value']} "
                    f"({row['status']})"
                )

    def apply_pc_camera_ip(self) -> None:
        if self.panel.camera_network_apply_thread is not None:
            self.panel._log("[연결] PC IP 적용이 이미 진행 중입니다.")
            return
        interface_name = self.panel.interface_edit.text().strip()
        address_text = self.panel.host_address_edit.text().strip()
        if not interface_name or not address_text:
            self.panel._log("[연결] PC IP 적용 실패: 인터페이스와 주소를 먼저 지정하세요.")
            self.panel.camera_connection_label.setText("camera host IP: missing")
            return
        self.panel._log(f"[연결] PC IP 적용 시도: {interface_name} {address_text}")
        self.panel.apply_pc_ip_button.setEnabled(False)
        self.panel.camera_connection_label.setText("camera host IP: applying...")
        self.panel.camera_network_apply_thread = QtCore.QThread(self.panel)
        self.panel.camera_network_apply_worker = CameraNetworkAddressApplyWorker(
            interface_name=interface_name,
            address_text=address_text,
        )
        self.panel.camera_network_apply_worker.moveToThread(self.panel.camera_network_apply_thread)
        self.panel.camera_network_apply_thread.started.connect(self.panel.camera_network_apply_worker.run)
        self.panel.camera_network_apply_worker.applied.connect(self.panel._apply_camera_network_address_result)
        self.panel.camera_network_apply_worker.failed.connect(self.panel._camera_network_apply_failed)
        self.panel.camera_network_apply_worker.finished.connect(self.panel.camera_network_apply_thread.quit)
        self.panel.camera_network_apply_worker.finished.connect(self.panel.camera_network_apply_worker.deleteLater)
        self.panel.camera_network_apply_thread.finished.connect(self.panel.camera_network_apply_thread.deleteLater)
        self.panel.camera_network_apply_thread.finished.connect(self.panel._clear_camera_network_apply)
        self.panel.camera_network_apply_thread.start()

    def _apply_camera_network_address_result(self, result: object) -> None:
        data = dict(result or {})
        interface_name = str(data.get("interface_name") or self.panel.interface_edit.text().strip())
        address_text = str(data.get("address_text") or self.panel.host_address_edit.text().strip())
        if not bool(data.get("ok")):
            message = str(data.get("message") or "unknown error")
            self.panel._log(f"[연결] PC IP 적용 실패: {message}")
            self.panel.camera_connection_label.setText("camera host IP: apply failed")
            self.panel._refresh_operation_header()
            return
        self.panel._log(f"[연결] PC IP 적용 완료: {data.get('message', '')}")
        addresses = str(data.get("addresses") or "")
        if addresses:
            self.panel._log(f"[연결] 네트워크 인터페이스 상태:\n{addresses}")
        self.panel.interface_edit.setText(interface_name)
        self.panel.host_address_edit.setText(address_text)
        self.panel._save_current_settings()
        self.panel.camera_connection_label.setText(f"camera host IP: applied {address_text}")
        self.panel._refresh_operation_header()

    def _camera_network_apply_failed(self, error: str) -> None:
        self.panel._log(f"[연결] PC IP 적용 worker 실패: {error}")
        self.panel.camera_connection_label.setText(f"camera host IP: failed ({error})")
        self.panel._refresh_operation_header()

    def _clear_camera_network_apply(self) -> None:
        self.panel.camera_network_apply_thread = None
        self.panel.camera_network_apply_worker = None
        self.panel.apply_pc_ip_button.setEnabled(True)

    def _update_mavlink_optional_timeline(self, camera: str, payload: dict[str, object]) -> None:
        sample_type = str(payload.get("sample_type") or "mavlink_status")
        values = {key: value for key, value in payload.items() if key != "sample_type"}
        self.panel._record_external_timeline_row(camera, sample_type, values)

    def _mavlink_connected_sides(self) -> set[str]:
        if self.panel.mavlink_pixhawk is None:
            return set()
        subscriber = self.panel.mavlink_pixhawk
        connections = getattr(subscriber, "connections", {})
        health_reader = getattr(subscriber, "mavlink_health_snapshot", None)
        if callable(health_reader):
            connected: set[str] = set()
            for side in ("left", "right"):
                if side not in connections:
                    continue
                try:
                    health = health_reader(side)
                except Exception:
                    continue
                if bool(getattr(health, "identity_verified", False)) and bool(
                    getattr(health, "command_eligible", False)
                ):
                    connected.add(side)
            return connected
        heartbeat_sides = getattr(subscriber, "_heartbeat_logged", set())
        try:
            return {side for side in ("left", "right") if side in connections and side in heartbeat_sides}
        except TypeError:
            return set()

    def _mavlink_open_sides(self) -> set[str]:
        if self.panel.mavlink_pixhawk is None:
            return set()
        connections = getattr(self.panel.mavlink_pixhawk, "connections", {})
        try:
            return {side for side in ("left", "right") if side in connections}
        except TypeError:
            return set()

    def _refresh_mavlink_side_actions(self) -> None:
        connected_sides = self.panel._mavlink_connected_sides()
        self.panel.left_px4_shell_button.setEnabled("left" in connected_sides)
        self.panel.right_px4_shell_button.setEnabled("right" in connected_sides)
        self.panel.left_persist_eth_button.setEnabled("left" in connected_sides)
        self.panel.right_persist_eth_button.setEnabled("right" in connected_sides)
        self.panel.left_reboot_pixhawk_button.setEnabled("left" in connected_sides)
        self.panel.right_reboot_pixhawk_button.setEnabled("right" in connected_sides)

    def _mavlink_target_sides(self, camera: str | None) -> tuple[str, ...]:
        if camera is None:
            return ("left", "right")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        return (camera,)

    def _sync_mavlink_subscriber_settings(self) -> None:
        if self.panel.mavlink_pixhawk is None:
            return
        if hasattr(self.panel.mavlink_pixhawk, "urls"):
            self.panel.mavlink_pixhawk.urls["left"] = self.panel.left_mavlink_url_edit.text().strip()
            self.panel.mavlink_pixhawk.urls["right"] = self.panel.right_mavlink_url_edit.text().strip()
        if hasattr(self.panel.mavlink_pixhawk, "baudrates"):
            self.panel.mavlink_pixhawk.baudrates["left"] = self.panel.left_mavlink_baud_spin.value()
            self.panel.mavlink_pixhawk.baudrates["right"] = self.panel.right_mavlink_baud_spin.value()

    def _ensure_mavlink_subscriber(self) -> DualPixhawkMavlinkSubscriber:
        if self.panel.mavlink_pixhawk is None:
            self.panel.mavlink_pixhawk = DualPixhawkMavlinkSubscriber(
                left_url=self.panel.left_mavlink_url_edit.text().strip(),
                right_url=self.panel.right_mavlink_url_edit.text().strip(),
                left_baud=self.panel.left_mavlink_baud_spin.value(),
                right_baud=self.panel.right_mavlink_baud_spin.value(),
                custom_watches=tuple(self.panel.custom_watch_specs),
            )
            self.panel.mavlink_pixhawk.extrinsic_changed.connect(self.panel._update_extrinsic)
            self.panel.mavlink_pixhawk.gps_changed.connect(self.panel._update_gps)
            self.panel.mavlink_pixhawk.mag_sensor_changed.connect(self.panel._update_mag_sensor)
            self.panel.mavlink_pixhawk.gps_sensor_changed.connect(self.panel._update_gps_sensor)
            self.panel.mavlink_pixhawk.gnss_relative_changed.connect(self.panel._update_gnss_relative)
            self.panel.mavlink_pixhawk.custom_watch_changed.connect(self.panel._update_custom_watch)
            optional_timeline_changed = getattr(self.panel.mavlink_pixhawk, "optional_timeline_changed", None)
            if optional_timeline_changed is not None:
                optional_timeline_changed.connect(self.panel._update_mavlink_optional_timeline)
            stream_setup_required = getattr(self.panel.mavlink_pixhawk, "stream_setup_required", None)
            if stream_setup_required is not None:
                stream_setup_required.connect(self.panel._handle_mavlink_stream_setup_required)
            self.panel.mavlink_pixhawk.shell_output.connect(self.panel._append_px4_shell_output)
            self.panel.mavlink_pixhawk.status_changed.connect(self.panel._handle_mavlink_status_changed)
        else:
            self.panel._sync_mavlink_subscriber_settings()
        return self.panel.mavlink_pixhawk

    def _handle_mavlink_status_changed(self, text: str) -> None:
        self.panel._log(f"[픽스호크] {text}")
        self.panel._refresh_session_toggle_buttons()
        self.panel._refresh_mavlink_side_actions()
        self._auto_refresh_px4_calibration_params_on_heartbeat()

    def _auto_refresh_px4_calibration_params_on_heartbeat(self) -> None:
        # Fires once per side, right when that side's first heartbeat is
        # logged (status_changed emits that message exactly once -- see
        # DualPixhawkMavlinkSubscriber._handle_mavlink_message). Without
        # this, the Pixhawk Parameters group sits at "readback 대기 중"
        # until the operator manually presses "QGC 결과 새로고침".
        auto_refreshed = self.panel._px4_calibration_auto_refreshed_sides
        for side in self.panel._mavlink_connected_sides():
            if side in auto_refreshed:
                continue
            auto_refreshed.add(side)
            self.panel._start_pixhawk_parameter_worker(side, "calibration_refresh")

    def _configured_mavlink_sides(self) -> tuple[str, ...]:
        sides = []
        if self.panel.left_mavlink_url_edit.text().strip():
            sides.append("left")
        if self.panel.right_mavlink_url_edit.text().strip():
            sides.append("right")
        return tuple(sides)

    def _mavlink_setup_target_sides(self, target_sides: tuple[str, ...]) -> set[str]:
        configured = set(self.panel._configured_mavlink_sides())
        connected = self.panel._mavlink_connected_sides()
        return {side for side in target_sides if side in configured or side in connected}

    def _apply_mavlink_pixhawk_setup(self, subscriber: DualPixhawkMavlinkSubscriber, target_sides: tuple[str, ...]) -> None:
        setup_sides = self.panel._mavlink_setup_target_sides(target_sides)
        if not setup_sides or not self.panel.mavlink_configure_streams_checkbox.isChecked():
            return
        self.panel._start_pixhawk_stream_setup_worker(subscriber, tuple(sorted(setup_sides)))

    def _start_pixhawk_stream_setup_worker(
        self,
        subscriber: DualPixhawkMavlinkSubscriber | None = None,
        cameras: Sequence[str] = (),
    ) -> bool:
        subscriber = subscriber or self.panel.mavlink_pixhawk
        if subscriber is None:
            return False
        if self.panel.pixhawk_stream_setup_thread is not None:
            self.panel._log("[픽스호크] MAVLink stream 설정이 이미 진행 중입니다.")
            return False
        target_sides = tuple(str(side) for side in cameras if str(side) in getattr(subscriber, "connections", {}))
        if not target_sides:
            return False
        thread = QtCore.QThread(self.panel)
        worker = PixhawkStreamSetupWorker(
            subscriber,
            host_ip=self.panel.pixhawk_host_address_edit.text().strip(),
            ports={
                "left": self.panel._mavlink_stream_port(self.panel.left_mavlink_url_edit.text().strip(), default=14550),
                "right": self.panel._mavlink_stream_port(self.panel.right_mavlink_url_edit.text().strip(), default=14551),
            },
            sysids={"left": 1, "right": 2},
            rate=self.panel.mavlink_stream_rate_spin.value(),
            cameras=target_sides,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda result: self.panel._log(
                f"[픽스호크] MAVLink stream 설정 완료: {', '.join(target_sides)} "
                f"host={result.get('host_ip', '')}"
            )
        )
        worker.failed.connect(lambda error: self.panel._log(f"[픽스호크] MAVLink stream 설정 실패: {error}"))
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_pixhawk_stream_setup_worker)
        self.panel.pixhawk_stream_setup_thread = thread
        self.panel.pixhawk_stream_setup_worker = worker
        thread.start()
        return True

    def _handle_mavlink_stream_setup_required(self, camera: str) -> None:
        if not self.panel.mavlink_configure_streams_checkbox.isChecked():
            return
        self.panel._start_pixhawk_stream_setup_worker(self.panel.mavlink_pixhawk, (str(camera),))

    def _clear_pixhawk_stream_setup_worker(self) -> None:
        self.panel.pixhawk_stream_setup_thread = None
        self.panel.pixhawk_stream_setup_worker = None

    def _set_pixhawk_usb_bootstrap_status(self, camera: str, state: str, detail: str) -> None:
        statuses = getattr(self.panel, "pixhawk_usb_bootstrap_status", None)
        if not isinstance(statuses, dict):
            statuses = {}
            self.panel.pixhawk_usb_bootstrap_status = statuses
        statuses[str(camera)] = {
            "state": str(state).upper(),
            "detail": str(detail),
            "updated_s": time.monotonic(),
        }

    def bootstrap_ethernet_mavlink_from_usb(self, camera: str) -> None:
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        label = self.panel._px4_mag_camera_label(camera)
        self._set_pixhawk_usb_bootstrap_status(
            camera,
            "RUNNING",
            "USB shell로 단일 Ethernet MAVLink 설정 저장·재부팅 중",
        )
        if camera == "left":
            usb_url = self.panel.left_pixhawk_bootstrap_url_edit.text().strip()
            usb_baud = self.panel.left_pixhawk_bootstrap_baud_spin.value()
        else:
            usb_url = self.panel.right_pixhawk_bootstrap_url_edit.text().strip()
            usb_baud = self.panel.right_pixhawk_bootstrap_baud_spin.value()
        if not usb_url:
            self._set_pixhawk_usb_bootstrap_status(camera, "ERROR", "USB 부트스트랩 포트가 비어 있습니다.")
            self.panel._log(f"[픽스호크] {label} ETH MAVLink USB 부트스트랩 실패: USB 부트스트랩 포트가 비어 있습니다.")
            return
        try:
            command = self.panel._ethernet_mavlink_bootstrap_command(camera, mode="normal")
        except Exception as exc:
            self._set_pixhawk_usb_bootstrap_status(camera, "ERROR", f"시작 명령 생성 실패: {exc}")
            self.panel._log(f"[픽스호크] {label} ETH MAVLink USB 부트스트랩 명령 생성 실패: {exc}")
            return
        if self.panel.pixhawk_usb_bootstrap_thread is not None:
            self._set_pixhawk_usb_bootstrap_status(camera, "RUNNING", "다른 USB ETH MAV 시작이 이미 진행 중입니다.")
            self.panel._log(f"[픽스호크] {label} USB 부트스트랩이 이미 진행 중입니다.")
            return
        try:
            fallback_command = self.panel._ethernet_mavlink_bootstrap_command(camera, mode="onboard")
        except Exception as exc:
            self._set_pixhawk_usb_bootstrap_status(camera, "ERROR", f"fallback 명령 생성 실패: {exc}")
            self.panel._log(f"[픽스호크] {label} ETH MAVLink USB fallback 명령 생성 실패: {exc}")
            return
        thread = QtCore.QThread(self.panel)
        worker = PixhawkUsbBootstrapWorker(
            camera=camera,
            usb_url=usb_url,
            baud_candidates=self.panel._usb_bootstrap_baud_candidates(usb_baud),
            command=command,
            fallback_command=fallback_command,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._apply_pixhawk_usb_bootstrap_result)
        worker.failed.connect(
            lambda error, camera=camera, label=label: self._pixhawk_usb_bootstrap_failed(camera, label, error)
        )
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_pixhawk_usb_bootstrap_worker)
        self.panel.pixhawk_usb_bootstrap_thread = thread
        self.panel.pixhawk_usb_bootstrap_worker = worker
        self.panel._log(f"[픽스호크] {label} USB Ethernet MAVLink 부트스트랩을 시작했습니다.")
        thread.start()

    def _pixhawk_usb_bootstrap_failed(self, camera: str, label: str, error: str) -> None:
        self._set_pixhawk_usb_bootstrap_status(camera, "ERROR", str(error))
        self.panel._log(f"[픽스호크] {label} ETH MAVLink USB 부트스트랩 실패: {error}")

    def _apply_pixhawk_usb_bootstrap_result(self, result: object) -> None:
        data = dict(result or {})
        camera = str(data.get("camera") or "")
        if camera not in {"left", "right"}:
            return
        selected_baud = int(data.get("baud") or 0)
        self._set_pixhawk_usb_bootstrap_status(
            camera,
            "RECOVERED",
            "단일 Ethernet MAVLink 설정·재부팅 완료; UDP telemetry 확인 중",
        )
        self.panel._set_usb_bootstrap_baud(camera, selected_baud)
        output = str(data.get("output") or "").strip()
        if output:
            self.panel._log(
                f"[픽스호크 USB] {self.panel._px4_mag_camera_label(camera)} PX4 shell 응답 "
                f"({data.get('flag_label', '')}): {output[-800:]}"
            )
        self.panel._log(
            f"[픽스호크] {self.panel._px4_mag_camera_label(camera)} USB {selected_baud} baud "
            "단일-instance 설정·재부팅 명령 완료. UDP MAVLink 수신을 다시 시작합니다."
        )
        self.panel._save_current_settings()
        self.panel.connect_mavlink_pixhawk(camera)

    def _clear_pixhawk_usb_bootstrap_worker(self) -> None:
        self.panel.pixhawk_usb_bootstrap_thread = None
        self.panel.pixhawk_usb_bootstrap_worker = None

    def _set_pixhawk_usb_recovery_status(self, camera: str, state: str, detail: str) -> None:
        statuses = getattr(self.panel, "pixhawk_usb_recovery_status", None)
        if not isinstance(statuses, dict):
            statuses = {}
            self.panel.pixhawk_usb_recovery_status = statuses
        statuses[str(camera)] = {
            "state": str(state).upper(),
            "detail": str(detail),
            "updated_s": time.monotonic(),
        }

    def _pixhawk_usb_recovery_block_reason(self, camera: str) -> str:
        if camera not in {"left", "right"}:
            return f"unknown Pixhawk side: {camera}"
        recording_active = getattr(self.panel, "_session_recording_active", None)
        if callable(recording_active) and bool(recording_active()):
            return "measurement recording is active"
        run_state = str(getattr(self.panel, "_measurement_run_state", "IDLE") or "IDLE").upper()
        if run_state in {"QUEUED", "STARTING", "RUNNING", "STOPPING"}:
            return f"Measurement lifecycle is {run_state}"
        if getattr(self.panel, "_session_save_thread", None) is not None:
            return "session save/finalization worker is active"
        if bool(getattr(self.panel, "_measurement_package_finalize_requested", False)):
            return "measurement package finalization is pending"
        if bool(getattr(self.panel, "_session_save_retry_required", False)):
            return "session save recovery is pending"
        for attribute, label in (
            ("pixhawk_connection_thread", "Pixhawk connect"),
            ("pixhawk_disconnection_thread", "Pixhawk disconnect"),
            ("pixhawk_stream_setup_thread", "PX4 stream setup"),
            ("pixhawk_usb_bootstrap_thread", "USB Ethernet bootstrap"),
            ("pixhawk_persistent_config_thread", "PX4 persistent config"),
            ("pixhawk_usb_discovery_thread", "USB discovery"),
            ("pixhawk_usb_recovery_thread", "USB recovery"),
        ):
            if getattr(self.panel, attribute, None) is not None:
                return f"{label} is active"
        for attribute, label in (
            ("pixhawk_shell_threads", "PX4 shell/reboot command"),
            ("pixhawk_parameter_threads", "PX4 parameter command"),
        ):
            active = getattr(self.panel, attribute, {}) or {}
            if active:
                return f"{label} is active"
        subscriber = getattr(self.panel, "mavlink_pixhawk", None)
        health_reader = getattr(subscriber, "mavlink_health_snapshot", None)
        if not callable(health_reader):
            return "current Ethernet MAVLink liveness evidence is unavailable"
        try:
            health = health_reader(camera)
        except Exception as exc:
            return f"Ethernet MAVLink liveness read failed: {exc}"
        state = str(
            getattr(
                getattr(health, "state", None),
                "value",
                getattr(health, "state", ""),
            )
        )
        if state != "ETH MAVLINK RX STALLED":
            return f"verified-USB recovery is allowed only for ETH MAVLINK RX STALLED (current={state or 'UNKNOWN'})"
        return ""

    def _verified_usb_identity_for_side(
        self,
        camera: str,
        *,
        now_s: float,
    ) -> tuple[dict[str, object] | None, str]:
        expected_system_id = 1 if camera == "left" else 2
        evidence = getattr(self.panel, "pixhawk_usb_identity_evidence", ()) or ()
        matching: list[dict[str, object]] = []
        stale_matching = False
        for item in evidence:
            if not isinstance(item, dict):
                continue
            try:
                system_id = int(item.get("system_id"))
            except (TypeError, ValueError):
                continue
            if system_id != expected_system_id:
                continue
            observed_s = item.get("observed_monotonic_s")
            try:
                age_s = max(0.0, now_s - float(observed_s))
            except (TypeError, ValueError):
                stale_matching = True
                continue
            if age_s > USB_MAVLINK_IDENTITY_MAX_AGE_S:
                stale_matching = True
                continue
            try:
                baud = int(item.get("baud") or 0)
            except (TypeError, ValueError):
                continue
            if not str(item.get("device") or "") or baud <= 0:
                continue
            if item.get("component_id") is None:
                continue
            candidate = dict(item)
            candidate["identity_age_s"] = age_s
            matching.append(candidate)
        if len(matching) > 1:
            return None, f"sysid {expected_system_id} is present on multiple current USB devices"
        if not matching:
            if stale_matching:
                return None, f"sysid {expected_system_id} USB identity evidence is stale"
            return None, f"no current unique USB heartbeat for sysid {expected_system_id}"
        return matching[0], ""

    def recover_stalled_pixhawk_via_verified_usb(self, camera: str) -> bool:
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        label = self.panel._px4_mag_camera_label(camera)
        blocked = self._pixhawk_usb_recovery_block_reason(camera)
        if blocked:
            detail = f"복구 차단: {blocked}"
            self._set_pixhawk_usb_recovery_status(camera, "ERROR", detail)
            self.panel._log(f"[픽스호크] {label} verified-USB 복구 차단: {blocked}")
            return False
        evidence, error = self._verified_usb_identity_for_side(camera, now_s=time.monotonic())
        if evidence is None:
            detail = (
                f"{error}. Find USB로 identity를 다시 확인하거나 {label}만 수동 전원 재부팅하세요."
            )
            self._set_pixhawk_usb_recovery_status(camera, "ERROR", detail)
            self.panel._log(f"[픽스호크] {label} verified-USB 복구 불가: {detail}")
            return False
        expected_system_id = 1 if camera == "left" else 2
        device = str(evidence.get("device") or "")
        component_id = int(evidence.get("component_id") or 0)
        result = QtWidgets.QMessageBox.question(
            self.panel,
            f"{label} verified-USB 재부팅",
            (
                f"{label} ETH MAVLink 전체 수신 정지를 복구합니다.\n\n"
                f"USB: {device}\n"
                f"MAVLink identity: sysid={expected_system_id}, compid={component_id}\n"
                "이 USB identity를 worker가 즉시 다시 검증한 뒤 해당 Pixhawk에만 재부팅 명령을 보냅니다.\n"
                "명령 전송은 복구 성공이 아니며 Ethernet heartbeat+ATTITUDE 10분 검증을 다시 시작합니다.\n\n"
                "계속할까요?"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if result != QtWidgets.QMessageBox.Yes:
            self._set_pixhawk_usb_recovery_status(camera, "CANCELLED", "operator가 재부팅을 취소했습니다.")
            self.panel._log(f"[픽스호크] {label} verified-USB 재부팅을 취소했습니다.")
            return False
        thread = QtCore.QThread(self.panel)
        worker = PixhawkUsbRecoveryWorker(
            camera=camera,
            evidence=evidence,
            expected_system_id=expected_system_id,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._apply_pixhawk_usb_recovery_result)
        worker.failed.connect(
            lambda failure, camera=camera: self.panel._pixhawk_usb_recovery_failed(camera, failure)
        )
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_pixhawk_usb_recovery_worker)
        self.panel.pixhawk_usb_recovery_thread = thread
        self.panel.pixhawk_usb_recovery_worker = worker
        self._set_pixhawk_usb_recovery_status(
            camera,
            "RUNNING",
            f"sysid={expected_system_id} USB identity 재검증 후 one-shot reboot 대기",
        )
        self.panel._log(
            f"[픽스호크] {label} verified-USB recovery 시작: {device}, "
            f"sysid={expected_system_id}, compid={component_id}"
        )
        thread.start()
        return True

    def _apply_pixhawk_usb_recovery_result(self, result: object) -> None:
        data = dict(result or {})
        camera = str(data.get("camera") or "")
        if camera not in {"left", "right"}:
            return
        label = self.panel._px4_mag_camera_label(camera)
        detail = (
            f"USB reboot command sent to sysid={data.get('system_id')} via {data.get('device')}; "
            "Ethernet expected-sysid packet 재개 및 10분 qualification 대기"
        )
        self._set_pixhawk_usb_recovery_status(camera, "COMMAND_SENT", detail)
        self.panel._log(
            f"[픽스호크] {label} {detail}. command send만으로 LIVE/READY로 처리하지 않습니다."
        )

    def _pixhawk_usb_recovery_failed(self, camera: str, error: str) -> None:
        label = self.panel._px4_mag_camera_label(camera)
        detail = f"USB identity 재검증/재부팅 실패: {error}; {label} 수동 전원 재부팅을 사용하세요."
        self._set_pixhawk_usb_recovery_status(camera, "ERROR", detail)
        self.panel._log(f"[픽스호크] {label} {detail}")

    def _clear_pixhawk_usb_recovery_worker(self) -> None:
        self.panel.pixhawk_usb_recovery_thread = None
        self.panel.pixhawk_usb_recovery_worker = None

    def _temporary_mavlink_heartbeat(self, subscriber: DualPixhawkMavlinkSubscriber, camera: str) -> bool:
        connection = getattr(subscriber, "connections", {}).get(camera)
        if connection is None:
            return False
        wait_heartbeat = getattr(connection, "wait_heartbeat", None)
        if wait_heartbeat is None:
            return True
        mav = getattr(connection, "mav", None)
        mavlink = getattr(connection, "mavlink", None)
        if mav is not None and hasattr(mav, "heartbeat_send"):
            try:
                mav.heartbeat_send(
                    getattr(mavlink, "MAV_TYPE_GENERIC", 0),
                    getattr(mavlink, "MAV_AUTOPILOT_INVALID", 8),
                    0,
                    0,
                    0,
                )
            except Exception:
                pass
        try:
            return wait_heartbeat(timeout=1.2) is not None
        except Exception:
            return False

    def _ethernet_mavlink_bootstrap_command(self, camera: str, *, mode: str = "normal") -> str:
        setup = self.panel._persistent_pixhawk_eth_setup(camera)
        if mode not in {"normal", "onboard"}:
            raise ValueError(f"Unsupported Ethernet MAVLink bootstrap mode: {mode}")
        extras = "/fs/microsd/etc/extras.txt"
        net_cfg = "/fs/microsd/net.cfg"
        extras_lines = list(setup["extras_lines"])
        net_cfg_lines = list(setup["net_cfg_lines"])
        # MAV_2_CONFIG is the sole owner of the Ethernet MAVLink instance.
        # A second manual start can bind the same UDP port on NuttX and emit a
        # duplicate unicast stream beside the parameter-owned broadcast stream.
        # Persist the interface files and reboot so rc.serial creates exactly
        # one instance; never stop or manually start one from recovery.
        return "\n".join(
            (
                "mkdir /fs/microsd/etc",
                *setup["param_lines"],
                "param save",
                f"echo '{extras_lines[0]}' > {extras}",
                *(f"echo '{line}' >> {extras}" for line in extras_lines[1:]),
                f"echo '{net_cfg_lines[0]}' > {net_cfg}",
                *(f"echo '{line}' >> {net_cfg}" for line in net_cfg_lines[1:]),
                "reboot",
            )
        )

    def connect_mavlink_pixhawk(self, camera: str | None = None, *, force_setup: bool = False) -> None:
        if self.panel._session_recording_active():
            self.panel._log("[픽스호크] 세션 기록 중에는 Pixhawk MAVLink 연결을 변경할 수 없습니다.")
            return
        if getattr(self.panel, "pixhawk_disconnection_thread", None) is not None:
            self.panel._log("[픽스호크] MAVLink 연결 해제가 끝난 뒤 다시 연결하세요.")
            return
        target_sides = self.panel._mavlink_target_sides(camera)
        if all(side in self.panel._mavlink_open_sides() for side in target_sides):
            stale_sides = tuple(side for side in target_sides if self._mavlink_side_is_stale(side))
            if not stale_sides:
                if force_setup and self.panel.mavlink_pixhawk is not None:
                    self.panel._apply_mavlink_pixhawk_setup(self.panel.mavlink_pixhawk, target_sides)
                return
            self._reconnect_stale_mavlink_sides(camera, stale_sides, force_setup)
            return
        if self.panel.pixhawk_connection_thread is not None:
            self.panel._log("[픽스호크] MAVLink 연결 작업이 이미 진행 중입니다.")
            return
        subscriber = self.panel._ensure_mavlink_subscriber()
        label = " / ".join(self.panel._px4_mag_camera_label(side) for side in target_sides)
        thread = QtCore.QThread(self.panel)
        worker = PixhawkConnectionWorker(subscriber, camera)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda _result, camera=camera, force_setup=force_setup, target_sides=target_sides:
            self.panel._apply_pixhawk_connection_result(camera, force_setup, target_sides)
        )
        worker.failed.connect(
            lambda error, camera=camera, target_sides=target_sides: self.panel._pixhawk_connection_failed(
                camera, target_sides, error
            )
        )
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_pixhawk_connection_worker)
        self.panel.pixhawk_connection_thread = thread
        self.panel.pixhawk_connection_worker = worker
        self.panel.pixhawk_connection_label.setText(f"픽스호크 MAVLink: {label} 연결 중...")
        self.panel._refresh_session_toggle_buttons()
        self.panel._log(f"[픽스호크] {label} MAVLink 연결을 시작했습니다.")
        thread.start()

    def _apply_pixhawk_connection_result(
        self,
        camera: str | None,
        force_setup: bool,
        target_sides: Sequence[str],
    ) -> None:
        subscriber = self.panel.mavlink_pixhawk
        if subscriber is None:
            return
        try:
            subscriber.start_receive_timer()
        except Exception as exc:
            self.panel._pixhawk_connection_failed(camera, tuple(target_sides), str(exc))
            return
        ensure_rtcm_worker = getattr(self.panel, "_ensure_pixhawk_rtcm_worker", None)
        if callable(ensure_rtcm_worker) and (
            getattr(self.panel, "rtcm_pipeline_worker", None) is not None
            or getattr(self.panel, "pixhawk_rtcm_worker", None) is not None
        ):
            ensure_rtcm_worker()
        if force_setup:
            self.panel._apply_mavlink_pixhawk_setup(subscriber, tuple(target_sides))
        elif self.panel.mavlink_configure_streams_checkbox.isChecked():
            self.panel._log(
                "[픽스호크] 일반 MAVLink 연결에서는 stream 설정을 자동 실행하지 않습니다. "
                "필요하면 Workstation의 MAVLink stream 설정을 적용하세요."
            )
        self.panel._refresh_session_toggle_buttons()
        self.panel._refresh_mavlink_side_actions()
        # A request that targets both sides (``camera=None``, e.g. auto-start)
        # can partially fail: DualPixhawkMavlinkSubscriber.start() only raises
        # when every requested side fails, so a lone P2 failure alongside a
        # successful P1 never reaches ``_pixhawk_connection_failed`` and must be
        # surfaced here instead of being reported as a full success.
        connect_errors = getattr(subscriber, "last_connect_errors", {})
        failed_sides = {side: connect_errors[side] for side in target_sides if side in connect_errors}
        if failed_sides:
            ok_sides = [side for side in target_sides if side not in failed_sides]
            ok_label = (
                " / ".join(self.panel._px4_mag_camera_label(side) for side in ok_sides)
                if ok_sides
                else "없음"
            )
            fail_label = ", ".join(
                f"{self.panel._px4_mag_camera_label(side)} ({error})"
                for side, error in failed_sides.items()
            )
            self.panel.pixhawk_connection_label.setText(
                f"픽스호크 MAVLink: {ok_label} 연결됨, 실패: {fail_label}"
            )
            self.panel._log(f"[픽스호크] MAVLink 부분 연결: 성공={ok_label}, 실패={fail_label}")
        else:
            label = " / ".join(self.panel._px4_mag_camera_label(side) for side in target_sides)
            self.panel.pixhawk_connection_label.setText(f"픽스호크 MAVLink: {label} 세션 시작, 데이터 대기 중")
            self.panel._log(f"[픽스호크] {label} MAVLink 세션이 시작됐습니다. 데이터 수신을 기다립니다.")
        self.panel._request_mavlink_eth0_status(tuple(target_sides))
        self.panel.refresh_px4_mag_manager()
        self.panel.refresh_px4_attitude_cal_status()
        self.panel._refresh_operation_header()

    def _pixhawk_connection_failed(
        self,
        camera: str | None,
        target_sides: Sequence[str],
        error: str,
    ) -> None:
        if self.panel.mavlink_pixhawk is not None:
            try:
                self.panel.mavlink_pixhawk.stop(camera)
            except Exception:
                pass
            if not getattr(self.panel.mavlink_pixhawk, "connections", {}):
                self.panel.mavlink_pixhawk = None
        self.panel._refresh_session_toggle_buttons()
        self.panel._refresh_mavlink_side_actions()
        label = " / ".join(self.panel._px4_mag_camera_label(side) for side in target_sides)
        self.panel.pixhawk_connection_label.setText(f"pixhawk MAVLink {label}: failed ({error})")
        self.panel._log(f"[픽스호크] {label} MAVLink 연결 실패: {error}")
        self.panel._refresh_operation_header()

    def _clear_pixhawk_connection_worker(self) -> None:
        self.panel.pixhawk_connection_thread = None
        self.panel.pixhawk_connection_worker = None
        self.panel._refresh_session_toggle_buttons()

    def _mavlink_side_is_stale(self, side: str) -> bool:
        """True only for confirmed all-RX silence, never ATTITUDE-only stale.

        A live host socket is not remote recovery evidence. New subscribers
        expose the expected-system packet state; the old ATTITUDE timeout is
        retained only for compatible test/legacy adapters.
        """

        subscriber = self.panel.mavlink_pixhawk
        if subscriber is None or side not in getattr(subscriber, "connections", {}):
            return False
        health_reader = getattr(subscriber, "mavlink_health_snapshot", None)
        if callable(health_reader):
            try:
                health = health_reader(side)
            except Exception:
                return False
            state = getattr(getattr(health, "state", None), "value", getattr(health, "state", ""))
            return str(state) == "ETH MAVLINK RX STALLED"
        if side in getattr(subscriber, "_active_command_cameras", set()):
            return False
        timestamps = self.panel.pixhawk_attitude_times.get(side)
        if not timestamps:
            return False
        return time.monotonic() - timestamps[-1] > _MAVLINK_STALE_TIMEOUT_S

    def _reconnect_stale_mavlink_sides(
        self,
        camera: str | None,
        stale_sides: Sequence[str],
        force_setup: bool,
    ) -> None:
        label = " / ".join(self.panel._px4_mag_camera_label(side) for side in stale_sides)
        self.panel._log(
            f"[픽스호크] {label} Ethernet MAVLink 전체 수신이 정지했습니다. "
            "host UDP socket 재연결은 PX4-side stall을 복구하지 않으므로 자동 실행하지 않습니다. "
            "RTCM/추가 명령을 차단하고 verified-USB 또는 named-side 전원 복구를 사용하세요."
        )
        status_label = getattr(self.panel, "pixhawk_connection_label", None)
        set_text = getattr(status_label, "setText", None)
        if callable(set_text):
            set_text(f"픽스호크 MAVLink: {label} ETH RX STALLED · RTCM/명령 차단")
        refresh_actions = getattr(self.panel, "_refresh_mavlink_side_actions", None)
        if callable(refresh_actions):
            refresh_actions()

    def disconnect_mavlink_pixhawk(self, camera: str | None = None) -> None:
        if self.panel._session_recording_active():
            self.panel._log("[픽스호크] 세션 기록 중에는 Pixhawk MAVLink 연결을 변경할 수 없습니다.")
            return
        if getattr(self.panel, "pixhawk_connection_thread", None) is not None:
            self.panel._log("[픽스호크] MAVLink 연결 작업이 끝난 뒤 연결을 해제하세요.")
            return
        if self.panel.pixhawk_disconnection_thread is not None:
            self.panel._log("[픽스호크] MAVLink 연결 해제 작업이 이미 진행 중입니다.")
            return
        target_sides = self.panel._mavlink_target_sides(camera)
        subscriber = self.panel.mavlink_pixhawk
        if subscriber is None:
            self._finish_pixhawk_disconnect(camera, target_sides)
            return
        if not getattr(subscriber, "connections", {}):
            # No transport can block, so avoid creating a QThread that the
            # retryable shutdown path would otherwise have to wait for.
            self._finish_pixhawk_disconnect(camera, target_sides)
            return
        # ``subscriber.stop()`` closes the underlying serial/socket connection,
        # which can block for as long as an in-flight shell/parameter worker
        # holds a concurrent blocking read on the same connection (they share
        # ``_command_state_lock``). Running that on the GUI thread froze the
        # whole app for the duration; do it on a worker thread instead, the
        # same way ``connect_mavlink_pixhawk`` already does for ``start()``.
        thread = QtCore.QThread(self.panel)
        worker = PixhawkDisconnectWorker(subscriber, camera)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda _result, camera=camera, target_sides=target_sides:
            self._finish_pixhawk_disconnect(camera, target_sides),
            QtCore.Qt.QueuedConnection,
        )
        worker.failed.connect(
            lambda error, camera=camera, target_sides=target_sides:
            self._finish_pixhawk_disconnect(camera, target_sides, error=error),
            QtCore.Qt.QueuedConnection,
        )
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(
            self._clear_pixhawk_disconnection_worker,
            QtCore.Qt.QueuedConnection,
        )
        self.panel.pixhawk_disconnection_thread = thread
        self.panel.pixhawk_disconnection_worker = worker
        self.panel._refresh_session_toggle_buttons()
        thread.start()

    def _finish_pixhawk_disconnect(
        self,
        camera: str | None,
        target_sides: Sequence[str],
        *,
        error: str | None = None,
    ) -> None:
        disconnected_subscriber = self.panel.mavlink_pixhawk
        if not getattr(self.panel.mavlink_pixhawk, "connections", {}):
            self.panel.mavlink_pixhawk = None
        if self.panel.mavlink_pixhawk is None and disconnected_subscriber is not None:
            controller_factory = getattr(self.panel, "_gnss_runtime_controller", None)
            if callable(controller_factory):
                controller = controller_factory()
                retire = getattr(controller, "_retire_pixhawk_rtcm_worker", None)
                if callable(retire):
                    retire(
                        expected_subscriber=disconnected_subscriber,
                        wait=False,
                    )
        connection_thread = getattr(self.panel, "pixhawk_connection_thread", None)
        if connection_thread is not None and connection_thread.isRunning():
            connection_thread.quit()
        for side in target_sides:
            self.panel.pixhawk_attitude_times[side].clear()
            self.panel._px4_calibration_auto_refreshed_sides.discard(side)
        self.panel._clear_px4_mag_manager_connection_state(camera)
        self.panel._clear_px4_attitude_cal_connection_state(camera)
        self.panel._refresh_session_toggle_buttons()
        self.panel._refresh_mavlink_side_actions()
        label = " / ".join(self.panel._px4_mag_camera_label(side) for side in target_sides)
        if error is not None:
            self.panel.pixhawk_connection_label.setText(f"픽스호크 MAVLink: {label} 연결 해제 중 오류 ({error})")
            self.panel._log(f"[픽스호크] {label} MAVLink 연결 해제 중 오류: {error}")
        elif self.panel.mavlink_pixhawk is None:
            self.panel.pixhawk_connection_label.setText("픽스호크: 연결 해제")
            self.panel._log(f"[픽스호크] {label} MAVLink 연결을 해제했습니다.")
        else:
            self.panel.pixhawk_connection_label.setText(f"픽스호크 MAVLink: {label} 연결 해제")
            self.panel._log(f"[픽스호크] {label} MAVLink 연결을 해제했습니다.")
        self.panel._refresh_operation_header()

    def _clear_pixhawk_disconnection_worker(self) -> None:
        self.panel.pixhawk_disconnection_thread = None
        self.panel.pixhawk_disconnection_worker = None
        self.panel._refresh_session_toggle_buttons()

    def apply_selected_pixhawk_candidate(self, side: str, *, source: str | None = None) -> None:
        if source is None:
            source = "pc" if side == "host" else "mavlink"
        entry = self.panel._selected_pixhawk_candidate(source)
        if entry is None:
            return
        kind = str(entry.get("kind", ""))
        value = str(entry.get("value", "")).strip()
        if not value:
            self.panel._log("[픽스호크] 선택 후보에 적용할 값이 없습니다.")
            return
        if kind == "PC_IP":
            if side != "host":
                self.panel._log("[픽스호크] PC IP 후보는 Host에 적용 버튼으로 적용하세요.")
                return
            old_text = self.panel.pixhawk_host_address_edit.text().strip()
            self.panel.pixhawk_host_address_edit.setText(value)
            self.panel._log(
                f"[픽스호크] 선택 PC IP 후보를 Pixhawk Host IP에 적용했습니다: "
                f"{old_text or '<empty>'} -> {value} ({entry.get('interface', '')})"
            )
            self.panel.pixhawk_connection_label.setText(f"Pixhawk Host IP 후보 적용됨: {value}")
            self.panel._save_current_settings()
            self.panel._refresh_operation_header()
            return
        if side not in {"left", "right"}:
            self.panel._log(f"[픽스호크] {kind} 후보는 P1/P2에 적용하세요.")
            return
        mavlink_was_active = side in self.panel._mavlink_open_sides()
        if kind == "USB":
            try:
                baud = int(entry.get("baud") or 0)
            except (TypeError, ValueError):
                baud = 0
            if side == "left":
                self.panel.left_pixhawk_bootstrap_url_edit.setText(value)
                if baud:
                    self.panel.left_pixhawk_bootstrap_baud_spin.setValue(baud)
            else:
                self.panel.right_pixhawk_bootstrap_url_edit.setText(value)
                if baud:
                    self.panel.right_pixhawk_bootstrap_baud_spin.setValue(baud)
        elif kind == "MAVLink":
            try:
                baud = int(entry.get("baud") or 0)
            except (TypeError, ValueError):
                baud = 0
            if side == "left":
                self.panel.left_mavlink_url_edit.setText(value)
                if baud:
                    self.panel.left_mavlink_baud_spin.setValue(baud)
            else:
                self.panel.right_mavlink_url_edit.setText(value)
                if baud:
                    self.panel.right_mavlink_baud_spin.setValue(baud)
        else:
            self.panel._log(f"[픽스호크] 지원하지 않는 후보 종류입니다: {kind}")
            return
        label = "Pixhawk 1" if side == "left" else "Pixhawk 2"
        self.panel._log(f"[픽스호크] 선택 후보를 {label}에 적용했습니다: {kind} {value}")
        if kind in {"USB", "MAVLink"}:
            self.panel._refresh_mavlink_side_actions()
            if kind == "MAVLink" and mavlink_was_active:
                self.panel._log(f"[픽스호크] {label} MAVLink URL 후보 적용으로 기존 {label} MAVLink 세션을 해제합니다. 다시 연결하세요.")
                self.panel.disconnect_mavlink_pixhawk(side)
            if kind == "USB":
                self.panel.pixhawk_connection_label.setText(f"{label} USB 부트스트랩 후보 적용됨: ETH MAV 시작(USB)을 누르세요.")
            else:
                self.panel.pixhawk_connection_label.setText(f"{label} MAVLink URL 후보 적용됨: {label} MAVLink 연결을 누르세요.")
        self.panel._refresh_operation_header()

    def apply_persistent_pixhawk_eth(self, camera: str) -> None:
        if self.panel.mavlink_pixhawk is None or camera not in self.panel.mavlink_pixhawk.connections:
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} ETH 영구 설정 실패: MAVLink 연결이 없습니다.")
            return
        try:
            command = self.panel._persistent_pixhawk_eth_command(camera)
            extras_text = self.panel._persistent_pixhawk_extras_text(camera)
            net_cfg_text = self.panel._persistent_pixhawk_net_cfg_text(camera)
        except Exception as exc:
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} ETH 설정 생성 실패: {exc}")
            return
        if self.panel.pixhawk_persistent_config_thread is not None:
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} ETH 영구 설정이 이미 진행 중입니다.")
            return
        thread = QtCore.QThread(self.panel)
        worker = PixhawkPersistentConfigWorker(
            self.panel.mavlink_pixhawk,
            camera=camera,
            command=command,
            extras_text=extras_text,
            net_cfg_text=net_cfg_text,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._apply_pixhawk_persistent_config_result)
        worker.failed.connect(
            lambda error, camera=camera: self.panel._log(
                f"[픽스호크] {camera_label_ko(camera)} ETH 영구 설정 실패: {error}. "
                "PX4 shell과 MAVLink FTP가 모두 막히면 앱만으로 저장할 수 없습니다."
            )
        )
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_pixhawk_persistent_config_worker)
        self.panel.pixhawk_persistent_config_thread = thread
        self.panel.pixhawk_persistent_config_worker = worker
        self.panel._log(f"[픽스호크] {camera_label_ko(camera)} ETH shell/FTP 영구 설정을 시작했습니다.")
        thread.start()

    def _apply_pixhawk_persistent_config_result(self, result: object) -> None:
        data = dict(result or {})
        camera = str(data.get("camera") or "")
        if camera not in {"left", "right"}:
            return
        output = str(data.get("output") or "").strip()
        if output:
            self.panel._log(
                f"[픽스호크] {camera_label_ko(camera)} ETH shell 응답 "
                f"({data.get('flag_label', '')}): {output[-800:]}"
            )
        self.panel._log(
            f"[픽스호크] {camera_label_ko(camera)} ETH MAVLink FTP 영구 저장 완료: "
            "/fs/microsd/net.cfg, /fs/microsd/etc/extras.txt. "
            "재부팅 후 eth0와 Ethernet MAVLink stream 적용을 확인하세요."
        )

    def _clear_pixhawk_persistent_config_worker(self) -> None:
        self.panel.pixhawk_persistent_config_thread = None
        self.panel.pixhawk_persistent_config_worker = None

    def reboot_pixhawk_from_mavlink(self, camera: str) -> None:
        if self.panel.mavlink_pixhawk is None or camera not in self.panel.mavlink_pixhawk.connections:
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} 재부팅 실패: MAVLink 연결이 없습니다.")
            return
        health_reader = getattr(self.panel.mavlink_pixhawk, "mavlink_health_snapshot", None)
        if callable(health_reader):
            try:
                health = health_reader(camera)
            except Exception as exc:
                self.panel._log(
                    f"[픽스호크] {camera_label_ko(camera)} 재부팅 차단: liveness 확인 실패 ({exc})"
                )
                return
            state = str(
                getattr(
                    getattr(health, "state", None),
                    "value",
                    getattr(health, "state", ""),
                )
            )
            if not bool(getattr(health, "command_eligible", False)):
                self.panel._log(
                    f"[픽스호크] {camera_label_ko(camera)} Ethernet 재부팅 명령 차단: "
                    f"state={state or 'UNKNOWN'}. stalled side는 verified-USB recovery를 사용하세요."
                )
                return
        result = QtWidgets.QMessageBox.question(
            self.panel,
            f"{camera_label_ko(camera)} PX4 재부팅",
            f"{camera_label_ko(camera)} Pixhawk에 MAVLink 재부팅 명령을 보냅니다.\n계속할까요?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if result != QtWidgets.QMessageBox.Yes:
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} MAVLink 재부팅 명령을 취소했습니다.")
            return
        self.panel._start_pixhawk_shell_worker(
            camera,
            "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN",
            operation="reboot",
            timeout_s=1.0,
        )

    def _persistent_pixhawk_eth_command(self, camera: str) -> str:
        setup = self.panel._persistent_pixhawk_eth_setup(camera)
        extras = "/fs/microsd/etc/extras.txt"
        extras_lines = list(setup["extras_lines"])
        return "\n".join(
            (
                "mkdir /fs/microsd/etc",
                *setup["param_lines"],
                "param save",
                *extras_lines,
                "mavlink status",
                f"echo '{extras_lines[0]}' > {extras}",
                *(f"echo '{line}' >> {extras}" for line in extras_lines[1:]),
                f"cat {extras}",
            )
        )

    def _persistent_pixhawk_extras_text(self, camera: str) -> str:
        setup = self.panel._persistent_pixhawk_eth_setup(camera)
        return "\n".join(setup["extras_lines"]) + "\n"

    def _persistent_pixhawk_net_cfg_text(self, camera: str) -> str:
        setup = self.panel._persistent_pixhawk_eth_setup(camera)
        return "\n".join(setup["net_cfg_lines"]) + "\n"

    def _persistent_pixhawk_eth_setup(self, camera: str) -> dict[str, object]:
        if camera == "left":
            eth_ip = self.panel.left_pixhawk_eth_ip_edit.text().strip()
            mavlink_url = self.panel.left_mavlink_url_edit.text().strip()
            mavlink_default_port = 14550
            mavlink_system_id = 1
        elif camera == "right":
            eth_ip = self.panel.right_pixhawk_eth_ip_edit.text().strip()
            mavlink_url = self.panel.right_mavlink_url_edit.text().strip()
            mavlink_default_port = 14551
            mavlink_system_id = 2
        else:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        host_ip = self.panel._pixhawk_host_ip_for_mavlink()
        for label, value in (("Pixhawk ETH IP", eth_ip), ("Pixhawk Host IP", host_ip)):
            if not value or any(ch.isspace() or ch in "'\";`$" for ch in value):
                raise ValueError(f"{label} 값이 비어 있거나 shell 명령에 사용할 수 없는 문자를 포함합니다: {value!r}")
        ifup_cmd = "ifup eth0"
        ifconfig_cmd = f"ifconfig eth0 {eth_ip} netmask 255.255.255.0 up"
        mavlink_port = self.panel._mavlink_stream_port(mavlink_url, default=mavlink_default_port)
        mavlink_rate = self.panel.mavlink_stream_rate_spin.value()
        param_lines = [
            f"param set MAV_SYS_ID {mavlink_system_id}",
            "param set MAV_2_CONFIG 1000",
            "param set MAV_2_MODE 0",
            "param set MAV_2_BROADCAST 1",
            f"param set MAV_2_REMOTE_PRT {mavlink_port}",
            f"param set MAV_2_UDP_PRT {mavlink_port}",
            f"param set MAV_2_RATE {mavlink_rate}",
        ]
        # rc.serial owns the one MAVLink instance through MAV_2_CONFIG.
        # extras.txt only establishes the board-specific Ethernet interface.
        extras_lines = [ifup_cmd, ifconfig_cmd]
        net_cfg_lines = [
            "DEVICE=eth0",
            "BOOTPROTO=static",
            "NETMASK=255.255.255.0",
            f"IPADDR={eth_ip}",
            f"ROUTER={host_ip}",
        ]
        return {
            "eth_ip": eth_ip,
            "host_ip": host_ip,
            "ifup_cmd": ifup_cmd,
            "ifconfig_cmd": ifconfig_cmd,
            "param_lines": param_lines,
            "extras_lines": extras_lines,
            "net_cfg_lines": net_cfg_lines,
        }

    def _start_pixhawk_shell_worker(
        self,
        camera: str,
        command: str,
        *,
        operation: str = "shell",
        timeout_s: float = 2.0,
        on_complete=None,
    ) -> bool:
        if self.panel.mavlink_pixhawk is None or camera not in self.panel._mavlink_open_sides():
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} MAVLink 명령 실패: 연결이 없습니다.")
            return False
        if camera in self.panel.pixhawk_shell_threads:
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} MAVLink 명령이 이미 진행 중입니다.")
            return False
        thread = QtCore.QThread(self.panel)
        worker = PixhawkShellCommandWorker(
            self.panel.mavlink_pixhawk,
            camera=camera,
            command=command,
            operation=operation,
            timeout_s=timeout_s,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda result, camera=camera, callback=on_complete: self.panel._apply_pixhawk_shell_worker_result(
                camera, result, callback
            )
        )
        worker.failed.connect(
            lambda error, camera=camera: self.panel._pixhawk_shell_worker_failed(camera, error)
        )
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(lambda camera=camera, thread=thread: self.panel._clear_pixhawk_shell_worker(camera, thread))
        self.panel.pixhawk_shell_threads[camera] = thread
        self.panel.pixhawk_shell_workers[camera] = worker
        thread.start()
        return True

    def _apply_pixhawk_shell_worker_result(self, camera: str, result: object, callback=None) -> None:
        data = dict(result or {})
        output = str(data.get("output") or "")
        if output:
            self.panel._append_px4_shell_output(camera, output)
        operation = str(data.get("operation") or "shell")
        command = str(data.get("command") or "")
        if operation == "reboot":
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} MAVLink 재부팅 명령을 전송했습니다.")
        else:
            flag = str(data.get("flag_label") or "")
            suffix = f" ({flag})" if flag else ""
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} PX4 Shell 명령 완료{suffix}: {command}")
        if operation.startswith("reset_calibration:"):
            group = operation.split(":", 1)[1]
            reset_errors = getattr(self.panel, "_px4_calibration_reset_error", None)
            if not isinstance(reset_errors, dict):
                reset_errors = {}
                setattr(self.panel, "_px4_calibration_reset_error", reset_errors)
            reset_errors.pop(f"{camera}:{group}", None)
            reset_pending = getattr(self.panel, "_px4_calibration_reset_pending", None)
            if isinstance(reset_pending, dict):
                reset_pending.pop(camera, None)
        if callable(callback):
            callback(data)

    def _pixhawk_shell_worker_failed(self, camera: str, error: str) -> None:
        message = f"[픽스호크] {camera_label_ko(camera)} MAVLink 명령 실패: {error}"
        self.panel._log(message)
        dialog = self.panel.px4_shell_dialogs.get(camera)
        if dialog is not None:
            dialog.append_output(f"\n[error] {error}\n")
        reset_pending = getattr(self.panel, "_px4_calibration_reset_pending", None)
        group = reset_pending.pop(camera, None) if isinstance(reset_pending, dict) else None
        if group:
            reset_errors = getattr(self.panel, "_px4_calibration_reset_error", None)
            if not isinstance(reset_errors, dict):
                reset_errors = {}
                setattr(self.panel, "_px4_calibration_reset_error", reset_errors)
            reset_errors[f"{camera}:{group}"] = str(error)

    def reset_px4_calibration_group(self, camera: str, group: str) -> bool:
        """Reset one calibration-owned parameter group on one Pixhawk.

        PX4's shell resolves the firmware-specific defaults.  The Workstation
        only owns the group membership and never substitutes a host-side
        numeric default table.
        """

        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        calibration = px4_calibration_group(group)
        if not calibration.resettable:
            self.panel._log(
                f"[픽스호크] {camera_label_ko(camera)} {calibration.label}은 수동 파라미터 그룹이라 초기화하지 않습니다."
            )
            return False
        if self.panel.mavlink_pixhawk is None or camera not in self.panel._mavlink_open_sides():
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} 캘리브레이션 초기화 실패: MAVLink 연결이 없습니다.")
            return False
        manager = self.panel.px4_mag_managers.get(camera)
        if bool(getattr(manager, "armed", False)):
            self.panel._log(
                f"[픽스호크] {camera_label_ko(camera)} {calibration.label} 초기화 차단: Disarm required"
            )
            return False
        parameter_text = ", ".join(calibration.reset_parameter_names)
        result = QtWidgets.QMessageBox.question(
            self.panel,
            f"Reset {calibration.label}",
            (
                f"{camera_label_ko(camera)}의 {calibration.label} 파라미터를 PX4 firmware default로 초기화합니다.\n\n"
                f"대상: {parameter_text}\n"
                f"{calibration.description}\n"
                "센서 device_id/priority는 그대로 유지되므로 재캘리브레이션 없이 계속 armable하며, "
                "이후 raw(보정 없음) 센서 값이 그대로 사용됩니다. 필요하면 QGC에서 다시 캘리브레이션하세요.\n"
                "기체는 반드시 Disarm 상태여야 합니다. 계속할까요?"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if result != QtWidgets.QMessageBox.Yes:
            self.panel._log(
                f"[픽스호크] {camera_label_ko(camera)} {calibration.label} 초기화를 취소했습니다."
            )
            return False

        command = "param reset " + " ".join(calibration.reset_parameter_names)
        reset_pending = getattr(self.panel, "_px4_calibration_reset_pending", None)
        if not isinstance(reset_pending, dict):
            reset_pending = {}
            setattr(self.panel, "_px4_calibration_reset_pending", reset_pending)
        reset_pending[camera] = calibration.key
        started = self.panel._start_pixhawk_shell_worker(
            camera,
            command,
            operation=f"reset_calibration:{calibration.key}",
            timeout_s=4.0,
            on_complete=lambda _result, camera=camera: self.panel._start_pixhawk_parameter_worker(
                camera, "calibration_refresh"
            ),
        )
        if not started:
            reset_pending.pop(camera, None)
        return started

    def _clear_pixhawk_shell_worker(self, camera: str, thread: QtCore.QThread) -> None:
        if self.panel.pixhawk_shell_threads.get(camera) is thread:
            self.panel.pixhawk_shell_threads.pop(camera, None)
            self.panel.pixhawk_shell_workers.pop(camera, None)

    def _start_pixhawk_parameter_worker(
        self,
        camera: str,
        operation: str,
        *,
        priorities: dict[str, int] | None = None,
    ) -> bool:
        subscriber = self.panel.mavlink_pixhawk
        if subscriber is None or camera not in self.panel._mavlink_open_sides():
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} PARAM 작업 실패: MAVLink 연결이 없습니다.")
            return False
        if camera in self.panel.pixhawk_parameter_threads:
            self.panel._log(f"[픽스호크] {camera_label_ko(camera)} PARAM 작업이 이미 진행 중입니다.")
            return False
        manager = self.panel.px4_mag_managers.get(camera) if hasattr(self.panel, "px4_mag_managers") else None
        actual_selected = self.panel.latest_vehicle_mag_device_ids.get(camera)
        armed = bool(getattr(manager, "armed", False))
        thread = QtCore.QThread(self.panel)
        worker = PixhawkParameterWorker(
            subscriber,
            camera=camera,
            operation=operation,
            actual_selected_device_id=actual_selected,
            armed=armed,
            priorities=priorities,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._apply_pixhawk_parameter_worker_result)
        worker.failed.connect(
            lambda error, camera=camera, operation=operation: self.panel._pixhawk_parameter_worker_failed(
                camera, operation, error
            )
        )
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(
            lambda camera=camera, thread=thread: self.panel._clear_pixhawk_parameter_worker(camera, thread)
        )
        self.panel.pixhawk_parameter_threads[camera] = thread
        self.panel.pixhawk_parameter_workers[camera] = worker
        self.panel._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=False,
            actions_enabled=False,
            tooltip="MAVLink PARAM 작업 중",
        )
        thread.start()
        return True

    def _apply_pixhawk_parameter_worker_result(self, result: object) -> None:
        data = dict(result or {})
        kind = str(data.get("kind") or "")
        status = data.get("status")
        camera = ""
        if hasattr(status, "actual_selected_device_id"):
            camera = next(
                (
                    side
                    for side in ("left", "right")
                    if self.panel.latest_vehicle_mag_device_ids.get(side)
                    == getattr(status, "actual_selected_device_id", None)
                    and side in self.panel.pixhawk_parameter_threads
                ),
                "",
            )
        camera = str(data.get("camera") or camera)
        # The worker payload is intentionally allowed to carry the side
        # explicitly; older fake workers may only return a status object.
        if not camera:
            for side, worker in self.panel.pixhawk_parameter_workers.items():
                if worker.operation == kind:
                    camera = side
                    break
        if kind == "calibration_refresh":
            mag_status = data.get("mag_status")
            attitude_status = data.get("attitude_status")
            if not camera or (mag_status is None and attitude_status is None):
                return
            self.panel._px4_calibration_parameter_error.pop(camera, None)
            if mag_status is not None:
                manager = self.panel.px4_mag_managers.get(camera)
                if manager is not None:
                    manager.actual_selected_device_id = getattr(mag_status, "actual_selected_device_id", None)
                    manager.armed = bool(getattr(mag_status, "armed", False))
                self.panel._render_px4_mag_status(camera, mag_status)
            if attitude_status is not None:
                self.panel._render_px4_attitude_cal_status(camera, attitude_status, using_real_params=True)
            return
        if not camera or status is None:
            return
        if kind in {"mag_refresh", "mag_apply"}:
            self.panel._px4_calibration_parameter_error.pop(camera, None)
            manager = self.panel.px4_mag_managers.get(camera)
            if manager is not None:
                manager.actual_selected_device_id = getattr(status, "actual_selected_device_id", None)
                manager.armed = bool(getattr(status, "armed", False))
            self.panel._render_px4_mag_status(camera, status)
            applied = tuple(str(item) for item in data.get("applied") or ())
            missing = tuple(str(item) for item in data.get("missing") or ())
            if kind == "mag_apply":
                self.panel._log(
                    f"[PX4 MAG] {self.panel._px4_mag_camera_label(camera)} Workstation PARAM 적용: "
                    f"{', '.join(applied) or 'none'}"
                )
                if missing:
                    self.panel._log(
                        f"[PX4 MAG] {self.panel._px4_mag_camera_label(camera)} 미적용 role: {', '.join(missing)}"
                    )
        elif kind == "attitude_refresh":
            self.panel._px4_calibration_parameter_error.pop(camera, None)
            self.panel._render_px4_attitude_cal_status(camera, status, using_real_params=True)

    def _pixhawk_parameter_worker_failed(self, camera: str, operation: str, error: str) -> None:
        if operation == "calibration_refresh":
            self.panel._px4_calibration_parameter_error[camera] = str(error)
        self.panel._log(
            f"[픽스호크] {camera_label_ko(camera)} PARAM 작업 실패 ({operation}): {error}"
        )
        if operation == "mag_refresh" or operation == "mag_apply":
            self.panel._set_px4_mag_manager_button_state(
                camera,
                refresh_enabled=True,
                actions_enabled=True,
                tooltip="",
            )
        elif operation == "attitude_refresh":
            button = self.panel.px4_attitude_cal_refresh_buttons.get(camera)
            if button is not None:
                button.setEnabled(True)

    def _clear_pixhawk_parameter_worker(self, camera: str, thread: QtCore.QThread) -> None:
        if self.panel.pixhawk_parameter_threads.get(camera) is not thread:
            return
        self.panel.pixhawk_parameter_threads.pop(camera, None)
        self.panel.pixhawk_parameter_workers.pop(camera, None)
        manager = self.panel.px4_mag_managers.get(camera) if hasattr(self.panel, "px4_mag_managers") else None
        armed = bool(getattr(manager, "armed", False))
        self.panel._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=True,
            actions_enabled=not armed,
            tooltip="Disarm required" if armed else "",
        )
        button = self.panel.px4_attitude_cal_refresh_buttons.get(camera) if hasattr(self.panel, "px4_attitude_cal_refresh_buttons") else None
        if button is not None:
            button.setEnabled(True)
