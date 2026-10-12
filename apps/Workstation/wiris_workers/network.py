"""WIRIS network workers with their original state, Qt signals and I/O lifetime.

Left/right source identity, image bytes, metadata and source clocks retain
their producer contracts; no NED/FRD/RDF reinterpretation is performed here.
"""
from __future__ import annotations

from collections.abc import Sequence
from PyQt5 import QtCore
from Workstation.workstation_core.network import DEFAULT_HOST_ADDRESS, rtsp_host_port
from Workstation.workstation_core.runtime_camera_status import camera_display_name
from Workstation.workstation_core.wiris_zoom import GET_ETHERNET_MODE_COMMAND, REBOOT_COMMAND, SET_ETHERNET_MODE_COMMAND, SET_IP_ADDRESS_TEMPLATE, SET_SUBNET_MASK_TEMPLATE, build_wiris_command
from . import io as _wiris_io
from . import policy as _wire_policy


class WirisStipWorker(QtCore.QObject):
    """Apply the deliberate WIRIS IP change without blocking the Qt event loop."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        interface_name: str,
        host_address: str,
        host_hints: Sequence[str],
        current_host: str,
        new_ip: str,
        mask: str,
        port: int,
        reboot: bool,
        timeout_s: float,
    ) -> None:
        super().__init__()
        self.interface_name = str(interface_name)
        self.host_address = str(host_address)
        self.host_hints = tuple(str(value) for value in host_hints)
        self.current_host = str(current_host)
        self.new_ip = str(new_ip)
        self.mask = str(mask)
        self.port = int(port)
        self.reboot = bool(reboot)
        self.timeout_s = max(0.1, float(timeout_s))

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            candidates: list[object] = []
            interface_name = self.interface_name
            address_text = self.host_address
            if not interface_name or not _wiris_io.interface_link_exists(interface_name):
                candidates = list(
                    _wiris_io.discover_host_interface_candidates(
                        host_address=address_text or DEFAULT_HOST_ADDRESS,
                        include_unconfigured=True,
                    )
                )
                if not candidates:
                    self.completed.emit({"state": "failed", "error": "camera interface not found"})
                    return
                selected = candidates[0]
                interface_name = str(getattr(selected.interface, "name", ""))
                address_text = str(getattr(selected.interface, "address", ""))
            resolved_address = _wiris_io.resolve_nm_interface_address(interface_name, self.host_hints)
            if resolved_address is not None:
                address_text = str(resolved_address)
            if not address_text:
                self.completed.emit({"state": "failed", "error": "camera host IP is missing", "candidates": candidates})
                return
            if not _wiris_io.interface_has_address(interface_name, address_text):
                applied, message = _wiris_io.apply_interface_address(interface_name, address_text)
                if not applied:
                    self.completed.emit(
                        {
                            "state": "failed",
                            "error": f"camera host IP apply failed: {message}",
                            "candidates": candidates,
                            "interface_name": interface_name,
                            "host_address": address_text,
                        }
                    )
                    return
            commands = [
                SET_IP_ADDRESS_TEMPLATE.format(address=self.new_ip),
                SET_SUBNET_MASK_TEMPLATE.format(mask=self.mask),
            ]
            if self.reboot:
                commands.append(REBOOT_COMMAND)
            statuses: list[str] = []
            for command_text in commands:
                command = build_wiris_command(
                    host=self.current_host,
                    port=self.port,
                    command=command_text,
                    timeout_s=self.timeout_s,
                )
                try:
                    response = _wiris_io.send_tcp_line(command)
                except Exception as exc:
                    self.completed.emit(
                        {
                            "state": "failed",
                            "error": f"{command_text}: {exc}",
                            "statuses": statuses,
                            "candidates": candidates,
                            "interface_name": interface_name,
                            "host_address": address_text,
                        }
                    )
                    return
                statuses.append(f"{command_text}={response or 'sent'}")
            self.completed.emit(
                {
                    "state": "sent",
                    "statuses": statuses,
                    "candidates": candidates,
                    "interface_name": interface_name,
                    "host_address": address_text,
                }
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


class WirisRtspEnableWorker(QtCore.QObject):
    enabled = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        hosts: dict[str, str],
        candidate_ports: dict[str, list[int]],
        urls: dict[str, str],
        timeout_s: float,
        target_labels: Sequence[str] = ("left", "right"),
        reactivate: bool = False,
        visible_encoder_profile: dict[str, int] | None = None,
    ) -> None:
        super().__init__()
        self.hosts = hosts
        self.candidate_ports = candidate_ports
        self.urls = urls
        self.timeout_s = timeout_s
        self.target_labels = tuple(label for label in target_labels if label in {"left", "right"})
        self.reactivate = bool(reactivate)
        # Retained for the explicit/manual profile helper and compatibility.
        # _enable_rtsp() deliberately never applies these stored values.
        self.visible_encoder_profile = dict(visible_encoder_profile or {})

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            result = self._enable_rtsp()
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.enabled.emit(result)
        finally:
            self.finished.emit()

    def _enable_rtsp(self) -> dict[str, object]:
        logs: list[str] = []
        statuses: list[str] = []
        corrected_ports: dict[str, int] = {}
        target_labels = self.target_labels or ("left", "right")
        for label in target_labels:
            host = self.hosts.get(label, "").strip()
            camera_name = camera_display_name(label)
            if not host:
                statuses.append(f"{label}=missing-host")
                logs.append(f"[카메라] {camera_name} WIRIS RTSP 활성화 건너뜀: 제어 host가 비어 있습니다.")
                continue
            port, before, corrected, failure_detail = self._resolve_ethernet_mode_port(label, host)
            if port is None:
                statuses.append(f"{label}=unsupported-port")
                logs.append(f"[카메라] {camera_name} Ethernet Mode 지원 포트를 찾지 못했습니다: {failure_detail}")
                logs.append(
                    f"[카메라] {camera_name} RTSP/Ethernet Mode 활성화 실패: "
                    f"{GET_ETHERNET_MODE_COMMAND} 응답={before}"
                )
                continue
            if corrected:
                corrected_ports[label] = port
                logs.append(
                    f"[카메라] {camera_name} Telnet port 자동 보정: "
                    f"{corrected} -> {port} ({GET_ETHERNET_MODE_COMMAND}={before})"
                )
            ethernet_mode_already_enabled = before.strip().upper() == "TRUE"
            # RTSP enable/reactivate is a read-before-write operation.  Sending
            # SETH TRUE again to an already-enabled camera is still a device
            # mutation and has previously disturbed an otherwise reachable RTSP
            # service, so every automatic path skips the redundant command.
            skip_set_ethernet = ethernet_mode_already_enabled
            response = "SKIPPED"
            if not skip_set_ethernet:
                try:
                    set_command = build_wiris_command(
                        host=host,
                        port=port,
                        command=SET_ETHERNET_MODE_COMMAND,
                        timeout_s=self.timeout_s,
                    )
                    response = _wiris_io.send_tcp_line(set_command)
                except Exception as exc:
                    statuses.append(f"{label}=enable-failed")
                    logs.append(f"[카메라] {camera_name} RTSP/Ethernet Mode 활성화 실패: {exc}")
                    continue
                if not self._set_ethernet_response_ok(response):
                    statuses.append(
                        f"{label}=reactivate-rejected" if self.reactivate else f"{label}=enable-rejected"
                    )
                    logs.append(
                        f"[카메라] {camera_name} RTSP/Ethernet Mode 활성화 실패: "
                        f"{SET_ETHERNET_MODE_COMMAND} 응답={response or 'empty'}"
                    )
                    continue
            statuses.append(f"{label}=reactivated-sent" if self.reactivate else f"{label}=enable-sent")
            if skip_set_ethernet:
                logs.append(
                    f"[카메라] {camera_name} RTSP/Ethernet Mode 이미 활성화됨: "
                    f"{GET_ETHERNET_MODE_COMMAND}={before}; {SET_ETHERNET_MODE_COMMAND} 생략"
                )
            else:
                logs.append(
                    f"[카메라] {camera_name} RTSP/Ethernet Mode "
                    f"{'재활성화' if self.reactivate else '활성화'}: "
                    f"{GET_ETHERNET_MODE_COMMAND}={before or 'n/a'}, "
                    f"{SET_ETHERNET_MODE_COMMAND} 응답={response or 'sent'}"
                    + (" (제어 서비스 보호를 위해 SETH FALSE는 전송하지 않음)" if self.reactivate else "")
                )
            if self.visible_encoder_profile:
                logs.append(
                    f"[카메라] {camera_name} 저장 visible encoder profile은 RTSP "
                    "활성화/재활성화에서 자동 적용하지 않음; "
                    "SEBR/SEQZ/SEGS/SEIR은 명시적 수동 WIRIS 명령에서만 전송"
                )

        should_check = any(
            status.endswith("=enable-sent") or status.endswith("=reactivated-sent")
            for status in statuses
        )
        closed: list[str] = []
        if should_check:
            logs.append(
                f"[카메라] RTSP port가 열릴 때까지 최대 {_wire_policy.RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS}초 대기한 뒤 프레임 수신을 확인합니다."
            )
            closed = self._wait_for_rtsp_ports(logs, labels=target_labels)
        else:
            logs.append(
                "[카메라] RTSP 재활성화에 성공한 카메라가 없어 프레임 확인을 건너뜁니다."
                if self.reactivate
                else "[카메라] RTSP 활성화에 성공한 카메라가 없어 프레임 확인을 건너뜁니다."
            )
        return {
            "mode": "reactivate" if self.reactivate else "enable",
            "target_labels": target_labels,
            "statuses": statuses,
            "logs": logs,
            "corrected_ports": corrected_ports,
            "should_check": should_check,
            "closed": closed,
        }

    def _apply_visible_encoder_profile(
        self,
        host: str,
        port: int,
        logs: list[str],
        camera_name: str,
    ) -> str:
        """Apply an encoder profile only from an explicit operator-owned path.

        Automatic RTSP enable, startup, Auto-Connect, and reactivation must not
        call this helper.  The manual WIRIS command UI remains the normal
        operator path for SEBR/SEQZ/SEGS/SEIR commands.
        """

        if not self.visible_encoder_profile:
            return ""
        fields = (
            ("bitrate", "SEBR VISIBLE", "GEBR VISIBLE"),
            ("qp", "SEQZ VISIBLE", "GEQZ VISIBLE"),
            ("gop", "SEGS VISIBLE", "GEGS VISIBLE"),
            ("idr", "SEIR VISIBLE", "GEIR VISIBLE"),
        )

        def read_value(get_command: str, *, expected_value: int | None = None) -> int:
            last_detail = "no readback"
            for attempt in range(_wire_policy.ENCODER_PROFILE_READBACK_MAX_ATTEMPTS + 1):
                if attempt:
                    QtCore.QThread.msleep(_wire_policy.ENCODER_PROFILE_READBACK_RETRY_INTERVAL_MS)
                try:
                    response = _wiris_io.send_tcp_line(
                        build_wiris_command(
                            host=host,
                            port=port,
                            command=get_command,
                            timeout_s=self.timeout_s,
                        )
                    )
                    value = int(response.strip())
                except Exception as exc:
                    last_detail = str(exc)
                    continue
                if expected_value is not None and value != expected_value:
                    last_detail = str(value)
                    continue
                QtCore.QThread.msleep(_wire_policy.ENCODER_PROFILE_COMMAND_SETTLE_MS)
                return value
            suffix = "" if expected_value is None else f" expected={expected_value}"
            raise RuntimeError(f"readback={last_detail}{suffix}")

        QtCore.QThread.msleep(_wire_policy.ENCODER_PROFILE_COMMAND_SETTLE_MS)
        expected: dict[str, int] = {}
        initial: dict[str, int] = {}
        for name, _set_prefix, get_command in fields:
            if name not in self.visible_encoder_profile:
                return f"missing {name}"
            expected[name] = int(self.visible_encoder_profile[name])
            try:
                initial[name] = read_value(get_command)
            except Exception as exc:
                return f"{get_command} 초기 readback 실패: {exc}"

        changed: list[str] = []
        readback = dict(initial)
        for name, set_prefix, get_command in fields:
            value = expected[name]
            if initial[name] == value:
                continue
            try:
                response = _wiris_io.send_tcp_line(
                    build_wiris_command(
                        host=host,
                        port=port,
                        command=f"{set_prefix} {value}",
                        timeout_s=self.timeout_s,
                    )
                )
            except Exception as exc:
                return f"{set_prefix} {value} 전송 실패: {exc}"
            if response.strip().upper() != "OK":
                return f"{set_prefix} {value} 응답={response or 'empty'}"

            QtCore.QThread.msleep(_wire_policy.ENCODER_PROFILE_COMMAND_SETTLE_MS)
            try:
                actual = read_value(get_command, expected_value=value)
            except Exception as exc:
                return f"{get_command} readback 실패: {exc}"
            readback[name] = actual
            changed.append(name)

        change_text = ",".join(changed) if changed else "none (readback 일치; SET 생략)"
        logs.append(
            f"[카메라] {camera_name} visible encoder profile 확인: "
            f"bitrate={readback['bitrate']}, qp={readback['qp']}, "
            f"gop={readback['gop']}, idr={readback['idr']}; changed={change_text}"
        )
        return ""

    def _resolve_ethernet_mode_port(self, camera: str, host: str) -> tuple[int | None, str, int | None, str]:
        failures = []
        ports = self.candidate_ports.get(camera, [])
        original_port = ports[0] if ports else None
        for port in ports:
            try:
                command = build_wiris_command(
                    host=host,
                    port=port,
                    command=GET_ETHERNET_MODE_COMMAND,
                    timeout_s=self.timeout_s,
                )
                response = _wiris_io.send_tcp_line(command)
            except Exception as exc:
                failures.append(f"{port}=failed:{exc}")
                continue
            if self._ethernet_mode_response_ok(response):
                corrected = original_port if original_port is not None and port != original_port else None
                return port, response, corrected, ", ".join(failures)
            failures.append(f"{port}={response or 'empty'}")
        return None, failures[-1] if failures else "no-candidate", None, ", ".join(failures)

    def _wait_for_rtsp_ports(self, logs: list[str], *, labels: Sequence[str] | None = None) -> list[str]:
        closed: list[str] = []
        QtCore.QThread.msleep(_wire_policy.RTSP_ENABLE_SETTLE_DELAY_MS)
        for attempt in range(_wire_policy.RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS + 1):
            closed = self._closed_rtsp_ports(labels=labels)
            if not closed:
                logs.append("[카메라] RTSP port 열림 확인 완료. 프레임 수신 확인을 시작합니다.")
                return []
            if attempt >= _wire_policy.RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS:
                logs.append("[카메라] RTSP port 대기 초과: " + ", ".join(closed))
                return closed
            logs.append("[카메라] RTSP port 대기 중: " + ", ".join(closed))
            QtCore.QThread.msleep(_wire_policy.RTSP_ENABLE_PORT_WAIT_INTERVAL_MS)
        return closed

    def _closed_rtsp_ports(self, *, labels: Sequence[str] | None = None) -> list[str]:
        closed = []
        selected_labels = tuple(labels) if labels is not None else tuple(self.urls)
        for camera in selected_labels:
            url = self.urls.get(camera, "")
            try:
                host, port = rtsp_host_port(url.strip())
            except Exception as exc:
                closed.append(f"{camera}=invalid-url:{exc}")
                continue
            if not _wiris_io.check_tcp_port(host, port, timeout=0.5):
                closed.append(f"{camera}={host}:{port}")
        return closed

    def _ethernet_mode_response_ok(self, response: str) -> bool:
        return response.strip().upper() in {"TRUE", "FALSE"}

    def _set_ethernet_response_ok(self, response: str) -> bool:
        return response.strip().upper() in {"OK", "TRUE"}

# Keep the named legacy class identity; implementation ownership is here.
WirisStipWorker.__module__ = "Workstation.wiris_runtime_workers"
WirisRtspEnableWorker.__module__ = "Workstation.wiris_runtime_workers"
