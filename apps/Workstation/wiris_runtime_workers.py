"""Workstation-owned WIRIS clock, network-mode, and RTSP workers.

This module belongs only to the workstation app. It does not import or reuse
the StereoWIRIS app implementation. Workers emit data through Qt signals and
leave UI state ownership with the workstation panel/controller.
"""

from __future__ import annotations

import ftplib
import math
import os
import re
import threading
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from statistics import median

from PyQt5 import QtCore

from Workstation.workstation_core.network import (
    DEFAULT_HOST_ADDRESS,
    apply_interface_address,
    check_tcp_port,
    discover_host_interface_candidates,
    interface_has_address,
    interface_link_exists,
    resolve_nm_interface_address,
    rtsp_host_port,
)
from Workstation.workstation_core.runtime_camera_status import camera_display_name
from Workstation.workstation_core.wiris_zoom import (
    GET_ETHERNET_MODE_COMMAND,
    LINE_TERMINATOR,
    REBOOT_COMMAND,
    SET_ETHERNET_MODE_COMMAND,
    SET_IP_ADDRESS_TEMPLATE,
    SET_SUBNET_MASK_TEMPLATE,
    build_wiris_command,
    build_zoom_command,
    send_tcp_line,
)
from Workstation.workstation_core.yolo_stream import (
    DECODER_AUTO,
    STREAM_BACKEND_OPENCV,
    STREAM_BACKEND_OPENCV_AUTO,
    STREAM_BACKEND_PYAV,
    build_gst_pipeline_text,
    decoder_candidates,
    normalize_decoder_mode,
    normalize_rtsp_transport,
    normalize_stream_backend,
)

RTSP_ENABLE_SETTLE_DELAY_MS = 4000
RTSP_ENABLE_PORT_WAIT_INTERVAL_MS = 1000
RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS = 12
WIRIS_STUS_RTT_PROBE_COMMAND = "HIWS"
WIRIS_STUS_RTT_PROBE_COUNT = 3
WIRIS_STUS_MAX_VALID_RTT_S = 0.250
WIRIS_STUS_PARSE_MARGIN_S = 0.002
WIRIS_LRF_COMMAND = "GLRD"


_LRF_NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:[.,]\d*)?|[.,]\d+)(?:[eE][-+]?\d+)?")


def parse_lrf_distance(response: object) -> float | None:
    """Parse the WIRIS ``GLRD`` response, whose distance is in metres.

    The camera may return a bare number, a number prefixed by ``OK``, or
    ``ERR`` when the optional LRF is unavailable.  Keep the parser tolerant of
    decimal commas and the spacing used in SDK examples, but never turn an
    error response into a numeric sample.
    """

    text = str(response or "").replace("\x00", "").strip()
    if not text or text.upper().startswith("ERR"):
        return None
    matches = _LRF_NUMBER_RE.findall(text)
    if not matches:
        return None
    token = matches[-1]
    if "," in token and "." not in token:
        token = token.replace(",", ".")
    try:
        value = float(token)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value >= 0.0 else None


class WirisClockCommandWorker(QtCore.QObject):
    """Send WIRIS STUS/GDTI commands without blocking the UI thread.

    WIRIS's STUS command does not account for command transfer/parsing time.
    For STUS, measure the control-channel RTT with harmless HIWS probes and
    set each camera to ``PC send time + RTT/2 + a small parse margin``.  The
    probe and compensation values are returned as evidence with the result.
    """

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
        *,
        command_text: str,
        shared_utc: bool = False,
    ) -> None:
        super().__init__()
        self.target_infos = tuple(
            (str(label), str(host), int(port), max(0.1, float(timeout_s)))
            for label, host, port, timeout_s in target_infos
        )
        self.command_text = str(command_text).strip().upper()
        self.shared_utc = bool(shared_utc)

    @staticmethod
    def _stus_command(unix_time_s: float) -> str:
        seconds = int(unix_time_s)
        microseconds = int(round((float(unix_time_s) - seconds) * 1_000_000))
        if microseconds >= 1_000_000:
            seconds += 1
            microseconds -= 1_000_000
        return f"STUS {seconds} {microseconds}"

    @staticmethod
    def _normalized_stus_rtt_s(rtt_median_s: object) -> float | None:
        try:
            value = float(rtt_median_s)
        except (TypeError, ValueError):
            return None
        if (
            not math.isfinite(value)
            or value < 0.0
            or value > WIRIS_STUS_MAX_VALID_RTT_S
        ):
            return None
        return value

    @classmethod
    def _stus_one_way_delay_s(cls, rtt_median_s: object) -> float:
        value = cls._normalized_stus_rtt_s(rtt_median_s)
        return 0.0 if value is None else value / 2.0

    @classmethod
    def _stus_compensation_s(cls, rtt_median_s: object) -> float:
        value = cls._normalized_stus_rtt_s(rtt_median_s)
        if value is None:
            return 0.0
        return value / 2.0 + WIRIS_STUS_PARSE_MARGIN_S

    @staticmethod
    def _empty_rtt_probe_result() -> dict[str, object]:
        return {
            "command": WIRIS_STUS_RTT_PROBE_COMMAND,
            "samples_s": [],
            "median_s": None,
            "errors": [],
        }

    def _measure_control_rtt(
        self,
        *,
        host: str,
        port: int,
        timeout_s: float,
    ) -> dict[str, object]:
        probe = self._empty_rtt_probe_result()
        samples: list[float] = []
        errors: list[str] = []
        for _index in range(WIRIS_STUS_RTT_PROBE_COUNT):
            started_mono_s = time.perf_counter()
            try:
                command = build_wiris_command(
                    host=host,
                    port=port,
                    command=WIRIS_STUS_RTT_PROBE_COMMAND,
                    timeout_s=timeout_s,
                    terminator=LINE_TERMINATOR,
                )
                response = send_tcp_line(command)
            except Exception as exc:
                errors.append(str(exc))
                continue
            elapsed_s = max(0.0, time.perf_counter() - started_mono_s)
            if not str(response or "").strip():
                errors.append("HIWS empty response")
                continue
            if elapsed_s > WIRIS_STUS_MAX_VALID_RTT_S:
                errors.append(
                    f"HIWS RTT {elapsed_s * 1000.0:.1f}ms exceeds "
                    f"{WIRIS_STUS_MAX_VALID_RTT_S * 1000.0:.0f}ms limit"
                )
                continue
            samples.append(elapsed_s)
        probe["samples_s"] = samples
        probe["median_s"] = median(samples) if samples else None
        probe["errors"] = errors
        return probe

    @QtCore.pyqtSlot()
    def run(self) -> None:
        results: dict[str, dict[str, object]] = {}
        command_times: dict[str, float] = {}
        tcp_start_times: dict[str, float] = {}
        tcp_start_utc_times: dict[str, float] = {}
        rtt_probe_by_camera: dict[str, dict[str, object]] = {}
        shared_command_time: dict[str, float] = {}
        barrier = (
            threading.Barrier(len(self.target_infos), action=lambda: shared_command_time.update(unix=time.time()))
            if self.shared_utc and self.command_text != "STUS" and len(self.target_infos) >= 2
            else None
        )

        def send_one(label: str, host: str, port: int, timeout_s: float) -> None:
            started_utc_s = time.time()
            probe = self._empty_rtt_probe_result()
            one_way_delay_s = 0.0
            parse_margin_s = 0.0
            compensation_s = 0.0
            tcp_start_utc_s: float | None = None
            try:
                if self.command_text == "STUS":
                    probe = self._measure_control_rtt(
                        host=host,
                        port=port,
                        timeout_s=timeout_s,
                    )
                    rtt_median_s = probe.get("median_s")
                    valid_rtt_s = self._normalized_stus_rtt_s(rtt_median_s)
                    one_way_delay_s = self._stus_one_way_delay_s(valid_rtt_s)
                    if valid_rtt_s is not None:
                        parse_margin_s = WIRIS_STUS_PARSE_MARGIN_S
                    compensation_s = one_way_delay_s + parse_margin_s
                rtt_probe_by_camera[label] = probe
                if barrier is not None:
                    barrier.wait(timeout=max(1.0, timeout_s))
                    tcp_start_utc_s = time.time()
                    command_time_s = float(shared_command_time.get("unix", tcp_start_utc_s))
                else:
                    # Capture the PC UTC immediately before the STUS socket
                    # call.  The target is camera-specific because each
                    # control connection has a different measured RTT.
                    tcp_start_utc_s = time.time()
                    command_time_s = tcp_start_utc_s + compensation_s
                command_text = self._stus_command(command_time_s) if self.command_text == "STUS" else self.command_text
                command = build_wiris_command(
                    host=host,
                    port=port,
                    command=command_text,
                    timeout_s=timeout_s,
                    terminator=LINE_TERMINATOR,
                )
                command_times[label] = command_time_s
                tcp_start_utc_times[label] = tcp_start_utc_s
                tcp_start_times[label] = time.perf_counter()
                response = send_tcp_line(command)
            except Exception as exc:
                results[label] = {
                    "response": f"실패: {exc}",
                    "query_started_utc_s": started_utc_s,
                    "query_finished_utc_s": time.time(),
                    "command": self._stus_command(command_times.get(label, started_utc_s))
                    if self.command_text == "STUS"
                    else self.command_text,
                    "host": host,
                    "port": port,
                    "stus_rtt_probe_command": probe.get("command"),
                    "stus_rtt_samples_s": list(probe.get("samples_s") or []),
                    "stus_rtt_median_s": probe.get("median_s"),
                    "stus_rtt_probe_errors": list(probe.get("errors") or []),
                    "stus_one_way_delay_s": one_way_delay_s,
                    "stus_parse_margin_s": parse_margin_s,
                    "stus_compensation_s": compensation_s,
                    "stus_requested_utc_s": command_times.get(label),
                    "tcp_start_utc_s": tcp_start_utc_s,
                }
                return
            results[label] = {
                "response": response or "<no response>",
                "query_started_utc_s": started_utc_s,
                "query_finished_utc_s": time.time(),
                "command": command.command,
                "host": host,
                "port": port,
                "stus_rtt_probe_command": probe.get("command"),
                "stus_rtt_samples_s": list(probe.get("samples_s") or []),
                "stus_rtt_median_s": probe.get("median_s"),
                "stus_rtt_probe_errors": list(probe.get("errors") or []),
                "stus_one_way_delay_s": one_way_delay_s,
                "stus_parse_margin_s": parse_margin_s,
                "stus_compensation_s": compensation_s,
                "stus_requested_utc_s": command_times.get(label),
                "tcp_start_utc_s": tcp_start_utc_s,
            }

        threads = [
            threading.Thread(
                target=send_one,
                args=(label, host, port, timeout_s),
                name=f"wiris-clock-{self.command_text.lower()}-{label}",
            )
            for label, host, port, timeout_s in self.target_infos
        ]
        try:
            for thread in threads:
                thread.start()
            max_timeout_s = max(
                (max(1.0, timeout_s) for _label, _host, _port, timeout_s in self.target_infos),
                default=1.0,
            )
            command_count = WIRIS_STUS_RTT_PROBE_COUNT + 1 if self.command_text == "STUS" else 1
            for thread in threads:
                thread.join(timeout=max_timeout_s * command_count + 1.0)
            for label, host, port, _timeout_s in self.target_infos:
                results.setdefault(
                    label,
                    {
                        "response": "실패: 전송 스레드 timeout",
                        "query_started_utc_s": None,
                        "query_finished_utc_s": None,
                        "command": self.command_text,
                        "host": host,
                        "port": port,
                    },
                )
            self.completed.emit(
                {
                    "command": self.command_text,
                    "results": results,
                    "command_times": command_times,
                    "tcp_start_times": tcp_start_times,
                    "tcp_start_utc_times": tcp_start_utc_times,
                    "requested_utc_by_camera": dict(command_times),
                    "stus_rtt_by_camera": rtt_probe_by_camera,
                    "compensation_method": (
                        "control-rtt-half-plus-parse-margin"
                        if self.command_text == "STUS"
                        else None
                    ),
                    "stus_parse_margin_s": WIRIS_STUS_PARSE_MARGIN_S
                    if self.command_text == "STUS"
                    else None,
                    "requested_utc_s": (
                        min(command_times.values())
                        if self.command_text == "STUS" and command_times
                        else shared_command_time.get("unix")
                        if self.shared_utc
                        else next(iter(command_times.values()), None)
                    ),
                }
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


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
            if not interface_name or not interface_link_exists(interface_name):
                candidates = list(
                    discover_host_interface_candidates(
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
            resolved_address = resolve_nm_interface_address(interface_name, self.host_hints)
            if resolved_address is not None:
                address_text = str(resolved_address)
            if not address_text:
                self.completed.emit({"state": "failed", "error": "camera host IP is missing", "candidates": candidates})
                return
            if not interface_has_address(interface_name, address_text):
                applied, message = apply_interface_address(interface_name, address_text)
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
                    response = send_tcp_line(command)
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
    ) -> None:
        super().__init__()
        self.hosts = hosts
        self.candidate_ports = candidate_ports
        self.urls = urls
        self.timeout_s = timeout_s
        self.target_labels = tuple(label for label in target_labels if label in {"left", "right"})
        self.reactivate = bool(reactivate)

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
            try:
                set_command = build_wiris_command(
                    host=host,
                    port=port,
                    command=SET_ETHERNET_MODE_COMMAND,
                    timeout_s=self.timeout_s,
                )
                response = send_tcp_line(set_command)
            except Exception as exc:
                statuses.append(f"{label}=enable-failed")
                logs.append(f"[카메라] {camera_name} RTSP/Ethernet Mode 활성화 실패: {exc}")
                continue
            if not self._set_ethernet_response_ok(response):
                statuses.append(f"{label}=reactivate-rejected" if self.reactivate else f"{label}=enable-rejected")
                logs.append(
                    f"[카메라] {camera_name} RTSP/Ethernet Mode 활성화 실패: "
                    f"{SET_ETHERNET_MODE_COMMAND} 응답={response or 'empty'}"
                )
                continue
            statuses.append(f"{label}=reactivated-sent" if self.reactivate else f"{label}=enable-sent")
            logs.append(
                f"[카메라] {camera_name} RTSP/Ethernet Mode "
                f"{'재활성화' if self.reactivate else '활성화'}: "
                f"{GET_ETHERNET_MODE_COMMAND}={before or 'n/a'}, {SET_ETHERNET_MODE_COMMAND} 응답={response or 'sent'}"
                + (" (제어 서비스 보호를 위해 SETH FALSE는 전송하지 않음)" if self.reactivate else "")
            )

        should_check = any(
            status.endswith("=enable-sent") or status.endswith("=reactivated-sent")
            for status in statuses
        )
        closed: list[str] = []
        if should_check:
            logs.append(
                f"[카메라] RTSP port가 열릴 때까지 최대 {RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS}초 대기한 뒤 프레임 수신을 확인합니다."
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
                response = send_tcp_line(command)
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
        QtCore.QThread.msleep(RTSP_ENABLE_SETTLE_DELAY_MS)
        for attempt in range(RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS + 1):
            closed = self._closed_rtsp_ports(labels=labels)
            if not closed:
                logs.append("[카메라] RTSP port 열림 확인 완료. 프레임 수신 확인을 시작합니다.")
                return []
            if attempt >= RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS:
                logs.append("[카메라] RTSP port 대기 초과: " + ", ".join(closed))
                return closed
            logs.append("[카메라] RTSP port 대기 중: " + ", ".join(closed))
            QtCore.QThread.msleep(RTSP_ENABLE_PORT_WAIT_INTERVAL_MS)
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
            if not check_tcp_port(host, port, timeout=0.5):
                closed.append(f"{camera}={host}:{port}")
        return closed

    def _ethernet_mode_response_ok(self, response: str) -> bool:
        return response.strip().upper() in {"TRUE", "FALSE"}

    def _set_ethernet_response_ok(self, response: str) -> bool:
        return response.strip().upper() in {"OK", "TRUE"}


class WirisCommandWorker(QtCore.QObject):
    """Send one SDK command to one or both cameras off the UI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
        *,
        command_text: str,
        terminator: str,
    ) -> None:
        super().__init__()
        self.target_infos = tuple(
            (str(label), str(host), int(port), max(0.1, float(timeout_s)))
            for label, host, port, timeout_s in target_infos
        )
        self.command_text = str(command_text)
        self.terminator = str(terminator)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        statuses: dict[str, str] = {}
        command_logs: dict[str, str] = {}
        send_times: dict[str, float] = {}
        barrier = threading.Barrier(len(self.target_infos)) if len(self.target_infos) >= 2 else None

        def send_one(label: str, host: str, port: int, timeout_s: float) -> None:
            try:
                command = build_wiris_command(
                    host=host,
                    port=port,
                    command=self.command_text,
                    timeout_s=timeout_s,
                    terminator=self.terminator,
                )
                if barrier is not None:
                    barrier.wait(timeout=max(1.0, timeout_s))
                send_times[label] = time.perf_counter()
                response = send_tcp_line(command)
            except Exception as exc:
                statuses[label] = f"실패: {exc}"
                return
            response_text = response or "<no response>"
            statuses[label] = response_text
            command_logs[label] = f"> {label} {command.host}:{command.port} {command.command}\n{response_text}\n"

        threads = [
            threading.Thread(
                target=send_one,
                args=(label, host, port, timeout_s),
                name=f"wiris-command-{label}",
            )
            for label, host, port, timeout_s in self.target_infos
        ]
        try:
            for thread in threads:
                thread.start()
            max_timeout_s = max(max(1.0, timeout_s) for _label, _host, _port, timeout_s in self.target_infos)
            for thread in threads:
                thread.join(timeout=max_timeout_s + 1.0)
            for label, _host, _port, _timeout_s in self.target_infos:
                statuses.setdefault(label, "실패: 전송 스레드 timeout")
            self.completed.emit(
                {
                    "statuses": statuses,
                    "command_logs": command_logs,
                    "send_times": send_times,
                    "target_infos": self.target_infos,
                }
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


class WirisPnPPhotoCaptureWorker(QtCore.QObject):
    """Enable one visible still channel and wait for camera-local storage.

    The WIRIS camera uses the same ``CPTR``/``ICPT`` lifecycle for the normal
    visible operating-zoom still and the visible HR still.  The channel's
    image-storage command is injected so the two evidence paths can remain
    separate while sharing the timing and completion contract.
    """

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    #: JPG/TIFF are the only still formats WIRIS Enterprise stores for CPTR.
    FTP_IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".tif", ".tiff")

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
        *,
        capture_id: str,
        image_enable_command: str = "SIHR TRUE",
        capture_channel: str = "hr",
        completion_timeout_s: float = 10.0,
        ftp_enabled: bool = False,
        ftp_settings_by_label: Mapping[str, Mapping[str, object]] | None = None,
        ftp_output_dir: str | os.PathLike[str] | None = None,
    ) -> None:
        super().__init__()
        self.target_infos = tuple(
            (str(label), str(host), int(port), max(0.1, float(timeout_s)))
            for label, host, port, timeout_s in target_infos
        )
        self.capture_id = str(capture_id)
        self.image_enable_command = str(image_enable_command).strip()
        self.capture_channel = str(capture_channel).strip().lower() or "hr"
        self.completion_timeout_s = max(0.0, float(completion_timeout_s))
        # FTP reaches the same WIRIS Enterprise host as the TCP command
        # channel above; only the port differs. Downloading is best-effort:
        # a failure here never turns an otherwise-successful CPTR capture
        # into a failed one, it only leaves ftp_downloaded False so the
        # operator falls back to the existing manual import path.
        # Each camera runs its own local FTP account, so the connection
        # settings are looked up per label (``left``/``right``) rather than
        # shared across both cameras.
        self.ftp_enabled = bool(ftp_enabled)
        self.ftp_settings_by_label: dict[str, dict[str, object]] = {
            str(label): dict(config) for label, config in (ftp_settings_by_label or {}).items()
        }
        self.ftp_output_dir = (
            None if ftp_output_dir is None else Path(ftp_output_dir) / f"{self.capture_channel}_capture"
        )

    @staticmethod
    def _capture_busy(response: str) -> bool:
        text = str(response or "").strip().upper()
        if not text:
            return False
        if (
            any(token in text for token in ("FALSE", "IDLE", "DONE", "READY", " 0", "=0"))
            or text == "0"
        ):
            return False
        return (
            any(token in text for token in ("TRUE", "BUSY", "CAPT", " 1", "=1"))
            or text == "1"
        )

    @staticmethod
    def _send(host: str, port: int, timeout_s: float, command_text: str) -> str:
        return send_tcp_line(
            build_wiris_command(
                host=host,
                port=port,
                command=command_text,
                timeout_s=timeout_s,
                terminator=LINE_TERMINATOR,
            )
        )

    def _latest_ftp_file_name(self, ftp: ftplib.FTP) -> str:
        names = [
            name
            for name in ftp.nlst()
            if Path(name).suffix.lower() in self.FTP_IMAGE_EXTENSIONS
        ]
        if not names:
            raise FileNotFoundError("WIRIS FTP remote directory has no JPG/TIFF images")

        def _modified_at(name: str) -> str:
            try:
                response = ftp.voidcmd(f"MDTM {name}")
                return response.strip().rsplit(" ", 1)[-1]
            except ftplib.all_errors:
                return ""

        return max(names, key=_modified_at)

    def _download_latest_ftp_file(self, host: str, label: str) -> Path:
        if self.ftp_output_dir is None:
            raise RuntimeError("no local FTP download directory was configured")
        self.ftp_output_dir.mkdir(parents=True, exist_ok=True)
        config = self.ftp_settings_by_label.get(label, {})
        port = int(config.get("port", 21))
        user = str(config.get("user") or "anonymous")
        password = str(config.get("password", ""))
        remote_dir = str(config.get("remote_dir") or "/")
        timeout_s = max(0.1, float(config.get("timeout_s", 10.0)))
        with ftplib.FTP() as ftp:
            ftp.connect(host, port, timeout=timeout_s)
            ftp.login(user=user, passwd=password)
            if remote_dir:
                ftp.cwd(remote_dir)
            latest_name = self._latest_ftp_file_name(ftp)
            destination = self.ftp_output_dir / f"{self.capture_id}_{label}_{latest_name}"
            with destination.open("wb") as handle:
                ftp.retrbinary(f"RETR {latest_name}", handle.write)
        return destination

    @QtCore.pyqtSlot()
    def run(self) -> None:
        results: dict[str, dict[str, object]] = {}

        def capture_one(label: str, host: str, port: int, timeout_s: float) -> None:
            item: dict[str, object] = {
                "state": "failed",
                f"{self.capture_channel}_enabled": False,
                "capture_complete": False,
                "stored_on_camera": False,
                "ftp_downloaded": False,
            }
            try:
                item[f"{self.capture_channel}_enable_response"] = self._send(
                    host, port, timeout_s, self.image_enable_command
                )
                item[f"{self.capture_channel}_enabled"] = True
                item["capture_requested_unix_ns"] = time.time_ns()
                item["capture_response"] = self._send(host, port, timeout_s, "CPTR")
                deadline_s = time.monotonic() + self.completion_timeout_s
                while True:
                    response = self._send(host, port, timeout_s, "ICPT")
                    item["capture_status_response"] = response
                    if not self._capture_busy(response):
                        item["capture_complete"] = True
                        item["stored_on_camera"] = True
                        item["state"] = "completed"
                        break
                    if time.monotonic() >= deadline_s:
                        item["state"] = "timeout"
                        item["error"] = "ICPT completion timeout"
                        break
                    time.sleep(0.25)
            except Exception as exc:
                item["error"] = str(exc)
            if item.get("stored_on_camera") and self.ftp_enabled:
                try:
                    downloaded = self._download_latest_ftp_file(host, label)
                    item["ftp_downloaded"] = True
                    item["ftp_local_path"] = str(downloaded)
                except Exception as exc:
                    item["ftp_downloaded"] = False
                    item["ftp_error"] = str(exc)
            item["finished_unix_ns"] = time.time_ns()
            results[label] = item

        threads = [
            threading.Thread(
                target=capture_one,
                args=(label, host, port, timeout_s),
                name=f"wiris-pnp-photo-{self.capture_id}-{label}",
            )
            for label, host, port, timeout_s in self.target_infos
        ]
        try:
            for thread in threads:
                thread.start()
            max_ftp_timeout_s = max(
                (float(config.get("timeout_s", 10.0)) for config in self.ftp_settings_by_label.values()),
                default=10.0,
            )
            ftp_wait_budget_s = max_ftp_timeout_s * 2.0 if self.ftp_enabled else 0.0
            maximum_wait_s = self.completion_timeout_s + ftp_wait_budget_s + sum(
                max(1.0, timeout_s) * 3.0
                for _label, _host, _port, timeout_s in self.target_infos[:1]
            )
            for thread in threads:
                thread.join(timeout=maximum_wait_s)
            for label, _host, _port, _timeout_s in self.target_infos:
                results.setdefault(
                    label,
                    {
                        "state": "failed",
                        "error": "capture worker thread timeout",
                        f"{self.capture_channel}_enabled": False,
                        "capture_complete": False,
                        "stored_on_camera": False,
                        "ftp_downloaded": False,
                        "finished_unix_ns": time.time_ns(),
                    },
                )
            camera_items = list(results.values())
            self.completed.emit(
                {
                    "capture_id": self.capture_id,
                    "capture_channel": self.capture_channel,
                    "image_enable_command": self.image_enable_command,
                    "commands": (self.image_enable_command, "CPTR", "ICPT"),
                    "cameras": results,
                    "stored_on_camera": bool(camera_items)
                    and all(item.get("stored_on_camera") for item in camera_items),
                    "ftp_downloaded": self.ftp_enabled
                    and bool(camera_items)
                    and all(item.get("ftp_downloaded") for item in camera_items),
                }
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


class WirisOZPhotoCaptureWorker(WirisPnPPhotoCaptureWorker):
    """Capture a normal visible operating-zoom still on both cameras."""

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
        *,
        capture_id: str,
        completion_timeout_s: float = 10.0,
        ftp_enabled: bool = False,
        ftp_settings_by_label: Mapping[str, Mapping[str, object]] | None = None,
        ftp_output_dir: str | os.PathLike[str] | None = None,
    ) -> None:
        super().__init__(
            target_infos,
            capture_id=capture_id,
            image_enable_command="SIVI TRUE",
            capture_channel="oz",
            completion_timeout_s=completion_timeout_s,
            ftp_enabled=ftp_enabled,
            ftp_settings_by_label=ftp_settings_by_label,
            ftp_output_dir=ftp_output_dir,
        )


class WirisLrfPollWorker(QtCore.QObject):
    """Read the optional WIRIS laser rangefinder once, off the UI thread.

    The control port is a single embedded TCP service shared with zoom,
    clock-sync, and RTSP-enable commands; each read opens and closes its own
    connection (``send_tcp_line``). Earlier versions of this worker looped
    forever at 1 Hz, which kept that connection churn running for as long as
    the Alignment workspace stayed open and could exhaust the camera's
    control service over a long session. Reads now happen only when a caller
    explicitly requests one.
    """

    sampled = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
    ) -> None:
        super().__init__()
        self.target_infos = tuple(
            (str(label), str(host), int(port), max(0.1, float(timeout_s)))
            for label, host, port, timeout_s in target_infos
        )

    def _poll_one(
        self,
        label: str,
        host: str,
        port: int,
        timeout_s: float,
    ) -> dict[str, object]:
        started_s = time.monotonic()
        response = ""
        distance_m: float | None = None
        error = ""
        try:
            if not host.strip():
                raise ValueError("WIRIS control host is empty")
            command = build_wiris_command(
                host=host,
                port=port,
                command=WIRIS_LRF_COMMAND,
                timeout_s=timeout_s,
                terminator=LINE_TERMINATOR,
            )
            response = send_tcp_line(command)
            distance_m = parse_lrf_distance(response)
            if not response:
                error = "empty response"
            elif distance_m is None:
                error = response.strip() or "distance parse failed"
        except Exception as exc:
            error = str(exc) or exc.__class__.__name__
        observed_s = time.monotonic()
        return {
            "side": label,
            "host": host,
            "port": port,
            "command": WIRIS_LRF_COMMAND,
            "response": response,
            "distance_m": distance_m,
            "error": error,
            "attempted_s": started_s,
            "observed_s": observed_s,
            "duration_s": max(0.0, observed_s - started_s),
            "observed_unix_s": time.time(),
        }

    def poll_once(self) -> tuple[dict[str, object], ...]:
        """Run one parallel left/right poll; exposed for deterministic tests."""

        results: dict[str, dict[str, object]] = {}

        def poll_one(info: tuple[str, str, int, float]) -> None:
            label, host, port, timeout_s = info
            results[label] = self._poll_one(label, host, port, timeout_s)

        threads = [
            threading.Thread(target=poll_one, args=(info,), name=f"wiris-lrf-{info[0]}")
            for info in self.target_infos
        ]
        for thread in threads:
            thread.start()
        for thread, info in zip(threads, self.target_infos):
            thread.join(timeout=max(1.0, info[3]) + 0.25)
            if thread.is_alive():
                results.setdefault(
                    info[0],
                    {
                        "side": info[0],
                        "host": info[1],
                        "port": info[2],
                        "command": WIRIS_LRF_COMMAND,
                        "response": "",
                        "distance_m": None,
                        "error": "poll thread timeout",
                        "attempted_s": time.monotonic(),
                        "observed_s": time.monotonic(),
                        "duration_s": 0.0,
                        "observed_unix_s": time.time(),
                    },
                )
        return tuple(results[label] for label, *_ in self.target_infos if label in results)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            self.sampled.emit({"samples": self.poll_once(), "polled_s": time.monotonic()})
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


class WirisZoomWorker(QtCore.QObject):
    """Send WIRIS zoom commands without blocking the UI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        targets: Sequence[tuple[str, str, int]],
        *,
        template: str,
        level: int | None,
        timeout_s: float,
    ) -> None:
        super().__init__()
        self.targets = tuple((str(label), str(host), int(port)) for label, host, port in targets)
        self.template = str(template)
        self.level = None if level is None else int(level)
        self.timeout_s = max(0.1, float(timeout_s))

    @QtCore.pyqtSlot()
    def run(self) -> None:
        statuses: list[dict[str, object]] = []
        try:
            for label, host, port in self.targets:
                try:
                    command = build_zoom_command(
                        host=host,
                        port=port,
                        template=self.template,
                        level=self.level,
                        timeout_s=self.timeout_s,
                    )
                    response = send_tcp_line(command)
                except Exception as exc:
                    statuses.append({"label": label, "state": "failed", "detail": str(exc)})
                else:
                    statuses.append(
                        {
                            "label": label,
                            "state": "sent",
                            "command": command.command,
                            "response": response,
                        }
                    )
            self.completed.emit(statuses)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()

class RtspStreamCheckWorker(QtCore.QObject):
    checked = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        urls: dict[str, str],
        *,
        latency_ms: int,
        decoder: str,
        backend: str,
        transport: str,
        timeout_s: float = 4.0,
    ) -> None:
        super().__init__()
        self.urls = urls
        self.latency_ms = latency_ms
        self.decoder = normalize_decoder_mode(decoder)
        self.backend = normalize_stream_backend(backend)
        self.transport = normalize_rtsp_transport(transport)
        self.timeout_s = timeout_s

    @QtCore.pyqtSlot()
    def run(self) -> None:
        if self.backend in {STREAM_BACKEND_OPENCV, STREAM_BACKEND_OPENCV_AUTO}:
            self._run_opencv()
            return
        if self.backend == STREAM_BACKEND_PYAV:
            self._run_pyav()
            return
        self._run_gstreamer()

    def _run_opencv(self) -> None:
        try:
            import cv2
        except Exception as exc:
            self.failed.emit(f"OpenCV unavailable: {exc}")
            self.finished.emit()
            return
        results = {}
        for label, url in self.urls.items():
            results[label] = self._check_one_opencv(cv2, url)
        self.checked.emit(results)
        self.finished.emit()

    def _check_one_opencv(self, cv2, url: str) -> dict[str, object]:
        backend_name = "opencv_auto" if self.backend == STREAM_BACKEND_OPENCV_AUTO else "opencv"
        api_preference = cv2.CAP_ANY if self.backend == STREAM_BACKEND_OPENCV_AUTO else cv2.CAP_FFMPEG
        started_s = time.monotonic()
        try:
            host, port = rtsp_host_port(url)
        except Exception as exc:
            return {"port_open": False, "frame_ok": False, "error": f"invalid url: {exc}", "backend": backend_name}
        if not check_tcp_port(host, port, timeout=min(self.timeout_s, 1.0)):
            return {"port_open": False, "frame_ok": False, "error": f"{host}:{port} closed", "backend": backend_name}
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            f"rtsp_transport;{self.transport}|stimeout;2000000|max_delay;500000"
        )
        cap = cv2.VideoCapture(url, api_preference)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        try:
            while time.monotonic() - started_s < self.timeout_s:
                ok, frame = cap.read()
                if ok and frame is not None:
                    height, width = frame.shape[:2]
                    return {
                        "port_open": True,
                        "frame_ok": True,
                        "width": width,
                        "height": height,
                        "elapsed_s": time.monotonic() - started_s,
                        "decoder": "opencv-auto" if self.backend == STREAM_BACKEND_OPENCV_AUTO else "opencv-ffmpeg",
                        "backend": backend_name,
                    }
                QtCore.QThread.msleep(20)
        finally:
            cap.release()
        return {
            "port_open": True,
            "frame_ok": False,
            "error": f"no frame within {self.timeout_s:.1f}s",
            "backend": backend_name,
        }

    def _run_pyav(self) -> None:
        try:
            import av
        except Exception as exc:
            self.failed.emit(f"PyAV unavailable: {exc}")
            self.finished.emit()
            return
        results = {}
        for label, url in self.urls.items():
            results[label] = self._check_one_pyav(av, url)
        self.checked.emit(results)
        self.finished.emit()

    def _check_one_pyav(self, av, url: str) -> dict[str, object]:
        started_s = time.monotonic()
        try:
            host, port = rtsp_host_port(url)
        except Exception as exc:
            return {"port_open": False, "frame_ok": False, "error": f"invalid url: {exc}", "backend": "pyav"}
        if not check_tcp_port(host, port, timeout=min(self.timeout_s, 1.0)):
            return {"port_open": False, "frame_ok": False, "error": f"{host}:{port} closed", "backend": "pyav"}
        options = {
            "rtsp_transport": self.transport,
            "stimeout": "2000000",
            "max_delay": "500000",
            "fflags": "nobuffer",
            "flags": "low_delay",
        }
        container = None
        try:
            container = av.open(url, mode="r", options=options)
            video_stream = next((stream for stream in container.streams if stream.type == "video"), None)
            if video_stream is None:
                return {"port_open": True, "frame_ok": False, "error": "no video stream", "backend": "pyav"}
            for packet in container.demux(video_stream):
                for frame in packet.decode():
                    pts_s = None
                    if frame.pts is not None and frame.time_base is not None:
                        pts_s = float(frame.pts * frame.time_base)
                    return {
                        "url": url,
                        "port_open": True,
                        "frame_ok": True,
                        "width": int(frame.width),
                        "height": int(frame.height),
                        "pts_s": pts_s,
                        "elapsed_s": time.monotonic() - started_s,
                        "decoder": "pyav",
                        "backend": "pyav",
                    }
                if time.monotonic() - started_s >= self.timeout_s:
                    break
            return {
                "url": url,
                "port_open": True,
                "frame_ok": False,
                "error": f"no frame within {self.timeout_s:.1f}s",
                "decoder": "pyav",
                "backend": "pyav",
            }
        except Exception as exc:
            return {"url": url, "port_open": True, "frame_ok": False, "error": str(exc), "decoder": "pyav", "backend": "pyav"}
        finally:
            if container is not None:
                container.close()

    def _run_gstreamer(self) -> None:
        try:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
        except Exception as exc:
            self.failed.emit(f"GStreamer unavailable: {exc}")
            self.finished.emit()
            return

        Gst.init(None)
        results = {}
        for label, url in self.urls.items():
            results[label] = self._check_one(Gst, url)
        self.checked.emit(results)
        self.finished.emit()

    def _check_one(self, Gst, url: str) -> dict[str, object]:
        try:
            host, port = rtsp_host_port(url)
            port_open = check_tcp_port(host, port, timeout=1.0)
        except Exception as exc:
            return {"url": url, "port_open": False, "frame_ok": False, "error": str(exc)}
        if not port_open:
            return {"url": url, "port_open": False, "frame_ok": False, "error": f"{host}:{port} closed"}

        last_error = ""
        candidates = decoder_candidates(self.decoder)
        for decoder in candidates:
            pipeline = None
            try:
                pipeline_text = build_gst_pipeline_text(
                    url,
                    latency_ms=self.latency_ms,
                    decoder=decoder,
                    transport=self.transport,
                )
                pipeline = Gst.parse_launch(pipeline_text)
                sink = pipeline.get_by_name("sink")
                bus = pipeline.get_bus()
                if sink is None or bus is None:
                    raise RuntimeError("pipeline missing appsink or bus")
                state_result = pipeline.set_state(Gst.State.PLAYING)
                if state_result == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError(f"{decoder} failed to enter PLAYING state")
                return self._wait_for_rtsp_frame(Gst, pipeline, sink, bus, url, decoder)
            except Exception as exc:
                last_error = str(exc)
                if pipeline is not None:
                    pipeline.set_state(Gst.State.NULL)
                if self.decoder == DECODER_AUTO and decoder != candidates[-1]:
                    continue
                break
        return {"url": url, "port_open": True, "frame_ok": False, "error": last_error or "decoder unavailable"}

    def _wait_for_rtsp_frame(self, Gst, pipeline, sink, bus, url: str, decoder: str) -> dict[str, object]:
        try:
            start_s = time.monotonic()
            while time.monotonic() - start_s < self.timeout_s:
                message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if message is not None:
                    if message.type == Gst.MessageType.ERROR:
                        error, debug = message.parse_error()
                        detail = error.message if debug is None else f"{error.message} ({debug})"
                    else:
                        detail = "EOS before frame"
                    return {"url": url, "port_open": True, "frame_ok": False, "error": detail, "decoder": decoder}

                sample = sink.emit("try-pull-sample", 200_000_000)
                if sample is None:
                    continue
                buffer = sample.get_buffer()
                pts_s = None if buffer.pts == Gst.CLOCK_TIME_NONE else buffer.pts / 1e9
                caps = sample.get_caps().get_structure(0)
                return {
                    "url": url,
                    "port_open": True,
                    "frame_ok": True,
                    "width": int(caps.get_value("width")),
                    "height": int(caps.get_value("height")),
                    "pts_s": pts_s,
                    "elapsed_s": time.monotonic() - start_s,
                    "decoder": decoder,
                }
            return {
                "url": url,
                "port_open": True,
                "frame_ok": False,
                "error": f"no frame within {self.timeout_s:.1f}s",
                "decoder": decoder,
            }
        except Exception as exc:
            return {"url": url, "port_open": True, "frame_ok": False, "error": str(exc), "decoder": decoder}
        finally:
            pipeline.set_state(Gst.State.NULL)
