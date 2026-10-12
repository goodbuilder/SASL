"""Camera discovery boundary workers."""
from __future__ import annotations

from PyQt5 import (
    QtCore,
)
from Workstation.workstation_core.network import (
    check_tcp_port,
    discover_host_interface_candidates,
    discover_rtsp_candidates,
    discover_tcp_port_candidates,
    discover_tcp_port_range_candidates,
    tcp_port_scan_sequence,
)
from Workstation.workstation_core.wiris_zoom import (
    DEFAULT_ZOOM_CONTROL_PORT,
    NUL_TERMINATOR,
    build_wiris_command,
    send_tcp_line,
)
from collections.abc import (
    Sequence,
)

def _control_candidate_value(candidate: object, key: str, default: object | None = None) -> object | None:
    if isinstance(candidate, dict):
        return candidate.get(key, default)
    return getattr(candidate, key, default)

def _preferred_wiris_control_ports(control_port: int | None) -> tuple[int, ...]:
    ports: list[int] = []
    for port in (control_port, DEFAULT_ZOOM_CONTROL_PORT):
        try:
            normalized = int(port or DEFAULT_ZOOM_CONTROL_PORT)
        except (TypeError, ValueError):
            continue
        if 1 <= normalized <= 65535 and normalized not in ports:
            ports.append(normalized)
    return tuple(ports or (DEFAULT_ZOOM_CONTROL_PORT,))

def _full_wiris_control_port_sequence(control_port: int | None) -> tuple[int, ...]:
    return tcp_port_scan_sequence(*_preferred_wiris_control_ports(control_port))

def _merge_tcp_control_candidates(*groups: list[object]) -> list[object]:
    merged: list[object] = []
    seen: set[tuple[str, int]] = set()
    for group in groups:
        for candidate in group:
            host = str(_control_candidate_value(candidate, "host", "") or "")
            try:
                port = int(_control_candidate_value(candidate, "port", 0) or 0)
            except (TypeError, ValueError):
                continue
            key = (host, port)
            if not host or key in seen:
                continue
            seen.add(key)
            merged.append(candidate)
    return merged

def _verify_wiris_control_candidates(candidates: list[object]) -> list[dict[str, object]]:
    verified: list[dict[str, object]] = []
    for candidate in candidates:
        host = str(_control_candidate_value(candidate, "host", "") or "")
        try:
            port = int(_control_candidate_value(candidate, "port", 0) or 0)
        except (TypeError, ValueError):
            continue
        interface = _control_candidate_value(candidate, "interface")
        try:
            command = build_wiris_command(
                host=host,
                port=port,
                command="HIWS",
                timeout_s=0.8,
                terminator=NUL_TERMINATOR,
            )
            response = send_tcp_line(command)
        except Exception as exc:
            verified.append(
                {
                    "host": host,
                    "port": port,
                    "interface": interface,
                    "ok": False,
                    "response": str(exc),
                }
            )
            continue
        verified.append(
            {
                "host": host,
                "port": port,
                "interface": interface,
                "ok": response.strip().upper() == "OK",
                "response": response,
            }
        )
    return verified

class CameraAutoDetectWorker(QtCore.QObject):
    detected = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        mode: str,
        control_port: int | None = None,
        host_hints: tuple[str, ...] = (),
        target_side: str | None = None,
        full_control_scan: bool = True,
    ) -> None:
        super().__init__()
        self.mode = mode
        self.control_port = control_port
        self.host_hints = tuple(host_hints)
        self.target_side = target_side
        self.full_control_scan = bool(full_control_scan)

    def _discover_wiris_control(self, arp_scan_reports: list[object]) -> tuple[list[object], list[dict[str, object]]]:
        preferred_ports = _preferred_wiris_control_ports(self.control_port)
        preferred_candidates = discover_tcp_port_range_candidates(
            preferred_ports,
            timeout=0.2,
            max_hosts_per_network=0,
            fallback_scan_if_empty=False,
            stop_after=None,
            arp_scan_reports=arp_scan_reports,
            host_hints=self.host_hints,
        )
        preferred_verified = _verify_wiris_control_candidates(preferred_candidates)
        if len([candidate for candidate in preferred_verified if candidate.get("ok")]) >= 2:
            return list(preferred_candidates), preferred_verified

        full_scan_candidates: list[object] = []
        if self.full_control_scan:
            full_scan_candidates = discover_tcp_port_range_candidates(
                _full_wiris_control_port_sequence(self.control_port),
                timeout=0.08,
                max_hosts_per_network=0,
                fallback_scan_if_empty=False,
                stop_after=None,
                arp_scan_reports=None,
                host_hints=self.host_hints,
            )

        fallback_candidates: list[object] = []
        if len([candidate for candidate in preferred_verified if candidate.get("ok")]) < 2:
            for port in preferred_ports:
                fallback_candidates = _merge_tcp_control_candidates(
                    fallback_candidates,
                    discover_tcp_port_candidates(
                        int(port),
                        timeout=0.2,
                        max_hosts_per_network=0,
                        fallback_scan_if_empty=True,
                        stop_after=None,
                        arp_scan_reports=None,
                        host_hints=self.host_hints,
                    ),
                )

        control_candidates = _merge_tcp_control_candidates(
            list(preferred_candidates),
            list(full_scan_candidates),
            fallback_candidates,
        )
        return control_candidates, _verify_wiris_control_candidates(control_candidates)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            arp_scan_reports = []
            host_interfaces = discover_host_interface_candidates()
            if self.mode == "auto_connect":
                control, verified = self._discover_wiris_control(arp_scan_reports)
                result = {
                    "mode": "auto_connect",
                    "full_control_scan": self.full_control_scan,
                    "host_interfaces": host_interfaces,
                    "control": control,
                    "verified_control": verified,
                    "arp_scan": arp_scan_reports,
                }
            elif self.mode == "control":
                control, verified = self._discover_wiris_control(arp_scan_reports)
                result = {
                    "mode": "control",
                    "full_control_scan": self.full_control_scan,
                    "target_side": self.target_side,
                    "host_interfaces": host_interfaces,
                    "control": control,
                    "verified_control": verified,
                    "arp_scan": arp_scan_reports,
                }
            else:
                result = {
                    "mode": "rtsp",
                    "host_interfaces": host_interfaces,
                    "rtsp": discover_rtsp_candidates(
                        timeout=0.2,
                        max_hosts_per_network=0,
                        fallback_scan_if_empty=True,
                        stop_after=None,
                        arp_scan_reports=arp_scan_reports,
                        host_hints=self.host_hints,
                    ),
                    "arp_scan": arp_scan_reports,
                }
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.detected.emit(result)
        finally:
            self.finished.emit()

class WirisControlCheckWorker(QtCore.QObject):
    """Probe WIRIS control ports without blocking the Qt event loop."""

    checked = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, targets: Sequence[tuple[str, str, int]], *, timeout_s: float) -> None:
        super().__init__()
        self.targets = tuple((str(label), str(host), int(port)) for label, host, port in targets)
        self.timeout_s = max(0.1, float(timeout_s))

    @QtCore.pyqtSlot()
    def run(self) -> None:
        results: list[dict[str, object]] = []
        try:
            for label, host, port in self.targets:
                if not host.strip():
                    results.append({"label": label, "host": host, "port": port, "state": "missing-host"})
                    continue
                try:
                    is_open = bool(check_tcp_port(host.strip(), port, timeout=self.timeout_s))
                except Exception as exc:
                    results.append(
                        {
                            "label": label,
                            "host": host,
                            "port": port,
                            "state": "error",
                            "error": str(exc),
                        }
                    )
                else:
                    results.append(
                        {
                            "label": label,
                            "host": host,
                            "port": port,
                            "state": "control-open" if is_open else "control-closed",
                        }
                    )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.checked.emit(results)
        finally:
            self.finished.emit()
