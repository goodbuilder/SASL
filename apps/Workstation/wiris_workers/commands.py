"""WIRIS commands workers with their original state, Qt signals and I/O lifetime.

Left/right source identity, image bytes, metadata and source clocks retain
their producer contracts; no NED/FRD/RDF reinterpretation is performed here.
"""
from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from PyQt5 import QtCore
from Workstation.workstation_core.wiris_zoom import LINE_TERMINATOR, build_wiris_command, build_zoom_command, WirisControlBusyError
from . import io as _wiris_io
from . import policy as _wire_policy


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
                    _wire_policy._wait_for_peer_or_continue(
                        barrier,
                        timeout_s=max(1.0, timeout_s),
                    )
                send_times[label] = time.perf_counter()
                response = _wiris_io.send_tcp_line(command)
            except Exception as exc:
                _wire_policy._abort_barrier(barrier)
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


class WirisImageFormatWorker(QtCore.QObject):
    """Send an ordered list of SDK commands to both cameras.

    Used for the Cameras tab's Image Format group: a status refresh sends
    the four Get commands (``GIVI``/``GIRJ``/``GISR``/``GIHR``) plus
    ``GILC``, a checkbox toggle sends one ``Sxxx TRUE``/``Sxxx FALSE``, and
    the OZ-only preset sends four Set commands in one round. Also reused by
    the calibration OZ capture-only trigger to force a known-good format
    state before every capture.
    """

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()
    progress = QtCore.pyqtSignal(object)

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
        *,
        commands: Sequence[str],
    ) -> None:
        super().__init__()
        self.target_infos = tuple(
            (str(label), str(host), int(port), max(0.1, float(timeout_s)))
            for label, host, port, timeout_s in target_infos
        )
        self.commands = tuple(str(command) for command in commands)

    def _send_all(self, host: str, port: int, timeout_s: float) -> dict[str, str]:
        responses: dict[str, str] = {}
        label = ""
        for candidate_label, candidate_host, candidate_port, _candidate_timeout in self.target_infos:
            if candidate_host == host and candidate_port == port:
                label = candidate_label
                break
        for command_text in self.commands:
            self.progress.emit(
                {
                    "source": "calibration",
                    "phase": "command",
                    "side": label,
                    "command": command_text,
                    "state": "sending",
                }
            )
            try:
                responses[command_text] = _wiris_io.send_tcp_line(
                    build_wiris_command(host=host, port=port, command=command_text, timeout_s=timeout_s)
                )
            except Exception as exc:
                responses[command_text] = f"ERROR: {exc}"
            self.progress.emit(
                {
                    "source": "calibration",
                    "phase": "response",
                    "side": label,
                    "command": command_text,
                    "state": "received" if not responses[command_text].startswith("ERROR:") else "failed",
                    "response": responses[command_text] or "<no response>",
                }
            )
        return responses

    @QtCore.pyqtSlot()
    def run(self) -> None:
        results: dict[str, dict[str, str]] = {}
        try:
            for label, host, port, timeout_s in self.target_infos:
                results[label] = self._send_all(host, port, timeout_s)
            self.completed.emit(results)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


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
                command=_wire_policy.WIRIS_LRF_COMMAND,
                timeout_s=timeout_s,
                terminator=LINE_TERMINATOR,
            )
            response = _wiris_io.send_tcp_line(command)
            distance_m = _wire_policy.parse_lrf_distance(response)
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
            "command": _wire_policy.WIRIS_LRF_COMMAND,
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
                        "command": _wire_policy.WIRIS_LRF_COMMAND,
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
                    response = _wiris_io.send_tcp_line(command)
                except WirisControlBusyError as exc:
                    statuses.append(
                        {"label": label, "state": "busy", "detail": str(exc)}
                    )
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

# Keep the named legacy class identity; implementation ownership is here.
WirisCommandWorker.__module__ = "Workstation.wiris_runtime_workers"
WirisImageFormatWorker.__module__ = "Workstation.wiris_runtime_workers"
WirisLrfPollWorker.__module__ = "Workstation.wiris_runtime_workers"
WirisZoomWorker.__module__ = "Workstation.wiris_runtime_workers"
