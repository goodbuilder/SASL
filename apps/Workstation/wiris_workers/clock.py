"""WIRIS clock workers with their original state, Qt signals and I/O lifetime.

Left/right source identity, image bytes, metadata and source clocks retain
their producer contracts; no NED/FRD/RDF reinterpretation is performed here.
"""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Mapping, Sequence
from statistics import median
from PyQt5 import QtCore
from Workstation.workstation_core.wiris_zoom import LINE_TERMINATOR, build_wiris_command
from . import io as _wiris_io
from . import policy as _wire_policy


class WirisClockCommandWorker(QtCore.QObject):
    """Send WIRIS STUS/GDTI commands without blocking the UI thread.

    WIRIS's STUS command does not account for command transfer/parsing time.
    Every STUS round first requires bounded, exact-OK HIWS probes from all
    selected cameras.  The initial round uses the robust RTT median only as a
    target seed.  A correction round uses fresh Left/Right differential RTCP
    Sender Report feedback while retaining HIWS as a no-mutation reachability
    preflight.  Absolute SR-PC remains diagnostic-only.
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
        stus_compensation_by_camera: Mapping[str, float] | None = None,
        stus_feedback_offset_by_camera: Mapping[str, float] | None = None,
        stus_attempt_index: int = 1,
        operation_id: str = "",
        operation_deadline_monotonic_s: float | None = None,
    ) -> None:
        super().__init__()
        self.target_infos = tuple(
            (str(label), str(host), int(port), max(0.1, float(timeout_s)))
            for label, host, port, timeout_s in target_infos
        )
        self.command_text = str(command_text).strip().upper()
        self.shared_utc = bool(shared_utc)
        self.stus_compensation_by_camera = dict(stus_compensation_by_camera or {})
        self.stus_feedback_offset_by_camera = dict(stus_feedback_offset_by_camera or {})
        self.stus_attempt_index = max(1, int(stus_attempt_index))
        self.operation_id = str(operation_id or "")
        try:
            deadline = float(operation_deadline_monotonic_s)
        except (TypeError, ValueError):
            deadline = None
        self.operation_deadline_monotonic_s = (
            deadline if deadline is not None and math.isfinite(deadline) else None
        )
        self._cancel_event = threading.Event()

    def cancel(self) -> None:
        """Thread-safe direct cancellation for shutdown and stream-stop paths."""

        self._cancel_event.set()

    def _raise_if_cancelled(self) -> None:
        if self._cancel_event.is_set():
            raise InterruptedError("STUS operation cancelled")

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
            or value > _wire_policy.WIRIS_STUS_MAX_VALID_RTT_S
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
        return value / 2.0 + _wire_policy.WIRIS_STUS_PARSE_MARGIN_S

    @staticmethod
    def _normalized_explicit_stus_compensation_s(value: object) -> float | None:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        if (
            not math.isfinite(parsed)
            or abs(parsed) > _wire_policy.WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S
        ):
            return None
        return parsed

    def _remaining_io_timeout_s(self, requested_timeout_s: float) -> float:
        requested_timeout_s = max(0.1, float(requested_timeout_s))
        deadline = self.operation_deadline_monotonic_s
        if deadline is None:
            return requested_timeout_s
        remaining_s = deadline - time.monotonic()
        if remaining_s < 0.01:
            raise TimeoutError(
                f"STUS operation exceeded {_wire_policy.WIRIS_STUS_MAX_OPERATION_S:.0f}s deadline"
            )
        return min(requested_timeout_s, remaining_s)

    def _operation_deadline_exceeded(self) -> bool:
        deadline = self.operation_deadline_monotonic_s
        return deadline is not None and time.monotonic() >= deadline

    def _stus_response_timeout_s(self) -> float:
        """Bound only the clock-mutation ACK wait by the overall operation."""

        return self._remaining_io_timeout_s(_wire_policy.WIRIS_STUS_RESPONSE_TIMEOUT_S)

    @staticmethod
    def _is_ok_ack(response: object) -> bool:
        """Accept only the WIRIS command ACK that the controller can trust."""

        return str(response or "").replace("\x00", "").strip().upper() == "OK"

    @staticmethod
    def _empty_rtt_probe_result() -> dict[str, object]:
        return {
            "command": _wire_policy.WIRIS_STUS_RTT_PROBE_COMMAND,
            "samples_s": [],
            "median_s": None,
            "errors": [],
            "warnings": [],
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
        warnings: list[str] = []
        for _index in range(_wire_policy.WIRIS_STUS_RTT_PROBE_COUNT):
            self._raise_if_cancelled()
            started_mono_s = time.perf_counter()
            try:
                command = build_wiris_command(
                    host=host,
                    port=port,
                    command=_wire_policy.WIRIS_STUS_RTT_PROBE_COMMAND,
                    timeout_s=self._remaining_io_timeout_s(timeout_s),
                    terminator=LINE_TERMINATOR,
                )
                response = _wiris_io.send_tcp_line(command)
                self._raise_if_cancelled()
            except InterruptedError:
                raise
            except Exception as exc:
                errors.append(str(exc))
                continue
            elapsed_s = max(0.0, time.perf_counter() - started_mono_s)
            if not str(response or "").strip():
                errors.append("HIWS empty response")
                continue
            if not self._is_ok_ack(response):
                errors.append(f"HIWS non-OK response: {str(response).strip()}")
                continue
            if elapsed_s > _wire_policy.WIRIS_STUS_MAX_VALID_RTT_S:
                errors.append(
                    f"HIWS RTT {elapsed_s * 1000.0:.1f}ms exceeds "
                    f"{_wire_policy.WIRIS_STUS_MAX_VALID_RTT_S * 1000.0:.0f}ms limit"
                )
                continue
            samples.append(elapsed_s)
            if elapsed_s > _wire_policy.WIRIS_STUS_HIGH_RTT_WARNING_S:
                warnings.append(
                    f"HIWS RTT {elapsed_s * 1000.0:.1f}ms is high but within "
                    f"{_wire_policy.WIRIS_STUS_MAX_VALID_RTT_S * 1000.0:.0f}ms bound"
                )
            if len(samples) >= _wire_policy.WIRIS_STUS_MIN_VALID_RTT_PROBES:
                break
        probe["samples_s"] = samples
        if len(samples) >= _wire_policy.WIRIS_STUS_MIN_VALID_RTT_PROBES:
            probe["median_s"] = median(samples)
        else:
            errors.append(
                f"HIWS valid samples {len(samples)}/{_wire_policy.WIRIS_STUS_MIN_VALID_RTT_PROBES}"
            )
        probe["errors"] = errors
        probe["warnings"] = warnings
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
            one_way_delay_s: float | None = 0.0
            parse_margin_s = 0.0
            compensation_s = 0.0
            feedback_offset_s: float | None = None
            compensation_source = "none"
            tcp_start_utc_s: float | None = None
            response_timeout_s: float | None = None
            try:
                self._raise_if_cancelled()
                if self.command_text == "STUS":
                    probe = dict(rtt_probe_by_camera.get(label) or self._empty_rtt_probe_result())
                    if label in self.stus_compensation_by_camera:
                        explicit_compensation_s = self._normalized_explicit_stus_compensation_s(
                            self.stus_compensation_by_camera.get(label)
                        )
                        if explicit_compensation_s is None:
                            raise ValueError(
                                f"{label} closed-loop STUS compensation exceeds "
                                f"±{_wire_policy.WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S:.3f}s limit"
                            )
                        compensation_s = explicit_compensation_s
                        one_way_delay_s = None
                        feedback_offset_s = self._normalized_explicit_stus_compensation_s(
                            self.stus_feedback_offset_by_camera.get(label)
                        )
                        compensation_source = "rtcp-relative-closed-loop"
                        probe["compensation_note"] = (
                            "closed-loop uses fresh Left/Right relative RTCP feedback"
                        )
                    else:
                        rtt_median_s = probe.get("median_s")
                        valid_rtt_s = self._normalized_stus_rtt_s(rtt_median_s)
                        one_way_delay_s = self._stus_one_way_delay_s(valid_rtt_s)
                        if valid_rtt_s is not None:
                            parse_margin_s = _wire_policy.WIRIS_STUS_PARSE_MARGIN_S
                        compensation_s = float(one_way_delay_s) + parse_margin_s
                        compensation_source = "control-rtt-half-plus-parse-margin"
                rtt_probe_by_camera[label] = probe
                if barrier is not None:
                    _wire_policy._wait_for_peer_or_continue(
                        barrier,
                        timeout_s=max(1.0, timeout_s),
                    )
                    tcp_start_utc_s = time.time()
                    command_time_s = float(shared_command_time.get("unix", tcp_start_utc_s))
                else:
                    # Capture the PC UTC immediately before the STUS socket
                    # call.  The target is camera-specific because each
                    # control connection has a different measured RTT.
                    tcp_start_utc_s = time.time()
                    command_time_s = tcp_start_utc_s + compensation_s
                command_text = self._stus_command(command_time_s) if self.command_text == "STUS" else self.command_text
                response_timeout_s = (
                    self._stus_response_timeout_s()
                    if self.command_text == "STUS"
                    else self._remaining_io_timeout_s(timeout_s)
                )
                command = build_wiris_command(
                    host=host,
                    port=port,
                    command=command_text,
                    timeout_s=response_timeout_s,
                    terminator=LINE_TERMINATOR,
                )
                command_times[label] = command_time_s
                tcp_start_utc_times[label] = tcp_start_utc_s
                tcp_start_times[label] = time.perf_counter()
                self._raise_if_cancelled()
                response = _wiris_io.send_tcp_line(command)
            except Exception as exc:
                _wire_policy._abort_barrier(barrier)
                results[label] = {
                    "response": f"실패: {exc}",
                    "ack_ok": False,
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
                    "stus_rtt_probe_warnings": list(probe.get("warnings") or []),
                    "stus_one_way_delay_s": one_way_delay_s,
                    "stus_parse_margin_s": parse_margin_s,
                    "stus_compensation_s": compensation_s,
                    "stus_compensation_source": compensation_source,
                    "stus_feedback_offset_s": feedback_offset_s,
                    "stus_requested_utc_s": command_times.get(label),
                    "tcp_start_utc_s": tcp_start_utc_s,
                    "stus_response_timeout_s": response_timeout_s,
                }
                return
            terminal_after_response = ""
            if self._cancel_event.is_set():
                terminal_after_response = "operation-cancelled-after-response"
            elif self._operation_deadline_exceeded():
                terminal_after_response = "operation-deadline-exceeded-after-response"
            results[label] = {
                "response": response or "<no response>",
                "ack_ok": self._is_ok_ack(response),
                "query_started_utc_s": started_utc_s,
                "query_finished_utc_s": time.time(),
                "command": command.command,
                "host": host,
                "port": port,
                "stus_rtt_probe_command": probe.get("command"),
                "stus_rtt_samples_s": list(probe.get("samples_s") or []),
                "stus_rtt_median_s": probe.get("median_s"),
                "stus_rtt_probe_errors": list(probe.get("errors") or []),
                "stus_rtt_probe_warnings": list(probe.get("warnings") or []),
                "stus_one_way_delay_s": one_way_delay_s,
                "stus_parse_margin_s": parse_margin_s,
                "stus_compensation_s": compensation_s,
                "stus_compensation_source": compensation_source,
                "stus_feedback_offset_s": feedback_offset_s,
                "stus_requested_utc_s": command_times.get(label),
                "tcp_start_utc_s": tcp_start_utc_s,
                "stus_response_timeout_s": response_timeout_s,
                "terminal_after_response": terminal_after_response,
            }

        try:
            if self.command_text == "STUS":
                if not self.target_infos:
                    raise ValueError("STUS requires at least one camera target")
                # STUS changes external device state.  The accepted dual-camera
                # transaction is deliberately asymmetric: Left must return an
                # explicit OK before Right is touched.  Sorting here makes the
                # safety contract independent of UI/list construction order.
                ordered_targets = sorted(
                    self.target_infos,
                    key=lambda row: 0 if row[0] == "left" else 1 if row[0] == "right" else 2,
                )
                # HIWS is observation, not mutation. Measure both independent
                # control channels concurrently, then retain deterministic
                # Left-ACK-before-Right ordering for STUS itself.
                probe_lock = threading.Lock()

                def measure_target(
                    label: str,
                    host: str,
                    port: int,
                    timeout_s: float,
                ) -> None:
                    try:
                        probe = self._measure_control_rtt(
                            host=host,
                            port=port,
                            timeout_s=timeout_s,
                        )
                    except InterruptedError:
                        return
                    except Exception as exc:  # noqa: BLE001 - external probe boundary.
                        probe = self._empty_rtt_probe_result()
                        probe["errors"] = [f"HIWS probe failed: {exc}"]
                    with probe_lock:
                        rtt_probe_by_camera[label] = probe

                probe_threads = [
                    threading.Thread(
                        target=measure_target,
                        args=(label, host, port, timeout_s),
                        name=f"wiris-stus-hiws-{label}",
                    )
                    for label, host, port, timeout_s in ordered_targets
                ]
                for thread in probe_threads:
                    thread.start()
                for thread in probe_threads:
                    thread.join()
                self._raise_if_cancelled()
                failed_preflight_labels = [
                    label
                    for label, _host, _port, _timeout_s in ordered_targets
                    if self._normalized_stus_rtt_s(
                        dict(rtt_probe_by_camera.get(label) or {}).get("median_s")
                    )
                    is None
                ]
                if failed_preflight_labels:
                    failure_text = ",".join(failed_preflight_labels)
                    for label, host, port, _timeout_s in ordered_targets:
                        probe = dict(
                            rtt_probe_by_camera.get(label)
                            or self._empty_rtt_probe_result()
                        )
                        results[label] = {
                            "response": f"SKIPPED: HIWS preflight failed ({failure_text})",
                            "ack_ok": False,
                            "skipped": True,
                            "skipped_due_to": "stus-hiws-preflight-failed",
                            "query_started_utc_s": None,
                            "query_finished_utc_s": None,
                            "command": "STUS (not sent)",
                            "host": host,
                            "port": port,
                            "stus_rtt_probe_command": probe.get("command"),
                            "stus_rtt_samples_s": list(probe.get("samples_s") or []),
                            "stus_rtt_median_s": probe.get("median_s"),
                            "stus_rtt_probe_errors": list(probe.get("errors") or []),
                            "stus_rtt_probe_warnings": list(probe.get("warnings") or []),
                            "stus_one_way_delay_s": None,
                            "stus_parse_margin_s": None,
                            "stus_compensation_s": None,
                            "stus_compensation_source": "skipped",
                            "stus_feedback_offset_s": self.stus_feedback_offset_by_camera.get(label),
                            "stus_requested_utc_s": None,
                            "tcp_start_utc_s": None,
                            "stus_response_timeout_s": None,
                        }
                left_ack_failed = False
                left_skip_reason = "left-stus-ack-not-ok"
                for label, host, port, timeout_s in (
                    () if failed_preflight_labels else ordered_targets
                ):
                    right_skip_reason = ""
                    if label == "right":
                        if left_ack_failed:
                            right_skip_reason = left_skip_reason
                        elif self._cancel_event.is_set():
                            right_skip_reason = "operation-cancelled-before-right"
                        elif self._operation_deadline_exceeded():
                            right_skip_reason = "operation-deadline-exceeded-before-right"
                    if right_skip_reason:
                        probe = self._empty_rtt_probe_result()
                        rtt_probe_by_camera[label] = probe
                        results[label] = {
                            "response": f"SKIPPED: {right_skip_reason}",
                            "ack_ok": False,
                            "skipped": True,
                            "skipped_due_to": right_skip_reason,
                            "query_started_utc_s": None,
                            "query_finished_utc_s": None,
                            "command": "STUS (not sent)",
                            "host": host,
                            "port": port,
                            "stus_rtt_probe_command": probe.get("command"),
                            "stus_rtt_samples_s": [],
                            "stus_rtt_median_s": None,
                            "stus_rtt_probe_errors": [],
                            "stus_rtt_probe_warnings": [],
                            "stus_one_way_delay_s": None,
                            "stus_parse_margin_s": None,
                            "stus_compensation_s": None,
                            "stus_compensation_source": "skipped",
                            "stus_feedback_offset_s": self.stus_feedback_offset_by_camera.get(label),
                            "stus_requested_utc_s": None,
                            "tcp_start_utc_s": None,
                            "stus_response_timeout_s": None,
                        }
                        continue
                    self._raise_if_cancelled()
                    send_one(label, host, port, timeout_s)
                    if label == "left":
                        left_result = dict(results.get(label) or {})
                        terminal_reason = str(
                            left_result.get("terminal_after_response") or ""
                        )
                        if terminal_reason:
                            left_ack_failed = True
                            left_skip_reason = terminal_reason
                        elif not self._is_ok_ack(left_result.get("response")):
                            left_ack_failed = True
            else:
                threads = [
                    threading.Thread(
                        target=send_one,
                        args=(label, host, port, timeout_s),
                        name=f"wiris-clock-{self.command_text.lower()}-{label}",
                    )
                    for label, host, port, timeout_s in self.target_infos
                ]
                for thread in threads:
                    thread.start()
                max_timeout_s = max(
                    (max(1.0, timeout_s) for _label, _host, _port, timeout_s in self.target_infos),
                    default=1.0,
                )
                for thread in threads:
                    thread.join(timeout=max_timeout_s + 1.0)
            for label, host, port, _timeout_s in self.target_infos:
                results.setdefault(
                    label,
                    {
                        "response": "실패: 전송 스레드 timeout",
                        "ack_ok": False,
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
                        "rtcp-relative-closed-loop"
                        if self.command_text == "STUS" and self.stus_compensation_by_camera
                        else "control-rtt-half-plus-parse-margin"
                        if self.command_text == "STUS"
                        else None
                    ),
                    "stus_attempt_index": self.stus_attempt_index,
                    "operation_id": self.operation_id,
                    "operation_deadline_monotonic_s": self.operation_deadline_monotonic_s,
                    "stus_max_response_timeout_s": _wire_policy.WIRIS_STUS_RESPONSE_TIMEOUT_S,
                    "stus_feedback_offset_by_camera": dict(self.stus_feedback_offset_by_camera),
                    "stus_feedback_role": "left-right-relative-error",
                    "stus_parse_margin_s": _wire_policy.WIRIS_STUS_PARSE_MARGIN_S
                    if self.command_text == "STUS"
                    else None,
                    "dispatch_policy": (
                        "left-ok-ack-then-right"
                        if self.command_text == "STUS"
                        else "parallel-shared-utc" if self.shared_utc else "parallel"
                    ),
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

# Keep the named legacy class identity; implementation ownership is here.
WirisClockCommandWorker.__module__ = "Workstation.wiris_runtime_workers"
