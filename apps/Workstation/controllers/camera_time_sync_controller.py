"""Camera clock synchronization controller for the workstation app.

The controller coordinates WIRIS STUS evidence and RTCP/NTP observations.
Timestamp values are Unix UTC seconds or monotonic seconds as explicitly named;
it performs no camera RDF, body FRD, or world NED coordinate conversion.
"""

from __future__ import annotations

import time
from collections import deque
from math import isfinite
from statistics import median

from PyQt5 import QtCore

from Workstation.workstation_core.measurement_widgets import ClockOffsetPoint
from Workstation.workstation_core.runtime_camera_status import (
    infer_runtime_camera_timestamp_reference,
    is_sr_pc_diagnostic_reason,
)
from Workstation.wiris_workers.policy import WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S, WIRIS_STUS_MAX_ATTEMPTS, WIRIS_STUS_MAX_OPERATION_S
from Workstation.workstation_core.rtcp_clock_observer import RtcpClockObserverWorker
from Workstation.workstation_core.stus_scheduler import STUS_MODE_AUTO, StusScheduler
CAMERA_RTCP_PAIR_MAX_DELTA_S = 0.100
CAMERA_RTCP_MAX_PC_OFFSET_S = 0.100
CAMERA_SYNC_FRAME_HISTORY_WINDOW_S = 1.0
CAMERA_SYNC_HISTORY_MAX_AGE_S = CAMERA_SYNC_FRAME_HISTORY_WINDOW_S
CAMERA_SYNC_MAX_SR_AGE_S = 7.0
CAMERA_SYNC_RECEIVE_PAIR_MAX_DELTA_S = 0.350
CAMERA_SYNC_WARMUP_TIMEOUT_S = 30.0
CAMERA_SYNC_REQUIRED_SR_COUNT = 2
CAMERA_SYNC_MAX_CLOCK_RATE_ERROR = 0.005
CAMERA_SYNC_WARMUP_POLL_INTERVAL_MS = 250
CAMERA_SYNC_MONITOR_POLL_INTERVAL_MS = 1000
CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR = 3
CAMERA_SYNC_CORRECTION_MAX_OFFSET_SPAN_S = 0.050
CAMERA_SYNC_CORRECTION_MAX_OFFSET_MAD_S = 0.020
CAMERA_SYNC_MAX_HOST_CLOCK_STEP_S = 0.050
CAMERA_SYNC_STATUS_NOT_VERIFIED = "not-verified"
CAMERA_SYNC_STATUS_RECONNECTING = "reconnecting"
CAMERA_SYNC_STATUS_WARMING = "warming"
CAMERA_SYNC_STATUS_VERIFIED = "verified"
CAMERA_SYNC_STATUS_FAILED = "failed"


def _finite_float(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if isfinite(parsed) else None


def stus_relative_pair_compensation_by_camera(
    *,
    previous_compensation_by_camera: dict[str, object],
    sr_pc_offset_by_camera: dict[str, object],
) -> tuple[dict[str, float], float, dict[str, float]]:
    """Remove only differential Left/Right clock error from STUS targets.

    Per-camera ``SR-PC`` is still observed, but its absolute/common-mode value
    is not a success or mutation gate.  Subtracting the two-side median leaves
    only relative error.  The resulting external target remains bounded.
    """

    previous: dict[str, float] = {}
    offsets: dict[str, float] = {}
    for camera in ("left", "right"):
        previous_s = _finite_float(previous_compensation_by_camera.get(camera))
        offset_s = _finite_float(sr_pc_offset_by_camera.get(camera))
        if previous_s is None or offset_s is None:
            raise ValueError(f"{camera} STUS relative closed-loop input is not finite")
        previous[camera] = previous_s
        offsets[camera] = offset_s
    common_mode_s = median(offsets.values())
    relative_error_by_camera = {
        camera: offsets[camera] - common_mode_s
        for camera in ("left", "right")
    }
    next_by_camera = {
        camera: previous[camera] - relative_error_by_camera[camera]
        for camera in ("left", "right")
    }
    for camera, next_s in next_by_camera.items():
        if abs(next_s) > WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S:
            raise ValueError(
                f"{camera} STUS relative closed-loop target exceeds "
                f"±{WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S:.3f}s limit"
            )
    return next_by_camera, common_mode_s, relative_error_by_camera


def _sr_pc_observation_s(row: dict[str, object]) -> float | None:
    """Return raw SR-PC when available, otherwise its legacy bounded value."""

    raw_s = _finite_float(row.get("sr_pc_offset_raw_s"))
    return raw_s if raw_s is not None else _finite_float(row.get("sr_pc_offset_s"))


def _format_sr_pc_diagnostic(offset_s: float) -> str:
    return (
        f"{offset_s * 1000.0:+.1f}ms"
        if abs(offset_s) < 10.0
        else f"{offset_s:+.3f}s"
    )


def current_frame_observation_age_s(
    metadata: dict[str, object],
    *,
    now_monotonic_s: float,
) -> float | None:
    """Return the current age of a frame received on the monotonic clock.

    ``timestamp_s`` is captured when the frame crosses the application receive
    boundary.  Re-evaluating it against the current monotonic observation is
    what makes a frozen camera become stale even though its last metadata dict
    remains internally self-consistent.
    """

    received_monotonic_s = _finite_float(metadata.get("timestamp_s"))
    now_monotonic_s = _finite_float(now_monotonic_s)
    if received_monotonic_s is None or now_monotonic_s is None:
        return None
    elapsed_s = now_monotonic_s - received_monotonic_s
    if elapsed_s < -0.1:
        return None
    return max(0.0, elapsed_s)


def current_sender_report_age_s(
    metadata: dict[str, object],
    *,
    now_monotonic_s: float,
    now_wall_s: float,
) -> float | None:
    """Recompute Sender Report age at the current observation.

    Stream metadata stores the SR age at frame receipt, not at UI/controller
    inspection time.  Advance that value by the frame's monotonic age and, when
    the raw SR wall-clock arrival is available, independently compare it with
    the current wall clock.  The larger valid age is deliberately conservative
    and prevents a dual-camera freeze from retaining a green state.
    """

    candidates: list[float] = []
    age_at_frame_s = _finite_float(metadata.get("rtcp_sr_age_s"))
    frame_age_s = current_frame_observation_age_s(
        metadata,
        now_monotonic_s=now_monotonic_s,
    )
    if age_at_frame_s is not None and age_at_frame_s >= 0.0 and frame_age_s is not None:
        candidates.append(age_at_frame_s + frame_age_s)

    received_wall_s = _finite_float(metadata.get("rtcp_sr_received_wall_s"))
    now_wall_s = _finite_float(now_wall_s)
    if received_wall_s is not None and now_wall_s is not None:
        wall_age_s = now_wall_s - received_wall_s
        if wall_age_s >= -0.1:
            candidates.append(max(0.0, wall_age_s))
    return max(candidates) if candidates else None

CAMERA_TIME_SYNC_METHODS = [
    "_set_camera_sync_state",
    "_camera_sync_widget_value",
    "_camera_sync_history_window_s",
    "_camera_sync_receive_pair_max_delta_s",
    "_camera_sync_rtcp_pair_max_delta_s",
    "_camera_sync_max_sr_age_s",
    "_camera_sync_max_pc_offset_s",
    "_camera_sync_max_clock_rate_error",
    "_camera_sync_required_sr_count",
    "_camera_sync_warmup_timeout_s",
    "_camera_sync_readiness",
    "_camera_sync_correction_observation",
    "_start_rtcp_clock_observer",
    "_restart_rtcp_clock_observer",
    "_stop_rtcp_clock_observer",
    "_handle_rtcp_clock_observation",
    "_rtcp_clock_observer_failed",
    "_clear_rtcp_clock_observer",
    "_reset_stus_schedule_for_connection",
    "_poll_stus_schedule",
    "_stus_schedule_snapshot",
    "_decorate_clock_epoch_metadata",
    "_poll_camera_sync_state",
    "_begin_camera_sync_after_stus",
    "_clock_sync_side_summary",
    "_rtcp_sender_report_count",
    "_clock_sync_pair_metadata",
    "_camera_time_sync_metrics",
    "_camera_time_sync_snapshot",
    "_capture_recording_clock_sync_start_observation",
    "_camera_time_sync_drift_metadata",
    "_format_clock_sync_status",
    "_record_clock_offset_sample",
    "_update_clock_sync_display",
    "_is_clock_sync_tab_visible",
    "refresh_clock_sync_status"
]


class CameraTimeSyncController:
    def __init__(self, panel):
        self.panel = panel

    def _observer_targets(self) -> dict[str, dict[str, object]]:
        latency_widget = getattr(self.panel, "rtsp_latency_spin", None)
        latency_value = getattr(latency_widget, "value", None)
        latency_ms = int(latency_value()) if callable(latency_value) else 100
        transport_widget = getattr(self.panel, "rtsp_transport_combo", None)
        transport_value = getattr(transport_widget, "currentData", None)
        transport = str(transport_value() or "tcp") if callable(transport_value) else "tcp"
        return {
            "left": {
                "url": str(self.panel.left_url_edit.text()).strip(),
                "latency_ms": latency_ms,
                "transport": transport,
            },
            "right": {
                "url": str(self.panel.right_url_edit.text()).strip(),
                "latency_ms": latency_ms,
                "transport": transport,
            },
        }

    def _start_rtcp_clock_observer(self, *, operation_id: str = "") -> bool:
        if getattr(self.panel, "_rtcp_clock_observer_thread", None) is not None:
            return False
        targets = self._observer_targets()
        if any(not str(row.get("url") or "") for row in targets.values()):
            return False
        generation = int(getattr(self.panel, "_rtcp_clock_observer_generation", 0)) + 1
        self.panel._rtcp_clock_observer_generation = generation
        histories = getattr(self.panel, "_rtcp_clock_observation_history", None)
        if isinstance(histories, dict):
            for side in ("left", "right"):
                clear = getattr(histories.get(side), "clear", None)
                if callable(clear):
                    clear()
        thread = QtCore.QThread(self.panel)
        worker_factory = getattr(
            self.panel,
            "_rtcp_clock_observer_worker_factory",
            RtcpClockObserverWorker,
        )
        worker = worker_factory(
            targets,
            observer_generation=generation,
        )
        self.panel._rtcp_clock_observer_thread = thread
        self.panel._rtcp_clock_observer_worker = worker
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.observation_ready.connect(self.panel._handle_rtcp_clock_observation)
        worker.status_changed.connect(
            lambda text: self.panel._log(f"[시계/RTCP] {text}")
        )
        worker.failed.connect(self.panel._rtcp_clock_observer_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(
            lambda thread=thread, worker=worker: self.panel._clear_rtcp_clock_observer(
                thread,
                worker,
            )
        )
        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        if isinstance(operation, dict) and str(operation.get("operation_id") or "") == str(
            operation_id or ""
        ):
            operation["observer_generation"] = generation
        thread.start()
        return True

    def _restart_rtcp_clock_observer(self, *, operation_id: str = "") -> bool:
        self.panel._rtcp_clock_observer_pending_start = {
            "operation_id": str(operation_id or ""),
        }
        worker = getattr(self.panel, "_rtcp_clock_observer_worker", None)
        thread = getattr(self.panel, "_rtcp_clock_observer_thread", None)
        if worker is not None or thread is not None:
            cancel = getattr(worker, "cancel", None)
            if callable(cancel):
                cancel()
            return True
        pending = self.panel._rtcp_clock_observer_pending_start
        self.panel._rtcp_clock_observer_pending_start = None
        return self._start_rtcp_clock_observer(
            operation_id=str((pending or {}).get("operation_id") or "")
        )

    def _stop_rtcp_clock_observer(self) -> None:
        self.panel._rtcp_clock_observer_pending_start = None
        worker = getattr(self.panel, "_rtcp_clock_observer_worker", None)
        cancel = getattr(worker, "cancel", None)
        if callable(cancel):
            cancel()

    def _clear_rtcp_clock_observer(self, thread: object, worker: object) -> None:
        if getattr(self.panel, "_rtcp_clock_observer_thread", None) is thread:
            self.panel._rtcp_clock_observer_thread = None
        if getattr(self.panel, "_rtcp_clock_observer_worker", None) is worker:
            self.panel._rtcp_clock_observer_worker = None
        pending = getattr(self.panel, "_rtcp_clock_observer_pending_start", None)
        if not isinstance(pending, dict):
            return
        self.panel._rtcp_clock_observer_pending_start = None
        self._start_rtcp_clock_observer(
            operation_id=str(pending.get("operation_id") or "")
        )

    def _handle_rtcp_clock_observation(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        try:
            generation = int(payload.get("observer_generation"))
        except (TypeError, ValueError):
            return
        if generation != int(getattr(self.panel, "_rtcp_clock_observer_generation", -1)):
            return
        side = str(payload.get("side") or "")
        histories = getattr(self.panel, "_rtcp_clock_observation_history", None)
        if side not in {"left", "right"} or not isinstance(histories, dict):
            return
        history = histories.get(side)
        if history is None:
            return
        identity = (payload.get("ssrc"), payload.get("ntp_raw"))
        if not any(
            (row.get("ssrc"), row.get("ntp_raw")) == identity
            for row in history
        ):
            history.append(dict(payload))
            append_data = getattr(self.panel, "append_workstation_data", None)
            if callable(append_data):
                try:
                    append_data(
                        "events",
                        {
                            "event_type": "camera_clock_observation",
                            **dict(payload),
                            "coordinate_contract": (
                                "camera RTCP/PC clock evidence; no NED/FRD/RDF conversion"
                            ),
                        },
                        priority=False,
                    )
                except Exception as exc:  # noqa: BLE001 - observation remains live.
                    self.panel._log(f"[시계] RTCP observation 기록 실패: {exc}")
        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        if not isinstance(operation, dict) or not operation.get("active"):
            return
        phase = str(operation.get("phase") or "")
        if phase.startswith("observer-restarting-attempt-"):
            if all(histories.get(camera) for camera in ("left", "right")):
                attempt = int(operation.get("attempts_started") or 1)
                operation["phase"] = f"observing-attempt-{attempt}"
                self.panel._set_camera_sync_state(
                    CAMERA_SYNC_STATUS_WARMING,
                    f"STUS attempt {attempt}: post-ACK RTCP observer SR 안정화 대기",
                )
            return
        if phase != "observer-preflight":
            return
        # One actual pad-probe SR per side proves the decoder-free observer
        # boundary. The command worker independently owns 2-of-3 HIWS reachability
        # preflight before any clock mutation.
        ready = all(len(histories.get(camera) or ()) >= 1 for camera in ("left", "right"))
        if not ready:
            return
        dispatch = getattr(self.panel, "_dispatch_wiris_time_sync_initial", None)
        if callable(dispatch):
            dispatch()

    def _rtcp_clock_observer_failed(self, error: str) -> None:
        reason = f"RTCP clock observer failed: {error}"
        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        if isinstance(operation, dict) and operation.get("active"):
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_FAILED, reason)
            finish = getattr(self.panel, "_finish_wiris_time_sync_operation", None)
            if callable(finish):
                finish("failed", reason)
        elif str(getattr(self.panel, "camera_sync_status", "")) == CAMERA_SYNC_STATUS_VERIFIED:
            self.panel._set_camera_sync_state(
                CAMERA_SYNC_STATUS_FAILED,
                "지속 동기화 검증 실패: " + reason,
            )

    def _reset_stus_schedule_for_connection(self) -> None:
        generation = int(getattr(self.panel, "_stus_connection_generation", 0)) + 1
        self.panel._stus_connection_generation = generation
        scheduler = getattr(self.panel, "_stus_scheduler", None)
        if isinstance(scheduler, StusScheduler):
            scheduler.reset_for_connection(generation)

    def _stus_schedule_snapshot(self) -> dict[str, object]:
        scheduler = getattr(self.panel, "_stus_scheduler", None)
        if not isinstance(scheduler, StusScheduler):
            return {
                "schema": "workstation-stus-schedule-v1",
                "mode": "manual",
                "state": "UNAVAILABLE",
                "seeded": False,
            }
        return scheduler.snapshot(now_monotonic_s=time.monotonic())

    def _poll_stus_schedule(self) -> None:
        scheduler = getattr(self.panel, "_stus_scheduler", None)
        if not isinstance(scheduler, StusScheduler):
            return
        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        active = isinstance(operation, dict) and bool(operation.get("active"))
        trigger = scheduler.poll(
            now_monotonic_s=time.monotonic(),
            operation_active=active,
        )
        if trigger != STUS_MODE_AUTO:
            return
        request = getattr(self.panel, "set_selected_wiris_time_from_pc", None)
        if callable(request):
            accepted = request(target="both", trigger=STUS_MODE_AUTO)
            if accepted is False:
                scheduler.mark_finished(
                    now_monotonic_s=time.monotonic(),
                    trigger=STUS_MODE_AUTO,
                    verified=False,
                    reason="scheduled STUS request rejected",
                )

    def _decorate_clock_epoch_metadata(
        self,
        camera: str,
        metadata: dict[str, object],
    ) -> dict[str, object]:
        """Attach the logical STUS boundary without changing media timestamps."""

        decorated = dict(metadata)
        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        last_sync = getattr(self.panel, "_last_wiris_pc_utc_sync", None)
        epoch_id = ""
        epoch_state = "unverified"
        if isinstance(operation, dict) and bool(operation.get("active")):
            epoch_id = str(operation.get("clock_epoch_id") or "")
            epoch_state = "transition"
        elif isinstance(last_sync, dict) and str(last_sync.get("result") or "") == "success":
            epoch_id = str(last_sync.get("clock_epoch_id") or "")
            closed_loop = last_sync.get("closed_loop_operation")
            ack_by_camera = (
                dict(closed_loop.get("last_stus_ack_finished_utc_by_camera") or {})
                if isinstance(closed_loop, dict)
                else {}
            )
            try:
                post_ack = float(decorated.get("rtcp_sr_received_wall_s")) > float(
                    ack_by_camera[camera]
                )
            except (KeyError, TypeError, ValueError):
                post_ack = False
            epoch_state = "current" if post_ack else "not-verified"
        decorated["clock_epoch_id"] = epoch_id or None
        decorated["clock_epoch_state"] = epoch_state
        status_by_side = getattr(self.panel, "frame_timing_status_by_side", None)
        if isinstance(status_by_side, dict) and camera in {"left", "right"}:
            status_by_side[camera] = epoch_state
            self.panel.frame_timing_status = (
                "current"
                if set(status_by_side.values()) == {"current"}
                else "not-verified"
            )
            self.panel.frame_timing_reason = (
                "both decoded streams have post-STUS RTCP mapping"
                if self.panel.frame_timing_status == "current"
                else "decoded frame mapping is pending or crosses a STUS clock epoch"
            )
        return decorated

    def _set_camera_sync_state(self, status: str, reason: str) -> None:
        self.panel.camera_sync_status = str(status)
        self.panel.camera_sync_reason = str(reason)
        if hasattr(self.panel, "clock_sync_status_label"):
            self.panel._update_clock_sync_display()

    def _camera_sync_widget_value(self, attribute: str, fallback: float) -> float:
        try:
            widget = getattr(self.panel, attribute)
            return float(widget.value())
        except (AttributeError, RuntimeError, TypeError, ValueError):
            return float(fallback)

    def _camera_sync_history_window_s(self) -> float:
        return max(
            0.1,
            self.panel._camera_sync_widget_value(
                "camera_sync_history_window_spin",
                CAMERA_SYNC_FRAME_HISTORY_WINDOW_S,
            ),
        )

    def _camera_sync_receive_pair_max_delta_s(self) -> float:
        return max(
            0.0,
            self.panel._camera_sync_widget_value(
                "camera_sync_receive_pair_max_delta_spin",
                CAMERA_SYNC_RECEIVE_PAIR_MAX_DELTA_S * 1000.0,
            )
            / 1000.0,
        )

    def _camera_sync_rtcp_pair_max_delta_s(self) -> float:
        return max(
            0.0,
            self.panel._camera_sync_widget_value(
                "camera_sync_rtcp_pair_max_delta_spin",
                CAMERA_RTCP_PAIR_MAX_DELTA_S * 1000.0,
            )
            / 1000.0,
        )

    def _camera_sync_max_sr_age_s(self) -> float:
        return max(
            0.1,
            self.panel._camera_sync_widget_value("camera_sync_max_sr_age_spin", CAMERA_SYNC_MAX_SR_AGE_S),
        )

    def _camera_sync_max_pc_offset_s(self) -> float:
        return max(
            0.0,
            self.panel._camera_sync_widget_value(
                "camera_sync_max_pc_offset_spin",
                CAMERA_RTCP_MAX_PC_OFFSET_S * 1000.0,
            )
            / 1000.0,
        )

    def _camera_sync_max_clock_rate_error(self) -> float:
        return max(
            0.0,
            self.panel._camera_sync_widget_value(
                "camera_sync_clock_rate_error_spin",
                CAMERA_SYNC_MAX_CLOCK_RATE_ERROR * 100.0,
            )
            / 100.0,
        )

    def _camera_sync_required_sr_count(self) -> int:
        try:
            widget = getattr(self.panel, "camera_sync_required_sr_count_spin")
            return max(1, int(widget.value()))
        except (AttributeError, RuntimeError, TypeError, ValueError):
            return CAMERA_SYNC_REQUIRED_SR_COUNT

    def _camera_sync_warmup_timeout_s(self) -> float:
        return max(
            1.0,
            self.panel._camera_sync_widget_value(
                "camera_sync_warmup_timeout_spin",
                CAMERA_SYNC_WARMUP_TIMEOUT_S,
            ),
        )

    def _camera_sync_readiness(self) -> tuple[bool, str]:
        now_mono_s = time.monotonic()
        histories = getattr(self.panel, "_rtcp_clock_observation_history", None)
        if not isinstance(histories, dict):
            return False, "RTCP clock observer history 없음"
        observer_generation = int(
            getattr(self.panel, "_rtcp_clock_observer_generation", -1)
        )
        last_sr_counts = dict(getattr(self.panel, "_camera_sync_last_sr_counts", {}) or {})
        observed_sr_counts: dict[str, int] = {}
        missing: list[str] = []
        medians: dict[str, float] = {}
        for camera in ("left", "right"):
            camera_name = self.panel._camera_name_ko(camera)
            rows_by_identity: dict[tuple[int, int], dict[str, object]] = {}
            for raw in list(histories.get(camera) or ()):
                row = dict(raw)
                try:
                    generation = int(row.get("observer_generation"))
                    identity = (int(row["ssrc"]), int(row["ntp_raw"]))
                except (KeyError, TypeError, ValueError):
                    continue
                if generation != observer_generation:
                    continue
                rows_by_identity[identity] = row
            rows = sorted(
                rows_by_identity.values(),
                key=lambda row: float(row.get("received_monotonic_s") or -1.0),
            )
            count = len(rows)
            observed_sr_counts[camera] = count
            required_sr_count = max(
                CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR,
                self.panel._camera_sync_required_sr_count(),
            )
            if count < required_sr_count:
                missing.append(f"{camera_name} observer SR {count}/{required_sr_count}")
                continue
            recent = rows[-CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR:]
            offsets = [
                value
                for value in (_sr_pc_observation_s(row) for row in recent)
                if value is not None
            ]
            if len(offsets) != CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR:
                missing.append(f"{camera_name} SR-PC observation 없음")
                continue
            median_offset_s = median(offsets)
            span_s = max(offsets) - min(offsets)
            mad_s = median(abs(value - median_offset_s) for value in offsets)
            # Robust median/MAD deliberately tolerates one isolated SR outlier;
            # the full span remains evidence but does not revoke clock health.
            if mad_s > CAMERA_SYNC_CORRECTION_MAX_OFFSET_MAD_S:
                missing.append(
                    f"{camera_name} relative clock observation unstable "
                    f"MAD={mad_s * 1000.0:.1f}ms "
                    f"(span diagnostic={span_s * 1000.0:.1f}ms)"
                )
            medians[camera] = median_offset_s
            previous_count = last_sr_counts.get(camera)
            if previous_count is not None and count < int(previous_count):
                missing.append(f"{camera_name} SR counter regressed {int(previous_count)}→{count}")
            latest = recent[-1]
            received_mono_s = _finite_float(latest.get("received_monotonic_s"))
            age_s = None if received_mono_s is None else max(0.0, now_mono_s - received_mono_s)
            if age_s is None:
                missing.append(f"{camera_name} SR age 없음")
            elif age_s > self.panel._camera_sync_max_sr_age_s():
                missing.append(f"{camera_name} SR stale {age_s:.2f}s")
            if str(latest.get("arrival_basis") or "") != "rtcp-pad-probe":
                missing.append(f"{camera_name} SR arrival pad-probe 없음")
            host_clock_step_s = _finite_float(latest.get("host_clock_step_s"))
            if (
                host_clock_step_s is not None
                and abs(host_clock_step_s) > CAMERA_SYNC_MAX_HOST_CLOCK_STEP_S
            ):
                missing.append(
                    f"PC clock step={host_clock_step_s * 1000.0:+.1f}ms"
                )
            for key, label in (
                ("ntp_rate", "NTP clock"),
                ("media_rate", "RTP clock"),
            ):
                rate = _finite_float(latest.get(key))
                if rate is None:
                    missing.append(f"{camera_name} {label} 없음")
                    continue
                if abs(rate - 1.0) > self.panel._camera_sync_max_clock_rate_error():
                    missing.append(f"{camera_name} {label}={rate:.6f}")
        if set(medians) == {"left", "right"}:
            pair_delta_s = medians["right"] - medians["left"]
            if abs(pair_delta_s) > self.panel._camera_sync_rtcp_pair_max_delta_s():
                missing.append(f"좌우 RTCP clock Δt={abs(pair_delta_s) * 1000.0:.1f}ms")
        else:
            missing.append("좌우 RTCP clock pair 없음")
        if missing:
            return False, "; ".join(missing)
        self.panel._camera_sync_last_sr_counts = observed_sr_counts
        return True, (
            "RTCP observer 좌우 relative clock Δt 검증 완료; "
            "SR-PC diagnostic-only "
            f"L={_format_sr_pc_diagnostic(medians['left'])} "
            f"R={_format_sr_pc_diagnostic(medians['right'])}"
        )

    def _camera_sync_correction_observation(self) -> tuple[str, dict[str, object]]:
        """Build one bounded candidate from decoder-independent post-STUS SRs."""

        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        if not isinstance(operation, dict) or not operation.get("active"):
            return "waiting", {"reason": "active STUS operation 없음"}
        if not str(operation.get("phase") or "").startswith("observing-attempt-"):
            return "waiting", {"reason": f"phase={operation.get('phase', 'unknown')}"}
        now_mono_s = time.monotonic()
        now_wall_s = time.time()
        histories = getattr(self.panel, "_rtcp_clock_observation_history", None)
        if not isinstance(histories, dict):
            return "waiting", {"reason": "RTCP clock observer history 없음"}
        observer_generation = int(operation.get("observer_generation") or -1)
        waiting: list[str] = []
        offsets: dict[str, float] = {}
        offset_statistics: dict[str, dict[str, float | int]] = {}
        sr_counts: dict[str, int] = {}
        sr_ages: dict[str, float] = {}
        last_received_monotonic: dict[str, float] = {}
        ack_finished_by_camera = dict(
            operation.get("last_stus_ack_finished_utc_by_camera") or {}
        )
        for camera in ("left", "right"):
            camera_name = self.panel._camera_name_ko(camera)
            ack_finished_utc_s = _finite_float(ack_finished_by_camera.get(camera))
            rows_by_identity: dict[tuple[int, int], dict[str, object]] = {}
            for raw in list(histories.get(camera) or ()):
                row = dict(raw)
                try:
                    generation = int(row.get("observer_generation"))
                    identity = (int(row["ssrc"]), int(row["ntp_raw"]))
                except (KeyError, TypeError, ValueError):
                    continue
                if generation != observer_generation:
                    continue
                received_wall_s = _finite_float(row.get("received_wall_s"))
                if (
                    ack_finished_utc_s is not None
                    and (received_wall_s is None or received_wall_s <= ack_finished_utc_s)
                ):
                    continue
                if str(row.get("arrival_basis") or "") != "rtcp-pad-probe":
                    continue
                received_mono_s = _finite_float(row.get("received_monotonic_s"))
                offset_s = _sr_pc_observation_s(row)
                host_clock_step_s = _finite_float(row.get("host_clock_step_s"))
                if received_mono_s is None or offset_s is None:
                    continue
                age_s = max(0.0, now_mono_s - received_mono_s)
                if age_s > self.panel._camera_sync_max_sr_age_s():
                    continue
                if (
                    host_clock_step_s is not None
                    and abs(host_clock_step_s) > CAMERA_SYNC_MAX_HOST_CLOCK_STEP_S
                ):
                    continue
                rows_by_identity[identity] = row
            recent_samples = sorted(
                rows_by_identity.values(),
                key=lambda row: float(row["received_monotonic_s"]),
            )[-CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR:]
            sr_counts[camera] = len(rows_by_identity)
            if len(recent_samples) < CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR:
                waiting.append(
                    f"{camera_name} unique post-ACK observer SR "
                    f"{len(recent_samples)}/{CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR}"
                )
                continue
            values = [
                float(value)
                for row in recent_samples
                if (value := _sr_pc_observation_s(row)) is not None
            ]
            if len(values) != CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR:
                waiting.append(f"{camera_name} SR-PC diagnostic observation 대기")
                continue
            latest_ntp_rate = _finite_float(recent_samples[-1].get("ntp_rate"))
            latest_media_rate = _finite_float(recent_samples[-1].get("media_rate"))
            if (
                latest_ntp_rate is None
                or latest_media_rate is None
                or abs(latest_ntp_rate - 1.0) > self.panel._camera_sync_max_clock_rate_error()
                or abs(latest_media_rate - 1.0) > self.panel._camera_sync_max_clock_rate_error()
            ):
                waiting.append(f"{camera_name} observer clock rate 안정화 대기")
                continue
            median_offset_s = median(values)
            span_s = max(values) - min(values)
            mad_s = median(abs(value - median_offset_s) for value in values)
            offset_statistics[camera] = {
                "unique_samples": len(recent_samples),
                "median_s": median_offset_s,
                "span_s": span_s,
                "mad_s": mad_s,
            }
            if mad_s > CAMERA_SYNC_CORRECTION_MAX_OFFSET_MAD_S:
                waiting.append(
                    f"{camera_name} relative clock feedback 안정화 대기 "
                    f"(MAD={mad_s * 1000.0:.1f}ms, "
                    f"span diagnostic={span_s * 1000.0:.1f}ms)"
                )
            else:
                offsets[camera] = median_offset_s
                last_received = max(float(row["received_monotonic_s"]) for row in recent_samples)
                last_received_monotonic[camera] = last_received
                sr_ages[camera] = max(0.0, now_mono_s - last_received)
        if waiting:
            return "waiting", {
                "reason": "; ".join(waiting),
                "sr_count_by_camera": sr_counts,
                "sr_age_by_camera": sr_ages,
                "offset_statistics_by_camera": offset_statistics,
            }

        pair_delta_s = offsets["right"] - offsets["left"]
        receive_pair_delta_s = abs(
            last_received_monotonic["right"] - last_received_monotonic["left"]
        )
        pair_limit_s = self.panel._camera_sync_rtcp_pair_max_delta_s()
        if abs(pair_delta_s) <= pair_limit_s:
            return "converged", {
                "reason": (
                    "3 unique stable pad-probe SRs establish Left/Right relative "
                    f"Δt={abs(pair_delta_s) * 1000.0:.1f}ms; SR-PC diagnostic-only "
                    f"L={_format_sr_pc_diagnostic(offsets['left'])} "
                    f"R={_format_sr_pc_diagnostic(offsets['right'])}"
                ),
                "sr_pc_offset_by_camera": offsets,
                "sr_pc_role": "diagnostic-only",
                "pair_delta_s": pair_delta_s,
                "offset_statistics_by_camera": offset_statistics,
            }
        previous = dict(operation.get("last_compensation_by_camera") or {})
        try:
            (
                next_compensation,
                common_mode_s,
                relative_error_by_camera,
            ) = stus_relative_pair_compensation_by_camera(
                previous_compensation_by_camera=previous,
                sr_pc_offset_by_camera=offsets,
            )
        except ValueError as exc:
            return "limit-failed", {
                "reason": str(exc),
                "sr_pc_offset_by_camera": offsets,
                "sr_pc_role": "diagnostic-only",
                "pair_delta_s": pair_delta_s,
                "previous_compensation_by_camera": previous,
            }
        return "candidate", {
            "schema": "workstation-stus-correction-observation-v3",
            "attempt_from": int(operation.get("attempts_started") or 1),
            "observed_at_unix_s": now_wall_s,
            "observed_at_monotonic_s": now_mono_s,
            "observer_generation": observer_generation,
            "sr_pc_offset_by_camera": offsets,
            "sr_pc_role": "diagnostic-only",
            "sr_pc_common_mode_s": common_mode_s,
            "relative_error_by_camera": relative_error_by_camera,
            "previous_compensation_by_camera": previous,
            "next_compensation_by_camera": next_compensation,
            "pair_delta_s": pair_delta_s,
            "receive_pair_delta_s": receive_pair_delta_s,
            "sr_count_by_camera": sr_counts,
            "sr_age_by_camera": sr_ages,
            "offset_statistics_by_camera": offset_statistics,
            "limits": {
                "max_attempts": int(operation.get("max_attempts") or WIRIS_STUS_MAX_ATTEMPTS),
                "max_operation_s": WIRIS_STUS_MAX_OPERATION_S,
                "max_abs_compensation_s": WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S,
                "max_pair_delta_s": pair_limit_s,
                "sr_pc_role": "diagnostic-only",
                "required_unique_pad_probe_sr": CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR,
                "feedback_span_reference_s": CAMERA_SYNC_CORRECTION_MAX_OFFSET_SPAN_S,
                "feedback_span_role": "diagnostic-only",
                "max_feedback_mad_s": CAMERA_SYNC_CORRECTION_MAX_OFFSET_MAD_S,
            },
        }

    def _poll_camera_sync_state(self) -> None:
        status = self.panel.camera_sync_status
        if status not in {
            CAMERA_SYNC_STATUS_RECONNECTING,
            CAMERA_SYNC_STATUS_WARMING,
            CAMERA_SYNC_STATUS_VERIFIED,
        }:
            self.panel._camera_sync_timer.stop()
            return
        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        operation_active = isinstance(operation, dict) and bool(operation.get("active"))
        finish_operation = getattr(self.panel, "_finish_wiris_time_sync_operation", None)

        def fail_operation(reason: str) -> None:
            self.panel._camera_sync_timer.stop()
            self.panel._camera_sync_stus_sr_baseline = {}
            self.panel._camera_sync_last_sr_counts = {}
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_FAILED, reason)
            if callable(finish_operation):
                finish_operation("failed", reason)
            elif operation_active:
                operation["active"] = False
                operation["phase"] = "failed"

        now_mono_s = time.monotonic()
        if operation_active:
            overall_deadline_s = _finite_float(operation.get("deadline_monotonic_s"))
            if overall_deadline_s is not None and now_mono_s >= overall_deadline_s:
                fail_operation(
                    f"STUS closed-loop timeout ({WIRIS_STUS_MAX_OPERATION_S:.0f}s limit): "
                    + self.panel.camera_sync_reason
                )
                return
        phase = str(operation.get("phase") or "") if operation_active else ""
        if operation_active and phase in {"initial-command", "correction-command"}:
            attempts_started = int(operation.get("attempts_started") or 1)
            max_attempts = int(operation.get("max_attempts") or WIRIS_STUS_MAX_ATTEMPTS)
            self.panel._set_camera_sync_state(
                CAMERA_SYNC_STATUS_WARMING,
                f"STUS closed-loop attempt {attempts_started}/{max_attempts} command ACK 대기",
            )
            return
        deadline = self.panel._camera_sync_deadline_monotonic_s
        if status != CAMERA_SYNC_STATUS_VERIFIED and deadline is not None and now_mono_s >= deadline:
            fail_operation("동기화 검증 timeout: " + self.panel.camera_sync_reason)
            return
        ready, reason = self.panel._camera_sync_readiness()
        observation_state = "waiting"
        observation: dict[str, object] = {"reason": reason}
        if operation_active:
            observation_state, observation = self.panel._camera_sync_correction_observation()
            if observation_state == "converged":
                ready = True
                reason = str(
                    observation.get("reason")
                    or "RTCP clock observer convergence verified"
                )
            if observation_state == "limit-failed":
                fail_operation(
                    "STUS closed-loop fail-closed: "
                    + str(observation.get("reason") or "correction limit exceeded")
                )
                return
            if observation_state == "candidate":
                attempts_started = int(operation.get("attempts_started") or 1)
                max_attempts = int(operation.get("max_attempts") or WIRIS_STUS_MAX_ATTEMPTS)
                if attempts_started >= max_attempts:
                    fail_operation(
                        f"STUS closed-loop did not converge after {attempts_started}/{max_attempts} attempts: "
                        + reason
                    )
                    return
                request_correction = getattr(
                    self.panel,
                    "_request_wiris_time_sync_correction",
                    None,
                )
                if not callable(request_correction):
                    fail_operation("STUS closed-loop correction owner is unavailable")
                    return
                requested = bool(
                    request_correction(
                        compensation_by_camera=dict(
                            observation.get("next_compensation_by_camera") or {}
                        ),
                        feedback_offset_by_camera=dict(
                            observation.get("relative_error_by_camera") or {}
                        ),
                        observation=dict(observation),
                    )
                )
                if requested:
                    self.panel._set_camera_sync_state(
                        CAMERA_SYNC_STATUS_WARMING,
                        "STUS closed-loop attempt 2/2: fresh Left/Right relative feedback applied; "
                        "Left OK ACK before Right mutation",
                    )
                return
            if ready and observation_state != "converged":
                self.panel._set_camera_sync_state(
                    CAMERA_SYNC_STATUS_WARMING,
                    str(observation.get("reason") or "stable unique SR convergence evidence 대기"),
                )
                return
        if ready:
            self.panel._camera_sync_deadline_monotonic_s = None
            self.panel._camera_sync_stus_sr_baseline = {}
            set_interval = getattr(self.panel._camera_sync_timer, "setInterval", None)
            if callable(set_interval):
                set_interval(CAMERA_SYNC_MONITOR_POLL_INTERVAL_MS)
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_VERIFIED, reason)
            if operation_active:
                if callable(finish_operation):
                    finish_operation("verified", reason)
                else:
                    operation["active"] = False
                    operation["phase"] = "verified"
        elif operation_active:
            self.panel._set_camera_sync_state(
                CAMERA_SYNC_STATUS_WARMING,
                str(observation.get("reason") or reason),
            )
        elif status == CAMERA_SYNC_STATUS_VERIFIED:
            self.panel._camera_sync_timer.stop()
            revoked_reason = "지속 동기화 검증 실패: " + reason
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_FAILED, revoked_reason)
            scheduler = getattr(self.panel, "_stus_scheduler", None)
            mark_revoked = getattr(scheduler, "mark_clock_revoked", None)
            if callable(mark_revoked):
                mark_revoked(revoked_reason)
            last_sync = getattr(self.panel, "_last_wiris_pc_utc_sync", None)
            if isinstance(last_sync, dict) and last_sync.get("command") == "STUS":
                last_sync["result"] = "revoked"
                last_sync["verification_reason"] = revoked_reason
                last_sync["revoked_at_unix_s"] = time.time()
        else:
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_WARMING, reason)

    def _begin_camera_sync_after_stus(
        self,
        *,
        target: str,
        result: str,
    ) -> None:
        if target != "both" or result != "success":
            self.panel._camera_sync_stus_sr_baseline = {}
            self.panel._camera_sync_last_sr_counts = {}
            self.panel._set_camera_sync_state(
                CAMERA_SYNC_STATUS_FAILED,
                "좌우 카메라 모두에 STUS가 성공해야 동기화를 검증할 수 있습니다.",
            )
            finish = getattr(self.panel, "_finish_wiris_time_sync_operation", None)
            if callable(finish):
                finish("failed", "one or more STUS ACKs were not OK")
            return
        # The video/YOLO/MP4 owner must remain continuous across STUS. Recreate
        # only the decoder-free RTCP observer so its next Sender Reports are a
        # post-command mapping epoch. Never restart CameraPane here.
        self.panel._camera_sync_stus_sr_baseline = {"left": 0, "right": 0}
        self.panel._camera_sync_last_sr_counts = {}
        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        if isinstance(operation, dict) and operation.get("active"):
            attempt_index = int(operation.get("attempts_started") or 1)
            operation["phase"] = f"observer-restarting-attempt-{attempt_index}"
            operation["feedback_attempt_index"] = int(operation.get("attempts_started") or 1)
            operation["sr_feedback_samples_by_camera"] = {"left": [], "right": []}
        self.panel._camera_sync_deadline_monotonic_s = time.monotonic() + self.panel._camera_sync_warmup_timeout_s()
        self.panel._camera_sync_transport_override = None
        set_interval = getattr(self.panel._camera_sync_timer, "setInterval", None)
        if callable(set_interval):
            set_interval(CAMERA_SYNC_WARMUP_POLL_INTERVAL_MS)
        self.panel._set_camera_sync_state(
            CAMERA_SYNC_STATUS_RECONNECTING,
            "STUS 완료: 영상/YOLO/MP4는 유지하고 RTCP clock observer만 재연결합니다.",
        )
        restart_observer = getattr(self.panel, "_restart_rtcp_clock_observer", None)
        operation_id = (
            str(operation.get("operation_id") or "")
            if isinstance(operation, dict)
            else ""
        )
        started = callable(restart_observer) and bool(
            restart_observer(operation_id=operation_id)
        )
        if not started:
            self.panel._camera_sync_timer.stop()
            reason = "STUS ACK는 완료됐지만 replacement RTCP clock observer 요청이 실패했습니다."
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_FAILED, reason)
            finish = getattr(self.panel, "_finish_wiris_time_sync_operation", None)
            if callable(finish):
                finish("failed", reason)
            return
        self.panel._camera_sync_timer.start()

    def _clock_sync_side_summary(self, camera: str, metadata: dict[str, object], now_wall_s: float, now_mono_s: float) -> tuple[str, float | None]:
        invalid_reason = metadata.get("rtcp_timing_invalid_reason")
        if invalid_reason and not is_sr_pc_diagnostic_reason(invalid_reason):
            return (
                f"{self.panel._camera_name_ko(camera)}: RTCP/NTP 무효({invalid_reason})",
                None,
            )
        camera_rtc_s = self.panel._camera_rtc_timestamp_s_from_metadata(metadata)
        receive_mono_s = metadata.get("timestamp_s")
        if camera_rtc_s is None:
            return f"{self.panel._camera_name_ko(camera)}: RTCP/NTP 없음", None
        try:
            frame_age_s = max(0.0, now_mono_s - float(receive_mono_s)) if receive_mono_s is not None else 0.0
        except (TypeError, ValueError):
            frame_age_s = 0.0
        frame_receive_delay_s = metadata.get("rtcp_frame_receive_delay_s")
        try:
            if frame_receive_delay_s is not None and isfinite(float(frame_receive_delay_s)):
                return (
                    f"{self.panel._camera_name_ko(camera)}: transit_delay={float(frame_receive_delay_s) * 1000.0:+.1f} ms, "
                    f"frame_age={frame_age_s:.2f} s, basis={infer_runtime_camera_timestamp_reference(metadata)}",
                    float(frame_receive_delay_s) * 1000.0,
                )
        except (TypeError, ValueError):
            pass
        pc_utc_at_frame_s = now_wall_s - frame_age_s
        offset_ms = (float(camera_rtc_s) - pc_utc_at_frame_s) * 1000.0
        reference = infer_runtime_camera_timestamp_reference(metadata)
        return (
            f"{self.panel._camera_name_ko(camera)}: offset={offset_ms:+.1f} ms, "
            f"frame_age={frame_age_s:.2f} s, basis={reference}",
            offset_ms,
        )

    def _rtcp_sender_report_count(self, metadata: dict[str, object]) -> int | None:
        value = metadata.get("rtcp_sr_received_count")
        try:
            count = int(value)
        except (TypeError, ValueError):
            return None
        return count if count >= 0 else None

    def _clock_sync_pair_metadata(self, now_mono_s: float | None = None) -> tuple[dict[str, object], dict[str, object]]:
        """Select a temporally corresponding recent RTCP frame pair.

        The latest left and latest right frames are not always a pair when the
        cameras decode at different rates.  Select the freshest receive-close
        pair first, then use receive and RTCP deltas as tie breakers.  Never
        cherry-pick an older, healthier-looking RTCP pair from the history.
        """
        now_mono_s = time.monotonic() if now_mono_s is None else float(now_mono_s)
        latest = (
            dict(self.panel.camera_frame_metadata.get("left") or {}),
            dict(self.panel.camera_frame_metadata.get("right") or {}),
        )

        def usable(metadata: dict[str, object]) -> bool:
            return self.panel._camera_rtc_timestamp_s_from_metadata(metadata) is not None

        def recent(history: deque[dict[str, object]]) -> list[dict[str, object]]:
            rows: list[dict[str, object]] = []
            for metadata in history:
                if not usable(metadata):
                    continue
                timestamp_s = metadata.get("timestamp_s")
                if timestamp_s is None:
                    rows.append(dict(metadata))
                    continue
                try:
                    age_s = now_mono_s - float(timestamp_s)
                except (TypeError, ValueError):
                    continue
                if -0.1 <= age_s <= self.panel._camera_sync_history_window_s():
                    rows.append(dict(metadata))
            return rows

        left_rows = recent(self.panel.camera_frame_metadata_history["left"])
        right_rows = recent(self.panel.camera_frame_metadata_history["right"])
        if not left_rows or not right_rows:
            return latest

        candidates: list[
            tuple[float, float, float, dict[str, object], dict[str, object]]
        ] = []
        for left in left_rows:
            left_rtcp_s = self.panel._camera_rtc_timestamp_s_from_metadata(left)
            left_receive_s = left.get("timestamp_s")
            for right in right_rows:
                right_rtcp_s = self.panel._camera_rtc_timestamp_s_from_metadata(right)
                if left_rtcp_s is None or right_rtcp_s is None:
                    continue
                receive_delta_s = float("inf")
                freshness_age_s = float("inf")
                if left_receive_s is not None and right.get("timestamp_s") is not None:
                    try:
                        left_receive_value = float(left_receive_s)
                        right_receive_value = float(right["timestamp_s"])
                        receive_delta_s = abs(right_receive_value - left_receive_value)
                        freshness_age_s = max(
                            0.0,
                            now_mono_s - min(left_receive_value, right_receive_value),
                        )
                    except (TypeError, ValueError):
                        pass
                candidates.append(
                    (
                        freshness_age_s,
                        receive_delta_s,
                        abs(float(right_rtcp_s) - float(left_rtcp_s)),
                        left,
                        right,
                    )
                )
        if not candidates:
            return latest
        close_candidates = [
            candidate
            for candidate in candidates
            if candidate[1] <= self.panel._camera_sync_receive_pair_max_delta_s()
        ]
        search_space = close_candidates or candidates
        _freshness_age, _receive_delta, _rtcp_delta, left, right = min(
            search_space,
            key=lambda candidate: (candidate[0], candidate[1], candidate[2]),
        )
        return left, right

    def _camera_time_sync_metrics(
        self,
        left_metadata: dict[str, object] | None = None,
        right_metadata: dict[str, object] | None = None,
        *,
        left_receive_mono_s: float | None = None,
        right_receive_mono_s: float | None = None,
        now_wall_s: float | None = None,
        now_mono_s: float | None = None,
    ) -> dict[str, object]:
        """Summarize RTCP/NTP timing without treating it as verified UTC."""
        now_wall_s = time.time() if now_wall_s is None else float(now_wall_s)
        now_mono_s = time.monotonic() if now_mono_s is None else float(now_mono_s)
        left_metadata = left_metadata or {}
        right_metadata = right_metadata or {}
        left_rtcp_s = self.panel._camera_rtc_timestamp_s_from_metadata(left_metadata)
        right_rtcp_s = self.panel._camera_rtc_timestamp_s_from_metadata(right_metadata)
        receive_pair_delta_s = None
        try:
            if left_metadata.get("timestamp_s") is not None and right_metadata.get("timestamp_s") is not None:
                receive_pair_delta_s = abs(
                    float(right_metadata["timestamp_s"]) - float(left_metadata["timestamp_s"])
                )
        except (TypeError, ValueError):
            receive_pair_delta_s = None
        invalid_cameras = [
            self.panel._camera_name_ko(camera)
            for camera, metadata in (("left", left_metadata), ("right", right_metadata))
            if metadata.get("rtcp_timing_invalid_reason")
            and not is_sr_pc_diagnostic_reason(metadata.get("rtcp_timing_invalid_reason"))
        ]
        offsets_ms: dict[str, float] = {}
        sr_offsets_ms: dict[str, float] = {}
        transit_delays_ms: dict[str, float] = {}
        for camera, metadata, explicit_receive_s, rtcp_s in (
            ("left", left_metadata, left_receive_mono_s, left_rtcp_s),
            ("right", right_metadata, right_receive_mono_s, right_rtcp_s),
        ):
            invalid_reason = metadata.get("rtcp_timing_invalid_reason")
            if rtcp_s is None or (
                invalid_reason and not is_sr_pc_diagnostic_reason(invalid_reason)
            ):
                continue
            sr_offset_s = metadata.get("rtcp_sr_pc_offset_s")
            try:
                if sr_offset_s is not None and isfinite(float(sr_offset_s)):
                    sr_offsets_ms[camera] = float(sr_offset_s) * 1000.0
            except (TypeError, ValueError):
                pass
            frame_receive_delay_s = metadata.get("rtcp_frame_receive_delay_s")
            try:
                if frame_receive_delay_s is not None and isfinite(float(frame_receive_delay_s)):
                    transit_delays_ms[camera] = float(frame_receive_delay_s) * 1000.0
            except (TypeError, ValueError):
                pass
            receive_s = explicit_receive_s
            if receive_s is None:
                receive_s = metadata.get("timestamp_s")
            if receive_s is None:
                # A transit-delay-only observation has no PC receive anchor;
                # keep it as a transport diagnostic, but do not invent an
                # absolute PC UTC offset from the current wall clock.
                continue
            try:
                frame_age_s = max(0.0, now_mono_s - float(receive_s))
            except (TypeError, ValueError):
                frame_age_s = 0.0
            pc_utc_at_frame_s = now_wall_s - frame_age_s
            offsets_ms[camera] = (float(rtcp_s) - pc_utc_at_frame_s) * 1000.0

        pair_delta_s = None
        if left_rtcp_s is not None and right_rtcp_s is not None:
            pair_delta_s = float(right_rtcp_s) - float(left_rtcp_s)
        max_offset_ms = max((abs(value) for value in offsets_ms.values()), default=None)
        max_sr_offset_ms = max((abs(value) for value in sr_offsets_ms.values()), default=None)
        max_transit_delay_ms = max((abs(value) for value in transit_delays_ms.values()), default=None)
        if pair_delta_s is None:
            if invalid_cameras:
                status = "invalid-rtcp"
                warning = f"{', '.join(invalid_cameras)} Sender Report NTP가 PC 수신 UTC와 500 ms 이상 어긋났습니다."
            else:
                status = "missing-rtcp"
                warning = "좌우 프레임 중 하나에 RTCP/NTP 시각이 없습니다."
        elif abs(pair_delta_s) > self.panel._camera_sync_rtcp_pair_max_delta_s():
            status = "pair-delta-failed"
            warning = (
                f"좌우 RTCP/NTP 차이가 {self.panel._camera_sync_rtcp_pair_max_delta_s() * 1000.0:.0f} ms를 초과했습니다."
            )
        else:
            status = "within-relative-threshold"
            diagnostic_limit_ms = self.panel._camera_sync_max_pc_offset_s() * 1000.0
            if max_sr_offset_ms is not None and max_sr_offset_ms > diagnostic_limit_ms:
                warning = (
                    f"좌우 RTCP/NTP Δt={abs(pair_delta_s) * 1000.0:.1f} ms는 기준 이내입니다. "
                    f"SR-PC 최대 {max_sr_offset_ms:.1f} ms는 진단값이며 gate가 아닙니다."
                )
            else:
                warning = (
                    "좌우 상대 오차는 임계값 이내입니다. "
                    "SR-PC 절대값은 진단 전용입니다."
                )
        return {
            "status": status,
            "basis": "rtcp-sender-report-ntp",
            "pair_delta_s": pair_delta_s,
            "receive_pair_delta_s": receive_pair_delta_s,
            "max_pc_offset_ms": max_offset_ms,
            "max_sr_pc_offset_ms": max_sr_offset_ms,
            "max_frame_transit_delay_ms": max_transit_delay_ms,
            "left_frame_transit_delay_ms": transit_delays_ms.get("left"),
            "right_frame_transit_delay_ms": transit_delays_ms.get("right"),
            "left_pc_offset_ms": offsets_ms.get("left"),
            "right_pc_offset_ms": offsets_ms.get("right"),
            "left_sr_pc_offset_ms": sr_offsets_ms.get("left"),
            "right_sr_pc_offset_ms": sr_offsets_ms.get("right"),
            "warning": warning,
        }

    def _camera_time_sync_snapshot(
        self,
        scope: str,
        *,
        now_wall_s: float | None = None,
        now_mono_s: float | None = None,
    ) -> dict[str, object]:
        """Capture one JSON-safe RTCP/NTP observation before stream state is cleared."""
        now_wall_s = time.time() if now_wall_s is None else float(now_wall_s)
        now_mono_s = time.monotonic() if now_mono_s is None else float(now_mono_s)
        left_metadata, right_metadata = self.panel._clock_sync_pair_metadata(now_mono_s)
        metrics = self.panel._camera_time_sync_metrics(
            left_metadata,
            right_metadata,
            now_wall_s=now_wall_s,
            now_mono_s=now_mono_s,
        )
        return {
            "scope": str(scope),
            "observed_at_unix_s": now_wall_s,
            "observed_at_monotonic_s": now_mono_s,
            "metrics": dict(metrics),
        }

    def _capture_recording_clock_sync_start_observation(self) -> None:
        if not self.panel._session_recording_active() or self.panel._recording_clock_sync_start_observation is not None:
            return
        snapshot = self.panel._camera_time_sync_snapshot("first_valid_pair_after_recording_start")
        metrics = snapshot["metrics"]
        if isinstance(metrics, dict) and metrics.get("pair_delta_s") is not None:
            self.panel._recording_clock_sync_start_observation = snapshot

    def _camera_time_sync_drift_metadata(self) -> dict[str, object]:
        start = self.panel._recording_clock_sync_start_observation
        end = self.panel._recording_clock_sync_end_observation
        if not isinstance(start, dict) or not isinstance(end, dict):
            return {
                "status": "not-enough-observations",
                "start_pair_delta_s": None,
                "end_pair_delta_s": None,
                "pair_delta_change_s": None,
                "elapsed_s": None,
                "relative_drift_ppm": None,
            }
        start_metrics = start.get("metrics")
        end_metrics = end.get("metrics")
        if not isinstance(start_metrics, dict) or not isinstance(end_metrics, dict):
            return {
                "status": "invalid-observations",
                "start_pair_delta_s": None,
                "end_pair_delta_s": None,
                "pair_delta_change_s": None,
                "elapsed_s": None,
                "relative_drift_ppm": None,
            }
        start_delta = start_metrics.get("pair_delta_s")
        end_delta = end_metrics.get("pair_delta_s")
        try:
            start_delta = float(start_delta)
            end_delta = float(end_delta)
            elapsed_s = float(end["observed_at_monotonic_s"]) - float(start["observed_at_monotonic_s"])
        except (KeyError, TypeError, ValueError):
            return {
                "status": "missing-pair-delta",
                "start_pair_delta_s": None,
                "end_pair_delta_s": None,
                "pair_delta_change_s": None,
                "elapsed_s": None,
                "relative_drift_ppm": None,
            }
        change_s = end_delta - start_delta
        relative_drift_ppm = None if elapsed_s <= 0.0 else change_s / elapsed_s * 1_000_000.0
        return {
            "status": "measured" if relative_drift_ppm is not None else "invalid-elapsed-time",
            "start_pair_delta_s": start_delta,
            "end_pair_delta_s": end_delta,
            "pair_delta_change_s": change_s,
            "elapsed_s": elapsed_s,
            "relative_drift_ppm": relative_drift_ppm,
        }

    def _format_clock_sync_status(
        self,
        now_wall_s: float | None = None,
        now_mono_s: float | None = None,
    ) -> tuple[str, str]:
        now_wall_s = time.time() if now_wall_s is None else float(now_wall_s)
        now_mono_s = time.monotonic() if now_mono_s is None else float(now_mono_s)
        left_metadata, right_metadata = self.panel._clock_sync_pair_metadata(now_mono_s)
        left_text, left_offset_ms = self.panel._clock_sync_side_summary("left", left_metadata, now_wall_s, now_mono_s)
        right_text, right_offset_ms = self.panel._clock_sync_side_summary("right", right_metadata, now_wall_s, now_mono_s)
        left_rtc_s = self.panel._camera_rtc_timestamp_s_from_metadata(left_metadata)
        right_rtc_s = self.panel._camera_rtc_timestamp_s_from_metadata(right_metadata)
        invalid_cameras = [
            self.panel._camera_name_ko(camera)
            for camera, metadata in (("left", left_metadata), ("right", right_metadata))
            if metadata.get("rtcp_timing_invalid_reason")
            and not is_sr_pc_diagnostic_reason(metadata.get("rtcp_timing_invalid_reason"))
        ]
        if left_rtc_s is not None and right_rtc_s is not None:
            pair_delta_ms = (float(right_rtc_s) - float(left_rtc_s)) * 1000.0
            pair_text = f"좌우 RTCP/NTPΔ={pair_delta_ms:+.1f} ms"
        else:
            pair_text = "좌우 RTCP/NTPΔ=RTCP/NTP 부족"
        quality_parts = []
        if left_offset_ms is not None:
            quality_parts.append(abs(left_offset_ms))
        if right_offset_ms is not None:
            quality_parts.append(abs(right_offset_ms))
        worst_quality_ms = max(quality_parts) if quality_parts else None
        has_transit_delay = any(
            metadata.get("rtcp_frame_receive_delay_s") is not None
            for metadata in (left_metadata, right_metadata)
        )
        left_sr_count = self.panel._rtcp_sender_report_count(left_metadata)
        right_sr_count = self.panel._rtcp_sender_report_count(right_metadata)
        sr_count_text = (
            "RTCP SR 수신: "
            f"L={'--' if left_sr_count is None else left_sr_count} "
            f"R={'--' if right_sr_count is None else right_sr_count}"
        )
        quality_label = "RTCP frame transit max" if has_transit_delay else "PC UTC 최대 offset"
        status = (
            f"스트림 RTCP/NTP 상태: {pair_text}"
            if worst_quality_ms is None
            else f"스트림 RTCP/NTP 상태: {pair_text}, {quality_label}={worst_quality_ms:.1f} ms"
        )
        status += f", {sr_count_text}"
        if invalid_cameras:
            status += f", 절대 UTC 검증 실패={','.join(invalid_cameras)}"
        detail = "\n".join(
            (
                "time_basis: RTCP Sender Report NTP-derived stream frame timestamp; WIRIS GDTI/UTC mapping unverified",
                "pc_utc_at_frame: current PC UTC minus app receive frame age",
                sr_count_text,
                left_text,
                right_text,
                pair_text,
            )
        )
        return status, detail

    def _record_clock_offset_sample(self, now_s: float, *, min_interval_s: float = 0.5) -> None:
        """Append one RTCP timing sample for the 시간 tab history plot.

        Sampled from the frame path regardless of which tab is showing: the
        plot exists to expose slow drift, which a history with holes in it
        cannot show. Only the painting is gated on visibility.
        """

        if not hasattr(self.panel, "clock_offset_plot"):
            return
        if now_s - self.panel._last_clock_offset_sample_s < min_interval_s:
            return
        self.panel._last_clock_offset_sample_s = now_s

        def metric(camera: str, key: str, scale: float) -> float | None:
            value = (self.panel.camera_frame_metadata.get(camera) or {}).get(key)
            try:
                value = float(value)
            except (TypeError, ValueError):
                return None
            return value * scale if isfinite(value) else None

        self.panel.clock_offset_plot.add_point(
            ClockOffsetPoint(
                timestamp_s=now_s,
                left_transit_delay_ms=metric("left", "rtcp_frame_receive_delay_s", 1000.0),
                right_transit_delay_ms=metric("right", "rtcp_frame_receive_delay_s", 1000.0),
                left_transport_delay_ms=metric("left", "rtcp_frame_transport_delay_s", 1000.0),
                right_transport_delay_ms=metric("right", "rtcp_frame_transport_delay_s", 1000.0),
                left_sr_age_s=metric("left", "rtcp_sr_age_s", 1.0),
                right_sr_age_s=metric("right", "rtcp_sr_age_s", 1.0),
            )
        )

    def _update_clock_sync_display(self) -> str | None:
        """Refresh the RTCP/NTP status label/detail text without logging.

        Called continuously from the frame-arrival path while the 시간 tab is
        visible; the manual button below still logs a snapshot on click.
        """

        if not hasattr(self.panel, "clock_sync_status_label"):
            return None
        status, detail = self.panel._format_clock_sync_status()
        sync_status_text = (
            f"동기화 상태: {self.panel.camera_sync_status} | {self.panel.camera_sync_reason}"
        )
        stus_sync = self.panel._last_wiris_pc_utc_sync
        stus_status_text = ""
        stus_detail = ""
        if isinstance(stus_sync, dict) and stus_sync.get("command") == "STUS":
            stus_status_text = f"PC UTC 설정 전송: {stus_sync.get('result', '')}"
            compensation_method = str(stus_sync.get("compensation_method") or "control-rtt").strip()
            attempt_index = int(stus_sync.get("attempt_index") or 1)
            max_attempts = int(stus_sync.get("max_attempts") or WIRIS_STUS_MAX_ATTEMPTS)
            if compensation_method == "rtcp-relative-closed-loop":
                target_detail = (
                    "target lead removes Left/Right differential around common mode; "
                    "SR-PC absolute is diagnostic-only; 3 unique stable rtcp-pad-probe SRs"
                )
            else:
                target_detail = "initial target lead = bounded HIWS RTT/2 + parse margin"
            stus_detail = (
                f"STUS attempt {attempt_index}/{max_attempts} ({compensation_method}): {target_detail}.\n"
                f"Bounds: {WIRIS_STUS_MAX_ATTEMPTS} attempts, {WIRIS_STUS_MAX_OPERATION_S:.0f}s, "
                f"±{WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S:.3f}s; every round is Left exact-OK before Right.\n"
                "Fresh RTCP/NTP evidence, not ACK alone, owns success."
            )
        status = " | ".join(filter(None, (stus_status_text, sync_status_text, status)))
        detail = "\n".join(filter(None, (stus_detail, sync_status_text, detail)))
        self.panel.clock_sync_status_label.setText(status)
        self.panel.clock_sync_status_label.setToolTip(detail)
        self.panel.clock_sync_summary_text.setPlainText(detail)
        return status

    def _is_clock_sync_tab_visible(self) -> bool:
        return hasattr(self.panel, "time_tab_index") and self.panel.tabs.currentIndex() == self.panel.time_tab_index

    def refresh_clock_sync_status(self) -> None:
        status = self.panel._update_clock_sync_display()
        if status is not None:
            self.panel.workstation_time_sync_checkpoints.append(
                {
                    "timestamp_s": time.monotonic(),
                    "status": str(self.panel.camera_sync_status),
                    "reason": str(self.panel.camera_sync_reason),
                    "phase": "final" if self.panel.alignment_iteration_history else "initial",
                }
            )
            self.panel.workstation_time_sync_checkpoints = self.panel.workstation_time_sync_checkpoints[-20:]
            self.panel._log(f"[시계] {status}")
