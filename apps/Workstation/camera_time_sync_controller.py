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

from Workstation.workstation_core.measurement_widgets import ClockOffsetPoint
from Workstation.workstation_core.runtime_camera_status import infer_runtime_camera_timestamp_reference
from Workstation.wiris_runtime_workers import (
    WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S,
    WIRIS_STUS_MAX_ATTEMPTS,
    WIRIS_STUS_MAX_OPERATION_S,
)
CAMERA_RTCP_PAIR_MAX_DELTA_S = 0.050
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


def stus_closed_loop_compensation_s(
    *,
    previous_compensation_s: object,
    sr_pc_offset_s: object,
) -> float:
    """Return the next absolute STUS target lead from signed RTCP feedback.

    ``rtcp_sr_pc_offset_s`` is ``camera SR NTP - PC receive UTC``.  With an
    absolute-time STUS command, the next target lead is therefore the previous
    lead minus that observed error.  Both the observation and resulting lead
    are bounded before any external mutation is authorized.
    """

    previous_s = _finite_float(previous_compensation_s)
    offset_s = _finite_float(sr_pc_offset_s)
    if previous_s is None or offset_s is None:
        raise ValueError("STUS closed-loop input is not finite")
    if abs(offset_s) > WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S:
        raise ValueError(
            "STUS SR-PC feedback exceeds "
            f"±{WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S:.3f}s limit"
        )
    next_s = previous_s - offset_s
    if abs(next_s) > WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S:
        raise ValueError(
            "STUS closed-loop target exceeds "
            f"±{WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S:.3f}s limit"
        )
    return next_s


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
        if not self.panel.camera_connected:
            return False, "좌우 RTSP 스트림이 실행 중이 아닙니다."
        now_mono_s = time.monotonic()
        now_wall_s = time.time()
        try:
            self.panel.camera_frame_metadata_history
        except (AttributeError, RuntimeError):
            has_frame_history = False
        else:
            has_frame_history = True
        if has_frame_history:
            left_metadata, right_metadata = self.panel._clock_sync_pair_metadata(now_mono_s=now_mono_s)
        else:
            left_metadata = self.panel.camera_frame_metadata.get("left") or {}
            right_metadata = self.panel.camera_frame_metadata.get("right") or {}
        metadata_by_camera = {"left": left_metadata, "right": right_metadata}
        sr_baseline = dict(getattr(self.panel, "_camera_sync_stus_sr_baseline", {}) or {})
        last_sr_counts = dict(getattr(self.panel, "_camera_sync_last_sr_counts", {}) or {})
        observed_sr_counts: dict[str, int] = {}
        missing: list[str] = []
        history_window_reader = getattr(self.panel, "_camera_sync_history_window_s", None)
        history_window_s = (
            max(0.1, float(history_window_reader()))
            if callable(history_window_reader)
            else CAMERA_SYNC_FRAME_HISTORY_WINDOW_S
        )
        for camera, metadata in metadata_by_camera.items():
            camera_name = self.panel._camera_name_ko(camera)
            frame_age_s = current_frame_observation_age_s(
                metadata,
                now_monotonic_s=now_mono_s,
            )
            if frame_age_s is None:
                missing.append(f"{camera_name} frame age 없음")
            elif frame_age_s > history_window_s:
                missing.append(f"{camera_name} frame stale {frame_age_s:.2f}s")

            try:
                count = int(metadata.get("rtcp_sr_received_count"))
            except (TypeError, ValueError):
                count = 0
            count = max(0, count)
            observed_sr_counts[camera] = count
            required_sr_count = self.panel._camera_sync_required_sr_count()
            if count < required_sr_count:
                missing.append(f"{camera_name} SR {count}/{required_sr_count}")
                continue
            baseline_count = sr_baseline.get(camera)
            if baseline_count is not None and count <= int(baseline_count):
                missing.append(f"{camera_name} STUS 이후 새 SR 대기")
                continue
            if str(metadata.get("rtcp_sr_arrival_basis") or "") != "rtcp-pad-probe":
                missing.append(f"{camera_name} SR arrival pad-probe 없음")
            invalid_reason = str(metadata.get("rtcp_timing_invalid_reason") or "").strip()
            if invalid_reason:
                missing.append(f"{camera_name} RTCP timing invalid={invalid_reason}")
            signed_sr_pc_offset_s = _finite_float(metadata.get("rtcp_sr_pc_offset_s"))
            if signed_sr_pc_offset_s is None:
                missing.append(f"{camera_name} signed SR-PC offset 없음")
            elif abs(signed_sr_pc_offset_s) > self.panel._camera_sync_max_pc_offset_s():
                missing.append(
                    f"{camera_name} signed SR-PC offset="
                    f"{signed_sr_pc_offset_s * 1000.0:+.1f}ms"
                )
            previous_count = last_sr_counts.get(camera)
            if previous_count is not None and count < int(previous_count):
                missing.append(f"{camera_name} SR counter regressed {int(previous_count)}→{count}")
            age_s = current_sender_report_age_s(
                metadata,
                now_monotonic_s=now_mono_s,
                now_wall_s=now_wall_s,
            )
            if age_s is None:
                missing.append(f"{camera_name} SR age 없음")
            elif age_s > self.panel._camera_sync_max_sr_age_s():
                missing.append(f"{camera_name} SR stale {age_s:.2f}s")
            for key, label in (
                ("rtcp_camera_ntp_rate", "NTP clock"),
                ("rtcp_camera_media_rate", "RTP clock"),
            ):
                try:
                    rate = float(metadata.get(key))
                except (TypeError, ValueError):
                    missing.append(f"{camera_name} {label} 없음")
                    continue
                if not isfinite(rate) or abs(rate - 1.0) > self.panel._camera_sync_max_clock_rate_error():
                    missing.append(f"{camera_name} {label}={rate:.6f}")

        metrics = self.panel._camera_time_sync_metrics(
            left_metadata,
            right_metadata,
            now_wall_s=now_wall_s,
            now_mono_s=now_mono_s,
        )
        pair_delta_s = _finite_float(metrics.get("pair_delta_s"))
        if pair_delta_s is None:
            missing.append("좌우 RTCP/NTP pair 없음")
        elif abs(pair_delta_s) > self.panel._camera_sync_rtcp_pair_max_delta_s():
            missing.append(f"좌우 RTCP/NTP Δt={abs(pair_delta_s) * 1000.0:.1f}ms")
        receive_pair_delta_s = _finite_float(metrics.get("receive_pair_delta_s"))
        if receive_pair_delta_s is None:
            try:
                receive_pair_delta_s = _finite_float(
                    abs(float(right_metadata["timestamp_s"]) - float(left_metadata["timestamp_s"]))
                )
            except (KeyError, TypeError, ValueError):
                receive_pair_delta_s = None
        if receive_pair_delta_s is None:
            missing.append("좌우 실제 수신시각 pair 없음")
        elif receive_pair_delta_s > self.panel._camera_sync_receive_pair_max_delta_s():
            missing.append(f"좌우 실제 수신 Δt={receive_pair_delta_s * 1000.0:.1f}ms")
        max_sr_pc_offset_ms = _finite_float(metrics.get("max_sr_pc_offset_ms"))
        if max_sr_pc_offset_ms is None:
            missing.append("RTCP SR/PC UTC offset 없음")
        elif abs(max_sr_pc_offset_ms) > self.panel._camera_sync_max_pc_offset_s() * 1000.0:
            missing.append(f"RTCP SR/PC UTC offset={abs(max_sr_pc_offset_ms):.1f}ms")
        if missing:
            return False, "; ".join(missing)
        self.panel._camera_sync_last_sr_counts = observed_sr_counts
        return True, "RTCP/NTP·PC UTC·좌우 Δt 검증 완료"

    def _camera_sync_correction_observation(self) -> tuple[str, dict[str, object]]:
        """Build one bounded correction candidate from fresh post-STUS SRs.

        This method never sends a command.  It admits feedback only when both
        sides have current frames, fresh/new Sender Reports, stable clock rates,
        and a receive-close pair.  Missing evidence remains a warmup condition;
        an out-of-bound signed offset or target is a terminal fail-closed result.
        """

        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        if not isinstance(operation, dict) or not operation.get("active"):
            return "waiting", {"reason": "active STUS operation 없음"}
        if not str(operation.get("phase") or "").startswith("observing-attempt-"):
            return "waiting", {"reason": f"phase={operation.get('phase', 'unknown')}"}
        if not bool(getattr(self.panel, "camera_connected", False)):
            return "waiting", {"reason": "좌우 RTSP 스트림 재연결 대기"}

        now_mono_s = time.monotonic()
        now_wall_s = time.time()
        left_metadata, right_metadata = self.panel._clock_sync_pair_metadata(
            now_mono_s=now_mono_s
        )
        metadata_by_camera = {
            "left": dict(left_metadata or {}),
            "right": dict(right_metadata or {}),
        }
        baseline = dict(getattr(self.panel, "_camera_sync_stus_sr_baseline", {}) or {})
        attempt_index = int(operation.get("attempts_started") or 1)
        if int(operation.get("feedback_attempt_index") or 0) != attempt_index:
            operation["feedback_attempt_index"] = attempt_index
            operation["sr_feedback_samples_by_camera"] = {"left": [], "right": []}
        sample_store = operation.setdefault(
            "sr_feedback_samples_by_camera",
            {"left": [], "right": []},
        )
        history_window_s = self.panel._camera_sync_history_window_s()
        waiting: list[str] = []
        offsets: dict[str, float] = {}
        pending_samples: dict[str, dict[str, object]] = {}
        offset_statistics: dict[str, dict[str, float | int]] = {}
        sr_counts: dict[str, int] = {}
        sr_ages: dict[str, float] = {}
        pre_reconnect_generations = dict(
            operation.get("pre_reconnect_stream_generation_by_camera") or {}
        )
        ack_finished_by_camera = dict(
            operation.get("last_stus_ack_finished_utc_by_camera") or {}
        )
        for camera, metadata in metadata_by_camera.items():
            camera_gate_error_count = len(waiting)
            camera_name = self.panel._camera_name_ko(camera)
            frame_age_s = current_frame_observation_age_s(
                metadata,
                now_monotonic_s=now_mono_s,
            )
            if frame_age_s is None or frame_age_s > history_window_s:
                waiting.append(f"{camera_name} current frame 대기")
            previous_generation = pre_reconnect_generations.get(camera)
            if previous_generation is not None:
                try:
                    observed_generation = int(metadata.get("stream_generation"))
                except (TypeError, ValueError):
                    observed_generation = -1
                if observed_generation <= int(previous_generation):
                    waiting.append(
                        f"{camera_name} replacement stream generation 대기 "
                        f"({observed_generation}<={int(previous_generation)})"
                    )
            try:
                sr_count = max(0, int(metadata.get("rtcp_sr_received_count")))
            except (TypeError, ValueError):
                sr_count = 0
            sr_counts[camera] = sr_count
            if sr_count < self.panel._camera_sync_required_sr_count():
                waiting.append(f"{camera_name} SR {sr_count}/{self.panel._camera_sync_required_sr_count()}")
            baseline_count = baseline.get(camera)
            if baseline_count is not None and sr_count <= int(baseline_count):
                waiting.append(f"{camera_name} STUS 이후 새 SR 대기")
            sr_age_s = current_sender_report_age_s(
                metadata,
                now_monotonic_s=now_mono_s,
                now_wall_s=now_wall_s,
            )
            if sr_age_s is None or sr_age_s > self.panel._camera_sync_max_sr_age_s():
                waiting.append(f"{camera_name} fresh SR 대기")
            else:
                sr_ages[camera] = sr_age_s
            ack_finished_utc_s = _finite_float(ack_finished_by_camera.get(camera))
            sr_received_wall_s = _finite_float(metadata.get("rtcp_sr_received_wall_s"))
            if (
                ack_finished_utc_s is not None
                and (
                    sr_received_wall_s is None
                    or sr_received_wall_s <= ack_finished_utc_s
                )
            ):
                waiting.append(f"{camera_name} post-ACK SR 대기")
            for key, label in (
                ("rtcp_camera_ntp_rate", "NTP clock"),
                ("rtcp_camera_media_rate", "RTP clock"),
            ):
                rate = _finite_float(metadata.get(key))
                if rate is None or abs(rate - 1.0) > self.panel._camera_sync_max_clock_rate_error():
                    waiting.append(f"{camera_name} {label} 안정화 대기")
            offset_s = _finite_float(metadata.get("rtcp_sr_pc_offset_s"))
            if offset_s is None:
                waiting.append(f"{camera_name} signed SR-PC offset 대기")
            else:
                arrival_basis = str(metadata.get("rtcp_sr_arrival_basis") or "")
                sr_identity = metadata.get("rtcp_sr_ntp_raw")
                if arrival_basis != "rtcp-pad-probe":
                    waiting.append(f"{camera_name} SR arrival pad-probe 대기")
                elif sr_identity is None:
                    waiting.append(f"{camera_name} unique SR identity 대기")
                elif len(waiting) != camera_gate_error_count:
                    # Never let an old-generation, pre-ACK, stale, bad-rate or
                    # otherwise inadmissible SR become one of the actuator's
                    # three samples merely because it has a pad-probe identity.
                    pass
                else:
                    pending_samples[camera] = {
                        "sr_identity": sr_identity,
                        "sr_count": sr_count,
                        "offset_s": offset_s,
                        "observed_monotonic_s": now_mono_s,
                    }

        metrics = self.panel._camera_time_sync_metrics(
            metadata_by_camera["left"],
            metadata_by_camera["right"],
            now_wall_s=now_wall_s,
            now_mono_s=now_mono_s,
        )
        pair_delta_s = _finite_float(metrics.get("pair_delta_s"))
        receive_pair_delta_s = _finite_float(metrics.get("receive_pair_delta_s"))
        if pair_delta_s is None:
            waiting.append("좌우 RTCP/NTP pair 대기")
        if (
            receive_pair_delta_s is None
            or receive_pair_delta_s > self.panel._camera_sync_receive_pair_max_delta_s()
        ):
            waiting.append("좌우 receive-close pair 대기")
        if waiting:
            return "waiting", {
                "reason": "; ".join(waiting),
                "sr_count_by_camera": sr_counts,
                "sr_age_by_camera": sr_ages,
                "offset_statistics_by_camera": offset_statistics,
            }

        # Admit the Left/Right SR identities atomically only after every
        # per-side and receive-close pair boundary is valid. An invalid pair
        # cannot pre-fill two of the three actuator samples.
        for camera in ("left", "right"):
            sample = pending_samples.get(camera)
            if sample is None:
                waiting.append(f"{self.panel._camera_name_ko(camera)} admissible SR 대기")
                continue
            camera_samples = list(sample_store.get(camera) or [])
            if not any(
                row.get("sr_identity") == sample.get("sr_identity")
                for row in camera_samples
            ):
                camera_samples.append(dict(sample))
                camera_samples = camera_samples[-8:]
                sample_store[camera] = camera_samples
            recent_samples = camera_samples[-CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR:]
            if len(recent_samples) < CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR:
                waiting.append(
                    f"{self.panel._camera_name_ko(camera)} unique pad-probe SR "
                    f"{len(recent_samples)}/{CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR}"
                )
                continue
            values = [float(row["offset_s"]) for row in recent_samples]
            median_offset_s = median(values)
            span_s = max(values) - min(values)
            mad_s = median(abs(value - median_offset_s) for value in values)
            offset_statistics[camera] = {
                "unique_samples": len(recent_samples),
                "median_s": median_offset_s,
                "span_s": span_s,
                "mad_s": mad_s,
            }
            if (
                span_s > CAMERA_SYNC_CORRECTION_MAX_OFFSET_SPAN_S
                or mad_s > CAMERA_SYNC_CORRECTION_MAX_OFFSET_MAD_S
            ):
                waiting.append(
                    f"{self.panel._camera_name_ko(camera)} SR-PC feedback 분산 안정화 대기 "
                    f"(span={span_s * 1000.0:.1f}ms, MAD={mad_s * 1000.0:.1f}ms)"
                )
            else:
                offsets[camera] = median_offset_s
        if waiting:
            return "waiting", {
                "reason": "; ".join(waiting),
                "sr_count_by_camera": sr_counts,
                "sr_age_by_camera": sr_ages,
                "offset_statistics_by_camera": offset_statistics,
            }

        offset_limit_s = self.panel._camera_sync_max_pc_offset_s()
        pair_limit_s = self.panel._camera_sync_rtcp_pair_max_delta_s()
        needs_correction = any(abs(value) > offset_limit_s for value in offsets.values()) or (
            pair_delta_s is not None and abs(pair_delta_s) > pair_limit_s
        )
        if not needs_correction:
            return "converged", {
                "reason": "3 unique stable pad-probe SRs are within timing limits",
                "sr_pc_offset_by_camera": offsets,
                "offset_statistics_by_camera": offset_statistics,
            }
        previous = dict(operation.get("last_compensation_by_camera") or {})
        next_compensation: dict[str, float] = {}
        try:
            for camera in ("left", "right"):
                next_compensation[camera] = stus_closed_loop_compensation_s(
                    previous_compensation_s=previous.get(camera),
                    sr_pc_offset_s=offsets[camera],
                )
        except ValueError as exc:
            return "limit-failed", {
                "reason": str(exc),
                "sr_pc_offset_by_camera": offsets,
                "previous_compensation_by_camera": previous,
            }
        return "candidate", {
            "schema": "workstation-stus-correction-observation-v1",
            "attempt_from": int(operation.get("attempts_started") or 1),
            "observed_at_unix_s": now_wall_s,
            "observed_at_monotonic_s": now_mono_s,
            "sr_pc_offset_by_camera": offsets,
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
                "max_sr_pc_offset_s": offset_limit_s,
                "max_pair_delta_s": pair_limit_s,
                "required_unique_pad_probe_sr": CAMERA_SYNC_CORRECTION_REQUIRED_UNIQUE_SR,
                "max_feedback_span_s": CAMERA_SYNC_CORRECTION_MAX_OFFSET_SPAN_S,
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
            if self.panel._session_recording_active():
                fail_operation(
                    "STUS closed-loop correction aborted: recording owns the camera streams"
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
                            observation.get("sr_pc_offset_by_camera") or {}
                        ),
                        observation=dict(observation),
                    )
                )
                if requested:
                    self.panel._set_camera_sync_state(
                        CAMERA_SYNC_STATUS_WARMING,
                        "STUS closed-loop attempt 2/2: fresh signed SR-PC feedback applied; "
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
        if not self.panel.camera_connected:
            self.panel._camera_sync_stus_sr_baseline = {}
            self.panel._camera_sync_last_sr_counts = {}
            self.panel._set_camera_sync_state(
                CAMERA_SYNC_STATUS_NOT_VERIFIED,
                "STUS는 완료됐지만 RTSP 스트림이 없어 검증을 시작하지 않았습니다.",
            )
            finish = getattr(self.panel, "_finish_wiris_time_sync_operation", None)
            if callable(finish):
                finish("not-verified", "RTSP stream unavailable after STUS")
            return
        if self.panel._session_recording_active():
            reason = "세션 기록이 시작되어 RTSP 재연결을 건너뛰었습니다. 기록 종료 후 다시 동기화하세요."
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_NOT_VERIFIED, reason)
            self.panel._log(f"[시계] {reason}")
            finish = getattr(self.panel, "_finish_wiris_time_sync_operation", None)
            if callable(finish):
                finish("failed", reason)
            return
        # The camera RTSP sender can keep the NTP/RTP mapping created when the
        # session was opened even after STUS changes the camera wall clock.
        # Recreate only the local RTSP/GStreamer sessions so the next Sender
        # Reports use the new clock.  Do not toggle Ethernet Mode and do not
        # override the configured UDP/TCP transport.
        self.panel._camera_sync_stus_sr_baseline = {"left": 0, "right": 0}
        self.panel._camera_sync_last_sr_counts = {}
        operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        if isinstance(operation, dict) and operation.get("active"):
            operation["phase"] = f"observing-attempt-{int(operation.get('attempts_started') or 1)}"
            operation["feedback_attempt_index"] = int(operation.get("attempts_started") or 1)
            operation["sr_feedback_samples_by_camera"] = {"left": [], "right": []}
            pre_reconnect_generations: dict[str, int] = {}
            for camera in ("left", "right"):
                pane = getattr(self.panel, f"{camera}_pane", None)
                try:
                    pre_reconnect_generations[camera] = int(pane.stream_generation)
                except (AttributeError, TypeError, ValueError):
                    continue
            operation["pre_reconnect_stream_generation_by_camera"] = pre_reconnect_generations
        self.panel._camera_sync_deadline_monotonic_s = time.monotonic() + self.panel._camera_sync_warmup_timeout_s()
        self.panel._camera_sync_transport_override = None
        set_interval = getattr(self.panel._camera_sync_timer, "setInterval", None)
        if callable(set_interval):
            set_interval(CAMERA_SYNC_WARMUP_POLL_INTERVAL_MS)
        self.panel._set_camera_sync_state(
            CAMERA_SYNC_STATUS_RECONNECTING,
            "STUS 완료: Ethernet Mode는 유지하고 로컬 RTSP 수신기만 재연결합니다.",
        )
        started = self.panel._start_camera_streams(
            enable_detection=self.panel.yolo_enabled,
            switch_to_depth=self.panel.yolo_enabled,
        )
        if started is False:
            self.panel._camera_sync_timer.stop()
            reason = "STUS ACK는 완료됐지만 replacement RTSP stream 요청이 실패했습니다."
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_FAILED, reason)
            finish = getattr(self.panel, "_finish_wiris_time_sync_operation", None)
            if callable(finish):
                finish("failed", reason)
            return
        self.panel._camera_sync_timer.start()

    def _clock_sync_side_summary(self, camera: str, metadata: dict[str, object], now_wall_s: float, now_mono_s: float) -> tuple[str, float | None]:
        if metadata.get("rtcp_timing_invalid_reason"):
            return f"{self.panel._camera_name_ko(camera)}: RTCP/NTP 무효(SR-PC offset 초과)", None
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
        ]
        offsets_ms: dict[str, float] = {}
        sr_offsets_ms: dict[str, float] = {}
        transit_delays_ms: dict[str, float] = {}
        for camera, metadata, explicit_receive_s, rtcp_s in (
            ("left", left_metadata, left_receive_mono_s, left_rtcp_s),
            ("right", right_metadata, right_receive_mono_s, right_rtcp_s),
        ):
            if rtcp_s is None or metadata.get("rtcp_timing_invalid_reason"):
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
        elif invalid_cameras:
            status = "absolute-rtcp-invalid"
            warning = (
                f"좌우 RTCP/NTP pair는 {abs(pair_delta_s) * 1000.0:.1f} ms 이내지만, "
                f"{', '.join(invalid_cameras)} Sender Report NTP의 PC UTC 기준이 500 ms 이상 어긋났습니다. "
                "절대 UTC 기준으로 사용할 수 없습니다."
            )
        elif max_offset_ms is not None and max_offset_ms > self.panel._camera_sync_max_pc_offset_s() * 1000.0:
            status = "pc-offset-failed"
            warning = (
                f"RTCP/NTP와 PC UTC 차이가 {self.panel._camera_sync_max_pc_offset_s() * 1000.0:.0f} ms를 초과했습니다. "
                "카메라 GDTI와 RTCP 매핑을 검증해야 합니다."
            )
        else:
            status = "within-threshold-unverified"
            warning = "좌우/PC 오차는 임계값 이내지만 RTCP/NTP의 UTC 기준은 아직 검증되지 않았습니다."
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
            if compensation_method == "rtcp-sr-closed-loop":
                target_detail = (
                    "target lead = previous lead - median signed SR-PC offset; "
                    "3 unique stable rtcp-pad-probe SRs"
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
