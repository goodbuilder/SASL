from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping

from PyQt5 import QtCore


class StatusSeverity(str, Enum):
    UNKNOWN = "unknown"
    INFO = "info"
    OK = "ok"
    WARN = "warn"
    BAD = "bad"


@dataclass(frozen=True)
class RuntimeStatus:
    key: str
    label: str
    state: str
    severity: StatusSeverity
    detail: str = ""
    source: str = ""
    age_s: float | None = None
    clock: str = ""
    observed_monotonic_s: float = 0.0


class StatusModel(QtCore.QObject):
    """Single presentation-facing status source.

    The model stores evidence projections; it does not create telemetry or
    reinterpret measurement geometry. Each record retains its producer and
    timestamp basis so labels cannot become an implicit source of truth.
    """

    changed = QtCore.pyqtSignal(object)

    def __init__(self, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self._records: dict[str, RuntimeStatus] = {}

    def set_status(
        self,
        key: str,
        label: str,
        state: str,
        severity: StatusSeverity | str,
        *,
        detail: str = "",
        source: str = "",
        age_s: float | None = None,
        clock: str = "",
        observed_monotonic_s: float | None = None,
    ) -> RuntimeStatus:
        level = severity if isinstance(severity, StatusSeverity) else StatusSeverity(str(severity))
        if age_s is not None:
            age_s = max(0.0, float(age_s))
        record = RuntimeStatus(
            key=str(key),
            label=str(label),
            state=str(state),
            severity=level,
            detail=str(detail),
            source=str(source),
            age_s=age_s,
            clock=str(clock),
            observed_monotonic_s=(
                time.monotonic() if observed_monotonic_s is None else float(observed_monotonic_s)
            ),
        )
        previous = self._records.get(record.key)
        same_semantics = (
            previous is not None
            and previous.key == record.key
            and previous.label == record.label
            and previous.state == record.state
            and previous.severity == record.severity
            and previous.detail == record.detail
            and previous.source == record.source
            and previous.clock == record.clock
            and (
                (previous.age_s is None and record.age_s is None)
                or (
                    previous.age_s is not None
                    and record.age_s is not None
                    and round(previous.age_s, 2) == round(record.age_s, 2)
                )
            )
        )
        if not same_semantics:
            self._records[record.key] = record
            self.changed.emit(self.snapshot())
            return record
        return previous

    def record(self, key: str) -> RuntimeStatus | None:
        return self._records.get(str(key))

    def snapshot(self) -> dict[str, RuntimeStatus]:
        return dict(self._records)

    def as_mapping(self) -> Mapping[str, RuntimeStatus]:
        return self._records


def _safe_call(target: object, name: str, *args: object, default: Any = None, **kwargs: object) -> Any:
    method = getattr(target, name, None)
    if not callable(method):
        return default
    try:
        return method(*args, **kwargs)
    except Exception:
        return default


def _sample_age(samples: object, now_s: float) -> float | None:
    try:
        if not samples:
            return None
        value = list(samples)[-1]
        return max(0.0, now_s - float(value))
    except (TypeError, ValueError, IndexError):
        return None


class BackendStatusAdapter(QtCore.QObject):
    """Derive status from backend evidence at a bounded polling cadence."""

    def __init__(self, backend: object, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.model = StatusModel(self)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self.refresh)
        self._last_snapshot: dict[str, RuntimeStatus] = {}

    def start(self) -> None:
        self.refresh()
        self.timer.start()

    def stop(self) -> None:
        self.timer.stop()

    def refresh(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        now_s = time.monotonic()
        self._refresh_camera(panel, "left", "CAM L", now_s)
        self._refresh_camera(panel, "right", "CAM R", now_s)
        self._refresh_pixhawk(panel, "left", "P1", now_s)
        self._refresh_pixhawk(panel, "right", "P2", now_s)
        self._refresh_gnss(panel, now_s)
        self._refresh_detection(panel, now_s)
        self._refresh_synchronization(panel, now_s)
        self._refresh_measurement(panel, now_s)
        self._refresh_recording(panel, now_s)
        self._refresh_fps(panel, now_s)

    def _refresh_camera(self, panel: object, side: str, label: str, now_s: float) -> None:
        panes = getattr(panel, f"{side}_pane", None)
        pane_state = str(getattr(panes, "current_status", "중지됨") or "중지됨")
        connected = bool(getattr(panel, "camera_connected", False))
        frame_times = getattr(panel, "camera_frame_times", {}) or {}
        age_s = _sample_age(frame_times.get(side, []), now_s)
        if age_s is not None and age_s < 2.0:
            state, severity, detail = "LIVE", StatusSeverity.OK, f"frame age {age_s:.2f} s"
        elif "error" in pane_state.lower() or "실패" in pane_state or "오류" in pane_state:
            state, severity, detail = "ERROR", StatusSeverity.BAD, pane_state
        elif connected or pane_state not in {"중지됨", ""}:
            state, severity, detail = "STARTING", StatusSeverity.WARN, pane_state or "first frame 대기"
        else:
            state, severity, detail = "STOPPED", StatusSeverity.UNKNOWN, "no frame evidence"
        self.model.set_status(
            f"camera.{side}",
            label,
            state,
            severity,
            detail=detail,
            source=f"CameraPane[{side}].current_status + frame_changed",
            age_s=age_s,
            clock="app_receive_monotonic",
            observed_monotonic_s=now_s,
        )

    def _refresh_pixhawk(self, panel: object, side: str, label: str, now_s: float) -> None:
        mavlink_sides = set(_safe_call(panel, "_mavlink_open_sides", default=()) or ())
        attitude_times = getattr(panel, "pixhawk_attitude_times", {}) or {}
        age_s = _sample_age(attitude_times.get(side, []), now_s)
        transport = "MAVLink" if side in mavlink_sides else "telemetry"
        if age_s is not None and float(age_s) < 2.0:
            state, severity = "LIVE", StatusSeverity.OK
            detail = f"{transport} attitude age {float(age_s):.2f} s"
        elif side in mavlink_sides:
            state, severity = "CONNECTED", StatusSeverity.WARN
            detail = "MAVLink heartbeat; attitude sample 대기 또는 stale"
        else:
            state, severity = "DISCONNECTED", StatusSeverity.UNKNOWN
            detail = "heartbeat/sample evidence 없음"
        extrinsics = getattr(panel, "latest_extrinsics", {}) or {}
        extrinsic = extrinsics.get(side) if isinstance(extrinsics, dict) else None
        if extrinsic is not None:
            detail += (
                f"; rpy={float(getattr(extrinsic, 'roll_deg', 0.0)):.2f}/"
                f"{float(getattr(extrinsic, 'pitch_deg', 0.0)):.2f}/"
                f"{float(getattr(extrinsic, 'yaw_deg', 0.0)):.2f} deg"
                f"; PX4 t={getattr(extrinsic, 'px4_timestamp_s', '--')}"
            )
        self.model.set_status(
            f"pixhawk.{side}",
            label,
            state,
            severity,
            detail=detail,
            source="MAVLink heartbeat + attitude sample",
            age_s=None if age_s is None else float(age_s),
            clock="app_receive_monotonic",
            observed_monotonic_s=now_s,
        )

    def _refresh_gnss(self, panel: object, now_s: float) -> None:
        manager = getattr(panel, "gnss_operation_manager", None)
        state_value = getattr(getattr(manager, "state", None), "value", None)
        state_value = str(state_value or "").upper()
        gps_states = getattr(panel, "latest_gps", {}) or {}
        baseline = getattr(panel, "latest_rtk_baseline", None)
        receiver_details = []
        receiver_ages = []
        receiver_labels: list[str] = []
        for side, label in (("left", "P1"), ("right", "P2")):
            gps = gps_states.get(side) if isinstance(gps_states, dict) else None
            if gps is None:
                receiver_details.append(f"{label}=NO DATA")
                continue
            age_s = max(0.0, now_s - float(getattr(gps, "timestamp_s", now_s)))
            receiver_ages.append(age_s)
            fix = str(getattr(gps, "rtk_label", "UNKNOWN") or "UNKNOWN")
            receiver_labels.append(fix)
            fix_type = getattr(gps, "fix_type", None)
            sats = getattr(gps, "satellites_used", None)
            eph = getattr(gps, "eph_m", None)
            epv = getattr(gps, "epv_m", None)
            receiver_details.append(
                f"{label}={fix}(fix={fix_type if fix_type is not None else '--'}, "
                f"sats={sats if sats is not None else '--'}, "
                f"eph={eph if eph is not None else '--'}m, "
                f"epv={epv if epv is not None else '--'}m, age={age_s:.2f}s)"
            )
        baseline_age = None
        baseline_detail = "baseline NED=NO DATA"
        if baseline is not None:
            left_age = float(getattr(baseline, "left_age_s", 0.0))
            right_age = float(getattr(baseline, "right_age_s", 0.0))
            baseline_age = max(0.0, left_age, right_age)
            baseline_detail = (
                "baseline NED="
                f"{float(getattr(baseline, 'north_m', 0.0)):+.3f}/"
                f"{float(getattr(baseline, 'east_m', 0.0)):+.3f}/"
                f"{float(getattr(baseline, 'down_m', 0.0)):+.3f} m, "
                f"length={float(getattr(baseline, 'baseline_m', 0.0)):.3f} m, "
                f"quality={getattr(baseline, 'fix_quality', 'UNKNOWN')}, "
                f"age P1/P2={left_age:.2f}/{right_age:.2f}s"
            )
            projection = getattr(panel, "latest_p1_body_frd_baseline", None)
            if projection is None:
                baseline_detail += "; P1 body FRD projection=WAITING"
            else:
                baseline_detail += (
                    "; P1→P2 @ P1 body FRD F/R/D="
                    f"{float(getattr(projection, 'forward_m', 0.0)):+.3f}/"
                    f"{float(getattr(projection, 'right_m', 0.0)):+.3f}/"
                    f"{float(getattr(projection, 'down_m', 0.0)):+.3f} m"
                )
        readiness = _safe_call(manager, "readiness", default=None)
        readiness_ready = getattr(readiness, "ready", None)
        readiness_text = ""
        if readiness_ready is not None:
            readiness_text = f"; readiness={'READY' if bool(readiness_ready) else 'BLOCKED'}"
        has_receiver = bool(receiver_ages)
        fix_quality = str(getattr(baseline, "fix_quality", "") or "").upper() if baseline is not None else ""
        if state_value in {"FAILED", "ERROR", "FAULT", "INVALIDATED"}:
            state, severity = "ERROR", StatusSeverity.BAD
        elif baseline is not None and fix_quality == "GOOD":
            state, severity = "FIX", StatusSeverity.OK
        elif baseline is not None and "RTK_FLOAT" in receiver_labels:
            state, severity = "FLOAT", StatusSeverity.WARN
        elif baseline is not None and receiver_labels:
            state, severity = "NO FIX", StatusSeverity.WARN
        elif has_receiver:
            state, severity = "NO FIX", StatusSeverity.WARN
        elif state_value in {"READY", "RUNNING", "CONNECTED", "ACTIVE"}:
            state, severity = "READY", StatusSeverity.OK
        elif state_value in {"DEGRADED", "CONNECTING", "WAITING", "STARTING"}:
            state, severity = "DEGRADED", StatusSeverity.WARN
        else:
            state, severity = "NO DATA", StatusSeverity.UNKNOWN
        detail = "; ".join(
            part for part in (state_value or "operation state 대기", *receiver_details, baseline_detail + readiness_text) if part
        )
        self.model.set_status(
            "gnss",
            "RTK / GNSS",
            state,
            severity,
            detail=detail,
            source="GnssOperationManager + GpsPositionState + RtkBaselineEstimate",
            age_s=baseline_age if baseline_age is not None else (max(receiver_ages) if receiver_ages else None),
            clock="GPS app_receive_monotonic; baseline NED",
            observed_monotonic_s=now_s,
        )

    def _refresh_detection(self, panel: object, now_s: float) -> None:
        enabled = bool(getattr(panel, "yolo_enabled", False))
        stats = getattr(panel, "camera_stream_stats", {}) or {}
        inferred = []
        for side in ("left", "right"):
            value = (stats.get(side) or {}).get("inferred_hz")
            if value not in (None, ""):
                inferred.append(float(value))
        observations = []
        for side in ("left", "right"):
            observation = getattr(panel, f"latest_{side}", None)
            if observation is not None:
                observations.append((side, observation))
        target_text = "target=none"
        target_age = None
        if observations:
            labels = []
            ages = []
            for side, observation in observations:
                age = _safe_call(observation, "age_s", default=None)
                if age is None:
                    try:
                        age = max(0.0, now_s - float(getattr(observation, "timestamp_s")))
                    except (TypeError, ValueError):
                        age = None
                if age is not None:
                    ages.append(float(age))
                class_name = str(getattr(observation, "class_name", "") or getattr(observation, "class_id", "?"))
                labels.append(f"{side}={class_name}")
            target_text = "target=" + ",".join(labels)
            target_age = max(ages) if ages else None
        if enabled and inferred and max(inferred) > 0.0:
            state, severity, detail = "RUNNING", StatusSeverity.OK, f"inference {sum(inferred) / len(inferred):.1f} Hz"
        elif enabled:
            state, severity, detail = "STARTING", StatusSeverity.WARN, "inference output 대기"
        else:
            state, severity, detail = "DISABLED", StatusSeverity.UNKNOWN, "command intent: disabled"
        detail = f"{detail}; {target_text}"
        self.model.set_status(
            "detection",
            "DET",
            state,
            severity,
            detail=detail,
            source="YoloStreamWorker stats/detection signal",
            age_s=target_age,
            clock="worker/app receive",
            observed_monotonic_s=now_s,
        )

    def _refresh_synchronization(self, panel: object, now_s: float) -> None:
        raw_state = str(getattr(panel, "camera_sync_status", "not-verified") or "not-verified").lower()
        state_map = {
            "verified": ("SYNCED", StatusSeverity.OK),
            "warming": ("WARMING", StatusSeverity.WARN),
            "reconnecting": ("RECONNECTING", StatusSeverity.WARN),
            "failed": ("ERROR", StatusSeverity.BAD),
            "not-verified": ("UNVERIFIED", StatusSeverity.WARN),
        }
        state, severity = state_map.get(raw_state, ("UNKNOWN", StatusSeverity.UNKNOWN))
        reason = str(getattr(panel, "camera_sync_reason", "sync evidence 대기") or "sync evidence 대기")
        pair_delta = getattr(panel, "selected_runtime_frame_pair_delta_s", None)
        pair_basis = str(getattr(panel, "selected_runtime_frame_pair_basis", "") or "")
        rtc_delta = getattr(panel, "selected_runtime_frame_pair_camera_rtc_delta_s", None)
        pair_parts = []
        if pair_delta is not None:
            pair_parts.append(f"selected pair Δ={float(pair_delta) * 1000.0:.1f} ms")
        if pair_basis:
            pair_parts.append(f"basis={pair_basis}")
        if rtc_delta is not None:
            pair_parts.append(f"RTCP/NTP Δ={float(rtc_delta) * 1000.0:.1f} ms")
        detail = "; ".join((reason, *pair_parts))
        self.model.set_status(
            "synchronization",
            "SYNC",
            state,
            severity,
            detail=detail,
            source="camera_sync_status + selected runtime frame pair",
            clock="app_receive_monotonic; RTCP/NTP when present",
            observed_monotonic_s=now_s,
        )

    def _refresh_measurement(self, panel: object, now_s: float) -> None:
        quality = str(getattr(panel, "last_distance_quality", "BAD") or "BAD").upper()
        detail = str(getattr(panel, "last_distance_status", "measurement evidence 대기") or "")
        if "대기" in detail or "WAIT" in detail.upper():
            state, severity = "WAITING", StatusSeverity.WARN
        elif quality in {"GOOD", "VALID"}:
            state, severity = "VALID", StatusSeverity.OK
        elif quality in {"DEGRADED", "WARN", "WARNING"}:
            state, severity = "DEGRADED", StatusSeverity.WARN
        else:
            state, severity = "REJECTED", StatusSeverity.BAD
        self.model.set_status(
            "measurement",
            "MEAS",
            state,
            severity,
            detail=detail,
            source="MeasurementAttempt/controller result",
            clock="attempt selected timestamps",
            observed_monotonic_s=now_s,
        )

    def _refresh_recording(self, panel: object, now_s: float) -> None:
        save_thread = getattr(panel, "_session_save_thread", None)
        retry = bool(getattr(panel, "_session_save_retry_required", False))
        active = bool(_safe_call(panel, "_session_recording_active", default=False))
        if save_thread is not None:
            state, severity, detail = "FINALIZING", StatusSeverity.WARN, "session save worker running"
        elif retry:
            state, severity, detail = "RECOVERABLE", StatusSeverity.WARN, "retryable spool pending"
        elif bool(getattr(panel, "_session_save_finalized", False)):
            state, severity, detail = "SAVED", StatusSeverity.OK, "session artifacts finalized"
        elif active:
            state, severity, detail = "ACTIVE", StatusSeverity.OK, "session/spool active"
        else:
            state, severity, detail = "IDLE", StatusSeverity.UNKNOWN, "no active session"
        session_id = str(getattr(panel, "recording_session_id", "") or "")
        if session_id:
            detail = f"{detail}; session={session_id}"
        rows = getattr(panel, "distance_recording_rows", None)
        pair_count = None
        try:
            pair_count = len(rows) if rows is not None else None
        except TypeError:
            pair_count = None
        if pair_count is not None:
            detail = f"{detail}; pairs={pair_count}"
        self.model.set_status(
            "recording",
            "REC",
            state,
            severity,
            detail=detail,
            source="SpoolRows/session manifest/save worker",
            clock="session monotonic anchor",
            observed_monotonic_s=now_s,
        )

    def _refresh_fps(self, panel: object, now_s: float) -> None:
        stats = getattr(panel, "camera_stream_stats", {}) or {}
        values = []
        for side in ("left", "right"):
            value = (stats.get(side) or {}).get("decoded_hz")
            if value not in (None, ""):
                values.append(float(value))
        detail = " / ".join(f"{value:.1f} Hz" for value in values) if values else "--"
        self.model.set_status(
            "fps",
            "FPS",
            "AVAILABLE" if values else "NO DATA",
            StatusSeverity.INFO if values else StatusSeverity.UNKNOWN,
            detail=detail,
            source="YoloStreamWorker stats_ready",
            clock="worker/app receive",
            observed_monotonic_s=now_s,
        )
