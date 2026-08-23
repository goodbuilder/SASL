from __future__ import annotations

import math
import time
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum
from typing import Any, Iterator, Mapping

from PyQt5 import QtCore

from Workstation.workstation_core.gnss_monitor import (
    NtripHealthState,
    evaluate_ntrip_health,
)


# LRF reads are on-demand (a manual Alignment button click), not a
# continuous 1 Hz poll, so a reading stays "LIVE" for a while after the
# query that produced it rather than going stale within one poll period.
LRF_STALE_AFTER_S = 60.0


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
        self._batch_depth = 0
        self._batch_changed = False

    @contextmanager
    def batch(self) -> Iterator[None]:
        """Coalesce multiple status writes into one presentation signal."""

        self._batch_depth += 1
        try:
            yield
        finally:
            self._batch_depth -= 1
            if self._batch_depth == 0 and self._batch_changed:
                self._batch_changed = False
                self.changed.emit(self.snapshot())

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
            if self._batch_depth:
                self._batch_changed = True
            else:
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

    polled = QtCore.pyqtSignal(object)

    def __init__(self, backend: object, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.model = StatusModel(self)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self.refresh)

    def start(self) -> None:
        self.refresh()
        self.timer.start()

    def stop(self) -> None:
        self.timer.stop()

    def refresh(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        now_s = time.monotonic()
        failures: list[str] = []
        sections = (
            ("camera.left", self._refresh_camera, (panel, "left", "CAM L", now_s)),
            ("camera.right", self._refresh_camera, (panel, "right", "CAM R", now_s)),
            ("lrf", self._refresh_lrf, (panel, now_s)),
            ("pixhawk.left", self._refresh_pixhawk, (panel, "left", "P1", now_s)),
            ("pixhawk.right", self._refresh_pixhawk, (panel, "right", "P2", now_s)),
            ("gnss.base", self._refresh_base_gnss, (panel, now_s)),
            ("ntrip", self._refresh_ntrip, (panel, now_s)),
            ("gnss", self._refresh_gnss, (panel, now_s)),
            ("detection", self._refresh_detection, (panel, now_s)),
            ("synchronization", self._refresh_synchronization, (panel, now_s)),
            ("measurement", self._refresh_measurement, (panel, now_s)),
            ("recording", self._refresh_recording, (panel, now_s)),
            ("fps", self._refresh_fps, (panel, now_s)),
        )
        with self.model.batch():
            for name, refresher, args in sections:
                try:
                    refresher(*args)
                except Exception as exc:  # noqa: BLE001 - isolate independent status projections.
                    failures.append(f"{name}: {type(exc).__name__}: {exc}")
            if failures or self.model.record("status.adapter") is not None:
                self.model.set_status(
                    "status.adapter",
                    "STATUS",
                    "DEGRADED" if failures else "OK",
                    StatusSeverity.WARN if failures else StatusSeverity.OK,
                    detail="; ".join(failures) if failures else "all status sections refreshed",
                    source="BackendStatusAdapter.refresh section isolation",
                    clock="app_receive_monotonic",
                    observed_monotonic_s=now_s,
                )
        self.polled.emit(self.model.snapshot())

    def _refresh_camera(self, panel: object, side: str, label: str, now_s: float) -> None:
        panes = getattr(panel, f"{side}_pane", None)
        pane_state = str(getattr(panes, "current_status", "중지됨") or "중지됨")
        connected = bool(getattr(panel, "camera_connected", False))
        frame_times = getattr(panel, "camera_frame_times", {}) or {}
        age_s = _sample_age(frame_times.get(side, []), now_s)
        if "error" in pane_state.lower() or "실패" in pane_state or "오류" in pane_state:
            state, severity, detail = "ERROR", StatusSeverity.BAD, pane_state
        elif age_s is not None and age_s < 2.0:
            state, severity, detail = "LIVE", StatusSeverity.OK, f"frame age {age_s:.2f} s"
        elif age_s is not None:
            state, severity, detail = "STALE", StatusSeverity.WARN, f"last frame age {age_s:.2f} s"
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

    def _refresh_lrf(self, panel: object, now_s: float) -> None:
        values = getattr(panel, "latest_wiris_lrf", {}) or {}
        values = values if isinstance(values, Mapping) else {}
        for side, label in (("left", "LRF L"), ("right", "LRF R")):
            row = values.get(side)
            row = row if isinstance(row, Mapping) else {}
            distance_m = row.get("distance_m")
            last_good_s = row.get("last_good_s")
            last_attempt_s = row.get("last_attempt_s")
            try:
                distance_m = float(distance_m)
                if not math.isfinite(distance_m) or distance_m < 0.0:
                    raise ValueError
            except (TypeError, ValueError):
                distance_m = None
            try:
                good_age_s = None if last_good_s is None else max(0.0, now_s - float(last_good_s))
            except (TypeError, ValueError):
                good_age_s = None
            try:
                attempt_age_s = None if last_attempt_s is None else max(0.0, now_s - float(last_attempt_s))
            except (TypeError, ValueError):
                attempt_age_s = None
            error = str(row.get("last_error") or "").strip()
            response = str(row.get("response") or "").strip()
            if distance_m is None and not error:
                state, severity = "WAITING", StatusSeverity.UNKNOWN
                age_s = attempt_age_s
            elif good_age_s is not None and good_age_s > LRF_STALE_AFTER_S:
                state, severity = "STALE", StatusSeverity.WARN
                age_s = good_age_s
            elif error:
                state, severity = "ERROR", StatusSeverity.BAD
                age_s = attempt_age_s if attempt_age_s is not None else good_age_s
            else:
                state, severity = "LIVE", StatusSeverity.OK
                age_s = good_age_s
            detail_parts: list[str] = []
            if distance_m is not None:
                detail_parts.append(f"distance={distance_m:.3f} m")
            if response:
                detail_parts.append(f"response={response}")
            if error:
                detail_parts.append(f"error={error}")
            if not detail_parts:
                detail_parts.append("GLRD response 대기")
            self.model.set_status(
                f"lrf.{side}",
                label,
                state,
                severity,
                detail="; ".join(detail_parts),
                source="WIRIS GLRD / WirisLrfPollWorker",
                age_s=age_s,
                clock="app receive monotonic; GLRD distance metres",
                observed_monotonic_s=now_s,
            )

    def _refresh_base_gnss(self, panel: object, now_s: float) -> None:
        """Project only the correction-source/Base receiver lifecycle.

        This status deliberately excludes P1/P2 rover fixes and their NED/FRD
        baseline. Those remain part of the aggregate ``gnss`` readiness and
        Pixhawk/Alignment evidence.
        """

        manager = getattr(panel, "gnss_operation_manager", None)
        state = str(getattr(getattr(manager, "state", None), "value", "DISCONNECTED") or "DISCONNECTED").upper()
        config = getattr(manager, "config", None)
        mode = str(getattr(getattr(config, "operation_mode", None), "value", "UNKNOWN") or "UNKNOWN")
        serial_connected = bool(getattr(panel, "gnss_base_serial_connected", False))
        ntrip_connected = bool(getattr(panel, "gnss_network_connected", False))
        stats = getattr(panel, "gnss_base_rtcm_stats", {}) or {}
        stats = stats if isinstance(stats, Mapping) else {}
        last_frame_s = stats.get("last_frame_s")
        try:
            frame_age_s = None if last_frame_s is None else max(0.0, now_s - float(last_frame_s))
        except (TypeError, ValueError):
            frame_age_s = None
        frames = int(stats.get("frames") or 0)
        maximum_age_s = float(
            getattr(getattr(config, "readiness", None), "maximum_correction_age_s", 2.0) or 2.0
        )
        survey = getattr(panel, "latest_gnss_base_survey_in", None)
        survey = survey if isinstance(survey, Mapping) else {}

        if state in {"FAULT", "INVALIDATED", "ERROR", "FAILED"}:
            display_state, severity = "ERROR", StatusSeverity.BAD
        elif frame_age_s is not None and frame_age_s <= maximum_age_s:
            display_state, severity = "RTCM ACTIVE", StatusSeverity.OK
        elif state == "BASE_SURVEYING" or survey.get("active"):
            display_state, severity = "SURVEYING", StatusSeverity.WARN
        elif ntrip_connected:
            display_state, severity = "NTRIP ACTIVE", StatusSeverity.WARN
        elif serial_connected:
            display_state, severity = "CONNECTED", StatusSeverity.WARN
        else:
            display_state, severity = "DISCONNECTED", StatusSeverity.UNKNOWN

        detail_parts = [
            f"mode={mode}",
            f"operation={state}",
            f"serial={'connected' if serial_connected else 'disconnected'}",
            f"NTRIP={'connected' if ntrip_connected else 'disconnected'}",
            f"RTCM frames={frames}",
        ]
        if frame_age_s is not None:
            detail_parts.append(f"RTCM age={frame_age_s:.2f}s")
        if survey:
            detail_parts.append(
                "Survey-in="
                f"{'valid' if survey.get('valid') else 'active' if survey.get('active') else 'not-running'}"
                f"({survey.get('duration_s', 0)}s, {survey.get('mean_accuracy_m', '--')}m, "
                f"obs={survey.get('observations', '--')})"
            )
        self.model.set_status(
            "gnss.base",
            "BASE",
            display_state,
            severity,
            detail="; ".join(detail_parts),
            source="GnssOperationManager + Base serial/NTRIP/RTCM/NAV-SVIN evidence",
            age_s=frame_age_s,
            clock="app_receive_monotonic; Base WGS84/Survey ECEF contracts remain separate",
            observed_monotonic_s=now_s,
        )

    def _refresh_ntrip(self, panel: object, now_s: float) -> None:
        manager = getattr(panel, "gnss_operation_manager", None)
        config = getattr(manager, "config", None)
        mode = str(
            getattr(getattr(config, "operation_mode", None), "value", "UNKNOWN")
            or "UNKNOWN"
        ).upper()
        network_mode = mode in {"NETWORK_ASSISTED_FIXED_BASE", "NETWORK_RTK_DIRECT"}
        network_config = getattr(config, "network", None)
        readiness_config = getattr(config, "readiness", None)
        base_stats = getattr(panel, "gnss_base_rtcm_stats", {}) or {}
        base_stats = base_stats if isinstance(base_stats, Mapping) else {}
        stream_stats = getattr(panel, "gnss_network_stats", None)
        phase = str(getattr(panel, "gnss_network_phase", "DISCONNECTED") or "DISCONNECTED")
        health = evaluate_ntrip_health(
            network_mode=network_mode,
            connected=bool(getattr(panel, "gnss_network_connected", False)),
            running=getattr(panel, "gnss_network_thread", None) is not None,
            phase=phase,
            connected_at_s=getattr(stream_stats, "connected_at_monotonic_s", None),
            last_chunk_s=getattr(stream_stats, "last_chunk_monotonic_s", None),
            last_rtcm_s=base_stats.get("last_frame_s"),
            now_s=now_s,
            transport_stale_after_s=float(getattr(network_config, "read_timeout_s", 10.0) or 10.0),
            correction_stale_after_s=float(
                getattr(readiness_config, "maximum_correction_age_s", 3.0) or 3.0
            ),
            bytes_received=getattr(stream_stats, "bytes_received", 0),
            chunks_received=getattr(stream_stats, "chunks_received", 0),
            reconnect_count=getattr(panel, "gnss_network_reconnect_count", 0),
            last_error=str(getattr(panel, "gnss_network_last_error", "") or ""),
        )
        severity = {
            NtripHealthState.DISABLED: StatusSeverity.INFO,
            NtripHealthState.DISCONNECTED: StatusSeverity.UNKNOWN,
            NtripHealthState.CONNECTING: StatusSeverity.WARN,
            NtripHealthState.CONNECTED_NO_DATA: StatusSeverity.WARN,
            NtripHealthState.STREAMING: StatusSeverity.WARN,
            NtripHealthState.RTCM_ACTIVE: StatusSeverity.OK,
            NtripHealthState.RTCM_STALE: StatusSeverity.BAD,
            NtripHealthState.STALE: StatusSeverity.BAD,
            NtripHealthState.RECONNECTING: StatusSeverity.WARN,
            NtripHealthState.ERROR: StatusSeverity.BAD,
        }[health.state]
        chunks = health.chunks_received
        byte_rate = (
            health.bytes_received / health.session_age_s
            if health.session_age_s is not None and health.session_age_s > 0.0
            else 0.0
        )
        detail_parts = [
            f"phase={phase}",
            f"connected={'yes' if bool(getattr(panel, 'gnss_network_connected', False)) else 'no'}",
            f"chunks={chunks}",
            f"bytes={health.bytes_received}",
            f"rate={byte_rate:.1f} B/s",
        ]
        if health.transport_age_s is not None:
            detail_parts.append(f"last chunk={health.transport_age_s:.2f}s")
        else:
            detail_parts.append("last chunk=--")
        if health.rtcm_age_s is not None:
            detail_parts.append(f"RTCM age={health.rtcm_age_s:.2f}s")
        else:
            detail_parts.append("RTCM age=--")
        # GGA is the rover-position keep-alive a VRS/session caster (e.g.
        # NGII) requires to keep the stream open at all -- surfacing it here
        # is what tells "we never confirmed a live position" (last GGA=never)
        # apart from "the caster just stopped sending RTCM" (RTCM age above).
        last_gga_s = getattr(stream_stats, "last_gga_monotonic_s", None)
        if last_gga_s is not None:
            detail_parts.append(f"last GGA={max(0.0, now_s - last_gga_s):.2f}s")
        else:
            detail_parts.append("last GGA=never")
        detail_parts.append(f"GGA sent={getattr(stream_stats, 'gga_sent_count', 0)}")
        caster_host = str(getattr(network_config, "caster_host", "") or "")
        mountpoint = str(getattr(network_config, "mountpoint", "") or "")
        if caster_host or mountpoint:
            detail_parts.append(f"caster={caster_host or '--'}/{mountpoint or '--'}")
        detail_parts.append(
            f"reconnects={health.reconnect_count}"
        )
        if health.last_error:
            detail_parts.append(f"error={health.last_error}")
        self.model.set_status(
            "gnss.ntrip",
            "NTRIP",
            health.state.value,
            severity,
            detail="; ".join(detail_parts),
            source="NtripNetworkWorker + NtripStreamStats + RTCM parser",
            age_s=health.rtcm_age_s if health.rtcm_age_s is not None else health.transport_age_s,
            clock="app_receive_monotonic; caster transport and parsed RTCM are separate",
            observed_monotonic_s=now_s,
        )

    def _refresh_pixhawk(self, panel: object, side: str, label: str, now_s: float) -> None:
        mavlink_sides = set(_safe_call(panel, "_mavlink_open_sides", default=()) or ())
        attitude_times = getattr(panel, "pixhawk_attitude_times", {}) or {}
        age_s = _sample_age(attitude_times.get(side, []), now_s)
        transport = "MAVLink" if side in mavlink_sides else "telemetry"
        subscriber = getattr(panel, "mavlink_pixhawk", None)
        active_command_cameras = getattr(subscriber, "_active_command_cameras", ()) or ()
        command_active = side in active_command_cameras
        if command_active:
            state, severity = "PARAM SYNC", StatusSeverity.OK
            detail = "PX4 파라미터 조회 중 (정상)"
        elif age_s is not None and float(age_s) < 2.0:
            state, severity = "LIVE", StatusSeverity.OK
            detail = f"{transport} attitude age {float(age_s):.2f} s"
        elif age_s is not None:
            state, severity = "STALE", StatusSeverity.WARN
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
            source=(
                "MAVLink exclusive command reservation"
                if command_active
                else "MAVLink heartbeat + attitude sample"
            ),
            age_s=None if age_s is None else float(age_s),
            clock="app_receive_monotonic",
            observed_monotonic_s=now_s,
        )

    def _refresh_gnss(self, panel: object, now_s: float) -> None:
        manager = getattr(panel, "gnss_operation_manager", None)
        state_value = getattr(getattr(manager, "state", None), "value", None)
        state_value = str(state_value or "").upper()
        readiness_config = getattr(getattr(manager, "config", None), "readiness", None)
        try:
            configured_maximum_age = getattr(
                readiness_config, "maximum_gps_sample_age_s", 1.0
            )
            maximum_gps_age_s = float(
                1.0 if configured_maximum_age is None else configured_maximum_age
            )
        except (TypeError, ValueError):
            maximum_gps_age_s = 1.0
        if not math.isfinite(maximum_gps_age_s) or maximum_gps_age_s < 0.0:
            maximum_gps_age_s = 1.0
        gps_states = getattr(panel, "latest_gps", {}) or {}
        selected_gps_reader = getattr(panel, "_selected_rover_gps_state", None)
        baseline = getattr(panel, "latest_rtk_baseline", None)
        receiver_details = []
        receiver_ages: dict[str, float] = {}
        receiver_labels: list[str] = []
        for side, label in (("left", "P1"), ("right", "P2")):
            gps = (
                selected_gps_reader(side)
                if callable(selected_gps_reader)
                else gps_states.get(side)
                if isinstance(gps_states, dict)
                else None
            )
            if gps is None:
                receiver_details.append(f"{label}=NO DATA")
                continue
            try:
                timestamp_s = float(getattr(gps, "timestamp_s"))
                if not math.isfinite(timestamp_s):
                    raise ValueError
                age_s = now_s - timestamp_s
                if age_s < 0.0:
                    raise ValueError
            except (AttributeError, TypeError, ValueError):
                age_s = None
            if age_s is not None:
                receiver_ages[side] = age_s
            fix = str(getattr(gps, "rtk_label", "UNKNOWN") or "UNKNOWN")
            receiver_labels.append(fix)
            fix_type = getattr(gps, "fix_type", None)
            sats = getattr(gps, "satellites_used", None)
            eph = getattr(gps, "eph_m", None)
            epv = getattr(gps, "epv_m", None)
            freshness = (
                "NO TIMESTAMP"
                if age_s is None
                else "STALE"
                if age_s > maximum_gps_age_s
                else "FRESH"
            )
            age_text = "--" if age_s is None else f"{age_s:.2f}s"
            receiver_details.append(
                f"{label}={fix}/{freshness}(fix={fix_type if fix_type is not None else '--'}, "
                f"sats={sats if sats is not None else '--'}, "
                f"eph={eph if eph is not None else '--'}m, "
                f"epv={epv if epv is not None else '--'}m, age={age_text})"
            )
        baseline_age = None
        baseline_detail = "baseline NED=NO DATA"
        if baseline is not None:
            left_age = float(getattr(baseline, "left_age_s", 0.0))
            right_age = float(getattr(baseline, "right_age_s", 0.0))
            complete_pair = set(receiver_ages) == {"left", "right"}
            baseline_age = max(receiver_ages.values()) if complete_pair else None
            baseline_freshness = (
                "UNKNOWN"
                if baseline_age is None
                else "STALE"
                if baseline_age > maximum_gps_age_s
                else "FRESH"
            )
            baseline_age_text = "--" if baseline_age is None else f"{baseline_age:.2f}s"
            baseline_detail = (
                "baseline NED="
                f"{float(getattr(baseline, 'north_m', 0.0)):+.3f}/"
                f"{float(getattr(baseline, 'east_m', 0.0)):+.3f}/"
                f"{float(getattr(baseline, 'down_m', 0.0)):+.3f} m, "
                f"length={float(getattr(baseline, 'baseline_m', 0.0)):.3f} m, "
                f"quality={getattr(baseline, 'fix_quality', 'UNKNOWN')}, "
                f"freshness={baseline_freshness}, age={baseline_age_text} "
                f"(created P1/P2={left_age:.2f}/{right_age:.2f}s)"
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
        has_receiver = bool(receiver_labels)
        has_complete_timestamped_pair = set(receiver_ages) == {"left", "right"}
        receiver_stale = any(age > maximum_gps_age_s for age in receiver_ages.values())
        baseline_stale = baseline_age is not None and baseline_age > maximum_gps_age_s
        fix_quality = str(getattr(baseline, "fix_quality", "") or "").upper() if baseline is not None else ""
        current_pair_fixed = len(receiver_labels) == 2 and all(
            label.upper() == "RTK_FIXED" for label in receiver_labels
        )
        if state_value in {"FAILED", "ERROR", "FAULT", "INVALIDATED"}:
            state, severity = "ERROR", StatusSeverity.BAD
        elif receiver_stale or baseline_stale:
            state, severity = "STALE", StatusSeverity.WARN
        elif not has_receiver or not has_complete_timestamped_pair:
            # A manager lifecycle state is not rover-position evidence.  In
            # particular READY with no P1/P2 samples must remain NO DATA.
            state, severity = "NO DATA", StatusSeverity.UNKNOWN
        elif baseline is not None and fix_quality == "GOOD" and current_pair_fixed:
            state, severity = "FIX", StatusSeverity.OK
        elif baseline is not None and "RTK_FLOAT" in receiver_labels:
            state, severity = "FLOAT", StatusSeverity.WARN
        elif baseline is not None and receiver_labels:
            state, severity = "NO FIX", StatusSeverity.WARN
        elif has_receiver:
            state, severity = "NO FIX", StatusSeverity.WARN
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
            age_s=(
                baseline_age
                if baseline_age is not None
                else (max(receiver_ages.values()) if receiver_ages else None)
            ),
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
        quality = str(getattr(panel, "last_distance_quality", "WARNING") or "WARNING").upper()
        detail = str(getattr(panel, "last_distance_status", "measurement evidence 대기") or "")
        if "대기" in detail or "WAIT" in detail.upper():
            state, severity = "WAITING", StatusSeverity.WARN
        elif quality in {"GOOD", "VALID"}:
            state, severity = "VALID", StatusSeverity.OK
        elif quality in {"DEGRADED", "WARN", "WARNING"}:
            state, severity = "DEGRADED", StatusSeverity.WARN
        elif quality in {"ERROR", "FAILED"}:
            state, severity = "ERROR", StatusSeverity.BAD
        else:
            state, severity = "REJECTED", StatusSeverity.BAD
        measurement_age_s = None
        history = getattr(panel, "distance_history", None)
        try:
            if history:
                measurement_age_s = max(0.0, now_s - float(list(history)[-1].timestamp_s))
        except (TypeError, ValueError, IndexError, AttributeError):
            measurement_age_s = None
        if state == "VALID" and measurement_age_s is not None and measurement_age_s >= 2.0:
            state, severity = "STALE", StatusSeverity.WARN
        self.model.set_status(
            "measurement",
            "MEAS",
            state,
            severity,
            detail=detail,
            source="MeasurementAttempt/controller result",
            age_s=measurement_age_s,
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
        stats = stats if isinstance(stats, Mapping) else {}
        values = []
        for side in ("left", "right"):
            side_stats = stats.get(side) or {}
            if not isinstance(side_stats, Mapping):
                continue
            value = side_stats.get("decoded_hz")
            if value in (None, ""):
                continue
            try:
                parsed = float(value)
            except (TypeError, ValueError, OverflowError):
                continue
            if math.isfinite(parsed) and parsed >= 0.0:
                values.append(parsed)
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
