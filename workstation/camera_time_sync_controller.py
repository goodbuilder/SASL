"""Camera clock synchronization controller for the workstation app.

The controller coordinates WIRIS STUS/GDTI evidence and RTCP/NTP observations.
Timestamp values are Unix UTC seconds or monotonic seconds as explicitly named;
it performs no camera RDF, body FRD, or world NED coordinate conversion.
"""

from __future__ import annotations

import re
import time
from collections.abc import Sequence
from datetime import datetime, timezone
from math import isfinite

from PyQt5 import QtCore

from workstation.wiris_runtime_workers import WirisClockCommandWorker
from workstation_core.measurement_widgets import ClockOffsetPoint
from workstation_core.runtime_camera_status import infer_runtime_camera_timestamp_reference
from workstation_core.yolo_stream import RTSP_TRANSPORT_TCP

CAMERA_GDTI_RESOLUTION_S = 1.0
CAMERA_GDTI_MAX_PAIR_DELTA_S = 1.0
CAMERA_RTCP_PAIR_MAX_DELTA_S = 0.050
CAMERA_RTCP_MAX_PC_OFFSET_S = 0.100
CAMERA_SYNC_FRAME_HISTORY_WINDOW_S = 1.0
CAMERA_SYNC_HISTORY_MAX_AGE_S = CAMERA_SYNC_FRAME_HISTORY_WINDOW_S
CAMERA_SYNC_MAX_SR_AGE_S = 7.0
CAMERA_SYNC_RECEIVE_PAIR_MAX_DELTA_S = 0.350
CAMERA_SYNC_WARMUP_TIMEOUT_S = 30.0
CAMERA_SYNC_REQUIRED_SR_COUNT = 2
CAMERA_SYNC_MAX_CLOCK_RATE_ERROR = 0.005
CAMERA_SYNC_STATUS_NOT_VERIFIED = "not-verified"
CAMERA_SYNC_STATUS_RECONNECTING = "reconnecting"
CAMERA_SYNC_STATUS_WARMING = "warming"
CAMERA_SYNC_STATUS_VERIFIED = "verified"
CAMERA_SYNC_STATUS_FAILED = "failed"

CAMERA_TIME_SYNC_METHODS = [
    "_set_camera_sync_state",
    "_parse_wiris_gdti_unix_s",
    "_format_wiris_gdti_results",
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
    "_poll_camera_sync_state",
    "_begin_camera_sync_after_stus",
    "_start_wiris_gdti_worker",
    "_apply_wiris_gdti_result",
    "_wiris_gdti_failed",
    "_clear_wiris_gdti_worker",
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

    def _parse_wiris_gdti_unix_s(self, response: str) -> float | None:
        """Parse the documented WIRIS date/time text without trusting raw text."""

        text = str(response or "")
        match = re.search(
            r"(?P<year>\d{4})[/-](?P<month>\d{2})[/-](?P<day>\d{2})"
            r"(?:[ T-])(?P<hour>\d{2}):(?P<minute>\d{2}):(?P<second>\d{2})"
            r"(?:\.(?P<fraction>\d{1,6}))?",
            text,
        )
        if match is None:
            return None
        fraction = (match.group("fraction") or "").ljust(6, "0")
        try:
            value = datetime(
                int(match.group("year")),
                int(match.group("month")),
                int(match.group("day")),
                int(match.group("hour")),
                int(match.group("minute")),
                int(match.group("second")),
                int(fraction or 0),
                tzinfo=timezone.utc,
            )
        except ValueError:
            return None
        return value.timestamp()

    def _format_wiris_gdti_results(self, result: object) -> dict[str, dict[str, object]]:
        data = dict(result or {})
        raw_results = dict(data.get("results") or {})
        results: dict[str, dict[str, object]] = {}
        for camera, raw in raw_results.items():
            row = dict(raw or {})
            requested_utc_s = self.panel._camera_sync_requested_utc_by_camera.get(camera)
            if requested_utc_s is None:
                requested_utc_s = self.panel._camera_sync_requested_utc_s
            response = str(row.get("response") or "")
            parsed_s = self.panel._parse_wiris_gdti_unix_s(response)
            started_s = row.get("query_started_utc_s")
            finished_s = row.get("query_finished_utc_s")
            midpoint_s = None
            query_duration_s = None
            if started_s is not None and finished_s is not None:
                query_duration_s = max(0.0, float(finished_s) - float(started_s))
                midpoint_s = (float(started_s) + float(finished_s)) / 2.0
            reference_s = midpoint_s
            if reference_s is None:
                reference_s = requested_utc_s
            query_offset_ms = (
                None
                if parsed_s is None or reference_s is None
                else (parsed_s - reference_s) * 1000.0
            )
            tolerance_ms = CAMERA_GDTI_RESOLUTION_S * 500.0
            if query_duration_s is not None:
                tolerance_ms += query_duration_s * 500.0
            clock_valid = (
                parsed_s is not None
                and query_offset_ms is not None
                and abs(float(query_offset_ms)) <= tolerance_ms
            )
            results[camera] = {
                "response": response,
                "unix_s": parsed_s,
                "offset_ms": (
                    None
                    if parsed_s is None or requested_utc_s is None
                    else (parsed_s - requested_utc_s) * 1000.0
                ),
                "query_started_utc_s": started_s,
                "query_finished_utc_s": finished_s,
                "query_midpoint_utc_s": midpoint_s,
                "query_duration_ms": None if query_duration_s is None else query_duration_s * 1000.0,
                "query_offset_ms": query_offset_ms,
                "resolution_s": CAMERA_GDTI_RESOLUTION_S,
                "offset_tolerance_ms": tolerance_ms,
                "clock_valid": clock_valid,
            }
        return results

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
        missing: list[str] = []
        for camera, metadata in metadata_by_camera.items():
            count = int(metadata.get("rtcp_sr_received_count") or 0)
            required_sr_count = self.panel._camera_sync_required_sr_count()
            if count < required_sr_count:
                missing.append(f"{self.panel._camera_name_ko(camera)} SR {count}/{required_sr_count}")
                continue
            try:
                age_s = float(metadata.get("rtcp_sr_age_s"))
            except (TypeError, ValueError):
                missing.append(f"{self.panel._camera_name_ko(camera)} SR age 없음")
                continue
            if not math.isfinite(age_s) or age_s > self.panel._camera_sync_max_sr_age_s():
                missing.append(f"{self.panel._camera_name_ko(camera)} SR stale {age_s:.2f}s")
            for key, label in (
                ("rtcp_camera_ntp_rate", "NTP clock"),
                ("rtcp_camera_media_rate", "RTP clock"),
            ):
                try:
                    rate = float(metadata.get(key))
                except (TypeError, ValueError):
                    missing.append(f"{self.panel._camera_name_ko(camera)} {label} 없음")
                    continue
                if not math.isfinite(rate) or abs(rate - 1.0) > self.panel._camera_sync_max_clock_rate_error():
                    missing.append(f"{self.panel._camera_name_ko(camera)} {label}={rate:.6f}")

            gdti = self.panel._camera_sync_gdti.get(camera) or {}
            gdti_s = gdti.get("unix_s")
            if gdti_s is None:
                missing.append(f"{self.panel._camera_name_ko(camera)} GDTI 파싱 실패")
            elif gdti.get("clock_valid") is False:
                offset_ms = gdti.get("query_offset_ms")
                tolerance_ms = gdti.get("offset_tolerance_ms")
                try:
                    missing.append(
                        f"{self.panel._camera_name_ko(camera)} GDTI query offset="
                        f"{float(offset_ms):+.1f}ms (허용 {float(tolerance_ms):.1f}ms)"
                    )
                except (TypeError, ValueError):
                    missing.append(f"{self.panel._camera_name_ko(camera)} GDTI 시간 검증 실패")

        gdti_values = [
            (self.panel._camera_sync_gdti.get(camera) or {}).get("unix_s")
            for camera in ("left", "right")
        ]
        if all(value is not None for value in gdti_values):
            try:
                gdti_pair_delta_s = abs(float(gdti_values[1]) - float(gdti_values[0]))
            except (TypeError, ValueError):
                gdti_pair_delta_s = float("inf")
            if gdti_pair_delta_s > CAMERA_GDTI_MAX_PAIR_DELTA_S:
                missing.append(f"좌우 GDTI Δt={gdti_pair_delta_s:.3f}s")

        metrics = self.panel._camera_time_sync_metrics(
            left_metadata,
            right_metadata,
            now_wall_s=time.time(),
            now_mono_s=now_mono_s,
        )
        pair_delta_s = metrics.get("pair_delta_s")
        if pair_delta_s is None:
            missing.append("좌우 RTCP/NTP pair 없음")
        elif abs(float(pair_delta_s)) > self.panel._camera_sync_rtcp_pair_max_delta_s():
            missing.append(f"좌우 RTCP/NTP Δt={abs(float(pair_delta_s)) * 1000.0:.1f}ms")
        receive_pair_delta_s = metrics.get("receive_pair_delta_s")
        if receive_pair_delta_s is None:
            try:
                receive_pair_delta_s = abs(
                    float(right_metadata["timestamp_s"]) - float(left_metadata["timestamp_s"])
                )
            except (KeyError, TypeError, ValueError):
                receive_pair_delta_s = None
        if receive_pair_delta_s is None:
            missing.append("좌우 실제 수신시각 pair 없음")
        elif float(receive_pair_delta_s) > self.panel._camera_sync_receive_pair_max_delta_s():
            missing.append(f"좌우 실제 수신 Δt={float(receive_pair_delta_s) * 1000.0:.1f}ms")
        max_sr_pc_offset_ms = metrics.get("max_sr_pc_offset_ms")
        if max_sr_pc_offset_ms is None:
            missing.append("RTCP SR/PC UTC offset 없음")
        elif float(max_sr_pc_offset_ms) > self.panel._camera_sync_max_pc_offset_s() * 1000.0:
            missing.append(f"RTCP SR/PC UTC offset={float(max_sr_pc_offset_ms):.1f}ms")
        if missing:
            return False, "; ".join(missing)
        return True, "GDTI·RTCP/NTP·PC UTC·좌우 Δt 검증 완료"

    def _poll_camera_sync_state(self) -> None:
        if self.panel.camera_sync_status not in {
            CAMERA_SYNC_STATUS_RECONNECTING,
            CAMERA_SYNC_STATUS_WARMING,
        }:
            self.panel._camera_sync_timer.stop()
            return
        deadline = self.panel._camera_sync_deadline_monotonic_s
        if deadline is not None and time.monotonic() >= deadline:
            self.panel._camera_sync_timer.stop()
            self.panel._set_camera_sync_state(
                CAMERA_SYNC_STATUS_FAILED,
                "동기화 검증 timeout: " + self.panel.camera_sync_reason,
            )
            return
        ready, reason = self.panel._camera_sync_readiness()
        if ready:
            self.panel._camera_sync_timer.stop()
            self.panel._camera_sync_deadline_monotonic_s = None
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_VERIFIED, reason)
        else:
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_WARMING, reason)

    def _begin_camera_sync_after_stus(
        self,
        *,
        target: str,
        result: str,
        requested_utc_s: float | None,
        target_infos: Sequence[tuple[str, str, int, float]],
    ) -> None:
        self.panel._camera_sync_requested_utc_s = requested_utc_s
        if target != "both" or result != "success":
            self.panel._set_camera_sync_state(
                CAMERA_SYNC_STATUS_FAILED,
                "좌우 카메라 모두에 STUS가 성공해야 동기화를 검증할 수 있습니다.",
            )
            return
        self.panel._set_camera_sync_state(
            CAMERA_SYNC_STATUS_RECONNECTING,
            "STUS 완료: GDTI 확인 후 RTSP 스트림을 TCP로 재연결합니다.",
        )
        if not self.panel.camera_connected:
            self.panel._set_camera_sync_state(
                CAMERA_SYNC_STATUS_NOT_VERIFIED,
                "STUS는 완료됐지만 RTSP 스트림이 없어 검증을 시작하지 않았습니다.",
            )
            return
        self.panel._start_wiris_gdti_worker(target_infos)

    def _start_wiris_gdti_worker(self, target_infos: Sequence[tuple[str, str, int, float]]) -> None:
        if self.panel.wiris_gdti_thread is not None:
            self.panel._log("[시계] GDTI 조회가 이미 진행 중입니다.")
            return
        self.panel.wiris_command_status_label.setText("GDTI 확인: querying...")
        # The controller is a plain Python coordinator, not a QObject.  The
        # panel owns the Qt thread so its lifetime follows the Workstation
        # widget and Qt receives a valid QObject parent.
        self.panel.wiris_gdti_thread = QtCore.QThread(self.panel)
        self.panel.wiris_gdti_worker = WirisClockCommandWorker(target_infos, command_text="GDTI")
        self.panel.wiris_gdti_worker.moveToThread(self.panel.wiris_gdti_thread)
        self.panel.wiris_gdti_thread.started.connect(self.panel.wiris_gdti_worker.run)
        self.panel.wiris_gdti_worker.completed.connect(self.panel._apply_wiris_gdti_result)
        self.panel.wiris_gdti_worker.failed.connect(self.panel._wiris_gdti_failed)
        self.panel.wiris_gdti_worker.finished.connect(self.panel.wiris_gdti_thread.quit)
        self.panel.wiris_gdti_worker.finished.connect(self.panel.wiris_gdti_worker.deleteLater)
        self.panel.wiris_gdti_thread.finished.connect(self.panel.wiris_gdti_thread.deleteLater)
        self.panel.wiris_gdti_thread.finished.connect(self.panel._clear_wiris_gdti_worker)
        self.panel.wiris_gdti_thread.start()

    def _apply_wiris_gdti_result(self, result: object) -> None:
        self.panel._camera_sync_gdti = self.panel._format_wiris_gdti_results(result)
        if self.panel._session_recording_active():
            reason = "세션 기록이 시작되어 RTSP 재연결을 건너뛰었습니다. 기록 종료 후 다시 동기화하세요."
            self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_NOT_VERIFIED, reason)
            self.panel._log(f"[시계] {reason}")
            return
        self.panel._camera_sync_transport_override = RTSP_TRANSPORT_TCP
        self.panel._camera_sync_deadline_monotonic_s = time.monotonic() + self.panel._camera_sync_warmup_timeout_s()
        self.panel._start_camera_streams(enable_detection=self.panel.yolo_enabled, switch_to_depth=self.panel.yolo_enabled)
        self.panel._set_camera_sync_state(
            CAMERA_SYNC_STATUS_WARMING,
            "RTSP 재연결 완료 대기: 양쪽 RTCP Sender Report를 수집합니다.",
        )
        self.panel._camera_sync_timer.start()

    def _wiris_gdti_failed(self, error: str) -> None:
        self.panel._camera_sync_gdti = {}
        self.panel._set_camera_sync_state(CAMERA_SYNC_STATUS_FAILED, f"GDTI 조회 실패: {error}")
        self.panel._log(f"[시계] GDTI 조회 실패: {error}")

    def _clear_wiris_gdti_worker(self) -> None:
        self.panel.wiris_gdti_thread = None
        self.panel.wiris_gdti_worker = None

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

        The latest left and latest right frames are not a pair when the
        cameras decode at different rates. Prefer frames received close
        together, then minimize their RTCP-derived timestamp delta.
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

        candidates: list[tuple[float, float, dict[str, object], dict[str, object]]] = []
        for left in left_rows:
            left_rtcp_s = self.panel._camera_rtc_timestamp_s_from_metadata(left)
            left_receive_s = left.get("timestamp_s")
            for right in right_rows:
                right_rtcp_s = self.panel._camera_rtc_timestamp_s_from_metadata(right)
                if left_rtcp_s is None or right_rtcp_s is None:
                    continue
                receive_delta_s = float("inf")
                if left_receive_s is not None and right.get("timestamp_s") is not None:
                    try:
                        receive_delta_s = abs(float(right["timestamp_s"]) - float(left_receive_s))
                    except (TypeError, ValueError):
                        pass
                candidates.append(
                    (
                        abs(float(right_rtcp_s) - float(left_rtcp_s)),
                        receive_delta_s,
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
        _rtcp_delta, _receive_delta, left, right = min(
            search_space,
            key=lambda candidate: (candidate[0], candidate[1]),
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
            stus_detail = (
                f"STUS camera-specific target applied ({compensation_method}): PC send UTC + HIWS RTT/2 + parse margin.\n"
                "Run RTCP/NTP 동기화 확인 after stream frames arrive to verify "
                "left/right RTC and PC UTC offsets."
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
