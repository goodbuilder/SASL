"""Workstation GNSS base, NTRIP, survey-in, and RTCM controller.

WGS84 coordinates, UBX fields, RTCM payloads, and explicitly named NED
telemetry retain their producer contracts. No camera/FRD alias conversion is
performed in this UI orchestration module.
"""

from __future__ import annotations

import time
from math import isfinite
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from Workstation.workers.device_runtime_workers import PixhawkRtcmInjectionWorker
from Workstation.controllers.drone_telemetry_controller import DRONE_RTCM_SOURCE_BASE, DRONE_RTCM_SOURCE_NTRIP, normalize_drone_rtcm_source
from Workstation.gnss_runtime.base import BASE_FIELDS, BaseReceiverEffects, BaseReceiverRuntime, BaseReceiverState
from Workstation.gnss_runtime.corrections import (
    CORRECTION_FIELDS,
    CorrectionEvents,
    CorrectionRuntime,
    CorrectionRuntimeState,
)
from Workstation.gnss_runtime.credentials import (
    CredentialInputs,
    NtripCredentialError,
    NtripCredentialResolution,
    _ntrip_credential_account_key,
    maybe_persist_ntrip_credential,
    resolve_ntrip_credential,
)
from Workstation.gnss_runtime.evidence import EVIDENCE_FIELDS, EvidenceCapture, GnssEvidenceState, RtcmEvidenceRuntime
from Workstation.gnss_runtime.gga import (
    GGA_STATE_FIELDS,
    GgaSentenceOwner,
)
from Workstation.gnss_runtime.gga import (
    SYSTEM_CENTER_GGA_MAXIMUM_GPS_SAMPLE_AGE_S as SYSTEM_CENTER_GGA_MAXIMUM_GPS_SAMPLE_AGE_S,
)
from Workstation.gnss_runtime.gga import (
    SYSTEM_CENTER_GGA_MAXIMUM_LEFT_RIGHT_TIMESTAMP_DELTA_MS as SYSTEM_CENTER_GGA_MAXIMUM_LEFT_RIGHT_TIMESTAMP_DELTA_MS,
)
from Workstation.gnss_runtime.ntrip import NTRIP_FIELDS, NtripSessionRuntime, NtripSessionState
from Workstation.gnss_runtime.ntrip import NtripStatsForwarder as NtripStatsForwarder
from Workstation.gnss_runtime.operation import (
    OPERATION_FIELDS,
    GnssOperationRuntime,
    GnssOperationRuntimeState,
    NetworkAdmissionInputs,
    ReadinessObservation,
)
from Workstation.workers.gnss_runtime_workers import GnssBaseSerialDetectWorker, GnssBaseSerialWorker, NtripNetworkWorker, RtcmDataPlaneWorker
from Workstation.ui.telemetry_plots import TelemetryHistoryPoint
from Workstation.workstation_core import ntrip_credentials
from Workstation.workstation_core.gnss_network import format_nmea_gga as format_nmea_gga
from Workstation.workstation_core.gnss_network import gga_from_coordinate_sample as gga_from_coordinate_sample
from Workstation.workstation_core.gnss_network import gga_from_position as gga_from_position
from Workstation.workstation_core.gnss_operation import (
    STATE_STATUS_GROUP,
    CorrectionSourceIdentity,
    GgaPolicy,
    GnssOperationConfig,
    GnssOperationMode,
    GnssOperationState,
    GnssStatusGroup,
    HeightType,
    evaluate_gnss_readiness,
)
from Workstation.workstation_core.gnss_profile import GnssCoordinateSample
from Workstation.workstation_core.gnss_protocol import (
    GNSS_BASE_LOW_CNO_DBHZ,
    GNSS_BASE_LOW_ELEVATION_DEG,
    GNSS_BASE_SURVEY_IMPROVING_DELTA_M,
    GNSS_BASE_SURVEY_STAGNANT_DELTA_M,
    GNSS_BASE_SURVEY_TREND_WINDOW_S,
    Rtcm3StreamParser,
    parse_rtcm3_message_type,
)
from Workstation.workstation_core.rtcm_pipeline import (
    RTCM_PIPELINE_SNAPSHOT_SCHEMA,
    NtripSessionStats,
    RtcmEvidenceContext,
    RtcmRouteConfig,
)


def _connect_gui_signal(signal, callback) -> None:
    """Deliver a worker signal to a panel/UI callback on the Qt GUI thread.

    Panel controller methods are installed dynamically as plain Python
    delegates, so Qt's AutoConnection does not reliably infer the panel's
    thread affinity.  Every worker-to-UI boundary in this controller uses an
    explicit queued connection; the intentional DirectConnection adapters
    below remain limited to locked queues that never touch widgets.
    """

    signal.connect(callback, QtCore.Qt.QueuedConnection)


GNSS_RUNTIME_CONTROLLER_METHODS = [
    "_selected_rover_gps_state",
    "_refresh_gnss_base_tab",
    "_gnss_selected_operation_mode",
    "_gnss_active_operation_config",
    "_set_gnss_operation_controls_locked",
    "_build_gnss_operation_config",
    "_gnss_configure_manager",
    "_gnss_status_label_key_for_state",
    "_gnss_transition",
    "_handle_gnss_operation_mode_changed",
    "_gnss_source_identity",
    "_gnss_gga_provider",
    "_resolve_ntrip_credentials",
    "_refresh_ntrip_credential_status",
    "forget_gnss_ntrip_password",
    "set_gnss_ntrip_session_password",
    "start_gnss_network",
    "stop_gnss_network",
    "_handle_gnss_network_connected",
    "_handle_gnss_network_stats",
    "_handle_gnss_network_chunk",
    "_handle_gnss_network_failed",
    "_handle_gnss_network_reconnecting",
    "_handle_gnss_network_diagnostic",
    "_clear_gnss_network_worker",
    "_record_gnss_base_position_row",
    "_handle_gnss_base_position",
    "_fail_gnss_fixed_base_reconfiguration",
    "_handle_gnss_base_ubx_message",
    "_maybe_activate_gnss_fixed_base",
    "_refresh_gnss_operation_readiness",
    "connect_gnss_base_serial",
    "disconnect_gnss_base_serial",
    "start_gnss_base_survey_in",
    "stop_gnss_base_survey_in",
    "discover_gnss_base_serial_ports",
    "_handle_gnss_base_candidates_detected",
    "_handle_gnss_base_candidate_detect_failed",
    "_clear_gnss_base_detect_worker",
    "_set_gnss_base_candidate_rows",
    "_refresh_gnss_base_candidate_actions",
    "apply_selected_gnss_base_candidate",
    "apply_gnss_base_candidate",
    "_handle_gnss_base_serial_connected",
    "_handle_gnss_base_serial_stats",
    "_handle_gnss_base_serial_failed",
    "_handle_gnss_base_serial_disconnected",
    "_handle_gnss_base_satellite_count",
    "_set_gnss_base_quality_waiting",
    "_reset_gnss_base_rtcm_stats",
    "_format_rtcm_message_types",
    "_refresh_gnss_base_rtcm_stats",
    "_handle_gnss_base_satellite_quality",
    "_ensure_rtcm_pipeline",
    "_sync_rtcm_pipeline_config",
    "_sync_rtcm_evidence_context",
    "_close_rtcm_evidence_context_and_drain",
    "_apply_rtcm_pipeline_snapshot",
    "_handle_rtcm_pipeline_evidence_started",
    "_handle_rtcm_pipeline_integrity_failed",
    "_stop_rtcm_pipeline",
    "_gnss_base_rtcm_target_sides",
    "_record_gnss_base_rtcm_frame",
    "_record_gnss_base_rtcm_injection",
    "_handle_gnss_base_rtcm_frame",
    "_ensure_pixhawk_rtcm_worker",
    "_apply_pixhawk_rtcm_result",
    "_clear_pixhawk_rtcm_worker",
    "_handle_gnss_base_command_sent",
    "_handle_gnss_base_survey_in",
    "_update_gnss_base_survey_visualization",
    "_format_eta_seconds",
    "_update_gnss_base_survey_convergence",
]


class GnssRuntimeController:
    RTCM_UI_UPDATE_INTERVAL_S = 0.5

    def __init__(self, panel):
        self.panel = panel
        self._last_rtcm_ui_update_s = float("-inf")
        self._last_rtcm_result_ui_update_s = float("-inf")
        self._last_ntrip_ui_update_s = float("-inf")
        self._last_pipeline_drop_count = 0
        self._last_pipeline_blocked_stall_count = 0
        package_signal = getattr(panel, "measurement_package_changed", None)
        if hasattr(package_signal, "connect") and not bool(getattr(panel, "_rtcm_package_signal_connected", False)):
            package_signal.connect(self._handle_measurement_package_changed)
            panel._rtcm_package_signal_connected = True

    def _publish_measurement_source(self, source_id: str, state: str, detail: str) -> None:
        publisher = getattr(self.panel, "measurement_source_transition", None)
        if callable(publisher):
            try:
                publisher(source_id, state, detail)
            except (RuntimeError, TypeError, ValueError):
                pass

    def _handle_measurement_package_changed(self, _snapshot: object) -> None:
        self._sync_rtcm_evidence_context()

    def _rtcm_route_config(self) -> RtcmRouteConfig:
        config = getattr(self.panel, "gnss_active_operation_config", None)
        if config is None:
            builder = getattr(self.panel, "_build_gnss_operation_config", None)
            if callable(builder):
                try:
                    config = builder()
                except (AttributeError, TypeError, ValueError):
                    config = None
        mode = getattr(config, "operation_mode", None)
        assisted = mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE
        ntrip_targets = self._gnss_base_rtcm_target_sides(
            source=DRONE_RTCM_SOURCE_NTRIP,
            include_rover_targets=not assisted,
        )
        base_targets = self._gnss_base_rtcm_target_sides(
            source=DRONE_RTCM_SOURCE_BASE,
            include_rover_targets=True,
        )
        return GnssOperationRuntime.route(
            config,
            ntrip_targets=ntrip_targets,
            base_targets=base_targets,
            base_serial_available=bool(
                getattr(self.panel, "gnss_base_serial_connected", False)
                and getattr(self.panel, "gnss_base_serial_worker", None) is not None
            ),
        )

    def _rtcm_evidence_context(self) -> RtcmEvidenceContext:
        return self._evidence_runtime().context()

    def _ensure_rtcm_pipeline(self) -> RtcmDataPlaneWorker:
        return self._correction_runtime()._ensure_rtcm_pipeline()

    def _wire_rtcm_pipeline_base_worker(self, base_worker: object | None) -> None:
        return self._correction_runtime()._wire_rtcm_pipeline_base_worker(base_worker)

    def _wire_rtcm_pipeline_injection_worker(self, injection_worker: object | None) -> None:
        return self._correction_runtime()._wire_rtcm_pipeline_injection_worker(injection_worker)

    def _wire_rtcm_ntrip_source(
        self,
        source: NtripNetworkWorker,
        pipeline: RtcmDataPlaneWorker,
        *,
        session_token: int | None = None,
    ) -> None:
        return CorrectionRuntime._wire_rtcm_ntrip_source(source, pipeline, session_token=session_token)

    def _wire_rtcm_base_source(
        self,
        source: GnssBaseSerialWorker,
        pipeline: RtcmDataPlaneWorker,
    ) -> None:
        return CorrectionRuntime._wire_rtcm_base_source(source, pipeline)

    def _sync_rtcm_pipeline_config(self, *, reset: bool = False) -> None:
        return self._correction_runtime()._sync_rtcm_pipeline_config(reset=reset)

    def _sync_rtcm_evidence_context(self) -> None:
        return self._correction_runtime()._sync_rtcm_evidence_context()

    def _close_rtcm_evidence_context_and_drain(self, context_token: str) -> bool:
        return self._evidence_runtime().close_and_drain(
            getattr(self.panel, "rtcm_pipeline_worker", None), context_token
        )

    def _apply_rtcm_pipeline_snapshot(self, snapshot: object) -> None:
        if not isinstance(snapshot, dict) or snapshot.get("schema") != RTCM_PIPELINE_SNAPSHOT_SCHEMA:
            return
        self._correction_runtime().apply_snapshot_stats(snapshot)
        ntrip_stats = snapshot.get("ntrip_stats")
        runtime = getattr(self, "_ntrip_owner", None)
        if isinstance(ntrip_stats, NtripSessionStats):
            ntrip_stats = (
                ntrip_stats.stats if runtime is not None and runtime.accepts_stats(ntrip_stats.session_token) else None
            )
        elif runtime is not None:
            # Untagged legacy stats cannot authenticate a current session.
            ntrip_stats = None
        if ntrip_stats is not None:
            self.panel.gnss_network_stats = ntrip_stats
            if self.panel.gnss_network_connected and getattr(ntrip_stats, "last_chunk_monotonic_s", None) is not None:
                self.panel.gnss_network_phase = "STREAMING"
        if bool(snapshot.get("base_reference_seen")) and (
            self.panel.gnss_operation_manager.state == GnssOperationState.BASE_RECONFIGURING
        ):
            self.panel.gnss_fixed_base_reconfigure_rtcm = True
            self.panel._maybe_activate_gnss_fixed_base()

        integrity = self.panel.gnss_base_rtcm_stats["integrity"]
        dropped = int(integrity.get("injection_dropped_stale") or 0)
        if dropped > self._last_pipeline_drop_count:
            delta = dropped - self._last_pipeline_drop_count
            self._last_pipeline_drop_count = dropped
            self.panel._log(
                f"[GNSS] RTCM 오래된 주입 대기 프레임 {delta}개 폐기 (누계={dropped}, evidence 원본은 유지)"
            )
        blocked_stall = int(integrity.get("injection_blocked_stall") or 0)
        if blocked_stall > self._last_pipeline_blocked_stall_count:
            delta = blocked_stall - self._last_pipeline_blocked_stall_count
            self._last_pipeline_blocked_stall_count = blocked_stall
            self.panel._log(
                f"[GNSS] stalled MAVLink target RTCM 차단 {delta}회 (누계={blocked_stall}, transport failure와 분리)"
            )

        latest_result = snapshot.get("latest_injection_result")
        if isinstance(latest_result, dict):
            sent = tuple(latest_result.get("sent") or ())
            failed = tuple(latest_result.get("failed") or ())
            blocked = tuple(latest_result.get("blocked_targets") or ())
            if failed:
                text = "RTCM 주입 실패: " + ", ".join(
                    f"{self.panel._px4_mag_camera_label(str(side))}:{error}" for side, error in failed
                )
                level = "warn" if sent else "bad"
            elif blocked:
                blocked_labels = ", ".join(
                    self.panel._px4_mag_camera_label(str(item.get("side") or ""))
                    for item in blocked
                    if isinstance(item, dict)
                )
                sent_labels = ", ".join(self.panel._px4_mag_camera_label(str(side)) for side, _packets in sent)
                text = (
                    f"RTCM 일부 주입: {sent_labels} | stalled 차단 {blocked_labels}"
                    if sent
                    else f"RTCM stalled target 차단: {blocked_labels}"
                )
                level = "warn" if sent else "bad"
            elif sent:
                text = f"RTCM {int(latest_result.get('payload_bytes') or 0)} bytes -> " + ", ".join(
                    self.panel._px4_mag_camera_label(str(side)) for side, _packets in sent
                )
                level = "good"
            else:
                text = ""
                level = "idle"
            if text and hasattr(self.panel, "gnss_base_status_labels"):
                self.panel._set_health_label(
                    self.panel.gnss_base_status_labels["rtcm_injection"],
                    text,
                    level,
                )

        transport = self.panel.gnss_base_rtcm_stats["transport"]
        if (
            self.panel.gnss_network_connected
            and hasattr(self.panel, "gnss_base_status_labels")
            and int(transport.get("chunks") or 0) > 0
        ):
            unavailable = int(transport.get("base_forward_unavailable") or 0)
            if unavailable:
                text, level = (
                    f"NTRIP 수신 중이나 Base serial 전달 불가 {unavailable}회",
                    "bad",
                )
            else:
                text, level = (
                    f"NTRIP RTCM streaming | chunks={int(transport.get('chunks') or 0)}",
                    "good",
                )
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["network"],
                text,
                level,
            )
        self.panel._refresh_gnss_base_rtcm_stats()
        self.panel._refresh_gnss_operation_readiness()

    def _handle_rtcm_pipeline_evidence_started(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        current_context = self._rtcm_evidence_context()
        if (
            str(payload.get("session_id") or "") != current_context.session_id
            or str(payload.get("context_token") or "") != current_context.context_token
            or str(payload.get("session_root") or "") != str(current_context.session_root)
        ):
            return
        activator = getattr(self.panel, "activate_measurement_package_source", None)
        if callable(activator):
            activator(
                "gnss.rtcm",
                required_roles=("corrections/rtcm/raw",),
                metadata={
                    "activated_utc_ns": int(payload.get("first_receive_utc_ns") or time.time_ns()),
                    "first_pipeline_sequence": int(payload.get("first_sequence") or 0),
                    "source": "rtcm_data_plane_worker",
                },
            )

    def _handle_rtcm_pipeline_integrity_failed(self, payload: object) -> None:
        values = dict(payload) if isinstance(payload, dict) else {"error": str(payload)}
        if values.get("format") == "CSV":
            self.panel._log(f"[저장] RTCM CSV 저장 실패: {values.get('error', '')}")
            append = getattr(self.panel, "append_workstation_data", None)
            if callable(append):
                append(
                    "events",
                    {
                        "event_type": "rtcm_csv_failed",
                        "measurement_id": values.get("context_token"),
                        "detail": values.get("error", ""),
                        "payload": values,
                    },
                    priority=True,
                )
            return
        error = (
            "RTCM integrity failure: "
            f"sequence={int(values.get('sequence') or 0)}, {values.get('error') or 'unknown error'}"
        )
        package_state = str(getattr(self.panel, "_measurement_package_state", "IDLE") or "IDLE").upper()
        current_context = self._rtcm_evidence_context()
        failure_session_id = str(values.get("session_id") or "")
        failure_context_token = str(values.get("context_token") or "")
        belongs_to_current_context = failure_session_id == current_context.session_id and (
            not failure_context_token or failure_context_token == current_context.context_token
        )
        if bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", False)) and (
            belongs_to_current_context and package_state in {"COLLECTING", "POST_CAPTURE", "VERIFYING", "RECOVERABLE"}
        ):
            errors = getattr(self.panel, "_measurement_package_inventory_errors", None)
            if isinstance(errors, list) and error not in errors:
                errors.append(error)
            self.panel._measurement_package_error = error
            self.panel._session_save_retry_required = True
        if hasattr(self.panel, "gnss_base_status_labels"):
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["rtcm_injection"],
                error,
                "bad",
            )
        self.panel._log(f"[GNSS] {error}")

    def _stop_rtcm_pipeline(self) -> bool:
        return self._correction_runtime()._stop_rtcm_pipeline()

    def _clear_rtcm_pipeline(
        self,
        thread: QtCore.QThread,
        worker: RtcmDataPlaneWorker,
    ) -> None:
        return self._correction_runtime()._clear_rtcm_pipeline(thread, worker)

    def _rtcm_ui_update_due(self, now_s: float | None = None) -> bool:
        now_s = time.monotonic() if now_s is None else float(now_s)
        if now_s - self._last_rtcm_ui_update_s < self.RTCM_UI_UPDATE_INTERVAL_S:
            return False
        self._last_rtcm_ui_update_s = now_s
        return True

    def _rtcm_result_ui_update_due(self, now_s: float | None = None) -> bool:
        now_s = time.monotonic() if now_s is None else float(now_s)
        if now_s - self._last_rtcm_result_ui_update_s < self.RTCM_UI_UPDATE_INTERVAL_S:
            return False
        self._last_rtcm_result_ui_update_s = now_s
        return True

    def _ntrip_ui_update_due(self, now_s: float | None = None) -> bool:
        now_s = time.monotonic() if now_s is None else float(now_s)
        if now_s - self._last_ntrip_ui_update_s < self.RTCM_UI_UPDATE_INTERVAL_S:
            return False
        self._last_ntrip_ui_update_s = now_s
        return True

    def _selected_rover_gps_state(self, side: str):
        """Return the position from the operator-selected GPS1/GPS2 source."""

        side = str(side)
        latest = getattr(self.panel, "latest_gps", {}) or {}
        if side not in {"left", "right"}:
            return latest.get(side) if isinstance(latest, dict) else None
        selector = getattr(self.panel, "_selected_baseline_gps_state", None)
        if callable(selector):
            return selector(side)
        pair_selector = getattr(self.panel, "_selected_baseline_gps_states", None)
        if callable(pair_selector):
            selected = pair_selector()
            if isinstance(selected, (tuple, list)) and len(selected) == 2:
                return selected[0 if side == "left" else 1]
        return latest.get(side) if isinstance(latest, dict) else None

    def _refresh_gnss_base_tab(self) -> None:
        if not hasattr(self.panel, "gnss_base_status_labels"):
            return
        labels = self.panel.gnss_base_status_labels
        now_s = time.monotonic()
        for side, key in (("left", "left_rover"), ("right", "right_rover")):
            gps = self._selected_rover_gps_state(side)
            if gps is None:
                self.panel._set_health_label(labels[key], "GPS/RTK 대기 중", "idle")
                continue
            age_ms = max(0.0, now_s - gps.timestamp_s) * 1000.0
            sats = "sats=--" if gps.satellites_used is None else f"sats={gps.satellites_used}"
            eph = "" if gps.eph_m is None else f" | eph={gps.eph_m:.2f}m"
            if gps.latitude_deg is None or gps.longitude_deg is None:
                position = " | pos=--"
            else:
                height_m = gps.altitude_ellipsoid_m if gps.altitude_ellipsoid_m is not None else gps.altitude_m
                height_text = "--" if height_m is None else f"{height_m:.3f}m"
                position = f" | {gps.latitude_deg:.7f}, {gps.longitude_deg:.7f} | h={height_text}"
            text = f"{gps.rtk_label} | {sats}{eph}{position} | age={age_ms:.0f}ms"
            self.panel._set_health_label(labels[key], text, self.panel._gps_fix_level(gps.fix_type, age_ms=age_ms))

        baseline = self.panel.latest_rtk_baseline
        if baseline is None:
            self.panel._set_health_label(labels["baseline"], "대기 중", "idle")
            return
        usable, reason = self.panel._gps_baseline_usable_for_translation(baseline)
        level = "good" if usable and baseline.fix_quality == "GOOD" else "warn" if usable else "bad"
        text = (
            f"{baseline.baseline_m:.3f} m | {baseline.fix_quality} | "
            f"N/E/D={baseline.north_m:+.3f}/{baseline.east_m:+.3f}/{baseline.down_m:+.3f} m"
        )
        if not usable:
            text = f"{text} | {reason}"
        self.panel._set_health_label(labels["baseline"], text, level)

    def _gnss_selected_operation_mode(self) -> GnssOperationMode:
        value = self.panel.gnss_operation_mode_combo.currentData()
        try:
            return GnssOperationMode(value)
        except (TypeError, ValueError):
            return GnssOperationMode.LOCAL_SURVEY_IN

    def _gnss_active_operation_config(self) -> GnssOperationConfig:
        """Return the immutable active-session config, or the configured default.

        UI widgets are editable presentation state.  Once an operation starts,
        correction routing and datum semantics must come from the snapshot that
        was configured at start, never from a later combo-box change.
        """

        return self.panel.gnss_active_operation_config or self.panel._build_gnss_operation_config()

    def _set_gnss_operation_controls_locked(self, locked: bool) -> None:
        """Prevent mutable GNSS settings from changing an active operation."""

        names = (
            "gnss_operation_mode_combo",
            "gnss_site_label_edit",
            "gnss_site_address_edit",
            "gnss_ntrip_host_edit",
            "gnss_ntrip_port_spin",
            "gnss_ntrip_mountpoint_edit",
            "gnss_ntrip_username_env_edit",
            "gnss_ntrip_password_env_edit",
            "gnss_ntrip_gga_policy_combo",
            "gnss_fixed_height_combo",
            "gnss_fixed_acquisition_duration_spin",
            "gnss_fixed_min_duration_spin",
            "gnss_rover_min_fix_type_spin",
            "gnss_base_rtcm_target_combo",
            "gnss_base_serial_port_edit",
            "gnss_base_baud_spin",
        )
        for name in names:
            control = getattr(self.panel, name, None)
            if control is not None:
                control.setEnabled(not locked)

    def _build_gnss_operation_config(self) -> GnssOperationConfig:
        """Build the persisted GNSS contract from the visible controls.

        ``network.username`` is plain text and safe to persist.  The
        password is never read here and never enters this config -- see
        ``_resolve_ntrip_credentials`` for how it is looked up at connect
        time.  The raw serial and MAVLink paths are still owned by their
        existing widgets; this object records how correction bytes are
        sourced and routed.
        """

        previous = self.panel._workstation_setting_overrides.get("gnss_operation", self.panel.settings.gnss_operation)
        if not isinstance(previous, GnssOperationConfig):
            previous = GnssOperationConfig.from_mapping(previous)
        height_value = self.panel.gnss_fixed_height_combo.currentData()
        if height_value not in {
            HeightType.ELLIPSOID.value,
            HeightType.MSL.value,
            HeightType.UNKNOWN.value,
        }:
            height_value = HeightType.ELLIPSOID.value
        target = str(self.panel.gnss_base_rtcm_target_combo.currentData() or "disabled")
        mapping = previous.to_mapping()
        mapping.update(
            {
                "site_label": self.panel.gnss_site_label_edit.text().strip(),
                "site_address": self.panel.gnss_site_address_edit.text().strip(),
                "operation_mode": self.panel._gnss_selected_operation_mode().value,
                "local_base": {
                    **previous.local_base.to_mapping(),
                    "device": self.panel.gnss_base_serial_port_edit.text().strip(),
                    "baud": int(self.panel.gnss_base_baud_spin.value()),
                    "survey_in_min_duration_s": float(self.panel.gnss_base_survey_duration_spin.value()),
                    "survey_in_accuracy_limit_m": float(self.panel.gnss_base_survey_accuracy_spin.value()),
                },
                "network": {
                    **previous.network.to_mapping(),
                    "caster_host": self.panel.gnss_ntrip_host_edit.text().strip(),
                    "caster_port": int(self.panel.gnss_ntrip_port_spin.value()),
                    "mountpoint": self.panel.gnss_ntrip_mountpoint_edit.text().strip(),
                    "username": self.panel.gnss_ntrip_username_edit.text().strip(),
                    "remember_password": self.panel.gnss_ntrip_remember_checkbox.isChecked(),
                    "username_env": self.panel.gnss_ntrip_username_env_edit.text().strip() or "NTRIP_USERNAME",
                    "password_env": self.panel.gnss_ntrip_password_env_edit.text().strip() or "NTRIP_PASSWORD",
                    "gga_policy": str(
                        self.panel.gnss_ntrip_gga_policy_combo.currentData() or GgaPolicy.BASE_GNSS.value
                    ),
                },
                "fixed_base": {
                    **previous.fixed_base.to_mapping(),
                    "acquisition_duration_s": float(self.panel.gnss_fixed_acquisition_duration_spin.value()),
                    "minimum_fixed_duration_s": float(self.panel.gnss_fixed_min_duration_spin.value()),
                    "height_type": height_value,
                },
                "readiness": {
                    **previous.readiness.to_mapping(),
                    "minimum_fix_type": int(self.panel.gnss_rover_min_fix_type_spin.value()),
                },
                "routing": {
                    **previous.routing.to_mapping(),
                    "left_enabled": target in {"both", "left"},
                    "right_enabled": target in {"both", "right"},
                    "field_base_enabled": self.panel._gnss_selected_operation_mode()
                    == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE,
                },
            }
        )
        return GnssOperationConfig.from_mapping(mapping)

    def _gnss_configure_manager(self, config: GnssOperationConfig) -> None:
        self._operation_runtime().configure(config)

    def _gnss_status_label_key_for_state(self, state: GnssOperationState) -> str:
        group = STATE_STATUS_GROUP.get(state, GnssStatusGroup.READINESS)
        if group is GnssStatusGroup.NETWORK:
            return "network"
        if group is GnssStatusGroup.BASE_FIXED:
            return "base_fixed"
        return "operation"

    def _gnss_transition(self, state: GnssOperationState, reason: str) -> bool:
        return self._operation_runtime().transition(state, reason)

    def _present_gnss_transition(self, state, reason, error):
        if error is not None:
            self._gnss_health(self._gnss_status_label_key_for_state(state), f"상태 전환 실패: {error}", "bad")
            self.panel._log(f"[GNSS] 상태 전환 실패: {error}")
            return
        new_state = state
        group = STATE_STATUS_GROUP.get(new_state, GnssStatusGroup.READINESS)
        if group is GnssStatusGroup.NETWORK:
            # Group A (NTRIP) already has a dedicated "network" label kept up
            # to date by the NTRIP handlers themselves; writing NTRIP_CONNECTING/
            # NTRIP_ACTIVE transitions here would just duplicate/race with that.
            # But in NETWORK_ASSISTED_FIXED_BASE, Group B (base_fixed) can't
            # start until this NTRIP handshake finishes, so surface *why*
            # base_fixed is still idle instead of leaving stale placeholder
            # text with no explanation.
            if (
                self.panel._gnss_active_operation_config().operation_mode
                == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE
                and "base_fixed" in self.panel.gnss_base_status_labels
            ):
                waiting_text = (
                    "NTRIP 연결 대기 중"
                    if new_state == GnssOperationState.NTRIP_CONNECTING
                    else "NTRIP 연결됨 | Base 위치 샘플 수신 대기 중"
                )
                self.panel._set_health_label(self.panel.gnss_base_status_labels["base_fixed"], waiting_text, "warn")
        else:
            label_key = "base_fixed" if group is GnssStatusGroup.BASE_FIXED else "operation"
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels[label_key],
                f"{new_state.value} | {reason}",
                "good" if new_state == GnssOperationState.READY else "warn",
            )

    def _handle_gnss_operation_mode_changed(self) -> None:
        if not hasattr(self.panel, "gnss_operation_mode_combo"):
            return
        mode = self.panel._gnss_selected_operation_mode()
        local_mode = mode == GnssOperationMode.LOCAL_SURVEY_IN
        assisted_fixed_mode = mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE
        serial_allowed = mode in {
            GnssOperationMode.LOCAL_SURVEY_IN,
            GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE,
        }
        network_allowed = not local_mode

        # An assisted Fixed Base needs the Base receiver's current WGS84
        # position as the NTRIP GGA hint.  Make that choice automatic when
        # the operator selects the assisted path; direct mode may still use
        # another rover/session policy.
        if assisted_fixed_mode and hasattr(self.panel, "gnss_ntrip_gga_policy_combo"):
            self.panel._set_combo_data(self.panel.gnss_ntrip_gga_policy_combo, GgaPolicy.BASE_GNSS.value)
        elif (
            not assisted_fixed_mode
            and hasattr(self.panel, "gnss_ntrip_gga_policy_combo")
            and self.panel.gnss_ntrip_gga_policy_combo.currentData() == GgaPolicy.BASE_GNSS.value
        ):
            # BASE_GNSS has no position source outside assisted Fixed Base --
            # only that mode feeds latest_gnss_base_position from a local
            # Base receiver.  Leaving a policy of BASE_GNSS selected here
            # would silently stop sending GGA to a VRS mountpoint.
            self.panel._set_combo_data(self.panel.gnss_ntrip_gga_policy_combo, GgaPolicy.SYSTEM_CENTER.value)

        # Keep the settings page focused on the currently selected correction
        # path.  The underlying widgets remain alive so persisted settings and
        # the existing operation-config builder keep their stable interface.
        visibility = (
            ("gnss_base_config_group", serial_allowed),
            ("gnss_survey_config_group", local_mode),
            ("gnss_network_config_group", network_allowed),
            ("gnss_network_legacy_env_group", network_allowed),
            ("gnss_fixed_config_group", assisted_fixed_mode),
            ("gnss_base_candidate_group", serial_allowed),
            ("gnss_survey_progress_group", local_mode),
            ("gnss_survey_quality_group", local_mode),
            ("gnss_base_fixed_progress_group", serial_allowed),
        )
        for name, visible in visibility:
            group = getattr(self.panel, name, None)
            if group is not None:
                group.setVisible(visible)

        # The "survey_in" row inside base_group only means anything in
        # LOCAL_SURVEY_IN (NAV-SVIN); the underlying poll runs every mode
        # (NAV-PVT is needed regardless), so hide the row itself rather than
        # let a stale/irrelevant value keep showing.
        for key, visible in (("survey_in", local_mode),):
            value_label = (
                self.panel.gnss_base_status_labels.get(key) if hasattr(self.panel, "gnss_base_status_labels") else None
            )
            title_label = (
                self.panel.gnss_base_status_title_labels.get(key)
                if hasattr(self.panel, "gnss_base_status_title_labels")
                else None
            )
            if value_label is not None:
                value_label.setVisible(visible)
            if title_label is not None:
                title_label.setVisible(visible)

        for name, visible in (
            ("gnss_base_discover_button", serial_allowed),
            ("gnss_base_connect_button", serial_allowed),
            ("gnss_base_disconnect_button", serial_allowed),
            ("gnss_base_survey_start_button", local_mode),
            ("gnss_base_survey_stop_button", local_mode),
            ("gnss_network_start_button", network_allowed),
            ("gnss_network_stop_button", network_allowed),
            ("gnss_base_apply_candidate_button", serial_allowed),
        ):
            button = getattr(self.panel, name, None)
            if button is not None:
                button.setVisible(visible)

        if hasattr(self.panel, "gnss_base_connect_button"):
            self.panel.gnss_base_connect_button.setEnabled(
                serial_allowed and self.panel.gnss_base_serial_thread is None
            )
        if hasattr(self.panel, "gnss_base_discover_button"):
            self.panel.gnss_base_discover_button.setEnabled(
                serial_allowed and self.panel.gnss_base_detect_thread is None
            )
        if hasattr(self.panel, "gnss_base_survey_start_button"):
            self.panel.gnss_base_survey_start_button.setEnabled(local_mode and self.panel.gnss_base_serial_connected)
        if hasattr(self.panel, "gnss_base_survey_stop_button"):
            self.panel.gnss_base_survey_stop_button.setEnabled(local_mode and self.panel.gnss_base_serial_connected)
        if hasattr(self.panel, "gnss_network_start_button"):
            self.panel.gnss_network_start_button.setEnabled(network_allowed and self.panel.gnss_network_thread is None)
        if hasattr(self.panel, "gnss_base_status_labels") and "operation" in self.panel.gnss_base_status_labels:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["operation"],
                f"{mode.value} | {'현장 serial' if local_mode else 'NTRIP correction'}",
                "idle"
                if self.panel.gnss_operation_manager.state
                in {GnssOperationState.DISCONNECTED, GnssOperationState.STOPPED}
                else "warn",
            )

    def _gnss_source_identity(self, config: GnssOperationConfig) -> CorrectionSourceIdentity:
        return self._operation_runtime().source_identity(config)

    def _gga_owner(self) -> GgaSentenceOwner:
        application = getattr(self.panel, "application", None)
        features = getattr(application, "features", {})
        feature = features.get("gnss") if isinstance(features, dict) else None
        owner = getattr(getattr(feature, "state", None), "gga", None)
        if owner is not None:
            return owner
        owner = getattr(self, "_legacy_gga", None)
        if owner is None:

            def publish(value):
                for name in GGA_STATE_FIELDS:
                    setattr(self.panel, name, getattr(value, name))

            owner = GgaSentenceOwner(publish)
            self._legacy_gga = owner
        for name in GGA_STATE_FIELDS:
            if hasattr(self.panel, name):
                setattr(owner, name, getattr(self.panel, name))
        return owner

    def _gnss_gga_provider(self) -> str | None:
        """Return the current GGA sentence, or ``None`` with a logged reason.

        A VRS caster (e.g. NGII's RTS2.ngii.go.kr) silently withholds RTCM
        while it has no periodic GGA to compute the virtual station against
        -- the TCP transport stays "connected" the whole time, so nothing
        else here would ever surface that.  ``_gnss_last_gga_skip_reason``
        records why the *last* call could not produce a sentence (cleared as
        soon as one succeeds again) so the NTRIP status panel can show it.
        """

        sentence = self._resolve_gga_sentence()
        if sentence is not None:
            self._gga_owner().clear_skip()
        return sentence

    def _resolve_gga_sentence(self) -> str | None:
        config = self.panel._gnss_active_operation_config()
        policy = config.network.gga_policy
        base = getattr(self.panel, "latest_gnss_base_position", None) if policy == GgaPolicy.BASE_GNSS else None
        left = right = None
        if policy not in {GgaPolicy.BASE_GNSS, GgaPolicy.MANUAL}:
            left = self._selected_rover_gps_state("left")
            right = self._selected_rover_gps_state("right")
        return self._gga_owner().resolve(
            config, base_sample=base, left=left, right=right, log=lambda message: self.panel._log(message)
        )

    def _resolve_ntrip_credentials(self, network) -> NtripCredentialResolution:
        password = self.panel.gnss_ntrip_password_edit.text() if hasattr(self.panel, "gnss_ntrip_password_edit") else ""
        inputs = CredentialInputs(
            password=password,
            explicit=bool(getattr(self.panel, "_gnss_ntrip_session_password_explicit", True)),
            account_key=getattr(self.panel, "_gnss_ntrip_session_credential_key", None),
        )
        return resolve_ntrip_credential(network, inputs)

    def _maybe_persist_ntrip_credential(self, network, credential: NtripCredentialResolution) -> None:
        maybe_persist_ntrip_credential(network, credential)

    def _commit_pending_ntrip_credential(self, status_line: str) -> None:
        """Persist a typed credential only after the caster accepted it."""

        runtime = getattr(self, "_ntrip_owner", None)
        if runtime is not None:
            pending = runtime.take_accepted_credential(status_line)
        else:
            pending = getattr(self.panel, "_gnss_ntrip_pending_credential", None)
        if not pending:
            return
        network, credential = pending
        if " 200 " not in f" {str(status_line).upper()} " and not str(status_line).upper().endswith(" 200"):
            return
        self._maybe_persist_ntrip_credential(network, credential)
        self.panel._gnss_ntrip_pending_credential = None
        if hasattr(self.panel, "_refresh_ntrip_credential_status"):
            self.panel._refresh_ntrip_credential_status()

    def _ntrip_runtime(self) -> NtripSessionRuntime:
        runtime = getattr(self, "_ntrip_owner", None)
        if runtime is not None:
            return runtime
        application = getattr(self.panel, "application", None)
        features = getattr(application, "features", {})
        feature = features.get("gnss") if isinstance(features, dict) else None
        state = getattr(getattr(feature, "state", None), "ntrip", None)
        if state is None:
            # Only lightweight compatibility panels need mirrored handles.
            def publish(value):
                for name in NTRIP_FIELDS:
                    setattr(self.panel, name, getattr(value, name))

            state = NtripSessionState(publish)
            for name in NTRIP_FIELDS:
                setattr(state, name, getattr(self.panel, name, None))
        runtime = NtripSessionRuntime(
            self.panel if isinstance(self.panel, QtCore.QObject) else None,
            state,
            worker_factory=lambda **kwargs: NtripNetworkWorker(**kwargs),
            connected=self._apply_gnss_network_connected,
            reconnecting=self._apply_gnss_network_reconnecting,
            failed=self._apply_gnss_network_failed,
            diagnostic=self._apply_gnss_network_diagnostic,
            retired=self._ntrip_retired,
        )
        self._ntrip_owner = runtime
        return runtime

    def resource_inventory(self):
        resources = {}
        for name in ("_ntrip_owner", "_base_owner", "_correction_owner"):
            runtime = getattr(self, name, None)
            if runtime is not None:
                resources.update(runtime.resource_inventory())
        return resources

    def _ntrip_retired(self) -> None:
        self.panel.gnss_network_connected = False
        if getattr(self.panel, "gnss_network_phase", "") not in {"ERROR", "HTTP_AUTH_FAILED"}:
            self.panel.gnss_network_phase = "STOPPED"
        self.panel.gnss_active_operation_config = None
        self.panel._set_gnss_operation_controls_locked(False)
        if hasattr(self.panel, "gnss_network_start_button"):
            self.panel.gnss_network_start_button.setEnabled(
                self.panel._gnss_selected_operation_mode() != GnssOperationMode.LOCAL_SURVEY_IN
            )
        if hasattr(self.panel, "gnss_network_stop_button"):
            self.panel.gnss_network_stop_button.setEnabled(False)

    def _reject_ntrip_start(self, message: str, tone: str) -> None:
        """Every guard below routes through here so a rejected Start NTRIP
        click always leaves a line in the visible log, not only the small
        "NTRIP" status label in the GNSS Base status grid -- a click that
        only updates that label is easy to miss and reads as "did nothing".
        """

        self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], message, tone)
        self.panel._log(f"[GNSS] NTRIP 시작 거부: {message}")
        self._publish_measurement_source("ntrip", "UNAVAILABLE", str(message))

    def _rollback_ntrip_start_setup(self, message: str) -> None:
        """Undo the UI/state reservation when no network worker was started."""

        text = str(message)
        self.panel.gnss_operation_gate_active = False
        self.panel.gnss_active_operation_config = None
        self.panel.gnss_network_connected = False
        self.panel.gnss_network_phase = "ERROR"
        self.panel.gnss_network_last_error = text
        self.panel._gnss_ntrip_pending_credential = None
        self.panel._set_gnss_operation_controls_locked(False)
        if hasattr(self.panel, "gnss_network_start_button"):
            self.panel.gnss_network_start_button.setEnabled(
                self.panel._gnss_selected_operation_mode() != GnssOperationMode.LOCAL_SURVEY_IN
            )
        if hasattr(self.panel, "gnss_network_stop_button"):
            self.panel.gnss_network_stop_button.setEnabled(False)
        self.panel._gnss_transition(
            GnssOperationState.FAULT,
            f"NTRIP setup failure: {text}",
        )

    def _prepare_ntrip_rtcm_route(self, route: RtcmRouteConfig) -> bool:
        """Prepare optional Pixhawk delivery without gating NTRIP transport.

        A direct Network RTK operation can still prove caster authentication and
        receive/parse RTCM before a Pixhawk endpoint is available.  The missing
        endpoint remains a delivery/readiness failure, but it is no longer a
        reason to prevent the independent NTRIP worker from starting.
        """

        if not route.ntrip_targets:
            return True
        if self.panel.mavlink_pixhawk is None:
            message = "NTRIP transport 시작 | MAVLink RTCM route 대기 중"
            self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], message, "warn")
            self.panel._log(f"[GNSS] {message}")
            return True
        if self._ensure_pixhawk_rtcm_worker() is None:
            self._reject_ntrip_start("RTCM 주입 worker를 시작할 수 없습니다", "bad")
            return False
        return True

    def _ntrip_route_pending(self) -> bool:
        """Return whether a connected NTRIP stream lacks its Pixhawk route."""

        try:
            route = self._rtcm_route_config()
        except (AttributeError, TypeError, ValueError):
            # Lightweight status/test adapters may expose transport state
            # without the full GNSS routing surface. Their connection status
            # must retain the transport-only behavior.
            return False
        return bool(route.ntrip_targets and self.panel.mavlink_pixhawk is None)

    def start_gnss_network(self) -> None:
        runtime = getattr(self, "_ntrip_owner", None)
        if self.panel.gnss_network_thread is not None or (runtime is not None and runtime.busy):
            return
        config = self.panel._build_gnss_operation_config()
        assisted = config.operation_mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE
        sample = getattr(self.panel, "latest_gnss_base_position", None) if assisted else None
        request = NetworkAdmissionInputs(
            mode=config.operation_mode,
            caster_host=config.network.caster_host,
            mountpoint=config.network.mountpoint,
            gga_policy=config.network.gga_policy,
            fixed_height_type=config.fixed_base.height_type,
            base_serial_connected=bool(getattr(self.panel, "gnss_base_serial_connected", False)),
            base_position_present=sample is not None,
            base_position_monotonic_s=None if sample is None else sample.timestamp_s,
            now_monotonic_s=time.monotonic() if sample is not None else 0.0,
        )
        decision = GnssOperationRuntime.assess_network_start(request)
        if not decision.accepted:
            self._reject_ntrip_start(decision.message, decision.tone)
            return
        try:
            credential = self._resolve_ntrip_credentials(config.network)
        except NtripCredentialError as exc:
            self._reject_ntrip_start(str(exc), "bad")
            return
        try:
            if not self._operation_runtime().begin_network(config):
                return
        except (ValueError, RuntimeError) as exc:
            self._reject_ntrip_start(f"운용 설정 실패: {exc}", "bad")
            return
        self.panel.gnss_network_connected = False
        self.panel.gnss_network_phase = "CONNECTING"
        self._publish_measurement_source("ntrip", "WAITING_RESOURCE", "NTRIP caster connect requested")
        self.panel.gnss_network_stats = None
        self.panel.gnss_network_status_line = ""
        self.panel.gnss_network_last_error = ""
        self.panel.gnss_network_reconnect_count = 0
        self.panel.gnss_network_rtcm_parser = Rtcm3StreamParser()
        self._base_runtime().begin_network_acquisition()
        if hasattr(self.panel, "gnss_base_fixed_state_plot"):
            self.panel.gnss_base_fixed_state_plot.points.clear()
        if hasattr(self.panel, "gnss_base_fixed_scatter_plot"):
            self.panel.gnss_base_fixed_scatter_plot.points.clear()
        self.panel._reset_gnss_base_rtcm_stats()
        self.panel.gnss_network_start_button.setEnabled(False)
        self.panel.gnss_network_stop_button.setEnabled(True)
        self.panel._set_gnss_operation_controls_locked(True)
        self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], "NTRIP 연결 중", "warn")
        self.panel._gnss_ntrip_pending_credential = (
            (
                config.network,
                credential,
            )
            if credential.source == "session"
            else None
        )
        self.panel._gnss_ntrip_active_credential_source = credential.source
        self.panel._gnss_ntrip_active_credential_key = _ntrip_credential_account_key(
            config.network.caster_host,
            config.network.mountpoint,
            credential.username,
        )
        self.panel._refresh_ntrip_credential_status()
        try:
            pipeline = self._ensure_rtcm_pipeline()
            route = self._rtcm_route_config()
            if not self._prepare_ntrip_rtcm_route(route):
                self._rollback_ntrip_start_setup("RTCM 주입 worker를 시작할 수 없습니다")
                return
            self._sync_rtcm_pipeline_config(reset=True)
            self._sync_rtcm_evidence_context()
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            message = f"NTRIP RTCM 준비 실패: {exc}"
            self._reject_ntrip_start(message, "bad")
            self._rollback_ntrip_start_setup(message)
            return
        try:
            self._ntrip_runtime().start(
                config.network,
                credential,
                self.panel._gnss_gga_provider,
                wire_data_plane=lambda worker, token: self._wire_rtcm_ntrip_source(
                    worker, pipeline, session_token=token
                ),
            )
        except Exception as exc:
            message = f"NTRIP worker 시작 실패: {exc}"
            self._reject_ntrip_start(message, "bad")
            runtime = getattr(self, "_ntrip_owner", None)
            if runtime is None or not runtime.busy:
                self._rollback_ntrip_start_setup(message)
            else:
                # A partially started thread still owns its handles/locked UI.
                # Stop has already been requested by the runtime.
                self._apply_gnss_network_failed(message)

    def stop_gnss_network(self) -> bool:
        runtime = getattr(self, "_ntrip_owner", None)
        if runtime is not None:
            runtime.request_stop()
        else:
            worker = self.panel.gnss_network_worker
            if worker is not None:
                worker.request_stop()
        thread = self.panel.gnss_network_thread
        self.panel.gnss_network_connected = False
        self.panel.gnss_network_phase = "STOPPED"
        self._publish_measurement_source("ntrip", "LEFT", "NTRIP stopped")
        self.panel._gnss_ntrip_pending_credential = None
        self.panel._gnss_ntrip_active_credential_source = ""
        self.panel._gnss_ntrip_active_credential_key = None
        self.panel.latest_gnss_base_position = None
        self.panel.gnss_operation_gate_active = False
        self.panel.gnss_operation_manager.stop()
        if hasattr(self.panel, "gnss_base_status_labels"):
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["network"],
                "NTRIP 종료 중" if thread is not None and thread.isRunning() else "NTRIP 중지",
                "warn" if thread is not None and thread.isRunning() else "idle",
            )
        if thread is None or not thread.isRunning():
            self.panel.gnss_active_operation_config = None
            self._sync_rtcm_pipeline_config()
            self.panel._set_gnss_operation_controls_locked(False)
            self.panel._handle_gnss_operation_mode_changed()
            return True
        return False

    def _ntrip_credential_lookup_key(self) -> tuple[str, str, str]:
        config = self.panel._build_gnss_operation_config()
        network = config.network
        return network.caster_host, network.mountpoint, network.username

    def _refresh_ntrip_credential_status(self) -> None:
        """Recompute and display whether a password is currently saved.

        Never shows the password itself -- only Saved securely / Not saved /
        Unavailable, per the current host/mountpoint/username.
        """

        if not hasattr(self.panel, "gnss_ntrip_credential_status_label"):
            return
        host, mountpoint, username = self._ntrip_credential_lookup_key()
        status = ntrip_credentials.credential_status(host, mountpoint, username)
        text, tone = {
            ntrip_credentials.CredentialStorageStatus.SAVED: ("Saved securely", "good"),
            ntrip_credentials.CredentialStorageStatus.NOT_SAVED: ("Not saved", "idle"),
            ntrip_credentials.CredentialStorageStatus.UNAVAILABLE: ("Unavailable (session only)", "warn"),
        }[status]
        self.panel._set_health_label(self.panel.gnss_ntrip_credential_status_label, text, tone)

    def forget_gnss_ntrip_password(self) -> None:
        host, mountpoint, username = self._ntrip_credential_lookup_key()
        ntrip_credentials.delete_password(host, mountpoint, username)
        self.panel._refresh_ntrip_credential_status()

    def set_gnss_ntrip_session_password(
        self,
        password: str,
        *,
        caster_host: str | None = None,
        mountpoint: str | None = None,
        username: str | None = None,
    ) -> None:
        """Copy a typed password into the widget ``_resolve_ntrip_credentials``
        actually reads.

        The Settings workspace has its own separate "NTRIP caster (any
        provider)" password field so the value never has to round-trip
        through ``settings_payload()``'s JSON-safe dict (the password is
        deliberately not a ``NetworkConfig`` field -- see
        ``_resolve_ntrip_credentials``). That field lives on the
        ``SettingsWorkspace`` widget, not on ``panel``, so it must be handed
        over explicitly by a caller that has both objects in scope
        (``main_window.py``'s ``_settings_apply``) rather than discovered
        here. Without this bridge, typing a password there and clicking
        Apply Settings never reached ``gnss_ntrip_password_edit``, so Start
        NTRIP kept reporting "no saved password" no matter what was typed.
        """

        if hasattr(self.panel, "gnss_ntrip_password_edit"):
            self.panel.gnss_ntrip_password_edit.setText(str(password))
            self.panel._gnss_ntrip_session_password_explicit = True
            if caster_host is None or mountpoint is None or username is None:
                try:
                    caster_host, mountpoint, username = self._ntrip_credential_lookup_key()
                except (AttributeError, TypeError, ValueError):
                    caster_host = mountpoint = username = None
            self.panel._gnss_ntrip_session_credential_key = (
                None
                if caster_host is None or mountpoint is None or username is None
                else _ntrip_credential_account_key(caster_host, mountpoint, username)
            )

    def _handle_gnss_network_connected(self, status_line: str, stats: object) -> None:
        # Untagged compatibility calls are valid only before a real session
        # owner is installed. Admitted signals arrive through its GUI relay.
        if getattr(self, "_ntrip_owner", None) is None:
            self._apply_gnss_network_connected(status_line, stats)

    def _apply_gnss_network_connected(self, status_line: str, stats: object) -> None:
        self._commit_pending_ntrip_credential(status_line)
        self.panel.gnss_network_connected = True
        self.panel.gnss_network_phase = "CONNECTED"
        self.panel.gnss_network_stats = stats
        self.panel.gnss_network_last_error = ""
        self.panel.gnss_network_status_line = str(status_line)
        self._publish_measurement_source("ntrip", "PENDING", f"caster connected: {status_line}; RTCM pending")
        # NtripNetworkWorker reconnects transparently on any transient stream
        # drop (a caster closing the connection, a network blip) and
        # re-emits ``connected`` every time -- including well after the
        # operation state machine has already progressed past NTRIP_ACTIVE to
        # READY/MEASURING. Only INITIALIZING/NTRIP_CONNECTING can legally
        # transition into NTRIP_ACTIVE; attempting it again from a later
        # state used to log a scary "상태 전환 실패: invalid GNSS state
        # transition" fault on every ordinary reconnect, even though RTCM
        # kept flowing the whole time -- chunk routing in
        # _handle_gnss_network_chunk never depended on this state machine.
        current_state = self.panel.gnss_operation_manager.state
        if current_state in {GnssOperationState.INITIALIZING, GnssOperationState.NTRIP_CONNECTING}:
            self.panel._gnss_transition(GnssOperationState.NTRIP_ACTIVE, f"NTRIP active: {status_line}")
        if self._ntrip_route_pending():
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["network"],
                f"{status_line} | RTCM 대기 | MAVLink route 대기",
                "warn",
            )
        else:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["network"],
                f"{status_line} | RTCM 대기",
                "good",
            )

    def _handle_gnss_network_stats(self, stats: object) -> None:
        # Untagged compatibility calls are valid only before a real session
        # owner is installed. Admitted signals arrive through its GUI relay.
        if getattr(self, "_ntrip_owner", None) is None:
            self._apply_gnss_network_stats(stats)

    def _apply_gnss_network_stats(self, stats: object) -> None:
        self.panel.gnss_network_stats = stats
        if self.panel.gnss_network_connected and getattr(stats, "last_chunk_monotonic_s", None) is not None:
            self.panel.gnss_network_phase = "STREAMING"

    def _handle_gnss_network_chunk(self, chunk: object) -> None:
        # Untagged compatibility calls are valid only before a real session
        # owner is installed. Admitted signals arrive through its GUI relay.
        if getattr(self, "_ntrip_owner", None) is None:
            self._apply_gnss_network_chunk(chunk)

    def _apply_gnss_network_chunk(self, chunk: object) -> None:
        if not isinstance(chunk, (bytes, bytearray)):
            return
        payload = bytes(chunk)
        if payload:
            self._publish_measurement_source("ntrip", "LIVE", "NTRIP transport bytes received")
        self._append_rtcm_package_evidence(payload, source="ntrip", record_kind="transport_chunk")
        mode = self.panel._gnss_active_operation_config().operation_mode
        frames = self.panel.gnss_network_rtcm_parser.feed(payload)
        ntrip_ui_due = self._ntrip_ui_update_due()
        if mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE:
            worker = self.panel.gnss_base_serial_worker
            if worker is None or not self.panel.gnss_base_serial_connected:
                if ntrip_ui_due:
                    self.panel._set_health_label(
                        self.panel.gnss_base_status_labels["network"],
                        "NTRIP 수신 중이나 Base serial이 끊겼습니다",
                        "bad",
                    )
            else:
                worker.send_bytes("NTRIP RTCM -> fixed base", payload)
                if ntrip_ui_due:
                    self.panel._set_health_label(
                        self.panel.gnss_base_status_labels["network"],
                        f"NTRIP {len(payload)} bytes -> fixed base",
                        "good",
                    )
            # Assisted Fixed Base still forwards the raw NTRIP stream to the
            # Base receiver. If Drone is configured to use Network NTRIP, its
            # complete RTCM frames are also delivered independently, without
            # changing the existing P1/P2 rover route.
            if frames and self._drone_rtcm_source() == DRONE_RTCM_SOURCE_NTRIP:
                for frame in frames:
                    self.panel._handle_gnss_base_rtcm_frame(
                        frame,
                        source=DRONE_RTCM_SOURCE_NTRIP,
                        include_rover_targets=False,
                    )
            return
        if not frames:
            if ntrip_ui_due:
                self.panel._set_health_label(
                    self.panel.gnss_base_status_labels["network"],
                    f"NTRIP raw {len(payload)} bytes | RTCM frame 대기",
                    "warn",
                )
            return
        for frame in frames:
            self.panel._handle_gnss_base_rtcm_frame(frame, source=DRONE_RTCM_SOURCE_NTRIP)
        if ntrip_ui_due:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["network"],
                f"NTRIP RTCM {len(frames)} frame 수신",
                "good",
            )

    def _handle_gnss_network_diagnostic(self, message: str) -> None:
        # Untagged compatibility calls are valid only before a real session
        # owner is installed. Admitted signals arrive through its GUI relay.
        if getattr(self, "_ntrip_owner", None) is None:
            self._apply_gnss_network_diagnostic(message)

    def _apply_gnss_network_diagnostic(self, message: str) -> None:
        """Surface NtripClient's per-event evidence (connect/GGA TX outcome).

        These events never stop or restart anything -- they carry no status
        of their own -- so without this they would never reach the visible
        log at all, unlike a stream failure which already flows through
        ``reconnecting``/``failed``.
        """

        self.panel._log(f"[GNSS][NTRIP] {message}")

    def _handle_gnss_network_failed(self, message: str) -> None:
        # Untagged compatibility calls are valid only before a real session
        # owner is installed. Admitted signals arrive through its GUI relay.
        if getattr(self, "_ntrip_owner", None) is None:
            self._apply_gnss_network_failed(message)

    def _apply_gnss_network_failed(self, message: str) -> None:
        self.panel.gnss_network_connected = False
        auth_failed = "401" in str(message) or "unauthorized" in str(message).lower()
        self.panel.gnss_network_phase = "HTTP_AUTH_FAILED" if auth_failed else "ERROR"
        self.panel.gnss_network_last_error = str(message)
        self._publish_measurement_source("ntrip", "UNAVAILABLE", str(message))
        text = str(message)
        # 401 means the caster rejected the credentials; it is deliberately
        # not asserted here whether the username or the password was wrong.
        if auth_failed:
            display = f"NTRIP 인증 거부 (username/password 확인 필요): {text}"
        else:
            display = f"NTRIP 실패: {text}"
        self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], display, "bad")
        # A worker-thread failure after credentials already resolved is the
        # one NTRIP fault that used to leave zero trace in the log -- only
        # this same small status label, which is exactly why a rejected
        # password after Start NTRIP "does nothing" a second time.
        self.panel._log(f"[GNSS] {display}")
        self.panel._gnss_ntrip_pending_credential = None
        self.panel._gnss_transition(GnssOperationState.FAULT, f"NTRIP failure: {text}")

    def _handle_gnss_network_reconnecting(self, message: str) -> None:
        # Untagged compatibility calls are valid only before a real session
        # owner is installed. Admitted signals arrive through its GUI relay.
        if getattr(self, "_ntrip_owner", None) is None:
            self._apply_gnss_network_reconnecting(message)

    def _apply_gnss_network_reconnecting(self, message: str) -> None:
        if not self.panel.gnss_operation_gate_active:
            return
        self.panel.gnss_network_connected = False
        self.panel.gnss_network_phase = "RECONNECTING"
        self.panel.gnss_network_last_error = str(message)
        self._publish_measurement_source("ntrip", "WAITING_RESOURCE", f"NTRIP reconnecting: {message}")
        self.panel.gnss_network_reconnect_count = int(getattr(self.panel, "gnss_network_reconnect_count", 0) or 0) + 1
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["network"],
            f"NTRIP 재연결 대기: {message}",
            "warn",
        )
        self.panel._log(f"[GNSS] NTRIP 재연결 시도 {self.panel.gnss_network_reconnect_count}: {message}")

    def _clear_gnss_network_worker(self, thread: QtCore.QThread) -> None:
        runtime = getattr(self, "_ntrip_owner", None)
        if runtime is not None:
            runtime.clear_finished_legacy(thread)
            return
        if self.panel.gnss_network_thread is thread:
            self.panel.gnss_network_thread = None
            self.panel.gnss_network_worker = None
            self._ntrip_retired()
        thread.deleteLater()

    def _record_gnss_base_position_row(self, sample: GnssCoordinateSample) -> None:
        self._evidence_runtime().record_base_position(
            sample,
            operation_state=self.panel.gnss_operation_manager.state.value,
            timeline=self.panel._raw_timeline_row,
            append_external=self.panel.external_parameter_recording_rows.append,
        )

    def _handle_gnss_base_position(self, sample: object) -> None:
        return self._base_runtime().observe_position(sample)

    def _fail_gnss_fixed_base_reconfiguration(self, reason: str) -> None:
        return self._base_runtime().fail_reconfiguration(reason)

    def _handle_gnss_base_ubx_message(self, message_class: int, message_id: int, payload: object) -> None:
        return self._base_runtime().observe_ubx(message_class, message_id, payload)

    def _maybe_activate_gnss_fixed_base(self) -> None:
        return self._base_runtime().maybe_activate()

    def _refresh_gnss_operation_readiness(self) -> None:
        if not self.panel.gnss_operation_gate_active:
            return
        # Keep drift invalidation ahead of the base timeout, preserving the old order.
        config = self.panel.gnss_operation_manager.config
        configured = (
            self.panel._build_gnss_operation_config()
            if self.panel.gnss_active_operation_config is not None
            and self.panel.gnss_operation_manager.state in {GnssOperationState.READY, GnssOperationState.MEASURING}
            else config
        )
        if (
            self.panel.gnss_operation_manager.state == GnssOperationState.BASE_RECONFIGURING
            and self._base_runtime().check_reconfiguration_timeout()
        ):
            return
        observation = ReadinessObservation(
            configured,
            self.panel._gnss_base_rtcm_target_sides(),
            self.panel.gnss_base_rtcm_stats,
            self.panel.mavlink_pixhawk,
            {side: self._selected_rover_gps_state(side) for side in ("left", "right", "drone")},
            self.panel.latest_baseline_vector,
            self.panel.latest_baseline_quality,
        )
        self._operation_runtime().refresh(observation)

    def connect_gnss_base_serial(self) -> None:
        if self.panel.gnss_base_serial_thread is not None:
            return
        port = self.panel.gnss_base_serial_port_edit.text().strip()
        if not port:
            self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], "serial port 필요", "bad")
            self._publish_measurement_source("gnss.base", "UNAVAILABLE", "Base GNSS serial port is missing")
            return
        baud = int(self.panel.gnss_base_baud_spin.value())
        self._publish_measurement_source("gnss.base", "WAITING_RESOURCE", "Base GNSS serial connect requested")
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["connection"], f"connecting {port} @ {baud}", "warn"
        )
        self.panel._set_health_label(self.panel.gnss_base_status_labels["rtcm_rx"], "raw bytes 대기 중", "idle")
        self.panel._reset_gnss_base_rtcm_stats()
        self.panel.gnss_base_connect_button.setEnabled(False)
        self.panel.gnss_base_disconnect_button.setEnabled(True)
        height_value = self.panel.gnss_fixed_height_combo.currentData()
        try:
            height_type = HeightType(height_value)
        except (TypeError, ValueError):
            height_type = HeightType.ELLIPSOID
        self._base_runtime().start_serial(
            port=port, baud=baud, height_type=height_type, wire=self._prepare_base_rtcm_route
        )

    def disconnect_gnss_base_serial(self) -> None:
        self._base_runtime().stop()
        thread = self.panel.gnss_base_serial_thread
        if thread is not None:
            # The worker's serial read is bounded to 0.2 s and its finished
            # signal quits the thread.  Do not freeze the GUI while waiting;
            # closeEvent's retryable lifecycle checks the same thread again.
            if hasattr(self.panel, "gnss_base_connect_button"):
                self.panel.gnss_base_connect_button.setEnabled(False)
            if hasattr(self.panel, "gnss_base_disconnect_button"):
                self.panel.gnss_base_disconnect_button.setEnabled(False)
            if hasattr(self.panel, "gnss_base_status_labels") and "connection" in self.panel.gnss_base_status_labels:
                self.panel._set_health_label(
                    self.panel.gnss_base_status_labels["connection"],
                    "disconnecting",
                    "warn",
                )

    def start_gnss_base_survey_in(self) -> None:
        if self._base_runtime().start_survey(
            self.panel._build_gnss_operation_config(),
            duration_s=self.panel.gnss_base_survey_duration_spin.value(),
            accuracy_m=self.panel.gnss_base_survey_accuracy_spin.value(),
        ):
            self.panel._set_gnss_operation_controls_locked(True)

    def stop_gnss_base_survey_in(self) -> None:
        if self._base_runtime().stop_survey():
            self.panel._set_gnss_operation_controls_locked(False)
            self._gnss_health("survey_in", "survey-in 중지 명령 전송 대기", "warn")
            self.panel._handle_gnss_operation_mode_changed()

    def discover_gnss_base_serial_ports(self) -> None:
        if self.panel.gnss_base_detect_thread is not None:
            return
        self.panel.gnss_base_discover_button.setEnabled(False)
        self.panel.workstation_gnss_base_discovery_status = "SEARCHING"
        self.panel.gnss_base_apply_candidate_button.setEnabled(False)
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], "base port 탐색 중", "warn")
        self._base_runtime().start_detection(baud=self.panel.gnss_base_baud_spin.value())

    def _handle_gnss_base_candidates_detected(self, rows: object) -> None:
        candidate_rows = [dict(row) for row in rows] if isinstance(rows, list) else []
        self.panel._set_gnss_base_candidate_rows(candidate_rows)
        count = len(candidate_rows)
        ubx_count = sum(1 for row in candidate_rows if row.get("ubx"))
        self.panel.workstation_gnss_base_discovery_status = f"FOUND {count} · UBX {ubx_count}"
        level = "good" if ubx_count else "warn" if count else "bad"
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["connection"],
            f"탐색 완료: {count} candidates, UBX {ubx_count}",
            level,
        )

    def _handle_gnss_base_candidate_detect_failed(self, message: str) -> None:
        self.panel.workstation_gnss_base_discovery_status = f"ERROR · {message}"
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], f"탐색 실패: {message}", "bad")

    def _clear_gnss_base_detect_worker(self, thread: QtCore.QThread) -> None:
        runtime = getattr(self, "_base_owner", None)
        binding = None if runtime is None else runtime.bindings["detect"]
        if binding is not None and binding.thread is thread:
            runtime.retire(binding, "detect")
            return
        if self.panel.gnss_base_detect_thread is thread:
            self.panel.gnss_base_detect_thread = None
            self.panel.gnss_base_detect_worker = None
            if hasattr(self.panel, "gnss_base_discover_button"):
                self.panel.gnss_base_discover_button.setEnabled(True)
            self.panel._refresh_gnss_base_candidate_actions()
        thread.deleteLater()

    def _set_gnss_base_candidate_rows(self, rows: list[dict[str, object]]) -> None:
        self.panel._gnss_base_candidate_rows = rows
        table = self.panel.gnss_base_candidate_table
        table.setRowCount(len(rows))
        keys = ("port", "target", "baud", "status", "messages")
        for row_index, row in enumerate(rows):
            for column, key in enumerate(keys):
                item = QtWidgets.QTableWidgetItem(str(row.get(key, "")))
                table.setItem(row_index, column, item)
        table.resizeRowsToContents()
        self.panel._refresh_gnss_base_candidate_actions()

    def _refresh_gnss_base_candidate_actions(self) -> None:
        row = (
            self.panel.gnss_base_candidate_table.currentRow()
            if hasattr(self.panel, "gnss_base_candidate_table")
            else -1
        )
        self.panel.gnss_base_apply_candidate_button.setEnabled(0 <= row < len(self.panel._gnss_base_candidate_rows))

    def apply_selected_gnss_base_candidate(self) -> None:
        row = self.panel.gnss_base_candidate_table.currentRow()
        self.panel.apply_gnss_base_candidate(row)

    def apply_gnss_base_candidate(self, row: int) -> bool:
        """Apply one discovered Base receiver without depending on the hidden table selection."""

        if isinstance(row, bool) or not isinstance(row, int):
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["connection"], "포트 후보 선택이 잘못되었습니다", "bad"
            )
            return False
        if row < 0 or row >= len(self.panel._gnss_base_candidate_rows):
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["connection"], "포트 후보를 먼저 선택하세요", "warn"
            )
            return False
        entry = self.panel._gnss_base_candidate_rows[row]
        port = str(entry.get("port", "")).strip()
        if not port:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["connection"], "선택 후보 port가 비어 있습니다", "bad"
            )
            return False
        self.panel.gnss_base_serial_port_edit.setText(port)
        try:
            baud = int(entry.get("baud", self.panel.gnss_base_baud_spin.value()))
        except (TypeError, ValueError):
            baud = self.panel.gnss_base_baud_spin.value()
        self.panel.gnss_base_baud_spin.setValue(baud)
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["connection"], f"선택 포트 적용: {port}", "good"
        )
        return True

    def _handle_gnss_base_serial_connected(self, port: str, baud: int) -> None:
        self.panel.gnss_base_serial_connected = True
        self._sync_rtcm_pipeline_config()
        self.panel.latest_gnss_base_position = None
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["connection"], f"connected {port} @ {baud}", "good"
        )
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "UBX NAV-SVIN 대기 중", "idle")
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["base_satellites"], "UBX NAV-PVT/NAV-SAT 대기 중", "idle"
        )
        self.panel._set_gnss_base_quality_waiting("NAV-SAT 대기 중")
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["base_position"], "Base WGS84 위치 샘플 대기 중", "idle"
        )
        self.panel._handle_gnss_operation_mode_changed()
        self._publish_measurement_source("gnss.base", "PENDING", f"serial connected {port} @ {baud}; data pending")

    def _handle_gnss_base_serial_stats(self, total_bytes: int, rate_bps: float, age_s: float) -> None:
        age_text = "no data" if age_s < 0.0 else f"age={age_s:.1f}s"
        level = "good" if rate_bps > 0.0 and 0.0 <= age_s < 2.0 else "warn" if rate_bps > 0.0 else "idle"
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["rtcm_rx"],
            f"raw={total_bytes} bytes | {rate_bps:.1f} B/s | {age_text}",
            level,
        )
        if rate_bps > 0.0 and 0.0 <= age_s < 2.0:
            self._publish_measurement_source("gnss.base", "LIVE", "fresh Base GNSS serial bytes received")

    def _handle_gnss_base_serial_failed(self, message: str) -> None:
        self.panel.gnss_base_serial_connected = False
        self._sync_rtcm_pipeline_config()
        self._reset_gnss_base_acquisition_state()
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], f"failed: {message}", "bad")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "미연결", "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_satellites"], "대기 중", "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["rtcm_rx"], "없음", "idle")
        self.panel._set_gnss_base_quality_waiting("미연결")
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["base_position"], "Base WGS84 위치 없음", "idle"
        )
        self.panel.gnss_base_survey_start_button.setEnabled(False)
        self.panel.gnss_base_survey_stop_button.setEnabled(False)
        self._publish_measurement_source("gnss.base", "UNAVAILABLE", str(message))
        if self.panel.gnss_operation_gate_active:
            self.panel._gnss_transition(GnssOperationState.DEGRADED, f"Base serial failure: {message}")
        else:
            # No live measurement gate depends on this session, so fully
            # rewind the state machine (stop() forces STOPPED from any
            # state; STOPPED -> DISCONNECTED is an allowed transition) so
            # the acquisition-progress UI reflects a clean, initial state.
            self.panel.gnss_operation_manager.stop()
            self.panel._gnss_transition(GnssOperationState.DISCONNECTED, f"Base serial failure: {message}")

    def _handle_gnss_base_serial_disconnected(self, message: str) -> None:
        self.panel.gnss_base_serial_connected = False
        self._sync_rtcm_pipeline_config()
        self._reset_gnss_base_acquisition_state()
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], message, "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "미연결", "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_satellites"], "대기 중", "idle")
        self.panel._set_gnss_base_quality_waiting("미연결")
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["base_position"], "Base WGS84 위치 없음", "idle"
        )
        self.panel.gnss_base_survey_start_button.setEnabled(False)
        self.panel.gnss_base_survey_stop_button.setEnabled(False)
        self._publish_measurement_source("gnss.base", "LEFT", str(message))
        if self.panel.gnss_operation_gate_active:
            self.panel._gnss_transition(GnssOperationState.DEGRADED, f"Base serial disconnected: {message}")
        else:
            # See _handle_gnss_base_serial_failed: no live gate to preserve,
            # so rewind to a clean DISCONNECTED state via the same stop()
            # escape hatch used elsewhere (e.g. _gnss_configure_manager).
            self.panel.gnss_operation_manager.stop()
            self.panel._gnss_transition(GnssOperationState.DISCONNECTED, f"Base serial disconnected: {message}")

    def _handle_gnss_base_satellite_count(self, count: int, source: str) -> None:
        self.panel.gnss_base_satellite_count = int(count)
        self.panel.gnss_base_satellite_count_source = str(source)
        level = "good" if count >= 12 else "warn" if count >= 6 else "bad"
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["base_satellites"],
            f"{count} satellites | {source}",
            level,
        )

    def _set_gnss_base_quality_waiting(self, text: str) -> None:
        if not hasattr(self.panel, "gnss_base_quality_labels"):
            return
        waiting_text = {
            "satellites": text,
            "signal": "C/N0 대기 중" if text != "미연결" else text,
            "elevation": "elevation 대기 중" if text != "미연결" else text,
            "constellations": "constellation 대기 중" if text != "미연결" else text,
        }
        for key, label in self.panel.gnss_base_quality_labels.items():
            self.panel._set_health_label(label, waiting_text.get(key, text), "idle")

    def _reset_gnss_base_rtcm_stats(self) -> None:
        self._correction_runtime().reset_stats()
        self.panel._refresh_gnss_base_rtcm_stats()

    def _reset_gnss_base_acquisition_state(self) -> None:
        return self._base_runtime().reset_acquisition()

    def _format_rtcm_message_types(self, message_types: dict[int, int]) -> str:
        if not message_types:
            return "type 대기 중"
        ordered = sorted(message_types.items(), key=lambda item: (-item[1], item[0]))[:8]
        return " | ".join(f"{message_type}:{count}" for message_type, count in ordered)

    def _refresh_gnss_base_rtcm_stats(self) -> None:
        if not hasattr(self.panel, "gnss_base_status_labels"):
            return
        labels = self.panel.gnss_base_status_labels
        if not all(
            key in labels for key in ("rtcm_receive", "rtcm_messages", "rtcm_injection_stats", "rtcm_bandwidth")
        ):
            return
        stats = self.panel.gnss_base_rtcm_stats
        now_s = time.monotonic()
        frames = int(stats.get("frames") or 0)
        total_bytes = int(stats.get("bytes") or 0)
        start_s = float(stats.get("start_s") or now_s)
        elapsed_s = max(1e-6, now_s - start_s)
        fps = frames / elapsed_s
        byte_rate = total_bytes / elapsed_s
        last_frame_s = stats.get("last_frame_s")
        age_s = None
        if last_frame_s is not None:
            try:
                age_s = max(0.0, now_s - float(last_frame_s))
            except (TypeError, ValueError):
                age_s = None
        if frames <= 0:
            receive_text = "RTCM 대기 중"
            receive_level = "idle"
        else:
            age_text = "--" if age_s is None else f"{age_s:.1f}s"
            receive_text = f"frames={frames} | {fps:.2f} fps | {byte_rate / 1000.0:.2f} kB/s | age={age_text}"
            receive_level = (
                "good"
                if age_s is not None and age_s <= 2.0
                else "warn"
                if age_s is not None and age_s <= 5.0
                else "bad"
            )
        message_types = stats.get("message_types")
        message_type_counts = message_types if isinstance(message_types, dict) else {}
        base_position_present = any(int(key) in {1005, 1006} for key in message_type_counts)
        msm_present = any(1070 <= int(key) <= 1129 for key in message_type_counts)
        message_level = "good" if base_position_present and msm_present else "warn" if frames else "idle"
        injection = stats.get("injection") if isinstance(stats.get("injection"), dict) else {}
        injection_parts: list[str] = []
        total_packets = 0
        total_fail = 0
        total_blocked_stall = 0
        for side, label in (("left", "L"), ("right", "R"), ("drone", "T")):
            side_stats = injection.get(side) if isinstance(injection, dict) else None
            if not isinstance(side_stats, dict):
                side_stats = {}
            side_frames = int(side_stats.get("frames") or 0)
            side_bytes = int(side_stats.get("bytes") or 0)
            side_packets = int(side_stats.get("packets") or 0)
            side_fail = int(side_stats.get("fail") or 0)
            side_blocked_stall = int(side_stats.get("blocked_stall") or 0)
            total_packets += side_packets
            total_fail += side_fail
            total_blocked_stall += side_blocked_stall
            injection_parts.append(
                f"{label} frames={side_frames} bytes={side_bytes} fail={side_fail} blocked_stall={side_blocked_stall}"
            )
        target_sides = (
            self.panel._gnss_base_rtcm_target_sides(source=self._drone_rtcm_source())
            if hasattr(self.panel, "gnss_base_rtcm_target_combo")
            else ()
        )
        if not target_sides:
            injection_text = "비활성"
            injection_level = "idle"
        else:
            injection_text = " | ".join(injection_parts)
            injection_level = (
                "good" if frames and total_fail == 0 and total_blocked_stall == 0 else "warn" if frames else "idle"
            )
        avg_frame = (total_bytes / frames) if frames else 0.0
        integrity = stats.get("integrity") if isinstance(stats.get("integrity"), dict) else {}
        ingress = stats.get("ingress") if isinstance(stats.get("ingress"), dict) else {}
        received_sequence = int(integrity.get("last_received_sequence") or 0)
        persisted_sequence = int(integrity.get("last_persisted_sequence") or 0)
        injected_sequence = int(integrity.get("last_injected_sequence") or 0)
        evidence_failed = int(integrity.get("evidence_failed") or 0)
        dropped_stale = int(integrity.get("injection_dropped_stale") or 0)
        blocked_stall = int(integrity.get("injection_blocked_stall") or 0)
        queue_depth = int(ingress.get("queued") or 0)
        backpressure_waits = int(ingress.get("backpressure_waits") or 0)
        integrity_text = (
            f"seq rx/persist/inject={received_sequence}/{persisted_sequence}/{injected_sequence}"
            f" | stale_drop={dropped_stale} | blocked_stall={blocked_stall}"
            f" | q={queue_depth} | bp={backpressure_waits}"
        )
        bandwidth_text = (
            "packet 대기 중"
            if frames <= 0
            else f"MAVLink packets={total_packets} | avg frame={avg_frame:.0f}B | max frame={int(stats.get('max_frame_bytes') or 0)}B"
        )
        bandwidth_text = f"{bandwidth_text} | {integrity_text}"
        bandwidth_level = (
            "bad"
            if evidence_failed
            else "warn"
            if total_fail or dropped_stale or blocked_stall or backpressure_waits
            else "good"
            if frames
            else "idle"
        )
        self.panel._set_health_label(labels["rtcm_receive"], receive_text, receive_level)
        self.panel._set_health_label(
            labels["rtcm_messages"], self.panel._format_rtcm_message_types(message_type_counts), message_level
        )
        self.panel._set_health_label(labels["rtcm_injection_stats"], injection_text, injection_level)
        self.panel._set_health_label(labels["rtcm_bandwidth"], bandwidth_text, bandwidth_level)

    def _handle_gnss_base_satellite_quality(self, quality: object) -> None:
        if not isinstance(quality, dict):
            return
        self.panel.gnss_base_satellite_quality = dict(quality)
        if not hasattr(self.panel, "gnss_base_quality_labels"):
            return
        try:
            tracked = int(quality.get("tracked") or 0)
            used = int(quality.get("used") or 0)
            low_cno = int(quality.get("low_cno") or 0)
            low_elevation = int(quality.get("low_elevation") or 0)
        except (TypeError, ValueError):
            return
        avg_cno = quality.get("avg_cno_dbhz")
        try:
            avg_cno_value = float(avg_cno) if avg_cno is not None else None
        except (TypeError, ValueError):
            avg_cno_value = None
        constellations = quality.get("constellations")
        constellation_items: list[tuple[str, int]] = []
        if isinstance(constellations, dict):
            for name, count in constellations.items():
                try:
                    constellation_items.append((str(name), int(count)))
                except (TypeError, ValueError):
                    continue
        constellation_text = " / ".join(f"{name} {count}" for name, count in sorted(constellation_items)) or "--"
        satellite_level = "good" if used >= 12 else "warn" if used >= 6 or tracked >= 12 else "bad"
        signal_level = (
            "idle"
            if avg_cno_value is None
            else "good"
            if avg_cno_value >= 35.0 and low_cno == 0
            else "warn"
            if avg_cno_value >= 30.0
            else "bad"
        )
        elevation_level = "good" if low_elevation == 0 else "warn" if low_elevation <= max(1, tracked // 4) else "bad"
        constellation_level = (
            "good" if len(constellation_items) >= 3 else "warn" if len(constellation_items) >= 2 else "bad"
        )
        avg_cno_text = "--" if avg_cno_value is None else f"{avg_cno_value:.1f} dBHz"
        self.panel._set_health_label(
            self.panel.gnss_base_quality_labels["satellites"],
            f"used={used} tracked={tracked}",
            satellite_level,
        )
        self.panel._set_health_label(
            self.panel.gnss_base_quality_labels["signal"],
            f"avg C/N0={avg_cno_text} | low<{GNSS_BASE_LOW_CNO_DBHZ}={low_cno}",
            signal_level,
        )
        self.panel._set_health_label(
            self.panel.gnss_base_quality_labels["elevation"],
            f"low<{GNSS_BASE_LOW_ELEVATION_DEG}deg={low_elevation}",
            elevation_level,
        )
        self.panel._set_health_label(
            self.panel.gnss_base_quality_labels["constellations"],
            constellation_text,
            constellation_level,
        )

    def _drone_rtcm_source(self) -> str:
        return normalize_drone_rtcm_source(getattr(self.panel, "drone_telemetry_rtcm_source", DRONE_RTCM_SOURCE_BASE))

    def _drone_rtcm_route_enabled(self) -> bool:
        config = getattr(self.panel, "gnss_active_operation_config", None)
        if config is None:
            builder = getattr(self.panel, "_build_gnss_operation_config", None)
            if callable(builder):
                try:
                    config = builder()
                except (AttributeError, TypeError, ValueError):
                    config = None
        routing = getattr(config, "routing", None)
        return bool(getattr(routing, "drone_enabled", False))

    def _drone_rtcm_target_sides(self, source: str) -> tuple[str, ...]:
        if normalize_drone_rtcm_source(source) != self._drone_rtcm_source():
            return ()
        if not self._drone_rtcm_route_enabled():
            return ()
        if not bool(getattr(self.panel, "drone_telemetry_rtcm_forward", False)):
            return ()
        subscriber = getattr(self.panel, "mavlink_pixhawk", None)
        connections = getattr(subscriber, "connections", {}) if subscriber is not None else {}
        if isinstance(connections, dict) and "drone" in connections:
            return ("drone",)
        return ()

    def _gnss_base_rtcm_target_sides(
        self, *, source: str = DRONE_RTCM_SOURCE_BASE, include_rover_targets: bool = True
    ) -> tuple[str, ...]:
        config = self.panel.gnss_active_operation_config
        drone_config = config
        if drone_config is None:
            builder = getattr(self.panel, "_build_gnss_operation_config", None)
            if callable(builder):
                try:
                    drone_config = builder()
                except (AttributeError, TypeError, ValueError):
                    pass
        subscriber = getattr(self.panel, "mavlink_pixhawk", None)
        connections = getattr(subscriber, "connections", {})
        selected = (
            str(self.panel.gnss_base_rtcm_target_combo.currentData() or "disabled")
            if config is None and include_rover_targets
            else "disabled"
        )
        # Preserve editable rover-target fallback independently of the configured
        # drone route, which has always required a real enabled endpoint.
        rover = GnssOperationRuntime.target_sides(
            config,
            selected_target=selected,
            source=source,
            include_rover_targets=include_rover_targets,
            drone_source="",
            drone_forward=False,
            drone_connected=False,
        )
        drone = GnssOperationRuntime.target_sides(
            drone_config,
            selected_target="disabled",
            source=normalize_drone_rtcm_source(source),
            include_rover_targets=False,
            drone_source=self._drone_rtcm_source(),
            drone_forward=bool(getattr(self.panel, "drone_telemetry_rtcm_forward", False)),
            drone_connected=isinstance(connections, dict) and "drone" in connections,
        )
        return tuple(dict.fromkeys((*rover, *drone)))

    def _record_gnss_base_rtcm_frame(self, payload: bytes) -> None:
        return self._correction_runtime()._record_gnss_base_rtcm_frame(payload)

    def _append_rtcm_package_evidence(self, payload: bytes, *, source: str, record_kind: str) -> None:
        self._evidence_runtime().append_legacy(payload, source=source, record_kind=record_kind)

    def _record_gnss_base_rtcm_injection(
        self,
        side: str,
        *,
        payload_bytes: int,
        packet_count: int = 0,
        failed: bool = False,
        blocked_stall: bool = False,
    ) -> None:
        return self._correction_runtime()._record_gnss_base_rtcm_injection(
            side, payload_bytes=payload_bytes, packet_count=packet_count, failed=failed, blocked_stall=blocked_stall
        )

    def _handle_gnss_base_rtcm_frame(
        self,
        frame: object,
        *,
        source: str = DRONE_RTCM_SOURCE_BASE,
        include_rover_targets: bool = True,
    ) -> None:
        if not isinstance(frame, (bytes, bytearray)):
            return
        payload = bytes(frame)
        self._append_rtcm_package_evidence(payload, source=source, record_kind="rtcm_frame")
        ui_due = self._rtcm_ui_update_due()
        self.panel._record_gnss_base_rtcm_frame(payload)
        if (
            source == DRONE_RTCM_SOURCE_BASE
            and self.panel.gnss_operation_manager.state == GnssOperationState.BASE_RECONFIGURING
            and parse_rtcm3_message_type(payload) in {1005, 1006}
        ):
            self.panel.gnss_fixed_base_reconfigure_rtcm = True
        target_sides = self.panel._gnss_base_rtcm_target_sides(
            source=source,
            include_rover_targets=include_rover_targets,
        )
        if not target_sides:
            if ui_due:
                self.panel._set_health_label(self.panel.gnss_base_status_labels["rtcm_injection"], "비활성", "idle")
                self.panel._refresh_gnss_base_rtcm_stats()
                self.panel._refresh_gnss_operation_readiness()
            return
        if self.panel.mavlink_pixhawk is None:
            for side in target_sides:
                self.panel._record_gnss_base_rtcm_injection(side, payload_bytes=len(payload), failed=True)
            if ui_due:
                self.panel._set_health_label(
                    self.panel.gnss_base_status_labels["rtcm_injection"],
                    f"RTCM {len(payload)} bytes 수신 | MAVLink rover 미연결",
                    "warn",
                )
                self.panel._refresh_gnss_base_rtcm_stats()
                self.panel._refresh_gnss_operation_readiness()
            return
        worker = self.panel._ensure_pixhawk_rtcm_worker()
        if worker is None:
            for side in target_sides:
                self.panel._record_gnss_base_rtcm_injection(side, payload_bytes=len(payload), failed=True)
            if ui_due:
                self.panel._set_health_label(
                    self.panel.gnss_base_status_labels["rtcm_injection"],
                    f"RTCM 주입 대기 실패: MAVLink worker 없음 ({len(payload)} bytes)",
                    "bad",
                )
                self.panel._refresh_gnss_base_rtcm_stats()
                self.panel._refresh_gnss_operation_readiness()
            return
        # Carry the receive time so the worker drops a frame that waited
        # longer than MAX_INJECTION_AGE_S instead of pushing stale corrections
        # into the receiver in a late burst.
        worker.request_envelope.emit(
            {
                "targets": tuple(target_sides),
                "payload": payload,
                "source": str(source),
                "receive_monotonic_ns": time.monotonic_ns(),
            }
        )
        if ui_due:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["rtcm_injection"],
                f"RTCM {len(payload)} bytes 대기 중 -> "
                f"{', '.join(self.panel._px4_mag_camera_label(side) for side in target_sides)}",
                "warn",
            )

    def _ensure_pixhawk_rtcm_worker(self) -> PixhawkRtcmInjectionWorker | None:
        return self._correction_runtime()._ensure_pixhawk_rtcm_worker()

    def _restart_pixhawk_rtcm_worker_after_retire(
        self,
        old_thread: object,
        desired_subscriber: object,
    ) -> None:
        return self._correction_runtime()._restart_pixhawk_rtcm_worker_after_retire(old_thread, desired_subscriber)

    def _retire_pixhawk_rtcm_worker(
        self,
        *,
        expected_subscriber: object | None = None,
        wait: bool,
    ) -> bool:
        return self._correction_runtime()._retire_pixhawk_rtcm_worker(
            expected_subscriber=expected_subscriber, wait=wait
        )

    def _note_rtcm_frame_drop(self, dropped: int) -> None:
        """Log dropped RTCM frames at most every 5 s, carrying the pending count."""

        now_s = time.monotonic()
        pending = int(getattr(self.panel, "_rtcm_drop_pending_count", 0) or 0) + int(dropped)
        last_s = getattr(self.panel, "_rtcm_drop_last_log_s", None)
        if last_s is not None and now_s - float(last_s) < 5.0:
            self.panel._rtcm_drop_pending_count = pending
            return
        self.panel._rtcm_drop_pending_count = 0
        self.panel._rtcm_drop_last_log_s = now_s
        self.panel._log(f"[GNSS] RTCM 오래된 대기 프레임 {pending}개를 폐기했습니다.")

    def _apply_pixhawk_rtcm_result(self, result: object) -> None:
        data = dict(result or {})
        payload_bytes = int(data.get("payload_bytes") or 0)
        sent = tuple(data.get("sent") or ())
        failed = tuple(data.get("failed") or ())
        blocked = tuple(data.get("blocked_targets") or ())
        dropped = int(data.get("dropped") or 0)
        sent_labels: list[str] = []
        failed_labels: list[str] = []
        blocked_labels: list[str] = []
        for side, packet_count in sent:
            side = str(side)
            self.panel._record_gnss_base_rtcm_injection(
                side,
                payload_bytes=payload_bytes,
                packet_count=int(packet_count or 0),
            )
            sent_labels.append(self.panel._px4_mag_camera_label(side))
        for side, error in failed:
            side = str(side)
            self.panel._record_gnss_base_rtcm_injection(side, payload_bytes=payload_bytes, failed=True)
            failed_labels.append(f"{self.panel._px4_mag_camera_label(side)}:{error}")
        for item in blocked:
            if not isinstance(item, dict):
                continue
            side = str(item.get("side") or "")
            if not side:
                continue
            if str(item.get("category") or "") == "stall":
                self.panel._record_gnss_base_rtcm_injection(
                    side,
                    payload_bytes=payload_bytes,
                    blocked_stall=True,
                )
            blocked_labels.append(
                f"{self.panel._px4_mag_camera_label(side)}:{item.get('reason') or 'liveness blocked'}"
            )
        if dropped:
            self._note_rtcm_frame_drop(dropped)
        if failed_labels or blocked_labels:
            # The health label below is the operator surface; run.log needs the
            # same reason so a side that silently stops receiving corrections
            # can be diagnosed after the fact.
            now_s = time.monotonic()
            last_s = getattr(self.panel, "_rtcm_problem_last_log_s", None)
            if last_s is None or now_s - float(last_s) >= 5.0:
                self.panel._rtcm_problem_last_log_s = now_s
                parts = []
                if failed_labels:
                    parts.append("실패 " + ", ".join(failed_labels))
                if blocked_labels:
                    parts.append("차단 " + ", ".join(blocked_labels))
                self.panel._log(f"[GNSS] RTCM 주입 제한 ({payload_bytes} bytes): " + " | ".join(parts))
        # Receive/pending updates and completed injection updates use separate
        # clocks.  Otherwise every high-rate receive event consumes the one
        # slot that should publish delivery stats and advance readiness.
        if not failed_labels and not blocked_labels and not self._rtcm_result_ui_update_due():
            return
        if failed_labels or blocked_labels:
            self._last_rtcm_result_ui_update_s = time.monotonic()
        if sent_labels and not failed_labels and not blocked_labels:
            level = "good"
            text = f"RTCM {payload_bytes} bytes -> {', '.join(sent_labels)}"
        elif sent_labels and blocked_labels:
            level = "warn"
            text = f"RTCM 일부 주입: {', '.join(sent_labels)} | 차단 {', '.join(blocked_labels)}"
        elif sent_labels:
            level = "warn"
            text = f"RTCM 일부 주입: {', '.join(sent_labels)} | 실패 {', '.join(failed_labels)}"
        elif failed_labels:
            level = "bad"
            text = f"RTCM 주입 실패: {', '.join(failed_labels)}"
        elif blocked_labels:
            level = "bad"
            text = f"RTCM stalled target 차단: {', '.join(blocked_labels)}"
        else:
            return
        self.panel._set_health_label(self.panel.gnss_base_status_labels["rtcm_injection"], text, level)
        self.panel._refresh_gnss_base_rtcm_stats()
        self.panel._refresh_gnss_operation_readiness()

    def _clear_pixhawk_rtcm_worker(
        self,
        thread: QtCore.QThread | None = None,
        worker: PixhawkRtcmInjectionWorker | None = None,
    ) -> None:
        return self._correction_runtime()._clear_pixhawk_rtcm_worker(thread, worker)

    def _handle_gnss_base_command_sent(self, label: str, byte_count: int) -> None:
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["connection"],
            f"connected | sent {label} ({byte_count} bytes)",
            "good",
        )
        if label == "CFG-TMODE3 survey-in start":
            self.panel._gnss_survey_command_sent_generation = int(
                getattr(self.panel, "_gnss_survey_generation", 0) or 0
            )
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["survey_in"],
                "survey-in 시작 명령 전송 완료 | NAV-SVIN 상태 수신 대기",
                "warn",
            )
        elif label == "CFG-TMODE3 disabled":
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["survey_in"],
                "survey-in 중지 명령 전송 완료 | NAV-SVIN 상태 수신 대기",
                "warn",
            )

    def _handle_gnss_base_survey_in(self, state: object) -> None:
        if not isinstance(state, dict):
            return
        # Keep the producer payload intact for the Workstation Base GNSS
        # evidence view. NAV-SVIN mean_x/y/z remain receiver ECEF metres;
        # the presentation layer must not relabel them as NED or FRD.
        active = bool(state.get("active"))
        valid = bool(state.get("valid"))
        duration_s = int(state.get("duration_s") or 0)
        accuracy_m = float(state.get("mean_accuracy_m") or 0.0)
        observations = int(state.get("observations") or 0)
        mean_x_m = float(state.get("mean_x_m") or 0.0)
        mean_y_m = float(state.get("mean_y_m") or 0.0)
        mean_z_m = float(state.get("mean_z_m") or 0.0)
        status = "valid" if valid else "active" if active else "not running"
        level = "good" if valid else "warn" if active else "idle"
        text = (
            f"{status} | duration={duration_s}s | acc={accuracy_m:.4f}m | obs={observations} | "
            f"ECEF={mean_x_m:.3f}/{mean_y_m:.3f}/{mean_z_m:.3f}m"
        )
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], text, level)
        self.panel._update_gnss_base_survey_visualization(
            active=active,
            valid=valid,
            duration_s=duration_s,
            accuracy_m=accuracy_m,
            observations=observations,
        )
        if self._base_runtime().observe_survey(state):
            self.panel._refresh_gnss_operation_readiness()

    def _update_gnss_base_survey_visualization(
        self,
        *,
        active: bool,
        valid: bool,
        duration_s: int,
        accuracy_m: float,
        observations: int,
    ) -> None:
        if not hasattr(self.panel, "gnss_base_survey_progress_bar"):
            return
        target_duration_s = max(1, int(self.panel.gnss_base_survey_duration_spin.value()))
        target_accuracy_m = max(0.001, float(self.panel.gnss_base_survey_accuracy_spin.value()))
        progress = min(100, int(round(duration_s * 100.0 / target_duration_s)))
        self.panel.gnss_base_survey_progress_bar.setValue(progress)
        state_text = "valid" if valid else "active" if active else "not running"
        accuracy_delta_m = accuracy_m - target_accuracy_m
        time_text = "time ok" if progress >= 100 else f"time {progress}%"
        accuracy_text = "accuracy ok" if valid or accuracy_delta_m <= 0.0 else "accuracy pending"
        if not active and not valid:
            accuracy_text = "accuracy --"
        self.panel.gnss_base_survey_progress_bar.setFormat(f"{state_text} | {time_text} | {accuracy_text}")
        level = "good" if valid or accuracy_delta_m <= 0.0 else "warn" if active else "idle"
        self.panel._set_health_label(
            self.panel.gnss_base_survey_accuracy_label,
            f"target={target_accuracy_m:.4f}m | current={accuracy_m:.4f}m | Δ={accuracy_delta_m:+.4f}m | obs={observations}",
            level,
        )
        now_s = time.monotonic()
        self.panel.gnss_base_survey_samples.append((now_s, duration_s, accuracy_m, active, valid))
        self.panel.gnss_base_survey_accuracy_plot.add_point(TelemetryHistoryPoint(now_s, "accuracy", accuracy_m))
        self.panel.gnss_base_survey_accuracy_plot.add_point(TelemetryHistoryPoint(now_s, "target", target_accuracy_m))
        self.panel.gnss_base_survey_observation_plot.add_point(
            TelemetryHistoryPoint(now_s, "observations", float(observations))
        )
        self.panel._update_gnss_base_survey_convergence(
            active=active,
            valid=valid,
            duration_s=duration_s,
            accuracy_m=accuracy_m,
            target_accuracy_m=target_accuracy_m,
            target_duration_s=target_duration_s,
            now_s=now_s,
        )

    def _format_eta_seconds(self, seconds: float | None) -> str:
        if seconds is None or not isfinite(float(seconds)):
            return "--"
        seconds = max(0.0, float(seconds))
        if seconds < 60.0:
            return f"~{seconds:.0f}s"
        return f"~{seconds / 60.0:.1f}m"

    def _update_gnss_base_survey_convergence(
        self,
        *,
        active: bool,
        valid: bool,
        duration_s: int,
        accuracy_m: float,
        target_accuracy_m: float,
        target_duration_s: int,
        now_s: float,
    ) -> None:
        if not hasattr(self.panel, "gnss_base_survey_convergence_label"):
            return
        if valid:
            self.panel._set_health_label(
                self.panel.gnss_base_survey_convergence_label,
                "수렴: complete | ETA: done | 최근 개선: --",
                "good",
            )
            return
        if not active:
            self.panel._set_health_label(
                self.panel.gnss_base_survey_convergence_label,
                "수렴: not running | ETA: -- | 최근 개선: --",
                "idle",
            )
            return

        trend_samples = [
            sample
            for sample in self.panel.gnss_base_survey_samples
            if 0.0 <= now_s - sample[0] <= GNSS_BASE_SURVEY_TREND_WINDOW_S
        ]
        if len(trend_samples) < 2:
            self.panel._set_health_label(
                self.panel.gnss_base_survey_convergence_label,
                "수렴: collecting trend | ETA: -- | 최근 개선: --",
                "warn",
            )
            return

        first = trend_samples[0]
        elapsed_s = max(0.0, now_s - first[0])
        improvement_m = max(0.0, float(first[2]) - float(accuracy_m))
        time_remaining_s = max(0.0, float(target_duration_s) - float(duration_s))
        accuracy_remaining_m = max(0.0, float(accuracy_m) - float(target_accuracy_m))
        eta_s: float | None
        if accuracy_remaining_m <= 0.0:
            eta_s = time_remaining_s
        elif elapsed_s > 0.0 and improvement_m > 0.0:
            eta_s = max(time_remaining_s, accuracy_remaining_m / (improvement_m / elapsed_s))
        else:
            eta_s = None
        if elapsed_s >= GNSS_BASE_SURVEY_TREND_WINDOW_S and improvement_m < GNSS_BASE_SURVEY_STAGNANT_DELTA_M:
            convergence = f"stagnant {elapsed_s:.0f}s"
            level = "bad"
        elif improvement_m >= GNSS_BASE_SURVEY_IMPROVING_DELTA_M:
            convergence = "improving"
            level = "warn"
        elif improvement_m > 0.0:
            convergence = "slow"
            level = "warn"
        else:
            convergence = "waiting for improvement"
            level = "warn"
        self.panel._set_health_label(
            self.panel.gnss_base_survey_convergence_label,
            f"수렴: {convergence} | ETA: {self.panel._format_eta_seconds(eta_s)} | 최근 개선: -{improvement_m:.4f}m/{elapsed_s:.0f}s",
            level,
        )

    def _runtime_state(self, name, factory, fields):
        application = getattr(self.panel, "application", None)
        features = getattr(application, "features", {})
        feature = features.get("gnss") if isinstance(features, dict) else None
        state = getattr(getattr(feature, "state", None), name, None)
        if state is not None:
            return state
        # Lightweight historical adapters have no feature composition. Mirror
        # their finite compatibility fields at this outer edge only; real owners
        # always use the authoritative feature state above.
        states = self.__dict__.setdefault("_legacy_runtime_states", {})
        if name not in states:
            panel = self.panel

            class LegacyState(factory):
                def __setattr__(value, key, item):
                    super().__setattr__(key, item)
                    if key in fields:
                        setattr(panel, key, item)

            initial = {key: getattr(panel, key) for key in fields if hasattr(panel, key)}
            state = LegacyState()
            for key, value in initial.items():
                setattr(state, key, value)
            states[name] = state
        state = states[name]
        for key in fields:
            if hasattr(self.panel, key):
                setattr(state, key, getattr(self.panel, key))
        return state

    def _gnss_health(self, key, text, level):
        labels = getattr(self.panel, "gnss_base_status_labels", {})
        if key in labels:
            self.panel._set_health_label(labels[key], text, level)

    def _gnss_plot(self, name, point):
        plot = getattr(self.panel, name, None)
        if plot is not None:
            plot.add_point(point)

    def _operation_runtime(self):
        state = self._runtime_state("operation", GnssOperationRuntimeState, OPERATION_FIELDS)
        runtime = getattr(self, "_operation_owner", None)
        if runtime is None:
            runtime = GnssOperationRuntime(
                state,
                transitioned=self._present_gnss_transition,
                health=self._gnss_health,
                evaluate=lambda config, inputs: evaluate_gnss_readiness(config, inputs),
            )
            # Historical lightweight adapters provide their own state transition
            # function instead of a GnssOperationManager. Production composition
            # always uses the owner's real manager transition above.
            legacy_transition = getattr(self.panel, "_gnss_transition", None)
            if not callable(getattr(state.gnss_operation_manager, "transition", None)) and callable(legacy_transition):
                runtime.transition = legacy_transition
            self._operation_owner = runtime
        return runtime

    def _base_runtime(self):
        state = self._runtime_state("base", BaseReceiverState, BASE_FIELDS)
        self._operation_runtime()
        runtime = getattr(self, "_base_owner", None)
        if runtime is None:
            runtime = BaseReceiverRuntime(
                self.panel if isinstance(self.panel, QtCore.QObject) else None,
                state,
                self._operation_owner,
                config=lambda: self.panel._gnss_active_operation_config(),
                network_connected=lambda: self.panel.gnss_network_connected,
                effects=BaseReceiverEffects(
                    self._gnss_health,
                    self._gnss_plot,
                    lambda sample: self.panel._record_gnss_base_position_row(sample),
                    self._reset_gnss_base_rtcm_stats,
                ),
                serial_factory=lambda **kwargs: GnssBaseSerialWorker(**kwargs),
                detect_factory=lambda **kwargs: GnssBaseSerialDetectWorker(**kwargs),
                serial_events={
                    "connected": self._handle_gnss_base_serial_connected,
                    "stats_changed": self._handle_gnss_base_serial_stats,
                    "survey_in_changed": self._handle_gnss_base_survey_in,
                    "satellite_count_changed": self._handle_gnss_base_satellite_count,
                    "satellite_quality_changed": self._handle_gnss_base_satellite_quality,
                    "position_changed": self._handle_gnss_base_position,
                    "ubx_message_received": self._handle_gnss_base_ubx_message,
                    "command_sent": self._handle_gnss_base_command_sent,
                    "failed": self._handle_gnss_base_serial_failed,
                    "disconnected": self._handle_gnss_base_serial_disconnected,
                },
                detect_events={
                    "detected": self._handle_gnss_base_candidates_detected,
                    "failed": self._handle_gnss_base_candidate_detect_failed,
                },
                retired=self._base_resource_retired,
            )
            self._base_owner = runtime
        return runtime

    def _prepare_base_rtcm_route(self, worker):
        pipeline = self._ensure_rtcm_pipeline()
        self._wire_rtcm_base_source(worker, pipeline)
        self._wire_rtcm_pipeline_base_worker(worker)
        if self._rtcm_route_config().base_targets:
            self._ensure_pixhawk_rtcm_worker()
        self._sync_rtcm_pipeline_config(reset=True)
        self._sync_rtcm_evidence_context()

    def _base_resource_retired(self, kind):
        if kind == "detect":
            button = getattr(self.panel, "gnss_base_discover_button", None)
            if button is not None:
                button.setEnabled(True)
            self.panel._refresh_gnss_base_candidate_actions()
            return
        self._wire_rtcm_pipeline_base_worker(None)
        self._sync_rtcm_pipeline_config()
        for name, enabled in (
            ("gnss_base_connect_button", True),
            ("gnss_base_disconnect_button", False),
            ("gnss_base_survey_start_button", False),
            ("gnss_base_survey_stop_button", False),
        ):
            button = getattr(self.panel, name, None)
            if button is not None:
                button.setEnabled(enabled)

    def _reset_rtcm_observed_counters(self):
        self._last_pipeline_drop_count = 0
        self._last_pipeline_blocked_stall_count = 0

    def _rtcm_injection_retired(self):
        for name in (
            "_refresh_gnss_base_rtcm_stats",
            "_maybe_activate_gnss_fixed_base",
            "_refresh_gnss_operation_readiness",
        ):
            callback = getattr(self.panel, name, None)
            if callable(callback):
                callback()

    def _rtcm_evidence_drained(self, payload):
        callback = getattr(self.panel, "_handle_measurement_rtcm_evidence_drained", None)
        if callable(callback):
            callback(payload)

    def _correction_runtime(self):
        state = self._runtime_state("corrections", CorrectionRuntimeState, CORRECTION_FIELDS)
        runtime = getattr(self, "_correction_owner", None)
        if runtime is None:
            runtime = CorrectionRuntime(
                self.panel if isinstance(self.panel, QtCore.QObject) else None,
                state,
                route=self._rtcm_route_config,
                evidence=self._evidence_runtime(),
                base_worker=lambda: getattr(self.panel, "gnss_base_serial_worker", None),
                subscriber=lambda: getattr(self.panel, "mavlink_pixhawk", None),
                pipeline_factory=lambda **kwargs: RtcmDataPlaneWorker(**kwargs),
                injection_factory=lambda subscriber: PixhawkRtcmInjectionWorker(subscriber),
                thread_factory=lambda parent: QtCore.QThread(parent),
                events=CorrectionEvents(
                    self._apply_rtcm_pipeline_snapshot,
                    self._handle_rtcm_pipeline_evidence_started,
                    self._rtcm_evidence_drained,
                    self._handle_rtcm_pipeline_integrity_failed,
                    self._rtcm_injection_retired,
                    self._reset_rtcm_observed_counters,
                ),
            )
            self._correction_owner = runtime
        return runtime

    def _activate_rtcm_evidence(self, metadata):
        activator = getattr(self.panel, "activate_measurement_package_source", None)
        if callable(activator):
            activator("gnss.rtcm", required_roles=("corrections/rtcm/raw",), metadata=metadata)

    def _evidence_runtime(self):
        self._runtime_state("evidence", GnssEvidenceState, EVIDENCE_FIELDS)
        runtime = getattr(self, "_evidence_owner", None)
        if runtime is None:
            runtime = RtcmEvidenceRuntime(
                self._runtime_state("evidence", GnssEvidenceState, EVIDENCE_FIELDS),
                capture=self._capture_rtcm_evidence,
                activated=self._activate_rtcm_evidence,
            )
            self._evidence_owner = runtime
        return runtime

    def _capture_rtcm_evidence(self):
        hierarchy = bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True))
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not hierarchy:
            active_reader = getattr(self.panel, "_session_recording_active", None)
            active = bool(active_reader()) if callable(active_reader) else False
            root = None
            if callable(resolver):
                try:
                    root = Path(resolver("measurement-data"))
                except (OSError, TypeError, ValueError):
                    pass
            context_reader = getattr(self.panel, "measurement_storage_context", None)
            measurement = context_reader() if callable(context_reader) else {}
            if measurement.get("root"):
                root = Path(str(measurement["root"]))
            return EvidenceCapture(
                False,
                active=active,
                root=root,
                measurement_id=str(measurement.get("measurement_id") or "measurement-data-live"),
                file_stamp=str(measurement.get("measurement_file_stamp") or ""),
            )
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        root = None
        if session_id and callable(resolver):
            try:
                root = Path(resolver(session_id))
            except (OSError, TypeError, ValueError):
                pass
        return EvidenceCapture(
            True,
            root=root,
            state=str(getattr(self.panel, "_measurement_package_state", "IDLE") or "IDLE"),
            session_id=session_id,
            measurement_id=str(getattr(self.panel, "_measurement_id", "") or ""),
            revision=max(0, int(getattr(self.panel, "_measurement_package_revision", 0) or 0)),
            drain_requested_token=str(getattr(self.panel, "_measurement_package_rtcm_drain_requested_token", "") or ""),
        )
