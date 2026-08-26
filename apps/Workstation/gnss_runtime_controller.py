"""Workstation GNSS base, NTRIP, survey-in, and RTCM controller.

WGS84 coordinates, UBX fields, RTCM payloads, and explicitly named NED
telemetry retain their producer contracts. No camera/FRD alias conversion is
performed in this UI orchestration module.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from math import isfinite
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from Workstation.gnss_runtime_workers import (
    GnssBaseSerialDetectWorker,
    GnssBaseSerialWorker,
    NtripNetworkWorker,
    RtcmDataPlaneWorker,
)
from Workstation.workstation_core import ntrip_credentials
from Workstation.drone_telemetry_controller import (
    DRONE_RTCM_SOURCE_BASE,
    DRONE_RTCM_SOURCE_NTRIP,
    normalize_drone_rtcm_source,
)
from Workstation.device_runtime_workers import PixhawkRtcmInjectionWorker
from Workstation.telemetry_plots import TelemetryHistoryPoint
from Workstation.workstation_core.fixed_base_initializer import FixedBaseInitializer
from Workstation.workstation_core.gnss_network import format_nmea_gga, gga_from_coordinate_sample, gga_from_position
from Workstation.workstation_core.gnss_operation import (
    CorrectionSourceIdentity,
    evaluate_gnss_readiness,
    GgaPolicy,
    GnssOperationConfig,
    GnssOperationMode,
    GnssOperationState,
    GnssReadinessInputs,
    GnssReadinessSnapshot,
    GnssStatusGroup,
    HeightType,
    RoverCorrectionStatus,
    RoverReadinessInput,
    STATE_STATUS_GROUP,
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
    parse_ubx_ack,
    parse_ubx_cfg_tmode3,
    ubx_cfg_tmode3_disabled_frame,
    ubx_cfg_tmode3_fixed_frame,
    ubx_cfg_tmode3_poll_frame,
    ubx_cfg_tmode3_survey_in_frame,
    ubx_nav_svin_poll_frame,
)
from Workstation.workstation_core.rtcm_pipeline import (
    RTCM_PIPELINE_SNAPSHOT_SCHEMA,
    RtcmEvidenceContext,
    RtcmEvidenceRecord,
    RtcmRouteConfig,
    append_rtcm_evidence,
    load_rtcm_evidence_high_water,
)

class NtripCredentialError(RuntimeError):
    """Raised by ``_resolve_ntrip_credentials`` with a precise, user-facing reason.

    The message is safe to show as-is: it names which input is missing, never
    a credential value.
    """


@dataclass(frozen=True)
class NtripCredentialResolution:
    username: str
    password: str
    # "session" (typed into the UI this run), "keyring" (OS secure store), or
    # "legacy_env" (username_env/password_env indirection).
    source: str


def _ntrip_credential_account_key(
    caster_host: object,
    mountpoint: object,
    username: object,
) -> tuple[str, str, str]:
    """Return the non-secret identity that owns one session credential."""

    return (
        str(caster_host or "").strip().casefold(),
        str(mountpoint or "").strip().lstrip("/"),
        str(username or "").strip(),
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
    "_update_gnss_base_survey_convergence"
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
        if (
            hasattr(package_signal, "connect")
            and not bool(getattr(panel, "_rtcm_package_signal_connected", False))
        ):
            package_signal.connect(self._handle_measurement_package_changed)
            panel._rtcm_package_signal_connected = True

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
        return RtcmRouteConfig(
            ntrip_targets=ntrip_targets,
            base_targets=base_targets,
            forward_ntrip_to_base=assisted,
            base_serial_available=bool(
                getattr(self.panel, "gnss_base_serial_connected", False)
                and getattr(self.panel, "gnss_base_serial_worker", None) is not None
            ),
        )

    def _rtcm_evidence_context(self) -> RtcmEvidenceContext:
        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            active_reader = getattr(self.panel, "_session_recording_active", None)
            active = bool(active_reader()) if callable(active_reader) else False
            resolver = getattr(self.panel, "_recording_session_dir", None)
            root = None
            if callable(resolver):
                try:
                    root = Path(resolver("measurement-data"))
                except (OSError, TypeError, ValueError):
                    root = None
            return RtcmEvidenceContext(
                # RtcmEvidenceContext retains its legacy state vocabulary as
                # an internal writer contract only; there is no persisted
                # Measurement Session. COLLECTING makes the fixed RTCM files
                # active for Manual record and Measurement intervals.
                state="COLLECTING" if active else "IDLE",
                session_id="measurement-data" if active and root is not None else "",
                session_root=root if active else None,
                context_token=(
                    "measurement-data-live" if active and root is not None else ""
                ),
            )
        state = str(getattr(self.panel, "_measurement_package_state", "IDLE") or "IDLE")
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        measurement_id = str(getattr(self.panel, "_measurement_id", "") or "")
        revision_number = max(
            0,
            int(getattr(self.panel, "_measurement_package_revision", 0) or 0),
        )
        resolver = getattr(self.panel, "_recording_session_dir", None)
        root = None
        if session_id and callable(resolver):
            try:
                root = Path(resolver(session_id))
            except (OSError, TypeError, ValueError):
                root = None
        context_token = (
            f"{session_id}:{measurement_id or 'session'}:r{revision_number:04d}"
            if session_id and root is not None
            else ""
        )
        if context_token and str(
            getattr(
                self.panel,
                "_measurement_package_rtcm_drain_requested_token",
                "",
            )
            or ""
        ) == context_token:
            # A VERIFYING/RECOVERABLE package snapshot published after the
            # barrier ACK must not reactivate the same evidence owner.
            state = "IDLE"
        return RtcmEvidenceContext(
            state=state,
            session_id=session_id,
            session_root=root,
            context_token=context_token,
        )

    def _ensure_rtcm_pipeline(self) -> RtcmDataPlaneWorker:
        existing = getattr(self.panel, "rtcm_pipeline_worker", None)
        thread = getattr(self.panel, "rtcm_pipeline_thread", None)
        if existing is not None and thread is not None and thread.isRunning():
            return existing
        if thread is not None and thread.isRunning():
            raise RuntimeError("RTCM pipeline thread exists without its worker")

        thread = QtCore.QThread(self.panel)
        worker = RtcmDataPlaneWorker(snapshot_interval_s=self.RTCM_UI_UPDATE_INTERVAL_S)
        worker.moveToThread(thread)
        _connect_gui_signal(worker.snapshot_ready, self.panel._apply_rtcm_pipeline_snapshot)
        _connect_gui_signal(worker.evidence_started, self.panel._handle_rtcm_pipeline_evidence_started)
        drain_handler = getattr(
            self.panel,
            "_handle_measurement_rtcm_evidence_drained",
            None,
        )
        if callable(drain_handler):
            _connect_gui_signal(worker.evidence_drained, drain_handler)
        _connect_gui_signal(worker.integrity_failed, self.panel._handle_rtcm_pipeline_integrity_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        _connect_gui_signal(
            thread.finished,
            lambda thread=thread, worker=worker: self._clear_rtcm_pipeline(thread, worker),
        )
        thread.finished.connect(thread.deleteLater)
        self.panel.rtcm_pipeline_thread = thread
        self.panel.rtcm_pipeline_worker = worker
        self.panel._rtcm_pipeline_base_worker = None
        self.panel._rtcm_pipeline_injection_worker = None
        thread.start()
        worker.configure_requested.emit(self._rtcm_route_config().as_dict())
        worker.evidence_context_requested.emit(self._rtcm_evidence_context().as_dict())
        self._wire_rtcm_pipeline_base_worker(
            getattr(self.panel, "gnss_base_serial_worker", None)
        )
        self._wire_rtcm_pipeline_injection_worker(
            getattr(self.panel, "pixhawk_rtcm_worker", None)
        )
        return worker

    def _wire_rtcm_pipeline_base_worker(self, base_worker: object | None) -> None:
        pipeline = getattr(self.panel, "rtcm_pipeline_worker", None)
        if pipeline is None:
            return
        current = getattr(self.panel, "_rtcm_pipeline_base_worker", None)
        if current is base_worker:
            return
        try:
            pipeline.base_write_requested.disconnect()
        except (TypeError, RuntimeError):
            pass
        self.panel._rtcm_pipeline_base_worker = base_worker
        if base_worker is not None:
            # GnssBaseSerialWorker.run() owns a blocking serial loop, so its
            # QThread event loop cannot service a queued slot. send_bytes()
            # only performs a thread-safe Queue.put and is therefore the one
            # deliberate direct cross-thread adapter on this boundary.
            pipeline.base_write_requested.connect(
                base_worker.send_bytes,
                QtCore.Qt.DirectConnection,
            )

    def _wire_rtcm_pipeline_injection_worker(self, injection_worker: object | None) -> None:
        pipeline = getattr(self.panel, "rtcm_pipeline_worker", None)
        if pipeline is None:
            return
        current = getattr(self.panel, "_rtcm_pipeline_injection_worker", None)
        if current is injection_worker:
            return
        try:
            pipeline.injection_requested.disconnect()
        except (TypeError, RuntimeError):
            pass
        if current is not None:
            try:
                current.injected.disconnect(pipeline.injection_result_received)
            except (TypeError, RuntimeError):
                pass
        self.panel._rtcm_pipeline_injection_worker = injection_worker
        if injection_worker is not None:
            # submit_envelope only touches the worker's locked bounded queue;
            # device I/O remains in _drain_one on the injection QThread.
            pipeline.injection_requested.connect(
                injection_worker.submit_envelope,
                QtCore.Qt.DirectConnection,
            )
            injection_worker.injected.connect(pipeline.injection_result_received)

    def _wire_rtcm_ntrip_source(
        self,
        source: NtripNetworkWorker,
        pipeline: RtcmDataPlaneWorker,
    ) -> None:
        source.stats_snapshot_changed.connect(pipeline.ntrip_stats_received)
        source.chunk_envelope_received.connect(
            pipeline.submit_ntrip_envelope,
            QtCore.Qt.DirectConnection,
        )

    def _wire_rtcm_base_source(
        self,
        source: GnssBaseSerialWorker,
        pipeline: RtcmDataPlaneWorker,
    ) -> None:
        source.rtcm_frame_envelope_received.connect(
            pipeline.submit_base_envelope,
            QtCore.Qt.DirectConnection,
        )

    def _sync_rtcm_pipeline_config(self, *, reset: bool = False) -> None:
        pipeline = getattr(self.panel, "rtcm_pipeline_worker", None)
        if pipeline is None:
            return
        self._wire_rtcm_pipeline_base_worker(
            getattr(self.panel, "gnss_base_serial_worker", None)
        )
        self._wire_rtcm_pipeline_injection_worker(
            getattr(self.panel, "pixhawk_rtcm_worker", None)
        )
        payload = self._rtcm_route_config().as_dict()
        if reset:
            self._last_pipeline_drop_count = 0
            self._last_pipeline_blocked_stall_count = 0
            pipeline.reset_requested.emit(payload)
        else:
            pipeline.configure_requested.emit(payload)

    def _sync_rtcm_evidence_context(self) -> None:
        pipeline = getattr(self.panel, "rtcm_pipeline_worker", None)
        if pipeline is not None:
            pipeline.evidence_context_requested.emit(
                self._rtcm_evidence_context().as_dict()
            )

    def _close_rtcm_evidence_context_and_drain(
        self,
        context_token: str,
    ) -> bool:
        pipeline = getattr(self.panel, "rtcm_pipeline_worker", None)
        if pipeline is None:
            return False
        current = self._rtcm_evidence_context()
        return bool(
            pipeline.close_evidence_context_and_drain(
                {
                    "context_token": str(context_token),
                    "session_id": current.session_id,
                    "session_root": (
                        None
                        if current.session_root is None
                        else str(current.session_root)
                    ),
                }
            )
        )

    def _apply_rtcm_pipeline_snapshot(self, snapshot: object) -> None:
        if not isinstance(snapshot, dict) or snapshot.get("schema") != RTCM_PIPELINE_SNAPSHOT_SCHEMA:
            return
        injection = snapshot.get("injection")
        message_types = snapshot.get("message_types")
        self.panel.gnss_base_rtcm_stats = {
            "start_s": float(snapshot.get("start_s") or time.monotonic()),
            "last_frame_s": snapshot.get("last_frame_s"),
            "frames": int(snapshot.get("frames") or 0),
            "bytes": int(snapshot.get("bytes") or 0),
            "max_frame_bytes": int(snapshot.get("max_frame_bytes") or 0),
            "message_types": dict(message_types) if isinstance(message_types, dict) else {},
            "injection": (
                {side: dict(values) for side, values in injection.items() if isinstance(values, dict)}
                if isinstance(injection, dict)
                else {}
            ),
            "transport": dict(snapshot.get("transport") or {}),
            "integrity": dict(snapshot.get("integrity") or {}),
            "ingress": dict(snapshot.get("ingress") or {}),
        }
        ntrip_stats = snapshot.get("ntrip_stats")
        if ntrip_stats is not None:
            self.panel.gnss_network_stats = ntrip_stats
            if (
                self.panel.gnss_network_connected
                and getattr(ntrip_stats, "last_chunk_monotonic_s", None) is not None
            ):
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
                f"[GNSS] RTCM 오래된 주입 대기 프레임 {delta}개 폐기 "
                f"(누계={dropped}, evidence 원본은 유지)"
            )
        blocked_stall = int(integrity.get("injection_blocked_stall") or 0)
        if blocked_stall > self._last_pipeline_blocked_stall_count:
            delta = blocked_stall - self._last_pipeline_blocked_stall_count
            self._last_pipeline_blocked_stall_count = blocked_stall
            self.panel._log(
                f"[GNSS] stalled MAVLink target RTCM 차단 {delta}회 "
                f"(누계={blocked_stall}, transport failure와 분리)"
            )

        latest_result = snapshot.get("latest_injection_result")
        if isinstance(latest_result, dict):
            sent = tuple(latest_result.get("sent") or ())
            failed = tuple(latest_result.get("failed") or ())
            blocked = tuple(latest_result.get("blocked_targets") or ())
            if failed:
                text = "RTCM 주입 실패: " + ", ".join(
                    f"{self.panel._px4_mag_camera_label(str(side))}:{error}"
                    for side, error in failed
                )
                level = "warn" if sent else "bad"
            elif blocked:
                blocked_labels = ", ".join(
                    self.panel._px4_mag_camera_label(str(item.get("side") or ""))
                    for item in blocked
                    if isinstance(item, dict)
                )
                sent_labels = ", ".join(
                    self.panel._px4_mag_camera_label(str(side))
                    for side, _packets in sent
                )
                text = (
                    f"RTCM 일부 주입: {sent_labels} | stalled 차단 {blocked_labels}"
                    if sent
                    else f"RTCM stalled target 차단: {blocked_labels}"
                )
                level = "warn" if sent else "bad"
            elif sent:
                text = (
                    f"RTCM {int(latest_result.get('payload_bytes') or 0)} bytes -> "
                    + ", ".join(
                        self.panel._px4_mag_camera_label(str(side))
                        for side, _packets in sent
                    )
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
        error = (
            "RTCM integrity failure: "
            f"sequence={int(values.get('sequence') or 0)}, {values.get('error') or 'unknown error'}"
        )
        package_state = str(
            getattr(self.panel, "_measurement_package_state", "IDLE") or "IDLE"
        ).upper()
        current_context = self._rtcm_evidence_context()
        failure_session_id = str(values.get("session_id") or "")
        failure_context_token = str(values.get("context_token") or "")
        belongs_to_current_context = (
            failure_session_id == current_context.session_id
            and (
                not failure_context_token
                or failure_context_token == current_context.context_token
            )
        )
        if bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", False)) and (
            belongs_to_current_context
            and package_state in {"COLLECTING", "POST_CAPTURE", "VERIFYING", "RECOVERABLE"}
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
        thread = getattr(self.panel, "rtcm_pipeline_thread", None)
        worker = getattr(self.panel, "rtcm_pipeline_worker", None)
        if thread is None or not thread.isRunning():
            return True
        if worker is not None:
            worker.stop_requested.emit()
        return False

    def _clear_rtcm_pipeline(
        self,
        thread: QtCore.QThread,
        worker: RtcmDataPlaneWorker,
    ) -> None:
        if getattr(self.panel, "rtcm_pipeline_thread", None) is thread:
            self.panel.rtcm_pipeline_thread = None
            self.panel.rtcm_pipeline_worker = None
            self.panel._rtcm_pipeline_base_worker = None
            self.panel._rtcm_pipeline_injection_worker = None

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
                    "gga_policy": str(self.panel.gnss_ntrip_gga_policy_combo.currentData() or GgaPolicy.BASE_GNSS.value),
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
        state = self.panel.gnss_operation_manager.state
        if state not in {GnssOperationState.DISCONNECTED, GnssOperationState.STOPPED, GnssOperationState.CONFIGURED}:
            self.panel.gnss_operation_manager.stop()
        self.panel.gnss_operation_manager.configure(config)

    def _gnss_status_label_key_for_state(self, state: GnssOperationState) -> str:
        group = STATE_STATUS_GROUP.get(state, GnssStatusGroup.READINESS)
        if group is GnssStatusGroup.NETWORK:
            return "network"
        if group is GnssStatusGroup.BASE_FIXED:
            return "base_fixed"
        return "operation"

    def _gnss_transition(self, state: GnssOperationState, reason: str) -> bool:
        try:
            self.panel.gnss_operation_manager.transition(state, reason)
        except (RuntimeError, ValueError) as exc:
            label_key = self.panel._gnss_status_label_key_for_state(state)
            self.panel._set_health_label(self.panel.gnss_base_status_labels[label_key], f"상태 전환 실패: {exc}", "bad")
            self.panel._log(f"[GNSS] 상태 전환 실패: {exc}")
            return False
        new_state = self.panel.gnss_operation_manager.state
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
                self.panel._gnss_active_operation_config().operation_mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE
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
        return True

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
            value_label = self.panel.gnss_base_status_labels.get(key) if hasattr(self.panel, "gnss_base_status_labels") else None
            title_label = (
                self.panel.gnss_base_status_title_labels.get(key) if hasattr(self.panel, "gnss_base_status_title_labels") else None
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
            self.panel.gnss_base_connect_button.setEnabled(serial_allowed and self.panel.gnss_base_serial_thread is None)
        if hasattr(self.panel, "gnss_base_discover_button"):
            self.panel.gnss_base_discover_button.setEnabled(serial_allowed and self.panel.gnss_base_detect_thread is None)
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
                "idle" if self.panel.gnss_operation_manager.state in {GnssOperationState.DISCONNECTED, GnssOperationState.STOPPED} else "warn",
            )

    def _gnss_source_identity(self, config: GnssOperationConfig) -> CorrectionSourceIdentity:
        if config.operation_mode == GnssOperationMode.LOCAL_SURVEY_IN:
            return CorrectionSourceIdentity(
                source_kind="local-serial-survey-in",
                base_device=config.local_base.device,
                site_label=config.site_label,
                site_address=config.site_address,
            )
        return CorrectionSourceIdentity(
            source_kind=("ntrip-fixed-base" if config.operation_mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE else "ntrip-direct"),
            caster_host=config.network.caster_host,
            caster_port=config.network.caster_port,
            mountpoint=config.network.mountpoint,
            gga_policy=config.network.gga_policy,
            base_device=config.local_base.device if config.operation_mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE else "",
            site_label=config.site_label,
            site_address=config.site_address,
        )

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
            self.panel._gnss_last_gga_skip_reason = ""
            self.panel._gnss_last_gga_skip_monotonic_s = None
        return sentence

    def _resolve_gga_sentence(self) -> str | None:
        """Return the current GGA sentence, or ``None`` with a logged reason.

        This is polled at connect time and roughly once per
        ``on_gga_interval_s`` while streaming (see ``NtripClient``), so a
        genuinely stuck ``None`` state logs at that same modest rate --
        frequent enough to see live, never a per-recv()-loop spam risk.
        """

        active_config = self.panel._gnss_active_operation_config()
        policy = active_config.network.gga_policy

        def log_skip(key: str, message: str) -> None:
            now_s = time.monotonic()
            self.panel._gnss_last_gga_skip_reason = message
            self.panel._gnss_last_gga_skip_monotonic_s = now_s
            throttle = getattr(self.panel, "_gnss_gga_skip_log_throttle", None)
            if not isinstance(throttle, dict):
                throttle = {}
                self.panel._gnss_gga_skip_log_throttle = throttle
            previous_s = float(throttle.get(key, float("-inf")))
            if now_s - previous_s < 30.0:
                return
            throttle[key] = now_s
            self.panel._log(message)

        if policy == GgaPolicy.BASE_GNSS:
            sample = getattr(self.panel, "latest_gnss_base_position", None)
            if sample is None:
                log_skip(
                    "base-missing",
                    "[GNSS][NTRIP] GGA skipped: no Base GNSS sample received yet "
                    "(policy=BASE_GNSS)",
                )
                return None
            age_s = max(0.0, time.monotonic() - sample.timestamp_s)
            if age_s > 15.0:
                log_skip(
                    "base-stale",
                    f"[GNSS][NTRIP] GGA skipped: Base GNSS sample is stale "
                    f"(age={age_s:.1f}s > 15.0s, policy=BASE_GNSS)",
                )
                return None
            return gga_from_coordinate_sample(sample)
        if policy == GgaPolicy.MANUAL:
            log_skip(
                "manual",
                "[GNSS][NTRIP] GGA skipped: GGA policy is MANUAL; "
                "GGA is intentionally never sent",
            )
            return None
        left = self._selected_rover_gps_state("left")
        right = self._selected_rover_gps_state("right")
        now_s = time.monotonic()
        maximum_age_s = float(active_config.readiness.maximum_gps_sample_age_s)

        def fresh(position: object, side: str) -> object | None:
            if position is None:
                log_skip(
                    f"{side}-missing",
                    f"[GNSS][NTRIP] GGA skipped: {side} GNSS has no message "
                    f"(policy={policy.value})",
                )
                return None
            if not bool(getattr(position, "has_position", False)):
                log_skip(
                    f"{side}-incomplete",
                    f"[GNSS][NTRIP] GGA skipped: {side} GNSS position is incomplete "
                    f"(policy={policy.value})",
                )
                return None
            try:
                timestamp_s = float(getattr(position, "timestamp_s"))
                age_s = now_s - timestamp_s
            except (AttributeError, TypeError, ValueError):
                age_s = float("nan")
            if not isfinite(age_s) or age_s < 0.0:
                log_skip(
                    f"{side}-timestamp-invalid",
                    f"[GNSS][NTRIP] GGA skipped: {side} GNSS timestamp is invalid "
                    f"(policy={policy.value})",
                )
                return None
            if age_s > maximum_age_s:
                log_skip(
                    f"{side}-stale",
                    f"[GNSS][NTRIP] GGA skipped: {side} GNSS sample is stale "
                    f"(age={age_s:.3f}s > {maximum_age_s:.3f}s, policy={policy.value})",
                )
                return None
            return position

        if policy == GgaPolicy.LEFT_GNSS:
            selected = fresh(left, "left")
            return None if selected is None else gga_from_position(selected)
        if policy == GgaPolicy.PER_ROVER_SESSION:
            selected = fresh(left, "left")
            return None if selected is None else gga_from_position(selected)
        # SYSTEM_CENTER: average WGS84 geodetic values for the two antenna hints.
        fresh_left = fresh(left, "left")
        fresh_right = fresh(right, "right")
        if fresh_left is None or fresh_right is None:
            def describe(position: object) -> str:
                if position is None:
                    return "no message received"
                if not bool(getattr(position, "has_position", False)):
                    return "message received but lat/lon/alt incomplete"
                return "stale or invalid timestamp"

            log_skip(
                "system-center-unavailable",
                "[GNSS][NTRIP] GGA skipped: no left/right GNSS position available "
                f"(policy={policy.value}, left={describe(left)}, right={describe(right)})",
            )
            return None
        timestamp_delta_ms = abs(
            float(getattr(fresh_right, "timestamp_s"))
            - float(getattr(fresh_left, "timestamp_s"))
        ) * 1000.0
        maximum_delta_ms = float(
            active_config.readiness.maximum_left_right_timestamp_delta_ms
        )
        if timestamp_delta_ms > maximum_delta_ms:
            log_skip(
                "system-center-skew",
                "[GNSS][NTRIP] GGA skipped: left/right GNSS timestamp delta "
                f"{timestamp_delta_ms:.3f}ms > {maximum_delta_ms:.3f}ms",
            )
            return None
        positions = (fresh_left, fresh_right)
        lat = sum(float(position.latitude_deg) for position in positions) / len(positions)
        lon = sum(float(position.longitude_deg) for position in positions) / len(positions)
        alt = sum(float(position.altitude_m) for position in positions) / len(positions)
        satellites_values = [
            int(value)
            for value in (getattr(position, "satellites_used", None) for position in positions)
            if value is not None
        ]
        accuracy_values = [
            float(value)
            for value in (getattr(position, "eph_m", None) for position in positions)
            if value is not None and isfinite(float(value))
        ]
        return format_nmea_gga(
            lat,
            lon,
            alt,
            fix_quality=1,
            satellites=min(satellites_values) if satellites_values else 12,
            hdop=max(accuracy_values) if accuracy_values else 1.0,
        )

    def _resolve_ntrip_credentials(self, network) -> "NtripCredentialResolution":
        """Resolve the username/password to authenticate with, in priority order.

        1. A password explicitly edited in the UI (this run's session).
        2. The OS secure credential store (keyring), keyed by
           host/mountpoint/username.
        3. The legacy ``username_env``/``password_env`` indirection, read
           from the OS environment.
        4. The untouched masked deployment fallback.
        5. None of the above -> raise with a specific, non-guessing reason.

        The username itself is plain config (``network.username``, or the
        legacy env var's resolved value); only the password is ever secret.
        """

        username = str(network.username).strip()
        session_password = ""
        if hasattr(self.panel, "gnss_ntrip_password_edit"):
            session_password = self.panel.gnss_ntrip_password_edit.text()

        # Both NTRIP password widgets start with the masked deployment
        # fallback.  That non-empty initial value is not an operator edit and
        # must not make a saved account credential unreachable after restart.
        # Lightweight/test panels predating this marker retain the historical
        # interpretation that their supplied value is explicit.
        session_password_explicit = bool(
            getattr(self.panel, "_gnss_ntrip_session_password_explicit", True)
        )
        current_key = _ntrip_credential_account_key(
            network.caster_host,
            network.mountpoint,
            username,
        )
        session_key = getattr(self.panel, "_gnss_ntrip_session_credential_key", None)
        # Lightweight legacy/test panels have no account marker. Real UI
        # handoffs always supply one; preserving the no-marker behavior keeps
        # old facade clients source compatible without allowing an actual
        # edited password to bleed into a different caster/account.
        session_key_matches = session_key is None or tuple(session_key) == current_key
        if (
            username
            and session_password
            and session_password_explicit
            and session_key_matches
        ):
            return NtripCredentialResolution(username=username, password=session_password, source="session")

        if not username:
            # A legacy env-only configuration never set a username in the
            # new field; fall through to the legacy tier below instead of
            # failing immediately.
            legacy_username = os.getenv(network.username_env, "") if network.username_env else ""
            legacy_password = os.getenv(network.password_env, "") if network.password_env else ""
            if legacy_username and legacy_password:
                return NtripCredentialResolution(username=legacy_username, password=legacy_password, source="legacy_env")
            raise NtripCredentialError("NTRIP username이 필요합니다")

        storage_available = ntrip_credentials.secure_storage_available()
        if storage_available:
            stored = ntrip_credentials.load_password(network.caster_host, network.mountpoint, username)
            if stored:
                return NtripCredentialResolution(username=username, password=stored, source="keyring")

        legacy_username = os.getenv(network.username_env, "") if network.username_env else ""
        legacy_password = os.getenv(network.password_env, "") if network.password_env else ""
        if legacy_username and legacy_password:
            return NtripCredentialResolution(username=legacy_username, password=legacy_password, source="legacy_env")

        # No saved or environment credential exists.  Preserve the
        # deployment fallback for an untouched field profile, but only after
        # the stronger account-specific sources above have been exhausted.
        if username and session_password and session_key_matches:
            return NtripCredentialResolution(username=username, password=session_password, source="session")

        if not storage_available:
            raise NtripCredentialError(
                "보안 자격증명 저장소를 사용할 수 없습니다 -- password를 입력하세요"
            )
        raise NtripCredentialError(
            "저장된 NTRIP password가 없습니다 -- password를 입력하거나 저장된 자격증명을 확인하세요"
        )

    def _maybe_persist_ntrip_credential(self, network, credential: "NtripCredentialResolution") -> None:
        """Save a freshly-typed session password, but only when asked to.

        A password resolved from the keyring or the legacy env-var tier is
        already stored (or intentionally not persisted anywhere); only a
        brand-new "session" password is ever a candidate for saving, and
        only when ``remember_password`` is set.
        """

        if credential.source != "session" or not network.remember_password:
            return
        ntrip_credentials.save_password(
            network.caster_host, network.mountpoint, credential.username, credential.password
        )

    def _commit_pending_ntrip_credential(self, status_line: str) -> None:
        """Persist a typed credential only after the caster accepted it."""

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

    def _reject_ntrip_start(self, message: str, tone: str) -> None:
        """Every guard below routes through here so a rejected Start NTRIP
        click always leaves a line in the visible log, not only the small
        "NTRIP" status label in the GNSS Base status grid -- a click that
        only updates that label is easy to miss and reads as "did nothing".
        """

        self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], message, tone)
        self.panel._log(f"[GNSS] NTRIP 시작 거부: {message}")

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
                self.panel._gnss_selected_operation_mode()
                != GnssOperationMode.LOCAL_SURVEY_IN
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
        if self.panel.gnss_network_thread is not None:
            return
        config = self.panel._build_gnss_operation_config()
        if config.operation_mode == GnssOperationMode.LOCAL_SURVEY_IN:
            self._reject_ntrip_start("현장 모드에서는 NTRIP을 시작할 수 없습니다", "warn")
            return
        if not config.network.caster_host:
            self._reject_ntrip_start("caster host 필요", "bad")
            return
        if not config.network.mountpoint:
            self._reject_ntrip_start("mountpoint 필요", "bad")
            return
        if config.operation_mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE and not self.panel.gnss_base_serial_connected:
            self._reject_ntrip_start("보조 Fixed Base 모드는 먼저 Base serial을 연결하세요", "bad")
            return
        if config.operation_mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE:
            if config.network.gga_policy != GgaPolicy.BASE_GNSS:
                self._reject_ntrip_start("보조 Fixed Base는 Base GNSS GGA가 필요합니다", "bad")
                return
            if config.fixed_base.height_type != HeightType.ELLIPSOID:
                self._reject_ntrip_start("절대 WGS84 운용은 Fixed Base 높이를 ellipsoid로 설정하세요", "bad")
                return
            base_sample = getattr(self.panel, "latest_gnss_base_position", None)
            if base_sample is None or max(0.0, time.monotonic() - base_sample.timestamp_s) > 15.0:
                self._reject_ntrip_start("Base WGS84 위치 샘플 대기 중입니다", "warn")
                return
        try:
            credential = self._resolve_ntrip_credentials(config.network)
        except NtripCredentialError as exc:
            self._reject_ntrip_start(str(exc), "bad")
            return
        try:
            self.panel._gnss_configure_manager(config)
            self.panel.gnss_operation_manager.set_source_identity(self.panel._gnss_source_identity(config))
            if not self.panel._gnss_transition(GnssOperationState.INITIALIZING, "NTRIP 운용 초기화"):
                return
            if not self.panel._gnss_transition(GnssOperationState.NTRIP_CONNECTING, "NTRIP caster 연결 중"):
                return
        except (ValueError, RuntimeError) as exc:
            self._reject_ntrip_start(f"운용 설정 실패: {exc}", "bad")
            return
        self.panel.gnss_operation_gate_active = True
        self.panel.gnss_active_operation_config = config
        self.panel.gnss_network_connected = False
        self.panel.gnss_network_phase = "CONNECTING"
        self.panel.gnss_network_stats = None
        self.panel.gnss_network_status_line = ""
        self.panel.gnss_network_last_error = ""
        self.panel.gnss_network_reconnect_count = 0
        self.panel.gnss_network_rtcm_parser = Rtcm3StreamParser()
        self.panel.gnss_coordinate_samples.clear()
        if hasattr(self.panel, "gnss_base_fixed_state_plot"):
            self.panel.gnss_base_fixed_state_plot.points.clear()
        if hasattr(self.panel, "gnss_base_fixed_scatter_plot"):
            self.panel.gnss_base_fixed_scatter_plot.points.clear()
        self.panel.gnss_fixed_base_profile = None
        self.panel.gnss_fixed_base_live_profile = None
        self.panel.gnss_fixed_base_pending_profile = None
        self.panel.gnss_fixed_base_command_sent = False
        self.panel.gnss_fixed_base_reconfigure_expected = None
        self.panel.gnss_fixed_base_reconfigure_ack = False
        self.panel.gnss_fixed_base_reconfigure_readback = False
        self.panel.gnss_fixed_base_reconfigure_rtcm = False
        self.panel.gnss_fixed_base_reconfigure_deadline_s = None
        self.panel._reset_gnss_base_rtcm_stats()
        self.panel.gnss_network_start_button.setEnabled(False)
        self.panel.gnss_network_stop_button.setEnabled(True)
        self.panel._set_gnss_operation_controls_locked(True)
        self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], "NTRIP 연결 중", "warn")
        self.panel._gnss_ntrip_pending_credential = (
            config.network,
            credential,
        ) if credential.source == "session" else None
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
                self._rollback_ntrip_start_setup(
                    "RTCM 주입 worker를 시작할 수 없습니다"
                )
                return
            self._sync_rtcm_pipeline_config(reset=True)
            self._sync_rtcm_evidence_context()
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            message = f"NTRIP RTCM 준비 실패: {exc}"
            self._reject_ntrip_start(message, "bad")
            self._rollback_ntrip_start_setup(message)
            return
        thread = QtCore.QThread(self.panel)
        worker = NtripNetworkWorker(
            config=config.network,
            gga_provider=self.panel._gnss_gga_provider,
            username=credential.username,
            password=credential.password,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        _connect_gui_signal(worker.connected, self.panel._handle_gnss_network_connected)
        self._wire_rtcm_ntrip_source(worker, pipeline)
        _connect_gui_signal(worker.reconnecting, self.panel._handle_gnss_network_reconnecting)
        _connect_gui_signal(worker.failed, self.panel._handle_gnss_network_failed)
        _connect_gui_signal(worker.diagnostic, self.panel._handle_gnss_network_diagnostic)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        _connect_gui_signal(
            thread.finished,
            lambda thread=thread: self.panel._clear_gnss_network_worker(thread),
        )
        self.panel.gnss_network_thread = thread
        self.panel.gnss_network_worker = worker
        thread.start()

    def stop_gnss_network(self) -> bool:
        worker = self.panel.gnss_network_worker
        if worker is not None:
            worker.request_stop()
        thread = self.panel.gnss_network_thread
        self.panel.gnss_network_connected = False
        self.panel.gnss_network_phase = "STOPPED"
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
        self._commit_pending_ntrip_credential(status_line)
        self.panel.gnss_network_connected = True
        self.panel.gnss_network_phase = "CONNECTED"
        self.panel.gnss_network_stats = stats
        self.panel.gnss_network_last_error = ""
        self.panel.gnss_network_status_line = str(status_line)
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
        self.panel.gnss_network_stats = stats
        if (
            self.panel.gnss_network_connected
            and getattr(stats, "last_chunk_monotonic_s", None) is not None
        ):
            self.panel.gnss_network_phase = "STREAMING"

    def _handle_gnss_network_chunk(self, chunk: object) -> None:
        if not isinstance(chunk, (bytes, bytearray)):
            return
        payload = bytes(chunk)
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
        """Surface NtripClient's per-event evidence (connect/GGA TX outcome).

        These events never stop or restart anything -- they carry no status
        of their own -- so without this they would never reach the visible
        log at all, unlike a stream failure which already flows through
        ``reconnecting``/``failed``.
        """

        self.panel._log(f"[GNSS][NTRIP] {message}")

    def _handle_gnss_network_failed(self, message: str) -> None:
        self.panel.gnss_network_connected = False
        auth_failed = "401" in str(message) or "unauthorized" in str(message).lower()
        self.panel.gnss_network_phase = "HTTP_AUTH_FAILED" if auth_failed else "ERROR"
        self.panel.gnss_network_last_error = str(message)
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
        if not self.panel.gnss_operation_gate_active:
            return
        self.panel.gnss_network_connected = False
        self.panel.gnss_network_phase = "RECONNECTING"
        self.panel.gnss_network_last_error = str(message)
        self.panel.gnss_network_reconnect_count = int(
            getattr(self.panel, "gnss_network_reconnect_count", 0) or 0
        ) + 1
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["network"],
            f"NTRIP 재연결 대기: {message}",
            "warn",
        )
        self.panel._log(f"[GNSS] NTRIP 재연결 시도 {self.panel.gnss_network_reconnect_count}: {message}")

    def _clear_gnss_network_worker(self, thread: QtCore.QThread) -> None:
        if self.panel.gnss_network_thread is thread:
            self.panel.gnss_network_thread = None
            self.panel.gnss_network_worker = None
            self.panel.gnss_network_connected = False
            if getattr(self.panel, "gnss_network_phase", "") not in {"ERROR", "HTTP_AUTH_FAILED"}:
                self.panel.gnss_network_phase = "STOPPED"
            self.panel.gnss_active_operation_config = None
            self.panel._set_gnss_operation_controls_locked(False)
            if hasattr(self.panel, "gnss_network_start_button"):
                self.panel.gnss_network_start_button.setEnabled(self.panel._gnss_selected_operation_mode() != GnssOperationMode.LOCAL_SURVEY_IN)
            if hasattr(self.panel, "gnss_network_stop_button"):
                self.panel.gnss_network_stop_button.setEnabled(False)
        thread.deleteLater()

    def _record_gnss_base_position_row(self, sample: GnssCoordinateSample) -> None:
        """Persist one GNSS Base position sample (NAV-PVT) to gnss_base.jsonl.

        Reuses ``_raw_timeline_row`` (the same helper camera/px4 recording
        uses) so the row gets the same session_utc_ns/host_receive_utc_ns
        UTC columns.  The sample itself only carries a monotonic timestamp
        (the UBX NAV-PVT parser does not extract the message's UTC
        time-of-week fields), so source_utc_ns is left unset and callers
        fall back to the session-anchor-derived session_utc_ns.
        """

        rows = self.panel.gnss_base_recording_rows
        row = self.panel._raw_timeline_row(
            side="base",
            sample_type="gnss_base_position",
            row_contract="one row per GNSS Base position sample (NAV-PVT) received by the GUI",
            sample_monotonic_s=sample.timestamp_s,
            host_receive_monotonic_ns=time.monotonic_ns(),
            host_receive_utc_ns=time.time_ns(),
            source_utc_ns=None,
            source_utc_method="unavailable",
            sequence=len(rows),
        )
        if row is None:
            return
        row.update(
            {
                "latitude_deg": sample.latitude_deg,
                "longitude_deg": sample.longitude_deg,
                "ellipsoid_height_m": sample.ellipsoid_height_m,
                "msl_altitude_m": sample.msl_altitude_m,
                "height_type": sample.height_type.value,
                "fixed": sample.fixed,
                "device_id": sample.device_id,
                "gnss_operation_state": self.panel.gnss_operation_manager.state.value,
            }
        )
        rows.append(row)
        self.panel.external_parameter_recording_rows.append(row)

    def _handle_gnss_base_position(self, sample: object) -> None:
        if not isinstance(sample, GnssCoordinateSample):
            return
        self.panel.gnss_coordinate_samples.append(sample)
        self.panel.latest_gnss_base_position = sample
        if "base_position" in self.panel.gnss_base_status_labels:
            height_m = sample.selected_height_m
            height_text = "--" if height_m is None else f"{height_m:.3f}m"
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["base_position"],
                f"WGS84 {sample.latitude_deg:.7f}, {sample.longitude_deg:.7f} | h={height_text} ({sample.height_type.value}) | fix={sample.fixed}",
                "good" if sample.fixed else "warn",
            )
        config = self.panel._gnss_active_operation_config()
        # Base NAV-PVT is raw measurement evidence in every GNSS mode.  The
        # assisted-fixed mode below adds state-machine behavior, but it is not
        # the owner of persistence.
        self.panel._record_gnss_base_position_row(sample)
        if config.operation_mode != GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE:
            return
        if hasattr(self.panel, "gnss_base_fixed_state_plot"):
            self.panel.gnss_base_fixed_state_plot.add_point(
                TelemetryHistoryPoint(sample.timestamp_s, "fix", 1.0 if sample.fixed else 0.0)
            )
        if not self.panel.gnss_network_connected or self.panel.gnss_fixed_base_command_sent:
            return
        state = self.panel.gnss_operation_manager.state
        if state == GnssOperationState.NTRIP_ACTIVE:
            self.panel._gnss_transition(GnssOperationState.BASE_AS_ROVER_FLOAT, "Base가 NTRIP 보정 rover로 수렴 중")
            state = self.panel.gnss_operation_manager.state
        if sample.fixed and state == GnssOperationState.BASE_AS_ROVER_FLOAT:
            self.panel._gnss_transition(GnssOperationState.BASE_AS_ROVER_FIXED, "Base rover가 RTK FIXED")
            state = self.panel.gnss_operation_manager.state
        if sample.fixed and state == GnssOperationState.BASE_AS_ROVER_FIXED:
            self.panel._gnss_transition(GnssOperationState.FIXED_COORDINATE_COLLECTING, "고정 좌표 반복 측정 수집")
        if self.panel.gnss_operation_manager.state != GnssOperationState.FIXED_COORDINATE_COLLECTING:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["base_fixed"],
                f"{self.panel.gnss_operation_manager.state.value} | Base fix={sample.fixed}",
                "warn",
            )
            return
        if sample.height_type != config.fixed_base.height_type:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["base_fixed"],
                "Base 좌표 height datum이 활성 GNSS 운용 설정과 다릅니다",
                "bad",
            )
            self.panel._gnss_transition(GnssOperationState.FAULT, "Base coordinate height datum changed during operation")
            return
        validation = FixedBaseInitializer(config.fixed_base).validate(
            tuple(self.panel.gnss_coordinate_samples),
            acquisition_start_utc="",
            acquisition_end_utc="",
            source_caster=config.network.caster_host,
            source_mountpoint=config.network.mountpoint,
        )
        profile = validation.profile
        # Keep a live copy of the in-progress (not-yet-validated) profile so
        # the UI can show real scatter/duration numbers while collecting,
        # distinct from panel.gnss_fixed_base_profile which is only set once
        # the coordinate is fully validated AND applied to the receiver.
        self.panel.gnss_fixed_base_live_profile = profile
        if profile is None:
            self.panel._set_health_label(self.panel.gnss_base_status_labels["base_fixed"], "고정 좌표 프로파일 수집 중", "warn")
            return
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["base_fixed"],
            f"FIXED 수집 {profile.collection_duration_s:.1f}s/{config.fixed_base.acquisition_duration_s:.1f}s | "
            f"scatter H/V={profile.horizontal_scatter_m:.3f}/{profile.vertical_scatter_m:.3f}m",
            "warn" if not validation.valid else "good",
        )
        if hasattr(self.panel, "gnss_base_fixed_scatter_plot"):
            self.panel.gnss_base_fixed_scatter_plot.add_point(
                TelemetryHistoryPoint(sample.timestamp_s, "horizontal", profile.horizontal_scatter_m)
            )
            self.panel.gnss_base_fixed_scatter_plot.add_point(
                TelemetryHistoryPoint(sample.timestamp_s, "vertical", profile.vertical_scatter_m)
            )
            self.panel.gnss_base_fixed_scatter_plot.add_point(
                TelemetryHistoryPoint(sample.timestamp_s, "target_h", config.fixed_base.maximum_horizontal_scatter_m)
            )
            self.panel.gnss_base_fixed_scatter_plot.add_point(
                TelemetryHistoryPoint(sample.timestamp_s, "target_v", config.fixed_base.maximum_vertical_scatter_m)
            )
        if not validation.valid:
            return
        if profile.height_type != HeightType.ELLIPSOID:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["base_fixed"],
                "고정 좌표 검증됨이지만 TMODE3에는 WGS84 ellipsoid height가 필요합니다",
                "bad",
            )
            return
        worker = self.panel.gnss_base_serial_worker
        if worker is None or not self.panel.gnss_base_serial_connected:
            self.panel._set_health_label(self.panel.gnss_base_status_labels["base_fixed"], "고정 좌표 검증됨 | Base serial 재연결 필요", "bad")
            return
        if not self.panel._gnss_transition(GnssOperationState.FIXED_COORDINATE_VALIDATED, "반복 측정 좌표 검증 통과"):
            return
        if not self.panel._gnss_transition(GnssOperationState.BASE_RECONFIGURING, "CFG-TMODE3 fixed 좌표 적용"):
            return
        accuracy_m = max(0.01, profile.horizontal_scatter_m, profile.vertical_scatter_m)
        fixed_frame = ubx_cfg_tmode3_fixed_frame(
            latitude_deg=profile.latitude_deg,
            longitude_deg=profile.longitude_deg,
            ellipsoid_height_m=profile.height_m,
            fixed_position_accuracy_m=accuracy_m,
        )
        expected = parse_ubx_cfg_tmode3(fixed_frame[6:-2])
        if expected is None:
            self.panel._gnss_transition(GnssOperationState.FAULT, "CFG-TMODE3 fixed command could not be decoded")
            return
        self.panel._reset_gnss_base_rtcm_stats()
        worker.send_bytes("CFG-TMODE3 fixed coordinate", fixed_frame)
        self.panel.gnss_fixed_base_pending_profile = profile
        self.panel.gnss_fixed_base_reconfigure_expected = expected
        self.panel.gnss_fixed_base_reconfigure_ack = False
        self.panel.gnss_fixed_base_reconfigure_readback = False
        self.panel.gnss_fixed_base_reconfigure_rtcm = False
        self.panel.gnss_fixed_base_reconfigure_deadline_s = time.monotonic() + 10.0
        self.panel.gnss_fixed_base_command_sent = True
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["base_fixed"],
            "CFG-TMODE3 fixed 전송됨 | ACK/readback/post-config RTCM 대기",
            "warn",
        )

    def _fail_gnss_fixed_base_reconfiguration(self, reason: str) -> None:
        self.panel.gnss_fixed_base_pending_profile = None
        self.panel.gnss_fixed_base_reconfigure_expected = None
        self.panel.gnss_fixed_base_reconfigure_deadline_s = None
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_fixed"], f"Fixed Base 재설정 실패: {reason}", "bad")
        self.panel._gnss_transition(GnssOperationState.FAULT, f"fixed base reconfiguration failed: {reason}")

    def _handle_gnss_base_ubx_message(self, message_class: int, message_id: int, payload: object) -> None:
        """Advance fixed-base setup only from receiver evidence, not queued writes."""

        if not isinstance(payload, (bytes, bytearray)):
            return
        raw_payload = bytes(payload)
        ack = parse_ubx_ack(int(message_class), int(message_id), raw_payload)
        if self.panel.gnss_operation_manager.state == GnssOperationState.BASE_SURVEYING:
            if ack is None:
                return
            accepted, target_class, target_id = ack
            if (target_class, target_id) != (0x06, 0x71):
                return
            generation = int(getattr(self.panel, "_gnss_survey_generation", 0) or 0)
            sent_generation = int(
                getattr(self.panel, "_gnss_survey_command_sent_generation", 0) or 0
            )
            if generation <= 0 or sent_generation != generation:
                return
            if not accepted:
                self.panel._gnss_transition(
                    GnssOperationState.FAULT,
                    "survey-in CFG-TMODE3 ACK-NAK",
                )
                return
            self.panel._gnss_survey_ack_generation = generation
            worker = getattr(self.panel, "gnss_base_serial_worker", None)
            if worker is not None:
                worker.send_bytes(
                    "NAV-SVIN poll after survey-in ACK",
                    ubx_nav_svin_poll_frame(),
                )
            return
        if self.panel.gnss_operation_manager.state != GnssOperationState.BASE_RECONFIGURING:
            return
        if ack is not None:
            accepted, target_class, target_id = ack
            if (target_class, target_id) != (0x06, 0x71):
                return
            if not accepted:
                self.panel._fail_gnss_fixed_base_reconfiguration("CFG-TMODE3 ACK-NAK")
                return
            self.panel.gnss_fixed_base_reconfigure_ack = True
            worker = self.panel.gnss_base_serial_worker
            if worker is None or not self.panel.gnss_base_serial_connected:
                self.panel._fail_gnss_fixed_base_reconfiguration("Base serial disconnected before CFG-TMODE3 readback")
                return
            worker.send_bytes("CFG-TMODE3 poll after fixed coordinate ACK", ubx_cfg_tmode3_poll_frame())
            self.panel._set_health_label(self.panel.gnss_base_status_labels["base_fixed"], "CFG-TMODE3 ACK 수신 | readback 대기", "warn")
            return
        if (int(message_class), int(message_id)) != (0x06, 0x71):
            return
        expected = self.panel.gnss_fixed_base_reconfigure_expected
        received = parse_ubx_cfg_tmode3(raw_payload)
        if expected is None or received is None:
            self.panel._fail_gnss_fixed_base_reconfiguration("invalid CFG-TMODE3 readback")
            return
        keys = ("mode", "ecef_cm", "ecef_hp_0_1mm", "fixed_accuracy_0_1mm")
        if any(received.get(key) != expected.get(key) for key in keys):
            self.panel._fail_gnss_fixed_base_reconfiguration("CFG-TMODE3 readback mismatch")
            return
        self.panel.gnss_fixed_base_reconfigure_readback = True
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_fixed"], "CFG-TMODE3 readback 확인 | post-config RTCM 대기", "warn")
        self.panel._maybe_activate_gnss_fixed_base()

    def _maybe_activate_gnss_fixed_base(self) -> None:
        if self.panel.gnss_operation_manager.state != GnssOperationState.BASE_RECONFIGURING:
            return
        if not (
            self.panel.gnss_fixed_base_reconfigure_ack
            and self.panel.gnss_fixed_base_reconfigure_readback
            and self.panel.gnss_fixed_base_reconfigure_rtcm
        ):
            return
        profile = self.panel.gnss_fixed_base_pending_profile
        if profile is None:
            self.panel._fail_gnss_fixed_base_reconfiguration("validated profile was lost")
            return
        self.panel.gnss_fixed_base_profile = profile
        self.panel.gnss_fixed_base_pending_profile = None
        self.panel.gnss_fixed_base_reconfigure_deadline_s = None
        self.panel._gnss_transition(GnssOperationState.LOCAL_RTCM_ACTIVE, "CFG-TMODE3 ACK/readback와 post-config Base RTCM 확인")

    def _refresh_gnss_operation_readiness(self) -> None:
        if not self.panel.gnss_operation_gate_active:
            return
        config = self.panel.gnss_operation_manager.config
        if (
            self.panel.gnss_active_operation_config is not None
            and self.panel.gnss_operation_manager.state in {GnssOperationState.READY, GnssOperationState.MEASURING}
            and self.panel._build_gnss_operation_config() != config
        ):
            self.panel.gnss_operation_manager.invalidate("GNSS controls changed during active operation")
            return
        if self.panel.gnss_operation_manager.state == GnssOperationState.BASE_RECONFIGURING:
            deadline_s = self.panel.gnss_fixed_base_reconfigure_deadline_s
            if deadline_s is not None and time.monotonic() > deadline_s:
                self.panel._fail_gnss_fixed_base_reconfiguration("CFG-TMODE3 ACK/readback/post-config RTCM timeout")
            return
        target_sides = self.panel._gnss_base_rtcm_target_sides()
        now_s = time.monotonic()
        injection = self.panel.gnss_base_rtcm_stats.get("injection")
        injection_map = injection if isinstance(injection, dict) else {}
        mavlink = self.panel.mavlink_pixhawk
        connections = getattr(mavlink, "connections", None)
        heartbeat_sides = getattr(mavlink, "_heartbeat_logged", set())
        health_reader = getattr(mavlink, "mavlink_health_snapshot", None)
        routes: dict[str, RoverCorrectionStatus] = {}
        rover_samples: dict[str, RoverReadinessInput] = {}
        for side in target_sides:
            side_stats = injection_map.get(side)
            if not isinstance(side_stats, dict):
                side_stats = {}
            health = None
            if callable(health_reader):
                try:
                    health = health_reader(side, now_s=now_s)
                except Exception:
                    health = None
            if health is not None:
                endpoint_live = bool(getattr(health, "identity_verified", False)) and bool(
                    getattr(health, "rtcm_eligible", False)
                )
                endpoint_identity = (
                    f"mavlink:{side}:sysid={getattr(health, 'observed_system_id', '')}:"
                    f"generation={getattr(health, 'connection_generation', 0)}"
                    if endpoint_live
                    else ""
                )
                health_state = str(
                    getattr(
                        getattr(health, "state", None),
                        "value",
                        getattr(health, "state", ""),
                    )
                )
                health_fault = (
                    ""
                    if endpoint_live
                    else f"MAVLink {health_state or 'ineligible'}: "
                    f"{getattr(health, 'stall_reason', '') or 'RTCM blocked'}"
                )
            elif isinstance(connections, dict):
                endpoint_live = side in connections and side in heartbeat_sides
                endpoint_identity = f"mavlink:{side}" if endpoint_live else ""
                health_fault = ""
            else:
                # Test adapters may not expose the production subscriber
                # internals; their presence still represents a route endpoint.
                endpoint_identity = "" if mavlink is None else f"mavlink:{side}"
                health_fault = ""
            routes[side] = RoverCorrectionStatus(
                target=side,
                enabled=True,
                endpoint_identity=endpoint_identity,
                bytes_sent=int(side_stats.get("bytes") or 0),
                packets_sent=int(side_stats.get("packets") or 0),
                last_delivery_monotonic_s=side_stats.get("last_delivery_s"),
                fault_reason=(
                    health_fault
                    or (
                        "RTCM blocked: MAVLink stalled"
                        if int(side_stats.get("blocked_stall") or 0) > 0
                        else "injection failed"
                        if int(side_stats.get("fail") or 0) > 0
                        else ""
                    )
                ),
            )
        for side in ("left", "right", "drone"):
            gps = self._selected_rover_gps_state(side)
            if gps is None:
                self.panel.gnss_rover_fixed_since[side] = None
                continue
            if gps.fix_type == 6:
                if self.panel.gnss_rover_fixed_since.get(side) is None:
                    self.panel.gnss_rover_fixed_since[side] = now_s
            else:
                self.panel.gnss_rover_fixed_since[side] = None
            rover_samples[side] = RoverReadinessInput(
                target=side,
                fix_type=gps.fix_type,
                gps_sample_monotonic_s=gps.timestamp_s,
                eph_m=gps.eph_m,
                epv_m=getattr(gps, "epv_m", None),
                rtk_fixed_since_monotonic_s=self.panel.gnss_rover_fixed_since.get(side),
                device_id=str(getattr(gps, "topic", side)),
            )
        last_frame_s = self.panel.gnss_base_rtcm_stats.get("last_frame_s")
        baseline_vector = self.panel.latest_baseline_vector
        baseline_quality = self.panel.latest_baseline_quality
        source = self.panel.gnss_operation_manager.source_identity
        candidate = evaluate_gnss_readiness(
            config,
            GnssReadinessInputs(
                # Evaluate the policy as a candidate; the transition state is
                # applied below only after every enabled criterion passes.
                state=GnssOperationState.READY,
                now_monotonic_s=now_s,
                correction_received_monotonic_s=last_frame_s,
                correction_source_id="" if source is None else source.stable_id,
                routes=routes,
                rover_samples=rover_samples,
                left_right_timestamp_delta_ms=(
                    None if baseline_vector is None else baseline_vector.timestamp_delta_ms
                ),
                baseline_std_m=(
                    None if baseline_quality is None else baseline_quality.baseline_length_std_m
                ),
            ),
        )
        reasons = candidate.reasons
        # A disabled readiness gate remains observable in the status snapshot,
        # but must not leave the state machine permanently below READY.  The
        # measurement manager records the warning-only, no-contract path.
        transition_reasons = reasons if config.readiness_gate_enabled else ()
        state = self.panel.gnss_operation_manager.state
        # LOCAL_RTCM_ACTIVE is Group B's (STATE_STATUS_GROUP == BASE_FIXED)
        # terminal state -- reaching it means the Base has finished its own
        # convergence/fixed-coordinate sequence and rover convergence can
        # begin.  NTRIP_ACTIVE is Group A (STATE_STATUS_GROUP == NETWORK); it
        # only implies convergence-eligibility in NETWORK_RTK_DIRECT, where
        # there is no Group B stage at all (RTCM goes straight to rovers). In
        # NETWORK_ASSISTED_FIXED_BASE, NTRIP_ACTIVE just means the caster feed
        # is reaching the Base serial port; the Base must still complete its
        # own Fixed-coordinate sequence (up to LOCAL_RTCM_ACTIVE) before rover
        # convergence readiness has any meaning.
        convergence_eligible = state == GnssOperationState.LOCAL_RTCM_ACTIVE or (
            state == GnssOperationState.NTRIP_ACTIVE
            and config.operation_mode == GnssOperationMode.NETWORK_RTK_DIRECT
        )
        if convergence_eligible and not transition_reasons:
            self.panel._gnss_transition(GnssOperationState.ROVERS_CONVERGING, "RTCM rover 주입 시작")
            state = self.panel.gnss_operation_manager.state
        if state == GnssOperationState.ROVERS_CONVERGING and not transition_reasons:
            self.panel._gnss_transition(GnssOperationState.ALIGNMENT_COLLECTING, "양쪽 rover FIXED 수신")
            state = self.panel.gnss_operation_manager.state
        if state == GnssOperationState.ALIGNMENT_COLLECTING and not transition_reasons:
            self.panel._gnss_transition(GnssOperationState.ALIGNMENT_VALID, "GNSS rover 정합 조건 통과")
            state = self.panel.gnss_operation_manager.state
        if state == GnssOperationState.ALIGNMENT_VALID and not transition_reasons:
            self.panel._gnss_transition(GnssOperationState.READY, "GNSS measurement readiness 통과")
            state = self.panel.gnss_operation_manager.state
        if state == GnssOperationState.READY and transition_reasons:
            self.panel._gnss_transition(GnssOperationState.DEGRADED, "; ".join(reasons))
            state = self.panel.gnss_operation_manager.state
        if state == GnssOperationState.DEGRADED and not transition_reasons:
            self.panel._gnss_transition(GnssOperationState.READY, "GNSS 조건 복구")
            state = self.panel.gnss_operation_manager.state
        if state == GnssOperationState.MEASURING and transition_reasons:
            self.panel.gnss_operation_manager.invalidate("GNSS readiness changed: " + ", ".join(reasons))
            state = self.panel.gnss_operation_manager.state
        snapshot = GnssReadinessSnapshot(
            ready=not reasons,
            state=state,
            reasons=reasons,
            warnings=candidate.warnings,
            mode=config.operation_mode,
            correction_source_id="" if source is None else source.stable_id,
            timestamp_s=now_s,
            reason_messages=candidate.reason_messages,
        )
        self.panel.gnss_operation_manager.update_readiness(snapshot)
        if hasattr(self.panel, "gnss_base_status_labels") and "operation" in self.panel.gnss_base_status_labels and reasons:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["operation"],
                f"{state.value} | " + "; ".join(snapshot.reason_messages.get(code, code) for code in reasons[:3]),
                "warn",
            )

    def connect_gnss_base_serial(self) -> None:
        if self.panel.gnss_base_serial_thread is not None:
            return
        port = self.panel.gnss_base_serial_port_edit.text().strip()
        if not port:
            self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], "serial port 필요", "bad")
            return
        baud = int(self.panel.gnss_base_baud_spin.value())
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], f"connecting {port} @ {baud}", "warn")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["rtcm_rx"], "raw bytes 대기 중", "idle")
        self.panel._reset_gnss_base_rtcm_stats()
        self.panel.gnss_base_connect_button.setEnabled(False)
        self.panel.gnss_base_disconnect_button.setEnabled(True)
        thread = QtCore.QThread(self.panel)
        height_value = self.panel.gnss_fixed_height_combo.currentData()
        try:
            height_type = HeightType(height_value)
        except (TypeError, ValueError):
            height_type = HeightType.ELLIPSOID
        worker = GnssBaseSerialWorker(port=port, baud=baud, height_type=height_type)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        _connect_gui_signal(worker.connected, self.panel._handle_gnss_base_serial_connected)
        _connect_gui_signal(worker.stats_changed, self.panel._handle_gnss_base_serial_stats)
        _connect_gui_signal(worker.survey_in_changed, self.panel._handle_gnss_base_survey_in)
        _connect_gui_signal(worker.satellite_count_changed, self.panel._handle_gnss_base_satellite_count)
        _connect_gui_signal(worker.satellite_quality_changed, self.panel._handle_gnss_base_satellite_quality)
        _connect_gui_signal(worker.position_changed, self.panel._handle_gnss_base_position)
        _connect_gui_signal(worker.ubx_message_received, self.panel._handle_gnss_base_ubx_message)
        _connect_gui_signal(worker.command_sent, self.panel._handle_gnss_base_command_sent)
        _connect_gui_signal(worker.failed, self.panel._handle_gnss_base_serial_failed)
        _connect_gui_signal(worker.disconnected, self.panel._handle_gnss_base_serial_disconnected)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        _connect_gui_signal(
            thread.finished,
            lambda thread=thread: self.panel._clear_gnss_base_serial_worker(thread),
        )
        self.panel.gnss_base_serial_thread = thread
        self.panel.gnss_base_serial_worker = worker
        pipeline = self._ensure_rtcm_pipeline()
        self._wire_rtcm_base_source(worker, pipeline)
        self._wire_rtcm_pipeline_base_worker(worker)
        if self._rtcm_route_config().base_targets:
            self._ensure_pixhawk_rtcm_worker()
        self._sync_rtcm_pipeline_config(reset=True)
        self._sync_rtcm_evidence_context()
        thread.start()

    def disconnect_gnss_base_serial(self) -> None:
        worker = self.panel.gnss_base_serial_worker
        if worker is not None:
            worker.request_stop()
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
        worker = self.panel.gnss_base_serial_worker
        if worker is None:
            self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "serial 미연결", "bad")
            return
        config = self.panel._build_gnss_operation_config()
        if config.operation_mode != GnssOperationMode.LOCAL_SURVEY_IN:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["survey_in"],
                "현재 모드에서는 현장 Survey-in을 시작할 수 없습니다",
                "warn",
            )
            return
        self.panel._gnss_configure_manager(config)
        self.panel.gnss_operation_manager.set_source_identity(self.panel._gnss_source_identity(config))
        self.panel.gnss_active_operation_config = config
        self.panel.gnss_operation_gate_active = True
        self.panel._set_gnss_operation_controls_locked(True)
        generation = int(getattr(self.panel, "_gnss_survey_generation", 0) or 0) + 1
        self.panel._gnss_survey_generation = generation
        self.panel._gnss_survey_command_sent_generation = 0
        self.panel._gnss_survey_ack_generation = 0
        self.panel._gnss_survey_active_seen_generation = 0
        self.panel._gnss_survey_requested_duration_s = int(
            self.panel.gnss_base_survey_duration_spin.value()
        )
        self.panel._gnss_survey_requested_accuracy_m = float(
            self.panel.gnss_base_survey_accuracy_spin.value()
        )
        self.panel._gnss_transition(GnssOperationState.INITIALIZING, "현장 Base Survey-in 초기화")
        self.panel._gnss_transition(GnssOperationState.BASE_SURVEYING, "현장 Base Survey-in 진행")
        frame = ubx_cfg_tmode3_survey_in_frame(
            min_duration_s=self.panel.gnss_base_survey_duration_spin.value(),
            accuracy_m=self.panel.gnss_base_survey_accuracy_spin.value(),
        )
        self.panel.gnss_base_survey_samples.clear()
        worker.send_bytes("CFG-TMODE3 survey-in start", frame)
        worker.send_bytes("NAV-SVIN poll after survey-in start", ubx_nav_svin_poll_frame())
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "survey-in 시작 명령 전송 대기", "warn")

    def stop_gnss_base_survey_in(self) -> None:
        worker = self.panel.gnss_base_serial_worker
        if worker is None:
            self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "serial 미연결", "bad")
            return
        worker.send_bytes("CFG-TMODE3 disabled", ubx_cfg_tmode3_disabled_frame())
        worker.send_bytes("NAV-SVIN poll after survey-in stop", ubx_nav_svin_poll_frame())
        self.panel.gnss_operation_gate_active = False
        self.panel._gnss_survey_command_sent_generation = 0
        self.panel._gnss_survey_ack_generation = 0
        self.panel._gnss_survey_active_seen_generation = 0
        self.panel.gnss_operation_manager.stop()
        self.panel.gnss_active_operation_config = None
        self.panel._set_gnss_operation_controls_locked(False)
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "survey-in 중지 명령 전송 대기", "warn")
        self.panel._handle_gnss_operation_mode_changed()

    def discover_gnss_base_serial_ports(self) -> None:
        if self.panel.gnss_base_detect_thread is not None:
            return
        self.panel.gnss_base_discover_button.setEnabled(False)
        self.panel.workstation_gnss_base_discovery_status = "SEARCHING"
        self.panel.gnss_base_apply_candidate_button.setEnabled(False)
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], "base port 탐색 중", "warn")
        thread = QtCore.QThread(self.panel)
        worker = GnssBaseSerialDetectWorker(baud=self.panel.gnss_base_baud_spin.value())
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        _connect_gui_signal(worker.detected, self.panel._handle_gnss_base_candidates_detected)
        _connect_gui_signal(worker.failed, self.panel._handle_gnss_base_candidate_detect_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        _connect_gui_signal(
            thread.finished,
            lambda thread=thread: self.panel._clear_gnss_base_detect_worker(thread),
        )
        self.panel.gnss_base_detect_thread = thread
        self.panel.gnss_base_detect_worker = worker
        thread.start()

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
        row = self.panel.gnss_base_candidate_table.currentRow() if hasattr(self.panel, "gnss_base_candidate_table") else -1
        self.panel.gnss_base_apply_candidate_button.setEnabled(0 <= row < len(self.panel._gnss_base_candidate_rows))

    def apply_selected_gnss_base_candidate(self) -> None:
        row = self.panel.gnss_base_candidate_table.currentRow()
        self.panel.apply_gnss_base_candidate(row)

    def apply_gnss_base_candidate(self, row: int) -> bool:
        """Apply one discovered Base receiver without depending on the hidden table selection."""

        if isinstance(row, bool) or not isinstance(row, int):
            self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], "포트 후보 선택이 잘못되었습니다", "bad")
            return False
        if row < 0 or row >= len(self.panel._gnss_base_candidate_rows):
            self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], "포트 후보를 먼저 선택하세요", "warn")
            return False
        entry = self.panel._gnss_base_candidate_rows[row]
        port = str(entry.get("port", "")).strip()
        if not port:
            self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], "선택 후보 port가 비어 있습니다", "bad")
            return False
        self.panel.gnss_base_serial_port_edit.setText(port)
        try:
            baud = int(entry.get("baud", self.panel.gnss_base_baud_spin.value()))
        except (TypeError, ValueError):
            baud = self.panel.gnss_base_baud_spin.value()
        self.panel.gnss_base_baud_spin.setValue(baud)
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], f"선택 포트 적용: {port}", "good")
        return True

    def _handle_gnss_base_serial_connected(self, port: str, baud: int) -> None:
        self.panel.gnss_base_serial_connected = True
        self._sync_rtcm_pipeline_config()
        self.panel.latest_gnss_base_position = None
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], f"connected {port} @ {baud}", "good")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "UBX NAV-SVIN 대기 중", "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_satellites"], "UBX NAV-PVT/NAV-SAT 대기 중", "idle")
        self.panel._set_gnss_base_quality_waiting("NAV-SAT 대기 중")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_position"], "Base WGS84 위치 샘플 대기 중", "idle")
        self.panel._handle_gnss_operation_mode_changed()

    def _handle_gnss_base_serial_stats(self, total_bytes: int, rate_bps: float, age_s: float) -> None:
        age_text = "no data" if age_s < 0.0 else f"age={age_s:.1f}s"
        level = "good" if rate_bps > 0.0 and 0.0 <= age_s < 2.0 else "warn" if rate_bps > 0.0 else "idle"
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["rtcm_rx"],
            f"raw={total_bytes} bytes | {rate_bps:.1f} B/s | {age_text}",
            level,
        )

    def _handle_gnss_base_serial_failed(self, message: str) -> None:
        self.panel.gnss_base_serial_connected = False
        self._sync_rtcm_pipeline_config()
        self._reset_gnss_base_acquisition_state()
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], f"failed: {message}", "bad")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "미연결", "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_satellites"], "대기 중", "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["rtcm_rx"], "없음", "idle")
        self.panel._set_gnss_base_quality_waiting("미연결")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_position"], "Base WGS84 위치 없음", "idle")
        self.panel.gnss_base_survey_start_button.setEnabled(False)
        self.panel.gnss_base_survey_stop_button.setEnabled(False)
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
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_position"], "Base WGS84 위치 없음", "idle")
        self.panel.gnss_base_survey_start_button.setEnabled(False)
        self.panel.gnss_base_survey_stop_button.setEnabled(False)
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
        self.panel.gnss_base_rtcm_stats = self.panel._new_gnss_base_rtcm_stats()
        pipeline = getattr(self.panel, "rtcm_pipeline_worker", None)
        if pipeline is not None:
            self._last_pipeline_drop_count = 0
            self._last_pipeline_blocked_stall_count = 0
            pipeline.reset_requested.emit(self._rtcm_route_config().as_dict())
        self.panel._refresh_gnss_base_rtcm_stats()

    def _reset_gnss_base_acquisition_state(self) -> None:
        """Clear all Base fixed-coordinate acquisition progress.

        A dropped Base serial link means the receiver's own survey-in/TMODE3
        state can no longer be trusted, so a subsequent connect must start
        the acquisition sequence from zero instead of resuming stale samples
        or a stale (possibly no-longer-valid) fixed-coordinate profile.
        """
        self.panel.gnss_coordinate_samples.clear()
        self.panel.latest_gnss_base_position = None
        self.panel.gnss_fixed_base_profile = None
        self.panel.gnss_fixed_base_live_profile = None
        self.panel.gnss_fixed_base_pending_profile = None
        self.panel.gnss_fixed_base_command_sent = False
        self.panel.gnss_fixed_base_reconfigure_expected = None
        self.panel.gnss_fixed_base_reconfigure_ack = False
        self.panel.gnss_fixed_base_reconfigure_readback = False
        self.panel.gnss_fixed_base_reconfigure_rtcm = False
        self.panel.gnss_fixed_base_reconfigure_deadline_s = None
        self.panel.latest_gnss_base_survey_in = None
        self.panel.gnss_base_survey_samples.clear()
        self.panel._gnss_survey_command_sent_generation = 0
        self.panel._gnss_survey_ack_generation = 0
        self.panel._gnss_survey_active_seen_generation = 0
        self.panel.gnss_base_satellite_count = None
        self.panel.gnss_base_satellite_count_source = ""
        self.panel.gnss_base_satellite_quality = None
        self._reset_gnss_base_rtcm_stats()

    def _format_rtcm_message_types(self, message_types: dict[int, int]) -> str:
        if not message_types:
            return "type 대기 중"
        ordered = sorted(message_types.items(), key=lambda item: (-item[1], item[0]))[:8]
        return " | ".join(f"{message_type}:{count}" for message_type, count in ordered)

    def _refresh_gnss_base_rtcm_stats(self) -> None:
        if not hasattr(self.panel, "gnss_base_status_labels"):
            return
        labels = self.panel.gnss_base_status_labels
        if not all(key in labels for key in ("rtcm_receive", "rtcm_messages", "rtcm_injection_stats", "rtcm_bandwidth")):
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
            receive_level = "good" if age_s is not None and age_s <= 2.0 else "warn" if age_s is not None and age_s <= 5.0 else "bad"
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
                f"{label} frames={side_frames} bytes={side_bytes} "
                f"fail={side_fail} blocked_stall={side_blocked_stall}"
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
                "good"
                if frames and total_fail == 0 and total_blocked_stall == 0
                else "warn"
                if frames
                else "idle"
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
        self.panel._set_health_label(labels["rtcm_messages"], self.panel._format_rtcm_message_types(message_type_counts), message_level)
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
        constellation_text = " / ".join(
            f"{name} {count}" for name, count in sorted(constellation_items)
        ) or "--"
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
        constellation_level = "good" if len(constellation_items) >= 3 else "warn" if len(constellation_items) >= 2 else "bad"
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
        return normalize_drone_rtcm_source(
            getattr(self.panel, "drone_telemetry_rtcm_source", DRONE_RTCM_SOURCE_BASE)
        )

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
        self,
        *,
        source: str = DRONE_RTCM_SOURCE_BASE,
        include_rover_targets: bool = True,
    ) -> tuple[str, ...]:
        config = self.panel.gnss_active_operation_config
        if not include_rover_targets:
            targets: tuple[str, ...] = ()
        elif config is not None:
            targets = tuple(
                side
                for side, enabled in (("left", config.routing.left_enabled), ("right", config.routing.right_enabled))
                if enabled
            )
        else:
            target = str(self.panel.gnss_base_rtcm_target_combo.currentData() or "disabled")
            if target == "both":
                targets = ("left", "right")
            elif target in {"left", "right"}:
                targets = (target,)
            else:
                targets = ()
        # Drone Telemetry is appended only when the GNSS routing target is
        # enabled, this frame came from the source selected for the active
        # Drone link, forwarding is enabled, and the MAVLink endpoint is open.
        return tuple(dict.fromkeys((*targets, *self._drone_rtcm_target_sides(source))))

    def _record_gnss_base_rtcm_frame(self, payload: bytes) -> None:
        stats = self.panel.gnss_base_rtcm_stats
        stats["frames"] = int(stats.get("frames") or 0) + 1
        stats["bytes"] = int(stats.get("bytes") or 0) + len(payload)
        stats["max_frame_bytes"] = max(int(stats.get("max_frame_bytes") or 0), len(payload))
        stats["last_frame_s"] = time.monotonic()
        message_type = parse_rtcm3_message_type(payload)
        if message_type is not None:
            message_types = stats.get("message_types")
            if not isinstance(message_types, dict):
                message_types = {}
                stats["message_types"] = message_types
            message_types[message_type] = int(message_types.get(message_type, 0)) + 1

    def _append_rtcm_package_evidence(
        self,
        payload: bytes,
        *,
        source: str,
        record_kind: str,
    ) -> None:
        """Append exact RTCM bytes and an offset/hash index to the open session."""

        if not payload:
            return
        context = self._rtcm_evidence_context()
        if not context.active:
            return
        last_sequence, source_sequences = load_rtcm_evidence_high_water(context)
        source_key = (str(source), str(record_kind))
        evidence_record = RtcmEvidenceRecord(
            sequence=last_sequence + 1,
            source_sequence=int(source_sequences.get(source_key, 0)) + 1,
            source=str(source),
            record_kind=str(record_kind),
            payload=payload,
            receive_utc_ns=time.time_ns(),
            receive_monotonic_ns=time.monotonic_ns(),
            message_type=(
                parse_rtcm3_message_type(payload) if record_kind == "rtcm_frame" else None
            ),
        )
        record = append_rtcm_evidence(context, evidence_record)
        if record is None:
            return
        activator = getattr(self.panel, "activate_measurement_package_source", None)
        if callable(activator):
            activator(
                "gnss.rtcm",
                required_roles=("corrections/rtcm/raw",),
                metadata={"activated_utc_ns": record["receive_utc_ns"], "source": str(source)},
            )

    def _record_gnss_base_rtcm_injection(
        self,
        side: str,
        *,
        payload_bytes: int,
        packet_count: int = 0,
        failed: bool = False,
        blocked_stall: bool = False,
    ) -> None:
        injection = self.panel.gnss_base_rtcm_stats.get("injection")
        if not isinstance(injection, dict):
            return
        side_stats = injection.get(side)
        if not isinstance(side_stats, dict):
            return
        if failed:
            side_stats["fail"] = int(side_stats.get("fail") or 0) + 1
            return
        if blocked_stall:
            side_stats["blocked_stall"] = int(side_stats.get("blocked_stall") or 0) + 1
            return
        side_stats["frames"] = int(side_stats.get("frames") or 0) + 1
        side_stats["bytes"] = int(side_stats.get("bytes") or 0) + int(payload_bytes)
        side_stats["packets"] = int(side_stats.get("packets") or 0) + int(packet_count)
        side_stats["last_delivery_s"] = time.monotonic()

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
            self.panel.gnss_operation_manager.state == GnssOperationState.BASE_RECONFIGURING
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
        worker.request.emit(tuple(target_sides), payload)
        if ui_due:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["rtcm_injection"],
                f"RTCM {len(payload)} bytes 대기 중 -> "
                f"{', '.join(self.panel._px4_mag_camera_label(side) for side in target_sides)}",
                "warn",
            )

    def _ensure_pixhawk_rtcm_worker(self) -> PixhawkRtcmInjectionWorker | None:
        subscriber = self.panel.mavlink_pixhawk
        if subscriber is None:
            return None
        existing_thread = self.panel.pixhawk_rtcm_thread
        existing_worker = self.panel.pixhawk_rtcm_worker
        if (
            existing_thread is not None
            and existing_worker is not None
            and existing_thread.isRunning()
            and existing_worker.subscriber is subscriber
        ):
            self._wire_rtcm_pipeline_injection_worker(existing_worker)
            return existing_worker
        if existing_thread is not None or existing_worker is not None:
            restart_token = (existing_thread, subscriber)
            if (
                existing_thread is not None
                and existing_thread.isRunning()
                and getattr(
                    self.panel,
                    "_pixhawk_rtcm_restart_pending",
                    None,
                ) != restart_token
            ):
                self.panel._pixhawk_rtcm_restart_pending = restart_token
                _connect_gui_signal(
                    existing_thread.finished,
                    lambda old_thread=existing_thread, desired=subscriber: self._restart_pixhawk_rtcm_worker_after_retire(
                        old_thread,
                        desired,
                    ),
                )
            # QThread.wait() here used to freeze the GUI for up to one second
            # on every subscriber replacement.  Request retirement and let
            # the finished signal resume construction instead.
            if not self._retire_pixhawk_rtcm_worker(wait=False):
                return None
            if getattr(self.panel, "_pixhawk_rtcm_restart_pending", None) == restart_token:
                self.panel._pixhawk_rtcm_restart_pending = None
        self._ensure_rtcm_pipeline()
        thread = QtCore.QThread(self.panel)
        worker = PixhawkRtcmInjectionWorker(subscriber)
        worker.moveToThread(thread)
        worker.request.connect(worker.enqueue, QtCore.Qt.QueuedConnection)
        thread.finished.connect(thread.deleteLater)
        _connect_gui_signal(
            thread.finished,
            lambda thread=thread, worker=worker: self._clear_pixhawk_rtcm_worker(
                thread,
                worker,
            ),
        )
        self.panel.pixhawk_rtcm_thread = thread
        self.panel.pixhawk_rtcm_worker = worker
        self._wire_rtcm_pipeline_injection_worker(worker)
        thread.start()
        return worker

    def _restart_pixhawk_rtcm_worker_after_retire(
        self,
        old_thread: object,
        desired_subscriber: object,
    ) -> None:
        token = (old_thread, desired_subscriber)
        if getattr(self.panel, "_pixhawk_rtcm_restart_pending", None) != token:
            return
        self.panel._pixhawk_rtcm_restart_pending = None
        if getattr(self.panel, "mavlink_pixhawk", None) is desired_subscriber:
            self._ensure_pixhawk_rtcm_worker()

    def _retire_pixhawk_rtcm_worker(
        self,
        *,
        expected_subscriber: object | None = None,
        wait: bool,
    ) -> bool:
        thread = getattr(self.panel, "pixhawk_rtcm_thread", None)
        worker = getattr(self.panel, "pixhawk_rtcm_worker", None)
        if worker is not None and (
            expected_subscriber is None or worker.subscriber is expected_subscriber
        ):
            try:
                worker.clear_pending()
            except Exception:
                pass
        elif expected_subscriber is not None:
            return True
        if getattr(self.panel, "_rtcm_pipeline_injection_worker", None) is worker:
            self._wire_rtcm_pipeline_injection_worker(None)
        if thread is not None and thread.isRunning():
            thread.quit()
            if wait and not thread.wait(1000):
                return False
        if thread is None or not thread.isRunning():
            self._clear_pixhawk_rtcm_worker(thread, worker)
            return True
        return False

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
            self.panel._log(f"[GNSS] RTCM 오래된 대기 프레임 {dropped}개를 폐기했습니다.")
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
        current_thread = getattr(self.panel, "pixhawk_rtcm_thread", None)
        current_worker = getattr(self.panel, "pixhawk_rtcm_worker", None)
        if thread is not None and current_thread is not thread:
            return
        if worker is not None and current_worker is not worker:
            return
        if getattr(self.panel, "_rtcm_pipeline_injection_worker", None) is current_worker:
            self._wire_rtcm_pipeline_injection_worker(None)
        self.panel.pixhawk_rtcm_thread = None
        self.panel.pixhawk_rtcm_worker = None
        self._sync_rtcm_pipeline_config()
        for callback_name in (
            "_refresh_gnss_base_rtcm_stats",
            "_maybe_activate_gnss_fixed_base",
            "_refresh_gnss_operation_readiness",
        ):
            callback = getattr(self.panel, callback_name, None)
            if callable(callback):
                callback()

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
        self.panel.latest_gnss_base_survey_in = dict(state)
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
        generation = int(getattr(self.panel, "_gnss_survey_generation", 0) or 0)
        generation_ready = bool(
            generation > 0
            and int(
                getattr(self.panel, "_gnss_survey_command_sent_generation", 0) or 0
            )
            == generation
            and int(getattr(self.panel, "_gnss_survey_ack_generation", 0) or 0)
            == generation
        )
        if active and not valid and generation_ready:
            self.panel._gnss_survey_active_seen_generation = generation
        requested_duration_s = int(
            getattr(self.panel, "_gnss_survey_requested_duration_s", 0) or 0
        )
        requested_accuracy_m = float(
            getattr(self.panel, "_gnss_survey_requested_accuracy_m", 0.0) or 0.0
        )
        completion_correlated = bool(
            generation_ready
            and int(
                getattr(self.panel, "_gnss_survey_active_seen_generation", 0) or 0
            )
            == generation
            and duration_s >= requested_duration_s
            and requested_accuracy_m > 0.0
            and accuracy_m <= requested_accuracy_m
        )
        if (
            valid
            and completion_correlated
            and self.panel._gnss_active_operation_config().operation_mode
            == GnssOperationMode.LOCAL_SURVEY_IN
        ):
            if self.panel.gnss_operation_manager.state == GnssOperationState.BASE_SURVEYING:
                self.panel._gnss_transition(GnssOperationState.LOCAL_RTCM_ACTIVE, "현장 Survey-in valid; RTCM rover 라우팅")
            self.panel._refresh_gnss_operation_readiness()
        elif valid and self.panel.gnss_operation_manager.state == GnssOperationState.BASE_SURVEYING:
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["survey_in"],
                "valid NAV-SVIN ignored until current CFG ACK, active observation, and requested limits are confirmed",
                "warn",
            )

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
        self.panel.gnss_base_survey_observation_plot.add_point(TelemetryHistoryPoint(now_s, "observations", float(observations)))
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
