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

from PyQt5 import QtCore, QtWidgets

from Workstation.gnss_runtime_workers import (
    GnssBaseSerialDetectWorker,
    GnssBaseSerialWorker,
    NtripNetworkWorker,
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


GNSS_RUNTIME_CONTROLLER_METHODS = [
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
    def __init__(self, panel):
        self.panel = panel

    def _refresh_gnss_base_tab(self) -> None:
        if not hasattr(self.panel, "gnss_base_status_labels"):
            return
        labels = self.panel.gnss_base_status_labels
        now_s = time.monotonic()
        for side, key in (("left", "left_rover"), ("right", "right_rover")):
            gps = self.panel.latest_gps.get(side)
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

        This is polled at connect time and roughly once per
        ``on_gga_interval_s`` while streaming (see ``NtripClient``), so a
        genuinely stuck ``None`` state logs at that same modest rate --
        frequent enough to see live, never a per-recv()-loop spam risk.
        """

        policy = self.panel._gnss_active_operation_config().network.gga_policy
        if policy == GgaPolicy.BASE_GNSS:
            sample = getattr(self.panel, "latest_gnss_base_position", None)
            if sample is None:
                self.panel._log("[GNSS][NTRIP] GGA skipped: no Base GNSS sample received yet (policy=BASE_GNSS)")
                return None
            age_s = max(0.0, time.monotonic() - sample.timestamp_s)
            if age_s > 15.0:
                self.panel._log(
                    f"[GNSS][NTRIP] GGA skipped: Base GNSS sample is stale "
                    f"(age={age_s:.1f}s > 15.0s, policy=BASE_GNSS)"
                )
                return None
            return gga_from_coordinate_sample(sample)
        left = self.panel.latest_gps.get("left")
        right = self.panel.latest_gps.get("right")
        positions = [position for position in (left, right) if position is not None and position.has_position]
        if policy == GgaPolicy.MANUAL:
            self.panel._log("[GNSS][NTRIP] GGA skipped: GGA policy is MANUAL; GGA is intentionally never sent")
            return None
        if not positions:
            def _describe(position: object) -> str:
                if position is None:
                    return "no message received"
                if not getattr(position, "has_position", False):
                    return "message received but lat/lon/alt incomplete"
                return "ok"

            self.panel._log(
                "[GNSS][NTRIP] GGA skipped: no left/right GNSS position available "
                f"(policy={policy.value}, left={_describe(left)}, right={_describe(right)})"
            )
            return None
        if policy == GgaPolicy.LEFT_GNSS or len(positions) == 1:
            return gga_from_position(positions[0])
        if policy == GgaPolicy.PER_ROVER_SESSION:
            return gga_from_position(left or positions[0])
        # SYSTEM_CENTER: average WGS84 geodetic values for the two antenna hints.
        lat = sum(float(position.latitude_deg) for position in positions) / len(positions)
        lon = sum(float(position.longitude_deg) for position in positions) / len(positions)
        alt = sum(float(position.altitude_m) for position in positions) / len(positions)
        return format_nmea_gga(lat, lon, alt, fix_quality=1, satellites=12, hdop=1.0)

    def _resolve_ntrip_credentials(self, network) -> "NtripCredentialResolution":
        """Resolve the username/password to authenticate with, in priority order.

        1. The password currently typed into the UI (this run's session).
        2. The OS secure credential store (keyring), keyed by
           host/mountpoint/username.
        3. The legacy ``username_env``/``password_env`` indirection, read
           from the OS environment.
        4. None of the above -> raise with a specific, non-guessing reason.

        The username itself is plain config (``network.username``, or the
        legacy env var's resolved value); only the password is ever secret.
        """

        username = str(network.username).strip()
        session_password = ""
        if hasattr(self.panel, "gnss_ntrip_password_edit"):
            session_password = self.panel.gnss_ntrip_password_edit.text()

        if username and session_password:
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

    def _reject_ntrip_start(self, message: str, tone: str) -> None:
        """Every guard below routes through here so a rejected Start NTRIP
        click always leaves a line in the visible log, not only the small
        "NTRIP" status label in the GNSS Base status grid -- a click that
        only updates that label is easy to miss and reads as "did nothing".
        """

        self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], message, tone)
        self.panel._log(f"[GNSS] NTRIP 시작 거부: {message}")

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
        if config.operation_mode != GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE and self.panel.mavlink_pixhawk is None:
            self._reject_ntrip_start("RTCM을 받을 MAVLink Pixhawk가 없습니다", "bad")
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
        self._maybe_persist_ntrip_credential(config.network, credential)
        self.panel._refresh_ntrip_credential_status()
        thread = QtCore.QThread(self.panel)
        worker = NtripNetworkWorker(
            config=config.network,
            gga_provider=self.panel._gnss_gga_provider,
            username=credential.username,
            password=credential.password,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.connected.connect(self.panel._handle_gnss_network_connected)
        worker.stats_changed.connect(self.panel._handle_gnss_network_stats)
        worker.chunk_received.connect(self.panel._handle_gnss_network_chunk)
        worker.reconnecting.connect(self.panel._handle_gnss_network_reconnecting)
        worker.failed.connect(self.panel._handle_gnss_network_failed)
        worker.diagnostic.connect(self.panel._handle_gnss_network_diagnostic)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_gnss_network_worker(thread))
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

    def set_gnss_ntrip_session_password(self, password: str) -> None:
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

    def _handle_gnss_network_connected(self, status_line: str, stats: object) -> None:
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
        self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], f"{status_line} | RTCM 대기", "good")

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
        mode = self.panel._gnss_active_operation_config().operation_mode
        frames = self.panel.gnss_network_rtcm_parser.feed(payload)
        if mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE:
            worker = self.panel.gnss_base_serial_worker
            if worker is None or not self.panel.gnss_base_serial_connected:
                self.panel._set_health_label(
                    self.panel.gnss_base_status_labels["network"],
                    "NTRIP 수신 중이나 Base serial이 끊겼습니다",
                    "bad",
                )
            else:
                worker.send_bytes("NTRIP RTCM -> fixed base", payload)
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
            self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], f"NTRIP raw {len(payload)} bytes | RTCM frame 대기", "warn")
            return
        for frame in frames:
            self.panel._handle_gnss_base_rtcm_frame(frame, source=DRONE_RTCM_SOURCE_NTRIP)
        self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], f"NTRIP RTCM {len(frames)} frame 수신", "good")

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
        self.panel.gnss_network_phase = "ERROR"
        self.panel.gnss_network_last_error = str(message)
        text = str(message)
        # 401 means the caster rejected the credentials; it is deliberately
        # not asserted here whether the username or the password was wrong.
        if "401" in text or "unauthorized" in text.lower():
            display = f"NTRIP 인증 거부 (username/password 확인 필요): {text}"
        else:
            display = f"NTRIP 실패: {text}"
        self.panel._set_health_label(self.panel.gnss_base_status_labels["network"], display, "bad")
        # A worker-thread failure after credentials already resolved is the
        # one NTRIP fault that used to leave zero trace in the log -- only
        # this same small status label, which is exactly why a rejected
        # password after Start NTRIP "does nothing" a second time.
        self.panel._log(f"[GNSS] {display}")
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
            if getattr(self.panel, "gnss_network_phase", "") != "ERROR":
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
        if config.operation_mode != GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE:
            return
        self.panel._record_gnss_base_position_row(sample)
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
        if self.panel.gnss_operation_manager.state != GnssOperationState.BASE_RECONFIGURING:
            return
        raw_payload = bytes(payload)
        ack = parse_ubx_ack(int(message_class), int(message_id), raw_payload)
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
        routes: dict[str, RoverCorrectionStatus] = {}
        rover_samples: dict[str, RoverReadinessInput] = {}
        for side in target_sides:
            side_stats = injection_map.get(side)
            if not isinstance(side_stats, dict):
                side_stats = {}
            if isinstance(connections, dict):
                endpoint_live = side in connections and side in heartbeat_sides
                endpoint_identity = f"mavlink:{side}" if endpoint_live else ""
            else:
                # Test adapters may not expose the production subscriber
                # internals; their presence still represents a route endpoint.
                endpoint_identity = "" if mavlink is None else f"mavlink:{side}"
            routes[side] = RoverCorrectionStatus(
                target=side,
                enabled=True,
                endpoint_identity=endpoint_identity,
                bytes_sent=int(side_stats.get("bytes") or 0),
                packets_sent=int(side_stats.get("packets") or 0),
                last_delivery_monotonic_s=side_stats.get("last_delivery_s"),
                fault_reason=("injection failed" if int(side_stats.get("fail") or 0) > 0 else ""),
            )
        for side in ("left", "right", "drone"):
            gps = self.panel.latest_gps.get(side)
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
        worker.connected.connect(self.panel._handle_gnss_base_serial_connected)
        worker.stats_changed.connect(self.panel._handle_gnss_base_serial_stats)
        worker.survey_in_changed.connect(self.panel._handle_gnss_base_survey_in)
        worker.satellite_count_changed.connect(self.panel._handle_gnss_base_satellite_count)
        worker.satellite_quality_changed.connect(self.panel._handle_gnss_base_satellite_quality)
        worker.position_changed.connect(self.panel._handle_gnss_base_position)
        worker.ubx_message_received.connect(self.panel._handle_gnss_base_ubx_message)
        worker.rtcm_frame_received.connect(self.panel._handle_gnss_base_rtcm_frame)
        worker.command_sent.connect(self.panel._handle_gnss_base_command_sent)
        worker.failed.connect(self.panel._handle_gnss_base_serial_failed)
        worker.disconnected.connect(self.panel._handle_gnss_base_serial_disconnected)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_gnss_base_serial_worker(thread))
        self.panel.gnss_base_serial_thread = thread
        self.panel.gnss_base_serial_worker = worker
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
        worker.detected.connect(self.panel._handle_gnss_base_candidates_detected)
        worker.failed.connect(self.panel._handle_gnss_base_candidate_detect_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_gnss_base_detect_worker(thread))
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
        self.panel.latest_gnss_base_position = None
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], f"failed: {message}", "bad")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "미연결", "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_satellites"], "대기 중", "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["rtcm_rx"], "없음", "idle")
        self.panel._set_gnss_base_quality_waiting("미연결")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_position"], "Base WGS84 위치 없음", "idle")
        self.panel._reset_gnss_base_rtcm_stats()
        self.panel.gnss_base_survey_start_button.setEnabled(False)
        self.panel.gnss_base_survey_stop_button.setEnabled(False)
        if self.panel.gnss_operation_gate_active:
            self.panel._gnss_transition(GnssOperationState.DEGRADED, f"Base serial failure: {message}")

    def _handle_gnss_base_serial_disconnected(self, message: str) -> None:
        self.panel.gnss_base_serial_connected = False
        self.panel.latest_gnss_base_position = None
        self.panel._set_health_label(self.panel.gnss_base_status_labels["connection"], message, "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["survey_in"], "미연결", "idle")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_satellites"], "대기 중", "idle")
        self.panel._set_gnss_base_quality_waiting("미연결")
        self.panel._set_health_label(self.panel.gnss_base_status_labels["base_position"], "Base WGS84 위치 없음", "idle")
        self.panel._reset_gnss_base_rtcm_stats()
        self.panel.gnss_base_survey_start_button.setEnabled(False)
        self.panel.gnss_base_survey_stop_button.setEnabled(False)
        if self.panel.gnss_operation_gate_active:
            self.panel._gnss_transition(GnssOperationState.DEGRADED, f"Base serial disconnected: {message}")

    def _handle_gnss_base_satellite_count(self, count: int, source: str) -> None:
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
        self.panel._refresh_gnss_base_rtcm_stats()

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
        for side, label in (("left", "L"), ("right", "R"), ("drone", "D")):
            side_stats = injection.get(side) if isinstance(injection, dict) else None
            if not isinstance(side_stats, dict):
                side_stats = {}
            side_frames = int(side_stats.get("frames") or 0)
            side_bytes = int(side_stats.get("bytes") or 0)
            side_packets = int(side_stats.get("packets") or 0)
            side_fail = int(side_stats.get("fail") or 0)
            total_packets += side_packets
            total_fail += side_fail
            injection_parts.append(f"{label} frames={side_frames} bytes={side_bytes} fail={side_fail}")
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
            injection_level = "good" if frames and total_fail == 0 else "warn" if frames else "idle"
        avg_frame = (total_bytes / frames) if frames else 0.0
        bandwidth_text = (
            "packet 대기 중"
            if frames <= 0
            else f"MAVLink packets={total_packets} | avg frame={avg_frame:.0f}B | max frame={int(stats.get('max_frame_bytes') or 0)}B"
        )
        bandwidth_level = "good" if frames and total_fail == 0 else "warn" if frames else "idle"
        self.panel._set_health_label(labels["rtcm_receive"], receive_text, receive_level)
        self.panel._set_health_label(labels["rtcm_messages"], self.panel._format_rtcm_message_types(message_type_counts), message_level)
        self.panel._set_health_label(labels["rtcm_injection_stats"], injection_text, injection_level)
        self.panel._set_health_label(labels["rtcm_bandwidth"], bandwidth_text, bandwidth_level)

    def _handle_gnss_base_satellite_quality(self, quality: object) -> None:
        if not isinstance(quality, dict) or not hasattr(self.panel, "gnss_base_quality_labels"):
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

    def _record_gnss_base_rtcm_injection(
        self,
        side: str,
        *,
        payload_bytes: int,
        packet_count: int = 0,
        failed: bool = False,
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
            self.panel._set_health_label(self.panel.gnss_base_status_labels["rtcm_injection"], "비활성", "idle")
            self.panel._refresh_gnss_base_rtcm_stats()
            self.panel._refresh_gnss_operation_readiness()
            return
        if self.panel.mavlink_pixhawk is None:
            for side in target_sides:
                self.panel._record_gnss_base_rtcm_injection(side, payload_bytes=len(payload), failed=True)
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
            self.panel._set_health_label(
                self.panel.gnss_base_status_labels["rtcm_injection"],
                f"RTCM 주입 대기 실패: MAVLink worker 없음 ({len(payload)} bytes)",
                "bad",
            )
            self.panel._refresh_gnss_base_rtcm_stats()
            self.panel._refresh_gnss_operation_readiness()
            return
        worker.request.emit(tuple(target_sides), payload)
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
        if self.panel.pixhawk_rtcm_thread is not None and self.panel.pixhawk_rtcm_worker is not None:
            return self.panel.pixhawk_rtcm_worker
        thread = QtCore.QThread(self.panel)
        worker = PixhawkRtcmInjectionWorker(subscriber)
        worker.moveToThread(thread)
        worker.request.connect(worker.enqueue, QtCore.Qt.QueuedConnection)
        worker.injected.connect(self.panel._apply_pixhawk_rtcm_result)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_pixhawk_rtcm_worker)
        self.panel.pixhawk_rtcm_thread = thread
        self.panel.pixhawk_rtcm_worker = worker
        thread.start()
        return worker

    def _apply_pixhawk_rtcm_result(self, result: object) -> None:
        data = dict(result or {})
        payload_bytes = int(data.get("payload_bytes") or 0)
        sent = tuple(data.get("sent") or ())
        failed = tuple(data.get("failed") or ())
        dropped = int(data.get("dropped") or 0)
        sent_labels: list[str] = []
        failed_labels: list[str] = []
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
        if dropped:
            self.panel._log(f"[GNSS] RTCM 오래된 대기 프레임 {dropped}개를 폐기했습니다.")
        if sent_labels and not failed_labels:
            level = "good"
            text = f"RTCM {payload_bytes} bytes -> {', '.join(sent_labels)}"
        elif sent_labels:
            level = "warn"
            text = f"RTCM 일부 주입: {', '.join(sent_labels)} | 실패 {', '.join(failed_labels)}"
        elif failed_labels:
            level = "bad"
            text = f"RTCM 주입 실패: {', '.join(failed_labels)}"
        else:
            return
        self.panel._set_health_label(self.panel.gnss_base_status_labels["rtcm_injection"], text, level)
        self.panel._refresh_gnss_base_rtcm_stats()
        self.panel._refresh_gnss_operation_readiness()

    def _clear_pixhawk_rtcm_worker(self) -> None:
        self.panel.pixhawk_rtcm_thread = None
        self.panel.pixhawk_rtcm_worker = None
        self.panel._refresh_gnss_base_rtcm_stats()
        self.panel._maybe_activate_gnss_fixed_base()
        self.panel._refresh_gnss_operation_readiness()

    def _handle_gnss_base_command_sent(self, label: str, byte_count: int) -> None:
        self.panel._set_health_label(
            self.panel.gnss_base_status_labels["connection"],
            f"connected | sent {label} ({byte_count} bytes)",
            "good",
        )
        if label == "CFG-TMODE3 survey-in start":
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
        if valid and self.panel._gnss_active_operation_config().operation_mode == GnssOperationMode.LOCAL_SURVEY_IN:
            if self.panel.gnss_operation_manager.state == GnssOperationState.BASE_SURVEYING:
                self.panel._gnss_transition(GnssOperationState.LOCAL_RTCM_ACTIVE, "현장 Survey-in valid; RTCM rover 라우팅")
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
