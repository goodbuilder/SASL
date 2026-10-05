"""Workstation Pixhawk/GNSS sensor presentation controller.

Telemetry values retain PX4/MAVLink producer contracts. GPS local positions
remain NED where declared, attitude remains body-FRD to earth-NED, and this
controller only formats and presents sensor health.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import replace

from PyQt5 import QtWidgets

from Workstation.telemetry_plots import AttitudeHistoryPoint, TelemetryHistoryPoint
from Workstation.workstation_core.pixhawk_attitude import CameraExtrinsicState
from Workstation.workstation_core.px4_mag_manager import RM3100_IDS
from Workstation.workstation_core.rtk_geometry import GpsPositionState
from Workstation.workstation_core.theme import (
    ERROR, ERROR_TEXT, PANEL_ALT_BG, PANEL_BORDER,
    SUCCESS, SUCCESS_TEXT, TEXT_MUTED,
    WARNING, WARNING_TEXT,
)


def angle_diff_deg(a_deg: float, b_deg: float) -> float:
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


SENSOR_PRESENTATION_METHODS = [
    "_update_extrinsic",
    "_update_gps",
    "_update_mag_sensor",
    "_update_gps_sensor",
    "_refresh_f9p_compare_label",
    "_update_gnss_relative",
    "_freshest_sensor_slots",
    "_set_health_label",
    "_set_label_text_if_changed",
    "_telemetry_source_label",
    "_gps_position_summary_text",
    "_gps_fix_level",
    "_gps_sensor_status_text",
    "_vector_angle_deg",
    "_fix_type_label",
    "_refresh_pixhawk_sensor_health",
    "_refresh_one_pixhawk_sensor_health",
    "_update_relative_attitude_status",
    "_attitude_delta_level"
]


# Consecutive GPS packets carrying an identical position before the link is
# reported as frozen. A stationary RTK receiver can repeat a 1e-7 deg position
# for a few epochs; a flying vehicle cannot.
GNSS_FROZEN_POSITION_PACKETS = 10
GNSS_FROZEN_LOG_INTERVAL_S = 30.0


class SensorPresentationController:
    def __init__(self, panel):
        self.panel = panel
        self._last_camera_parameters_refresh_s = 0.0
        # camera -> (position, utc, repeat count, last log monotonic, first seen app time)
        self._gnss_frozen_watch: dict[str, tuple[tuple[object, ...], object, int, float, float]] = {}

    def _observe_gnss_frozen_position(self, camera: str, gps: GpsPositionState) -> None:
        """Log when a link keeps delivering the same GNSS position.

        Every GPS_RAW_INT rebuilds ``latest_gps[camera]`` from the packet and
        stamps the receive time, so an age that keeps resetting while the
        position never moves means the producer is repeating itself, not the
        Workstation. ``gps_utc_us`` separates the two producer-side causes:
        a frozen receiver output still advances UTC, whereas an autopilot
        re-sending one stale fix repeats the same UTC as well.
        """

        frozen_store = getattr(self.panel, "_gnss_position_frozen", None)
        if not isinstance(frozen_store, dict):
            frozen_store = {}
            setattr(self.panel, "_gnss_position_frozen", frozen_store)
        position = (gps.latitude_deg, gps.longitude_deg, gps.altitude_m)
        if any(value is None for value in position):
            self._gnss_frozen_watch.pop(camera, None)
            frozen_store.pop(camera, None)
            return
        utc = gps.gps_utc_us
        previous = self._gnss_frozen_watch.get(camera)
        now_s = time.monotonic()
        if previous is None or previous[0] != position:
            self._gnss_frozen_watch[camera] = (position, utc, 1, float("-inf"), float(gps.timestamp_s))
            frozen_store.pop(camera, None)
            return
        _position, previous_utc, count, last_log_s, first_seen_s = previous
        count += 1
        utc_frozen = utc is not None and previous_utc == utc
        raw = getattr(gps, "raw", None)
        raw_vel = getattr(raw, "vel", None) if raw is not None else None
        ground_speed_m_s = None if raw_vel in (None, 65535) else float(raw_vel) * 0.01
        if count >= GNSS_FROZEN_POSITION_PACKETS:
            # Operator-visible state: the Telemetry position table and the
            # Drone geometry status read this instead of the log. fix/sats/
            # speed tell a receiver hold (many sats, zero speed, DGPS/RTK fix)
            # apart from a lost fix that keeps its last position.
            frozen_store[camera] = {
                "packets": count,
                "utc_frozen": utc_frozen,
                "since_s": first_seen_s,
                "fix_type": gps.fix_type,
                "satellites": gps.satellites_used,
                "ground_speed_m_s": ground_speed_m_s,
            }
        if count >= GNSS_FROZEN_POSITION_PACKETS and now_s - last_log_s >= GNSS_FROZEN_LOG_INTERVAL_S:
            last_log_s = now_s
            logger = getattr(self.panel, "_log", None)
            if callable(logger):
                label = {"left": "P1", "right": "P2", "drone": "Drone"}.get(
                    camera.split("_", 1)[0], camera
                )
                cause = (
                    "GPS UTC도 정지: autopilot이 같은 fix를 재전송 중 (GPS driver/uORB 확인)"
                    if utc_frozen
                    else "GPS UTC는 진행: 수신기 위치 출력 정지 (static hold/fix 상실 확인)"
                )
                speed_text = "--" if ground_speed_m_s is None else f"{ground_speed_m_s:.2f}m/s"
                logger(
                    f"[픽스호크] {label} GNSS 위치 고정 감지: 연속 {count}개 패킷 동일 "
                    f"lat/lon/alt={position[0]:.7f}/{position[1]:.7f}/{position[2]:.3f} · "
                    f"fix={gps.fix_type} sats={gps.satellites_used} vel={speed_text} · {cause}"
                )
        self._gnss_frozen_watch[camera] = (position, utc, count, last_log_s, first_seen_s)

    def _update_extrinsic(self, camera: str, extrinsic: CameraExtrinsicState) -> None:
        # Drone Telemetry shares the receive subscriber but is a position/
        # correction endpoint, not a third stereo camera. Do not feed its
        # ATTITUDE packets into the two-camera attitude/baseline surfaces.
        if camera not in self.panel.attitude_buffers:
            return
        publisher = getattr(self.panel, "measurement_source_transition", None)
        if callable(publisher):
            publisher(
                f"pixhawk.{camera}",
                "LIVE",
                "fresh expected-system ATTITUDE received",
            )
        age_ms = max(0.0, time.monotonic() - extrinsic.timestamp_s) * 1000.0
        px4_text = f", px4_t={extrinsic.px4_timestamp_s:.3f}" if extrinsic.px4_timestamp_s is not None else ""
        self.panel.latest_extrinsics[camera] = extrinsic
        self.panel.attitude_monitor.update(camera, extrinsic)
        self.panel._record_external_timeline_row(
            camera,
            "pixhawk_attitude",
            {
                "namespace": extrinsic.namespace,
                "topic": extrinsic.topic,
                "mavlink_source_system": (
                    "" if extrinsic.mavlink_source_system is None else extrinsic.mavlink_source_system
                ),
                "mavlink_source_component": (
                    "" if extrinsic.mavlink_source_component is None else extrinsic.mavlink_source_component
                ),
                "mavlink_sequence": "" if extrinsic.mavlink_sequence is None else extrinsic.mavlink_sequence,
                "mavlink_message_id": (
                    "" if extrinsic.mavlink_message_id is None else extrinsic.mavlink_message_id
                ),
                "sample_app_timestamp_s": extrinsic.timestamp_s,
                "sample_px4_timestamp_s": self.panel._fmt_record_float(extrinsic.px4_timestamp_s, 6),
                "px4_timestamp_us": (
                    "" if extrinsic.px4_timestamp_us is None else extrinsic.px4_timestamp_us
                ),
                "px4_timestamp_sample_us": (
                    ""
                    if extrinsic.px4_timestamp_sample_us is None
                    else extrinsic.px4_timestamp_sample_us
                ),
                "roll_deg": self.panel._fmt_record_float(extrinsic.roll_deg, 6),
                "pitch_deg": self.panel._fmt_record_float(extrinsic.pitch_deg, 6),
                "yaw_deg": self.panel._fmt_record_float(extrinsic.yaw_deg, 6),
                "attitude_rollspeed_rad_s": self.panel._fmt_record_float(extrinsic.rollspeed_rad_s, 9),
                "attitude_pitchspeed_rad_s": self.panel._fmt_record_float(extrinsic.pitchspeed_rad_s, 9),
                "attitude_yawspeed_rad_s": self.panel._fmt_record_float(extrinsic.yawspeed_rad_s, 9),
                "attitude_time_boot_ms": "" if extrinsic.px4_timestamp_us is None else extrinsic.px4_timestamp_us // 1000,
                "q": json.dumps(list(extrinsic.q), separators=(",", ":")),
                "q_w": self.panel._fmt_record_float(extrinsic.q[0], 9),
                "q_x": self.panel._fmt_record_float(extrinsic.q[1], 9),
                "q_y": self.panel._fmt_record_float(extrinsic.q[2], 9),
                "q_z": self.panel._fmt_record_float(extrinsic.q[3], 9),
                "orientation_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
                "delta_q_reset_wxyz": (
                    ""
                    if extrinsic.delta_q_reset is None
                    else json.dumps(list(extrinsic.delta_q_reset), separators=(",", ":"))
                ),
                "quat_reset_counter": (
                    "" if extrinsic.quat_reset_counter is None else extrinsic.quat_reset_counter
                ),
                "age_s": self.panel._fmt_record_float(max(0.0, time.monotonic() - extrinsic.timestamp_s), 6),
            },
        )
        self.panel.attitude_buffers[camera].add(extrinsic)
        self.panel.pixhawk_attitude_times[camera].append(time.monotonic())
        if hasattr(self.panel, "signal_attitude_labels"):
            source = self.panel._telemetry_source_label(extrinsic.topic)
            self.panel.signal_attitude_labels[camera].setText(
                f"Attitude: {source} | roll={extrinsic.roll_deg:.2f} pitch={extrinsic.pitch_deg:.2f} "
                f"yaw={extrinsic.yaw_deg:.2f} | age={age_ms:.0f}ms{px4_text}"
            )
        if hasattr(self.panel, "signal_attitude_age_plot"):
            self.panel.signal_attitude_age_plot.add_point(TelemetryHistoryPoint(time.monotonic(), camera, age_ms))
        self.panel._update_pixhawk_connection_status()
        self.panel._update_relative_attitude_status()
        self.panel._refresh_rtk_alignment_view()
        self.panel._refresh_alignment_attitude_ui()
        self.panel._refresh_alignment_overview()
        self.panel._refresh_gnss_base_tab()
        self.panel._refresh_alignment_validation()
        self._refresh_camera_parameters_if_needed()
        self.panel._refresh_distance()

    def _update_gps(self, camera: str, gps: GpsPositionState) -> None:
        self.panel.latest_gps[camera] = gps
        self._observe_gnss_frozen_position(camera, gps)
        if camera in {"left", "right"}:
            publisher = getattr(self.panel, "measurement_source_transition", None)
            if callable(publisher):
                publisher(
                    f"pixhawk.{camera}",
                    "LIVE",
                    "fresh expected-system GPS received",
                )
            self.panel.gnss_monitor.update_position(camera, gps)
        elif camera.endswith("_GPS2"):
            # Keep the secondary receiver under its explicit side/source key
            # so GNSSMonitor can honor a P1/P2 GPS2 selection.
            side = camera.split("_", 1)[0]
            if side in {"left", "right"}:
                self.panel.gnss_monitor.update_position(camera, gps)
        if hasattr(self.panel, "baseline_gps_rate_history") and camera in self.panel.baseline_gps_rate_history:
            self.panel.baseline_gps_rate_history[camera].append(time.monotonic())
        raw = gps.raw
        raw_vel = getattr(raw, "vel", None) if raw is not None else None
        ground_speed_m_s = None if raw_vel in (None, 65535) else float(raw_vel) * 0.01
        raw_yaw = getattr(raw, "yaw", None) if raw is not None else None
        gps_yaw_deg = None if raw_yaw in (None, 65535) else float(raw_yaw) * 0.01
        raw_hdg_acc = getattr(raw, "hdg_acc", None) if raw is not None else None
        heading_accuracy_deg = (
            None if raw_hdg_acc in (None, 0, 65535) else float(raw_hdg_acc) * 1e-5
        )
        self.panel._record_external_timeline_row(
            camera,
            "gps_rtk",
            {
                "namespace": gps.namespace,
                "topic": gps.topic,
                "mavlink_source_system": "" if gps.mavlink_source_system is None else gps.mavlink_source_system,
                "mavlink_source_component": (
                    "" if gps.mavlink_source_component is None else gps.mavlink_source_component
                ),
                "mavlink_sequence": "" if gps.mavlink_sequence is None else gps.mavlink_sequence,
                "mavlink_message_id": "" if gps.mavlink_message_id is None else gps.mavlink_message_id,
                "sample_app_timestamp_s": gps.timestamp_s,
                "sample_px4_timestamp_s": self.panel._fmt_record_float(gps.px4_timestamp_s, 6),
                "px4_timestamp_us": "" if gps.px4_timestamp_us is None else gps.px4_timestamp_us,
                "px4_timestamp_sample_us": (
                    "" if gps.px4_timestamp_sample_us is None else gps.px4_timestamp_sample_us
                ),
                "gps_utc_us": "" if gps.gps_utc_us is None else gps.gps_utc_us,
                "gps_timestamp_time_relative_us": (
                    ""
                    if gps.gps_timestamp_time_relative_us is None
                    else gps.gps_timestamp_time_relative_us
                ),
                "time_source": gps.time_source,
                "source_utc_method": (
                    gps.time_source
                    if gps.time_source.startswith("mavlink_")
                    else "px4_vehicle_gps_position_time_utc_usec"
                ),
                "lat_deg": self.panel._fmt_record_float(gps.latitude_deg, 9),
                "lon_deg": self.panel._fmt_record_float(gps.longitude_deg, 9),
                "alt_m": self.panel._fmt_record_float(gps.altitude_m, 6),
                "altitude_ellipsoid_m": self.panel._fmt_record_float(gps.altitude_ellipsoid_m, 6),
                "fix_type": "" if gps.fix_type is None else str(gps.fix_type),
                "satellites": "" if gps.satellites_used is None else str(gps.satellites_used),
                "eph_m": self.panel._fmt_record_float(gps.eph_m, 6),
                "epv_m": self.panel._fmt_record_float(gps.epv_m, 6),
                "heading_deg": self.panel._fmt_record_float(gps.heading_deg, 6),
                "ground_speed_m_s": self.panel._fmt_record_float(ground_speed_m_s, 6),
                "gps_yaw_deg": self.panel._fmt_record_float(gps_yaw_deg, 6),
                "heading_accuracy_deg": self.panel._fmt_record_float(
                    heading_accuracy_deg if heading_accuracy_deg is not None else gps.heading_accuracy_deg,
                    6,
                ),
                "position_contract": "WGS84 latitude/longitude; MSL and ellipsoid altitude fields explicit",
                "velocity_contract": "world NED north/east/down metres_per_second",
                "rtk_label": gps.rtk_label,
                "age_s": self.panel._fmt_record_float(max(0.0, time.monotonic() - gps.timestamp_s), 6),
            },
        )
        if camera == "drone":
            recorder = getattr(self.panel, "_record_gnss_baseline_center", None)
            if callable(recorder):
                recorder(camera, gps)
        age_ms = max(0.0, time.monotonic() - gps.timestamp_s) * 1000.0
        accuracy_text = []
        if gps.satellites_used is not None:
            accuracy_text.append(f"sats={gps.satellites_used}")
        if gps.eph_m is not None:
            accuracy_text.append(f"Hacc={gps.eph_m:.2f}m")
        if gps.epv_m is not None:
            accuracy_text.append(f"Vacc={gps.epv_m:.2f}m")
        if gps.heading_deg is not None:
            accuracy_text.append(f"gpsYaw={gps.heading_deg:.2f}°")
        extra = " ".join(accuracy_text)
        source = self.panel._telemetry_source_label(gps.topic)
        text = f"{camera} GPS/RTK: {source} | {gps.rtk_label}, age={age_ms:.0f}ms, {extra}"
        if camera == "left":
            self.panel._set_health_label(self.panel.left_gps_label, text, self.panel._gps_fix_level(gps.fix_type, age_ms=age_ms))
        elif camera == "right":
            self.panel._set_health_label(self.panel.right_gps_label, text, self.panel._gps_fix_level(gps.fix_type, age_ms=age_ms))
        if hasattr(self.panel, "signal_gps_position_labels") and camera in self.panel.signal_gps_position_labels:
            self.panel._set_health_label(
                self.panel.signal_gps_position_labels[camera],
                f"GPS position: {self.panel._telemetry_source_label(gps.topic)} | {gps.rtk_label} | "
                f"{self.panel._gps_position_summary_text(gps, age_ms=age_ms)}",
                self.panel._gps_fix_level(gps.fix_type, age_ms=age_ms),
            )
        self.panel._refresh_rtk_baseline()
        self.panel._refresh_baseline_alignment_ui()
        self.panel._refresh_gnss_base_tab()
        self.panel._refresh_gnss_operation_readiness()
        self.panel._append_baseline_sample_if_active()
        self._refresh_camera_parameters_if_needed()
        self.panel._refresh_distance()

    def _refresh_camera_parameters_if_needed(self) -> None:
        """Avoid rebuilding hidden parameter text on every telemetry packet.

        Workstation keeps the legacy owner panel hidden and renders its own
        source-backed surfaces.  Rebuilding several QPlainTextEdit documents
        for every GPS/ATTITUDE packet monopolizes the GUI thread, especially
        while both Pixhawks stream at high rates.  A visible legacy panel only
        needs this display while its parameter tab is active, and even then a
        bounded refresh rate is sufficient.
        """

        refresh = getattr(self.panel, "_refresh_camera_parameters_tab", None)
        if not callable(refresh):
            return
        is_visible = getattr(self.panel, "isVisible", None)
        if callable(is_visible) and not bool(is_visible()):
            return
        tabs = getattr(self.panel, "tabs", None)
        parameter_index = getattr(self.panel, "parameter_tab_index", None)
        if tabs is not None and parameter_index is not None:
            try:
                if int(tabs.currentIndex()) != int(parameter_index):
                    return
            except (AttributeError, TypeError, ValueError):
                pass
        now_s = time.monotonic()
        if now_s - self._last_camera_parameters_refresh_s < 0.5:
            return
        self._last_camera_parameters_refresh_s = now_s
        refresh()

    def _update_mag_sensor(self, camera: str, state) -> None:
        topic = str(getattr(state, "topic", ""))
        namespace = str(getattr(state, "namespace", ""))
        timestamp_s = float(getattr(state, "timestamp_s", time.monotonic()))
        px4_timestamp_s = getattr(state, "px4_timestamp_s", None)
        if "vehicle_magnetometer" in topic and getattr(state, "device_id", None) is not None:
            if camera in self.panel.latest_vehicle_mag_device_ids:
                self.panel.latest_vehicle_mag_device_ids[camera] = int(state.device_id)
                self.panel.refresh_px4_mag_manager(camera)
        sensor_slot = str(getattr(state, "sensor_slot", ""))
        if topic.startswith("mavlink/SCALED_IMU") and getattr(state, "device_id", None) is None:
            # Stable PX4 Shell supplies the per-instance device identity, while
            # SCALED_IMU carries the higher-rate vector without that identity.
            # Preserve the latest Shell identity when the faster telemetry
            # sample replaces the same presentation slot; otherwise the active
            # PX4 yaw gate falls back to ID 확인 대기 on every SCALED_IMU tick.
            previous_state = self.panel.latest_mag_sensors.get(camera, {}).get(sensor_slot)
            previous_device_id = getattr(previous_state, "device_id", None)
            if (
                previous_state is not None
                and previous_device_id is not None
            ):
                try:
                    state = replace(state, device_id=int(previous_device_id))
                except (TypeError, ValueError):
                    try:
                        state.device_id = int(previous_device_id)
                    except (AttributeError, TypeError, ValueError):
                        pass
        device_id = self.panel._mag_state_display_device_id(camera, state)[0]
        device_type = self.panel._magnetometer_device_type(sensor_slot, device_id)
        is_px4_shell_can_mag = (
            topic.lower().startswith("px4-shell/sensor_mag/")
            and sensor_slot in {"CAN1", "CAN2"}
            and getattr(state, "device_id", None) in RM3100_IDS
        )
        is_dedicated_can_mag = any(
            message_name in topic.upper() for message_name in ("CAN_MAG", "CAN_MAGNETOMETER", "SASL_CAN_MAG")
        ) or (topic.upper().endswith("DEBUG_VECT") and sensor_slot in {"CAN1", "CAN2"}) or is_px4_shell_can_mag
        if is_dedicated_can_mag:
            can_states = getattr(self.panel, "latest_can_mag_sensors", None)
            if can_states is None:
                can_states = self.panel.latest_can_mag_sensors = {"left": {}, "right": {}}
            can_states.setdefault(camera, {})[state.sensor_slot] = state
        else:
            self.panel.latest_mag_sensors.setdefault(camera, {})[state.sensor_slot] = state
        stream_prefix = {
            "mavlink/SCALED_IMU": "mag_scaled_imu",
            "mavlink/SCALED_IMU2": "mag_scaled_imu2",
            "mavlink/SCALED_IMU3": "mag_scaled_imu3",
        }.get(topic)
        mag_detail_values = {}
        if stream_prefix is not None:
            for suffix in ("accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"):
                mag_detail_values[f"{stream_prefix}_{suffix}"] = self.panel._fmt_record_float(
                    getattr(state, suffix, None), 9
                )
            mag_detail_values[f"{stream_prefix}_time_boot_ms"] = (
                "" if getattr(state, "time_boot_ms", None) is None else state.time_boot_ms
            )
        self.panel._record_external_timeline_row(
            camera,
            "magnetometer",
            {
                "namespace": namespace,
                "topic": topic,
                "sensor_slot": sensor_slot,
                "sample_app_timestamp_s": timestamp_s,
                "sample_px4_timestamp_s": self.panel._fmt_record_float(px4_timestamp_s, 6),
                "device_id": "" if getattr(state, "device_id", None) is None else state.device_id,
                "node_id": "" if getattr(state, "node_id", None) is None else state.node_id,
                "mag_device_type": device_type,
                "mag_value_units": (
                    "gauss"
                    if "sensor_mag" in topic or "vehicle_magnetometer" in topic
                    else "milligauss"
                    if topic.startswith("mavlink/SCALED_IMU")
                    else "gauss"
                    if topic.upper().endswith("DEBUG_VECT") and sensor_slot in {"CAN1", "CAN2"}
                    else ""
                ),
                "mag_x": self.panel._fmt_record_float(getattr(state, "x", None), 9),
                "mag_y": self.panel._fmt_record_float(getattr(state, "y", None), 9),
                "mag_z": self.panel._fmt_record_float(getattr(state, "z", None), 9),
                "mag_vector_norm": self.panel._fmt_record_float(getattr(state, "magnitude", None), 9),
                "mag_temperature_c": self.panel._fmt_record_float(getattr(state, "temperature_c", None), 6),
                "mag_error_count": "" if getattr(state, "error_count", None) is None else state.error_count,
                "mag_px4_timestamp_s": self.panel._fmt_record_float(px4_timestamp_s, 6),
                **mag_detail_values,
            },
        )
        if topic.startswith("mavlink/SCALED_IMU"):
            table = self.panel.px4_mag_tables.get(camera) if hasattr(self.panel, "px4_mag_tables") else None
            if table is not None and table.rowCount() == 0 and self.panel._mag_state_display_device_id(camera, state)[0] is not None:
                self.panel.refresh_px4_mag_manager(camera)
        # A dedicated CAN_MAG/SASL_CAN_MAG sample for this slot must keep winning
        # over legacy SCALED_IMU-labeled samples regardless of arrival order --
        # same precedence as _mag_slot_state, reused here so the live label,
        # plots, and magnetometer_monitor (which feeds the CAN1/CAN2 difference
        # calibration metric) never regress to a stale/coincidental legacy read.
        # ``state`` (the raw message just received) is still used above for
        # timeline logging and device-id inference, which must reflect what was
        # actually received rather than the winning cached sample.
        display_state = self.panel._mag_slot_state(camera, sensor_slot)
        if display_state is None:
            display_state = state
        display_device_id, display_device_source = self.panel._mag_state_display_device_id(camera, display_state)
        self.panel.magnetometer_monitor.update(camera, display_state)
        label = self.panel.mag_sensor_labels.get(camera, {}).get(display_state.sensor_slot)
        age_ms = max(0.0, time.monotonic() - display_state.timestamp_s) * 1000.0
        components = "x/y/z=waiting"
        if display_state.x is not None and display_state.y is not None and display_state.z is not None:
            components = f"x/y/z={display_state.x:.3f}/{display_state.y:.3f}/{display_state.z:.3f}"
        vector_norm = f" norm={display_state.magnitude:.3f}" if display_state.magnitude is not None else ""
        device = "" if display_device_id is None else f" device={display_device_id} ({display_device_source})"
        text = f"{components}{vector_norm}, age={age_ms:.0f}ms{device}"
        if label is not None:
            label.setText(text)
        if display_state.magnitude is not None:
            legacy_plot = self.panel.mag_sensor_plots.get(camera)
            if legacy_plot is not None:
                legacy_plot.add_point(
                    TelemetryHistoryPoint(display_state.timestamp_s, display_state.sensor_slot, display_state.magnitude)
                )
            if hasattr(self.panel, "rm3100_norm_plots"):
                plot = self.panel.rm3100_norm_plots.get(camera)
                if plot is not None:
                    plot.add_point(
                        TelemetryHistoryPoint(display_state.timestamp_s, display_state.sensor_slot, display_state.magnitude)
                    )
        self.panel._refresh_pixhawk_sensor_health()
        self.panel._refresh_alignment_workflow_status()
        self.panel._refresh_alignment_magnetometer_ui()
        self.panel._refresh_alignment_overview()

    def _update_gps_sensor(self, camera: str, state) -> None:
        self.panel.latest_gps_sensors.setdefault(camera, {})[state.sensor_slot] = state
        self.panel.gnss_monitor.update_sensor(camera, state.sensor_slot, state)
        self.panel._record_external_timeline_row(
            camera,
            "sensor_gps",
            {
                "namespace": state.namespace,
                "topic": state.topic,
                "sensor_slot": state.sensor_slot,
                "mavlink_source_system": (
                    "" if getattr(state, "mavlink_source_system", None) is None else state.mavlink_source_system
                ),
                "mavlink_source_component": (
                    ""
                    if getattr(state, "mavlink_source_component", None) is None
                    else state.mavlink_source_component
                ),
                "mavlink_sequence": "" if getattr(state, "mavlink_sequence", None) is None else state.mavlink_sequence,
                "mavlink_message_id": (
                    "" if getattr(state, "mavlink_message_id", None) is None else state.mavlink_message_id
                ),
                "sample_app_timestamp_s": state.timestamp_s,
                "sample_px4_timestamp_s": self.panel._fmt_record_float(state.px4_timestamp_s, 6),
                "px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
                "px4_timestamp_sample_us": (
                    "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
                ),
                "gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
                "gps_timestamp_time_relative_us": (
                    ""
                    if state.gps_timestamp_time_relative_us is None
                    else state.gps_timestamp_time_relative_us
                ),
                "time_source": "px4_sensor_gps",
                "source_utc_method": "px4_sensor_gps_time_utc_usec",
                "device_id": "" if state.device_id is None else state.device_id,
                "fix_type": "" if state.fix_type is None else state.fix_type,
                "satellites": "" if state.satellites_used is None else state.satellites_used,
                "eph_m": self.panel._fmt_record_float(state.eph_m, 6),
                "epv_m": self.panel._fmt_record_float(state.epv_m, 6),
            },
        )
        label = self.panel.gps_sensor_labels.get(camera, {}).get(state.sensor_slot)
        text = self.panel._gps_sensor_status_text(state, include_source=True, include_detail=True)
        if label is not None:
            label.setText(text)
        if hasattr(self.panel, "signal_gps_sensor_labels") and camera in self.panel.signal_gps_sensor_labels:
            age_ms = max(0.0, time.monotonic() - state.timestamp_s) * 1000.0
            self.panel._set_health_label(
                self.panel.signal_gps_sensor_labels[camera],
                f"GPS sensor: {state.sensor_slot} | "
                f"{self.panel._gps_sensor_status_text(state, include_source=True, include_detail=False)}",
                self.panel._gps_fix_level(state.fix_type, age_ms=age_ms),
            )
        signal_series = f"{camera}_{state.sensor_slot}"
        if state.satellites_used is not None and hasattr(self.panel, "signal_gps_sats_plot"):
            self.panel.signal_gps_sats_plot.add_point(
                TelemetryHistoryPoint(state.timestamp_s, signal_series, float(state.satellites_used))
            )
        if state.eph_m is not None and hasattr(self.panel, "signal_gps_eph_plot"):
            self.panel.signal_gps_eph_plot.add_point(TelemetryHistoryPoint(state.timestamp_s, signal_series, state.eph_m))
        if state.satellites_used is not None:
            legacy_plot = self.panel.gps_sensor_plots.get(camera)
            if legacy_plot is not None:
                legacy_plot.add_point(
                    TelemetryHistoryPoint(state.timestamp_s, state.sensor_slot, float(state.satellites_used))
                )
            if hasattr(self.panel, "f9p_sats_plots"):
                plot = self.panel.f9p_sats_plots.get(camera)
                if plot is not None:
                    plot.add_point(TelemetryHistoryPoint(state.timestamp_s, state.sensor_slot, float(state.satellites_used)))
        if state.eph_m is not None and hasattr(self.panel, "f9p_quality_plots"):
            plot = self.panel.f9p_quality_plots.get(camera)
            if plot is not None:
                plot.add_point(TelemetryHistoryPoint(state.timestamp_s, state.sensor_slot, state.eph_m))
        self.panel._refresh_pixhawk_sensor_health()
        self.panel._refresh_alignment_workflow_status()
        self.panel._refresh_baseline_alignment_ui()
        self.panel._refresh_gnss_base_tab()
        self.panel._refresh_f9p_compare_label(camera)

    def _refresh_f9p_compare_label(self, camera: str) -> None:
        label = self.panel.f9p_compare_labels.get(camera)
        if label is None:
            return
        now_s = time.monotonic()
        gps_slots = self.panel.latest_gps_sensors.get(camera, {})
        gps_detected = sum(1 for slot in ("GPS1", "GPS2") if slot in gps_slots)
        gps1 = gps_slots.get("GPS1")
        gps2 = gps_slots.get("GPS2")
        if gps1 is None or gps2 is None:
            self.panel._set_health_label(label, f"{gps_detected}/2 detected", "bad" if gps_detected else "idle")
            return
        details = []
        if gps1.fix_type is not None and gps2.fix_type is not None:
            details.append(f"fix={self.panel._fix_type_label(gps1.fix_type)}/{self.panel._fix_type_label(gps2.fix_type)}")
        if gps1.satellites_used is not None and gps2.satellites_used is not None:
            details.append(f"Δsats={abs(gps1.satellites_used - gps2.satellites_used)}")
        if gps1.eph_m is not None and gps2.eph_m is not None:
            details.append(f"ΔHacc={abs(gps1.eph_m - gps2.eph_m):.2f}m")
        details.append(f"skew={abs(gps1.timestamp_s - gps2.timestamp_s) * 1000.0:.0f}ms")
        details.append(f"src={self.panel._telemetry_source_label(gps1.topic)} / {self.panel._telemetry_source_label(gps2.topic)}")
        poor_fix = (gps1.fix_type is not None and gps1.fix_type < 3) or (
            gps2.fix_type is not None and gps2.fix_type < 3
        )
        not_rtk = (gps1.fix_type or 0) < 5 or (gps2.fix_type or 0) < 5
        stale = now_s - gps1.timestamp_s > 1.0 or now_s - gps2.timestamp_s > 1.0
        level = "bad" if poor_fix else "warn" if stale or not_rtk else "good"
        self.panel._set_health_label(label, f"2/2 detected · {' · '.join(details)}", level)

    def _update_gnss_relative(self, camera: str, state) -> None:
        self.panel.latest_gnss_relative[camera] = state
        heading_deg = None if state.heading_rad is None else math.degrees(state.heading_rad)
        heading_accuracy_deg = None if state.heading_accuracy_rad is None else math.degrees(state.heading_accuracy_rad)
        self.panel._record_external_timeline_row(
            camera,
            "gnss_relative",
            {
                "namespace": state.namespace,
                "topic": state.topic,
                "sample_app_timestamp_s": state.timestamp_s,
                "mavlink_source_system": (
                    "" if getattr(state, "mavlink_source_system", None) is None else state.mavlink_source_system
                ),
                "mavlink_source_component": (
                    ""
                    if getattr(state, "mavlink_source_component", None) is None
                    else state.mavlink_source_component
                ),
                "mavlink_sequence": "" if getattr(state, "mavlink_sequence", None) is None else state.mavlink_sequence,
                "mavlink_message_id": (
                    "" if getattr(state, "mavlink_message_id", None) is None else state.mavlink_message_id
                ),
                "sample_px4_timestamp_s": self.panel._fmt_record_float(state.px4_timestamp_s, 6),
                "px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
                "px4_timestamp_sample_us": (
                    "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
                ),
                "gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
                "time_source": "px4_sensor_gnss_relative",
                "source_utc_method": "px4_sensor_gnss_relative_time_utc_usec",
                "device_id": "" if state.device_id is None else str(state.device_id),
                "heading_deg": self.panel._fmt_record_float(heading_deg, 6),
                "heading_accuracy_deg": self.panel._fmt_record_float(heading_accuracy_deg, 6),
                "position_length_m": self.panel._fmt_record_float(state.position_length_m, 6),
                "accuracy_length_m": self.panel._fmt_record_float(state.accuracy_length_m, 6),
                "gnss_fix_ok": self.panel._fmt_record_bool(state.gnss_fix_ok),
                "differential_solution": self.panel._fmt_record_bool(state.differential_solution),
                "relative_position_valid": self.panel._fmt_record_bool(state.relative_position_valid),
                "carrier_solution_floating": self.panel._fmt_record_bool(state.carrier_solution_floating),
                "carrier_solution_fixed": self.panel._fmt_record_bool(state.carrier_solution_fixed),
                "moving_base_mode": self.panel._fmt_record_bool(state.moving_base_mode),
                "heading_valid": self.panel._fmt_record_bool(state.heading_valid),
                "age_s": self.panel._fmt_record_float(max(0.0, time.monotonic() - state.timestamp_s), 6),
            },
        )
        self.panel._refresh_pixhawk_sensor_health()

    def _freshest_sensor_slots(self, sensor_map: dict[str, dict[str, object]]) -> dict[str, object]:
        merged: dict[str, object] = {}
        for states in sensor_map.values():
            for slot, state in states.items():
                previous = merged.get(slot)
                if previous is None or getattr(state, "timestamp_s", 0.0) > getattr(previous, "timestamp_s", 0.0):
                    merged[slot] = state
        return merged

    def _set_health_label(self, label: QtWidgets.QLabel, text: str, level: str) -> None:
        colors = {
            "good": (PANEL_ALT_BG, SUCCESS, SUCCESS_TEXT),
            "warn": (PANEL_ALT_BG, WARNING, WARNING_TEXT),
            "bad": (PANEL_ALT_BG, ERROR, ERROR_TEXT),
            "idle": (PANEL_ALT_BG, PANEL_BORDER, TEXT_MUTED),
        }
        background, border, color = colors.get(level, colors["idle"])
        style = (
            f"background: {background}; border: 1px solid {border}; "
            f"border-radius: 0px; color: {color}; padding: 6px 8px; font-weight: 600;"
        )
        if label.text() == text and label.property("healthLevel") == level and label.styleSheet() == style:
            return
        label.setProperty("healthLevel", level)
        label.setText(text)
        label.setStyleSheet(style)

    def _set_label_text_if_changed(self, label: QtWidgets.QLabel, text: str) -> None:
        if label.text() != text:
            label.setText(text)

    def _telemetry_source_label(self, topic: object) -> str:
        text = str(topic or "").strip()
        if not text:
            return "From: unknown"
        if text.startswith("mavlink/"):
            return f"MAVLink {text.removeprefix('mavlink/')}"
        return f"MAVLink {text}"

    def _gps_position_summary_text(self, gps: GpsPositionState, *, age_ms: float) -> str:
        sats = f"sats={gps.satellites_used}" if gps.satellites_used is not None else "sats=waiting"
        eph = f" | Hacc={gps.eph_m:.2f}m" if gps.eph_m is not None else ""
        return f"{sats}{eph} | age={age_ms:.0f}ms"

    def _gps_fix_level(self, fix_type: int | None, *, age_ms: float | None = None) -> str:
        if fix_type is None or fix_type < 3:
            return "bad"
        if age_ms is not None and age_ms > 3000:
            return "bad"
        if age_ms is not None and age_ms > 1000:
            return "warn"
        if fix_type >= 6:
            return "good"
        return "warn"

    def _gps_sensor_status_text(
        self,
        state: object,
        *,
        now_s: float | None = None,
        include_source: bool,
        include_detail: bool = True,
    ) -> str:
        if now_s is None:
            now_s = time.monotonic()
        age_ms = max(0.0, now_s - float(getattr(state, "timestamp_s", now_s))) * 1000.0
        fix_value = getattr(state, "fix_type", None)
        fix = self.panel._fix_type_label(fix_value)
        sats = (
            f"sats={getattr(state, 'satellites_used')}"
            if getattr(state, "satellites_used", None) is not None
            else "sats=waiting"
        )
        eph = f" | Hacc={getattr(state, 'eph_m'):.2f}m" if getattr(state, "eph_m", None) is not None else ""
        epv = ""
        device = ""
        px4_time = ""
        if include_detail:
            epv = f" | Vacc={getattr(state, 'epv_m'):.2f}m" if getattr(state, "epv_m", None) is not None else ""
            device = f" | device={getattr(state, 'device_id')}" if getattr(state, "device_id", None) is not None else ""
            px4_time = (
                f" | px4_t={getattr(state, 'px4_timestamp_s'):.3f}s"
                if getattr(state, "px4_timestamp_s", None) is not None
                else ""
            )
        source = f" | {self.panel._telemetry_source_label(getattr(state, 'topic', ''))}" if include_source else ""
        return f"fix={fix} | {sats}{eph}{epv} | age={age_ms:.0f}ms{device}{px4_time}{source}"

    def _vector_angle_deg(self, first, second) -> float | None:
        if (
            first.x is None
            or first.y is None
            or first.z is None
            or second.x is None
            or second.y is None
            or second.z is None
        ):
            return None
        first_norm = math.sqrt(first.x * first.x + first.y * first.y + first.z * first.z)
        second_norm = math.sqrt(second.x * second.x + second.y * second.y + second.z * second.z)
        if first_norm < 1e-9 or second_norm < 1e-9:
            return None
        dot = first.x * second.x + first.y * second.y + first.z * second.z
        ratio = max(-1.0, min(1.0, dot / (first_norm * second_norm)))
        return math.degrees(math.acos(ratio))

    def _fix_type_label(self, fix_type: int | None) -> str:
        return {
            0: "no fix",
            1: "dead reckoning",
            2: "2D",
            3: "3D",
            4: "DGPS",
            5: "RTK Float",
            6: "RTK Fixed",
            8: "Static",
        }.get(fix_type, "waiting" if fix_type is None else str(fix_type))

    def _refresh_pixhawk_sensor_health(self) -> None:
        if not hasattr(self.panel, "pixhawk_sensor_summary_labels"):
            return
        if not self.panel.pixhawk_sensor_summary_labels.get("left") and not self.panel.pixhawk_sensor_summary_labels.get("right"):
            return
        now_s = time.monotonic()
        for camera, title in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            self.panel._refresh_one_pixhawk_sensor_health(camera, title, now_s)

    def _refresh_one_pixhawk_sensor_health(self, camera: str, title: str, now_s: float) -> None:
        gps_slots = self.panel.latest_gps_sensors.get(camera, {})
        gnss_relative = self.panel.latest_gnss_relative.get(camera)
        rm_detected = sum(
            1 for slot in ("CAN1", "CAN2") if self.panel._mag_slot_state(camera, slot) is not None
        )
        for slot, label in self.panel.rm3100_slot_labels[camera].items():
            state = self.panel._mag_slot_state(camera, slot)
            if state is None:
                self.panel._set_health_label(
                    label,
                    "waiting",
                    "idle",
                )
                continue
            age_ms = max(0.0, now_s - state.timestamp_s) * 1000.0
            components = "x/y/z=waiting"
            if state.x is not None and state.y is not None and state.z is not None:
                components = f"x/y/z={state.x:.3f}/{state.y:.3f}/{state.z:.3f}G"
            magnitude = f" | norm={state.magnitude:.3f}G" if state.magnitude is not None else ""
            temp = f" | temp={state.temperature_c:.1f}C" if state.temperature_c is not None else ""
            errors = f" | errors={state.error_count}" if getattr(state, "error_count", None) is not None else ""
            device = f" | device={state.device_id}" if state.device_id is not None else ""
            level = "bad" if age_ms > 3000 else "warn" if age_ms > 1000 else "good"
            self.panel._set_health_label(label, f"{components}{magnitude}{temp}{errors} | age={age_ms:.0f}ms{device}", level)

        can1 = self.panel._mag_slot_state(camera, "CAN1")
        can2 = self.panel._mag_slot_state(camera, "CAN2")
        rm_level = "bad"
        rm_summary = f"Status: 대기 중 | {rm_detected}/2 detected"
        if can1 is not None and can2 is not None:
            can1_age = now_s - can1.timestamp_s
            can2_age = now_s - can2.timestamp_s
            delta_mag = self.panel.magnetometer_monitor.can_difference(camera)
            delta_display, delta_units = self.panel.magnetometer_monitor.can_difference_display(camera)
            angle = self.panel._vector_angle_deg(can1, can2)
            details = []
            if delta_mag is not None:
                details.append(
                    f"Δnorm={delta_display:.3f}{delta_units}"
                    if delta_display is not None
                    else f"Δnorm={delta_mag:.3f}G"
                )
                self.panel.rm3100_delta_plots[camera].add_point(TelemetryHistoryPoint(now_s, "Δnorm", delta_mag))
            if angle is not None:
                details.append(f"angle={angle:.1f}deg")
                self.panel.rm3100_delta_plots[camera].add_point(TelemetryHistoryPoint(now_s, "angle", angle))
            details.append(f"skew={abs(can1_age - can2_age) * 1000.0:.0f}ms")
            stale = can1_age > 1.0 or can2_age > 1.0
            abnormal_delta = delta_mag is not None and delta_mag > 0.20
            abnormal_angle = angle is not None and angle > 20.0
            rm_level = "warn" if stale or abnormal_delta or abnormal_angle else "good"
            status = "WARN" if rm_level == "warn" else "OK"
            can1_text = self.panel._mag_state_value_text(can1.magnitude, self.panel._mag_state_value_units(can1))
            can2_text = self.panel._mag_state_value_text(can2.magnitude, self.panel._mag_state_value_units(can2))
            delta_text = "--" if delta_display is None else self.panel._mag_state_value_text(delta_display, delta_units)
            angle_text = "--" if angle is None else f"{angle:.1f}deg"
            rm_summary = f"Status: {status} | CAN1 {can1_text} / CAN2 {can2_text} | Δ {delta_text}, {angle_text}"
            self.panel._set_health_label(self.panel.rm3100_compare_labels[camera], rm_summary, rm_level)
        else:
            self.panel._set_health_label(
                self.panel.rm3100_compare_labels[camera],
                rm_summary,
                "bad" if rm_detected else "idle",
            )
        self.panel._set_health_label(
            self.panel.pixhawk_sensor_summary_labels[camera]["rm3100"],
            f"{title}: {rm_summary}",
            rm_level if rm_detected else "idle",
        )

        gps_detected = sum(1 for slot in ("GPS1", "GPS2") if slot in gps_slots)
        for slot, label in self.panel.f9p_slot_labels[camera].items():
            state = gps_slots.get(slot)
            if state is None:
                self.panel._set_health_label(
                    label,
                    "waiting",
                    "idle",
                )
                continue
            age_ms = max(0.0, now_s - state.timestamp_s) * 1000.0
            poor_fix = state.fix_type is not None and state.fix_type < 3
            level = "bad" if age_ms > 3000 or poor_fix else "warn" if age_ms > 1000 or (state.fix_type or 0) < 5 else "good"
            self.panel._set_health_label(label, self.panel._gps_sensor_status_text(state, now_s=now_s, include_source=True), level)

        gps1 = gps_slots.get("GPS1")
        gps2 = gps_slots.get("GPS2")
        f9p_level = "bad"
        f9p_summary = f"{gps_detected}/2 detected"
        if gps1 is not None and gps2 is not None:
            details = []
            if gps1.fix_type is not None and gps2.fix_type is not None:
                details.append(f"fix={self.panel._fix_type_label(gps1.fix_type)}/{self.panel._fix_type_label(gps2.fix_type)}")
            if gps1.satellites_used is not None and gps2.satellites_used is not None:
                details.append(f"Δsats={abs(gps1.satellites_used - gps2.satellites_used)}")
            if gps1.eph_m is not None and gps2.eph_m is not None:
                details.append(f"Δeph={abs(gps1.eph_m - gps2.eph_m):.2f}m")
            details.append(f"skew={abs(gps1.timestamp_s - gps2.timestamp_s) * 1000.0:.0f}ms")
            details.append(f"src={self.panel._telemetry_source_label(gps1.topic)} / {self.panel._telemetry_source_label(gps2.topic)}")
            poor_fix = (gps1.fix_type is not None and gps1.fix_type < 3) or (gps2.fix_type is not None and gps2.fix_type < 3)
            not_rtk = (gps1.fix_type or 0) < 5 or (gps2.fix_type or 0) < 5
            stale = now_s - gps1.timestamp_s > 1.0 or now_s - gps2.timestamp_s > 1.0
            f9p_level = "bad" if poor_fix else "warn" if stale or not_rtk else "good"
            f9p_summary = f"2/2 detected · {' · '.join(details)}"
            self.panel._set_health_label(self.panel.f9p_compare_labels[camera], f9p_summary, f9p_level)
        else:
            self.panel._set_health_label(
                self.panel.f9p_compare_labels[camera],
                f9p_summary,
                "bad" if gps_detected else "idle",
            )
        self.panel._set_health_label(
            self.panel.pixhawk_sensor_summary_labels[camera]["f9p"],
            f"{title}: {f9p_summary}",
            f9p_level if gps_detected else "idle",
        )
        heading_text = "GNSS Heading: sensor_gnss_relative waiting"
        heading_level = "idle"
        if gnss_relative is not None:
            age_ms = max(0.0, now_s - gnss_relative.timestamp_s) * 1000.0
            heading_deg = None
            if gnss_relative.heading_rad is not None:
                heading_deg = math.degrees(gnss_relative.heading_rad)
            accuracy_deg = None
            if gnss_relative.heading_accuracy_rad is not None:
                accuracy_deg = math.degrees(gnss_relative.heading_accuracy_rad)
            heading_parts = [
                f"heading={heading_deg:.2f}deg" if heading_deg is not None else "heading=waiting",
                f"acc={accuracy_deg:.2f}deg" if accuracy_deg is not None else "acc=waiting",
                f"fixed={gnss_relative.carrier_solution_fixed}",
                f"moving_base={gnss_relative.moving_base_mode}",
                f"relative_valid={gnss_relative.relative_position_valid}",
                f"age={age_ms:.0f}ms",
            ]
            if gnss_relative.position_length_m is not None:
                heading_parts.append(f"baseline={gnss_relative.position_length_m:.2f}m")
            heading_level = (
                "good"
                if gnss_relative.heading_valid and gnss_relative.relative_position_valid and gnss_relative.carrier_solution_fixed
                else "warn"
                if gnss_relative.heading_valid or gnss_relative.relative_position_valid
                else "bad"
            )
            if age_ms > 3000:
                heading_level = "bad"
            elif age_ms > 1000 and heading_level == "good":
                heading_level = "warn"
            heading_text = f"GNSS Heading: {' · '.join(heading_parts)}"
        self.panel._set_health_label(self.panel.gnss_heading_labels[camera], heading_text, heading_level)
        self.panel._set_health_label(self.panel.pixhawk_sensor_summary_labels[camera]["heading"], heading_text, heading_level)

    def _update_relative_attitude_status(self) -> None:
        left = self.panel.latest_extrinsics.get("left")
        right = self.panel.latest_extrinsics.get("right")
        axis_values = (
            ("roll", None if left is None else left.roll_deg, None if right is None else right.roll_deg),
            ("pitch", None if left is None else left.pitch_deg, None if right is None else right.pitch_deg),
            ("yaw", None if left is None else left.yaw_deg, None if right is None else right.yaw_deg),
        )
        labels_by_axis = getattr(self.panel, "attitude_compare_labels", {})
        for axis, left_value, right_value in axis_values:
            labels = labels_by_axis.get(axis)
            if labels is None:
                continue
            left_label, right_label, delta_label = labels
            left_label.setText("대기 중" if left_value is None else f"{left_value:.4f} deg")
            right_label.setText("대기 중" if right_value is None else f"{right_value:.4f} deg")
            if left_value is None or right_value is None:
                self.panel._set_health_label(delta_label, "대기 중", "idle")
            else:
                delta = angle_diff_deg(right_value, left_value) if axis == "yaw" else right_value - left_value
                self.panel._set_health_label(
                    delta_label,
                    f"Δ{axis.capitalize()} {delta:+.2f}°",
                    self.panel._attitude_delta_level(axis, delta),
                )
        if left is None and right is None:
            self.panel._refresh_alignment_workflow_status()
            return
        now_s = time.monotonic()
        plots_by_axis = getattr(self.panel, "attitude_plots", {})
        for axis, left_value, right_value in axis_values:
            point = AttitudeHistoryPoint(
                timestamp_s=now_s,
                left_deg=left_value,
                right_deg=right_value,
            )
            self.panel.attitude_history[axis].append(point)
            plot = plots_by_axis.get(axis)
            if plot is not None:
                plot.add_point(point)
        self.panel._refresh_alignment_workflow_status()

    def _attitude_delta_level(self, axis: str, delta_deg: float) -> str:
        abs_delta = abs(delta_deg)
        good_limit = 3.0 if axis == "yaw" else 1.0
        bad_limit = 8.0 if axis == "yaw" else 3.0
        if abs_delta <= good_limit:
            return "good"
        if abs_delta <= bad_limit:
            return "warn"
        return "bad"
