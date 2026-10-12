from __future__ import annotations

import errno
import fcntl
import glob
import hashlib
import io
from ipaddress import IPv4Address
import math
import os
import re
import socket
import struct
import threading
import time
from contextlib import contextmanager, nullcontext
from collections import deque
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from Workstation.workstation_core.error_throttle import (
    DEFAULT_ERROR_LOG_THROTTLE_S,
    should_emit_throttled_error,
)
from Workstation.workstation_core.mavlink_liveness import (
    MavlinkLivenessState,
    MavlinkSideHealthSnapshot,
    MavlinkSideLiveness,
)
from Workstation.workstation_core.pixhawk_radio_policy import (
    RADIO_MINIMAL_STREAM_REQUESTS,
    RADIO_MINIMAL_DISABLED_MESSAGE_IDS,
    RadioStreamPolicy,
    RadioStreamTier,
    radio_stream_requests,
)
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    RM3100_BACKUP_IDS,
    RM3100_PRIMARY_IDS,
)
from Workstation.workstation_core.rtk_geometry import GpsPositionState, gps_state_from_message, gps_topic

from PyQt5 import QtCore
from Workstation.workstation_core.pixhawk_link.commands import CommandPorts, MavlinkCommands
from Workstation.workstation_core.pixhawk_link.transport import MavlinkTransport, TransportCallbacks

# Explicit migration aliases; implementations import their actual owners.
from Workstation.workstation_core.pixhawk_link.policy import (
    MAVLINK_MESSAGE_IDS,
    MAVLINK_STOCK_STREAM_REQUESTS,
    MAVLINK_STOCK_STREAM_CONSUMERS,
    MAVLINK_RM3100_STREAM_REQUESTS,
    MAVLINK_MAX_MESSAGES_PER_TICK,
    MAVLINK_MAX_PROCESS_TIME_S,
    MAVLINK_RECEIVE_ERROR_INTERVAL_S,
    MAVLINK_STREAM_REQUEST_MAX_ATTEMPTS,
    MAVLINK_STREAM_REQUEST_RETRY_S,
    MAVLINK_DUPLICATE_TTL_S,
    MAVLINK_DUPLICATE_CACHE_SIZE,
    MAVLINK_ATTITUDE_STALE_S,
    MAVLINK_ATTITUDE_RECOVERY_COOLDOWN_S,
    MAVLINK_IMU_STREAM_STALE_S,
    MAVLINK_IMU_STREAM_RECOVERY_COOLDOWN_S,
    MAVLINK_GCS_SOURCE_SYSTEM,
    MAVLINK_GCS_SOURCE_COMPONENT,
    MAVLINK_GCS_HEARTBEAT_INTERVAL_S,
    MAVLINK_COMMAND_RECEIVE_POLL_S,
    MAVLINK_SHELL_OUTPUT_IDLE_S,
    _NSH_PROMPT_TAIL_RE,
    MAVLINK_RTCM_STATUS_INTERVAL_S,
    MAVLINK_EXPECTED_SYSTEM_IDS,
    DEFAULT_USB_MAVLINK_BAUDRATES,
    USB_MAVLINK_IDENTITY_MAX_AGE_S,
    _MAVLINK_FLOAT_PARAM_TYPES,
)
from Workstation.workstation_core.pixhawk_link.wire_commands import (
    _serial_control_output_from_message,
    _mavlink_message_type,
    _shell_output_ends_with_prompt,
    _read_mavlink_shell_output,
    _send_mavlink_shell_payload,
    _send_mavlink_shell_command,
    _send_mavlink_rtcm_data,
    _write_mavlink_ftp_file,
    _mavlink_param_id,
    _mavlink_param_id_text,
    _mavlink_int_to_param_float,
    _mavlink_param_float_to_int,
    _mavlink_recv_param_value_loop,
    _mavlink_recv_param_value,
    _mavlink_decode_param_value,
    _mavlink_recv_param_value_typed,
    _mavlink_read_param,
    _mavlink_read_param_int,
    _mavlink_set_param_int,
    _decode_mavlink_serial_control,
    _mavlink_constant,
)
from Workstation.workstation_core.pixhawk_link.endpoints import (
    _mavlink_udpin_endpoint,
    _assert_mavlink_udpin_available,
    _disable_mavlink_udpin_reuse,
    _seed_mavlink_udpin_peer,
    camera_label_ko,
    _normalize_transport,
    _is_serial_transport,
    usb_mavlink_device_candidates,
    probe_usb_mavlink_devices,
)

# Explicit compatibility exports. Qt/link ownership remains in this module;
# read-only consumers may import pixhawk_link leaves without the subscriber.
from Workstation.workstation_core.pixhawk_link.types import (  # noqa: F401
    EulerAttitude,
    CameraExtrinsicState,
    RadioStatusState,
    MagSensorState,
    GpsSensorState,
    GnssRelativeState,
    CustomTopicSample,
    UsbMavlinkIdentity,
    MavlinkShellCommandResult,
)
from Workstation.workstation_core.pixhawk_link.decode import (  # noqa: F401
    parse_px4_sensor_mag_shell_output,
    parse_px4_vehicle_magnetometer_shell_device_id,
    magnetometer_value_units,
    magnetometer_value_to_gauss,
    quaternion_to_euler_deg,
    euler_rad_to_quaternion,
    camera_extrinsic_from_euler_rad,
    MAVLINK_MAG_STREAMS,
    MAVLINK_UI_IMU_STREAM_NAMES,
    MAVLINK_CAN_MAG_MESSAGE_TYPES,
    MAVLINK_MAG_SLOT_BY_MESSAGE,
    _finite_float,
    _nonfinite_numeric_reason,
    _optional_float,
    _optional_int,
    _optional_bool,
    _bounded_mavlink_id,
    mavlink_message_provenance,
    extract_field_path,
    _MAVLINK_OPTIONAL_TIMELINE_NUMERIC_SOURCE_FIELDS,
    _mavlink_nonfinite_source_reasons,
    _mavlink_optional_timeline_row,
    _scaled_mavlink_value,
    _scaled_mavlink_array_value,
    mavlink_graphable_fields_for_message,
    discover_mavlink_messages,
    _merge_gnss_relative_state,
    _gnss_relative_state_from_mavlink_gps_raw,
    _gnss_relative_state_from_mavlink_rtk,
    _gps_sensor_state_from_mavlink,
    _mag_sensor_state_from_mavlink,
    _can_mag_sensor_state_from_mavlink,
    _can_mag_sensor_state_from_debug_vect,
    _mavlink_scaled_degrees,
    _mavlink_scaled_millimetres,
    _mavlink_scaled_centimetres,
    _mavlink_scaled_dop,
    _mavlink_scaled_accuracy_mm,
    _mavlink_heading_cdeg,
    _mavlink_yaw_cdeg_to_rad,
    _mavlink_hdg_acc_deg_e5_to_rad,
)
from Workstation.workstation_core.pixhawk_link.timebase import (  # noqa: F401
    MAVLINK_BOOT_TIME_UINT32_MODULUS,
    MAVLINK_BOOT_TIME_UINT32_HALF_RANGE,
    MAVLINK_BOOT_TIME_REORDER_TOLERANCE_MS,
    MAVLINK_BOOT_TIME_REBOOT_REGRESSION_MS,
    MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_SAMPLES,
    MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_WINDOW_S,
    MAVLINK_BOOT_TIME_FORWARD_SLACK_MS,
    _MavlinkBootTimeSample,
    _MavlinkBootTimeRegressionCandidate,
    _MavlinkRemoteRebootEvidence,
    _MavlinkBootTimeObservation,
    _MavlinkBootTimeTracker,
)


# Compatibility name for existing callers; the default is now explicitly the
# Stock profile and contains no private SASL_CAN_MAG request.
# A PX4 NSH reply is complete once the prompt comes back or the link goes
# quiet. Waiting out the full command timeout instead kept the per-side
# MAVLink send lock busy for ~4.5 s per shell command (6 commands per
# sensor_mag poll cycle), which starved RTCM injection on the same link.


def _coerce_numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return float(int(value))
    if isinstance(value, (int, float)):
        return _finite_float(value)
    return None


def _format_custom_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, (list, tuple)):
        return (
            "["
            + ", ".join(_format_custom_value(item) for item in value[:6])
            + (", ..." if len(value) > 6 else "")
            + "]"
        )
    return str(value)


# Compatibility fields resolve the transport's actual objects. No copies.
def _transport_field(name: str) -> property:
    return property(
        lambda owner: getattr(owner._transport, name),
        lambda owner, value: setattr(owner._transport, name, value),
    )


class DualPixhawkMavlinkSubscriber(QtCore.QObject):
    _active_command_cameras = _transport_field("_active_command_cameras")
    _command_state_lock = _transport_field("_command_state_lock")
    _deferred_command_messages = _transport_field("_deferred_command_messages")
    _gcs_heartbeat_last_attempt_s = _transport_field("_gcs_heartbeat_last_attempt_s")
    _identity_mismatch_reported = _transport_field("_identity_mismatch_reported")
    _last_gcs_heartbeat_errors = _transport_field("_last_gcs_heartbeat_errors")
    _last_health_states = _transport_field("_last_health_states")
    _last_receive_errors = _transport_field("_last_receive_errors")
    _liveness_connection_ids = _transport_field("_liveness_connection_ids")
    _mavlink_boot_time_trackers = _transport_field("_mavlink_boot_time_trackers")
    _mavlink_liveness = _transport_field("_mavlink_liveness")
    _mavlink_send_locks = _transport_field("_mavlink_send_locks")
    _radio_lock_handles = _transport_field("_radio_lock_handles")
    _recent_raw_packets = _transport_field("_recent_raw_packets")
    _seeded_udp_peers = _transport_field("_seeded_udp_peers")
    baudrates = _transport_field("baudrates")
    connections = _transport_field("connections")
    expected_system_ids = _transport_field("expected_system_ids")
    last_connect_errors = _transport_field("last_connect_errors")
    peer_hosts = _transport_field("peer_hosts")
    transports = _transport_field("transports")
    urls = _transport_field("urls")

    _last_rtcm_status_emit_s = property(
        lambda owner: owner._commands.last_rtcm_status_emit_s,
        lambda owner, value: setattr(owner._commands, "last_rtcm_status_emit_s", value),
    )

    extrinsic_changed = QtCore.pyqtSignal(str, object)
    gps_changed = QtCore.pyqtSignal(str, object)
    mag_sensor_changed = QtCore.pyqtSignal(str, object)
    gps_sensor_changed = QtCore.pyqtSignal(str, object)
    gnss_relative_changed = QtCore.pyqtSignal(str, object)
    custom_watch_changed = QtCore.pyqtSignal(object)
    optional_timeline_changed = QtCore.pyqtSignal(str, object)
    stream_setup_required = QtCore.pyqtSignal(str)
    shell_output = QtCore.pyqtSignal(str, str)
    status_changed = QtCore.pyqtSignal(str)

    def __init__(
        self,
        *,
        left_url: str,
        right_url: str,
        left_baud: int = 57600,
        right_baud: int = 57600,
        drone_url: str = "",
        drone_baud: int = 57600,
        custom_watches: tuple[dict[str, str], ...] = (),
        expected_system_ids: dict[str, int] | None = None,
        peer_hosts: dict[str, str] | None = None,
        transports: dict[str, str] | None = None,
    ) -> None:
        super().__init__()
        self._transport = MavlinkTransport(
            left_url=left_url,
            right_url=right_url,
            left_baud=left_baud,
            right_baud=right_baud,
            drone_url=drone_url,
            drone_baud=drone_baud,
            expected_system_ids=expected_system_ids,
            peer_hosts=peer_hosts,
            transports=transports,
            callbacks=TransportCallbacks(
                status=self.status_changed.emit,
                message=self._handle_mavlink_message,
                reset_sources=self._reset_link_source_state,
                reset_stream_requests=self._reset_link_stream_requests,
                advance_stream_policy=self._advance_radio_stream_policy,
                apply_pending_setup=self._apply_pending_connection_setup,
                start_timer=self.start_receive_timer,
                stop_timer=self.stop_receive_timer,
            ),
        )
        self._radio_stream_policies = {side: RadioStreamPolicy() for side in self.urls}
        self._radio_rtcm_healthy = {side: True for side in self.urls}
        self._radio_status: dict[str, RadioStatusState] = {}
        self.custom_watches = list(custom_watches)
        self._heartbeat_logged: set[str] = set()
        self._unrecognized_debug_vect_names: set[str] = set()
        self._mavlink_stream_requests_sent: set[str] = set()
        self._mavlink_stream_request_attempts: dict[str, int] = {}
        self._mavlink_stream_last_request_s: dict[str, float] = {}
        self._sasl_can_mag_stream_confirmed: set[str] = set()
        self._last_attitude_message_s: dict[str, float] = {}
        self._attitude_stream_request_attempts: dict[str, int] = {}
        self._attitude_stream_last_request_s: dict[str, float] = {}
        self._attitude_stream_outage_reported: set[str] = set()
        self._last_imu_stream_message_s: dict[tuple[str, str], float] = {}
        self._imu_stream_request_attempts: dict[tuple[str, str], int] = {}
        self._imu_stream_last_request_s: dict[tuple[str, str], float] = {}
        self._imu_stream_outage_reported: set[tuple[str, str]] = set()
        self._mavlink_stream_config: dict[str, Any] | None = None
        self._mavlink_stream_configured: set[str] = set()
        self._latest_gnss_relative: dict[str, GnssRelativeState] = {}
        self._stream_setup_requests_sent: set[str] = set()
        self._commands = MavlinkCommands(
            CommandPorts(
                io_for=self._transport.command_io,
                reserve=self._transport.exclusive_command,
                health=self._transport.mavlink_health_snapshot,
            ),
            status=self.status_changed.emit,
        )
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.spin_once)

    def exclusive_command(
        self,
        camera: str,
        *,
        hold_send_lock: bool = True,
        command_label: str = "MAVLINK COMMAND",
        preserve_qualification: bool = False,
    ):
        return self._transport.exclusive_command(
            camera,
            hold_send_lock=hold_send_lock,
            command_label=command_label,
            preserve_qualification=preserve_qualification,
        )

    def start(self, camera: str | None = None, *, start_timer: bool = True, apply_pending_setup: bool = False) -> None:
        return self._transport.start(camera, start_timer=start_timer, apply_pending_setup=apply_pending_setup)

    @QtCore.pyqtSlot()
    def start_receive_timer(self) -> None:
        """Start the Qt receive timer from the subscriber's owning Qt thread."""

        if QtCore.QThread.currentThread() is not self.thread():
            QtCore.QMetaObject.invokeMethod(
                self,
                "start_receive_timer",
                QtCore.Qt.QueuedConnection,
            )
            return
        if self.connections:
            self.timer.start()

    @QtCore.pyqtSlot()
    def stop_receive_timer(self) -> None:
        """Stop the receive timer only from the subscriber's Qt owner thread."""

        if QtCore.QThread.currentThread() is not self.thread():
            QtCore.QMetaObject.invokeMethod(
                self,
                "stop_receive_timer",
                QtCore.Qt.QueuedConnection,
            )
            return
        self.timer.stop()

    def _acquire_radio_endpoint_lock(self, side: str, endpoint: str):
        return self._transport._acquire_radio_endpoint_lock(side, endpoint)

    def _release_radio_endpoint_lock(self, side: str) -> None:
        return self._transport._release_radio_endpoint_lock(side)

    def _open_liveness_generation(self, side: str, connection: Any) -> MavlinkSideLiveness:
        return self._transport._open_liveness_generation(side, connection)

    def _ensure_liveness_listener(self, side: str) -> MavlinkSideLiveness:
        return self._transport._ensure_liveness_listener(side)

    def mavlink_health_snapshot(self, side: str, *, now_s: float | None = None) -> MavlinkSideHealthSnapshot:
        return self._transport.mavlink_health_snapshot(side, now_s=now_s)

    def mavlink_health_snapshots(self, *, now_s: float | None = None) -> dict[str, MavlinkSideHealthSnapshot]:
        return self._transport.mavlink_health_snapshots(now_s=now_s)

    def _clear_mavlink_imu_stream_state(
        self,
        camera: str,
        *,
        clear_samples: bool,
    ) -> None:
        camera = str(camera)
        keys = {(camera, message_name) for message_name in MAVLINK_UI_IMU_STREAM_NAMES}
        if clear_samples:
            for key in keys:
                self._last_imu_stream_message_s.pop(key, None)
        for key in keys:
            self._imu_stream_request_attempts.pop(key, None)
            self._imu_stream_last_request_s.pop(key, None)
        self._imu_stream_outage_reported.difference_update(keys)

    def stop(self, camera: str | None = None) -> None:
        return self._transport.stop(camera)

    def _reset_link_source_state(self, side: str) -> None:
        self._heartbeat_logged.discard(side)
        self._mavlink_stream_requests_sent.discard(side)
        self._mavlink_stream_request_attempts.pop(side, None)
        self._mavlink_stream_last_request_s.pop(side, None)
        self._sasl_can_mag_stream_confirmed.discard(side)
        self._last_attitude_message_s.pop(side, None)
        self._attitude_stream_request_attempts.pop(side, None)
        self._attitude_stream_last_request_s.pop(side, None)
        self._attitude_stream_outage_reported.discard(side)
        self._clear_mavlink_imu_stream_state(side, clear_samples=True)
        self._mavlink_stream_configured.discard(side)
        self._stream_setup_requests_sent.discard(side)
        self._latest_gnss_relative.pop(side, None)
        self._commands.clear(side)
        self._radio_stream_policies.setdefault(side, RadioStreamPolicy()).reset()
        self._radio_status.pop(side, None)

    def _reset_link_stream_requests(self, camera: str) -> None:
        self._mavlink_stream_requests_sent.discard(camera)
        self._mavlink_stream_request_attempts.pop(camera, None)
        self._mavlink_stream_last_request_s.pop(camera, None)
        self._clear_mavlink_imu_stream_state(camera, clear_samples=True)

    def resource_inventory(self) -> dict[str, object]:
        return self._transport.resource_inventory()

    def add_custom_watch(self, spec: dict[str, str]) -> None:
        if spec not in self.custom_watches:
            self.custom_watches.append(spec)

    def set_transport(self, side: str, transport: str) -> None:
        side = str(side)
        if side not in self.urls:
            raise ValueError(f"Unknown Pixhawk role: {side}")
        normalized = _normalize_transport(transport)
        if side in self.connections and normalized != self.transports.get(side):
            raise RuntimeError(f"{camera_label_ko(side)} transport cannot change while connected")
        self.transports[side] = normalized
        self._radio_stream_policies.setdefault(side, RadioStreamPolicy()).reset()

    def set_radio_rtcm_health(self, side: str, healthy: bool) -> None:
        self._radio_rtcm_healthy[str(side)] = bool(healthy)

    def radio_stream_policy_snapshot(self, side: str, *, now_s: float | None = None):
        observed_s = time.monotonic() if now_s is None else float(now_s)
        return self._radio_stream_policies[str(side)].snapshot(now_s=observed_s)

    def radio_status_snapshot(self, side: str) -> RadioStatusState | None:
        return self._radio_status.get(str(side))

    def _uses_minimal_radio_profile(self, camera: str) -> bool:
        return str(camera) in {"left", "right"} and self.transports.get(str(camera)) == "radio"

    def _active_stream_requests(self, camera: str) -> tuple[tuple[str, float], ...]:
        if self.transports.get(str(camera)) != "radio":
            return MAVLINK_STOCK_STREAM_REQUESTS
        if self._uses_minimal_radio_profile(camera):
            return RADIO_MINIMAL_STREAM_REQUESTS
        policy = self._radio_stream_policies.setdefault(str(camera), RadioStreamPolicy())
        return radio_stream_requests(policy.tier)

    def _advance_radio_stream_policy(self, camera: str, *, now_s: float) -> None:
        camera = str(camera)
        if self.transports.get(camera) != "radio" or self._uses_minimal_radio_profile(camera):
            return
        health = self.mavlink_health_snapshot(camera, now_s=now_s)
        qualified = health.state is MavlinkLivenessState.LIVE
        healthy = qualified and self._radio_rtcm_healthy.get(camera, True)
        policy = self._radio_stream_policies.setdefault(camera, RadioStreamPolicy())
        if not policy.observe(qualified=qualified, healthy=healthy, now_s=now_s):
            return
        self._mavlink_stream_requests_sent.discard(camera)
        self._clear_mavlink_imu_stream_state(camera, clear_samples=False)
        self._request_mavlink_sensor_streams(camera)
        self.status_changed.emit(f"{camera_label_ko(camera)} Radio optional stream tier={policy.tier.name}")

    def remove_custom_watch(self, key: str) -> None:
        self.custom_watches = [spec for spec in self.custom_watches if str(spec.get("key", "")) != key]

    def configure_mavlink_streams(
        self,
        *,
        host_ip: str,
        ports: dict[str, int],
        sysids: dict[str, int] | None = None,
        rate: int = 2_000_000,
        cameras: set[str] | tuple[str, ...] | list[str] | None = None,
    ) -> None:
        sysids = sysids or dict(MAVLINK_EXPECTED_SYSTEM_IDS)
        self._mavlink_stream_config = {
            "host_ip": host_ip,
            "ports": dict(ports),
            "sysids": dict(sysids),
            "rate": int(rate),
        }
        target_cameras = set(cameras) if cameras is not None else set(self.connections.keys())
        for camera, connection in self.connections.items():
            if camera not in target_cameras:
                continue
            try:
                with self.exclusive_command(camera):
                    self._apply_mavlink_stream_config(camera, connection)
            except Exception as exc:
                self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink stream 명령 슬롯 실패: {exc}")

    def read_param_int(self, camera: str, name: str, *, timeout_s: float = 2.0) -> int | None:
        return self._commands.read_param_int(camera, name, timeout_s=timeout_s)

    def read_param(self, camera: str, name: str, *, timeout_s: float = 2.0) -> int | float | None:
        return self._commands.read_param(camera, name, timeout_s=timeout_s)

    def set_param_int(self, camera: str, name: str, value: int, *, timeout_s: float = 2.0) -> int | None:
        return self._commands.set_param_int(camera, name, value, timeout_s=timeout_s)

    def send_rtcm_data(self, camera: str, payload: bytes) -> int:
        return self._commands.send_rtcm_data(camera, payload)

    def send_rtcm_data_if_eligible(
        self, camera: str, payload: bytes, *, expected_generation: int | None = None
    ) -> dict[str, object]:
        return self._commands.send_rtcm_data_if_eligible(camera, payload, expected_generation=expected_generation)

    def _emit_rtcm_status_if_due(self, camera: str, payload: bytes, packet_count: int) -> None:
        return self._commands._emit_rtcm_status_if_due(camera, payload, packet_count)

    def _apply_pending_connection_setup(self, camera: str, connection: Any) -> None:
        if self._mavlink_stream_config is not None and camera not in self._mavlink_stream_configured:
            self._apply_mavlink_stream_config(camera, connection)

    def _apply_mavlink_stream_config(self, camera: str, connection: Any) -> None:
        config = self._mavlink_stream_config
        if config is None:
            return
        port = config["ports"].get(camera)
        if port is None:
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink stream 설정 건너뜀: 포트 없음")
            return
        host_ip = config["host_ip"]
        rate = int(config["rate"])
        sysid = int(config["sysids"].get(camera, 1))
        command = (
            f"param set MAV_SYS_ID {sysid}\n"
            "param set MAV_2_CONFIG 1000\n"
            "param set MAV_2_MODE 0\n"
            "param set MAV_2_BROADCAST 1\n"
            f"param set MAV_2_REMOTE_PRT {int(port)}\n"
            f"param set MAV_2_UDP_PRT {int(port)}\n"
            f"param set MAV_2_RATE {rate}\n"
            "param save"
        )
        try:
            self._commands.exchange_shell(camera, command)
        except Exception as exc:
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink stream 설정 명령 실패: {exc}")
            return
        self._mavlink_stream_configured.add(camera)
        self._stream_setup_requests_sent.discard(camera)
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink 단일-instance 영구 설정 저장: sysid={sysid}, "
            f"host={host_ip}, port={int(port)} (재부팅 후 적용)"
        )

    def _request_mavlink_sensor_streams(self, camera: str) -> None:
        connection = self.connections.get(camera)
        if connection is None:
            return
        health = self.mavlink_health_snapshot(camera, now_s=time.monotonic())
        if health.state is MavlinkLivenessState.RX_STALLED:
            return
        if camera in self._mavlink_stream_requests_sent:
            return
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            return
        mavlink = getattr(connection, "mavlink", None)
        command_id = _mavlink_constant(mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", 511)
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        now_s = time.monotonic()
        requests = self._active_stream_requests(camera)
        requested = []
        self._mavlink_stream_requests_sent.add(camera)
        self._mavlink_stream_request_attempts[camera] = 1
        self._mavlink_stream_last_request_s[camera] = now_s
        try:
            with self._mavlink_send_locks.setdefault(camera, threading.RLock()):
                if self._uses_minimal_radio_profile(camera):
                    for message_id in RADIO_MINIMAL_DISABLED_MESSAGE_IDS:
                        mav.command_long_send(
                            target_system,
                            target_component,
                            command_id,
                            0,
                            message_id,
                            -1,
                            0,
                            0,
                            0,
                            0,
                            0,
                        )
                for message_name, rate_hz in requests:
                    message_id = MAVLINK_MESSAGE_IDS[message_name]
                    interval_us = int(1_000_000 / float(rate_hz))
                    mav.command_long_send(
                        target_system,
                        target_component,
                        command_id,
                        0,
                        int(message_id),
                        interval_us,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                    requested.append(message_name)
        except Exception as exc:
            self._mavlink_stream_requests_sent.discard(camera)
            self._mavlink_stream_request_attempts.pop(camera, None)
            self._mavlink_stream_last_request_s.pop(camera, None)
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink sensor stream 요청 실패: {exc}")
            return
        if "ATTITUDE" in requested:
            self._attitude_stream_request_attempts[camera] = 1
            self._attitude_stream_last_request_s[camera] = now_s
        for message_name in MAVLINK_UI_IMU_STREAM_NAMES:
            if message_name not in requested:
                continue
            key = (camera, message_name)
            self._imu_stream_request_attempts[key] = 1
            self._imu_stream_last_request_s[key] = now_s
            self._imu_stream_outage_reported.discard(key)
        profile = (
            "Radio Minimal"
            if self._uses_minimal_radio_profile(camera)
            else f"Radio {self._radio_stream_policies[camera].tier.name}"
            if self.transports.get(camera) == "radio"
            else "Ethernet Stock"
        )
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink {profile} sensor stream 요청: {', '.join(requested)}"
        )

    def _request_missing_mavlink_imu_streams(self, camera: str) -> None:
        """Bound retries to UI-backed IMU streams with no fresh sample."""

        camera = str(camera)
        connection = self.connections.get(camera)
        if connection is None or camera not in self._mavlink_stream_requests_sent:
            return
        now_s = time.monotonic()
        health = self.mavlink_health_snapshot(camera, now_s=now_s)
        if health.state is MavlinkLivenessState.RX_STALLED:
            return
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            return

        due: list[tuple[str, tuple[str, str], int]] = []
        active_rates = dict(self._active_stream_requests(camera))
        for message_name in MAVLINK_UI_IMU_STREAM_NAMES:
            if message_name not in active_rates:
                continue
            key = (camera, message_name)
            last_sample_s = self._last_imu_stream_message_s.get(key)
            if last_sample_s is not None and now_s - last_sample_s <= MAVLINK_IMU_STREAM_STALE_S:
                self._imu_stream_request_attempts.pop(key, None)
                self._imu_stream_last_request_s.pop(key, None)
                self._imu_stream_outage_reported.discard(key)
                continue
            attempts = self._imu_stream_request_attempts.get(key, 0)
            last_request_s = self._imu_stream_last_request_s.get(
                key,
                float("-inf"),
            )
            if attempts >= MAVLINK_STREAM_REQUEST_MAX_ATTEMPTS:
                if now_s - last_request_s < MAVLINK_IMU_STREAM_RECOVERY_COOLDOWN_S:
                    if key not in self._imu_stream_outage_reported:
                        self._imu_stream_outage_reported.add(key)
                        self.status_changed.emit(
                            f"{camera_label_ko(camera)} {message_name} stream 응답이 없습니다. 재시도를 대기합니다."
                        )
                    continue
                attempts = 0
                self._imu_stream_outage_reported.discard(key)
            if now_s - last_request_s < MAVLINK_STREAM_REQUEST_RETRY_S:
                continue
            due.append((message_name, key, attempts))
        if not due:
            return

        mavlink = getattr(connection, "mavlink", None)
        command_id = _mavlink_constant(
            mavlink,
            "MAV_CMD_SET_MESSAGE_INTERVAL",
            511,
        )
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        requested: list[str] = []
        try:
            with self._mavlink_send_locks.setdefault(camera, threading.RLock()):
                for message_name, key, attempts in due:
                    rate_hz = active_rates[message_name]
                    mav.command_long_send(
                        target_system,
                        target_component,
                        command_id,
                        0,
                        MAVLINK_MESSAGE_IDS[message_name],
                        int(1_000_000 / float(rate_hz)),
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                    self._imu_stream_request_attempts[key] = attempts + 1
                    self._imu_stream_last_request_s[key] = now_s
                    requested.append(message_name)
        except Exception as exc:
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink IMU stream 요청 실패: {exc}")
            return
        self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink 누락 IMU stream 재요청: {', '.join(requested)}")

    def _request_stale_attitude_stream(self, camera: str) -> None:
        """Re-request ATTITUDE while heartbeat proves the transport is alive."""

        connection = self.connections.get(camera)
        if connection is None:
            return
        health = self.mavlink_health_snapshot(camera, now_s=time.monotonic())
        if health.state is MavlinkLivenessState.RX_STALLED:
            return
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            return
        now_s = time.monotonic()
        last_attitude_s = self._last_attitude_message_s.get(camera)
        if last_attitude_s is not None and now_s - last_attitude_s <= MAVLINK_ATTITUDE_STALE_S:
            self._attitude_stream_request_attempts.pop(camera, None)
            self._attitude_stream_last_request_s.pop(camera, None)
            self._attitude_stream_outage_reported.discard(camera)
            return
        attempts = self._attitude_stream_request_attempts.get(camera, 0)
        last_request_s = self._attitude_stream_last_request_s.get(camera, float("-inf"))
        if attempts >= MAVLINK_STREAM_REQUEST_MAX_ATTEMPTS:
            if now_s - last_request_s < MAVLINK_ATTITUDE_RECOVERY_COOLDOWN_S:
                if camera not in self._attitude_stream_outage_reported:
                    self._attitude_stream_outage_reported.add(camera)
                    self.status_changed.emit(
                        f"{camera_label_ko(camera)} MAVLink heartbeat는 수신 중이나 "
                        "ATTITUDE stream 복구 응답이 없습니다. 재시도를 대기합니다."
                    )
                return
            attempts = 0
            self._attitude_stream_outage_reported.discard(camera)
        if now_s - last_request_s < MAVLINK_STREAM_REQUEST_RETRY_S:
            return
        mavlink = getattr(connection, "mavlink", None)
        command_id = _mavlink_constant(mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", 511)
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        self._attitude_stream_request_attempts[camera] = attempts + 1
        self._attitude_stream_last_request_s[camera] = now_s
        try:
            mav.command_long_send(
                target_system,
                target_component,
                command_id,
                0,
                MAVLINK_MESSAGE_IDS["ATTITUDE"],
                int(1_000_000 / (5.0 if self._uses_minimal_radio_profile(camera) else 30.0)),
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink ATTITUDE stream 복구 요청 실패: {exc}")
            return
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink ATTITUDE stream 복구 요청 "
            f"({attempts + 1}/{MAVLINK_STREAM_REQUEST_MAX_ATTEMPTS})"
        )

    def send_shell_command(self, camera: str, command: str) -> None:
        return self._commands.send_shell_command(camera, command)

    def send_shell_command_with_result(
        self, camera: str, command: str, *, timeout_s: float = 2.0
    ) -> MavlinkShellCommandResult:
        return self._commands.send_shell_command_with_result(camera, command, timeout_s=timeout_s)

    def write_remote_file_mavftp(self, camera: str, remote_path: str, content: str | bytes) -> None:
        return self._commands.write_remote_file_mavftp(camera, remote_path, content)

    def reboot_autopilot(self, camera: str) -> None:
        return self._commands.reboot_autopilot(camera)

    def spin_once(self) -> None:
        return self._transport.spin_once()

    def _defer_command_message(self, camera: str, message: Any) -> None:
        return self._transport._defer_command_message(camera, message)

    def _observe_mavlink_transport_message(
        self, camera: str, message: Any, *, received_at_s: float | None = None
    ) -> bool:
        return self._transport._observe_mavlink_transport_message(camera, message, received_at_s=received_at_s)

    def _send_due_gcs_heartbeat(
        self, camera: str, connection: Any, *, now_s: float, wait_for_send_lock: bool = True
    ) -> None:
        return self._transport._send_due_gcs_heartbeat(
            camera, connection, now_s=now_s, wait_for_send_lock=wait_for_send_lock
        )

    def _emit_liveness_transition(self, camera: str, *, now_s: float) -> None:
        return self._transport._emit_liveness_transition(camera, now_s=now_s)

    def _is_duplicate_packet(self, camera: str, message: Any) -> bool:
        return self._transport._is_duplicate_packet(camera, message)

    def _emit_receive_error(self, camera: str, exc: BaseException) -> None:
        return self._transport._emit_receive_error(camera, exc)

    def _log_unrecognized_debug_vect(self, camera: str, message: Any) -> None:
        """Surface DEBUG_VECT names the CAN-mag decoder doesn't recognize.

        Only "CAN1_MAG"/"CAN2_MAG" are decoded (see
        _can_mag_sensor_state_from_debug_vect); any other firmware naming was
        previously dropped silently, making a firmware-side naming mismatch
        indistinguishable from no CAN mag data being sent at all.
        """

        name = getattr(message, "name", "")
        if isinstance(name, bytes):
            name = name.split(b"\0", 1)[0].decode("ascii", errors="ignore")
        name = str(name).strip(" \0")
        key = f"{camera}:{name}"
        if key in self._unrecognized_debug_vect_names:
            return
        self._unrecognized_debug_vect_names.add(key)
        self.status_changed.emit(
            f"{camera_label_ko(camera)} 인식되지 않는 DEBUG_VECT 이름 수신: '{name}' "
            "(CAN1_MAG/CAN2_MAG 아님 - CAN 자력계 데이터가 아닐 수 있음)"
        )

    def _observe_remote_px4_reboot(
        self, camera: str, message: Any, *, received_at_s: float, provenance: dict[str, int | None]
    ) -> bool:
        return self._transport._observe_remote_px4_reboot(
            camera, message, received_at_s=received_at_s, provenance=provenance
        )

    def _handle_mavlink_message(
        self,
        camera: str,
        message: Any,
        *,
        received_at_s: float | None = None,
        allow_heartbeat_actions: bool = True,
    ) -> None:
        message_type = message.get_type() if hasattr(message, "get_type") else ""
        namespace = self.urls.get(camera, "")
        now_s = time.monotonic() if received_at_s is None else float(received_at_s)
        provenance = mavlink_message_provenance(message)
        if message_type == "ATTITUDE" and not self._observe_remote_px4_reboot(
            camera,
            message,
            received_at_s=now_s,
            provenance=provenance,
        ):
            return
        self._emit_custom_watch_samples(camera, message, message_type, now_s)
        optional_row = _mavlink_optional_timeline_row(
            message_type,
            message,
            now_s,
            provenance=provenance,
        )
        if optional_row is not None:
            self.optional_timeline_changed.emit(camera, optional_row)
        if message_type == "RADIO_STATUS":
            self._radio_status[camera] = RadioStatusState(
                timestamp_s=now_s,
                rssi=_optional_int(message, "rssi"),
                remote_rssi=_optional_int(message, "remrssi"),
                noise=_optional_int(message, "noise"),
                remote_noise=_optional_int(message, "remnoise"),
                txbuf_pct=_optional_int(message, "txbuf"),
                rxerrors=_optional_int(message, "rxerrors"),
                fixed=_optional_int(message, "fixed"),
            )
            return
        if message_type == "ATTITUDE":
            self._last_attitude_message_s[camera] = now_s
            self._attitude_stream_request_attempts.pop(camera, None)
            self._attitude_stream_last_request_s.pop(camera, None)
            self._attitude_stream_outage_reported.discard(camera)
            roll_rad = _optional_float(message, "roll")
            pitch_rad = _optional_float(message, "pitch")
            yaw_rad = _optional_float(message, "yaw")
            if roll_rad is None or pitch_rad is None or yaw_rad is None:
                # Non-finite (NaN/Inf) attitude values: drop this malformed
                # message rather than propagate a corrupt extrinsic.
                return
            extrinsic = camera_extrinsic_from_euler_rad(
                camera=camera,
                namespace=namespace,
                roll_rad=roll_rad,
                pitch_rad=pitch_rad,
                yaw_rad=yaw_rad,
                timestamp_s=now_s,
            )
            time_boot_ms = getattr(message, "time_boot_ms", None)
            if time_boot_ms is not None:
                px4_timestamp_us = int(time_boot_ms) * 1000
                extrinsic = replace(
                    extrinsic,
                    px4_timestamp_s=float(px4_timestamp_us) * 1e-6,
                    px4_timestamp_us=px4_timestamp_us,
                )
            extrinsic = replace(
                extrinsic,
                rollspeed_rad_s=_optional_float(message, "rollspeed"),
                pitchspeed_rad_s=_optional_float(message, "pitchspeed"),
                yawspeed_rad_s=_optional_float(message, "yawspeed"),
                **provenance,
            )
            self.extrinsic_changed.emit(camera, extrinsic)
            return
        if message_type in {"GPS_RAW_INT", "GPS2_RAW"}:
            topic = f"mavlink/{message_type}"
            source_time_usec = getattr(message, "time_usec", None)
            source_time_us = int(source_time_usec) if source_time_usec is not None else None
            gps_utc_us = (
                source_time_us if source_time_us is not None and source_time_us >= 946_684_800_000_000 else None
            )
            gps = GpsPositionState(
                camera=camera,
                namespace=namespace,
                timestamp_s=now_s,
                topic=topic,
                latitude_deg=_mavlink_scaled_degrees(getattr(message, "lat", None)),
                longitude_deg=_mavlink_scaled_degrees(getattr(message, "lon", None)),
                altitude_m=_mavlink_scaled_millimetres(getattr(message, "alt", None)),
                fix_type=_optional_int(message, "fix_type"),
                satellites_used=_optional_int(message, "satellites_visible"),
                # PX4 GPS_RAW_INT.eph/epv are HDOP/VDOP ×100. The actual
                # metre accuracies are carried by the h_acc/v_acc extensions.
                eph_m=_mavlink_scaled_accuracy_mm(getattr(message, "h_acc", None)),
                epv_m=_mavlink_scaled_accuracy_mm(getattr(message, "v_acc", None)),
                heading_deg=_mavlink_heading_cdeg(getattr(message, "cog", None)),
                raw=message,
                gps_utc_us=gps_utc_us,
                altitude_ellipsoid_m=_mavlink_scaled_millimetres(getattr(message, "alt_ellipsoid", None)),
                hdop=_mavlink_scaled_dop(getattr(message, "eph", None)),
                vdop=_mavlink_scaled_dop(getattr(message, "epv", None)),
                time_source="mavlink_time_usec_epoch" if gps_utc_us is not None else "mavlink_time_usec_unresolved",
                **provenance,
            )
            if message_type == "GPS_RAW_INT":
                self.gps_changed.emit(camera, gps)
            else:
                self.gps_changed.emit(f"{camera}_GPS2", gps)
            self.gps_sensor_changed.emit(
                camera,
                replace(
                    _gps_sensor_state_from_mavlink(camera, namespace, message, topic, now_s),
                    **provenance,
                ),
            )
            gnss_relative = _gnss_relative_state_from_mavlink_gps_raw(camera, namespace, message, topic, now_s)
            if gnss_relative is not None:
                self._emit_mavlink_gnss_relative(camera, replace(gnss_relative, **provenance))
            return
        if message_type in {"GPS_RTK", "GPS2_RTK"}:
            gnss_relative = _gnss_relative_state_from_mavlink_rtk(
                camera,
                namespace,
                message,
                f"mavlink/{message_type}",
                now_s,
            )
            if gnss_relative is not None:
                self._emit_mavlink_gnss_relative(camera, replace(gnss_relative, **provenance))
            return
        if message_type in {"SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"}:
            mag = _mag_sensor_state_from_mavlink(camera, namespace, message, message_type, now_s)
            if mag is not None:
                key = (camera, message_type)
                self._last_imu_stream_message_s[key] = now_s
                self._imu_stream_request_attempts.pop(key, None)
                self._imu_stream_last_request_s.pop(key, None)
                self._imu_stream_outage_reported.discard(key)
                self.mag_sensor_changed.emit(camera, replace(mag, **provenance))
            return
        if message_type in MAVLINK_CAN_MAG_MESSAGE_TYPES:
            if message_type == "SASL_CAN_MAG":
                self._sasl_can_mag_stream_confirmed.add(camera)
            mag = _can_mag_sensor_state_from_mavlink(camera, namespace, message, message_type, now_s)
            if mag is not None:
                self.mag_sensor_changed.emit(camera, replace(mag, **provenance))
            return
        if message_type == "DEBUG_VECT":
            mag = _can_mag_sensor_state_from_debug_vect(camera, namespace, message, now_s)
            if mag is not None:
                self.mag_sensor_changed.emit(camera, replace(mag, **provenance))
            else:
                self._log_unrecognized_debug_vect(camera, message)
            return
        if message_type == "HEARTBEAT":
            # Command-owned recv_match records liveness at ingress and queues
            # telemetry with that exact receive timestamp. Replaying an old
            # HEARTBEAT must preserve downstream order/logging, but must not
            # run current-time stream setup or recovery for every historical
            # packet. The next live HEARTBEAT owns those actions.
            if allow_heartbeat_actions:
                self._request_mavlink_sensor_streams(camera)
                self._request_missing_mavlink_imu_streams(camera)
                self._request_stale_attitude_stream(camera)
            if camera not in self._heartbeat_logged:
                self._heartbeat_logged.add(camera)
                self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink heartbeat 수신")
            return
        if message_type == "SERIAL_CONTROL":
            output = _decode_mavlink_serial_control(message)
            if output:
                self.shell_output.emit(camera, output)

    def _emit_mavlink_gnss_relative(self, camera: str, state: GnssRelativeState) -> None:
        merged = _merge_gnss_relative_state(self._latest_gnss_relative.get(camera), state)
        self._latest_gnss_relative[camera] = merged
        self.gnss_relative_changed.emit(camera, merged)

    def _emit_custom_watch_samples(self, camera: str, message: Any, message_type: str, timestamp_s: float) -> None:
        if not self.custom_watches:
            return
        for spec in self.custom_watches:
            source = str(spec.get("source", "")).strip().lower()
            if source and source != "mavlink":
                continue
            target_message = str(spec.get("message", "") or spec.get("topic", "")).strip().upper()
            if target_message and target_message != message_type.upper():
                continue
            field_path = str(spec.get("field", "")).strip()
            if not field_path:
                continue
            try:
                value = extract_field_path(message, field_path)
            except Exception:
                continue
            numeric_value = _coerce_numeric(value)
            key = str(spec.get("key") or f"mavlink:{message_type}.{field_path}")
            self.custom_watch_changed.emit(
                CustomTopicSample(
                    key=f"{camera}:{key}",
                    topic=f"mavlink/{message_type}",
                    field_path=field_path,
                    value_text=_format_custom_value(value),
                    numeric_value=numeric_value,
                    timestamp_s=timestamp_s,
                )
            )


# MAV_PARAM_TYPE_REAL32 = 9, MAV_PARAM_TYPE_REAL64 = 10. Every other MAV_PARAM_TYPE is an
# integer whose bits are reinterpreted (not numerically cast) into the PARAM_VALUE float field,
# per the MAVLink param protocol convention PX4 uses.
