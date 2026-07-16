from __future__ import annotations

import glob
import math
import os
import io
import re
import struct
import subprocess
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from wiris_stereo_distance.rtk_geometry import (
    GPS_SOURCE_GLOBAL_POSITION,
    GPS_SOURCE_SENSOR_GPS,
    GpsPositionState,
    global_position_topic,
    gps_state_from_message,
    gps_topic,
)

from PyQt5 import QtCore

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover - depends on sourced ROS2 environment.
    rclpy = None
    SingleThreadedExecutor = None
    DurabilityPolicy = None
    HistoryPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    get_message = None


@dataclass(frozen=True)
class EulerAttitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class CameraExtrinsicState:
    camera: str
    namespace: str
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    timestamp_s: float
    topic: str
    q: tuple[float, float, float, float]
    px4_timestamp_s: float | None = None


@dataclass(frozen=True)
class MagSensorState:
    camera: str
    namespace: str
    sensor_slot: str
    timestamp_s: float
    topic: str
    device_id: int | None = None
    x: float | None = None
    y: float | None = None
    z: float | None = None
    magnitude: float | None = None
    temperature_c: float | None = None
    error_count: int | None = None
    px4_timestamp_s: float | None = None


@dataclass(frozen=True)
class GpsSensorState:
    camera: str
    namespace: str
    sensor_slot: str
    timestamp_s: float
    topic: str
    device_id: int | None = None
    fix_type: int | None = None
    satellites_used: int | None = None
    eph_m: float | None = None
    epv_m: float | None = None
    px4_timestamp_s: float | None = None


@dataclass(frozen=True)
class GnssRelativeState:
    camera: str
    namespace: str
    timestamp_s: float
    topic: str
    device_id: int | None = None
    heading_rad: float | None = None
    heading_accuracy_rad: float | None = None
    position_length_m: float | None = None
    accuracy_length_m: float | None = None
    gnss_fix_ok: bool | None = None
    differential_solution: bool | None = None
    relative_position_valid: bool | None = None
    carrier_solution_floating: bool | None = None
    carrier_solution_fixed: bool | None = None
    moving_base_mode: bool | None = None
    heading_valid: bool | None = None
    px4_timestamp_s: float | None = None


@dataclass(frozen=True)
class CustomTopicSample:
    key: str
    topic: str
    field_path: str
    value_text: str
    numeric_value: float | None
    timestamp_s: float


@dataclass(frozen=True)
class PixhawkTopicDiscovery:
    domain_id: int
    namespaces: tuple[str, ...]
    topics: tuple[str, ...]


def quaternion_to_euler_deg(q: list[float] | tuple[float, float, float, float]) -> EulerAttitude:
    w, x, y, z = (float(value) for value in q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return EulerAttitude(roll, pitch, yaw)


def euler_rad_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def camera_extrinsic_from_euler_rad(
    *,
    camera: str,
    namespace: str,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    timestamp_s: float | None = None,
    topic: str = "mavlink/ATTITUDE",
) -> CameraExtrinsicState:
    q = euler_rad_to_quaternion(roll_rad, pitch_rad, yaw_rad)
    attitude = quaternion_to_euler_deg(q)
    return CameraExtrinsicState(
        camera=camera,
        namespace=namespace.strip(),
        roll_deg=attitude.roll_deg,
        pitch_deg=attitude.pitch_deg,
        yaw_deg=attitude.yaw_deg,
        timestamp_s=time.monotonic() if timestamp_s is None else float(timestamp_s),
        topic=topic,
        q=q,
        px4_timestamp_s=None,
    )


def attitude_topic(namespace: str) -> str:
    normalized = namespace.strip().strip("/")
    return f"/{normalized}/fmu/out/vehicle_attitude" if normalized else "/fmu/out/vehicle_attitude"


def sensor_mag_topic(namespace: str) -> str:
    normalized = namespace.strip().strip("/")
    return f"/{normalized}/fmu/out/sensor_mag" if normalized else "/fmu/out/sensor_mag"


def sensor_mag_topics(namespace: str) -> tuple[str, ...]:
    normalized = namespace.strip().strip("/")
    prefix = f"/{normalized}/fmu/out" if normalized else "/fmu/out"
    return (
        f"{prefix}/sensor_mag",
        f"{prefix}/sensor_mag_0",
        f"{prefix}/sensor_mag_1",
        f"{prefix}/sensor_mag_2",
        f"{prefix}/sensor_mag_3",
        f"{prefix}/sensor_mag_4",
    )


def vehicle_magnetometer_topic(namespace: str) -> str:
    normalized = namespace.strip().strip("/")
    return f"/{normalized}/fmu/out/vehicle_magnetometer" if normalized else "/fmu/out/vehicle_magnetometer"


def sensor_gps_topic(namespace: str) -> str:
    normalized = namespace.strip().strip("/")
    return f"/{normalized}/fmu/out/sensor_gps" if normalized else "/fmu/out/sensor_gps"


def sensor_gps_topics(namespace: str) -> tuple[str, ...]:
    normalized = namespace.strip().strip("/")
    prefix = f"/{normalized}/fmu/out" if normalized else "/fmu/out"
    return (
        f"{prefix}/sensor_gps",
        f"{prefix}/sensor_gps_0",
        f"{prefix}/sensor_gps_1",
    )


def sensor_gnss_relative_topic(namespace: str) -> str:
    normalized = namespace.strip().strip("/")
    return f"/{normalized}/fmu/out/sensor_gnss_relative" if normalized else "/fmu/out/sensor_gnss_relative"


MAVLINK_MESSAGE_IDS = {
    "ATTITUDE": 30,
    "GPS_RAW_INT": 24,
    "GPS_RTK": 127,
    "GPS2_RAW": 124,
    "GPS2_RTK": 128,
    "SCALED_IMU": 26,
    "SCALED_IMU2": 116,
    "SCALED_IMU3": 129,
}
MAVLINK_RM3100_STREAM_REQUESTS = (
    ("ATTITUDE", 30.0),
    ("GPS_RAW_INT", 5.0),
    ("GPS2_RAW", 5.0),
    ("GPS_RTK", 5.0),
    ("GPS2_RTK", 5.0),
    ("SCALED_IMU", 20.0),
    ("SCALED_IMU2", 20.0),
    ("SCALED_IMU3", 20.0),
)


def camera_label_ko(camera: str) -> str:
    return {"left": "왼쪽", "right": "오른쪽"}.get(camera, camera)


def discover_pixhawk_topics(
    *,
    current_domain_id: int = 0,
    timeout_s: float = 2.0,
    extra_domain_ids: tuple[int, ...] = tuple(range(0, 11)),
) -> PixhawkTopicDiscovery | None:
    domain_ids = _domain_scan_order(current_domain_id, extra_domain_ids)
    for domain_id in domain_ids:
        topics = _ros2_topic_list(domain_id, timeout_s=timeout_s)
        px4_topics = tuple(
            topic for topic in topics if "/fmu/out/" in topic and _ros2_topic_has_publisher(domain_id, topic, timeout_s=timeout_s)
        )
        if not px4_topics:
            continue
        namespaces = _pixhawk_namespaces(px4_topics)
        return PixhawkTopicDiscovery(domain_id=domain_id, namespaces=namespaces, topics=px4_topics)
    return None


def discover_ros2_topic_types(domain_id: int, *, timeout_s: float = 1.5) -> list[dict[str, str]]:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(int(domain_id))
    try:
        result = subprocess.run(
            ["ros2", "topic", "list", "-t"],
            check=False,
            capture_output=True,
            text=True,
            env=env,
            timeout=timeout_s,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    if result.returncode != 0:
        return []
    topics: list[dict[str, str]] = []
    for line in result.stdout.splitlines():
        text = line.strip()
        if not text:
            continue
        match = re.match(r"^(?P<name>\S+)\s+\[(?P<type>[^\]]+)\]", text)
        if match:
            topics.append({"name": match.group("name"), "type": match.group("type")})
        else:
            topics.append({"name": text, "type": ""})
    return topics


def describe_ros2_message_fields(ros2_type: str) -> list[str]:
    if get_message is None:
        raise RuntimeError("rosidl_runtime_py를 사용할 수 없어 ROS2 메시지 field를 조회할 수 없습니다.")
    message_type = get_message(ros2_type)
    fields: list[str] = []
    _flatten_field_schema(fields, "", message_type)
    return fields


def _flatten_field_schema(output: list[str], prefix: str, value: Any) -> None:
    field_names: list[str] = []
    if hasattr(value, "get_fields_and_field_types"):
        field_names = list(value.get_fields_and_field_types().keys())
    elif hasattr(value, "__slots__"):
        field_names = [slot.lstrip("_") for slot in value.__slots__]
    if not field_names and prefix:
        output.append(prefix)
        return
    for field_name in field_names:
        child = getattr(value, field_name, None)
        child_prefix = f"{prefix}:{field_name}" if prefix else field_name
        if isinstance(child, (bool, int, float)) or child is None:
            output.append(child_prefix)
        elif isinstance(child, str):
            continue
        elif isinstance(child, (list, tuple)):
            if child:
                for index, item in enumerate(child[:3]):
                    if isinstance(item, (bool, int, float)):
                        output.append(f"{child_prefix}[{index}]")
                    else:
                        _flatten_field_schema(output, f"{child_prefix}[{index}]", item)
            else:
                output.append(f"{child_prefix}[0]")
        else:
            _flatten_field_schema(output, child_prefix, child)


def _domain_scan_order(current_domain_id: int, extra_domain_ids: tuple[int, ...]) -> tuple[int, ...]:
    ordered = [int(current_domain_id)]
    ordered.extend(int(domain_id) for domain_id in extra_domain_ids)
    seen: set[int] = set()
    result: list[int] = []
    for domain_id in ordered:
        if domain_id < 0 or domain_id > 232 or domain_id in seen:
            continue
        seen.add(domain_id)
        result.append(domain_id)
    return tuple(result)


def _ros2_topic_list(domain_id: int, *, timeout_s: float) -> tuple[str, ...]:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            check=False,
            capture_output=True,
            text=True,
            env=env,
            timeout=timeout_s,
        )
    except (OSError, subprocess.TimeoutExpired):
        return ()
    if result.returncode != 0:
        return ()
    return tuple(line.strip() for line in result.stdout.splitlines() if line.strip())


def _ros2_topic_has_publisher(domain_id: int, topic: str, *, timeout_s: float) -> bool:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic],
            check=False,
            capture_output=True,
            text=True,
            env=env,
            timeout=timeout_s,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    if result.returncode != 0:
        return False
    match = re.search(r"Publisher count:\s*(\d+)", result.stdout)
    return bool(match and int(match.group(1)) > 0)


def _pixhawk_namespaces(topics: tuple[str, ...]) -> tuple[str, ...]:
    namespaces: list[str] = []
    for topic in topics:
        match = re.match(r"^/(.*?)/fmu/out/", topic)
        if match:
            namespace = match.group(1).strip("/")
        elif topic.startswith("/fmu/out/"):
            namespace = ""
        else:
            continue
        if namespace not in namespaces:
            namespaces.append(namespace)
    return tuple(namespaces)


def camera_extrinsic_from_quaternion(
    *,
    camera: str,
    namespace: str,
    q: list[float] | tuple[float, float, float, float],
    timestamp_s: float | None = None,
) -> CameraExtrinsicState:
    attitude = quaternion_to_euler_deg(q)
    topic = attitude_topic(namespace)
    return CameraExtrinsicState(
        camera=camera,
        namespace=namespace.strip().strip("/"),
        roll_deg=attitude.roll_deg,
        pitch_deg=attitude.pitch_deg,
        yaw_deg=attitude.yaw_deg,
        timestamp_s=time.monotonic() if timestamp_s is None else float(timestamp_s),
        topic=topic,
        q=tuple(float(value) for value in q),
    )


def _optional_float(message: Any, *names: str) -> float | None:
    for name in names:
        if hasattr(message, name):
            value = getattr(message, name)
            if isinstance(value, (list, tuple)):
                return None
            return float(value)
    return None


def _optional_int(message: Any, *names: str) -> int | None:
    for name in names:
        if hasattr(message, name):
            return int(getattr(message, name))
    return None


def _optional_bool(message: Any, *names: str) -> bool | None:
    for name in names:
        if hasattr(message, name):
            return bool(getattr(message, name))
    return None


def _qos_for_px4_out():
    if QoSProfile is None:
        return 10
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


DEFAULT_USB_MAVLINK_BAUDRATES = (57600, 115200, 921600)


@dataclass(frozen=True)
class MavlinkShellCommandResult:
    command: str
    output: str
    flag_label: str


def usb_mavlink_device_candidates(
    patterns: tuple[str, ...] = ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"),
) -> list[str]:
    candidates: list[str] = []
    targets_seen: set[str] = set()
    for pattern in patterns:
        for candidate in sorted(glob.glob(pattern)):
            target = os.path.realpath(candidate)
            if target in targets_seen:
                continue
            targets_seen.add(target)
            candidates.append(candidate)
    return candidates


def probe_usb_mavlink_devices(
    candidates: list[str] | None = None,
    *,
    baudrates: tuple[int, ...] = DEFAULT_USB_MAVLINK_BAUDRATES,
    timeout_s: float = 1.0,
) -> list[tuple[str, int]]:
    try:
        from pymavlink import mavutil
    except Exception as exc:
        raise RuntimeError(f"pymavlink를 사용할 수 없어 USB Pixhawk 자동 탐지를 할 수 없습니다. ({exc})") from exc
    found: list[tuple[str, int]] = []
    for device in candidates if candidates is not None else usb_mavlink_device_candidates():
        for baud in baudrates:
            connection = None
            try:
                connection = mavutil.mavlink_connection(device, baud=int(baud), autoreconnect=False, robust_parsing=True)
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GENERIC,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                if connection.wait_heartbeat(timeout=timeout_s) is not None:
                    found.append((device, int(baud)))
                    break
            except Exception:
                continue
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
    return found


def extract_field_path(message: Any, field_path: str) -> Any:
    value: Any = message
    for part in re.split(r"[.:]", field_path):
        part = part.strip()
        if not part:
            continue
        match = re.fullmatch(r"([^\[]+)(?:\[(\d+)\])?", part)
        if not match:
            raise ValueError(f"지원하지 않는 field segment: {part}")
        name, index_text = match.groups()
        value = getattr(value, name)
        if index_text is not None:
            value = value[int(index_text)]
    return value


def _coerce_numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return float(int(value))
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _format_custom_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_format_custom_value(item) for item in value[:6]) + (", ..." if len(value) > 6 else "") + "]"
    return str(value)


class DualPixhawkAttitudeSubscriber(QtCore.QObject):
    extrinsic_changed = QtCore.pyqtSignal(str, object)
    gps_changed = QtCore.pyqtSignal(str, object)
    mag_sensor_changed = QtCore.pyqtSignal(str, object)
    gps_sensor_changed = QtCore.pyqtSignal(str, object)
    gnss_relative_changed = QtCore.pyqtSignal(str, object)
    custom_watch_changed = QtCore.pyqtSignal(object)
    status_changed = QtCore.pyqtSignal(str)

    def __init__(
        self,
        *,
        domain_id: int,
        left_namespace: str,
        right_namespace: str,
        custom_watches: tuple[dict[str, str], ...] = (),
        active_sides: tuple[str, ...] = ("left", "right"),
    ) -> None:
        super().__init__()
        self.domain_id = int(domain_id)
        self.namespaces = {
            "left": left_namespace.strip().strip("/"),
            "right": right_namespace.strip().strip("/"),
        }
        self.active_sides = tuple(side for side in active_sides if side in self.namespaces)
        if not self.active_sides:
            self.active_sides = ("left", "right")
        self.node: Any = None
        self.executor: Any = None
        self.context: Any = None
        self.subscriptions: list[Any] = []
        self.custom_watches = list(custom_watches)
        self.custom_watch_subscriptions: dict[str, Any] = {}
        self.latest_extrinsics: dict[str, CameraExtrinsicState] = {}
        self.latest_gps: dict[str, GpsPositionState] = {}
        self.latest_gps_by_source: dict[str, dict[str, GpsPositionState]] = {
            GPS_SOURCE_SENSOR_GPS: {},
            GPS_SOURCE_GLOBAL_POSITION: {},
        }
        self.attitude_buffers: dict[str, deque[CameraExtrinsicState]] = {
            "left": deque(maxlen=1000),
            "right": deque(maxlen=1000),
        }
        self.gps_buffers: dict[str, deque[GpsPositionState]] = {
            "left": deque(maxlen=500),
            "right": deque(maxlen=500),
        }
        self.gps_buffers_by_source: dict[str, dict[str, deque[GpsPositionState]]] = {
            GPS_SOURCE_SENSOR_GPS: {"left": deque(maxlen=500), "right": deque(maxlen=500)},
            GPS_SOURCE_GLOBAL_POSITION: {"left": deque(maxlen=500), "right": deque(maxlen=500)},
        }
        self.mag_device_slots: dict[str, dict[int, str]] = {"left": {}, "right": {}}
        self.gps_device_slots: dict[str, dict[int, str]] = {"left": {}, "right": {}}
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.spin_once)

    def start(self) -> None:
        if rclpy is None or SingleThreadedExecutor is None or get_message is None:
            raise RuntimeError("ROS2 Python 모듈을 사용할 수 없습니다. ROS2와 px4_msgs 환경을 먼저 source 해주세요.")
        import os

        os.environ["ROS_DOMAIN_ID"] = str(self.domain_id)
        if hasattr(rclpy, "context") and hasattr(rclpy.context, "Context"):
            self.context = rclpy.context.Context()
            rclpy.init(args=None, context=self.context)
        else:
            self.context = None
            if not rclpy.ok():
                rclpy.init(args=None)
        msg_type = self._optional_message_type("px4_msgs/msg/VehicleAttitude")
        sensor_mag_msg_type = self._optional_message_type("px4_msgs/msg/SensorMag")
        vehicle_mag_msg_type = self._optional_message_type("px4_msgs/msg/VehicleMagnetometer")
        sensor_gps_msg_type = self._optional_message_type("px4_msgs/msg/SensorGps")
        gnss_relative_msg_type = self._optional_message_type("px4_msgs/msg/SensorGnssRelative")
        gps_msg_type = self._optional_message_type("px4_msgs/msg/VehicleGpsPosition", quiet=True)
        global_position_msg_type = self._optional_message_type("px4_msgs/msg/VehicleGlobalPosition", quiet=True)
        side_suffix = "_".join(self.active_sides)
        node_name = f"wiris_stereo_{side_suffix}_pixhawk_attitude"
        if self.context is not None:
            self.node = rclpy.create_node(node_name, context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
        else:
            self.node = rclpy.create_node(node_name)
            self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.subscriptions = []
        for camera in self.active_sides:
            namespace = self.namespaces[camera]
            if msg_type is not None:
                topic = attitude_topic(namespace)
                subscription = self.node.create_subscription(
                    msg_type,
                    topic,
                    lambda message, camera=camera, namespace=namespace: self._handle_message(camera, namespace, message),
                    _qos_for_px4_out(),
                )
                self.subscriptions.append(subscription)
                self.status_changed.emit(f"구독 시작: {camera_label_ko(camera)} {topic}")
            gps_position_msg_type, gps_position_is_sensor_gps = self._gps_position_subscription_type(
                gps_topic(namespace),
                gps_msg_type,
                sensor_gps_msg_type,
            )
            if gps_position_msg_type is not None:
                gps_subscription = self.node.create_subscription(
                    gps_position_msg_type,
                    gps_topic(namespace),
                    lambda message, camera=camera, namespace=namespace, emit_sensor_gps=gps_position_is_sensor_gps: self._handle_gps_position_message(
                        camera,
                        namespace,
                        message,
                        emit_sensor_gps=emit_sensor_gps,
                    ),
                    _qos_for_px4_out(),
                )
                self.subscriptions.append(gps_subscription)
                self.status_changed.emit(f"구독 시작: {camera_label_ko(camera)} {gps_topic(namespace)}")
            if global_position_msg_type is not None:
                global_position_subscription = self.node.create_subscription(
                    global_position_msg_type,
                    global_position_topic(namespace),
                    lambda message, camera=camera, namespace=namespace: self._handle_global_position_message(
                        camera,
                        namespace,
                        message,
                    ),
                    _qos_for_px4_out(),
                )
                self.subscriptions.append(global_position_subscription)
                self.status_changed.emit(
                    f"구독 시작: {camera_label_ko(camera)} {global_position_topic(namespace)}"
                )
            if sensor_mag_msg_type is not None:
                for topic in sensor_mag_topics(namespace):
                    mag_subscription = self.node.create_subscription(
                        sensor_mag_msg_type,
                        topic,
                        lambda message, camera=camera, namespace=namespace, topic=topic: self._handle_mag_message(
                            camera,
                            namespace,
                            message,
                            topic=topic,
                        ),
                        _qos_for_px4_out(),
                    )
                    self.subscriptions.append(mag_subscription)
                    self.status_changed.emit(f"구독 시작: {camera_label_ko(camera)} {topic}")
            if vehicle_mag_msg_type is not None:
                vehicle_mag_subscription = self.node.create_subscription(
                    vehicle_mag_msg_type,
                    vehicle_magnetometer_topic(namespace),
                    lambda message, camera=camera, namespace=namespace: self._handle_mag_message(
                        camera,
                        namespace,
                        message,
                        topic=vehicle_magnetometer_topic(namespace),
                    ),
                    _qos_for_px4_out(),
                )
                self.subscriptions.append(vehicle_mag_subscription)
                self.status_changed.emit(
                    f"구독 시작: {camera_label_ko(camera)} {vehicle_magnetometer_topic(namespace)}"
                )
            if sensor_gps_msg_type is not None:
                for topic in sensor_gps_topics(namespace):
                    sensor_gps_subscription = self.node.create_subscription(
                        sensor_gps_msg_type,
                        topic,
                        lambda message, camera=camera, namespace=namespace, topic=topic: self._handle_sensor_gps_message(
                            camera,
                            namespace,
                            message,
                            topic=topic,
                        ),
                        _qos_for_px4_out(),
                    )
                    self.subscriptions.append(sensor_gps_subscription)
                    self.status_changed.emit(f"구독 시작: {camera_label_ko(camera)} {topic}")
            if gnss_relative_msg_type is not None:
                gnss_relative_subscription = self.node.create_subscription(
                    gnss_relative_msg_type,
                    sensor_gnss_relative_topic(namespace),
                    lambda message, camera=camera, namespace=namespace: self._handle_gnss_relative_message(
                        camera,
                        namespace,
                        message,
                    ),
                    _qos_for_px4_out(),
                )
                self.subscriptions.append(gnss_relative_subscription)
                self.status_changed.emit(
                    f"구독 시작: {camera_label_ko(camera)} {sensor_gnss_relative_topic(namespace)}"
                )
        for spec in self.custom_watches:
            self._subscribe_custom_watch(spec)
        self.timer.start()

    def add_custom_watch(self, spec: dict[str, str]) -> None:
        if spec not in self.custom_watches:
            self.custom_watches.append(spec)
        if self.node is not None:
            self._subscribe_custom_watch(spec)

    def remove_custom_watch(self, key: str) -> None:
        self.custom_watches = [spec for spec in self.custom_watches if str(spec.get("key", "")) != key]
        subscription = self.custom_watch_subscriptions.pop(key, None)
        if subscription is None:
            return
        if self.node is not None and hasattr(self.node, "destroy_subscription"):
            try:
                self.node.destroy_subscription(subscription)
            except Exception as exc:
                self.status_changed.emit(f"Custom watch 구독 해제 실패: {key} ({exc})")
        if subscription in self.subscriptions:
            self.subscriptions.remove(subscription)

    def _subscribe_custom_watch(self, spec: dict[str, str]) -> None:
        source = str(spec.get("source", "ros2")).strip().lower()
        if source and source != "ros2":
            return
        if self.node is None or get_message is None:
            return
        topic = str(spec.get("topic", "")).strip()
        field_path = str(spec.get("field", "")).strip()
        key = str(spec.get("key") or f"{topic}.{field_path}").strip()
        if key in self.custom_watch_subscriptions:
            return
        if not topic or not field_path:
            self.status_changed.emit("Custom watch topic/field가 비어 있어 건너뜁니다.")
            return
        if not topic.startswith("/"):
            topic = f"/{topic}"
        type_names = self._topic_type_names(topic)
        if not type_names:
            self.status_changed.emit(f"Custom watch topic 타입을 찾지 못했습니다: {topic}")
            return
        try:
            msg_type = get_message(type_names[0])
        except Exception as exc:
            self.status_changed.emit(f"Custom watch 메시지 타입 로드 실패: {topic} {type_names[0]} ({exc})")
            return
        subscription = self.node.create_subscription(
            msg_type,
            topic,
            lambda message, topic=topic, field_path=field_path, key=key: self._handle_custom_watch_message(
                key, topic, field_path, message
            ),
            _qos_for_px4_out(),
        )
        self.subscriptions.append(subscription)
        self.custom_watch_subscriptions[key] = subscription
        self.status_changed.emit(f"Custom watch 구독 시작: {key} ({topic} {field_path})")

    def _handle_custom_watch_message(self, key: str, topic: str, field_path: str, message: Any) -> None:
        try:
            value = extract_field_path(message, field_path)
        except Exception as exc:
            self.status_changed.emit(f"Custom watch field 읽기 실패: {key} ({exc})")
            return
        numeric_value = _coerce_numeric(value)
        self.custom_watch_changed.emit(
            CustomTopicSample(
                key=key,
                topic=topic,
                field_path=field_path,
                value_text=_format_custom_value(value),
                numeric_value=numeric_value,
                timestamp_s=time.monotonic(),
            )
        )

    def _optional_message_type(self, name: str, *, quiet: bool = False):
        try:
            return get_message(name)
        except Exception as exc:
            if not quiet:
                self.status_changed.emit(f"메시지 타입 없음: {name} ({exc})")
            return None

    def _gps_position_subscription_type(
        self, topic: str, gps_msg_type: Any, sensor_gps_msg_type: Any
    ) -> tuple[Any, bool]:
        topic_types = self._topic_type_names(topic)
        if "px4_msgs/msg/SensorGps" in topic_types and sensor_gps_msg_type is not None:
            return sensor_gps_msg_type, True
        if "px4_msgs/msg/VehicleGpsPosition" in topic_types and gps_msg_type is not None:
            return gps_msg_type, False
        if topic_types:
            self.status_changed.emit(f"지원하지 않는 GPS topic 타입: {topic} ({', '.join(topic_types)})")
            return None, False
        if gps_msg_type is not None:
            return gps_msg_type, False
        if sensor_gps_msg_type is not None:
            return sensor_gps_msg_type, True
        return None, False

    def _topic_type_names(self, topic: str) -> tuple[str, ...]:
        if self.node is None or not hasattr(self.node, "get_topic_names_and_types"):
            return ()
        try:
            topic_names_and_types = self.node.get_topic_names_and_types()
        except TypeError:
            topic_names_and_types = self.node.get_topic_names_and_types(no_demangle=False)
        except Exception as exc:
            self.status_changed.emit(f"topic 타입 확인 실패: {topic} ({exc})")
            return ()
        for topic_name, type_names in topic_names_and_types:
            if topic_name == topic:
                return tuple(str(type_name) for type_name in type_names)
        return ()

    def _handle_gps_position_message(
        self,
        camera: str,
        namespace: str,
        message: Any,
        *,
        emit_sensor_gps: bool,
    ) -> None:
        self._handle_gps_message(
            camera,
            namespace,
            message,
            source=GPS_SOURCE_SENSOR_GPS,
            topic=gps_topic(namespace),
        )
        if emit_sensor_gps:
            self._handle_sensor_gps_message(camera, namespace, message, topic=gps_topic(namespace))

    def _handle_global_position_message(self, camera: str, namespace: str, message: Any) -> None:
        self._handle_gps_message(
            camera,
            namespace,
            message,
            source=GPS_SOURCE_GLOBAL_POSITION,
            topic=global_position_topic(namespace),
        )

    def stop(self) -> None:
        self.timer.stop()
        if self.executor is not None and self.node is not None:
            self.executor.remove_node(self.node)
        if self.node is not None:
            self.node.destroy_node()
        self.node = None
        self.executor = None
        if self.context is not None:
            try:
                self.context.shutdown()
            except Exception:
                pass
        self.context = None
        self.subscriptions = []
        self.custom_watch_subscriptions = {}
        self.status_changed.emit("구독 중지됨")

    def spin_once(self) -> None:
        if self.executor is None:
            return
        try:
            self.executor.spin_once(timeout_sec=0.0)
        except Exception as exc:
            self.status_changed.emit(f"ROS2 spin 실패: {exc}")

    def _handle_message(self, camera: str, namespace: str, message: Any) -> None:
        q = getattr(message, "q", None)
        if q is None or len(q) != 4:
            return
        px4_timestamp = getattr(message, "timestamp", None)
        px4_timestamp_s = float(px4_timestamp) * 1e-6 if px4_timestamp is not None else None
        extrinsic = camera_extrinsic_from_quaternion(camera=camera, namespace=namespace, q=q)
        extrinsic = CameraExtrinsicState(
            camera=extrinsic.camera,
            namespace=extrinsic.namespace,
            roll_deg=extrinsic.roll_deg,
            pitch_deg=extrinsic.pitch_deg,
            yaw_deg=extrinsic.yaw_deg,
            timestamp_s=extrinsic.timestamp_s,
            topic=extrinsic.topic,
            q=extrinsic.q,
            px4_timestamp_s=px4_timestamp_s,
        )
        self.latest_extrinsics[camera] = extrinsic
        self.attitude_buffers.setdefault(camera, deque(maxlen=1000)).append(extrinsic)
        self.extrinsic_changed.emit(camera, extrinsic)

    def _slot_for_device(self, slots: dict[str, dict[int, str]], camera: str, device_id: int | None, prefix: str) -> str:
        if device_id is None:
            return f"{prefix}?"
        camera_slots = slots.setdefault(camera, {})
        if device_id not in camera_slots:
            camera_slots[device_id] = f"{prefix}{len(camera_slots) + 1}"
        return camera_slots[device_id]

    def _mag_slot_for_device(self, camera: str, device_id: int | None) -> str:
        return self._slot_for_device(self.mag_device_slots, camera, device_id, "MAG")

    def _handle_mag_message(self, camera: str, namespace: str, message: Any, *, topic: str | None = None) -> None:
        now_s = time.monotonic()
        device_id = getattr(message, "device_id", None)
        device_id = int(device_id) if device_id is not None else None
        vector = getattr(message, "magnetometer_ga", None)
        if isinstance(vector, (list, tuple)) and len(vector) >= 3:
            x, y, z = (float(vector[0]), float(vector[1]), float(vector[2]))
        else:
            x = _optional_float(message, "x")
            y = _optional_float(message, "y")
            z = _optional_float(message, "z")
        magnitude = math.sqrt(x * x + y * y + z * z) if x is not None and y is not None and z is not None else None
        px4_timestamp = getattr(message, "timestamp", None)
        resolved_topic = topic or sensor_mag_topic(namespace)
        state = MagSensorState(
            camera=camera,
            namespace=namespace,
            sensor_slot=(
                "SELECTED"
                if "vehicle_magnetometer" in resolved_topic
                else self._mag_slot_for_device(camera, device_id)
            ),
            timestamp_s=now_s,
            topic=resolved_topic,
            device_id=device_id,
            x=x,
            y=y,
            z=z,
            magnitude=magnitude,
            temperature_c=_optional_float(message, "temperature"),
            error_count=_optional_int(message, "error_count"),
            px4_timestamp_s=float(px4_timestamp) * 1e-6 if px4_timestamp is not None else None,
        )
        self.mag_sensor_changed.emit(camera, state)

    def _handle_sensor_gps_message(
        self,
        camera: str,
        namespace: str,
        message: Any,
        *,
        topic: str | None = None,
    ) -> None:
        now_s = time.monotonic()
        device_id = getattr(message, "device_id", None)
        device_id = int(device_id) if device_id is not None else None
        px4_timestamp = getattr(message, "timestamp", None)
        resolved_topic = topic or sensor_gps_topic(namespace)
        state = GpsSensorState(
            camera=camera,
            namespace=namespace,
            sensor_slot=self._slot_for_device(self.gps_device_slots, camera, device_id, "GPS"),
            timestamp_s=now_s,
            topic=resolved_topic,
            device_id=device_id,
            fix_type=_optional_int(message, "fix_type"),
            satellites_used=_optional_int(message, "satellites_used", "satellites"),
            eph_m=_optional_float(message, "eph"),
            epv_m=_optional_float(message, "epv"),
            px4_timestamp_s=float(px4_timestamp) * 1e-6 if px4_timestamp is not None else None,
        )
        self.gps_sensor_changed.emit(camera, state)
        # The canonical selected sensor_gps topic also carries the LLA needed
        # for a source-separated baseline. Instance topics remain health-only;
        # GPS1/GPS2 selection is handled by their existing sensor state path.
        if resolved_topic == sensor_gps_topic(namespace):
            self._handle_gps_message(
                camera,
                namespace,
                message,
                source=GPS_SOURCE_SENSOR_GPS,
                topic=resolved_topic,
            )

    def _handle_gnss_relative_message(self, camera: str, namespace: str, message: Any) -> None:
        now_s = time.monotonic()
        device_id = getattr(message, "device_id", None)
        device_id = int(device_id) if device_id is not None else None
        px4_timestamp = getattr(message, "timestamp", None)
        state = GnssRelativeState(
            camera=camera,
            namespace=namespace,
            timestamp_s=now_s,
            topic=sensor_gnss_relative_topic(namespace),
            device_id=device_id,
            heading_rad=_optional_float(message, "heading"),
            heading_accuracy_rad=_optional_float(message, "heading_accuracy"),
            position_length_m=_optional_float(message, "position_length"),
            accuracy_length_m=_optional_float(message, "accuracy_length"),
            gnss_fix_ok=_optional_bool(message, "gnss_fix_ok"),
            differential_solution=_optional_bool(message, "differential_solution"),
            relative_position_valid=_optional_bool(message, "relative_position_valid"),
            carrier_solution_floating=_optional_bool(message, "carrier_solution_floating"),
            carrier_solution_fixed=_optional_bool(message, "carrier_solution_fixed"),
            moving_base_mode=_optional_bool(message, "moving_base_mode"),
            heading_valid=_optional_bool(message, "heading_valid"),
            px4_timestamp_s=float(px4_timestamp) * 1e-6 if px4_timestamp is not None else None,
        )
        self.gnss_relative_changed.emit(camera, state)



# Keep GPS handling near the class definition while avoiding ROS2 imports in tests.
def _dual_pixhawk_handle_gps_message(
    self,
    camera: str,
    namespace: str,
    message: Any,
    *,
    source: str = GPS_SOURCE_SENSOR_GPS,
    topic: str | None = None,
) -> None:
    gps_state = gps_state_from_message(
        camera=camera,
        namespace=namespace,
        message=message,
        timestamp_s=time.monotonic(),
        source=source,
        topic=topic,
    )
    self.latest_gps_by_source.setdefault(source, {})[camera] = gps_state
    self.gps_buffers_by_source.setdefault(source, {}).setdefault(camera, deque(maxlen=500)).append(gps_state)
    if source == GPS_SOURCE_SENSOR_GPS:
        self.latest_gps[camera] = gps_state
        self.gps_buffers.setdefault(camera, deque(maxlen=500)).append(gps_state)
    self.gps_changed.emit(camera, gps_state)


DualPixhawkAttitudeSubscriber._handle_gps_message = _dual_pixhawk_handle_gps_message


class DualPixhawkMavlinkSubscriber(QtCore.QObject):
    extrinsic_changed = QtCore.pyqtSignal(str, object)
    gps_changed = QtCore.pyqtSignal(str, object)
    mag_sensor_changed = QtCore.pyqtSignal(str, object)
    gps_sensor_changed = QtCore.pyqtSignal(str, object)
    gnss_relative_changed = QtCore.pyqtSignal(str, object)
    custom_watch_changed = QtCore.pyqtSignal(object)
    shell_output = QtCore.pyqtSignal(str, str)
    status_changed = QtCore.pyqtSignal(str)

    def __init__(self, *, left_url: str, right_url: str, left_baud: int = 57600, right_baud: int = 57600, custom_watches: tuple[dict[str, str], ...] = ()) -> None:
        super().__init__()
        self.urls = {"left": left_url.strip(), "right": right_url.strip()}
        self.baudrates = {"left": int(left_baud), "right": int(right_baud)}
        self.connections: dict[str, Any] = {}
        self.custom_watches = list(custom_watches)
        self._heartbeat_logged: set[str] = set()
        self._mavlink_stream_requests_sent: set[str] = set()
        self._mavlink_stream_config: dict[str, Any] | None = None
        self._mavlink_stream_configured: set[str] = set()
        self._ros2_client_config: dict[str, Any] | None = None
        self._ros2_client_started: set[str] = set()
        self._latest_gnss_relative: dict[str, GnssRelativeState] = {}
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.spin_once)

    def start(self, camera: str | None = None) -> None:
        try:
            from pymavlink import mavutil
        except Exception as exc:
            raise RuntimeError(f"pymavlink를 사용할 수 없습니다. MAVLink 연결에는 pymavlink 설치가 필요합니다. ({exc})") from exc
        cameras = (camera,) if camera is not None else tuple(self.urls.keys())
        started_any = False
        for side in cameras:
            url = self.urls.get(side, "").strip()
            if not url:
                continue
            if side in self.connections:
                started_any = True
                continue
            try:
                self.connections[side] = mavutil.mavlink_connection(
                    url,
                    baud=self.baudrates.get(side, 57600),
                    autoreconnect=True,
                    robust_parsing=True,
                )
            except Exception as exc:
                self.status_changed.emit(f"{camera_label_ko(side)} MAVLink 연결 실패: {exc}")
                continue
            started_any = True
            self.status_changed.emit(
                f"{camera_label_ko(side)} MAVLink 연결 시작: {url} @ {self.baudrates.get(side, 57600)}"
            )
            self._apply_pending_connection_setup(side, self.connections[side])
        if not started_any:
            label = f"{camera_label_ko(camera)} " if camera is not None else ""
            raise RuntimeError(f"{label}MAVLink 연결 URL이 없거나 연결 시작에 실패했습니다.")
        if not self.connections:
            raise RuntimeError("MAVLink 연결 URL이 없거나 모든 연결 시작에 실패했습니다.")
        self.timer.start()

    def stop(self, camera: str | None = None) -> None:
        if camera is None:
            connections = list(self.connections.items())
        else:
            connection = self.connections.get(camera)
            connections = [] if connection is None else [(camera, connection)]
        for side, connection in connections:
            try:
                connection.close()
            except Exception:
                pass
            self.connections.pop(side, None)
            self._heartbeat_logged.discard(side)
            self._mavlink_stream_requests_sent.discard(side)
            self._mavlink_stream_configured.discard(side)
            self._ros2_client_started.discard(side)
            self._latest_gnss_relative.pop(side, None)
        if not self.connections:
            self.timer.stop()
        if camera is None:
            self.status_changed.emit("MAVLink 연결 중지됨")
        else:
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink 연결 중지됨")

    def add_custom_watch(self, spec: dict[str, str]) -> None:
        if spec not in self.custom_watches:
            self.custom_watches.append(spec)

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
        sysids = sysids or {"left": 1, "right": 2}
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
            self._apply_mavlink_stream_config(camera, connection)

    def start_px4_uxrce_client_shell(
        self,
        *,
        host_ip: str,
        port: int = 8888,
        namespaces: dict[str, str] | None = None,
        cameras: set[str] | tuple[str, ...] | list[str] | None = None,
    ) -> None:
        namespaces = namespaces or {}
        self._ros2_client_config = {
            "host_ip": host_ip,
            "port": int(port),
            "namespaces": dict(namespaces),
        }
        target_cameras = set(cameras) if cameras is not None else set(self.connections.keys())
        for camera, connection in self.connections.items():
            if camera not in target_cameras:
                continue
            self._apply_ros2_client_config(camera, connection)

    def start_ros2_client(
        self,
        *,
        host_ip: str,
        port: int = 8888,
        namespaces: dict[str, str] | None = None,
        cameras: set[str] | tuple[str, ...] | list[str] | None = None,
    ) -> None:
        self.start_px4_uxrce_client_shell(
            host_ip=host_ip,
            port=port,
            namespaces=namespaces,
            cameras=cameras,
        )

    def read_param_int(self, camera: str, name: str, *, timeout_s: float = 2.0) -> int | None:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        return _mavlink_read_param_int(connection, name, timeout_s=timeout_s)

    def set_param_int(self, camera: str, name: str, value: int, *, timeout_s: float = 2.0) -> int | None:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        actual = _mavlink_set_param_int(connection, name, int(value), timeout_s=timeout_s)
        if actual == int(value):
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 param 설정 확인: {name}={actual}")
        else:
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 param 설정 불일치: {name} expected={int(value)} actual={actual}")
        return actual

    def send_rtcm_data(self, camera: str, payload: bytes) -> int:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        packet_count = _send_mavlink_rtcm_data(connection, payload)
        self.status_changed.emit(
            f"{camera_label_ko(camera)} RTCM 주입: {len(payload)} bytes / {packet_count} MAVLink packets"
        )
        return packet_count

    def _apply_pending_connection_setup(self, camera: str, connection: Any) -> None:
        if self._mavlink_stream_config is not None and camera not in self._mavlink_stream_configured:
            self._apply_mavlink_stream_config(camera, connection)
        if self._ros2_client_config is not None and camera not in self._ros2_client_started:
            self._apply_ros2_client_config(camera, connection)

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
            f"mavlink start -x -u {int(port)} -t {host_ip} -o {int(port)} -r {rate} -m normal\n"
            "mavlink status"
        )
        try:
            result = _send_mavlink_shell_command(connection, command)
        except Exception as exc:
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink stream 설정 명령 실패: {exc}")
            return
        if result.flag_label != "respond+exclusive":
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell fallback 사용: {result.flag_label}")
        self._mavlink_stream_configured.add(camera)
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink stream 설정 명령 전송: sysid={sysid}, "
            f"host={host_ip}, port={int(port)}"
        )

    def _apply_ros2_client_config(self, camera: str, connection: Any) -> None:
        config = self._ros2_client_config
        if config is None:
            return
        namespaces = config["namespaces"]
        namespace = namespaces.get(camera, "").strip().strip("/")
        namespace_arg = f" -n {namespace}" if namespace else ""
        command = (
            "uxrce_dds_client stop\n"
            f"uxrce_dds_client start -t udp -p {int(config['port'])} -h {config['host_ip']}{namespace_arg}"
        )
        try:
            result = _send_mavlink_shell_command(connection, command)
        except Exception as exc:
            self.status_changed.emit(f"{camera_label_ko(camera)} ROS2 client 실행 명령 실패: {exc}")
            return
        if result.flag_label != "respond+exclusive":
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell fallback 사용: {result.flag_label}")
        self._ros2_client_started.add(camera)
        self.status_changed.emit(f"{camera_label_ko(camera)} ROS2 client 실행 명령 전송: {command}")

    def _request_mavlink_sensor_streams(self, camera: str) -> None:
        if camera in self._mavlink_stream_requests_sent:
            return
        connection = self.connections.get(camera)
        if connection is None:
            return
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            return
        mavlink = getattr(connection, "mavlink", None)
        command_id = _mavlink_constant(mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", 511)
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        requested = []
        self._mavlink_stream_requests_sent.add(camera)
        try:
            for message_name, rate_hz in MAVLINK_RM3100_STREAM_REQUESTS:
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
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink sensor stream 요청 실패: {exc}")
            return
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink sensor stream 요청: {', '.join(requested)}"
        )

    def send_shell_command(self, camera: str, command: str) -> None:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        result = _send_mavlink_shell_command(connection, command)
        if result.flag_label != "respond+exclusive":
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell fallback 사용: {result.flag_label}")
        self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell 명령 전송: {command}")

    def send_shell_command_with_result(self, camera: str, command: str, *, timeout_s: float = 2.0) -> MavlinkShellCommandResult:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        result = _send_mavlink_shell_command(connection, command, timeout_s=timeout_s)
        if result.flag_label != "respond+exclusive":
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell fallback 사용: {result.flag_label}")
        self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell 명령 전송: {command}")
        return result

    def write_remote_file_mavftp(self, camera: str, remote_path: str, content: str | bytes) -> None:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        _write_mavlink_ftp_file(connection, remote_path, content)
        self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink FTP 파일 저장: {remote_path}")

    def reboot_autopilot(self, camera: str) -> None:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            raise RuntimeError("MAVLink COMMAND_LONG을 지원하지 않는 연결입니다.")
        mavlink = getattr(connection, "mavlink", None)
        command_id = _mavlink_constant(mavlink, "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN", 246)
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        mav.command_long_send(
            target_system,
            target_component,
            command_id,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.status_changed.emit(f"{camera_label_ko(camera)} PX4 MAVLink 재부팅 명령 전송")

    def spin_once(self) -> None:
        for camera, connection in list(self.connections.items()):
            while True:
                try:
                    message = connection.recv_match(blocking=False)
                except Exception as exc:
                    self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink 수신 실패: {exc}")
                    break
                if message is None:
                    break
                self._handle_mavlink_message(camera, message)

    def _handle_mavlink_message(self, camera: str, message: Any) -> None:
        message_type = message.get_type() if hasattr(message, "get_type") else ""
        namespace = self.urls.get(camera, "")
        now_s = time.monotonic()
        self._emit_custom_watch_samples(camera, message, message_type, now_s)
        if message_type == "ATTITUDE":
            extrinsic = camera_extrinsic_from_euler_rad(
                camera=camera,
                namespace=namespace,
                roll_rad=float(getattr(message, "roll")),
                pitch_rad=float(getattr(message, "pitch")),
                yaw_rad=float(getattr(message, "yaw")),
                timestamp_s=now_s,
            )
            self.extrinsic_changed.emit(camera, extrinsic)
            return
        if message_type in {"GPS_RAW_INT", "GPS2_RAW"}:
            topic = f"mavlink/{message_type}"
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
                eph_m=_mavlink_scaled_centimetres(getattr(message, "eph", None)),
                epv_m=_mavlink_scaled_centimetres(getattr(message, "epv", None)),
                heading_deg=_mavlink_heading_cdeg(getattr(message, "cog", None)),
                rtcm_injection_rate_hz=_optional_float(message, "rtcm_injection_rate"),
                source=GPS_SOURCE_SENSOR_GPS,
                raw=message,
            )
            if message_type == "GPS_RAW_INT":
                self.gps_changed.emit(camera, gps)
            else:
                self.gps_changed.emit(f"{camera}_GPS2", gps)
            self.gps_sensor_changed.emit(
                camera, _gps_sensor_state_from_mavlink(camera, namespace, message, topic, now_s)
            )
            gnss_relative = _gnss_relative_state_from_mavlink_gps_raw(camera, namespace, message, topic, now_s)
            if gnss_relative is not None:
                self._emit_mavlink_gnss_relative(camera, gnss_relative)
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
                self._emit_mavlink_gnss_relative(camera, gnss_relative)
            return
        if message_type == "GLOBAL_POSITION_INT":
            gps = GpsPositionState(
                camera=camera,
                namespace=namespace,
                timestamp_s=now_s,
                topic="mavlink/GLOBAL_POSITION_INT",
                latitude_deg=_mavlink_scaled_degrees(getattr(message, "lat", None)),
                longitude_deg=_mavlink_scaled_degrees(getattr(message, "lon", None)),
                altitude_m=_mavlink_scaled_millimetres(getattr(message, "alt", None)),
                heading_deg=_mavlink_heading_cdeg(getattr(message, "hdg", None)),
                source=GPS_SOURCE_GLOBAL_POSITION,
                raw=message,
            )
            self.gps_changed.emit(camera, gps)
            return
        if message_type in {"SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"}:
            mag = _mag_sensor_state_from_mavlink(camera, namespace, message, message_type, now_s)
            if mag is not None:
                self.mag_sensor_changed.emit(camera, mag)
            return
        if message_type == "HEARTBEAT":
            connection = self.connections.get(camera)
            if connection is not None:
                self._apply_pending_connection_setup(camera, connection)
            self._request_mavlink_sensor_streams(camera)
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
            if source and source not in {"mavlink", "uam_mavlink"}:
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



def mavlink_graphable_fields_for_message(message_name: str) -> list[str]:
    name = message_name.upper().strip()
    known = {
        "ATTITUDE": ["roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed", "time_boot_ms"],
        "GPS_RAW_INT": ["lat", "lon", "alt", "eph", "epv", "vel", "cog", "fix_type", "satellites_visible", "yaw", "hdg_acc", "time_usec"],
        "GPS2_RAW": ["lat", "lon", "alt", "eph", "epv", "vel", "cog", "fix_type", "satellites_visible", "yaw", "hdg_acc", "time_usec"],
        "GPS_RTK": ["baseline_a_mm", "baseline_b_mm", "baseline_c_mm", "baseline_coords_type", "accuracy", "rtk_receiver_id", "time_last_baseline_ms"],
        "GPS2_RTK": ["baseline_a_mm", "baseline_b_mm", "baseline_c_mm", "baseline_coords_type", "accuracy", "rtk_receiver_id", "time_last_baseline_ms"],
        "GLOBAL_POSITION_INT": ["lat", "lon", "alt", "relative_alt", "vx", "vy", "vz", "hdg", "time_boot_ms"],
        "SCALED_IMU": ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag", "temperature", "time_boot_ms"],
        "SCALED_IMU2": ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag", "temperature", "time_boot_ms"],
        "SCALED_IMU3": ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag", "temperature", "time_boot_ms"],
        "LOCAL_POSITION_NED": ["x", "y", "z", "vx", "vy", "vz", "time_boot_ms"],
        "VFR_HUD": ["airspeed", "groundspeed", "heading", "throttle", "alt", "climb"],
        "SYS_STATUS": ["voltage_battery", "current_battery", "battery_remaining", "drop_rate_comm", "errors_comm"],
        "BATTERY_STATUS": ["current_battery", "battery_remaining", "temperature", "voltages[0]", "voltages[1]", "voltages[2]"],
        "RAW_IMU": ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag", "time_usec"],
    }
    return known.get(name, [])


def discover_mavlink_messages() -> list[dict[str, str]]:
    names = (
        "ATTITUDE",
        "GPS_RAW_INT",
        "GPS2_RAW",
        "GPS_RTK",
        "GPS2_RTK",
        "GLOBAL_POSITION_INT",
        "SCALED_IMU",
        "SCALED_IMU2",
        "SCALED_IMU3",
        "LOCAL_POSITION_NED",
        "VFR_HUD",
        "SYS_STATUS",
        "BATTERY_STATUS",
        "RAW_IMU",
    )
    return [{"name": name, "type": "MAVLink"} for name in names]


def _merge_gnss_relative_state(
    previous: GnssRelativeState | None,
    current: GnssRelativeState,
    *,
    max_merge_age_s: float = 3.0,
) -> GnssRelativeState:
    if previous is None or current.timestamp_s - previous.timestamp_s > max_merge_age_s:
        return current
    preserved = {}
    for field_name in (
        "device_id",
        "heading_rad",
        "heading_accuracy_rad",
        "position_length_m",
        "accuracy_length_m",
        "gnss_fix_ok",
        "differential_solution",
        "relative_position_valid",
        "carrier_solution_floating",
        "carrier_solution_fixed",
        "moving_base_mode",
        "heading_valid",
        "px4_timestamp_s",
    ):
        if getattr(current, field_name) is None:
            preserved[field_name] = getattr(previous, field_name)
    return replace(current, **preserved) if preserved else current


def _gnss_relative_state_from_mavlink_gps_raw(
    camera: str,
    namespace: str,
    message: Any,
    topic: str,
    timestamp_s: float,
) -> GnssRelativeState | None:
    fix_type = _optional_int(message, "fix_type")
    heading_rad = _mavlink_yaw_cdeg_to_rad(getattr(message, "yaw", None))
    heading_valid = heading_rad is not None
    if fix_type is None and not heading_valid:
        return None
    return GnssRelativeState(
        camera=camera,
        namespace=namespace,
        timestamp_s=timestamp_s,
        topic=topic,
        heading_rad=heading_rad,
        heading_accuracy_rad=_mavlink_hdg_acc_deg_e5_to_rad(getattr(message, "hdg_acc", None)),
        gnss_fix_ok=None if fix_type is None else fix_type >= 3,
        differential_solution=None if fix_type is None else fix_type >= 4,
        relative_position_valid=heading_valid,
        carrier_solution_floating=None if fix_type is None else fix_type == 5,
        carrier_solution_fixed=None if fix_type is None else fix_type == 6,
        moving_base_mode=heading_valid,
        heading_valid=heading_valid,
    )


def _gnss_relative_state_from_mavlink_rtk(
    camera: str,
    namespace: str,
    message: Any,
    topic: str,
    timestamp_s: float,
) -> GnssRelativeState | None:
    baseline_a_mm = _optional_float(message, "baseline_a_mm")
    baseline_b_mm = _optional_float(message, "baseline_b_mm")
    baseline_c_mm = _optional_float(message, "baseline_c_mm")
    if baseline_a_mm is None or baseline_b_mm is None or baseline_c_mm is None:
        return None
    position_length_m = math.sqrt(
        baseline_a_mm * baseline_a_mm + baseline_b_mm * baseline_b_mm + baseline_c_mm * baseline_c_mm
    ) * 1e-3
    baseline_coords_type = _optional_int(message, "baseline_coords_type")
    heading_rad = None
    if baseline_coords_type == 1:
        heading_rad = math.atan2(baseline_b_mm, baseline_a_mm)
        if heading_rad < 0.0:
            heading_rad += math.tau
    accuracy_mm = _optional_float(message, "accuracy")
    relative_position_valid = position_length_m > 0.0
    return GnssRelativeState(
        camera=camera,
        namespace=namespace,
        timestamp_s=timestamp_s,
        topic=topic,
        device_id=_optional_int(message, "rtk_receiver_id"),
        heading_rad=heading_rad,
        position_length_m=position_length_m,
        accuracy_length_m=None if accuracy_mm is None else accuracy_mm * 1e-3,
        gnss_fix_ok=relative_position_valid,
        differential_solution=relative_position_valid,
        relative_position_valid=relative_position_valid,
        moving_base_mode=relative_position_valid,
        heading_valid=heading_rad is not None,
    )


def _gps_sensor_state_from_mavlink(
    camera: str, namespace: str, message: Any, topic: str, timestamp_s: float
) -> GpsSensorState:
    slot = "GPS2" if topic.endswith("GPS2_RAW") else "GPS1"
    return GpsSensorState(
        camera=camera,
        namespace=namespace,
        sensor_slot=slot,
        timestamp_s=timestamp_s,
        topic=topic,
        device_id=None,
        fix_type=_optional_int(message, "fix_type"),
        satellites_used=_optional_int(message, "satellites_visible"),
        eph_m=_mavlink_scaled_centimetres(getattr(message, "eph", None)),
        epv_m=_mavlink_scaled_centimetres(getattr(message, "epv", None)),
        px4_timestamp_s=None,
    )


def _mag_sensor_state_from_mavlink(
    camera: str, namespace: str, message: Any, message_type: str, timestamp_s: float
) -> MagSensorState | None:
    x = _optional_float(message, "xmag")
    y = _optional_float(message, "ymag")
    z = _optional_float(message, "zmag")
    if x is None or y is None or z is None:
        return None
    slot = message_type if message_type in {"SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"} else "MAVLINK_MAG"
    return MagSensorState(
        camera=camera,
        namespace=namespace,
        sensor_slot=slot,
        timestamp_s=timestamp_s,
        topic=f"mavlink/{message_type}",
        device_id=None,
        x=x,
        y=y,
        z=z,
        magnitude=math.sqrt(x * x + y * y + z * z),
        temperature_c=_optional_float(message, "temperature"),
        px4_timestamp_s=None,
    )


def _mavlink_scaled_degrees(value: Any) -> float | None:
    return None if value is None else float(value) * 1e-7


def _mavlink_scaled_millimetres(value: Any) -> float | None:
    return None if value is None else float(value) * 1e-3


def _mavlink_scaled_centimetres(value: Any) -> float | None:
    if value is None or int(value) == 65535:
        return None
    return float(value) * 1e-2


def _mavlink_heading_cdeg(value: Any) -> float | None:
    if value is None or int(value) == 65535:
        return None
    return float(value) * 1e-2


def _mavlink_yaw_cdeg_to_rad(value: Any) -> float | None:
    if value is None:
        return None
    yaw_cdeg = int(value)
    if yaw_cdeg == 0 or yaw_cdeg == 65535:
        return None
    if yaw_cdeg == 36000:
        yaw_cdeg = 0
    return math.radians((yaw_cdeg % 36000) * 1e-2)


def _mavlink_hdg_acc_deg_e5_to_rad(value: Any) -> float | None:
    if value is None or int(value) == 4294967295:
        return None
    return math.radians(float(value) * 1e-5)


def _serial_control_output_from_message(message: Any) -> str:
    try:
        count = int(getattr(message, "count", 0) or 0)
        data = bytes(getattr(message, "data", [])[:count])
    except Exception:
        return ""
    return data.decode("utf-8", errors="replace")


def _read_mavlink_shell_output(connection: Any, *, timeout_s: float) -> str:
    recv_match = getattr(connection, "recv_match", None)
    if recv_match is None:
        return ""
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    chunks: list[str] = []
    while time.monotonic() < deadline:
        message = recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.05)
        if message is None:
            continue
        text = _serial_control_output_from_message(message)
        if text:
            chunks.append(text)
    return "".join(chunks)


def _send_mavlink_shell_payload(mav: Any, *, device: int, flags: int, payload: bytes) -> None:
    for offset in range(0, len(payload), 70):
        chunk = payload[offset : offset + 70]
        data = list(chunk) + [0] * (70 - len(chunk))
        mav.serial_control_send(device, flags, 0, 0, len(chunk), data)


def _send_mavlink_shell_command(
    connection: Any,
    command: str,
    *,
    timeout_s: float = 0.8,
) -> MavlinkShellCommandResult:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "serial_control_send"):
        raise RuntimeError("MAVLink serial control을 지원하지 않는 연결입니다.")
    mavlink = getattr(connection, "mavlink", None)
    device = _mavlink_constant(mavlink, "SERIAL_CONTROL_DEV_SHELL", 10)
    respond_flag = _mavlink_constant(mavlink, "SERIAL_CONTROL_FLAG_RESPOND", 2)
    exclusive_flag = _mavlink_constant(mavlink, "SERIAL_CONTROL_FLAG_EXCLUSIVE", 4)
    variants = (
        ("respond+exclusive", respond_flag | exclusive_flag),
        ("respond", respond_flag),
    )
    payload = (command.rstrip() + "\n").encode("utf-8")
    selected_label, selected_flags = variants[-1]
    probe_output = ""
    for label, flags in variants:
        # Opening the shell first matters on PX4 NSH links that only respond after a read request.
        mav.serial_control_send(device, flags, 0, 0, 0, [0] * 70)
        output = _read_mavlink_shell_output(connection, timeout_s=min(timeout_s, 0.25))
        if output or label == variants[-1][0]:
            selected_label = label
            selected_flags = flags
            probe_output = output
            break
    _send_mavlink_shell_payload(mav, device=device, flags=selected_flags, payload=payload)
    command_output = _read_mavlink_shell_output(connection, timeout_s=timeout_s)
    return MavlinkShellCommandResult(
        command=command,
        output=probe_output + command_output,
        flag_label=selected_label,
    )


def _send_mavlink_rtcm_data(connection: Any, payload: bytes) -> int:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "gps_rtcm_data_send"):
        raise RuntimeError("MAVLink GPS_RTCM_DATA를 지원하지 않는 연결입니다.")
    data = bytes(payload)
    if not data:
        return 0
    max_chunk = 180
    max_fragmented_payload = max_chunk * 4
    if len(data) > max_fragmented_payload:
        raise ValueError(f"RTCM frame too large for MAVLink GPS_RTCM_DATA: {len(data)} > {max_fragmented_payload} bytes")
    chunks = [data[offset : offset + max_chunk] for offset in range(0, len(data), max_chunk)]
    sequence_id = int(time.monotonic() * 1000) & 0x1F
    fragmented = len(chunks) > 1
    for fragment_id, chunk in enumerate(chunks):
        flags = 0
        if fragmented:
            flags = 1 | ((fragment_id & 0x03) << 1) | ((sequence_id & 0x1F) << 3)
        mav.gps_rtcm_data_send(flags, len(chunk), list(chunk) + [0] * (max_chunk - len(chunk)))
    return len(chunks)


def _write_mavlink_ftp_file(connection: Any, remote_path: str, content: str | bytes) -> None:
    try:
        from pymavlink import mavftp
        from pymavlink.mavftp import FtpError
    except Exception as exc:
        raise RuntimeError(f"pymavlink MAVFTP를 사용할 수 없습니다. ({exc})") from exc
    if not remote_path.startswith("/"):
        raise ValueError(f"MAVLink FTP remote path must be absolute: {remote_path}")
    payload = content.encode("utf-8") if isinstance(content, str) else bytes(content)
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    ftp = mavftp.MAVFTP(connection, target_system, target_component)
    directory = str(Path(remote_path).parent)
    if directory and directory not in {"/", "/fs/microsd"}:
        mkdir_result = ftp.cmd_mkdir([directory])
        if mkdir_result.return_code not in (FtpError.Success, FtpError.FileExists):
            raise RuntimeError(f"MAVLink FTP mkdir 실패: {directory} ({mkdir_result.return_code})")
    rm_result = ftp.cmd_rm([remote_path])
    if rm_result.return_code not in (FtpError.Success, FtpError.FileNotFound):
        raise RuntimeError(f"MAVLink FTP 기존 파일 삭제 실패: {remote_path} ({rm_result.return_code})")
    handle = io.BytesIO(payload)
    create_result = ftp.cmd_put(["extras.txt", remote_path], fh=handle)
    if create_result.return_code != FtpError.Success:
        raise RuntimeError(f"MAVLink FTP 파일 생성 실패: {remote_path} ({create_result.return_code})")
    write_result = ftp.process_ftp_reply("WriteFile", timeout=8)
    if write_result.return_code != FtpError.Success:
        raise RuntimeError(f"MAVLink FTP 파일 쓰기 실패: {remote_path} ({write_result.return_code})")


def _mavlink_param_id(name: str) -> bytes:
    encoded = name.encode("ascii", errors="strict")
    if len(encoded) > 16:
        raise ValueError(f"MAVLink PARAM id is longer than 16 bytes: {name}")
    return encoded


def _mavlink_param_id_text(message: Any) -> str:
    raw = getattr(message, "param_id", "")
    if isinstance(raw, bytes):
        return raw.rstrip(b"\x00").decode("ascii", errors="replace")
    return str(raw).rstrip("\x00")


def _mavlink_int_to_param_float(value: int) -> float:
    return struct.unpack("<f", struct.pack("<I", int(value) & 0xFFFFFFFF))[0]


def _mavlink_param_float_to_int(value: Any) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def _mavlink_recv_param_value(connection: Any, name: str, *, timeout_s: float) -> int | None:
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=min(0.2, remaining))
        if message is None:
            continue
        if _mavlink_param_id_text(message) != name:
            continue
        return _mavlink_param_float_to_int(getattr(message, "param_value"))
    return None


def _mavlink_read_param_int(connection: Any, name: str, *, timeout_s: float = 2.0) -> int | None:
    if hasattr(connection, "param_fetch_one"):
        try:
            value = connection.param_fetch_one(name, timeout=timeout_s)
        except TypeError:
            value = connection.param_fetch_one(name)
        except Exception:
            value = None
        if value is not None:
            try:
                return _mavlink_param_float_to_int(value)
            except Exception:
                try:
                    return int(value)
                except Exception:
                    pass
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "param_request_read_send"):
        return None
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    mav.param_request_read_send(target_system, target_component, _mavlink_param_id(name), -1)
    return _mavlink_recv_param_value(connection, name, timeout_s=timeout_s)


def _mavlink_set_param_int(connection: Any, name: str, value: int, *, timeout_s: float = 2.0) -> int | None:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "param_set_send"):
        raise RuntimeError("MAVLink PARAM_SET을 지원하지 않는 연결입니다.")
    mavlink = getattr(connection, "mavlink", None)
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    param_type = _mavlink_constant(mavlink, "MAV_PARAM_TYPE_INT32", 6)
    mav.param_set_send(
        target_system,
        target_component,
        _mavlink_param_id(name),
        _mavlink_int_to_param_float(int(value)),
        param_type,
    )
    return _mavlink_recv_param_value(connection, name, timeout_s=timeout_s)


def _decode_mavlink_serial_control(message: Any) -> str:
    count = int(getattr(message, "count", 0) or 0)
    data = getattr(message, "data", b"")
    if isinstance(data, bytes):
        payload = data[:count]
    else:
        payload = bytes(int(value) & 0xFF for value in list(data)[:count])
    return payload.replace(b"\x00", b"").decode("utf-8", errors="replace")


def _mavlink_constant(mavlink: Any, name: str, fallback: int) -> int:
    if mavlink is not None and hasattr(mavlink, name):
        return int(getattr(mavlink, name))
    return fallback
