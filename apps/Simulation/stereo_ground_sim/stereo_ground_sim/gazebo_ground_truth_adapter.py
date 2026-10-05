"""Authoritative Gazebo Pose_V to vehicle-reference PoseStamped adapter.

The adapter deliberately consumes the raw Gazebo ``Pose_V`` stream instead
of relying on a bridge conversion that may discard entity identity or stamps.
The source stamp is copied verbatim; subprocess receive time is never used as
the measurement timestamp.
"""

from __future__ import annotations

import json
import math
import subprocess
import threading
from dataclasses import dataclass
from typing import Any, Iterable

from .coordinate_frames import (
    REFERENCE_OFFSET_PARENT_M,
    REFERENCE_OFFSET_PARENT_RPY_RAD,
    SOURCE_GAZEBO_ENTITY,
    VEHICLE_REFERENCE_DEFINITION,
    VEHICLE_REFERENCE_LINK,
    WORLD_FRAME,
)


# The target model owns a PosePublisher, so its scoped model/link identities
# never collide with the camera carriers' own ``base_link`` names.
POSE_TOPIC = "/model/x500_15m/pose"
OUTPUT_TOPIC = "/simulation/ground_truth/vehicle_reference_link"
RVIZ_MARKER_TOPIC = "/stereo/ground_truth_markers"
GAZEBO_MESSAGE_TYPE = "gz.msgs.Pose_V"
ROS_MESSAGE_TYPE = "geometry_msgs/msg/PoseStamped"
DRONE_MESH_RESOURCE = "package://stereo_ground_sim/models/x500_15m/meshes/NXP-HGD-CF.dae"
# Must match this model's base_link_visual mesh <scale> in its model.sdf, or
# the RViz marker will render at the wrong size relative to the real Gazebo
# geometry.
DRONE_MESH_VISUAL_SCALE = 24.5245
_ADDITIONAL_TARGET_MODEL_VISUALS: dict[str, tuple[str, float]] = {
    "x500_2m": (
        "package://stereo_ground_sim/models/x500_2m/meshes/NXP-HGD-CF.dae",
        4.0638,
    ),
}


def _resolve_target_model_visual(model_name: str) -> tuple[str, float]:
    """Return (mesh_resource, visual_scale) for the given target model name."""
    if model_name == "x500_15m":
        return DRONE_MESH_RESOURCE, DRONE_MESH_VISUAL_SCALE
    try:
        return _ADDITIONAL_TARGET_MODEL_VISUALS[model_name]
    except KeyError:
        known = ", ".join(sorted({"x500_15m", *_ADDITIONAL_TARGET_MODEL_VISUALS}))
        raise ValueError(
            f"unknown target model {model_name!r}; register its mesh resource/visual "
            f"scale in _ADDITIONAL_TARGET_MODEL_VISUALS (known: {known})"
        ) from None


class PosePayloadError(ValueError):
    """Raised when a raw Pose_V JSON payload cannot provide authoritative GT."""


@dataclass(frozen=True)
class PoseRecord:
    stamp_ns: int
    entity_name: str
    entity_id: int
    position_world_enu_m: tuple[float, float, float]
    orientation_world_from_entity_xyzw: tuple[float, float, float, float]

    @property
    def stamp_sec(self) -> int:
        return self.stamp_ns // 1_000_000_000

    @property
    def stamp_nsec(self) -> int:
        return self.stamp_ns % 1_000_000_000


def stamp_ns_from_header(header: Any) -> int:
    """Convert a Pose_V header stamp to signed integer nanoseconds."""

    if not isinstance(header, dict):
        raise PosePayloadError("Pose_V top-level header is missing")
    stamp = header.get("stamp", header)
    if not isinstance(stamp, dict):
        raise PosePayloadError("Pose_V header.stamp is missing")
    try:
        sec = int(stamp.get("sec", stamp.get("seconds", 0)))
        nsec = int(stamp.get("nsec", stamp.get("nanosec", stamp.get("nanoseconds", 0))))
    except (TypeError, ValueError):
        raise PosePayloadError("Pose_V header.stamp sec/nsec is invalid") from None
    if nsec < 0 or nsec >= 1_000_000_000:
        raise PosePayloadError("Pose_V header.stamp.nsec is out of range")
    return sec * 1_000_000_000 + nsec


def _iter_dicts(value: Any) -> Iterable[dict[str, Any]]:
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from _iter_dicts(child)
    elif isinstance(value, list):
        for child in value:
            yield from _iter_dicts(child)


def _number(mapping: dict[str, Any], key: str) -> float:
    try:
        # Gazebo protobuf JSON omits scalar fields whose value is zero.
        value = float(mapping.get(key, 0.0))
    except (KeyError, TypeError, ValueError):
        raise PosePayloadError(f"pose field {key} is invalid") from None
    if not math.isfinite(value):
        raise PosePayloadError(f"pose field {key} is non-finite")
    return value


def _entity_id(value: Any) -> int:
    # Model-scoped PosePublisher messages omit numeric ECM ids.  The scoped
    # topic and fully-qualified entity name already provide run identity.
    if value is None:
        return -1
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise PosePayloadError("pose entity id is missing or invalid") from None
    return parsed


def _pose_record_from_mapping(mapping: dict[str, Any], stamp_ns: int) -> PoseRecord:
    position = mapping.get("position", mapping.get("pos"))
    orientation = mapping.get("orientation", mapping.get("rot", {}))
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise PosePayloadError("pose position/orientation is missing")
    name = mapping.get("name")
    if not isinstance(name, str) or not name:
        raise PosePayloadError("pose entity name is missing")
    return PoseRecord(
        stamp_ns=int(stamp_ns),
        entity_name=name,
        entity_id=_entity_id(mapping.get("id")),
        position_world_enu_m=(_number(position, "x"), _number(position, "y"), _number(position, "z")),
        orientation_world_from_entity_xyzw=(
            _number(orientation, "x"),
            _number(orientation, "y"),
            _number(orientation, "z"),
            _number(orientation, "w"),
        ),
    )


def _named_pose_mappings(payload: dict[str, Any], entity_name: str) -> list[dict[str, Any]]:
    mappings = []
    for mapping in _iter_dicts(payload):
        if mapping.get("name") != entity_name:
            continue
        if "position" not in mapping and "pos" not in mapping:
            continue
        mappings.append(mapping)
    return mappings


def _pose_v_stamp_ns(payload: dict[str, Any], mappings: list[dict[str, Any]]) -> int:
    if isinstance(payload.get("header"), dict):
        return stamp_ns_from_header(payload["header"])
    stamps = {stamp_ns_from_header(mapping.get("header")) for mapping in mappings}
    if len(stamps) != 1:
        raise PosePayloadError("Pose_V entity stamps are missing or inconsistent")
    return stamps.pop()


def _quaternion_normalize(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(float(value) * float(value) for value in quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        raise PosePayloadError("pose orientation quaternion is invalid")
    return tuple(float(value) / norm for value in quaternion)  # type: ignore[return-value]


def _quaternion_multiply(
    parent_xyzw: tuple[float, float, float, float],
    child_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    px, py, pz, pw = parent_xyzw
    cx, cy, cz, cw = child_xyzw
    return (
        pw * cx + px * cw + py * cz - pz * cy,
        pw * cy - px * cz + py * cw + pz * cx,
        pw * cz + px * cy - py * cx + pz * cw,
        pw * cw - px * cx - py * cy - pz * cz,
    )


def _rotate_vector_by_quaternion(
    quaternion_xyzw: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion_xyzw
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def pose_record_world_from_payload(
    payload: dict[str, Any],
    record: PoseRecord,
    *,
    parent_model_name: str = "x500_15m",
) -> PoseRecord:
    """Resolve a base-link record against its matching model world pose.

    Gazebo's current SceneBroadcaster emits the model pose in world ENU while
    the root ``base_link`` pose is relative to that model.  The matching
    model record is therefore required to publish an absolute world pose.
    """

    model_mappings = _named_pose_mappings(payload, parent_model_name)
    if not model_mappings:
        raise PosePayloadError(
            f"parent model {parent_model_name!r} is unavailable for world pose resolution"
        )
    if len(model_mappings) != 1:
        raise PosePayloadError(
            f"parent model {parent_model_name!r} has duplicate name/id entries"
        )
    model_record = _pose_record_from_mapping(model_mappings[0], record.stamp_ns)
    model_quaternion = _quaternion_normalize(model_record.orientation_world_from_entity_xyzw)
    link_quaternion = _quaternion_normalize(record.orientation_world_from_entity_xyzw)
    rotated_link_position = _rotate_vector_by_quaternion(
        model_quaternion,
        record.position_world_enu_m,
    )
    world_orientation = _quaternion_normalize(_quaternion_multiply(model_quaternion, link_quaternion))
    world_position = tuple(
        float(model_record.position_world_enu_m[index]) + float(rotated_link_position[index])
        for index in range(3)
    )
    return PoseRecord(
        stamp_ns=record.stamp_ns,
        entity_name=record.entity_name,
        entity_id=record.entity_id,
        position_world_enu_m=world_position,  # type: ignore[arg-type]
        orientation_world_from_entity_xyzw=world_orientation,
    )


def parse_pose_v_payload(
    payload: dict[str, Any],
    *,
    entity_name: str = SOURCE_GAZEBO_ENTITY,
    expected_entity_id: int | None = None,
) -> PoseRecord:
    """Parse one raw Pose_V JSON payload and retain name, id and stamp."""

    if not isinstance(payload, dict):
        raise PosePayloadError("Pose_V payload is not an object")
    mappings = _named_pose_mappings(payload, entity_name)
    if not mappings:
        raise PosePayloadError(f"authoritative entity {entity_name!r} is unavailable")
    if len(mappings) != 1:
        raise PosePayloadError(f"authoritative entity {entity_name!r} has duplicate name/id entries")
    stamp_ns = _pose_v_stamp_ns(payload, mappings)
    record = _pose_record_from_mapping(mappings[0], stamp_ns)
    if expected_entity_id is not None and record.entity_id != int(expected_entity_id):
        raise PosePayloadError(
            f"authoritative entity id changed from {expected_entity_id} to {record.entity_id}"
        )
    return record


class PoseVState:
    """Per-run identity/stamp gate shared by runtime and unit tests."""

    def __init__(self, entity_name: str = SOURCE_GAZEBO_ENTITY) -> None:
        self.entity_name = str(entity_name)
        self.entity_id: int | None = None
        self.last_stamp_ns: int | None = None
        self.reset_count = 0
        self.unavailable_reason = ""

    def consume(self, payload: dict[str, Any]) -> PoseRecord | None:
        try:
            record = parse_pose_v_payload(
                payload,
                entity_name=self.entity_name,
            )
        except PosePayloadError as exc:
            self.unavailable_reason = str(exc)
            return None
        is_reset = self.last_stamp_ns is not None and record.stamp_ns < self.last_stamp_ns
        if is_reset:
            self.reset_count += 1
            self.entity_id = record.entity_id
        elif self.entity_id is None:
            self.entity_id = record.entity_id
        elif record.entity_id != self.entity_id:
            self.unavailable_reason = (
                f"authoritative entity id changed from {self.entity_id} to {record.entity_id}"
            )
            return None
        self.last_stamp_ns = record.stamp_ns
        self.unavailable_reason = ""
        return record


def pose_record_to_ros_message(record: PoseRecord) -> Any:
    """Build a PoseStamped without importing ROS during pure parser tests."""

    try:
        from geometry_msgs.msg import PoseStamped
    except ImportError as exc:  # pragma: no cover - depends on ROS installation
        raise RuntimeError("geometry_msgs is required to publish Ground Truth") from exc
    message = PoseStamped()
    message.header.stamp.sec = int(record.stamp_sec)
    message.header.stamp.nanosec = int(record.stamp_nsec)
    message.header.frame_id = WORLD_FRAME
    message.pose.position.x = float(record.position_world_enu_m[0])
    message.pose.position.y = float(record.position_world_enu_m[1])
    message.pose.position.z = float(record.position_world_enu_m[2])
    (
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ) = record.orientation_world_from_entity_xyzw
    return message


class GazeboGroundTruthAdapter:
    """Small ROS adapter; construction requires an installed ROS 2 runtime."""

    def __init__(
        self,
        *,
        topic: str | None = None,
        output_topic: str = OUTPUT_TOPIC,
        command: tuple[str, ...] | None = None,
        parent_model_name: str = "x500_15m",
        source_link_name: str = "",
    ) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from rcl_interfaces.msg import SetParametersResult
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from visualization_msgs.msg import Marker, MarkerArray
        except ImportError as exc:  # pragma: no cover - depends on ROS installation
            raise RuntimeError("rclpy and geometry_msgs are required for the adapter") from exc

        class _Node(Node):
            pass

        self._rclpy = rclpy
        self._node = _Node("gazebo_ground_truth_adapter")
        self._node.declare_parameter("parent_model_name", str(parent_model_name))
        self._node.declare_parameter("source_link_name", str(source_link_name))
        self._publisher = self._node.create_publisher(PoseStamped, output_topic, 10)
        marker_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_publisher = self._node.create_publisher(
            MarkerArray, RVIZ_MARKER_TOPIC, marker_qos
        )
        self._marker_type = Marker
        self._marker_array_type = MarkerArray
        self._parent_model_name = str(self._node.get_parameter("parent_model_name").value)
        self._mesh_resource, self._mesh_visual_scale = _resolve_target_model_visual(
            self._parent_model_name
        )
        configured_source_link = str(self._node.get_parameter("source_link_name").value)
        self._source_link_name = configured_source_link or f"{self._parent_model_name}::{SOURCE_GAZEBO_ENTITY}"
        self._topic = str(topic or f"/model/{self._parent_model_name}/pose")

        self._set_parameters_result_type = SetParametersResult
        self._node.add_on_set_parameters_callback(self._reject_identity_parameter_update)
        self._command = (
            tuple(command)
            if command is not None
            else ("gz", "topic", "-e", "-t", self._topic, "--json-output")
        )
        self._state = PoseVState(self._source_link_name)
        self._shutdown = threading.Event()
        self._process: subprocess.Popen[str] | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._node.get_logger().info(
            "Gazebo Ground Truth adapter: "
            f"source_gazebo_entity={self._source_link_name}, "
            f"parent_model_name={self._parent_model_name}, "
            f"published_reference_name={VEHICLE_REFERENCE_LINK}, "
            f"reference_definition={VEHICLE_REFERENCE_DEFINITION}, "
            f"reference_offset_parent_m={REFERENCE_OFFSET_PARENT_M}, "
            f"reference_offset_parent_rpy_rad={REFERENCE_OFFSET_PARENT_RPY_RAD}"
        )

    @property
    def node(self) -> Any:
        return self._node

    def _reject_identity_parameter_update(self, parameters: Iterable[Any]) -> Any:
        immutable = {"parent_model_name", "source_link_name"}
        if any(parameter.name in immutable for parameter in parameters):
            return self._set_parameters_result_type(
                successful=False,
                reason="Ground Truth entity identity parameters are immutable for one run",
            )
        return self._set_parameters_result_type(successful=True)

    def _publish_rviz_marker(self, record: PoseRecord) -> None:
        """Publish the authoritative vehicle pose directly for RViz.

        Contract: pose and marker frame are ``gazebo_world_enu`` in metres.
        This path is intentionally independent of stereo inference and ray
        generation, so visualization follows Gazebo even without detections.
        """

        marker = self._marker_type()
        marker.header.frame_id = WORLD_FRAME
        marker.header.stamp.sec = record.stamp_sec
        marker.header.stamp.nanosec = record.stamp_nsec
        marker.ns = "stereo_actual_drone"
        marker.id = 40
        marker.type = self._marker_type.MESH_RESOURCE
        marker.action = self._marker_type.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            record.position_world_enu_m
        )
        (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = record.orientation_world_from_entity_xyzw
        marker.scale.x = marker.scale.y = marker.scale.z = self._mesh_visual_scale
        marker.color.r = marker.color.g = marker.color.b = 0.72
        marker.color.a = 1.0
        marker.mesh_resource = self._mesh_resource
        marker.mesh_use_embedded_materials = False
        self._marker_publisher.publish(self._marker_array_type(markers=[marker]))

    def _run(self) -> None:
        while not self._shutdown.is_set():
            try:
                process = subprocess.Popen(
                    self._command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1,
                )
            except OSError as exc:
                self._node.get_logger().error(f"failed to start Gazebo pose stream: {exc}")
                self._shutdown.wait(0.5)
                continue
            self._process = process
            stderr_thread = threading.Thread(target=self._log_stderr, args=(process,), daemon=True)
            stderr_thread.start()
            try:
                if process.stdout is not None:
                    for line in process.stdout:
                        if self._shutdown.is_set():
                            break
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            payload = json.loads(line)
                        except json.JSONDecodeError as exc:
                            self._node.get_logger().warning(f"Gazebo Pose_V JSON parse failure: {exc}")
                            continue
                        reset_count = self._state.reset_count
                        record = self._state.consume(payload)
                        if record is None:
                            self._node.get_logger().warning(
                                f"Ground Truth unavailable: {self._state.unavailable_reason}"
                            )
                            continue
                        try:
                            record = pose_record_world_from_payload(
                                payload,
                                record,
                                parent_model_name=self._parent_model_name,
                            )
                        except PosePayloadError as exc:
                            self._node.get_logger().warning(
                                f"Ground Truth unavailable: {exc}"
                            )
                            continue
                        if self._state.reset_count > reset_count:
                            self._node.get_logger().warning(
                                "Gazebo simulation stamp decreased; treating the sample as a reset/restart"
                            )
                        try:
                            self._publisher.publish(pose_record_to_ros_message(record))
                            self._publish_rviz_marker(record)
                        except Exception as exc:
                            if self._shutdown.is_set() or not self._rclpy.ok():
                                break
                            self._node.get_logger().error(
                                f"Ground Truth publish failure: {exc}"
                            )
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1.0)
                stderr_thread.join(timeout=0.5)
                self._process = None
            if not self._shutdown.is_set():
                self._shutdown.wait(0.5)

    def _log_stderr(self, process: subprocess.Popen[str]) -> None:
        if process.stderr is None:
            return
        for line in process.stderr:
            line = line.strip()
            if line:
                self._node.get_logger().warning(f"Gazebo pose stream stderr: {line}")

    def shutdown(self) -> None:
        self._shutdown.set()
        process = self._process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self._node.destroy_node()
        except Exception:
            pass


def main() -> None:  # pragma: no cover - requires a live ROS/Gazebo runtime
    import rclpy

    rclpy.init()
    adapter = GazeboGroundTruthAdapter()
    try:
        rclpy.spin(adapter.node)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = [
    "GAZEBO_MESSAGE_TYPE",
    "OUTPUT_TOPIC",
    "RVIZ_MARKER_TOPIC",
    "POSE_TOPIC",
    "PosePayloadError",
    "PoseRecord",
    "PoseVState",
    "ROS_MESSAGE_TYPE",
    "GazeboGroundTruthAdapter",
    "main",
    "parse_pose_v_payload",
    "pose_record_world_from_payload",
    "pose_record_to_ros_message",
    "stamp_ns_from_header",
]
