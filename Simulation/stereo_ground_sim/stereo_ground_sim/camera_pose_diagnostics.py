"""Expose desired-camera versus Gazebo-observed optical-pose diagnostics."""

from __future__ import annotations

import json
import math
import subprocess
import threading
from dataclasses import asdict, dataclass
from typing import Any

from .gazebo_ground_truth_adapter import (
    PosePayloadError,
    PoseRecord,
    PoseVState,
    parse_pose_v_payload,
    pose_record_world_from_payload,
)


def _quaternion_from_rpy(rpy_rad: tuple[float, float, float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = (float(value) for value in rpy_rad)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _subtract(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(float(left[index]) - float(right[index]) for index in range(3))  # type: ignore[return-value]


def _norm(values: tuple[float, float, float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def _orientation_delta_deg(
    left_xyzw: tuple[float, float, float, float],
    right_xyzw: tuple[float, float, float, float],
) -> float:
    dot = abs(sum(float(left_xyzw[index]) * float(right_xyzw[index]) for index in range(4)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


@dataclass(frozen=True)
class CameraPoseComparison:
    schema: str
    camera_label: str
    gazebo_stamp_ns: int
    desired_optical_position_world_enu_m: tuple[float, float, float]
    observed_optical_position_world_enu_m: tuple[float, float, float]
    desired_to_observed_optical_delta_world_enu_m: tuple[float, float, float]
    desired_to_observed_optical_delta_norm_m: float
    desired_to_observed_optical_orientation_delta_deg: float
    observed_base_link_position_world_enu_m: tuple[float, float, float]
    observed_base_link_orientation_world_from_entity_xyzw: tuple[float, float, float, float]
    observed_optical_minus_base_link_world_enu_m: tuple[float, float, float]
    observed_optical_minus_base_link_norm_m: float
    observed_optical_minus_base_link_orientation_delta_deg: float
    observed_base_link_entity_name: str
    observed_base_link_entity_id: int
    observed_optical_entity_name: str
    observed_optical_entity_id: int
    observed_optical_orientation_world_from_entity_xyzw: tuple[float, float, float, float]


def compare_camera_pose_payload(
    payload: dict[str, Any],
    *,
    camera_label: str,
    parent_model_name: str,
    desired_optical_position_world_enu_m: tuple[float, float, float],
    desired_optical_rpy_world_rad: tuple[float, float, float],
    expected_base_entity_id: int | None = None,
    expected_optical_entity_id: int | None = None,
) -> CameraPoseComparison:
    """Compare one camera's configured optical pose to one Gazebo ``Pose_V`` sample."""

    base = parse_pose_v_payload(
        payload,
        entity_name=f"{parent_model_name}::base_link",
        expected_entity_id=expected_base_entity_id,
    )
    optical = parse_pose_v_payload(
        payload,
        entity_name=f"{parent_model_name}::optical_link",
        expected_entity_id=expected_optical_entity_id,
    )
    if base.stamp_ns != optical.stamp_ns:
        raise PosePayloadError("camera base_link and optical_link stamps differ")
    observed_base = pose_record_world_from_payload(payload, base, parent_model_name=parent_model_name)
    observed_optical = pose_record_world_from_payload(payload, optical, parent_model_name=parent_model_name)
    desired_orientation = _quaternion_from_rpy(desired_optical_rpy_world_rad)
    desired_delta = _subtract(
        observed_optical.position_world_enu_m,
        desired_optical_position_world_enu_m,
    )
    optical_minus_base = _subtract(
        observed_optical.position_world_enu_m,
        observed_base.position_world_enu_m,
    )
    return CameraPoseComparison(
        schema="camera_pose_comparison/v1",
        camera_label=str(camera_label),
        gazebo_stamp_ns=observed_optical.stamp_ns,
        desired_optical_position_world_enu_m=desired_optical_position_world_enu_m,
        observed_optical_position_world_enu_m=observed_optical.position_world_enu_m,
        desired_to_observed_optical_delta_world_enu_m=desired_delta,
        desired_to_observed_optical_delta_norm_m=_norm(desired_delta),
        desired_to_observed_optical_orientation_delta_deg=_orientation_delta_deg(
            observed_optical.orientation_world_from_entity_xyzw,
            desired_orientation,
        ),
        observed_base_link_position_world_enu_m=observed_base.position_world_enu_m,
        observed_base_link_orientation_world_from_entity_xyzw=(
            observed_base.orientation_world_from_entity_xyzw
        ),
        observed_optical_minus_base_link_world_enu_m=optical_minus_base,
        observed_optical_minus_base_link_norm_m=_norm(optical_minus_base),
        observed_optical_minus_base_link_orientation_delta_deg=_orientation_delta_deg(
            observed_optical.orientation_world_from_entity_xyzw,
            observed_base.orientation_world_from_entity_xyzw,
        ),
        observed_base_link_entity_name=observed_base.entity_name,
        observed_base_link_entity_id=observed_base.entity_id,
        observed_optical_entity_name=observed_optical.entity_name,
        observed_optical_entity_id=observed_optical.entity_id,
        observed_optical_orientation_world_from_entity_xyzw=(
            observed_optical.orientation_world_from_entity_xyzw
        ),
    )


class GazeboCameraPoseDiagnostics:
    """Publish one self-describing comparison stream for a camera model."""

    def __init__(self) -> None:  # pragma: no cover - requires live ROS/Gazebo
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
        except ImportError as exc:
            raise RuntimeError("rclpy and std_msgs are required for camera diagnostics") from exc

        class _Node(Node):
            pass

        self._rclpy = rclpy
        self._node = _Node("gazebo_camera_pose_diagnostics")
        for name, default in (
            ("camera_label", "left"),
            ("parent_model_name", "left_stereo_camera"),
            ("world_name", "stereo_ground_drone"),
            ("output_topic", "/simulation/camera_pose_diagnostics/left"),
            ("generated_sdf_sha256", ""),
        ):
            self._node.declare_parameter(name, default)
        self._node.declare_parameter("desired_optical_position_world_enu_m", [0.0, 0.0, 0.0])
        self._node.declare_parameter("desired_optical_rpy_world_rad", [0.0, 0.0, 0.0])
        self._camera_label = str(self._node.get_parameter("camera_label").value)
        self._parent_model_name = str(self._node.get_parameter("parent_model_name").value)
        world_name = str(self._node.get_parameter("world_name").value)
        self._sdf_sha256 = str(self._node.get_parameter("generated_sdf_sha256").value)
        self._desired_position = tuple(float(value) for value in self._node.get_parameter("desired_optical_position_world_enu_m").value)
        self._desired_rpy = tuple(float(value) for value in self._node.get_parameter("desired_optical_rpy_world_rad").value)
        if len(self._desired_position) != 3 or len(self._desired_rpy) != 3:
            raise ValueError("desired optical pose parameters must each contain exactly three values")
        output_topic = str(self._node.get_parameter("output_topic").value)
        self._string_type = String
        self._publisher = self._node.create_publisher(
            String,
            output_topic,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._command = (
            "gz",
            "topic",
            "-e",
            "-t",
            f"/model/{self._parent_model_name}/pose_static",
            "--json-output",
        )
        self._base_state = PoseVState(f"{self._parent_model_name}::base_link")
        self._optical_state = PoseVState(f"{self._parent_model_name}::optical_link")
        self._shutdown = threading.Event()
        self._process: subprocess.Popen[str] | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def node(self) -> Any:
        return self._node

    def _run(self) -> None:
        while not self._shutdown.is_set():
            try:
                process = subprocess.Popen(self._command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
            except OSError as exc:
                self._node.get_logger().error(f"failed to start camera pose stream: {exc}")
                self._shutdown.wait(0.5)
                continue
            self._process = process
            try:
                for line in process.stdout or ():
                    if self._shutdown.is_set():
                        break
                    try:
                        payload = json.loads(line)
                        base = self._base_state.consume(payload)
                        optical = self._optical_state.consume(payload)
                        if base is None or optical is None:
                            continue
                        comparison = compare_camera_pose_payload(
                            payload,
                            camera_label=self._camera_label,
                            parent_model_name=self._parent_model_name,
                            desired_optical_position_world_enu_m=self._desired_position,  # type: ignore[arg-type]
                            desired_optical_rpy_world_rad=self._desired_rpy,  # type: ignore[arg-type]
                            expected_base_entity_id=self._base_state.entity_id,
                            expected_optical_entity_id=self._optical_state.entity_id,
                        )
                    except (json.JSONDecodeError, PosePayloadError, ValueError) as exc:
                        self._node.get_logger().warning(f"camera pose diagnostic unavailable: {exc}")
                        continue
                    message = {**asdict(comparison), "generated_sdf_sha256": self._sdf_sha256}
                    self._publisher.publish(
                        self._string_type(
                            data=json.dumps(message, sort_keys=True, separators=(",", ":"))
                        )
                    )
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                self._process = None
            if not self._shutdown.is_set():
                self._shutdown.wait(0.5)

    def shutdown(self) -> None:
        self._shutdown.set()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._node.destroy_node()


def main() -> None:  # pragma: no cover - requires live ROS/Gazebo
    import rclpy

    rclpy.init()
    adapter = GazeboCameraPoseDiagnostics()
    try:
        rclpy.spin(adapter.node)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
