"""Live ROS boundary for world-ENU GT, PX4 local NED and simulation clock."""

from __future__ import annotations

import threading
import time

from .runtime_observation import ObservationCache


class RosObservationBridge:
    def __init__(self, cache: ObservationCache) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from px4_msgs.msg import VehicleLocalPosition
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock

        self._rclpy = rclpy
        self.cache = cache
        self.last_error: str | None = None
        rclpy.init()
        self.node = Node("stereo_v1_1_flight_observer")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Clock, "/clock", self._clock, qos)
        self.node.create_subscription(
            PoseStamped, "/simulation/ground_truth/vehicle_reference_link", self._gt, qos
        )
        self.node.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._local, qos
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name="campaign-ros-observer", daemon=False)
        self.thread.start()

    def _safe(self, action) -> None:
        try:
            action()
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"

    def _clock(self, msg) -> None:
        self._safe(lambda: self.cache.observe_clock(
            int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        ))

    def _gt(self, msg) -> None:
        received = time.monotonic_ns()
        self._safe(lambda: self.cache.observe_gt(
            int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
            str(msg.header.frame_id),
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            received,
            [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w],
        ))

    def _local(self, msg) -> None:
        received = time.monotonic_ns()
        self._safe(lambda: self.cache.observe_local(
            timestamp_us=msg.timestamp,
            position_xyz=[msg.x, msg.y, msg.z],
            velocity_xyz=[msg.vx, msg.vy, msg.vz],
            heading_rad=msg.heading,
            xy_valid=msg.xy_valid,
            z_valid=msg.z_valid,
            xy_reset_counter=msg.xy_reset_counter,
            z_reset_counter=msg.z_reset_counter,
            receive_monotonic_ns=received,
        ))

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5.0)
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
