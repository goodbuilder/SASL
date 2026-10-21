"""Live ROS boundary for world-ENU GT, PX4 local NED and simulation clock."""

from __future__ import annotations

import threading
import time

from .runtime_observation import ObservationCache


class RosObservationBridge:
    """ROS observation owner; optional sink must buffer records without disk I/O.

    Diagnostics are opt-in, with monotonic nanosecond receipt/lock timings.
    Sink failures increment diagnostic_error_count and never change control.
    """
    def __init__(self, cache: ObservationCache, diagnostic_sink=None) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from px4_msgs.msg import VehicleLocalPosition
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock

        self.diagnostic_sink = diagnostic_sink
        self.diagnostic_error_count = 0
        self._clock_handler_lock = threading.RLock()
        self._clock_handler = None
        self._rclpy = rclpy
        self.cache = cache
        self.last_error: str | None = None
        rclpy.init()
        self.node = Node("stereo_v1_1_flight_observer")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(
            Clock, "/clock", self._clock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
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

    def set_clock_handler(self, handler, *, initial_snapshot=None) -> None:
        """Replace/remove science observer; removal waits for active callback.

        Handler receives the current validated snapshot (possibly None) and
        callback receive monotonic ns. It must not invoke this API from another
        thread while holding a lock needed by its active handler.
        """
        with self._clock_handler_lock:
            if handler is not None and initial_snapshot is not None:
                handler(initial_snapshot, time.monotonic_ns())
            self._clock_handler = handler

    def _clock(self, msg) -> None:
        received = time.monotonic_ns()
        sink = self.diagnostic_sink
        details = ({"component": "clock_observer", "action": "clock_received",
                    "clock_receive_monotonic_ns": received,
                    "source_timestamp_ns": int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)}
                   if sink is not None else None)
        def observe():
            with self._clock_handler_lock:
                if details is not None:
                    acquired = time.monotonic_ns()
                    details.update(handler_lock_acquired_monotonic_ns=acquired,
                                   handler_lock_wait_ns=acquired-received)
                self.cache.observe_clock(
                    int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec), received)
                if self._clock_handler is not None and self.last_error is None:
                    if details is None:
                        snapshot = self.cache.snapshot(received)
                    else:
                        snapshot = self.cache.snapshot(received, diagnostic_details=details)
                        details["action"] = "handler_called"
                    self._clock_handler(snapshot, received)
                elif details is not None:
                    details["action"] = "no_handler" if self._clock_handler is None else "observer_error"
        self._safe(observe)
        if details is not None:
            details["callback_end_monotonic_ns"] = time.monotonic_ns()
            details["observer_error"] = self.last_error
            try:
                sink(details)
            except Exception:
                self.diagnostic_error_count += 1

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
        self.set_clock_handler(None)
        executor_stopped = self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5.0)
        self.diagnostic_shutdown_complete = bool(executor_stopped) and not self.thread.is_alive()
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
