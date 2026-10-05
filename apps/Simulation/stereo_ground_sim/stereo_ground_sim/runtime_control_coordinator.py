"""ROS runtime activation, shutdown, Gazebo control, and image intake."""

import math
import threading
import time

from stereo_ground_sim.gazebo_target_controller import GazeboTargetController
from stereo_ground_sim.mission_timeline import MissionTimelineConfig, mission_state
from stereo_ground_sim.runtime_worker_lifecycle import RuntimeWorkerLifecycle
from stereo_ground_sim.time_alignment import image_stamp_ns
from stereo_ground_sim.visualization_payloads import plot_payloads

RUNTIME_CONTROL_METHODS = [
    "_should_subscribe_camera_images",
    "_should_load_yolo_model",
    "_measurement_processing_hz",
    "_initialize_yolo_runtime_if_enabled",
    "_create_camera_image_subscriptions_if_enabled",
    "_start_yolo_worker_if_enabled",
    "_start_rviz_live_worker_if_enabled",
    "_ensure_runtime_worker_lifecycle",
    "_disable_yolo_runtime",
    "_actual_distance",
    "_mission_state",
    "_publish_actual_distance",
    "_publish_ground_truth_position",
    "_plot_publish_loop",
    "_publish_plot_samples",
    "_read_actual_distance",
    "_finalize_research_recording",
    "shutdown",
    "_command_gazebo_target_pose",
    "_ensure_gazebo_target_controller",
    "_set_gazebo_target_pose",
    "_respawn_drone_service",
    "_left_cb",
    "_right_cb",
    "_store_image"
]


class RuntimeControlCoordinator:
    def __init__(self, node, *, float32_type, float32_multi_array_type, image_message_type, image_stamp):
        self.node = node
        self.float32_type = float32_type
        self.float32_multi_array_type = float32_multi_array_type
        self.image_message_type = image_message_type
        self.image_stamp = image_stamp

    def _should_subscribe_camera_images(self):
        if self.node.observation_mode == "ideal_reference_point":
            # The ideal-reference path still needs synchronized images to form
            # a measurement pair.  It does not run YOLO, so a zero YOLO rate
            # must not disable the camera subscriptions used for this oracle
            # and GPS-baseline comparison mode.
            return True
        if self.node.yolo_target_hz <= 0.0:
            return False
        return (
            self.node.observation_mode == "bbox_yolo"
            and bool(self.node.yolo_model_path.strip())
            and not self.node.yolo_runtime_disabled
        )

    def _should_load_yolo_model(self):
        return (
            self.node.observation_mode == "bbox_yolo"
            and self.node.yolo_target_hz > 0.0
            and bool(self.node.yolo_model_path.strip())
            and not self.node.yolo_runtime_disabled
        )

    def _measurement_processing_hz(self):
        """Return the input-processing rate for the active observation mode."""

        observation_mode = getattr(self.node, "observation_mode", "bbox_yolo")
        yolo_target_hz = getattr(self.node, "yolo_target_hz", 0.1)
        source_hz = (
            getattr(self.node, "camera_update_rate_hz", yolo_target_hz)
            if observation_mode == "ideal_reference_point"
            else yolo_target_hz
        )
        return max(0.1, float(source_hz))

    def _initialize_yolo_runtime_if_enabled(self):
        if not self.node._should_load_yolo_model():
            return False
        self.node.resolved_yolo_device = self.node._resolve_yolo_device()
        self.node._load_yolo_model()
        return True

    def _create_camera_image_subscriptions_if_enabled(self, camera_qos):
        self.node.image_subscriptions = []
        if not self.node._should_subscribe_camera_images():
            return False
        self.node.image_subscriptions.extend(
            [
                self.node.create_subscription(
                    self.image_message_type,
                    "/left_camera/image",
                    self.node._left_cb,
                    camera_qos,
                ),
                self.node.create_subscription(
                    self.image_message_type,
                    "/right_camera/image",
                    self.node._right_cb,
                    camera_qos,
                ),
            ]
        )
        return True

    def _start_yolo_worker_if_enabled(self):
        lifecycle = self.node._ensure_runtime_worker_lifecycle()
        started = lifecycle.start(
            "measurement",
            self.node._yolo_worker_loop,
            enabled=self.node._should_subscribe_camera_images(),
        )
        self.node.yolo_thread = lifecycle.threads["measurement"]
        return started

    def _start_rviz_live_worker_if_enabled(self):
        lifecycle = self.node._ensure_runtime_worker_lifecycle()
        enabled = (
            self.node._should_subscribe_camera_images()
            and getattr(self.node, "enable_rviz_rays", False)
            and getattr(self.node, "rviz_live_latest_only", False)
            and getattr(self.node, "observation_mode", "bbox_yolo") == "bbox_yolo"
        )
        started = lifecycle.start("rviz", self.node._rviz_live_worker_loop, enabled=enabled)
        self.node.rviz_live_thread = lifecycle.threads["rviz"]
        return started

    def _ensure_runtime_worker_lifecycle(self):
        lifecycle = getattr(self.node, "runtime_worker_lifecycle", None)
        if lifecycle is None:
            lifecycle = RuntimeWorkerLifecycle(threading.Thread)
            lifecycle.threads["measurement"] = getattr(self.node, "yolo_thread", None)
            lifecycle.threads["rviz"] = getattr(self.node, "rviz_live_thread", None)
            lifecycle.threads["plot"] = getattr(self.node, "plot_thread", None)
            self.node.runtime_worker_lifecycle = lifecycle
        return lifecycle

    def _disable_yolo_runtime(self, reason):
        self.node.yolo_runtime_disabled = True
        self.node.yolo_runtime_disable_reason = str(reason)
        self.node.yolo_load_error = str(reason)
        if not self.node.yolo_runtime_warning_logged:
            self.node.get_logger().warning(str(reason))
            self.node.yolo_runtime_warning_logged = True

    def _actual_distance(self):
        if self.node.cached_ground_truth_position is None:
            return None
        return self.node.cached_actual_distance

    def _mission_state(self):
        config = MissionTimelineConfig(
            profile=self.node.mission_profile,
            start_distance_m=self.node.approach_start_distance_m,
            end_distance_m=self.node.approach_end_distance_m,
            initial_world_enu_z_m=self.node.target_spawn_z_m,
            hover_world_enu_z_m=self.node.takeoff_altitude_m,
            takeoff_duration_s=self.node.takeoff_duration_s,
            first_hover_duration_s=self.node.first_hover_duration_s,
            approach_duration_s=self.node.approach_duration_s,
            final_hover_duration_s=self.node.final_hover_duration_s,
            land_duration_s=self.node.land_duration_s,
        )
        return mission_state(config, time.monotonic() - self.node.start_time)

    def _publish_actual_distance(self):
        state = self.node._mission_state()
        actual = self.node._read_actual_distance()
        if actual is not None:
            self.node.cached_actual_distance = actual
            self.node.reference_distance_m = actual
        with self.node.lock:
            self.node.metrics["actual_distance_m"] = actual
            self.node.metrics["mission_phase"] = self.node._recording_mission_phase(state["phase"])
        if actual is not None:
            self.node.actual_distance_pub.publish(self.float32_type(data=float(actual)))
            if self.node.cached_ground_truth_position is not None:
                self.node._publish_ground_truth_position()
        if self.node.fly_target_in_gazebo:
            self.node._command_gazebo_target_pose(state)
        self.node._publish_plot_samples()

    def _publish_ground_truth_position(self):
        position = self.node.cached_ground_truth_position
        if position is None:
            return
        self.node.ground_truth_position_pub.publish(
            self.float32_multi_array_type(
                data=[
                    float(position["forward_m"]),
                    float(position["right_m"]),
                    float(position["down_m"]),
                    float(position["range_m"]),
                ]
            )
        )

    def _plot_publish_loop(self):
        next_tick = time.monotonic()
        while not self.node.shutdown_event.is_set():
            self.node._publish_actual_distance()
            period_s = 1.0 / max(1.0, float(self.node.plot_publish_hz))
            next_tick += period_s
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                self.node.shutdown_event.wait(sleep_s)
            else:
                next_tick = time.monotonic()

    def _publish_plot_samples(self):
        with self.node.lock:
            metrics = dict(self.node.metrics)
        payload = plot_payloads(metrics, actual_fallback=self.node._actual_distance())
        self.node.distance_pub.publish(self.float32_type(data=payload["distance"]))
        self.node.error_pub.publish(self.float32_type(data=payload["error"]))
        self.node.disparity_pub.publish(self.float32_type(data=payload["disparity"]))
        self.node.nominal_pose_plot_pub.publish(
            self.float32_multi_array_type(data=payload["nominal_pose_frd_range"])
        )
        self.node.corrected_position_pub.publish(
            self.float32_multi_array_type(data=payload["corrected_frd_range"])
        )
        optical_publisher = getattr(self.node, "optical_pose_position_pub", None)
        if optical_publisher is not None:
            optical_publisher.publish(
                self.float32_multi_array_type(data=payload["optical_pose_frd_range"])
            )
        ideal_publisher = getattr(self.node, "ideal_position_pub", None)
        if ideal_publisher is not None:
            ideal_publisher.publish(self.float32_multi_array_type(data=payload["ideal_frd_range"]))
        if payload["bbox_adjustment"] is not None:
            self.node.bbox_adjustment_pub.publish(
                self.node._bbox_adjustment_to_msg(payload["bbox_adjustment"])
            )
        self.node.profile_status_pub.publish(self.node._profile_status_to_msg(metrics))
        self.node.comparison_pub.publish(self.float32_multi_array_type(data=payload["comparison"]))

    def _read_actual_distance(self):
        if self.node.cached_ground_truth_position is None:
            return None
        return self.node.cached_actual_distance

    def _finalize_research_recording(self):
        coordinator = self.node._ensure_research_terminal_coordinator_locked()
        if not coordinator.enabled:
            return None
        summary = coordinator.close(
            {
                "run_id": self.node.research_run_id, "case_id": self.node.research_case_id,
                "shutdown_drain_status": self.node.shutdown_drain_status,
                "accepted": self.node.measurement_accepted_count,
                "processed": self.node.measurement_processed_count,
                "discarded": self.node.accepted_pair_discard_count,
                "measurement_queue_overflow_count": self.node.measurement_queue_overflow_count,
                "measurement_overrun_invalid": self.node.measurement_overrun_invalid,
                "measurement_rejected_after_overrun_count": self.node.measurement_rejected_after_overrun_count,
                "measurement_processing_error_count": self.node.measurement_processing_error_count,
                "gt_evaluation_success_count": self.node.gt_evaluation_success_count,
                "gt_evaluation_error_count": self.node.gt_evaluation_error_count,
                "simulation_reset_count": self.node.simulation_epoch.reset_count,
                "input_rejected_after_shutdown_count": self.node.input_rejected_after_shutdown_count,
                "dropped_on_shutdown_count": self.node.dropped_on_shutdown_count,
                "remaining_measurement_queue": len(self.node.measurement_pair_queue),
                "remaining_gt_queue": len(self.node.gt_alignment_ready_queue),
                "inflight_pair": self.node.inflight_pair_id,
            },
            error=self.node.get_logger().error,
        )
        self.node.research_run_summary = summary
        self.node._sync_research_terminal_state(coordinator)
        return summary

    def shutdown(self):
        if not hasattr(self.node, "yolo_condition"):
            self.node.shutdown_event.set()
            self.node._finalize_research_recording()
            return
        started = time.monotonic()
        drain_timeout_s = 5.0
        with self.node.yolo_condition:
            self.node.accepting_input = False
            flushed_pairs = self.node.image_pairer.flush()
            self.node._register_measurement_pairs_locked(flushed_pairs)
            self.node.gt_alignment_ready_queue.extend(self.node.pending_gt_aligner.flush())
            self.node.drain_requested = True
            self.node.shutdown_initial_measurement_count = len(self.node.measurement_pair_queue)
            self.node.shutdown_initial_gt_alignment_count = len(self.node.gt_alignment_ready_queue)
            self.node.shutdown_initial_inflight_pair_id = self.node.inflight_pair_id
            self.node.yolo_condition.notify_all()
            if self.node.yolo_thread is None:
                self.node.drain_complete = (
                    not self.node.measurement_pair_queue and not self.node.gt_alignment_ready_queue
                )
            deadline = started + drain_timeout_s
            while not self.node.drain_complete and time.monotonic() < deadline:
                self.node.yolo_condition.wait(timeout=min(0.1, deadline - time.monotonic()))
            self.node.shutdown_drain_duration_s = time.monotonic() - started
            self.node.shutdown_remaining_measurement_count = len(self.node.measurement_pair_queue)
            self.node.shutdown_remaining_gt_alignment_count = len(self.node.gt_alignment_ready_queue)
            self.node.shutdown_inflight_pair_id = self.node.inflight_pair_id
            self.node.shutdown_gt_evaluation_error_count = getattr(
                self.node, "gt_evaluation_error_count", 0
            )
            if self.node.drain_complete:
                accounted = (
                    self.node.measurement_processed_count + self.node.accepted_pair_discard_count
                )
                self.node.shutdown_drain_status = (
                    "SUCCESS"
                    if accounted == self.node.measurement_accepted_count
                    and self.node.measurement_processing_error_count == 0
                    and self.node.shutdown_gt_evaluation_error_count == 0
                    else "INCOMPLETE"
                )
            else:
                self.node.shutdown_drain_status = "INCOMPLETE"
                self.node.discard_measurement_commits = True
                remaining_ids = {pair.pair_id for pair in self.node.measurement_pair_queue}
                remaining_ids.update(item[0].pair_id for item in self.node.gt_alignment_ready_queue)
                if self.node.inflight_pair_id is not None:
                    remaining_ids.add(self.node.inflight_pair_id)
                self.node.dropped_on_shutdown_count = len(remaining_ids)
                if remaining_ids:
                    self.node.dropped_on_shutdown_pair_id_min = min(remaining_ids)
                    self.node.dropped_on_shutdown_pair_id_max = max(remaining_ids)
                newly_discarded_ids = tuple(
                    pair_id
                    for pair_id in remaining_ids
                    if pair_id in self.node.pairs_by_pair_id
                    and pair_id not in self.node.measurement_results_by_pair_id
                )
                self.node.accepted_pair_discard_count += len(newly_discarded_ids)
                for pair_id in tuple(remaining_ids):
                    pair = self.node.pairs_by_pair_id.get(pair_id)
                    metrics = self.node.measurement_results_by_pair_id.get(pair_id)
                    self.node._submit_research_terminal_locked(
                        pair,
                        terminal_status="shutdown_discarded",
                        terminal_reason="shutdown_timeout",
                        measurement_status=(
                            "processed" if metrics is not None else "discarded"
                        ),
                        evaluation_status="not_evaluated",
                        metrics=metrics,
                        alignment=self.node.gt_alignments_by_pair_id.get(pair_id),
                    )
                    self.node._drop_pair_state_locked(pair_id, "shutdown_timeout")
        if self.node.shutdown_drain_status == "INCOMPLETE":
            self.node.get_logger().warning(
                "Shutdown drain INCOMPLETE: "
                f"duration_s={self.node.shutdown_drain_duration_s:.3f}, "
                f"remaining_measurements={self.node.shutdown_remaining_measurement_count}, "
                f"remaining_gt={self.node.shutdown_remaining_gt_alignment_count}, "
                f"inflight_pair_id={self.node.shutdown_inflight_pair_id}, "
                f"dropped={self.node.dropped_on_shutdown_count}, "
                f"measurement_errors={self.node.measurement_processing_error_count}, "
                f"evaluation_errors={self.node.shutdown_gt_evaluation_error_count}"
            )
        else:
            self.node.get_logger().info(
                "Shutdown drain SUCCESS: "
                f"duration_s={self.node.shutdown_drain_duration_s:.3f}, "
                f"initial_measurements={self.node.shutdown_initial_measurement_count}, "
                f"initial_gt={self.node.shutdown_initial_gt_alignment_count}, "
                f"initial_inflight={self.node.shutdown_initial_inflight_pair_id}, "
                f"accepted={self.node.measurement_accepted_count}, "
                f"processed={self.node.measurement_processed_count}, "
                f"evaluation_successes={getattr(self.node, 'gt_evaluation_success_count', 0)}, "
                f"evaluation_errors={self.node.shutdown_gt_evaluation_error_count}"
            )
        self.node.shutdown_event.set()
        with self.node.yolo_condition:
            self.node.yolo_condition.notify_all()
        lifecycle = self.node._ensure_runtime_worker_lifecycle()
        lifecycle.threads["measurement"] = self.node.yolo_thread
        lifecycle.threads["rviz"] = getattr(self.node, "rviz_live_thread", None)
        lifecycle.join("measurement", 2.0)
        lifecycle.join("rviz", 2.0)
        self.node._shutdown_yolo_process()
        self.node._finalize_research_recording()

    def _command_gazebo_target_pose(self, state):
        controller = self.node._ensure_gazebo_target_controller()
        result = controller.command_mission_state(
            spawn_world_enu_x_m=self.node.target_spawn_x_m,
            state=state,
            yaw_world_enu_deg=self.node.target_spawn_yaw_deg,
        )
        self.node.last_gazebo_velocity_update = controller.last_command_monotonic_s
        return result

    def _ensure_gazebo_target_controller(self):
        controller = getattr(self.node, "gazebo_target_controller", None)
        if controller is None:
            controller = GazeboTargetController(
                self.node.target_model_name,
                warning=self.node.get_logger().warning,
            )
            controller.last_command_monotonic_s = getattr(
                self.node, "last_gazebo_velocity_update", 0.0
            )
            self.node.gazebo_target_controller = controller
        return controller

    def _set_gazebo_target_pose(self, x_m, y_m, z_m, yaw_deg):
        return self.node._ensure_gazebo_target_controller().set_pose(
            x_m, y_m, z_m, yaw_deg
        )

    def _respawn_drone_service(self, _request, response):
        """Teleport the target model back to the configured launch pose."""

        success = self.node._set_gazebo_target_pose(
            self.node.target_spawn_x_m,
            self.node.target_spawn_y_m,
            self.node.target_spawn_z_m,
            self.node.target_spawn_yaw_deg,
        )
        if success:
            with self.node.yolo_condition:
                self.node._reset_ground_truth_after_respawn_locked()
            self.node.start_time = time.monotonic()
            self.node.last_gazebo_velocity_update = 0.0
            self.node._ensure_gazebo_target_controller().last_command_monotonic_s = 0.0
            response.success = True
            response.message = (
                f"Respawned {self.node.target_model_name} at "
                f"({self.node.target_spawn_x_m:.2f}, {self.node.target_spawn_y_m:.2f}, "
                f"{self.node.target_spawn_z_m:.2f}) m"
            )
        else:
            response.success = False
            response.message = "Gazebo set_pose service failed"
        return response

    def _left_cb(self, msg):
        self.node._store_image("left", msg)

    def _right_cb(self, msg):
        self.node._store_image("right", msg)

    def _store_image(self, side, msg):
        try:
            stamp_ns = self.image_stamp(msg)
        except ValueError as exc:
            self.node.image_pairer.invalid_timestamp_count += 1
            self.node.get_logger().warning(f"Invalid {side} image simulation timestamp: {exc}")
            return
        with self.node.yolo_condition:
            if not self.node.accepting_input:
                self.node.input_rejected_after_shutdown_count += 1
                return
            latest_clock = self.node.simulation_epoch.last_clock_stamp_ns
            if latest_clock is not None and stamp_ns > latest_clock:
                lead_ns = stamp_ns - latest_clock
                if lead_ns <= getattr(self.node, "image_reorder_lead_limit_ns", 0):
                    pass
                else:
                    self.node.stale_pre_reset_count += 1
                    self.node.get_logger().warning(
                        f"{side} image stamp {stamp_ns} is ahead of current "
                        f"/clock {latest_clock} by {lead_ns} ns; ignored"
                    )
                    return
            elif not self.node.simulation_epoch.accepts_stamp(stamp_ns):
                self.node.stale_pre_reset_count += 1
                self.node.get_logger().warning(
                    f"{side} image stamp {stamp_ns} is ahead of current /clock {latest_clock}; ignored"
                )
                return
            try:
                pairs = self.node.image_pairer.add(
                    side,
                    stamp_ns,
                    msg,
                    epoch=self.node.simulation_epoch.epoch,
                )
            except ValueError as exc:
                self.node.get_logger().warning(f"Invalid {side} image simulation timestamp: {exc}")
                return
            self.node._register_measurement_pairs_locked(pairs)
