"""Simulation epoch, pair intake, and Ground Truth routing coordinator.

Ground Truth records enter as World ENU metres and are converted by the
GroundTruthProvider before any baseline-center FRD evaluation. PX4/body and
camera optical frames remain owned by their dedicated pose providers.
"""

from collections import deque
import time

from stereo_ground_sim.coordinate_frames import (
    SOURCE_GAZEBO_ENTITY,
    VEHICLE_REFERENCE_DEFINITION,
    VEHICLE_REFERENCE_LINK,
    WORLD_FRAME,
    ground_truth_world_to_baseline,
)
from stereo_ground_sim.ground_truth_provider import GroundTruthProvider
from stereo_ground_sim.pair_work_scheduler import PairWorkScheduler
from stereo_ground_sim.pose_source_provider import PoseSourceProvider
from stereo_ground_sim.rviz_geometry import reference_ray_world_enu_from_point
from stereo_ground_sim.time_alignment import PendingGroundTruthRecord

PAIR_GROUND_TRUTH_ROUTING_METHODS = [
    "_clock_cb",
    "_reset_for_simulation_epoch_locked",
    "_capture_operational_sources_locked",
    "_ensure_pose_source_provider_locked",
    "_register_measurement_pairs_locked",
    "_record_measurement_queue_depth_locked",
    "_ensure_pair_work_scheduler_locked",
    "_drop_pair_state_locked",
    "_finalize_measurement_queue_overflow_locked",
    "_ground_truth_pose_cb",
    "_reject_ground_truth_record_locked",
    "_ensure_ground_truth_provider_locked",
    "_sync_ground_truth_provider_state",
    "_route_ground_truth_record_locked",
    "_try_release_gt_quarantine_locked",
    "_queue_or_accept_ground_truth_record_locked",
    "_accept_ground_truth_locked",
    "_drain_future_ground_truth_locked"
]


class PairGroundTruthCoordinator:
    def __init__(self, node):
        self.node = node

    def _clock_cb(self, msg):
        try:
            stamp_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        except (AttributeError, TypeError, ValueError):
            self.node.get_logger().warning("Invalid /clock message ignored")
            return
        with self.node.yolo_condition:
            event = self.node.simulation_epoch.observe_clock(stamp_ns)
            if event.reset:
                self.node._reset_for_simulation_epoch_locked(event.epoch, event.stamp_ns)
                self.node.get_logger().warning(
                    f"Simulation /clock decreased; entered epoch {event.epoch}"
                )
            else:
                self.node._drain_future_ground_truth_locked(stamp_ns)

    def _reset_for_simulation_epoch_locked(self, epoch, reset_stamp_ns=None):
        self.node._ensure_gt_evaluation_state_locked()
        self.node.observed_left_camera = None
        self.node.observed_right_camera = None
        self.node.camera_oracle_pair = (None, None)
        self.node.last_camera_oracle_rejection_reason = "simulation_epoch_reset"
        pending_pairs = tuple(self.node.pairs_by_pair_id.values())
        discarded_pending = sum(
            pair.pair_id not in self.node.measurement_results_by_pair_id
            for pair in pending_pairs
        )
        self.node.accepted_pair_discard_count += discarded_pending
        for pair in pending_pairs:
            self.node._submit_research_terminal_locked(
                pair,
                terminal_status="simulation_reset_discarded",
                terminal_reason="simulation_clock_reset",
                measurement_status=(
                    "processed"
                    if pair.pair_id in self.node.measurement_results_by_pair_id
                    else "discarded"
                ),
                evaluation_status="not_evaluated",
                metrics=self.node.measurement_results_by_pair_id.get(pair.pair_id),
                alignment=self.node.gt_alignments_by_pair_id.get(pair.pair_id),
            )
        dropped = self.node.image_pairer.reset_for_epoch(epoch)
        dropped += self.node.pending_gt_aligner.clear()
        dropped += len(self.node.measurement_pair_queue)
        dropped += len(self.node.gt_alignment_ready_queue)
        self.node.measurement_pair_queue.clear()
        self.node.gt_alignment_ready_queue.clear()
        self.node.rviz_live_latest_pair = None
        self.node.rviz_live_latest_result = None
        self.node.rviz_live_latest_processed_pair_id = 0
        self.node.gt_position_buffer.reset_for_epoch(epoch)
        if reset_stamp_ns is None:
            reset_stamp_ns = self.node.simulation_epoch.last_clock_stamp_ns
        reset_stamp_ns = 0 if reset_stamp_ns is None else int(reset_stamp_ns)
        duration_ns = int(
            getattr(self.node, "gt_reset_quarantine_duration_ns", 200_000_000)
        )
        provider = self.node._ensure_ground_truth_provider_locked()
        provider.reset(reset_stamp_ns, duration_ns)
        self.node._sync_ground_truth_provider_state(provider)
        self.node.measurement_results_by_pair_id.clear()
        self.node.gt_alignments_by_pair_id.clear()
        self.node.pairs_by_pair_id.clear()
        getattr(self.node, "operational_sources_by_pair_id", {}).clear()
        self.node.evaluated_pair_ids.clear()
        self.node.gt_evaluation_failed_pair_ids.clear()
        self.node.gt_published_pair_ids.clear()
        self.node.cached_ground_truth_position = None
        self.node.cached_actual_distance = None
        self.node.ground_truth_stamp_ns = None
        self.node.old_epoch_frame_count += dropped

    def _capture_operational_sources_locked(self):
        """Freeze the sensor values available when an image pair is accepted.

        The image timestamps and ``captured_simulation_timestamp_ns`` use the
        Gazebo/ROS Simulation clock.  PX4 source timestamps remain separately
        recorded in their native microsecond domain.  No ENU/NED/FRD/RDF value
        is converted here; this is only an immutable timing/provenance snapshot.
        """

        provider = self.node._ensure_pose_source_provider_locked()
        epoch = getattr(self.node, "simulation_epoch", None)
        return provider.snapshot(
            simulation_timestamp_ns=getattr(epoch, "last_clock_stamp_ns", None),
            monotonic_ns=time.monotonic_ns(),
            epoch=getattr(epoch, "epoch", 0),
        )

    def _ensure_pose_source_provider_locked(self):
        provider = getattr(self.node, "pose_source_provider", None)
        if provider is None:
            provider = PoseSourceProvider()
            provider.attitudes = getattr(
                self.node, "px4_attitudes", {"left": None, "right": None}
            )
            provider.raw_gps = getattr(
                self.node,
                "gps_baseline_fixes",
                {"left": None, "right": None, "uam": None},
            )
            provider.raw_gps_failures = getattr(
                self.node,
                "gps_baseline_failures",
                {"left": None, "right": None, "uam": None},
            )
            provider.global_positions = getattr(
                self.node, "global_baseline_fixes", {"left": None, "right": None}
            )
            self.node.pose_source_provider = provider
        self.node.px4_attitudes = provider.attitudes
        self.node.gps_baseline_fixes = provider.raw_gps
        self.node.gps_baseline_failures = provider.raw_gps_failures
        self.node.global_baseline_fixes = provider.global_positions
        return provider

    def _register_measurement_pairs_locked(self, pairs):
        ready_count = 0
        for pair in pairs:
            if pair.epoch != self.node.simulation_epoch.epoch:
                self.node.old_epoch_frame_count += 1
                continue
            if (
                getattr(self.node, "rviz_live_latest_only", False)
                and getattr(self.node, "enable_rviz_rays", False)
                and getattr(self.node, "observation_mode", "bbox_yolo") == "bbox_yolo"
            ):
                self.node.rviz_live_latest_pair = pair
            if self.node.measurement_overrun_invalid:
                self.node.measurement_rejected_after_overrun_count += 1
                self.node.measurement_rejected_pair_ids.add(pair.pair_id)
                self.node.last_measurement_rejection_reason = (
                    "measurement_run_invalid_after_queue_overrun"
                )
                if self.node.measurement_rejected_after_overrun_count == 1 or (
                    self.node.measurement_rejected_after_overrun_count % 100 == 0
                ):
                    self.node.get_logger().warning(
                        "Measurement input rejected after queue overrun invalidated "
                        f"the run: rejected_pair_id={pair.pair_id}"
                    )
                continue
            if (
                getattr(self.node, "measurement_latest_only", False)
                and getattr(self.node, "observation_mode", "bbox_yolo") == "bbox_yolo"
                and not getattr(self.node, "enable_research_recording", False)
            ):
                while self.node.measurement_pair_queue:
                    stale_pair = self.node.measurement_pair_queue.popleft()
                    self.node._drop_pair_state_locked(stale_pair.pair_id, "measurement_latest_only")
                    self.node.measurement_latest_drop_count = (
                        getattr(self.node, "measurement_latest_drop_count", 0) + 1
                    )
            if len(self.node.measurement_pair_queue) >= self.node.measurement_queue_capacity:
                self.node.measurement_queue_overflow_count += 1
                self.node.measurement_overrun_invalid = True
                self.node.measurement_rejected_pair_ids.add(pair.pair_id)
                self.node.last_measurement_rejection_reason = "measurement_queue_overflow"
                self.node._drop_pair_state_locked(pair.pair_id, "measurement_queue_overflow")
                if self.node.measurement_queue_overflow_count == 1 or (
                    self.node.measurement_queue_overflow_count % 100 == 0
                ):
                    self.node.get_logger().error(
                        "Measurement queue overrun: strict research run is invalid; "
                        f"capacity={self.node.measurement_queue_capacity}, rejected_pair_id={pair.pair_id}"
                    )
                continue
            self.node.pairs_by_pair_id[pair.pair_id] = pair
            snapshots = getattr(self.node, "operational_sources_by_pair_id", None)
            if snapshots is None:
                snapshots = {}
                self.node.operational_sources_by_pair_id = snapshots
            snapshots[pair.pair_id] = self.node._capture_operational_sources_locked()
            if getattr(self.node, "observation_mode", "bbox_yolo") == "bbox_yolo":
                self.node.measurement_pair_queue.append(pair)
            alignments = self.node.pending_gt_aligner.add_pair(pair)
            self.node.gt_alignment_ready_queue.extend(alignments)
            self.node.measurement_accepted_count += 1
            self.node._record_measurement_queue_depth_locked()
            ready_count += 1
        if ready_count:
            self.node.yolo_condition.notify_all()
        return ready_count

    def _record_measurement_queue_depth_locked(self):
        depth = len(self.node.measurement_pair_queue)
        self.node.measurement_queue_depth_sample_count += 1
        self.node.measurement_queue_depth_sum += depth
        self.node.measurement_queue_depth_min = (
            depth if self.node.measurement_queue_depth_min is None
            else min(self.node.measurement_queue_depth_min, depth)
        )
        self.node.measurement_queue_depth_max = max(self.node.measurement_queue_depth_max, depth)

    def _ensure_pair_work_scheduler_locked(self):
        scheduler = getattr(self.node, "pair_work_scheduler", None)
        if scheduler is None:
            scheduler = PairWorkScheduler()
            scheduler.measurements = getattr(self.node, "measurement_pair_queue", deque())
            scheduler.alignments = getattr(self.node, "gt_alignment_ready_queue", deque())
            scheduler.prefer_alignment_next = bool(
                getattr(self.node, "_prefer_gt_alignment_next", False)
            )
            self.node.pair_work_scheduler = scheduler
        self.node.measurement_pair_queue = scheduler.measurements
        self.node.gt_alignment_ready_queue = scheduler.alignments
        return scheduler

    def _drop_pair_state_locked(self, pair_id, reason):
        """Remove every pending state for one pair under ``yolo_condition``."""

        pair_id = int(pair_id)
        scheduler = self.node._ensure_pair_work_scheduler_locked()
        scheduler.remove_pair(pair_id)
        self.node.measurement_pair_queue = scheduler.measurements
        self.node.gt_alignment_ready_queue = scheduler.alignments
        self.node.pending_gt_aligner.remove_pair(pair_id)
        self.node.pairs_by_pair_id.pop(pair_id, None)
        getattr(self.node, "operational_sources_by_pair_id", {}).pop(pair_id, None)
        self.node.measurement_results_by_pair_id.pop(pair_id, None)
        self.node.gt_alignments_by_pair_id.pop(pair_id, None)
        self.node.evaluated_pair_ids.discard(pair_id)
        self.node.gt_published_pair_ids.discard(pair_id)
        return reason

    def _finalize_measurement_queue_overflow_locked(self, pair_id):
        """Terminally reject one already-accepted pair on queue overflow."""
        pair_id = int(pair_id)
        if pair_id not in self.node.pairs_by_pair_id:
            return False
        self.node.measurement_queue_overflow_count += 1
        self.node.measurement_overrun_invalid = True
        self.node.measurement_rejected_pair_ids.add(pair_id)
        self.node.accepted_pair_discard_count += 1
        self.node.last_measurement_rejection_reason = "measurement_queue_overflow"
        self.node._submit_research_terminal_locked(
            self.node.pairs_by_pair_id.get(pair_id),
            terminal_status="measurement_queue_overflow",
            terminal_reason="measurement_queue_overflow",
            measurement_status="discarded",
            evaluation_status="not_evaluated",
            metrics=self.node.measurement_results_by_pair_id.get(pair_id),
            alignment=self.node.gt_alignments_by_pair_id.get(pair_id),
        )
        self.node._drop_pair_state_locked(pair_id, "measurement_queue_overflow")
        return True

    def _ground_truth_pose_cb(self, msg):
        if str(msg.header.frame_id) != WORLD_FRAME:
            self.node.get_logger().warning(
                f"Ground Truth ignored: expected frame_id={WORLD_FRAME}, got {msg.header.frame_id!r}"
            )
            return
        self.node._ensure_camera_frames()
        point_world = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        orientation_xyzw = (
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        # RViz vehicle rendering is published directly by
        # ``gazebo_ground_truth_adapter`` on ``/stereo/ground_truth_markers``.
        # Keeping it out of the ray stream prevents the initial marker from
        # lingering when image-pair or inference work is unavailable.
        try:
            # The Ideal RViz ray is a display-only World-ENU reference and
            # does not require detector pixels.  Publish it directly from GT
            # so a missing/disabled YOLO model cannot suppress every ray at
            # startup.  Measurement and CSV outputs remain pair-driven.
            self._publish_ground_truth_reference_ray(point_world)
            position = ground_truth_world_to_baseline(point_world, self.node.baseline_center_frd)
            if hasattr(self.node, "gt_position_buffer"):
                with self.node.yolo_condition:
                    if not self.node.accepting_input:
                        return
                    record = PendingGroundTruthRecord(
                        stamp_ns=stamp_ns,
                        epoch=self.node.simulation_epoch.epoch,
                        position_world_enu_m=point_world,
                        orientation_world_from_reference_xyzw=orientation_xyzw,
                    )
                    self.node._route_ground_truth_record_locked(record, position)
        except (TypeError, ValueError) as exc:
            self.node.inconsistent_clock_count += 1
            self.node.get_logger().warning(f"Ground Truth ignored: {exc}")
            return

    def _publish_ground_truth_reference_ray(self, point_world_enu_m):
        """Publish one display-only Ideal ray without waiting for an image pair.

        Input target and camera origins are Gazebo World ENU metres. Camera
        orientations retain the configured optical RDF contract; the helper
        returns Marker geometry in World ENU.
        """

        if not getattr(self.node, "enable_rviz_rays", False):
            return False
        if getattr(self.node, "ray_marker_pub", None) is None:
            return False
        now = time.monotonic()
        publish_hz = max(
            0.1, float(getattr(self.node, "rviz_live_ray_hz", 15.0))
        )
        last_publish = float(
            getattr(self.node, "last_ideal_ray_marker_publish_monotonic", 0.0)
        )
        if now - last_publish < 1.0 / publish_hz:
            return False
        self.node._ensure_camera_frames()
        frame = getattr(self.node, "baseline_center_frd", None)
        left = getattr(self.node, "nominal_left_camera", None)
        right = getattr(self.node, "nominal_right_camera", None)
        if frame is None or left is None or right is None:
            return False
        ideal = reference_ray_world_enu_from_point(
            frame,
            left,
            right,
            point_world_enu_m,
        )
        if ideal is None:
            return False
        self.node._publish_ray_markers(ideal=ideal)
        self.node.last_ideal_ray_marker_publish_monotonic = now
        return True

    def _reject_ground_truth_record_locked(self, *, quarantine=False):
        provider = self.node._ensure_ground_truth_provider_locked()
        provider.reject(quarantine=quarantine)
        self.node._sync_ground_truth_provider_state(provider)

    def _ensure_ground_truth_provider_locked(self):
        provider = getattr(self.node, "ground_truth_provider", None)
        if provider is None:
            provider = GroundTruthProvider(
                capacity=getattr(self.node, "future_gt_capacity", 64),
                reorder_lead_limit_ns=getattr(
                    self.node, "gt_reorder_lead_limit_ns", 200_000_000
                ),
            )
            provider.future = getattr(self.node, "future_gt_pending", deque())
            provider.quarantine_candidates = getattr(
                self.node, "gt_quarantine_candidates", deque()
            )
            provider.quarantine_active = getattr(self.node, "gt_quarantine_active", False)
            provider.quarantine_floor_stamp_ns = getattr(
                self.node, "gt_quarantine_floor_stamp_ns", None
            )
            provider.quarantine_last_stamp_ns = getattr(
                self.node, "gt_quarantine_last_stamp_ns", None
            )
            provider.quarantine_required_samples = getattr(
                self.node, "gt_quarantine_required_contiguous_samples", 2
            )
            provider.future_overflow_count = getattr(self.node, "future_gt_overflow_count", 0)
            provider.stale_count = getattr(self.node, "stale_pre_reset_gt_count", 0)
            provider.quarantine_rejected_count = getattr(
                self.node, "gt_quarantine_rejected_count", 0
            )
            provider.quarantine_release_count = getattr(
                self.node, "gt_quarantine_release_count", 0
            )
            self.node.ground_truth_provider = provider
        return provider

    def _sync_ground_truth_provider_state(self, provider):
        self.node.future_gt_pending = provider.future
        self.node.gt_quarantine_candidates = provider.quarantine_candidates
        self.node.gt_quarantine_active = provider.quarantine_active
        self.node.gt_quarantine_floor_stamp_ns = provider.quarantine_floor_stamp_ns
        self.node.gt_quarantine_last_stamp_ns = provider.quarantine_last_stamp_ns
        self.node.future_gt_overflow_count = provider.future_overflow_count
        self.node.stale_pre_reset_gt_count = provider.stale_count
        self.node.gt_quarantine_rejected_count = provider.quarantine_rejected_count
        self.node.gt_quarantine_release_count = provider.quarantine_release_count

    def _route_ground_truth_record_locked(self, record, position=None):
        provider = self.node._ensure_ground_truth_provider_locked()
        result = provider.route(
            record,
            epoch=self.node.simulation_epoch.epoch,
            latest_clock_ns=self.node.simulation_epoch.last_clock_stamp_ns,
            accept=lambda item: self.node._accept_ground_truth_locked(
                item, position if item is record else None
            ),
            warning=self.node.get_logger().warning,
        )
        self.node._sync_ground_truth_provider_state(provider)
        return result

    def _try_release_gt_quarantine_locked(self):
        provider = self.node._ensure_ground_truth_provider_locked()
        result = provider.release_if_ready(
            epoch=self.node.simulation_epoch.epoch,
            latest_clock_ns=self.node.simulation_epoch.last_clock_stamp_ns,
            accept=self.node._accept_ground_truth_locked,
            warning=self.node.get_logger().warning,
        )
        self.node._sync_ground_truth_provider_state(provider)
        return result

    def _queue_or_accept_ground_truth_record_locked(self, record, position=None):
        provider = self.node._ensure_ground_truth_provider_locked()
        result = provider.queue_or_accept(
            record,
            epoch=self.node.simulation_epoch.epoch,
            latest_clock_ns=self.node.simulation_epoch.last_clock_stamp_ns,
            accept=lambda item: self.node._accept_ground_truth_locked(item, position),
            warning=self.node.get_logger().warning,
        )
        self.node._sync_ground_truth_provider_state(provider)
        return result

    def _accept_ground_truth_locked(self, record, position=None):
        if record.epoch != self.node.simulation_epoch.epoch:
            self.node._reject_ground_truth_record_locked(quarantine=True)
            return False
        if position is None:
            position = ground_truth_world_to_baseline(
                record.position_world_enu_m, self.node.baseline_center_frd
            )
        self.node.gt_position_buffer.add(
            record.stamp_ns,
            record.position_world_enu_m,
            record.orientation_world_from_reference_xyzw,
            epoch=record.epoch,
        )
        aligned_pairs = self.node.pending_gt_aligner.pop_watermarked()
        self.node.gt_alignment_ready_queue.extend(aligned_pairs)
        if aligned_pairs:
            self.node.yolo_condition.notify_all()
        position = dict(position)
        position.update(
            {
                "ground_truth_source": "gazebo_vehicle_reference_link_world_pose",
                "source_gazebo_entity": SOURCE_GAZEBO_ENTITY,
                "published_reference_name": VEHICLE_REFERENCE_LINK,
                "reference_definition": VEHICLE_REFERENCE_DEFINITION,
                "ground_truth_world_enu_m": record.position_world_enu_m,
                "ground_truth_stamp_ns": record.stamp_ns,
            }
        )
        self.node.cached_ground_truth_position = position
        self.node.ground_truth_stamp_ns = position["ground_truth_stamp_ns"]
        self.node.cached_actual_distance = float(position["forward_m"])
        self.node.actual_distance_sample_seq = getattr(self.node, "actual_distance_sample_seq", 0) + 1
        return True

    def _drain_future_ground_truth_locked(self, clock_stamp_ns):
        provider = self.node._ensure_ground_truth_provider_locked()
        provider.drain(
            clock_stamp_ns,
            epoch=self.node.simulation_epoch.epoch,
            accept=self.node._accept_ground_truth_locked,
        )
        self.node._sync_ground_truth_provider_state(provider)
