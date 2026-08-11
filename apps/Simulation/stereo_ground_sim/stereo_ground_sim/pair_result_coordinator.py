"""Pair measurement commit, Ground Truth evaluation, and terminal coordination.

All estimate and Ground Truth vectors entering error calculation must already
share baseline-center FRD [Forward, Right, Down] metres. World-ENU Ground Truth
conversion is explicit and occurs before evaluation.
"""

import time

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME
from stereo_ground_sim.coordinate_frames import (
    ground_truth_world_to_baseline,
)
from stereo_ground_sim.diagnostic_ray_pipeline import ray_gt_error
from stereo_ground_sim.evaluation_pipeline import evaluation_metadata, vector_error
from stereo_ground_sim.observation_contract import (
    observation_metadata,
    observation_record_fields,
)
from stereo_ground_sim.pair_evaluation_registry import PairEvaluationRegistry
from stereo_ground_sim.pair_measurement_orchestrator import PairMeasurementOrchestrator
from stereo_ground_sim.px4_pose_contract import (
    PX4_ATTITUDE_COMPOSITION,
    PX4_ATTITUDE_FRAME,
    PX4_ATTITUDE_MODEL_VERSION,
    PX4_ATTITUDE_REFERENCE_SOURCE,
)
from stereo_ground_sim.rviz_geometry import (
    reference_ray_world_enu_from_point,
    uam_gps_ray_visualization_world_enu_from_point,
)
from stereo_ground_sim.time_alignment import BoundedIdHistory


PAIR_RESULT_COORDINATOR_METHODS = [
    "_finalize_pair_failure_locked",
    "_compute_pair_measurement",
    "_ensure_pair_measurement_orchestrator",
    "_rviz_live_worker_loop",
    "_latest_rviz_ideal_ray",
    "_commit_pair_measurement_if_current",
    "_complete_pair_measurement_locked",
    "_observation_metadata",
    "_observation_record_fields",
    "_update_metrics_from_messages",
    "_ground_truth_alignment_to_baseline",
    "_store_gt_alignment_locked",
    "_evaluate_pair_with_fault_containment_locked",
    "_finalize_gt_evaluation_failure_locked",
    "_ensure_gt_evaluation_state_locked",
    "_ensure_pair_evaluation_registry_locked",
    "_try_evaluate_pair_locked",
    "_vector_error",
    "_evaluation_metadata",
    "_pair_result_metadata",
    "_time_alignment_counters"
]


class PairResultCoordinator:
    def __init__(self, node, *, float32_type, camera_oracle_unset):
        self.node = node
        self.float32_type = float32_type
        self.camera_oracle_unset = camera_oracle_unset

    def _finalize_pair_failure_locked(self, pair, exc):
        """Contain one pair failure and leave the worker ready for the next pair."""

        self.node.measurement_processing_error_count += 1
        self.node.accepted_pair_discard_count += 1
        self.node.last_measurement_error_type = type(exc).__name__
        self.node.last_measurement_error_message = str(exc)
        self.node._submit_research_terminal_locked(
            pair,
            terminal_status="measurement_processing_error",
            terminal_reason=f"{type(exc).__name__}: {exc}",
            measurement_status="discarded",
            evaluation_status="not_evaluated",
            metrics=self.node.measurement_results_by_pair_id.get(pair.pair_id),
            alignment=self.node.gt_alignments_by_pair_id.get(pair.pair_id),
        )
        self.node._drop_pair_state_locked(pair.pair_id, "measurement_processing_error")
        if self.node.inflight_pair_id == pair.pair_id:
            self.node.inflight_pair_id = None
            self.node.inflight_pair_epoch = None
        self.node.yolo_inference_in_progress = False
        self.node.yolo_condition.notify_all()

    def _compute_pair_measurement(self, pair, phase, device=None):
        """Compute one pair without publishing or mutating pair result state."""
        return self.node._ensure_pair_measurement_orchestrator().compute(
            pair, phase, device=device
        )

    def _ensure_pair_measurement_orchestrator(self):
        orchestrator = getattr(self.node, "pair_measurement_orchestrator", None)
        if orchestrator is None:
            dependencies = {
                "camera_snapshot": self.node._camera_oracle_snapshot,
                "operational_snapshot": lambda pair_id: getattr(
                    self.node, "operational_sources_by_pair_id", {}
                ).get(pair_id),
                "observation_mode": lambda: getattr(
                    self.node, "observation_mode", "bbox_yolo"
                ),
                "alignment": lambda pair_id: getattr(
                    self.node, "gt_alignments_by_pair_id", {}
                ).get(pair_id),
                "ideal_detection": lambda point, width, height, snapshot: self.node._ideal_detection_pair_from_reference_world(
                    point, width, height, camera_oracle_snapshot=snapshot
                ),
                "decode": self.node._msg_to_rgb,
                "device": lambda: self.node.resolved_yolo_device
                or self.node._resolve_yolo_device(),
                "detect": lambda left, right, selected: self.node._detect_drone_pair(
                    left, right, device=selected
                ),
                "nominal": self.node._triangulate_center_pixels,
                "optical": lambda *args: self.node._triangulate_pixel_centers_oracle(
                    *args[:-1], camera_oracle_snapshot=args[-1]
                ),
                "gps": self.node._triangulate_gps_baselines,
                "diagnostics": lambda *args: self.node._diagnostic_rays(
                    *args[:7],
                    operational_snapshot=args[7],
                    observation_timestamp_ns=args[8],
                ),
            }
            orchestrator = PairMeasurementOrchestrator(dependencies)
            self.node.pair_measurement_orchestrator = orchestrator
        return orchestrator

    def _rviz_live_worker_loop(self):
        return self.node._runtime_worker_loops()._rviz_live_worker_loop()
    def _latest_rviz_ideal_ray(self, pair):
        """Compute a display-only ideal ray from the latest GT cache."""

        position = getattr(self.node, "cached_ground_truth_position", None)
        point_world_enu_m = None
        if isinstance(position, dict):
            point_world_enu_m = position.get("ground_truth_world_enu_m")
        if point_world_enu_m is None:
            return None
        width = int(getattr(pair.left_message, "width", 0))
        height = int(getattr(pair.left_message, "height", 0))
        if width <= 0 or height <= 0:
            return None
        ideal = self.node._triangulate_dkl_ideal_from_reference_world(
            point_world_enu_m,
            width,
            height,
        )
        if ideal is not None:
            return ideal
        self.node._ensure_camera_frames()
        return reference_ray_world_enu_from_point(
            self.node.baseline_center_frd,
            self.node.nominal_left_camera,
            self.node.nominal_right_camera,
            point_world_enu_m,
        )

    def _commit_pair_measurement_if_current(self, pair, computed):
        """Linearization point for reset versus measurement publication.

        The caller holds ``yolo_condition``.  A clock reset uses the same
        lock, so either the complete current-epoch result commits first or the
        reset commits first and this result is discarded without publication.
        """

        self.node.inflight_pair_id = None
        self.node.inflight_pair_epoch = None
        self.node.yolo_inference_in_progress = False
        if self.node.discard_measurement_commits:
            self.node.yolo_condition.notify_all()
            return None
        if pair.epoch != self.node.simulation_epoch.epoch:
            self.node.old_epoch_result_discard_count += 1
            pair_still_pending = (
                not hasattr(self.node, "pairs_by_pair_id")
                or pair.pair_id in self.node.pairs_by_pair_id
            )
            if pair_still_pending:
                self.node.accepted_pair_discard_count += 1
                self.node._submit_research_terminal_locked(
                    pair,
                    terminal_status="old_epoch_result_discarded",
                    terminal_reason="completed_result_from_old_simulation_epoch",
                    measurement_status="discarded",
                    evaluation_status="not_evaluated",
                    metrics=getattr(self.node, "measurement_results_by_pair_id", {}).get(
                        pair.pair_id
                    ),
                    alignment=getattr(self.node, "gt_alignments_by_pair_id", {}).get(
                        pair.pair_id
                    ),
                )
            self.node.get_logger().warning(
                "Discarded completed YOLO result from old simulation epoch: "
                f"pair_id={pair.pair_id}, pair_epoch={pair.epoch}, "
                f"current_epoch={self.node.simulation_epoch.epoch}"
            )
            if pair_still_pending and hasattr(self.node, "pairs_by_pair_id"):
                self.node._drop_pair_state_locked(
                    pair.pair_id, "old_epoch_result_discarded"
                )
            self.node.yolo_condition.notify_all()
            return None
        left_image = computed.get("left_image")
        right_image = computed.get("right_image")
        if left_image is None or right_image is None:
            self.node.accepted_pair_discard_count += 1
            if getattr(self.node, "observation_mode", "bbox_yolo") == "ideal_reference_point":
                self.node.ideal_projection_rejected_count += 1
                self.node.last_ideal_projection_error = computed.get("ideal_error", "invalid_image")
                self.node.last_ideal_projection_pair_id = pair.pair_id
            self.node._submit_research_terminal_locked(
                pair,
                terminal_status="image_decode_failure",
                terminal_reason=computed.get("ideal_error", "image_decode_failure"),
                measurement_status="discarded",
                evaluation_status="not_evaluated",
                alignment=getattr(self.node, "gt_alignments_by_pair_id", {}).get(
                    pair.pair_id
                ),
            )
            self.node._drop_pair_state_locked(pair.pair_id, "image_decode_failure")
            self.node.yolo_condition.notify_all()
            return None
        left = computed.get("left")
        right = computed.get("right")
        if getattr(self.node, "observation_mode", "bbox_yolo") == "ideal_reference_point" and (left is None or right is None):
            self.node.accepted_pair_discard_count += 1
            self.node.ideal_projection_rejected_count += 1
            self.node.last_ideal_projection_error = computed.get("ideal_error", "projection_invalid_or_out_of_fov")
            self.node.last_ideal_projection_pair_id = pair.pair_id
            self.node._submit_research_terminal_locked(
                pair,
                terminal_status="ideal_projection_rejected",
                terminal_reason=self.node.last_ideal_projection_error,
                measurement_status="discarded",
                evaluation_status="not_evaluated",
                alignment=getattr(self.node, "gt_alignments_by_pair_id", {}).get(
                    pair.pair_id
                ),
            )
            self.node._drop_pair_state_locked(pair.pair_id, "ideal_projection_rejected")
            self.node.yolo_condition.notify_all()
            return None
        if getattr(self.node, "observation_mode", "bbox_yolo") == "ideal_reference_point":
            self.node.ideal_projection_success_count += 1
        metrics = self.node._calculate_metrics(
            left_image, left, right, None, computed.get("phase")
        )
        raw = computed.get("raw")
        optical = computed.get("corrected")
        gps_baselines = computed.get("gps_baselines", {})
        uam_reference = self.node._uam_raw_gps_reference()
        uam_gps_ray = (
            uam_gps_ray_visualization_world_enu_from_point(
                self.node.baseline_center_frd,
                uam_reference.get("world_enu_m"),
            )
            if uam_reference.get("computation_status") == "computed"
            else None
        )
        diagnostic_rays = computed.get("diagnostic_rays", {})
        diagnostic_analysis = self.node._diagnostic_analysis(optical, diagnostic_rays)
        for name, comparison in diagnostic_analysis["comparisons"].items():
            if isinstance(diagnostic_rays.get(name), dict):
                diagnostic_rays[name].update(
                    {
                        "delta_valid": comparison["valid"],
                        "delta_failure_reason": comparison["failure_reason"],
                        **{
                            key: value
                            for key, value in comparison.items()
                            if key not in ("valid", "failure_reason")
                        },
                    }
                )
        metrics.update(diagnostic_analysis["flat"])
        for name in ("attitude", "operational"):
            metrics.update(
                self.node._diagnostic_ray_record_fields(name, diagnostic_rays.get(name))
            )
        metrics["_diagnostic_rays"] = diagnostic_rays
        metrics["_truth_ray"] = optical
        metrics["_raw_ray"] = raw
        metrics["_gps_baselines"] = gps_baselines
        if raw is not None:
            metrics.update(
                {
                    "computation_status": raw.get("computation_status"),
                    "estimated_forward_m": raw.get("forward_m"),
                    "estimated_right_m": raw.get("right_m"),
                    "estimated_down_m": raw.get("down_m"),
                    "estimated_range_m": raw.get("range_m"),
                    "ray_gap_m": raw.get("ray_gap_m"),
                    "intersection_angle_deg": raw.get("intersection_angle_deg"),
                }
            )
            self.node._publish_ray_metrics(
                pair_id=pair.pair_id,
                nominal_pose=raw,
                optical_pose=optical,
                gps_baseline=gps_baselines.get("gps_baseline"),
                global_baseline=gps_baselines.get("global_baseline"),
                **diagnostic_rays,
            )
        if not self.node._rviz_latest_only_markers_enabled():
            self.node._publish_ray_markers(
                nominal_pose=raw,
                optical_pose=optical,
                gps_baseline=gps_baselines.get("gps_baseline"),
                global_baseline=gps_baselines.get("global_baseline"),
                uam_gps_ray=uam_gps_ray,
                **diagnostic_rays,
            )
        metrics.update(
            {
                "optical_computation_status": (
                    "invalid_input" if optical is None else optical.get("computation_status")
                ),
                "optical_forward_m": None if optical is None else optical.get("forward_m"),
                "optical_right_m": None if optical is None else optical.get("right_m"),
                "optical_down_m": None if optical is None else optical.get("down_m"),
                "optical_range_m": None if optical is None else optical.get("range_m"),
                "optical_ray_gap_m": None if optical is None else optical.get("ray_gap_m"),
                "optical_intersection_angle_deg": (
                    None if optical is None else optical.get("intersection_angle_deg")
                ),
            }
        )
        for source, gps_baseline in gps_baselines.items():
            prefix = f"{source}_baseline"
            metrics.update(
                {
                    f"{prefix}_computation_status": gps_baseline.get("computation_status"),
                    f"{prefix}_forward_m": gps_baseline.get("forward_m"),
                    f"{prefix}_right_m": gps_baseline.get("right_m"),
                    f"{prefix}_down_m": gps_baseline.get("down_m"),
                    f"{prefix}_range_m": gps_baseline.get("range_m"),
                    f"{prefix}_ray_gap_m": gps_baseline.get("ray_gap_m"),
                    f"{prefix}_intersection_angle_deg": gps_baseline.get("intersection_angle_deg"),
                    f"{prefix}_world_enu_m": gps_baseline.get("gps_baseline_world_enu_m"),
                    f"{prefix}_length_m": gps_baseline.get("gps_baseline_length_m"),
                    f"{prefix}_delta_world_enu_m": gps_baseline.get("gps_baseline_delta_world_enu_m"),
                    f"{prefix}_delta_norm_m": gps_baseline.get("gps_baseline_delta_norm_m"),
                    f"{prefix}_left_eph_m": gps_baseline.get("gps_left_eph_m"),
                    f"{prefix}_right_eph_m": gps_baseline.get("gps_right_eph_m"),
                }
            )
        # Preserve the existing single-source topic and metric contract.  In
        # Both mode it carries raw GPS, while the explicit source topics below
        # carry both results for the operator GUI.
        gps_baseline = gps_baselines.get("gps_baseline") or gps_baselines.get("global_baseline")
        if gps_baseline is not None:
            metrics.update(
                {
                    "gps_baseline_computation_status": gps_baseline.get("computation_status"),
                    "gps_baseline_forward_m": gps_baseline.get("forward_m"),
                    "gps_baseline_right_m": gps_baseline.get("right_m"),
                    "gps_baseline_down_m": gps_baseline.get("down_m"),
                }
            )
        metrics.update(
            {
                "uam_raw_gps_reference_computation_status": uam_reference.get(
                    "computation_status"
                ),
                "uam_raw_gps_reference_forward_m": uam_reference.get("forward_m"),
                "uam_raw_gps_reference_right_m": uam_reference.get("right_m"),
                "uam_raw_gps_reference_down_m": uam_reference.get("down_m"),
                "uam_raw_gps_reference_range_m": uam_reference.get("range_m"),
                "uam_raw_gps_reference_world_enu_m": uam_reference.get("world_enu_m"),
                "uam_raw_gps_reference_left_timestamp_us": uam_reference.get("left_timestamp_us"),
                "uam_raw_gps_reference_timestamp_us": uam_reference.get("uam_timestamp_us"),
                "uam_raw_gps_reference_left_age_ms": uam_reference.get("left_age_ms"),
                "uam_raw_gps_reference_uam_age_ms": uam_reference.get("uam_age_ms"),
                "uam_raw_gps_reference_pair_skew_ms": uam_reference.get("pair_skew_ms"),
                "uam_raw_gps_reference_left_eph_m": uam_reference.get("left_eph_m"),
                "uam_raw_gps_reference_uam_eph_m": uam_reference.get("uam_eph_m"),
            }
        )
        metrics.update(
            self.node._pair_result_metadata(
                pair, None, None, computed.get("corrected"), count_alignment=False
            )
        )
        metrics["camera_oracle_snapshot"] = computed.get(
            "camera_oracle_snapshot", self.camera_oracle_unset
        )
        metrics.update(self.node._observation_metadata())
        metrics.update(self.node._observation_record_fields(left, right))
        self.node.last_pair_result = dict(metrics)
        self.node.left_detection = left
        self.node.right_detection = right
        self.node.metrics = metrics
        if getattr(self.node, "observation_mode", "bbox_yolo") == "bbox_yolo":
            detection_message = self.node._detections_to_msg(left, right)
            # ``/stereo/detections`` is the ordered measurement stream for
            # recording, while rqt overlays consume the display-only latest
            # stream. Publish both from the completed measurement so a normal
            # worker run shows the same bbox that produced its Nominal ray.
            self.node.detections_pub.publish(detection_message)
            self.node.live_detections_pub.publish(detection_message)
        disparity = metrics.get("disparity_px")
        distance = metrics.get("distance_m")
        error = metrics.get("distance_error_m")
        if distance is not None:
            self.node.distance_pub.publish(self.float32_type(data=float(distance)))
        if error is not None:
            self.node.error_pub.publish(self.float32_type(data=float(error)))
        if disparity is not None:
            self.node.disparity_pub.publish(self.float32_type(data=float(disparity)))
        self.node._publish_estimated_position(metrics)
        self.node._publish_canonical_pair_positions(metrics)
        finished = computed.get("finished", time.monotonic())
        elapsed_ms = (finished - computed.get("started", finished)) * 1000.0
        self.node.yolo_inference_ms_pub.publish(self.float32_type(data=float(elapsed_ms)))
        stage_timings_ms = computed.get("stage_timings_ms", {})
        for publisher, key in (
            (getattr(self.node, "image_decode_ms_pub", None), "image_decode_ms"),
            (getattr(self.node, "yolo_predict_ms_pub", None), "yolo_predict_ms"),
            (getattr(self.node, "ray_geometry_ms_pub", None), "ray_geometry_ms"),
        ):
            if publisher is not None:
                publisher.publish(
                    self.float32_type(data=float(stage_timings_ms.get(key, 0.0)))
                )
        if self.node.last_yolo_completion is not None:
            period_s = finished - self.node.last_yolo_completion
            if period_s > 0.0:
                self.node.yolo_update_hz_pub.publish(self.float32_type(data=float(1.0 / period_s)))
        self.node.last_yolo_completion = finished
        self.node._complete_pair_measurement_locked(pair.pair_id, metrics)
        self.node._evaluate_pair_with_fault_containment_locked(
            pair.pair_id,
            trigger="measurement_complete",
        )
        self.node.yolo_condition.notify_all()
        return metrics

    def _complete_pair_measurement_locked(self, pair_id, metrics):
        """Record measurement success independently of Ground Truth evaluation."""

        registry = self.node._ensure_pair_evaluation_registry_locked()
        metrics["measurement_status"] = "processed"
        metrics.setdefault("evaluation_status", "pending")
        registry.store_measurement(pair_id, metrics)
        self.node.measurement_processed_count += 1

    def _observation_metadata(self):
        return observation_metadata(getattr(self.node, "observation_mode", "bbox_yolo"))

    def _observation_record_fields(self, left, right):
        """Return immutable per-pair observation provenance for S2 recording."""

        return observation_record_fields(
            getattr(self.node, "observation_mode", "bbox_yolo"), left, right
        )

    def _update_metrics_from_messages(
        self,
        left_msg,
        right_msg,
        actual,
        phase,
        device=None,
        *,
        pair=None,
        gt_alignment=None,
        gt_baseline=None,
    ):
        if pair is None:
            left_image = self.node._msg_to_rgb(left_msg)
            right_image = self.node._msg_to_rgb(right_msg)
            if left_image is None or right_image is None:
                return None
            if device is None:
                device = self.node.resolved_yolo_device or self.node._resolve_yolo_device()
            left, right = self.node._detect_drone_pair(left_image, right_image, device=device)
            metrics = self.node._calculate_metrics(left_image, left, right, actual, phase)
            corrected = None
            if left is not None and right is not None:
                corrected = self.node._triangulate_pixel_centers_oracle(
                    left["cx"], left["cy"], right["cx"], right["cy"],
                    left_image.shape[1], left_image.shape[0],
                )
            metrics.update(
                self.node._pair_result_metadata(
                    pair, gt_alignment, gt_baseline, corrected,
                    count_alignment=gt_alignment is not None,
                )
            )
            return metrics
        computed = self.node._compute_pair_measurement(pair, phase, device=device)
        with self.node.yolo_condition:
            return self.node._commit_pair_measurement_if_current(pair, computed)

    def _ground_truth_alignment_to_baseline(self, alignment):
        if alignment is None or alignment.position_world_enu_m is None:
            return None
        try:
            return ground_truth_world_to_baseline(
                alignment.position_world_enu_m,
                self.node.baseline_center_frd,
            )
        except (TypeError, ValueError):
            return None

    def _store_gt_alignment_locked(self, pair, alignment):
        registry = self.node._ensure_pair_evaluation_registry_locked()
        if pair.epoch != self.node.simulation_epoch.epoch:
            self.node.old_epoch_frame_count += 1
            return
        # A legitimate alignment is created only for a pair registered in the
        # active pair map.  Do not resurrect terminal state after its bounded
        # evaluated/failed tombstone has eventually been evicted.
        if pair.pair_id not in self.node.pairs_by_pair_id:
            return
        if registry.is_terminal(pair.pair_id):
            return
        if not registry.store_alignment(pair.pair_id, alignment):
            return
        if getattr(self.node, "observation_mode", "bbox_yolo") == "ideal_reference_point":
            if pair.pair_id not in {item.pair_id for item in self.node.measurement_pair_queue}:
                if len(self.node.measurement_pair_queue) >= self.node.measurement_queue_capacity:
                    self.node._finalize_measurement_queue_overflow_locked(pair.pair_id)
                    return
                self.node.measurement_pair_queue.append(pair)
                self.node._record_measurement_queue_depth_locked()
        self.node._evaluate_pair_with_fault_containment_locked(
            pair.pair_id,
            trigger="late_gt_alignment",
        )

    def _evaluate_pair_with_fault_containment_locked(self, pair_id, *, trigger):
        """Attempt one GT evaluation without invalidating measurement success."""

        self.node._ensure_gt_evaluation_state_locked()
        try:
            return self.node._try_evaluate_pair_locked(pair_id)
        except Exception as exc:
            self.node._finalize_gt_evaluation_failure_locked(
                pair_id,
                exc,
                trigger=trigger,
            )
            return False

    def _finalize_gt_evaluation_failure_locked(self, pair_id, exc, *, trigger):
        registry = self.node._ensure_pair_evaluation_registry_locked()
        pair_id = int(pair_id)
        if (
            pair_id in self.node.evaluated_pair_ids
            or pair_id in self.node.gt_evaluation_failed_pair_ids
        ):
            return False
        self.node.gt_evaluation_error_count += 1
        self.node.last_gt_evaluation_error_type = type(exc).__name__
        self.node.last_gt_evaluation_error_message = str(exc)
        self.node.last_gt_evaluation_error_trigger = str(trigger)
        pair = self.node.pairs_by_pair_id.get(pair_id)
        metrics = self.node.measurement_results_by_pair_id.get(pair_id)
        if metrics is not None:
            metrics["evaluation_status"] = "evaluation_error"
            self.node.last_pair_result = dict(metrics)
            measurement_status = "processed"
        else:
            self.node.accepted_pair_discard_count += 1
            measurement_status = "discarded"
        self.node._submit_research_terminal_locked(
            pair,
            terminal_status="evaluation_error",
            terminal_reason=f"{trigger}:{type(exc).__name__}: {exc}",
            measurement_status=measurement_status,
            evaluation_status="evaluation_error",
            metrics=metrics,
            alignment=self.node.gt_alignments_by_pair_id.get(pair_id),
        )
        registry.finalize(pair_id, failed=True)
        self.node.yolo_condition.notify_all()
        try:
            self.node.get_logger().error(
                "Ground Truth evaluation failed for "
                f"pair_id={pair_id}, trigger={trigger}: "
                f"{type(exc).__name__}: {exc}"
            )
        except Exception:
            pass
        return True

    def _ensure_gt_evaluation_state_locked(self):
        if not hasattr(self.node, "gt_evaluation_failed_pair_ids"):
            capacity = int(getattr(self.node, "max_pair_result_history", 128))
            self.node.gt_evaluation_failed_pair_ids = BoundedIdHistory(capacity)
        if not hasattr(self.node, "gt_evaluation_error_count"):
            self.node.gt_evaluation_error_count = 0
        if not hasattr(self.node, "gt_evaluation_success_count"):
            self.node.gt_evaluation_success_count = 0
        if not hasattr(self.node, "last_gt_evaluation_error_type"):
            self.node.last_gt_evaluation_error_type = ""
            self.node.last_gt_evaluation_error_message = ""
            self.node.last_gt_evaluation_error_trigger = ""

    def _ensure_pair_evaluation_registry_locked(self):
        """Return the pair registry, adopting legacy partial-test state."""

        self.node._ensure_gt_evaluation_state_locked()
        registry = getattr(self.node, "pair_evaluation_registry", None)
        if registry is None:
            registry = PairEvaluationRegistry(
                int(getattr(self.node, "max_pair_result_history", 128))
            )
            registry.pairs = getattr(self.node, "pairs_by_pair_id", {})
            registry.measurements = getattr(self.node, "measurement_results_by_pair_id", {})
            registry.alignments = getattr(self.node, "gt_alignments_by_pair_id", {})
            registry.evaluated_ids = getattr(
                self.node, "evaluated_pair_ids", registry.evaluated_ids
            )
            registry.failed_ids = getattr(
                self.node, "gt_evaluation_failed_pair_ids", registry.failed_ids
            )
            registry.published_gt_ids = getattr(
                self.node, "gt_published_pair_ids", registry.published_gt_ids
            )
            self.node.pair_evaluation_registry = registry
        self.node.pairs_by_pair_id = registry.pairs
        self.node.measurement_results_by_pair_id = registry.measurements
        self.node.gt_alignments_by_pair_id = registry.alignments
        self.node.evaluated_pair_ids = registry.evaluated_ids
        self.node.gt_evaluation_failed_pair_ids = registry.failed_ids
        self.node.gt_published_pair_ids = registry.published_gt_ids
        return registry

    def _try_evaluate_pair_locked(self, pair_id):
        registry = self.node._ensure_pair_evaluation_registry_locked()
        ready = registry.ready(pair_id)
        if ready is None:
            return False
        pair, metrics, alignment = ready
        gt_baseline = self.node._ground_truth_alignment_to_baseline(alignment)
        ideal = None
        if alignment.position_world_enu_m is not None:
            width = int(getattr(pair.left_message, "width", 0))
            height = int(getattr(pair.left_message, "height", 0))
            if (
                width > 0
                and height > 0
                and getattr(self.node, "horizontal_fov_rad", None) is not None
            ):
                ideal = self.node._triangulate_dkl_ideal_from_reference_world(
                    alignment.position_world_enu_m, width, height
                )
        metrics.update(
            {
                "ideal_computation_status": (
                    "reference_unavailable"
                    if ideal is None
                    else ideal.get("computation_status")
                ),
                "ideal_forward_m": None if ideal is None else ideal.get("forward_m"),
                "ideal_right_m": None if ideal is None else ideal.get("right_m"),
                "ideal_down_m": None if ideal is None else ideal.get("down_m"),
                "ideal_range_m": None if ideal is None else ideal.get("range_m"),
                "ideal_ray_gap_m": None if ideal is None else ideal.get("ray_gap_m"),
                "ideal_intersection_angle_deg": (
                    None if ideal is None else ideal.get("intersection_angle_deg")
                ),
                "ideal_left_u_px": None if ideal is None else ideal.get("left_u_px"),
                "ideal_left_v_px": None if ideal is None else ideal.get("left_v_px"),
                "ideal_right_u_px": None if ideal is None else ideal.get("right_u_px"),
                "ideal_right_v_px": None if ideal is None else ideal.get("right_v_px"),
                "ideal_disparity_px": (
                    None
                    if ideal is None
                    or ideal.get("left_u_px") is None
                    or ideal.get("right_u_px") is None
                    else float(ideal["left_u_px"]) - float(ideal["right_u_px"])
                ),
                "ideal_pixel_source": "ground_truth_world_enu_quantized_nominal_projection",
            }
        )
        evaluation = self.node._evaluation_metadata(pair, alignment, gt_baseline, metrics)
        metrics.update(evaluation)
        diagnostic_rays = metrics.get("_diagnostic_rays", {})
        for name in ("attitude", "operational"):
            result = diagnostic_rays.get(name)
            if isinstance(result, dict):
                result.update(
                    {
                        "gt_valid": evaluation.get(f"{name}_gt_valid"),
                        "gt_failure_reason": evaluation.get(
                            f"{name}_gt_failure_reason"
                        ),
                        "gt_forward_error_m": evaluation.get(
                            f"{name}_gt_forward_error_m"
                        ),
                        "gt_right_error_m": evaluation.get(
                            f"{name}_gt_right_error_m"
                        ),
                        "gt_down_error_m": evaluation.get(
                            f"{name}_gt_down_error_m"
                        ),
                        "gt_range_error_m": evaluation.get(
                            f"{name}_gt_range_error_m"
                        ),
                        "gt_position_error_m": evaluation.get(
                            f"{name}_gt_position_error_m"
                        ),
                    }
                )
        # attitude/operational reuse the evaluation_metadata()-computed gt_*
        # fields above (also relied on by CSV/XLSX recording); the remaining
        # Rays have no such flat-prefix precedent, so compute their
        # Ground-Truth error directly from each ray's own FRD vector.
        for name, ray in (
            ("nominal_pose", metrics.get("_raw_ray")),
            ("optical_pose", metrics.get("_truth_ray")),
            ("ideal", ideal),
            ("gps_baseline", metrics.get("_gps_baselines", {}).get("gps_baseline")),
            ("global_baseline", metrics.get("_gps_baselines", {}).get("global_baseline")),
            ("attitude_true_north", diagnostic_rays.get("attitude_true_north")),
        ):
            if isinstance(ray, dict):
                ray.update(ray_gt_error(ray, gt_baseline))
        self.node._publish_ray_metrics(
            pair_id=pair_id,
            nominal_pose=metrics.get("_raw_ray"),
            optical_pose=metrics.get("_truth_ray"),
            ideal=ideal,
            gps_baseline=metrics.get("_gps_baselines", {}).get("gps_baseline"),
            global_baseline=metrics.get("_gps_baselines", {}).get("global_baseline"),
            gt_source=evaluation.get("gt_source"),
            gt_fallback_used=evaluation.get("gt_fallback_used"),
            **diagnostic_rays,
        )
        # Ideal metrics intentionally remain unavailable when nominal pixels
        # are outside the image.  For RViz, however, retain a display-only
        # reference ray to the aligned Ground Truth point in that case.
        ideal_marker = ideal
        if (
            ideal_marker is None
            and alignment.position_world_enu_m is not None
            and hasattr(self.node, "nominal_left_camera")
            and hasattr(self.node, "nominal_right_camera")
        ):
            self.node._ensure_camera_frames()
            ideal_marker = reference_ray_world_enu_from_point(
                self.node.baseline_center_frd,
                self.node.nominal_left_camera,
                self.node.nominal_right_camera,
                alignment.position_world_enu_m,
            )
        if ideal_marker is not None:
            if not self.node._rviz_latest_only_markers_enabled():
                self.node._publish_ray_markers(ideal=ideal_marker)
        # The initial publish happens before GT alignment exists.  Republish
        # after the DKL reference has been evaluated so rqt receives it without
        # waiting for the periodic plot sampler.
        if getattr(self.node, "nominal_pose_plot_pub", None) is not None:
            self.node._publish_estimated_position(metrics)
        unavailable = alignment.gt_alignment_mode == "unavailable"
        if gt_baseline is not None and pair_id not in self.node.gt_published_pair_ids:
            self.node.canonical_ground_truth_position_pub.publish(
                self.node._point_stamped(
                    pair.pair_midpoint_stamp_ns,
                    gt_baseline["forward_m"],
                    gt_baseline["right_m"],
                    gt_baseline["down_m"],
                )
            )
            self.node.gt_published_pair_ids.add(pair_id)
        metrics["evaluation_status"] = (
            "unavailable" if unavailable else "evaluated"
        )
        self.node.last_pair_result = dict(metrics)
        self.node.gt_alignment_counts[alignment.gt_alignment_mode] += 1
        if not unavailable:
            self.node.gt_evaluation_success_count += 1
        self.node._submit_research_terminal_locked(
            pair,
            terminal_status="gt_unavailable" if unavailable else "evaluated",
            terminal_reason="gt_unavailable" if unavailable else "evaluated",
            measurement_status="processed",
            evaluation_status="unavailable" if unavailable else "evaluated",
            metrics=metrics,
            alignment=alignment,
        )
        registry.finalize(pair_id)
        return True

    @staticmethod
    def _vector_error(metrics, prefix, gt_baseline):
        return vector_error(metrics, prefix, gt_baseline)

    def _evaluation_metadata(self, pair, alignment, gt_baseline, metrics):
        return evaluation_metadata(pair, alignment, gt_baseline, metrics)

    def _pair_result_metadata(
        self,
        pair,
        alignment,
        gt_baseline,
        corrected,
        *,
        count_alignment=True,
    ):
        self.node._ensure_attitude_error_state()
        alignment_mode = "unavailable" if alignment is None else alignment.gt_alignment_mode
        if count_alignment and hasattr(self.node, "gt_alignment_counts"):
            self.node.gt_alignment_counts[alignment_mode] += 1
        metadata = {
            "target_model_name": getattr(self.node, "target_model_name", "unspecified"),
            "pair_id": None if pair is None else pair.pair_id,
            "epoch": None if pair is None else pair.epoch,
            "left_stamp_ns": None if pair is None else pair.left_stamp_ns,
            "right_stamp_ns": None if pair is None else pair.right_stamp_ns,
            "pair_midpoint_stamp_ns": None if pair is None else pair.pair_midpoint_stamp_ns,
            "pair_delta_ns": None if pair is None else pair.pair_delta_ns,
            "pairing_mode": None if pair is None else pair.pairing_mode,
            "gt_alignment_mode": alignment_mode,
            "gt_before_stamp_ns": None if alignment is None else alignment.gt_before_stamp_ns,
            "gt_after_stamp_ns": None if alignment is None else alignment.gt_after_stamp_ns,
            "gt_interpolation_alpha": None if alignment is None else alignment.gt_interpolation_alpha,
            "gt_nearest_delta_ns": None if alignment is None else alignment.gt_nearest_delta_ns,
            "gt_forward_m": None if gt_baseline is None else gt_baseline["forward_m"],
            "gt_right_m": None if gt_baseline is None else gt_baseline["right_m"],
            "gt_down_m": None if gt_baseline is None else gt_baseline["down_m"],
            "gt_range_m": None if gt_baseline is None else gt_baseline["range_m"],
            "cam_l_optical_roll_deg": self.node.cam_l_optical_roll_deg,
            "cam_l_optical_pitch_deg": self.node.cam_l_optical_pitch_deg,
            "cam_l_optical_yaw_deg": self.node.cam_l_optical_yaw_deg,
            "cam_r_optical_roll_deg": self.node.cam_r_optical_roll_deg,
            "cam_r_optical_pitch_deg": self.node.cam_r_optical_pitch_deg,
            "cam_r_optical_yaw_deg": self.node.cam_r_optical_yaw_deg,
            "cam_l_px4_attitude_roll_deg": self.node.cam_l_px4_attitude_roll_deg,
            "cam_l_px4_attitude_pitch_deg": self.node.cam_l_px4_attitude_pitch_deg,
            "cam_l_px4_attitude_yaw_deg": self.node.cam_l_px4_attitude_yaw_deg,
            "cam_r_px4_attitude_roll_deg": self.node.cam_r_px4_attitude_roll_deg,
            "cam_r_px4_attitude_pitch_deg": self.node.cam_r_px4_attitude_pitch_deg,
            "cam_r_px4_attitude_yaw_deg": self.node.cam_r_px4_attitude_yaw_deg,
            "px4_attitude_model_version": PX4_ATTITUDE_MODEL_VERSION,
            "px4_attitude_reference_source": PX4_ATTITUDE_REFERENCE_SOURCE,
            "px4_attitude_frame": PX4_ATTITUDE_FRAME,
            "px4_attitude_composition": PX4_ATTITUDE_COMPOSITION,
            "px4_attitude_quaternion_order": "wxyz",
            "legacy_px4_absolute_attitude_used": True,
            "common_optical_roll_error_deg": self.node.common_optical_roll_error_deg,
            "relative_optical_roll_error_deg": self.node.relative_optical_roll_error_deg,
            "common_optical_pitch_error_deg": self.node.common_optical_pitch_error_deg,
            "relative_optical_pitch_error_deg": self.node.relative_optical_pitch_error_deg,
            "common_optical_yaw_error_deg": self.node.common_optical_yaw_error_deg,
            "relative_optical_yaw_error_deg": self.node.relative_optical_yaw_error_deg,
            "effective_left_optical_roll_error_deg": self.node.effective_left_optical_roll_error_deg,
            "effective_left_optical_pitch_error_deg": self.node.effective_left_optical_pitch_error_deg,
            "effective_left_optical_yaw_error_deg": self.node.effective_left_optical_yaw_error_deg,
            "effective_right_optical_roll_error_deg": self.node.effective_right_optical_roll_error_deg,
            "effective_right_optical_pitch_error_deg": self.node.effective_right_optical_pitch_error_deg,
            "effective_right_optical_yaw_error_deg": self.node.effective_right_optical_yaw_error_deg,
            "attitude_error_frame": self.node.attitude_error_frame,
            "attitude_angle_unit": self.node.attitude_angle_unit,
            "attitude_parameterization": self.node.attitude_parameterization,
            "attitude_rotation_composition": self.node.attitude_rotation_composition,
            "correction_source": "gazebo_observed_optical_pose",
            "attitude_corrected_computation_status": (
                "invalid_input" if corrected is None else corrected.get("computation_status")
            ),
            "attitude_corrected_forward_m": None if corrected is None else corrected.get("forward_m"),
            "attitude_corrected_right_m": None if corrected is None else corrected.get("right_m"),
            "attitude_corrected_down_m": None if corrected is None else corrected.get("down_m"),
            "attitude_corrected_range_m": None if corrected is None else corrected.get("range_m"),
            "attitude_corrected_ray_gap_m": None if corrected is None else corrected.get("ray_gap_m"),
            "attitude_corrected_intersection_angle_deg": (
                None if corrected is None else corrected.get("intersection_angle_deg")
            ),
            "attitude_corrected_left_depth_m": None if corrected is None else corrected.get("left_depth_m"),
            "attitude_corrected_right_depth_m": None if corrected is None else corrected.get("right_depth_m"),
            "attitude_corrected_geometry_rank": None if corrected is None else corrected.get("geometry_rank"),
            "attitude_corrected_condition_number": None if corrected is None else corrected.get("condition_number"),
            "output_frame": BASELINE_CENTER_FRD_FRAME,
        }
        if pair is not None and pair.pair_id % 100 == 0:
            self.node.get_logger().info(f"Simulation-time alignment counters: {self.node._time_alignment_counters()}")
        return metadata

    def _time_alignment_counters(self):
        pairer = self.node.image_pairer
        return {
            "left_received_count": pairer.left_received_count,
            "right_received_count": pairer.right_received_count,
            "paired_count": pairer.paired_count,
            "exact_pair_count": pairer.exact_pair_count,
            "nearest_pair_count": pairer.nearest_pair_count,
            "unmatched_left_count": pairer.unmatched_left_count,
            "unmatched_right_count": pairer.unmatched_right_count,
            "image_reset_count": pairer.image_reset_count,
            "gt_reset_count": self.node.gt_position_buffer.reset_count,
            "simulation_reset_count": self.node.simulation_epoch.reset_count,
            "simulation_epoch": self.node.simulation_epoch.epoch,
            "stale_pre_reset_count": self.node.stale_pre_reset_count,
            "stale_pre_reset_gt_count": self.node.stale_pre_reset_gt_count,
            "gt_quarantine_active": self.node.gt_quarantine_active,
            "gt_quarantine_rejected_count": self.node.gt_quarantine_rejected_count,
            "gt_quarantine_release_count": self.node.gt_quarantine_release_count,
            "future_gt_pending_count": len(self.node.future_gt_pending),
            "future_gt_overflow_count": self.node.future_gt_overflow_count,
            "measurement_queue_capacity": self.node.measurement_queue_capacity,
            "measurement_queue_depth": len(self.node.measurement_pair_queue),
            "measurement_queue_depth_max": self.node.measurement_queue_depth_max,
            "measurement_queue_overflow_count": self.node.measurement_queue_overflow_count,
            "measurement_overrun_invalid": self.node.measurement_overrun_invalid,
            "measurement_rejected_after_overrun_count": (
                self.node.measurement_rejected_after_overrun_count
            ),
            "old_epoch_result_discard_count": self.node.old_epoch_result_discard_count,
            "accepted_pair_discard_count": self.node.accepted_pair_discard_count,
            "measurement_processing_error_count": self.node.measurement_processing_error_count,
            "gt_evaluation_success_count": self.node.gt_evaluation_success_count,
            "gt_evaluation_error_count": self.node.gt_evaluation_error_count,
            "gt_evaluation_failed_pair_count": len(
                self.node.gt_evaluation_failed_pair_ids
            ),
            "last_gt_evaluation_error_type": self.node.last_gt_evaluation_error_type,
            "last_gt_evaluation_error_message": self.node.last_gt_evaluation_error_message,
            "last_gt_evaluation_error_trigger": self.node.last_gt_evaluation_error_trigger,
            "input_rejected_after_shutdown_count": self.node.input_rejected_after_shutdown_count,
            "last_measurement_error_type": self.node.last_measurement_error_type,
            "last_measurement_error_message": self.node.last_measurement_error_message,
            "invalid_timestamp_count": pairer.invalid_timestamp_count,
            "inconsistent_clock_count": self.node.inconsistent_clock_count
            + self.node.gt_position_buffer.inconsistent_clock_count,
            "gt_exact_count": self.node.gt_alignment_counts["exact"],
            "gt_interpolated_count": self.node.gt_alignment_counts["interpolated"],
            "gt_nearest_before_count": self.node.gt_alignment_counts["nearest_before"],
            "gt_nearest_after_count": self.node.gt_alignment_counts["nearest_after"],
            "gt_unavailable_count": self.node.gt_alignment_counts["unavailable"],
            "raw_computed_count": self.node.raw_computed_count,
            "corrected_computed_count": self.node.corrected_computed_count,
            "observation_mode": self.node.observation_mode,
            "ideal_projection_success_count": self.node.ideal_projection_success_count,
            "ideal_projection_rejected_count": self.node.ideal_projection_rejected_count,
            "last_ideal_projection_error": self.node.last_ideal_projection_error,
            "last_ideal_projection_pair_id": self.node.last_ideal_projection_pair_id,
            "research_recording_enabled": self.node._research_recording_enabled(),
            "research_terminal_record_count": getattr(
                self.node, "research_terminal_record_count", 0
            ),
            "research_record_queue_depth": (
                0 if not self.node._research_recording_enabled()
                else self.node.research_recorder.queue_depth
            ),
            "research_record_queue_depth_max": (
                0 if not self.node._research_recording_enabled()
                else self.node.research_recorder.queue_depth_max
            ),
            "research_record_queue_capacity": getattr(
                self.node, "research_record_queue_capacity", 0
            ),
            "research_recording_overrun_invalid": getattr(
                self.node, "research_recording_overrun_invalid", False
            ),
            "research_recording_error_count": getattr(
                self.node, "research_recording_error_count", 0
            ),
        }
