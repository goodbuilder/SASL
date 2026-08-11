"""Measurement and latest-only RViz worker loop scheduling.

This owner controls condition waits, work ordering, target-rate pacing, drain
termination, and per-iteration fault containment. Pair measurement math and
coordinate conversion remain delegated to their dedicated pipelines.
"""

import time

from std_msgs.msg import Float32

from stereo_ground_sim.rviz_geometry import (
    uam_gps_ray_visualization_world_enu_from_point,
)


class RuntimeWorkerLoops:
    def __init__(self, node):
        self.node = node

    def _yolo_worker_loop(self):
        if getattr(self.node, "observation_mode", "bbox_yolo") == "bbox_yolo":
            self.node._warm_yolo_model()
        next_tick = time.monotonic()
        while True:
            pair = None
            alignment_work = None
            with self.node.yolo_condition:
                scheduler = self.node._ensure_pair_work_scheduler_locked()
                while not self.node.measurement_pair_queue and not self.node.gt_alignment_ready_queue:
                    if self.node.drain_requested:
                        self.node.drain_complete = True
                        self.node.yolo_condition.notify_all()
                        return
                    if self.node.shutdown_event.is_set():
                        return
                    self.node.yolo_condition.wait(timeout=0.1)
                if self.node.measurement_pair_queue and not (
                    scheduler.prefer_alignment_next and self.node.gt_alignment_ready_queue
                ):
                    target_hz = self.node._measurement_processing_hz()
                    wait_s = next_tick - time.monotonic()
                    if wait_s > 0.0 and not self.node.drain_requested:
                        self.node.yolo_condition.wait(timeout=wait_s)
                        continue
                work_kind, work = scheduler.pop_next()
                self.node._prefer_gt_alignment_next = scheduler.prefer_alignment_next
                if work_kind == "measurement":
                    pair = work
                elif work_kind == "alignment":
                    alignment_work = work
                if alignment_work is not None:
                    pair, alignment = alignment_work
                    try:
                        alignment_started = time.monotonic()
                        self.node._store_gt_alignment_locked(pair, alignment)
                        publisher = getattr(self.node, "gt_alignment_ms_pub", None)
                        if publisher is not None:
                            publisher.publish(
                                Float32(
                                    data=float(
                                        (time.monotonic() - alignment_started) * 1000.0
                                    )
                                )
                            )
                    except Exception as exc:
                        self.node._finalize_gt_evaluation_failure_locked(
                            pair.pair_id,
                            exc,
                            trigger="late_gt_alignment",
                        )
                    continue
                if pair is None:
                    continue
                phase = self.node._recording_mission_phase()
                self.node.yolo_inference_in_progress = True
                self.node.inflight_pair_id = pair.pair_id
                self.node.inflight_pair_epoch = pair.epoch
                self.node.last_yolo_update = time.monotonic()
                next_tick = self.node.last_yolo_update + 1.0 / target_hz
            failure = None
            try:
                computed = self.node._compute_pair_measurement(pair, phase)
                with self.node.yolo_condition:
                    commit_started = time.monotonic()
                    metrics = self.node._commit_pair_measurement_if_current(pair, computed)
                    publisher = getattr(self.node, "result_commit_ms_pub", None)
                    if publisher is not None:
                        publisher.publish(
                            Float32(
                                data=float(
                                    (time.monotonic() - commit_started) * 1000.0
                                )
                            )
                        )
                    if (
                        metrics is not None
                        and getattr(self.node, "enable_rviz_rays", False)
                        and getattr(self.node, "rviz_live_latest_only", False)
                    ):
                        # A completed pair is the single source for both the
                        # ordered measurement stream and latest-only RViz.
                        # Keep only the newest completed result so the display
                        # worker never performs a second image decode or YOLO
                        # inference for the same stereo observation.
                        self.node.rviz_live_latest_result = (pair, computed)
                        self.node.yolo_condition.notify_all()
            except Exception as exc:
                failure = exc
                with self.node.yolo_condition:
                    self.node._finalize_pair_failure_locked(pair, exc)
            if failure is not None:
                try:
                    self.node.get_logger().error(
                        "Measurement processing failed for "
                        f"pair_id={pair.pair_id}: "
                        f"{type(failure).__name__}: {failure}"
                    )
                except Exception:
                    pass


    def _rviz_live_worker_loop(self):
        """Publish the newest completed measurement in RViz without re-running YOLO."""

        if getattr(self.node, "observation_mode", "bbox_yolo") != "bbox_yolo":
            return
        self.node._warm_yolo_model()
        next_tick = time.monotonic()
        while not self.node.shutdown_event.is_set():
            pair = None
            with self.node.yolo_condition:
                while not self.node.shutdown_event.is_set():
                    latest = getattr(self.node, "rviz_live_latest_result", None)
                    if (
                        latest is not None
                        and latest[0].pair_id
                        > getattr(self.node, "rviz_live_latest_processed_pair_id", 0)
                    ):
                        break
                    self.node.yolo_condition.wait(timeout=0.1)
                if self.node.shutdown_event.is_set():
                    return
                wait_s = next_tick - time.monotonic()
                if wait_s > 0.0:
                    self.node.yolo_condition.wait(timeout=wait_s)
                    continue
                pair, computed = self.node.rviz_live_latest_result
                self.node.rviz_live_latest_processed_pair_id = pair.pair_id
                next_tick = time.monotonic() + 1.0 / max(
                    0.1, float(getattr(self.node, "rviz_live_ray_hz", 15.0))
                )
            try:
                raw = computed.get("raw")
                # The display-only Ideal Ray is independent of YOLO.  Keep
                # computing it when the vehicle is outside the image and no
                # raw bbox triangulation is available; _latest_rviz_ideal_ray
                # falls back to the cached World-ENU reference point.
                ideal = self.node._latest_rviz_ideal_ray(pair)
                uam_reference = self.node._uam_raw_gps_reference()
                uam_gps_ray = (
                    uam_gps_ray_visualization_world_enu_from_point(
                        self.node.baseline_center_frd,
                        uam_reference.get("world_enu_m"),
                    )
                    if uam_reference.get("computation_status") == "computed"
                    else None
                )
                if raw is not None:
                    diagnostic_rays = computed.get("diagnostic_rays", {})
                    self.node._publish_ray_markers(
                        nominal_pose=raw,
                        optical_pose=computed.get("corrected"),
                        ideal=ideal,
                        gps_baseline=computed.get("gps_baselines", {}).get("gps_baseline"),
                        global_baseline=computed.get("gps_baselines", {}).get("global_baseline"),
                        uam_gps_ray=uam_gps_ray,
                        **diagnostic_rays,
                    )
                    if ideal is not None:
                        # Share the same throttle timestamp with the direct GT
                        # publisher so their combined Ideal-marker rate stays
                        # within rviz_live_ray_hz.
                        self.node.last_ideal_ray_marker_publish_monotonic = time.monotonic()
                    self.node.rviz_live_publish_count += 1
                elif ideal is not None:
                    # No detector-derived ray exists outside the FOV, but the
                    # simulation reference ray should remain visible.
                    self.node._publish_ray_markers(ideal=ideal, uam_gps_ray=uam_gps_ray)
                    self.node.last_ideal_ray_marker_publish_monotonic = time.monotonic()
                    self.node.rviz_live_publish_count += 1
                elif uam_gps_ray is not None:
                    self.node._publish_ray_markers(uam_gps_ray=uam_gps_ray)
                    self.node.rviz_live_publish_count += 1
            except Exception as exc:
                self.node.rviz_live_error_count += 1
                if self.node.rviz_live_error_count == 1 or self.node.rviz_live_error_count % 50 == 0:
                    self.node.get_logger().warning(
                        "Latest-only RViz ray publish failed: "
                        f"{type(exc).__name__}: {exc}"
                    )
