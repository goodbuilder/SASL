"""ROS serialization and canonical measurement publication.

All PointStamped and numeric position arrays use baseline-center FRD
[forward, right, down] metres. Timestamps are pair midpoint nanoseconds.
This adapter owns message construction/publication while the node retains
publisher creation and public compatibility delegates.
"""

import json
import math
import numpy as np

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME


MEASUREMENT_PUBLICATION_METHODS = [
    "_point_stamped",
    "_publish_canonical_pair_positions",
    "_publish_estimated_position",
    "_bbox_adjustment_to_msg",
    "_profile_status_to_msg",
    "_detections_to_msg",
    "_detection_from_values",
    "_detection_to_values",
    "_external_detections_cb"
]


class MeasurementPublicationAdapter:
    def __init__(self, node, *, point_stamped_type, float32_multi_array_type, string_type):
        self.node = node
        self.point_stamped_type = point_stamped_type
        self.float32_multi_array_type = float32_multi_array_type
        self.string_type = string_type

    def _point_stamped(self, stamp_ns, forward_m, right_m, down_m):
        message = self.point_stamped_type()
        message.header.stamp.sec = int(stamp_ns) // 1_000_000_000
        message.header.stamp.nanosec = int(stamp_ns) % 1_000_000_000
        message.header.frame_id = BASELINE_CENTER_FRD_FRAME
        message.point.x = float(forward_m)
        message.point.y = float(right_m)
        message.point.z = float(down_m)
        return message

    def _publish_canonical_pair_positions(self, metrics):
        stamp_ns = metrics.get("pair_midpoint_stamp_ns")
        pair_id = metrics.get("pair_id")
        if stamp_ns is None:
            return
        if (
            metrics.get("computation_status") == "computed"
            and pair_id is not None
            and pair_id > self.node.last_raw_published_pair_id
        ):
            self.node.raw_computed_count += 1
            self.node.nominal_pose_position_pub.publish(
                self.node._point_stamped(
                    stamp_ns,
                    metrics["estimated_forward_m"],
                    metrics["estimated_right_m"],
                    metrics["estimated_down_m"],
                )
            )
            self.node.last_raw_published_pair_id = pair_id
        if (
            metrics.get("attitude_corrected_computation_status") == "computed"
            and pair_id is not None
            and pair_id > self.node.last_corrected_published_pair_id
        ):
            self.node.corrected_computed_count += 1
            self.node.attitude_corrected_position_pub.publish(
                self.node._point_stamped(
                    stamp_ns,
                    metrics["attitude_corrected_forward_m"],
                    metrics["attitude_corrected_right_m"],
                    metrics["attitude_corrected_down_m"],
                )
            )
            self.node.last_corrected_published_pair_id = pair_id
        source_publisher_names = {
            "gps_baseline": (
                "gps_baseline_position_pub",
                "gps_baseline_plot_pub",
            ),
            "global_baseline": (
                "global_baseline_position_pub",
                "global_baseline_plot_pub",
            ),
        }
        last_published_by_source = getattr(
            self.node, "last_gps_baseline_published_pair_id_by_source", None
        )
        if last_published_by_source is None:
            last_published_by_source = {}
            self.node.last_gps_baseline_published_pair_id_by_source = (
                last_published_by_source
            )
        for source, publisher_names in source_publisher_names.items():
            prefix = source
            if (
                metrics.get(f"{prefix}_computation_status") == "computed"
                and pair_id is not None
                and pair_id > last_published_by_source.get(source, 0)
            ):
                position_publisher = getattr(self.node, publisher_names[0], None)
                plot_publisher = getattr(self.node, publisher_names[1], None)
                if position_publisher is None or plot_publisher is None:
                    continue
                forward_m = metrics[f"{prefix}_forward_m"]
                right_m = metrics[f"{prefix}_right_m"]
                down_m = metrics[f"{prefix}_down_m"]
                position_publisher.publish(
                    self.node._point_stamped(
                        stamp_ns,
                        forward_m,
                        right_m,
                        down_m,
                    )
                )
                plot_publisher.publish(
                    self.float32_multi_array_type(
                        data=[float(forward_m), float(right_m), float(down_m)]
                    )
                )
                last_published_by_source[source] = pair_id
        if (
            metrics.get("uam_raw_gps_reference_computation_status") == "computed"
            and pair_id is not None
            and pair_id > getattr(self.node, "last_uam_raw_gps_reference_published_pair_id", 0)
        ):
            forward_m = metrics["uam_raw_gps_reference_forward_m"]
            right_m = metrics["uam_raw_gps_reference_right_m"]
            down_m = metrics["uam_raw_gps_reference_down_m"]
            self.node.uam_raw_gps_reference_position_pub.publish(
                self.node._point_stamped(stamp_ns, forward_m, right_m, down_m)
            )
            self.node.uam_raw_gps_reference_plot_pub.publish(
                self.float32_multi_array_type(data=[float(forward_m), float(right_m), float(down_m)])
            )
            self.node.last_uam_raw_gps_reference_published_pair_id = pair_id

    def _publish_estimated_position(self, metrics):
        estimated_publisher = getattr(self.node, "nominal_pose_plot_pub", None)
        if estimated_publisher is not None:
            estimated_publisher.publish(
                self.float32_multi_array_type(
                    data=[
                        math.nan
                        if metrics.get("estimated_forward_m") is None
                        else float(metrics["estimated_forward_m"]),
                        math.nan
                        if metrics.get("estimated_right_m") is None
                        else float(metrics["estimated_right_m"]),
                        math.nan
                        if metrics.get("estimated_down_m") is None
                        else float(metrics["estimated_down_m"]),
                        math.nan
                        if metrics.get("estimated_range_m") is None
                        else float(metrics["estimated_range_m"]),
                    ]
                )
            )
        corrected_publisher = getattr(self.node, "corrected_position_pub", None)
        if corrected_publisher is not None:
            corrected_publisher.publish(
                self.float32_multi_array_type(
                    data=[
                        math.nan
                        if metrics.get("corrected_forward_m") is None
                        else float(metrics["corrected_forward_m"]),
                        math.nan
                        if metrics.get("corrected_right_m") is None
                        else float(metrics["corrected_right_m"]),
                        math.nan
                        if metrics.get("corrected_down_m") is None
                        else float(metrics["corrected_down_m"]),
                        math.nan
                        if metrics.get("corrected_range_m") is None
                        else float(metrics["corrected_range_m"]),
                    ]
                )
            )
        optical_publisher = getattr(self.node, "optical_pose_position_pub", None)
        if optical_publisher is not None:
            optical_publisher.publish(
                self.float32_multi_array_type(
                    data=[
                        math.nan if metrics.get("optical_forward_m") is None else float(metrics["optical_forward_m"]),
                        math.nan if metrics.get("optical_right_m") is None else float(metrics["optical_right_m"]),
                        math.nan if metrics.get("optical_down_m") is None else float(metrics["optical_down_m"]),
                        math.nan if metrics.get("optical_range_m") is None else float(metrics["optical_range_m"]),
                    ]
                )
            )
        ideal_publisher = getattr(self.node, "ideal_position_pub", None)
        if ideal_publisher is not None:
            ideal_publisher.publish(
                self.float32_multi_array_type(
                    data=[
                        math.nan if metrics.get("ideal_forward_m") is None else float(metrics["ideal_forward_m"]),
                        math.nan if metrics.get("ideal_right_m") is None else float(metrics["ideal_right_m"]),
                        math.nan if metrics.get("ideal_down_m") is None else float(metrics["ideal_down_m"]),
                        math.nan if metrics.get("ideal_range_m") is None else float(metrics["ideal_range_m"]),
                    ]
                )
            )
        bbox_adjustment = metrics.get("bbox_adjustment")
        bbox_publisher = getattr(self.node, "bbox_adjustment_pub", None)
        if bbox_adjustment is not None and bbox_publisher is not None:
            bbox_publisher.publish(self.node._bbox_adjustment_to_msg(bbox_adjustment))
        profile_publisher = getattr(self.node, "profile_status_pub", None)
        if profile_publisher is not None:
            profile_publisher.publish(self.node._profile_status_to_msg(metrics))

    def _bbox_adjustment_to_msg(self, adjustment):
        return self.float32_multi_array_type(
            data=[
                float(adjustment["left_cx"]),
                float(adjustment["left_cy"]),
                float(adjustment["right_cx"]),
                float(adjustment["right_cy"]),
                float(adjustment["left_du"]),
                float(adjustment["left_dv"]),
                float(adjustment["right_du"]),
                float(adjustment["right_dv"]),
                float(adjustment.get("profile_id_hash", 0.0)),
                float(adjustment.get("fallback_level", math.nan)),
                float(adjustment.get("sample_count", math.nan)),
                float(adjustment.get("raw_disparity_px", math.nan)),
                float(adjustment.get("raw_range_m", math.nan)),
            ]
        )

    def _profile_status_to_msg(self, metrics):
        status = dict(metrics.get("profile_status") or self.node._profile_status())
        status.update(
            {
                "pair_id": metrics.get("pair_id"),
                "epoch": metrics.get("epoch"),
                "left_stamp_ns": metrics.get("left_stamp_ns"),
                "right_stamp_ns": metrics.get("right_stamp_ns"),
                "pair_midpoint_stamp_ns": metrics.get("pair_midpoint_stamp_ns"),
                "measurement_queue_depth": len(getattr(self.node, "measurement_pair_queue", ())),
                "measurement_queue_capacity": getattr(self.node, "measurement_queue_capacity", None),
                "measurement_latest_only": bool(getattr(self.node, "measurement_latest_only", False)),
                "measurement_latest_drop_count": getattr(self.node, "measurement_latest_drop_count", 0),
                "measurement_overrun_invalid": getattr(self.node, "measurement_overrun_invalid", False),
            }
        )
        adapter = getattr(self.node, "observation_fusion_adapter", None)
        residual_profile = getattr(self.node, "disparity_residual_profile", None)
        status["observation_fusion_status"] = str(metrics.get("observation_fusion_status", "disabled"))
        status["observation_fusion_adapter"] = "" if adapter is None else type(adapter).__name__
        status["observation_fusion_buffer_count"] = len(getattr(self.node, "observation_fusion_buffer", []))
        status["observation_fusion_required_count"] = self.node._observation_fusion_required_observations(adapter)
        status["disparity_residual_status"] = str(metrics.get("disparity_residual_status", "disabled"))
        status["disparity_residual_profile_id"] = "" if residual_profile is None else residual_profile.profile_id
        status["disparity_residual_feature_summary_status"] = str(
            metrics.get("disparity_residual_feature_summary_status", "disabled")
        )
        status["disparity_residual_feature_selector_summary_status"] = str(
            metrics.get("disparity_residual_feature_selector_summary_status", "disabled")
        )
        status["disparity_residual_feature_selector_window_count"] = int(
            metrics.get("disparity_residual_feature_selector_window_count", 0)
        )
        status["disparity_residual_feature_selector_status"] = str(
            metrics.get("disparity_residual_feature_selector_status", "disabled")
        )
        status["disparity_residual_feature_selected_label"] = str(
            metrics.get("disparity_residual_feature_selected_label", "")
        )
        return self.string_type(data=json.dumps(status, ensure_ascii=False, sort_keys=True))

    def _detections_to_msg(self, left, right):
        msg = self.float32_multi_array_type()
        msg.data = self.node._detection_to_values(left) + self.node._detection_to_values(right)
        return msg

    def _detection_from_values(self, values):
        if len(values) < 8:
            return None
        try:
            valid = float(values[0]) > 0.5
            x1 = float(values[1])
            y1 = float(values[2])
            x2 = float(values[3])
            y2 = float(values[4])
            confidence = float(values[5])
            cx = float(values[6])
            cy = float(values[7])
        except (TypeError, ValueError):
            return None
        if not valid:
            return None
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2, confidence, cx, cy)):
            return None
        return {
            "bbox": (x1, y1, x2, y2),
            "confidence": confidence,
            "cx": cx,
            "cy": cy,
        }

    def _detection_to_values(self, detection):
        if detection is None or "bbox" not in detection:
            return [0.0, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan]
        x1, y1, x2, y2 = detection["bbox"]
        return [
            1.0,
            float(x1),
            float(y1),
            float(x2),
            float(y2),
            float(detection.get("confidence", math.nan)),
            float(detection.get("cx", math.nan)),
            float(detection.get("cy", math.nan)),
        ]

    def _external_detections_cb(self, msg):
        values = list(getattr(msg, "data", []))
        left = self.node._detection_from_values(values[:8])
        right = self.node._detection_from_values(values[8:16])
        state = self.node._mission_state()
        actual = self.node._read_actual_distance()
        width = int(getattr(self.node, "external_bbox_detection_width_px", 1280))
        height = int(getattr(self.node, "external_bbox_detection_height_px", 720))
        synthetic_image = np.zeros((height, width, 3), dtype=np.uint8)
        metrics = self.node._calculate_metrics(
            synthetic_image,
            left,
            right,
            math.nan if actual is None else float(actual),
            state["phase"],
        )
        metrics["status"] = "External bbox tracking"
        with self.node.lock:
            self.node.left_detection = left
            self.node.right_detection = right
            self.node.metrics = metrics
        self.node.detections_pub.publish(self.node._detections_to_msg(left, right))
        self.node._publish_estimated_position(metrics)
