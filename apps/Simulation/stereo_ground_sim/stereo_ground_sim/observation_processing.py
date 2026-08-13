"""Observation fusion, correction-profile, and metric calculation coordinator.

Inputs are OpenCV image observations (u/right, v/down pixels) and already
contracted ray results. This component preserves baseline-center FRD output
metadata and never converts values to world ENU, NED, or camera-centre fields.
"""

import math
import time
from collections import deque

import numpy as np

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME
from stereo_ground_sim.disparity_residual_feature_selector import (
    DisparityResidualFeatureSelector,
)
from stereo_ground_sim.disparity_residual_feature_summary import (
    feature_summary_from_arrays,
    feature_summary_from_runtime_rows,
)
from stereo_ground_sim.reprojection_profile import (
    OriginBBoxReprojectionProfile,
    StereoCameraModel,
)
from stereo_ground_sim.runtime_deployment_selector import (
    DisparityResidualRuntimeProfile,
    MultiBurstObservationFusion,
    RepeatedObservationFusion,
    RuntimeImageObservation,
    RuntimeMeasurementSample,
    RuntimeStereoCameraContract,
)
from stereo_ground_sim.runtime_guard_policy import (
    BoundedGainForwardCellGuardPolicy,
    EstimatedToTruthAffineProfile,
    EstimatedToTruthDisparityProfile,
    EstimatedToTruthSim3Profile,
)


OBSERVATION_PROCESSING_METHODS = [
    "_publish_synthetic_detection_sample",
    "_raw_from_runtime_sample",
    "_apply_observation_fusion",
    "_adjust_bbox_centers",
    "_camera_model",
    "_profile_status",
    "_apply_bbox_reprojection_profile",
    "_bbox_feature_base",
    "_bbox_feature_value",
    "_apply_bbox_output_profile",
    "_apply_bounded_gain_forward_cell_guard_profile",
    "_apply_estimated_to_truth_sim3_profile",
    "_apply_estimated_to_truth_disparity_profile",
    "_apply_estimated_to_truth_affine_profile",
    "_apply_disparity_residual_profile",
    "_select_disparity_residual_profile_from_features",
    "_append_disparity_residual_feature_selector_window",
    "_disparity_residual_feature_selector_summary",
    "_disparity_residual_feature_summary",
    "_calculate_metrics"
]


class ObservationProcessingCoordinator:
    def __init__(self, node):
        self.node = node

    def _publish_synthetic_detection_sample(self):
        if not self.node.enable_synthetic_detections:
            return
        state = self.node._mission_state()
        actual = self.node._read_actual_distance()
        position = self.node.cached_ground_truth_position
        if position is None or position.get("ground_truth_world_enu_m") is None:
            return
        width = int(self.node.synthetic_detection_width_px)
        height = int(self.node.synthetic_detection_height_px)
        left, right = self.node._ideal_detection_pair_from_reference_world(
            position["ground_truth_world_enu_m"],
            width=width,
            height=height,
        )
        if left is None or right is None:
            return
        synthetic_image = np.zeros((height, width, 3), dtype=np.uint8)
        metrics = self.node._calculate_metrics(
            synthetic_image,
            left,
            right,
            math.nan if actual is None else float(actual),
            state["phase"],
        )
        metrics["status"] = "Synthetic bbox tracking for headless/operator pipeline validation"
        with self.node.lock:
            self.node.left_detection = left
            self.node.right_detection = right
            self.node.metrics = metrics
        self.node.detections_pub.publish(self.node._detections_to_msg(left, right))
        self.node._publish_estimated_position(metrics)

    def _raw_from_runtime_sample(self, sample):
        if sample.left_u_px is None or sample.left_v_px is None:
            return {
                "right_m": None,
                "down_m": None,
                "forward_m": None,
                "range_m": None,
                "distance_m": None,
                "disparity_px": float(sample.measured_disparity_px),
                "computation_status": "invalid_input:missing_pixels",
            }
        left_u = float(sample.left_u_px)
        left_v = float(sample.left_v_px)
        right_u = left_u - float(sample.measured_disparity_px)
        right_v = left_v
        return self.node._triangulate_center_pixels(left_u, left_v, right_u, right_v, 1280, 720)

    def _apply_observation_fusion(self, left, right, width, height):
        adapter = getattr(self.node, "observation_fusion_adapter", None)
        if adapter is None:
            return None, "disabled"
        disparity = abs(float(left["cx"]) - float(right["cx"]))
        if disparity < 0.5:
            return None, "invalid_disparity"
        observation = RuntimeImageObservation.from_mapping(
            {
                "left_u_px": float(left["cx"]),
                "left_v_px": float(left["cy"]),
                "measured_disparity_px": disparity,
            }
        )
        if observation is None:
            return None, "invalid_observation"
        self.node.observation_fusion_buffer.append(observation)
        adapter.camera = RuntimeStereoCameraContract(
            width_px=float(width),
            height_px=float(height),
            horizontal_fov_rad=float(self.node.horizontal_fov_rad),
            baseline_m=float(self.node.baseline_m),
        )
        observations = list(self.node.observation_fusion_buffer)
        if isinstance(adapter, RepeatedObservationFusion):
            fused = adapter.fuse(observations)
        elif isinstance(adapter, MultiBurstObservationFusion):
            per_burst = int(adapter.min_observations_per_burst)
            required = int(adapter.min_bursts) * per_burst
            recent = observations[-required:]
            bursts = [recent[index : index + per_burst] for index in range(0, len(recent), per_burst)]
            fused = adapter.fuse(bursts)
        else:
            return None, "unsupported_adapter"
        if fused is None:
            return None, f"warming_up:{len(observations)}/{self.node._observation_fusion_required_observations(adapter)}"
        return self.node._raw_from_runtime_sample(fused), f"fused:{type(adapter).__name__}:{len(observations)}"

    def _adjust_bbox_centers(self, left, right):
        matrix = self.node.bbox_center_adjustment
        if matrix is None or not isinstance(matrix, np.ndarray):
            return None
        left_vec = np.array([float(left["cx"]), float(left["cy"]), 1.0])
        right_vec = np.array([float(right["cx"]), float(right["cy"]), 1.0])
        left_adjusted = matrix[0:3, 0:3] @ left_vec + matrix[0:3, 3]
        right_adjusted = matrix[3:6, 0:3] @ right_vec + matrix[3:6, 3]
        return {
            "left_cx": float(left_adjusted[0]),
            "left_cy": float(left_adjusted[1]),
            "right_cx": float(right_adjusted[0]),
            "right_cy": float(right_adjusted[1]),
            "left_du": float(left_adjusted[0] - left_vec[0]),
            "left_dv": float(left_adjusted[1] - left_vec[1]),
            "right_du": float(right_adjusted[0] - right_vec[0]),
            "right_dv": float(right_adjusted[1] - right_vec[1]),
            "profile_id_hash": 0.0,
            "fallback_level": 0.0,
            "sample_count": 0.0,
            "raw_disparity_px": math.nan,
            "raw_range_m": math.nan,
        }
    def _camera_model(self, width, height):
        return StereoCameraModel(
            width_px=float(width),
            height_px=float(height),
            horizontal_fov_rad=float(self.node.horizontal_fov_rad),
            baseline_m=float(self.node.baseline_m),
        )

    def _profile_status(self, profile_id="", profile_mode="", fallback_level=3, fallback_reason="no_profile"):
        return {
            "profile_id": str(profile_id or ""),
            "profile_mode": str(profile_mode or ""),
            "fallback_level": int(fallback_level),
            "fallback_reason": str(fallback_reason or ""),
        }

    def _apply_bbox_reprojection_profile(self, left, right, raw, width, height):
        profile = self.node.bbox_center_adjustment
        if not isinstance(profile, OriginBBoxReprojectionProfile):
            return None, None, self.node._profile_status()
        if raw["disparity_px"] is None or raw["range_m"] is None:
            result = profile.apply(
                left.get("cx", math.nan) if left else math.nan,
                left.get("cy", math.nan) if left else math.nan,
                right.get("cx", math.nan) if right else math.nan,
                right.get("cy", math.nan) if right else math.nan,
                raw_disparity_px=raw.get("disparity_px"),
                raw_range_m=raw.get("range_m"),
            )
            return None, None, self.node._profile_status(
                result.profile_id,
                profile.mode,
                result.fallback_level,
                result.fallback_reason,
            )
        result = profile.apply(
            float(left["cx"]),
            float(left["cy"]),
            float(right["cx"]),
            float(right["cy"]),
            raw_disparity_px=raw["disparity_px"],
            raw_range_m=raw["range_m"],
        )
        adjustment = {
            "left_cx": result.corrected_left_u,
            "left_cy": result.corrected_left_v,
            "right_cx": result.corrected_right_u,
            "right_cy": result.corrected_right_v,
            "left_du": result.du_left,
            "left_dv": result.dv_left,
            "right_du": result.du_right,
            "right_dv": result.dv_right,
            "profile_id_hash": float(profile.profile_id_hash),
            "fallback_level": float(result.fallback_level),
            "sample_count": float(result.sample_count),
            "raw_disparity_px": float(raw["disparity_px"]),
            "raw_range_m": float(raw["range_m"]),
        }
        corrected = self.node._triangulate_center_pixels(
            result.corrected_left_u,
            result.corrected_left_v,
            result.corrected_right_u,
            result.corrected_right_v,
            width,
            height,
        )
        return corrected, adjustment, self.node._profile_status(
            result.profile_id,
            profile.mode,
            result.fallback_level,
            result.fallback_reason,
        )

    def _bbox_feature_base(self, left, right, raw, width, height):
        left_x1, left_y1, left_x2, left_y2 = left["bbox"]
        right_x1, right_y1, right_x2, right_y2 = right["bbox"]
        left_width = max(0.0, float(left_x2) - float(left_x1))
        left_height = max(0.0, float(left_y2) - float(left_y1))
        right_width = max(0.0, float(right_x2) - float(right_x1))
        right_height = max(0.0, float(right_y2) - float(right_y1))
        left_cx = float(left["cx"])
        left_cy = float(left["cy"])
        right_cx = float(right["cx"])
        right_cy = float(right["cy"])
        disparity = left_cx - right_cx
        abs_disparity = abs(disparity)
        width = float(width)
        height = float(height)
        center_x_norm = (((left_cx + right_cx) * 0.5) / width) - 0.5
        center_y_norm = (((left_cy + right_cy) * 0.5) / height) - 0.5
        base = {
            "bias": 1.0,
            "estimated_forward_m": raw["forward_m"],
            "estimated_right_m": raw["right_m"],
            "estimated_down_m": raw["down_m"],
            "estimated_range_m": raw["range_m"],
            "left_x1_norm": float(left_x1) / width,
            "left_y1_norm": float(left_y1) / height,
            "left_x2_norm": float(left_x2) / width,
            "left_y2_norm": float(left_y2) / height,
            "right_x1_norm": float(right_x1) / width,
            "right_y1_norm": float(right_y1) / height,
            "right_x2_norm": float(right_x2) / width,
            "right_y2_norm": float(right_y2) / height,
            "left_cx_norm": left_cx / width,
            "left_cy_norm": left_cy / height,
            "right_cx_norm": right_cx / width,
            "right_cy_norm": right_cy / height,
            "center_x_norm": center_x_norm,
            "center_y_norm": center_y_norm,
            "signed_disparity_norm": disparity / width,
            "disparity_norm": abs_disparity / width,
            "width_over_disparity": width / max(abs_disparity, 1.0e-6),
            "left_width_norm": left_width / width,
            "left_height_norm": left_height / height,
            "right_width_norm": right_width / width,
            "right_height_norm": right_height / height,
            "avg_width_norm": ((left_width + right_width) * 0.5) / width,
            "avg_height_norm": ((left_height + right_height) * 0.5) / height,
            "width_ratio": left_width / max(right_width, 1.0e-6),
            "height_ratio": left_height / max(right_height, 1.0e-6),
            "left_confidence": float(left.get("confidence", 0.0)),
            "right_confidence": float(right.get("confidence", 0.0)),
            "confidence_min": min(float(left.get("confidence", 0.0)), float(right.get("confidence", 0.0))),
        }
        return base

    def _bbox_feature_value(self, name, base):
        if name in base:
            return float(base[name])
        if name.endswith("^2"):
            return self.node._bbox_feature_value(name[:-2], base) ** 2
        if "*" in name:
            value = 1.0
            for part in name.split("*"):
                value *= self.node._bbox_feature_value(part, base)
            return float(value)
        raise KeyError(name)

    def _apply_bbox_output_profile(self, left, right, raw, width, height):
        profile = self.node.bbox_center_adjustment
        if not isinstance(profile, dict):
            return None
        if profile.get("mode") == "origin_bbox_anchor":
            anchor = profile["anchor"]
            left_x1, left_y1, left_x2, left_y2 = left["bbox"]
            right_x1, right_y1, right_x2, right_y2 = right["bbox"]
            left_cx = float(left_x1) + anchor["left_x_fraction"] * (float(left_x2) - float(left_x1))
            right_cx = float(right_x1) + anchor["right_x_fraction"] * (float(right_x2) - float(right_x1))
            left_cy = float(left_y1) + anchor["left_y_fraction"] * (float(left_y2) - float(left_y1))
            right_cy = float(right_y1) + anchor["right_y_fraction"] * (float(right_y2) - float(right_y1))
            return self.node._triangulate_center_pixels(left_cx, left_cy, right_cx, right_cy, width, height)
        if profile.get("mode") != "origin_bbox_output_ridge":
            return None
        if raw["right_m"] is None or raw["down_m"] is None or raw["forward_m"] is None:
            return None
        try:
            base = self.node._bbox_feature_base(left, right, raw, width, height)
            values = np.array(
                [self.node._bbox_feature_value(name, base) for name in profile["feature_names"]],
                dtype=float,
            )
        except Exception as exc:
            self.node.get_logger().warning(f"failed to compute bbox output profile features: {exc}")
            return None
        normalized = (values - profile["feature_mean"]) / profile["feature_scale"]
        predicted = profile["coefficients"] @ normalized
        estimated = np.array([raw["forward_m"], raw["right_m"], raw["down_m"]], dtype=float)
        corrected_frd = estimated + profile["blend"] * (predicted - estimated)
        if not np.all(np.isfinite(corrected_frd)):
            return None
        corrected_forward_m = float(corrected_frd[0])
        corrected_right_m = float(corrected_frd[1])
        corrected_down_m = float(corrected_frd[2])
        return {
            "right_m": corrected_right_m,
            "down_m": corrected_down_m,
            "forward_m": corrected_forward_m,
            "range_m": math.sqrt(
                corrected_right_m * corrected_right_m
                + corrected_down_m * corrected_down_m
                + corrected_forward_m * corrected_forward_m
            ),
            "disparity_px": raw["disparity_px"],
        }

    def _apply_bounded_gain_forward_cell_guard_profile(self, left, right, raw, width, height):
        profile = self.node.bbox_center_adjustment
        if not isinstance(profile, BoundedGainForwardCellGuardPolicy):
            return None, self.node._profile_status()
        result = profile.apply(
            left,
            right,
            raw,
            width_px=float(width),
            height_px=float(height),
            horizontal_fov_rad=float(self.node.horizontal_fov_rad),
            baseline_m=float(self.node.baseline_m),
        )
        return result.corrected, result.status

    def _apply_estimated_to_truth_sim3_profile(self, raw, left=None, right=None):
        profile = self.node.bbox_center_adjustment
        if not isinstance(profile, EstimatedToTruthSim3Profile):
            return None, self.node._profile_status()
        profile_raw = dict(raw)
        if left is not None:
            profile_raw.setdefault("left_u_px", left.get("cx"))
            profile_raw.setdefault("left_v_px", left.get("cy"))
        if right is not None:
            profile_raw.setdefault("right_u_px", right.get("cx"))
            profile_raw.setdefault("right_v_px", right.get("cy"))
        result = profile.apply(profile_raw)
        return result.corrected, result.status

    def _apply_estimated_to_truth_disparity_profile(self, raw):
        profile = self.node.bbox_center_adjustment
        if not isinstance(profile, EstimatedToTruthDisparityProfile):
            return None, self.node._profile_status()
        result = profile.apply(raw)
        return result.corrected, result.status

    def _apply_estimated_to_truth_affine_profile(self, raw):
        profile = self.node.bbox_center_adjustment
        if not isinstance(profile, EstimatedToTruthAffineProfile):
            return None, self.node._profile_status()
        result = profile.apply(raw)
        return result.corrected, result.status

    def _apply_disparity_residual_profile(self, corrected, raw, profile=None):
        if profile is None:
            profile = getattr(self.node, "disparity_residual_profile", None)
        if not isinstance(profile, DisparityResidualRuntimeProfile):
            return corrected, "disabled"
        if corrected is None:
            return corrected, "skipped:no_corrected_base"
        disparity = raw.get("disparity_px")
        if disparity is None or not math.isfinite(float(disparity)) or float(disparity) <= 0.0:
            return corrected, "skipped:invalid_disparity"
        sample = RuntimeMeasurementSample(
            raw_right_m=float(corrected["right_m"]),
            raw_down_m=float(corrected["down_m"]),
            raw_forward_m=float(corrected["forward_m"]),
            measured_disparity_px=float(disparity),
            raw_range_m=float(corrected.get("range_m", math.nan)),
            left_u_px=raw.get("left_u_px"),
            left_v_px=raw.get("left_v_px"),
        )
        residual = profile.apply(sample)
        if residual is None:
            return corrected, "skipped:profile_noop"
        return self.node._raw_from_runtime_sample(residual), f"applied:{profile.profile_id or profile.method}"

    def _select_disparity_residual_profile_from_features(self, feature_summary):
        selector = getattr(self.node, "disparity_residual_feature_selector", None)
        if not isinstance(selector, DisparityResidualFeatureSelector):
            return None, "disabled", ""
        selection = selector.select(feature_summary)
        label = str(selection.get("selected_label", ""))
        profile_path = str(selection.get("selected_profile", ""))
        if not selection.get("selected_profile_exists", False):
            return None, f"skipped:profile_missing:{label}", label
        try:
            profile = DisparityResidualRuntimeProfile.from_json_file(profile_path)
        except Exception as exc:
            return None, f"skipped:{type(exc).__name__}:{label}", label
        return profile, f"selected:{label}", label

    def _append_disparity_residual_feature_selector_window(self, *, raw, corrected):
        if raw is None:
            return "skipped:no_raw"
        required = ("left_u_px", "left_v_px", "disparity_px", "right_m", "down_m", "forward_m")
        if any(raw.get(field) is None for field in required):
            return "skipped:missing_runtime_fields"
        observation = corrected if corrected is not None else raw
        try:
            row = {
                "left_u_px": float(raw["left_u_px"]),
                "left_v_px": float(raw["left_v_px"]),
                "measured_disparity_px": float(raw["disparity_px"]),
                "fused_disparity_px": float(observation.get("disparity_px", raw["disparity_px"])),
                "raw_right_m": float(raw["right_m"]),
                "raw_down_m": float(raw["down_m"]),
                "raw_forward_m": float(raw["forward_m"]),
                "observation_right_m": float(observation["right_m"]),
                "observation_down_m": float(observation["down_m"]),
                "observation_forward_m": float(observation["forward_m"]),
            }
        except Exception as exc:
            return f"skipped:{type(exc).__name__}"
        window = getattr(self.node, "disparity_residual_feature_selector_window", None)
        if not isinstance(window, deque):
            self.node._reset_disparity_residual_feature_selector_window()
            window = self.node.disparity_residual_feature_selector_window
        window.append(row)
        return f"ready:{len(window)}"

    def _disparity_residual_feature_selector_summary(self, *, raw, corrected):
        append_status = self.node._append_disparity_residual_feature_selector_window(raw=raw, corrected=corrected)
        if not append_status.startswith("ready:"):
            return {}, append_status
        window = getattr(self.node, "disparity_residual_feature_selector_window", deque())
        min_samples = max(1, int(getattr(self.node, "disparity_residual_feature_selector_min_samples", 1)))
        if len(window) < min_samples:
            return {}, f"accumulating:{len(window)}/{min_samples}"
        try:
            feature_row = feature_summary_from_runtime_rows(
                list(window),
                feature_label="stereodistance_node_runtime_window",
                observation_rdf_source="corrected_runtime_window",
            )
        except Exception as exc:
            return {}, f"skipped:{type(exc).__name__}"
        return feature_row, f"ready:runtime_window:{len(window)}"

    def _disparity_residual_feature_summary(self, *, raw, corrected):
        if raw is None:
            return {}, "skipped:no_raw"
        required = ("left_u_px", "left_v_px", "disparity_px", "right_m", "down_m", "forward_m")
        if any(raw.get(field) is None for field in required):
            return {}, "skipped:missing_runtime_fields"
        observation = corrected if corrected is not None else raw
        observation_source = "corrected_current_sample" if corrected is not None else "raw_current_sample_fallback"
        try:
            feature_row = feature_summary_from_arrays(
                left_u_px=[float(raw["left_u_px"])],
                left_v_px=[float(raw["left_v_px"])],
                measured_disparity_px=[float(raw["disparity_px"])],
                raw_rdf_m=[
                    [
                        float(raw["right_m"]),
                        float(raw["down_m"]),
                        float(raw["forward_m"]),
                    ]
                ],
                fused_disparity_px=[float(observation.get("disparity_px", raw["disparity_px"]))],
                observation_rdf_m=[
                    [
                        float(observation["right_m"]),
                        float(observation["down_m"]),
                        float(observation["forward_m"]),
                    ]
                ],
                feature_label="stereodistance_node_current_sample",
                observation_rdf_source=observation_source,
            )
        except Exception as exc:
            return {}, f"skipped:{type(exc).__name__}"
        return feature_row, "ready:current_sample"

    def _calculate_metrics(self, left_image, left, right, actual, phase):
        if left is None or right is None:
            return {
                "distance_m": None,
                "disparity_px": None,
                "estimated_right_m": None,
                "estimated_down_m": None,
                "estimated_forward_m": None,
                "estimated_range_m": None,
                "corrected_right_m": None,
                "corrected_down_m": None,
                "corrected_forward_m": None,
                "corrected_range_m": None,
                "corrected_disparity_px": None,
                "legacy_fused_raw": None,
                "bbox_adjustment": None,
                "profile_status": self.node._profile_status(
                    getattr(self.node.bbox_center_adjustment, "profile_id", ""),
                    getattr(self.node.bbox_center_adjustment, "mode", ""),
                    3,
                    "incomplete_bbox_pair",
                ),
                "observation_fusion_status": "incomplete_bbox_pair",
                "actual_distance_m": actual,
                "distance_error_m": None,
                "mission_phase": phase,
                "status": self.node.yolo_load_error or "YOLO target not detected in both images",
            }

        width = left_image.shape[1]
        height = left_image.shape[0]
        # TEMPORARY diagnostic timing (SASL-perf-investigation): pinpointing
        # which sub-step dominates the ~11-21ms result_commit_ms measured on
        # /stereo/result_commit_ms. Pure measurement, no behavior change.
        # Remove once the bottleneck is confirmed and addressed.
        _t_start = time.monotonic()
        raw = self.node._triangulate_center_pixels(left["cx"], left["cy"], right["cx"], right["cy"], width, height)
        _t_triangulate = time.monotonic()
        observation_fusion_status = "disabled"
        fused_raw, observation_fusion_status = self.node._apply_observation_fusion(left, right, width, height)
        _t_fusion = time.monotonic()
        legacy_fused_observation = fused_raw
        raw.setdefault("left_u_px", float(left["cx"]))
        raw.setdefault("left_v_px", float(left["cy"]))
        raw.setdefault("right_u_px", float(right["cx"]))
        raw.setdefault("right_v_px", float(right["cy"]))
        corrected = fused_raw
        bbox_adjustment = None
        profile_status = self.node._profile_status()
        if isinstance(self.node.bbox_center_adjustment, OriginBBoxReprojectionProfile):
            corrected, bbox_adjustment, profile_status = self.node._apply_bbox_reprojection_profile(
                left,
                right,
                raw,
                width,
                height,
            )
        elif isinstance(self.node.bbox_center_adjustment, BoundedGainForwardCellGuardPolicy):
            corrected, profile_status = self.node._apply_bounded_gain_forward_cell_guard_profile(
                left,
                right,
                raw,
                width,
                height,
            )
        elif isinstance(self.node.bbox_center_adjustment, EstimatedToTruthSim3Profile):
            corrected, profile_status = self.node._apply_estimated_to_truth_sim3_profile(raw, left, right)
        elif isinstance(self.node.bbox_center_adjustment, EstimatedToTruthDisparityProfile):
            corrected, profile_status = self.node._apply_estimated_to_truth_disparity_profile(raw)
        elif isinstance(self.node.bbox_center_adjustment, EstimatedToTruthAffineProfile):
            corrected, profile_status = self.node._apply_estimated_to_truth_affine_profile(raw)
        elif isinstance(self.node.bbox_center_adjustment, dict):
            corrected = self.node._apply_bbox_output_profile(left, right, raw, width, height)
            profile_status = self.node._profile_status(
                self.node.bbox_center_adjustment.get("profile_id", ""),
                self.node.bbox_center_adjustment.get("mode", "legacy_json"),
                0 if corrected is not None else 3,
                "legacy_output_profile" if corrected is not None else "legacy_output_profile_noop",
            )
        else:
            bbox_adjustment = self.node._adjust_bbox_centers(left, right)
        if bbox_adjustment is not None:
            corrected = self.node._triangulate_center_pixels(
                bbox_adjustment["left_cx"],
                bbox_adjustment["left_cy"],
                bbox_adjustment["right_cx"],
                bbox_adjustment["right_cy"],
                width,
                height,
            )
            if not isinstance(self.node.bbox_center_adjustment, OriginBBoxReprojectionProfile):
                profile_status = self.node._profile_status("global_affine_csv", "global_affine_csv", 0, "csv_affine")
        _t_profile = time.monotonic()
        pre_residual_feature_summary, pre_residual_feature_summary_status = self.node._disparity_residual_feature_summary(
            raw=raw,
            corrected=corrected,
        )
        _t_feature_summary_1 = time.monotonic()
        selector_feature_summary = pre_residual_feature_summary
        selector_feature_summary_status = pre_residual_feature_summary_status
        selected_residual_profile = None
        feature_selector_status = "disabled"
        feature_selected_label = ""
        if not isinstance(getattr(self.node, "disparity_residual_profile", None), DisparityResidualRuntimeProfile):
            if isinstance(getattr(self.node, "disparity_residual_feature_selector", None), DisparityResidualFeatureSelector):
                selector_feature_summary, selector_feature_summary_status = (
                    self.node._disparity_residual_feature_selector_summary(raw=raw, corrected=corrected)
                )
            if selector_feature_summary:
                selected_residual_profile, feature_selector_status, feature_selected_label = (
                    self.node._select_disparity_residual_profile_from_features(selector_feature_summary)
                )
            elif selector_feature_summary_status != pre_residual_feature_summary_status:
                feature_selector_status = selector_feature_summary_status
        _t_selector = time.monotonic()
        corrected, disparity_residual_status = self.node._apply_disparity_residual_profile(
            corrected,
            raw,
            profile=selected_residual_profile,
        )
        _t_residual_apply = time.monotonic()
        if disparity_residual_status.startswith("applied:"):
            feature_summary, feature_summary_status = self.node._disparity_residual_feature_summary(raw=raw, corrected=corrected)
        else:
            feature_summary, feature_summary_status = pre_residual_feature_summary, pre_residual_feature_summary_status
        _t_feature_summary_2 = time.monotonic()
        if feature_summary_status != "ready:current_sample":
            feature_summary_status = pre_residual_feature_summary_status
            feature_summary = pre_residual_feature_summary
        error = (
            None
            if raw["distance_m"] is None or actual is None or not math.isfinite(float(actual))
            else raw["distance_m"] - actual
        )
        _get_logger = getattr(self.node, "get_logger", None)
        if _get_logger is not None:
            try:
                _get_logger().info(
                    "PERF _calculate_metrics ms: "
                    f"triangulate={1000*(_t_triangulate-_t_start):.2f} "
                    f"fusion={1000*(_t_fusion-_t_triangulate):.2f} "
                    f"profile={1000*(_t_profile-_t_fusion):.2f} "
                    f"feature_summary_1={1000*(_t_feature_summary_1-_t_profile):.2f} "
                    f"selector={1000*(_t_selector-_t_feature_summary_1):.2f} "
                    f"residual_apply={1000*(_t_residual_apply-_t_selector):.2f} "
                    f"feature_summary_2={1000*(_t_feature_summary_2-_t_residual_apply):.2f} "
                    f"total={1000*(_t_feature_summary_2-_t_start):.2f}",
                    throttle_duration_sec=2.0,
                )
            except Exception:
                pass
        return {
            "left_u_px": raw.get("left_u_px"),
            "left_v_px": raw.get("left_v_px"),
            "right_u_px": raw.get("right_u_px"),
            "right_v_px": raw.get("right_v_px"),
            "distance_m": raw["distance_m"],
            "disparity_px": raw["disparity_px"],
            "estimated_right_m": raw["right_m"],
            "estimated_down_m": raw["down_m"],
            "estimated_forward_m": raw["forward_m"],
            "estimated_range_m": raw["range_m"],
            "raw_forward_m": raw["forward_m"],
            "raw_right_m": raw["right_m"],
            "raw_down_m": raw["down_m"],
            "raw_range_m": raw["range_m"],
            "raw_computation_status": raw.get("computation_status", "computed"),
            "raw_ray_gap_m": raw.get("ray_gap_m"),
            "raw_intersection_angle_deg": raw.get("intersection_angle_deg"),
            "raw_left_depth_m": raw.get("left_depth_m"),
            "raw_right_depth_m": raw.get("right_depth_m"),
            "point_on_left_ray_baseline_center_frd_m": raw.get("point_on_left_ray_baseline_center_frd_m"),
            "point_on_right_ray_baseline_center_frd_m": raw.get("point_on_right_ray_baseline_center_frd_m"),
            "midpoint_baseline_center_frd_m": raw.get("midpoint_baseline_center_frd_m"),
            "ray_gap_m": raw.get("ray_gap_m"),
            "intersection_angle_deg": raw.get("intersection_angle_deg"),
            "left_depth_m": raw.get("left_depth_m"),
            "right_depth_m": raw.get("right_depth_m"),
            "geometry_rank": raw.get("geometry_rank"),
            "condition_number": raw.get("condition_number"),
            "computation_status": raw.get("computation_status", "computed"),
            "output_frame": raw.get("output_frame", BASELINE_CENTER_FRD_FRAME),
            "corrected_right_m": None if corrected is None else corrected["right_m"],
            "corrected_down_m": None if corrected is None else corrected["down_m"],
            "corrected_forward_m": None if corrected is None else corrected["forward_m"],
            "corrected_range_m": None if corrected is None else corrected["range_m"],
            "corrected_disparity_px": None if corrected is None else corrected["disparity_px"],
            "legacy_fused_raw": legacy_fused_observation,
            "bbox_adjustment": bbox_adjustment,
            "profile_status": profile_status,
            "observation_fusion_status": observation_fusion_status,
            "disparity_residual_status": disparity_residual_status,
            "disparity_residual_feature_summary_status": feature_summary_status,
            "disparity_residual_feature_summary": feature_summary,
            "disparity_residual_feature_selector_summary_status": selector_feature_summary_status,
            "disparity_residual_feature_selector_window_count": len(
                getattr(self.node, "disparity_residual_feature_selector_window", [])
            ),
            "disparity_residual_feature_selector_status": feature_selector_status,
            "disparity_residual_feature_selected_label": feature_selected_label,
            "actual_distance_m": actual,
            "distance_error_m": error,
            "mission_phase": phase,
            "status": f"YOLO tracking {self.node.yolo_class_name}; observation_fusion={observation_fusion_status}",
        }
