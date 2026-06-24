#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bbox_profile_random_validation as validation  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="Train an origin-bbox output correction profile from runtime calibration trajectories."
    )
    parser.add_argument("--calibration-trials", type=int, default=8)
    parser.add_argument(
        "--calibration-csv",
        default="",
        help="Reuse an existing calibration_samples.csv instead of launching simulation.",
    )
    parser.add_argument("--seed", type=int, default=20260610)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--warmup-s", type=float, default=12.0)
    parser.add_argument("--settle-s", type=float, default=3.0)
    parser.add_argument("--trial-duration-s", type=float, default=18.0)
    parser.add_argument("--analysis-skip-s", type=float, default=3.0)
    parser.add_argument("--command-hz", type=float, default=2.0)
    parser.add_argument("--sample-hz", type=float, default=12.0)
    parser.add_argument("--min-samples-per-trial", type=int, default=40)
    parser.add_argument("--baseline-m", type=float, default=25.0)
    parser.add_argument("--camera-height-m", type=float, default=10.0)
    parser.add_argument("--horizontal-fov-rad", type=float, default=0.154997710698)
    parser.add_argument("--image-width-px", type=int, default=1280)
    parser.add_argument("--image-height-px", type=int, default=720)
    parser.add_argument("--camera-update-rate-hz", type=float, default=30.0)
    parser.add_argument("--camera-clip-near-m", type=float, default=0.1)
    parser.add_argument("--camera-clip-far-m", type=float, default=4000.0)
    parser.add_argument("--plot-publish-hz", type=float, default=30.0)
    parser.add_argument("--actual-distance-read-hz", type=float, default=15.0)
    parser.add_argument("--yolo-model", default=validation.DEFAULT_YOLO_MODEL)
    parser.add_argument("--yolo-class", default="UAV")
    parser.add_argument("--yolo-confidence", type=float, default=0.10)
    parser.add_argument("--yolo-device", default="cuda:0")
    parser.add_argument("--yolo-target-hz", type=float, default=40.0)
    parser.add_argument("--yolo-imgsz", type=int, default=1280)
    parser.add_argument("--yolo-python", default=sys.executable)
    parser.add_argument(
        "--yolo-pythonpath",
        default="",
    )
    parser.add_argument("--target-model-name", default="x500_15m")
    parser.add_argument("--target-yaw-deg", type=float, default=-90.0)
    parser.add_argument("--forward-min-m", type=float, default=260.0)
    parser.add_argument("--forward-max-m", type=float, default=1500.0)
    parser.add_argument("--altitude-min-m", type=float, default=5.8)
    parser.add_argument("--altitude-max-m", type=float, default=13.5)
    parser.add_argument("--overlap-margin", type=float, default=0.55)
    return parser.parse_args()


def finite_tuple(values):
    return len(values) > 0 and all(validation.finite(value) for value in values)


def launch_args(args, first_point):
    return [
        "ros2",
        "launch",
        "stereo_ground_sim",
        "stereo_ground_sim.launch.py",
        "gz_gui:=false",
        "launch_rviz:=false",
        "mission_profile:=manual",
        "fly_target_in_gazebo:=false",
        "actual_distance_source:=gazebo",
        f"target_spawn_x_m:={first_point[0]}",
        f"target_spawn_y_m:={first_point[1]}",
        f"target_spawn_z_m:={first_point[2]}",
        f"target_spawn_yaw_deg:={args.target_yaw_deg}",
        f"target_model_name:={args.target_model_name}",
        f"baseline_m:={args.baseline_m}",
        f"camera_height_m:={args.camera_height_m}",
        f"horizontal_fov_rad:={args.horizontal_fov_rad}",
        f"camera_image_width_px:={args.image_width_px}",
        f"camera_image_height_px:={args.image_height_px}",
        f"camera_update_rate_hz:={args.camera_update_rate_hz}",
        f"camera_clip_near_m:={args.camera_clip_near_m}",
        f"camera_clip_far_m:={args.camera_clip_far_m}",
        f"actual_distance_read_hz:={args.actual_distance_read_hz}",
        f"plot_publish_hz:={args.plot_publish_hz}",
        f"yolo_model_path:={args.yolo_model}",
        f"yolo_class_name:={args.yolo_class}",
        f"yolo_confidence:={args.yolo_confidence}",
        f"yolo_device:={args.yolo_device}",
        f"yolo_target_hz:={args.yolo_target_hz}",
        f"yolo_imgsz:={args.yolo_imgsz}",
        f"yolo_python:={args.yolo_python}",
        "overlay_publish_hz:=0.0",
        "enable_bbox_center_adjustment:=false",
        "bbox_center_adjustment_path:=unused",
        "left_roll_error_deg:=0.0",
        "left_pitch_error_deg:=0.0",
        "left_yaw_error_deg:=0.0",
        "right_roll_error_deg:=0.0",
        "right_pitch_error_deg:=0.0",
        "right_yaw_error_deg:=0.0",
    ]


class CalibrationRecorder(Node):
    def __init__(
        self,
        sample_hz,
        *,
        roi_output_dir=None,
        roi_save_stride=0,
        roi_crop_margin=0.25,
        roi_jpeg_quality=90,
        roi_image_format="ppm",
    ):
        super().__init__("origin_bbox_profile_calibration_recorder")
        self.lock = threading.Lock()
        self.latest_gt = None
        self.latest_est = None
        self.latest_det = None
        self.latest_images = {}
        self.image_counts = {"left": 0, "right": 0}
        self.current_trial = None
        self.trial_started = None
        self.rows_by_trial = {}
        self.sample_seq_by_trial = {}
        self.roi_output_dir = Path(roi_output_dir) if roi_output_dir else None
        self.roi_save_stride = max(0, int(roi_save_stride or 0))
        self.roi_crop_margin = max(0.0, float(roi_crop_margin))
        self.roi_jpeg_quality = max(1, min(100, int(roi_jpeg_quality)))
        self.roi_image_format = str(roi_image_format or "ppm").strip().lower()
        self._cv2 = None
        if self.roi_output_dir is not None and self.roi_save_stride > 0:
            if self.roi_image_format in ("jpg", "jpeg"):
                try:
                    import cv2 as cv2_module

                    self._cv2 = cv2_module
                except Exception as exc:
                    self.get_logger().warning(f"ROI JPEG capture unavailable; falling back to PPM crops: {exc}")
                    self.roi_image_format = "ppm"
            self.roi_output_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Float32MultiArray, "/ground_truth_position", self._gt_cb, 10)
        self.create_subscription(Float32MultiArray, "/stereo/estimated_position", self._est_cb, 10)
        self.create_subscription(Float32MultiArray, "/stereo/detections", self._det_cb, 10)
        self.image_subscriptions = []
        if self.roi_output_dir is not None and self.roi_save_stride > 0:
            image_qos = QoSProfile(
                depth=2,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.image_subscriptions.extend(
                [
                    self.create_subscription(
                        ImageMsg,
                        "/left_camera/image",
                        lambda msg: self._image_cb("left", msg),
                        image_qos,
                    ),
                    self.create_subscription(
                        ImageMsg,
                        "/right_camera/image",
                        lambda msg: self._image_cb("right", msg),
                        image_qos,
                    ),
                ]
            )
        self.create_timer(1.0 / max(1.0, float(sample_hz)), self._sample)

    def _gt_cb(self, msg):
        values = list(msg.data[:3])
        if finite_tuple(values):
            with self.lock:
                self.latest_gt = tuple(float(value) for value in values)

    def _est_cb(self, msg):
        values = list(msg.data[:3])
        if finite_tuple(values):
            with self.lock:
                self.latest_est = tuple(float(value) for value in values)

    def _det_cb(self, msg):
        values = list(msg.data[:16])
        if len(values) == 16 and values[0] > 0.5 and values[8] > 0.5 and finite_tuple(values[1:8] + values[9:16]):
            with self.lock:
                self.latest_det = tuple(float(value) for value in values)

    def _image_cb(self, side, msg):
        image = self._msg_to_bgr(msg)
        if image is None:
            return
        with self.lock:
            self.latest_images[side] = image
            self.image_counts[side] = self.image_counts.get(side, 0) + 1

    def _msg_to_bgr(self, msg):
        encoding = str(msg.encoding).lower()
        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            self.get_logger().warning(f"Unsupported ROI image encoding: {msg.encoding}")
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            rows = raw.reshape((msg.height, msg.step))
            pixels = rows[:, : msg.width * channels].reshape((msg.height, msg.width, channels))
        except ValueError:
            self.get_logger().warning("ROI image buffer shape did not match metadata; frame skipped.")
            return None
        if encoding == "bgr8":
            return pixels.copy()
        if encoding == "rgb8":
            return pixels[:, :, ::-1].copy()
        if encoding == "bgra8":
            return pixels[:, :, :3].copy()
        if encoding == "rgba8":
            return pixels[:, :, 2::-1].copy()
        return np.repeat(pixels, 3, axis=2).copy()

    def begin_trial(self, trial_id):
        with self.lock:
            self.current_trial = int(trial_id)
            self.trial_started = time.monotonic()
            self.rows_by_trial.setdefault(int(trial_id), [])
            self.sample_seq_by_trial.setdefault(int(trial_id), 0)

    def end_trial(self):
        with self.lock:
            self.current_trial = None
            self.trial_started = None

    def _sample(self):
        with self.lock:
            if self.current_trial is None or self.trial_started is None:
                return
            if self.latest_gt is None or self.latest_est is None or self.latest_det is None:
                return
            elapsed = time.monotonic() - self.trial_started
            gt = self.latest_gt
            est = self.latest_est
            det = self.latest_det
            self.sample_seq_by_trial[self.current_trial] = self.sample_seq_by_trial.get(self.current_trial, 0) + 1
            sample_seq = self.sample_seq_by_trial[self.current_trial]
            row = {
                "trial": self.current_trial,
                "trial_t_s": elapsed,
                "sample_seq": sample_seq,
                "gt_forward_m": gt[0],
                "gt_right_m": gt[1],
                "gt_down_m": gt[2],
                "estimated_forward_m": est[0],
                "estimated_right_m": est[1],
                "estimated_down_m": est[2],
                "left_x1_px": det[1],
                "left_y1_px": det[2],
                "left_x2_px": det[3],
                "left_y2_px": det[4],
                "left_confidence": det[5],
                "left_cx_px": det[6],
                "left_cy_px": det[7],
                "right_x1_px": det[9],
                "right_y1_px": det[10],
                "right_x2_px": det[11],
                "right_y2_px": det[12],
                "right_confidence": det[13],
                "right_cx_px": det[14],
                "right_cy_px": det[15],
                "left_image_count": self.image_counts.get("left", 0),
                "right_image_count": self.image_counts.get("right", 0),
            }
            row.update(self._save_roi_images_locked(row))
            self.rows_by_trial[self.current_trial].append(row)

    def _save_roi_images_locked(self, row):
        if self.roi_output_dir is None or self.roi_save_stride <= 0:
            return {}
        sample_seq = int(row["sample_seq"])
        if sample_seq % self.roi_save_stride != 0:
            return {}
        saved = {}
        trial_id = int(row["trial"])
        for side in ("left", "right"):
            image = self.latest_images.get(side)
            if image is None:
                continue
            prefix = side
            bbox = (
                float(row[f"{prefix}_x1_px"]),
                float(row[f"{prefix}_y1_px"]),
                float(row[f"{prefix}_x2_px"]),
                float(row[f"{prefix}_y2_px"]),
            )
            crop = self._crop_bbox(image, bbox)
            if crop is None:
                continue
            suffix = ".jpg" if self._cv2 is not None and self.roi_image_format in ("jpg", "jpeg") else ".ppm"
            path = self.roi_output_dir / f"trial_{trial_id:02d}" / f"{side}_{sample_seq:06d}{suffix}"
            path.parent.mkdir(parents=True, exist_ok=True)
            if self._cv2 is not None and self.roi_image_format in ("jpg", "jpeg"):
                ok = self._cv2.imwrite(str(path), crop, [int(self._cv2.IMWRITE_JPEG_QUALITY), self.roi_jpeg_quality])
            else:
                ok = self._write_ppm(path, crop)
            if ok:
                saved[f"{side}_roi_path"] = str(path)
        return saved

    def _write_ppm(self, path, bgr_image):
        if bgr_image.ndim != 3 or bgr_image.shape[2] < 3:
            return False
        rgb = bgr_image[:, :, 2::-1]
        header = f"P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n".encode("ascii")
        with Path(path).open("wb") as handle:
            handle.write(header)
            handle.write(np.ascontiguousarray(rgb).tobytes())
        return True

    def _crop_bbox(self, image, bbox):
        height, width = image.shape[:2]
        x1, y1, x2, y2 = bbox
        box_width = x2 - x1
        box_height = y2 - y1
        if box_width <= 1.0 or box_height <= 1.0:
            return None
        pad_x = box_width * self.roi_crop_margin
        pad_y = box_height * self.roi_crop_margin
        x1_i = max(0, int(math.floor(x1 - pad_x)))
        y1_i = max(0, int(math.floor(y1 - pad_y)))
        x2_i = min(width, int(math.ceil(x2 + pad_x)))
        y2_i = min(height, int(math.ceil(y2 + pad_y)))
        if x2_i <= x1_i or y2_i <= y1_i:
            return None
        return image[y1_i:y2_i, x1_i:x2_i].copy()

    def snapshot(self):
        with self.lock:
            return {trial: list(rows) for trial, rows in self.rows_by_trial.items()}


def row_feature_base(row, width, height):
    left_x1 = float(row["left_x1_px"])
    left_y1 = float(row["left_y1_px"])
    left_x2 = float(row["left_x2_px"])
    left_y2 = float(row["left_y2_px"])
    right_x1 = float(row["right_x1_px"])
    right_y1 = float(row["right_y1_px"])
    right_x2 = float(row["right_x2_px"])
    right_y2 = float(row["right_y2_px"])
    left_cx = float(row["left_cx_px"])
    left_cy = float(row["left_cy_px"])
    right_cx = float(row["right_cx_px"])
    right_cy = float(row["right_cy_px"])
    left_width = max(0.0, left_x2 - left_x1)
    left_height = max(0.0, left_y2 - left_y1)
    right_width = max(0.0, right_x2 - right_x1)
    right_height = max(0.0, right_y2 - right_y1)
    estimated_forward = float(row["estimated_forward_m"])
    estimated_right = float(row["estimated_right_m"])
    estimated_down = float(row["estimated_down_m"])
    estimated_range = math.sqrt(
        estimated_forward * estimated_forward
        + estimated_right * estimated_right
        + estimated_down * estimated_down
    )
    disparity = left_cx - right_cx
    abs_disparity = abs(disparity)
    return {
        "bias": 1.0,
        "estimated_forward_m": estimated_forward,
        "estimated_right_m": estimated_right,
        "estimated_down_m": estimated_down,
        "estimated_range_m": estimated_range,
        "left_x1_norm": left_x1 / width,
        "left_y1_norm": left_y1 / height,
        "left_x2_norm": left_x2 / width,
        "left_y2_norm": left_y2 / height,
        "right_x1_norm": right_x1 / width,
        "right_y1_norm": right_y1 / height,
        "right_x2_norm": right_x2 / width,
        "right_y2_norm": right_y2 / height,
        "left_cx_norm": left_cx / width,
        "left_cy_norm": left_cy / height,
        "right_cx_norm": right_cx / width,
        "right_cy_norm": right_cy / height,
        "center_x_norm": (((left_cx + right_cx) * 0.5) / width) - 0.5,
        "center_y_norm": (((left_cy + right_cy) * 0.5) / height) - 0.5,
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
        "left_confidence": float(row["left_confidence"]),
        "right_confidence": float(row["right_confidence"]),
        "confidence_min": min(float(row["left_confidence"]), float(row["right_confidence"])),
    }


def feature_value(name, base):
    if name in base:
        return float(base[name])
    if name.endswith("^2"):
        return feature_value(name[:-2], base) ** 2
    if "*" in name:
        value = 1.0
        for part in name.split("*"):
            value *= feature_value(part, base)
        return float(value)
    raise KeyError(name)


def feature_matrix(rows, feature_names, width, height):
    return np.array(
        [
            [feature_value(name, row_feature_base(row, width, height)) for name in feature_names]
            for row in rows
        ],
        dtype=float,
    )


def target_matrix(rows):
    return np.array(
        [
            [float(row["gt_forward_m"]), float(row["gt_right_m"]), float(row["gt_down_m"])]
            for row in rows
        ],
        dtype=float,
    )


def estimated_matrix(rows):
    return np.array(
        [
            [
                float(row["estimated_forward_m"]),
                float(row["estimated_right_m"]),
                float(row["estimated_down_m"]),
            ]
            for row in rows
        ],
        dtype=float,
    )


def fit_model(rows, feature_names, width, height, ridge_lambda):
    x_raw = feature_matrix(rows, feature_names, width, height)
    y = target_matrix(rows)
    mean = np.mean(x_raw, axis=0)
    scale = np.std(x_raw, axis=0)
    for index, name in enumerate(feature_names):
        if name == "bias":
            mean[index] = 0.0
            scale[index] = 1.0
    scale = np.where(np.abs(scale) < 1.0e-12, 1.0, scale)
    x = (x_raw - mean) / scale
    regularizer = ridge_lambda * np.eye(x.shape[1])
    for index, name in enumerate(feature_names):
        if name == "bias":
            regularizer[index, index] = 0.0
    coefficients = np.linalg.solve(x.T @ x + regularizer, x.T @ y).T
    return {
        "feature_mean": mean,
        "feature_scale": scale,
        "coefficients": coefficients,
    }


def predict(rows, feature_names, model, blend, width, height):
    x_raw = feature_matrix(rows, feature_names, width, height)
    x = (x_raw - model["feature_mean"]) / model["feature_scale"]
    predicted = x @ model["coefficients"].T
    estimated = estimated_matrix(rows)
    return estimated + np.array(blend, dtype=float) * (predicted - estimated)


def mae_improvement(rows, corrected):
    estimated = estimated_matrix(rows)
    target = target_matrix(rows)
    estimated_mae = np.mean(np.abs(estimated - target), axis=0)
    corrected_mae = np.mean(np.abs(corrected - target), axis=0)
    return estimated_mae - corrected_mae, estimated_mae, corrected_mae


def candidate_feature_sets():
    bbox_linear = [
        "bias",
        "estimated_forward_m",
        "estimated_right_m",
        "estimated_down_m",
        "estimated_range_m",
        "center_x_norm",
        "center_y_norm",
        "signed_disparity_norm",
        "disparity_norm",
        "width_over_disparity",
        "left_cx_norm",
        "left_cy_norm",
        "right_cx_norm",
        "right_cy_norm",
        "avg_width_norm",
        "avg_height_norm",
        "width_ratio",
        "height_ratio",
        "confidence_min",
    ]
    bbox_poly = bbox_linear + [
        "estimated_forward_m^2",
        "estimated_right_m^2",
        "estimated_down_m^2",
        "center_x_norm^2",
        "center_y_norm^2",
        "disparity_norm^2",
        "width_over_disparity^2",
        "estimated_forward_m*center_x_norm",
        "estimated_forward_m*center_y_norm",
        "estimated_forward_m*avg_width_norm",
        "estimated_forward_m*avg_height_norm",
        "estimated_right_m*center_x_norm",
        "estimated_down_m*center_y_norm",
        "center_x_norm*width_over_disparity",
        "center_y_norm*width_over_disparity",
    ]
    compact_poly = [
        "bias",
        "estimated_forward_m",
        "estimated_right_m",
        "estimated_down_m",
        "center_x_norm",
        "center_y_norm",
        "width_over_disparity",
        "avg_width_norm",
        "avg_height_norm",
        "estimated_forward_m^2",
        "estimated_forward_m*center_x_norm",
        "estimated_forward_m*center_y_norm",
        "estimated_forward_m*avg_width_norm",
        "estimated_forward_m*avg_height_norm",
    ]
    return {
        "estimated_affine": ["bias", "estimated_forward_m", "estimated_right_m", "estimated_down_m"],
        "origin_bbox_linear": bbox_linear,
        "origin_bbox_poly": bbox_poly,
        "origin_bbox_compact_poly": compact_poly,
    }


def choose_blend(fold_predictions):
    alpha_grid = [0.05, 0.1, 0.2, 0.35, 0.5, 0.75, 1.0]
    chosen = []
    axis_scores = []
    for axis in range(3):
        best_alpha = 0.05
        best_score = -float("inf")
        for alpha in alpha_grid:
            fold_improvements = []
            for rows, predicted in fold_predictions:
                estimated = estimated_matrix(rows)[:, axis]
                target = target_matrix(rows)[:, axis]
                corrected = estimated + alpha * (predicted[:, axis] - estimated)
                estimated_mae = float(np.mean(np.abs(estimated - target)))
                corrected_mae = float(np.mean(np.abs(corrected - target)))
                fold_improvements.append(estimated_mae - corrected_mae)
            score = min(fold_improvements)
            if score > best_score:
                best_score = score
                best_alpha = alpha
        chosen.append(best_alpha)
        axis_scores.append(best_score)
    return chosen, axis_scores


def cross_validate(rows_by_trial, feature_names, ridge_lambda, args):
    trial_ids = sorted(rows_by_trial)
    fold_predictions = []
    fold_rows = []
    for trial_id in trial_ids:
        train_rows = [row for tid, rows in rows_by_trial.items() if tid != trial_id for row in rows]
        test_rows = rows_by_trial[trial_id]
        model = fit_model(train_rows, feature_names, args.image_width_px, args.image_height_px, ridge_lambda)
        predicted = predict(
            test_rows,
            feature_names,
            model,
            [1.0, 1.0, 1.0],
            args.image_width_px,
            args.image_height_px,
        )
        fold_predictions.append((test_rows, predicted))
        fold_rows.append(test_rows)
    blend, axis_scores = choose_blend(fold_predictions)
    improvements = []
    estimated_maes = []
    corrected_maes = []
    for rows, predicted in fold_predictions:
        estimated = estimated_matrix(rows)
        corrected = estimated + np.array(blend) * (predicted - estimated)
        improvement, estimated_mae, corrected_mae = mae_improvement(rows, corrected)
        improvements.append(improvement)
        estimated_maes.append(estimated_mae)
        corrected_maes.append(corrected_mae)
    improvements = np.array(improvements)
    return {
        "blend": blend,
        "axis_min_improvement": np.min(improvements, axis=0),
        "axis_mean_improvement": np.mean(improvements, axis=0),
        "axis_scores": axis_scores,
        "fold_improvements": improvements,
        "estimated_mae_mean": np.mean(np.array(estimated_maes), axis=0),
        "corrected_mae_mean": np.mean(np.array(corrected_maes), axis=0),
        "score": float(np.min(improvements)),
    }


def write_csv(path, rows):
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def collect_calibration(args, output_dir):
    rng = validation.random.Random(args.seed)
    trajectories = [validation.build_random_trajectory(rng, args) for _ in range(args.calibration_trials)]
    first_point = trajectories[0][0]
    if not all(validation.point_in_common_fov(*point, args) for path in trajectories for point in path):
        raise RuntimeError("generated calibration trajectory outside common FOV")

    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp/ros2-origin-bbox-profile-log")
    env.setdefault("GZ_CONFIG_HOME", "/tmp/gz-origin-bbox-profile-config")
    root = Path(__file__).resolve().parents[1]
    model_path = str(root / "install/stereo_ground_sim/share/stereo_ground_sim/models")
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        item for item in [model_path, env.get("GZ_SIM_RESOURCE_PATH", "")] if item
    )
    if args.yolo_pythonpath and Path(args.yolo_pythonpath).is_dir():
        env["PYTHONPATH"] = os.pathsep.join(
            item for item in [args.yolo_pythonpath, env.get("PYTHONPATH", "")] if item
        )

    validation.kill_matching_processes("stereo_ground_drone_attitude_error.sdf")
    validation.kill_matching_processes("ros2 launch stereo_ground_sim stereo_ground_sim.launch.py")
    sim, sim_log = validation.run_process(launch_args(args, first_point), output_dir / "calibration_simulation.log", env)

    rclpy.init(args=None)
    recorder = CalibrationRecorder(args.sample_hz)
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()

    try:
        time.sleep(args.warmup_s)
        if sim.poll() is not None:
            raise RuntimeError(f"simulation exited during warmup with code {sim.returncode}")
        period = 1.0 / max(0.1, args.command_hz)
        for trial_id, trajectory in enumerate(trajectories, start=1):
            print(f"calibration trial {trial_id}/{args.calibration_trials}: waypoints={len(trajectory)}", flush=True)
            validation.command_target_pose(args.target_model_name, trajectory[0], args.target_yaw_deg)
            time.sleep(args.settle_s)
            recorder.begin_trial(trial_id)
            started = time.monotonic()
            while True:
                elapsed = time.monotonic() - started
                if elapsed >= args.trial_duration_s:
                    break
                if sim.poll() is not None:
                    raise RuntimeError(f"simulation exited early with code {sim.returncode}")
                point = validation.interpolate(trajectory, elapsed / args.trial_duration_s)
                validation.command_target_pose(args.target_model_name, point, args.target_yaw_deg)
                time.sleep(period)
            validation.command_target_pose(args.target_model_name, trajectory[-1], args.target_yaw_deg)
            recorder.end_trial()
    finally:
        recorder.end_trial()
        snapshot = recorder.snapshot()
        recorder.destroy_node()
        rclpy.shutdown()
        validation.stop_process(sim)
        validation.kill_matching_processes("stereo_ground_drone_attitude_error.sdf")
        sim_log.close()

    filtered = {}
    for trial_id, rows in snapshot.items():
        usable = [row for row in rows if float(row["trial_t_s"]) >= args.analysis_skip_s]
        if len(usable) >= args.min_samples_per_trial:
            filtered[trial_id] = usable
    if len(filtered) < 3:
        raise RuntimeError(f"need at least 3 usable calibration trials, got {len(filtered)}")
    all_rows = [row for rows in filtered.values() for row in rows]
    write_csv(output_dir / "calibration_samples.csv", all_rows)
    return filtered, trajectories


def load_calibration_csv(path, args, output_dir):
    rows_by_trial = {}
    with Path(path).open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if float(row["trial_t_s"]) < args.analysis_skip_s:
                continue
            trial_id = int(row["trial"])
            rows_by_trial.setdefault(trial_id, []).append(row)
    filtered = {
        trial_id: rows
        for trial_id, rows in rows_by_trial.items()
        if len(rows) >= args.min_samples_per_trial
    }
    if len(filtered) < 3:
        raise RuntimeError(f"need at least 3 usable calibration trials, got {len(filtered)}")
    all_rows = [row for rows in filtered.values() for row in rows]
    write_csv(output_dir / "calibration_samples.csv", all_rows)
    return filtered, []


def main():
    args = parse_args()
    if args.calibration_trials < 3:
        raise SystemExit("--calibration-trials must be at least 3")
    root = Path(__file__).resolve().parents[1]
    output_dir = (
        Path(args.output_dir)
        if args.output_dir
        else root / "reports" / f"origin_bbox_output_profile_{dt.datetime.now():%Y%m%d_%H%M%S}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.calibration_csv:
        rows_by_trial, trajectories = load_calibration_csv(args.calibration_csv, args, output_dir)
    else:
        rows_by_trial, trajectories = collect_calibration(args, output_dir)
    all_rows = [row for rows in rows_by_trial.values() for row in rows]
    candidates = []
    for candidate_name, feature_names in candidate_feature_sets().items():
        for ridge_lambda in [0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0]:
            try:
                cv = cross_validate(rows_by_trial, feature_names, ridge_lambda, args)
            except np.linalg.LinAlgError:
                continue
            candidates.append(
                {
                    "candidate": candidate_name,
                    "feature_names": feature_names,
                    "ridge_lambda": ridge_lambda,
                    **cv,
                }
            )
    if not candidates:
        raise RuntimeError("no trainable candidates")
    candidates.sort(
        key=lambda item: (
            float(item["score"]),
            float(np.mean(item["axis_mean_improvement"])),
            -len(item["feature_names"]),
        ),
        reverse=True,
    )
    best = candidates[0]
    axis_best = [
        max(candidates, key=lambda item, axis=axis: float(item["axis_min_improvement"][axis]))
        for axis in range(3)
    ]
    union_feature_names = []
    for item in axis_best:
        for name in item["feature_names"]:
            if name not in union_feature_names:
                union_feature_names.append(name)
    union_x = feature_matrix(all_rows, union_feature_names, args.image_width_px, args.image_height_px)
    union_mean = np.mean(union_x, axis=0)
    union_scale = np.std(union_x, axis=0)
    for index, name in enumerate(union_feature_names):
        if name == "bias":
            union_mean[index] = 0.0
            union_scale[index] = 1.0
    union_scale = np.where(np.abs(union_scale) < 1.0e-12, 1.0, union_scale)
    coefficients = np.zeros((3, len(union_feature_names)), dtype=float)
    blend = []
    for axis, item in enumerate(axis_best):
        model = fit_model(
            all_rows,
            item["feature_names"],
            args.image_width_px,
            args.image_height_px,
            item["ridge_lambda"],
        )
        for source_index, name in enumerate(item["feature_names"]):
            target_index = union_feature_names.index(name)
            coefficients[axis, target_index] = model["coefficients"][axis, source_index]
        blend.append(item["blend"][axis])

    profile = {
        "mode": "origin_bbox_output_ridge",
        "created_at": dt.datetime.now().isoformat(timespec="seconds"),
        "candidate": "axis_selected_origin_bbox_output_ridge",
        "axis_candidates": [
            {
                "axis": axis_name,
                "candidate": item["candidate"],
                "ridge_lambda": item["ridge_lambda"],
                "feature_count": len(item["feature_names"]),
                "blend": item["blend"][axis],
                "axis_min_improvement_m": item["axis_min_improvement"][axis],
                "axis_mean_improvement_m": item["axis_mean_improvement"][axis],
            }
            for axis, axis_name, item in zip(range(3), ["forward", "right", "down"], axis_best)
        ],
        "axis_contract": "stereo_center_frd:forward_right_down",
        "output_axes": ["forward_m", "right_m", "down_m"],
        "input_contract": "raw Origin-BBOX detections plus raw stereo estimated FRD",
        "image_width_px": args.image_width_px,
        "image_height_px": args.image_height_px,
        "feature_names": union_feature_names,
        "feature_mean": union_mean.tolist(),
        "feature_scale": union_scale.tolist(),
        "coefficients": coefficients.tolist(),
        "blend": blend,
        "ridge_lambda": [item["ridge_lambda"] for item in axis_best],
        "cross_validation": {
            "folds": len(rows_by_trial),
            "samples": len(all_rows),
            "overall_best_candidate": best["candidate"],
            "overall_best_axis_min_improvement_m": best["axis_min_improvement"].tolist(),
            "axis_selected_min_improvement_m": [
                axis_best[axis]["axis_min_improvement"][axis] for axis in range(3)
            ],
            "axis_selected_mean_improvement_m": [
                axis_best[axis]["axis_mean_improvement"][axis] for axis in range(3)
            ],
            "score_m": min(axis_best[axis]["axis_min_improvement"][axis] for axis in range(3)),
        },
    }
    profile_path = output_dir / "origin_bbox_output_profile.json"
    profile_path.write_text(json.dumps(profile, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    candidate_rows = []
    for item in candidates:
        candidate_rows.append(
            {
                "candidate": item["candidate"],
                "ridge_lambda": item["ridge_lambda"],
                "feature_count": len(item["feature_names"]),
                "blend_forward": item["blend"][0],
                "blend_right": item["blend"][1],
                "blend_down": item["blend"][2],
                "min_improvement_forward_m": item["axis_min_improvement"][0],
                "min_improvement_right_m": item["axis_min_improvement"][1],
                "min_improvement_down_m": item["axis_min_improvement"][2],
                "mean_improvement_forward_m": item["axis_mean_improvement"][0],
                "mean_improvement_right_m": item["axis_mean_improvement"][1],
                "mean_improvement_down_m": item["axis_mean_improvement"][2],
                "score_m": item["score"],
            }
        )
    write_csv(output_dir / "candidate_summary.csv", candidate_rows)
    metadata = {
        "created_at": dt.datetime.now().isoformat(timespec="seconds"),
        "profile_path": str(profile_path),
        "calibration_trials": len(rows_by_trial),
        "calibration_samples": len(all_rows),
        "calibration_trajectories": trajectories,
        "best_candidate": {
            "candidate": best["candidate"],
            "ridge_lambda": best["ridge_lambda"],
            "feature_count": len(best["feature_names"]),
            "blend": best["blend"],
            "axis_min_improvement_m": best["axis_min_improvement"].tolist(),
            "axis_mean_improvement_m": best["axis_mean_improvement"].tolist(),
            "score_m": best["score"],
        },
        "axis_selected_candidate": {
            "blend": blend,
            "feature_count": len(union_feature_names),
            "axis_min_improvement_m": [
                axis_best[axis]["axis_min_improvement"][axis] for axis in range(3)
            ],
            "axis_mean_improvement_m": [
                axis_best[axis]["axis_mean_improvement"][axis] for axis in range(3)
            ],
            "axes": profile["axis_candidates"],
        },
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Profile: {profile_path}")
    print(f"Candidate summary: {output_dir / 'candidate_summary.csv'}")
    print(f"Best candidate: {best['candidate']} lambda={best['ridge_lambda']} blend={best['blend']}")
    print(f"CV min improvement FRD: {best['axis_min_improvement']}")
    print(f"Axis-selected blend: {blend}")
    print(
        "Axis-selected CV min improvement FRD: "
        f"{[axis_best[axis]['axis_min_improvement'][axis] for axis in range(3)]}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        try:
            os.killpg(os.getpid(), signal.SIGINT)
        except Exception:
            pass
        raise SystemExit(130)
