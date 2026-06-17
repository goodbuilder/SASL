#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import json
import math
import os
import random
import signal
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


AXIS_CONTRACT = "stereo_center_frd:forward_right_down"
DEFAULT_YOLO_MODEL = (
    "/home/moontaehyung/영상감시시스템/ref/wiris_stereo (copy)/wiris_stereo/"
    "yolov8n_uav.pt"
)
DEFAULT_PROFILE = (
    "/home/moontaehyung/영상감시시스템/ref/Stereo_Camera_3D_Position_DKL/"
    "Cent_Bbox_Adj_Mtrx.csv"
)
DEFAULT_PROFILE_SOURCE = (
    "/home/moontaehyung/영상감시시스템/ref/Stereo_Camera_3D_Position_DKL/"
    "Gazebo_Sim/time (csv)/8x/pitch0_yaw0.csv"
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Validate a bbox-center adjustment profile with no attitude error using "
            "random Gazebo target trajectories."
        )
    )
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260609)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--profile-path", default=DEFAULT_PROFILE)
    parser.add_argument("--profile-source-csv", default=DEFAULT_PROFILE_SOURCE)
    parser.add_argument("--warmup-s", type=float, default=12.0)
    parser.add_argument("--settle-s", type=float, default=3.0)
    parser.add_argument("--trial-duration-s", type=float, default=24.0)
    parser.add_argument("--analysis-skip-s", type=float, default=3.0)
    parser.add_argument("--command-hz", type=float, default=2.0)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument(
        "--nonregression-epsilon-m",
        type=float,
        default=0.0,
        help=(
            "Allowed MAE regression tolerance for runtime non-regression gates. "
            "Use a small value only to absorb live ROS/Gazebo sampling jitter."
        ),
    )
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
    parser.add_argument("--yolo-model", default=DEFAULT_YOLO_MODEL)
    parser.add_argument("--yolo-class", default="UAV")
    parser.add_argument("--yolo-confidence", type=float, default=0.10)
    parser.add_argument("--yolo-device", default="cuda:0")
    parser.add_argument("--yolo-target-hz", type=float, default=40.0)
    parser.add_argument("--yolo-imgsz", type=int, default=1280)
    parser.add_argument("--yolo-python", default="/home/moontaehyung/Capstone/.venv/bin/python")
    parser.add_argument(
        "--yolo-pythonpath",
        default="/home/moontaehyung/Capstone/.venv/lib/python3.10/site-packages",
    )
    parser.add_argument("--target-model-name", default="x500_15m")
    parser.add_argument("--target-yaw-deg", type=float, default=-90.0)
    parser.add_argument("--left-roll-error-deg", type=float, default=0.0)
    parser.add_argument("--left-pitch-error-deg", type=float, default=0.0)
    parser.add_argument("--left-yaw-error-deg", type=float, default=0.0)
    parser.add_argument("--right-roll-error-deg", type=float, default=0.0)
    parser.add_argument("--right-pitch-error-deg", type=float, default=0.0)
    parser.add_argument("--right-yaw-error-deg", type=float, default=0.0)
    parser.add_argument("--forward-min-m", type=float, default=260.0)
    parser.add_argument("--forward-max-m", type=float, default=1500.0)
    parser.add_argument("--altitude-min-m", type=float, default=5.8)
    parser.add_argument("--altitude-max-m", type=float, default=13.5)
    parser.add_argument(
        "--overlap-margin",
        type=float,
        default=0.55,
        help="Use this fraction of the computed common FOV width/height for random waypoints.",
    )
    return parser.parse_args()


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number)


def percentile(values, p):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * (p / 100.0)
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return ordered[int(index)]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def write_csv(path, fieldnames, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run_process(command, log_path, env):
    log = Path(log_path).open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        preexec_fn=os.setsid,
    )
    return process, log


def stop_process(process, timeout_s=12.0):
    if process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout_s)
            return
        except subprocess.TimeoutExpired:
            continue


def kill_matching_processes(pattern, timeout_s=5.0):
    result = subprocess.run(
        ["pgrep", "-af", pattern],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    pids = []
    for line in result.stdout.splitlines():
        parts = line.split(maxsplit=1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        command = parts[1] if len(parts) > 1 else ""
        if pid == os.getpid() or "pgrep -af" in command or "codex-linux-sandbox" in command:
            continue
        pids.append(pid)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + timeout_s
    while pids and time.monotonic() < deadline:
        remaining = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                remaining.append(pid)
            except ProcessLookupError:
                pass
        pids = remaining
        if pids:
            time.sleep(0.2)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def profile_overlap_summary(path):
    path = Path(path)
    rows = []
    if not path.exists():
        return {"exists": False, "path": str(path)}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                left_u = float(row["cam1_cx"])
                left_v = float(row["cam1_cy"])
                right_u = float(row["cam2_cx"])
                right_v = float(row["cam2_cy"])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append((left_u, left_v, right_u, right_v))
    if not rows:
        return {"exists": True, "path": str(path), "samples": 0}
    inside = [
        item
        for item in rows
        if 0.0 <= item[0] <= 1920.0
        and 0.0 <= item[1] <= 1080.0
        and 0.0 <= item[2] <= 1920.0
        and 0.0 <= item[3] <= 1080.0
    ]
    return {
        "exists": True,
        "path": str(path),
        "samples": len(rows),
        "inside_1920x1080_samples": len(inside),
        "implicit_common_fov_by_dual_detection": len(inside) == len(rows),
        "left_u_min": min(item[0] for item in rows),
        "left_u_max": max(item[0] for item in rows),
        "left_v_min": min(item[1] for item in rows),
        "left_v_max": max(item[1] for item in rows),
        "right_u_min": min(item[2] for item in rows),
        "right_u_max": max(item[2] for item in rows),
        "right_v_min": min(item[3] for item in rows),
        "right_v_max": max(item[3] for item in rows),
    }


def vertical_fov_rad(horizontal_fov_rad, width, height):
    return 2.0 * math.atan(math.tan(horizontal_fov_rad * 0.5) * (float(height) / float(width)))


def overlap_bounds(forward_m, args):
    half_h = math.tan(args.horizontal_fov_rad * 0.5) * forward_m
    left_min = -half_h - args.baseline_m * 0.5
    left_max = half_h - args.baseline_m * 0.5
    right_min = -half_h + args.baseline_m * 0.5
    right_max = half_h + args.baseline_m * 0.5
    common_min = max(left_min, right_min)
    common_max = min(left_max, right_max)
    half_v = math.tan(vertical_fov_rad(args.horizontal_fov_rad, args.image_width_px, args.image_height_px) * 0.5) * forward_m
    z_min = args.camera_height_m - half_v
    z_max = args.camera_height_m + half_v
    return common_min, common_max, z_min, z_max


def point_in_common_fov(x_m, y_m, z_m, args):
    x_min, x_max, z_min, z_max = overlap_bounds(y_m, args)
    return x_min <= x_m <= x_max and z_min <= z_m <= z_max


def random_waypoint(rng, args):
    for _ in range(100):
        forward = rng.uniform(args.forward_min_m, args.forward_max_m)
        x_min, x_max, z_min, z_max = overlap_bounds(forward, args)
        if x_min >= x_max or z_min >= z_max:
            continue
        center_x = (x_min + x_max) * 0.5
        half_x = (x_max - x_min) * 0.5 * args.overlap_margin
        x = rng.uniform(center_x - half_x, center_x + half_x)
        z_low = max(args.altitude_min_m, args.camera_height_m - (z_max - z_min) * 0.25)
        z_high = min(args.altitude_max_m, args.camera_height_m + (z_max - z_min) * 0.25)
        if z_low > z_high:
            z_low = max(args.altitude_min_m, z_min)
            z_high = min(args.altitude_max_m, z_max)
        if z_low > z_high:
            continue
        z = rng.uniform(z_low, z_high)
        if point_in_common_fov(x, forward, z, args):
            return x, forward, z
    raise RuntimeError("failed to sample a waypoint inside the common FOV")


def build_random_trajectory(rng, args):
    waypoint_count = rng.randint(4, 6)
    points = [random_waypoint(rng, args) for _ in range(waypoint_count)]
    points.sort(key=lambda item: item[1], reverse=True)
    return points


def interpolate(points, ratio):
    if ratio <= 0.0:
        return points[0]
    if ratio >= 1.0:
        return points[-1]
    scaled = ratio * (len(points) - 1)
    index = int(math.floor(scaled))
    local = scaled - index
    start = points[index]
    end = points[index + 1]
    return tuple(start[i] + (end[i] - start[i]) * local for i in range(3))


def command_target_pose(model_name, point, yaw_deg):
    gazebo_cli = shutil.which("gz") or shutil.which("ign")
    if gazebo_cli is None:
        raise FileNotFoundError("neither 'gz' nor 'ign' Gazebo CLI was found in PATH")
    cli_name = Path(gazebo_cli).name
    request_flag = "-r" if cli_name == "ign" else "--req"
    pose_type = "ign_msgs.Pose" if cli_name == "ign" else "gz.msgs.Pose"
    bool_type = "ign_msgs.Boolean" if cli_name == "ign" else "gz.msgs.Boolean"
    x_m, y_m, z_m = point
    yaw = math.radians(yaw_deg)
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    pose_msg = (
        f'name: "{model_name}", '
        f"position: {{x: {x_m:.6f}, y: {y_m:.6f}, z: {z_m:.6f}}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.9f}, w: {qw:.9f}}}"
    )
    completed = subprocess.run(
        [
            gazebo_cli,
            "service",
            "-s",
            "/world/stereo_ground_drone/set_pose",
            "--reqtype",
            pose_type,
            "--reptype",
            bool_type,
            "--timeout",
            "1000",
            request_flag,
            pose_msg,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        timeout=5.0,
        check=False,
    )
    if completed.returncode != 0:
        detail = (completed.stderr or "").strip()
        raise RuntimeError(f"{cli_name} set_pose request failed with code {completed.returncode}: {detail}")


class ValidationRecorder(Node):
    def __init__(self, sample_hz):
        super().__init__("bbox_profile_validation_recorder")
        self.lock = threading.Lock()
        self.latest_gt = None
        self.latest_est = None
        self.latest_corr = None
        self.current_trial = None
        self.trial_started = None
        self.rows_by_trial = {}
        self.create_subscription(Float32MultiArray, "/ground_truth_position", self._gt_cb, 10)
        self.create_subscription(Float32MultiArray, "/stereo/estimated_position", self._est_cb, 10)
        self.create_subscription(Float32MultiArray, "/stereo/corrected_position", self._corr_cb, 10)
        self.create_timer(1.0 / max(1.0, float(sample_hz)), self._sample)

    def _gt_cb(self, msg):
        values = list(msg.data[:3])
        if len(values) == 3 and all(finite(value) for value in values):
            with self.lock:
                self.latest_gt = tuple(float(value) for value in values)

    def _est_cb(self, msg):
        values = list(msg.data[:3])
        if len(values) == 3 and all(finite(value) for value in values):
            with self.lock:
                self.latest_est = tuple(float(value) for value in values)

    def _corr_cb(self, msg):
        values = list(msg.data[:3])
        if len(values) == 3 and all(finite(value) for value in values):
            with self.lock:
                self.latest_corr = tuple(float(value) for value in values)

    def begin_trial(self, trial_id):
        with self.lock:
            self.current_trial = int(trial_id)
            self.trial_started = time.monotonic()
            self.rows_by_trial.setdefault(int(trial_id), [])

    def end_trial(self):
        with self.lock:
            self.current_trial = None
            self.trial_started = None

    def _sample(self):
        with self.lock:
            if self.current_trial is None or self.trial_started is None:
                return
            if self.latest_gt is None or self.latest_est is None or self.latest_corr is None:
                return
            elapsed = time.monotonic() - self.trial_started
            gt = self.latest_gt
            est = self.latest_est
            corr = self.latest_corr
            self.rows_by_trial[self.current_trial].append(
                {
                    "trial": self.current_trial,
                    "trial_t_s": elapsed,
                    "gt_forward_m": gt[0],
                    "gt_right_m": gt[1],
                    "gt_down_m": gt[2],
                    "estimated_forward_m": est[0],
                    "estimated_right_m": est[1],
                    "estimated_down_m": est[2],
                    "corrected_forward_m": corr[0],
                    "corrected_right_m": corr[1],
                    "corrected_down_m": corr[2],
                }
            )

    def snapshot(self):
        with self.lock:
            return {trial: list(rows) for trial, rows in self.rows_by_trial.items()}


def evaluate_trial(rows, analysis_skip_s, min_samples, nonregression_epsilon_m=0.0):
    usable = [row for row in rows if float(row["trial_t_s"]) >= analysis_skip_s]
    axes = [
        ("right", "gt_right_m", "estimated_right_m", "corrected_right_m"),
        ("down", "gt_down_m", "estimated_down_m", "corrected_down_m"),
        ("forward", "gt_forward_m", "estimated_forward_m", "corrected_forward_m"),
    ]
    result = {"samples": len(usable), "pass": True, "failure_reason": ""}
    if len(usable) < min_samples:
        result["pass"] = False
        result["failure_reason"] = f"insufficient samples: {len(usable)} < {min_samples}"
        return result
    for axis, gt_key, est_key, corr_key in axes:
        est_errors = [abs(float(row[est_key]) - float(row[gt_key])) for row in usable]
        corr_errors = [abs(float(row[corr_key]) - float(row[gt_key])) for row in usable]
        est_mae = sum(est_errors) / len(est_errors)
        corr_mae = sum(corr_errors) / len(corr_errors)
        result[f"{axis}_estimated_mae_m"] = est_mae
        result[f"{axis}_corrected_mae_m"] = corr_mae
        result[f"{axis}_improvement_m"] = est_mae - corr_mae
        result[f"{axis}_estimated_p95_m"] = percentile(est_errors, 95)
        result[f"{axis}_corrected_p95_m"] = percentile(corr_errors, 95)
        if corr_mae > est_mae + max(0.0, float(nonregression_epsilon_m)):
            result["pass"] = False
            if not result["failure_reason"]:
                result["failure_reason"] = (
                    f"{axis} corrected MAE {corr_mae:.6f} > "
                    f"estimated MAE {est_mae:.6f} + epsilon {nonregression_epsilon_m:.6f}"
                )
    return result


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
        "enable_bbox_center_adjustment:=true",
        f"bbox_center_adjustment_path:={args.profile_path}",
        f"left_roll_error_deg:={args.left_roll_error_deg}",
        f"left_pitch_error_deg:={args.left_pitch_error_deg}",
        f"left_yaw_error_deg:={args.left_yaw_error_deg}",
        f"right_roll_error_deg:={args.right_roll_error_deg}",
        f"right_pitch_error_deg:={args.right_pitch_error_deg}",
        f"right_yaw_error_deg:={args.right_yaw_error_deg}",
    ]


def main():
    args = parse_args()
    if args.trials <= 0:
        raise SystemExit("--trials must be greater than zero")
    root = Path(__file__).resolve().parents[1]
    output_dir = (
        Path(args.output_dir)
        if args.output_dir
        else root / "reports" / f"bbox_profile_random_validation_{dt.datetime.now():%Y%m%d_%H%M%S}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    profile_summary = profile_overlap_summary(args.profile_source_csv)
    rng = random.Random(args.seed)
    trajectories = [build_random_trajectory(rng, args) for _ in range(args.trials)]
    first_point = trajectories[0][0]
    all_in_overlap = all(point_in_common_fov(*point, args) for path in trajectories for point in path)

    metadata = {
        "created_at": dt.datetime.now().isoformat(timespec="seconds"),
        "axis_contract": AXIS_CONTRACT,
        "pass_rule": (
            "for every trial, RDF axis corrected MAE must be no greater than "
            "estimated MAE plus nonregression_epsilon_m"
        ),
        "nonregression_epsilon_m": args.nonregression_epsilon_m,
        "attitude_error_deg": {
            "left_roll": args.left_roll_error_deg,
            "left_pitch": args.left_pitch_error_deg,
            "left_yaw": args.left_yaw_error_deg,
            "right_roll": args.right_roll_error_deg,
            "right_pitch": args.right_pitch_error_deg,
            "right_yaw": args.right_yaw_error_deg,
        },
        "profile_path": args.profile_path,
        "profile_source_overlap": profile_summary,
        "random_seed": args.seed,
        "trajectory_waypoints_all_inside_common_fov": all_in_overlap,
        "trajectories": trajectories,
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if not all_in_overlap:
        print("Generated trajectory had a waypoint outside the common FOV.", file=sys.stderr)
        return 2

    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp/ros2-bbox-profile-validation-log")
    env.setdefault("GZ_CONFIG_HOME", "/tmp/gz-bbox-profile-validation-config")
    model_path = str(root / "install/stereo_ground_sim/share/stereo_ground_sim/models")
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        item for item in [model_path, env.get("GZ_SIM_RESOURCE_PATH", "")] if item
    )
    if args.yolo_pythonpath and Path(args.yolo_pythonpath).is_dir():
        env["PYTHONPATH"] = os.pathsep.join(
            item for item in [args.yolo_pythonpath, env.get("PYTHONPATH", "")] if item
        )

    kill_matching_processes("stereo_ground_drone_attitude_error.sdf")
    kill_matching_processes("ros2 launch stereo_ground_sim stereo_ground_sim.launch.py")
    sim, sim_log = run_process(launch_args(args, first_point), output_dir / "simulation.log", env)

    rclpy.init(args=None)
    recorder = ValidationRecorder(args.sample_hz)
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()

    try:
        time.sleep(args.warmup_s)
        if sim.poll() is not None:
            raise RuntimeError(f"simulation exited during warmup with code {sim.returncode}")
        period = 1.0 / max(0.1, args.command_hz)
        for trial_id, trajectory in enumerate(trajectories, start=1):
            print(f"trial {trial_id}/{args.trials}: waypoints={len(trajectory)}", flush=True)
            command_target_pose(args.target_model_name, trajectory[0], args.target_yaw_deg)
            time.sleep(args.settle_s)
            recorder.begin_trial(trial_id)
            started = time.monotonic()
            while True:
                elapsed = time.monotonic() - started
                if elapsed >= args.trial_duration_s:
                    break
                if sim.poll() is not None:
                    raise RuntimeError(f"simulation exited early with code {sim.returncode}")
                point = interpolate(trajectory, elapsed / args.trial_duration_s)
                command_target_pose(args.target_model_name, point, args.target_yaw_deg)
                time.sleep(period)
            command_target_pose(args.target_model_name, trajectory[-1], args.target_yaw_deg)
            recorder.end_trial()
    finally:
        recorder.end_trial()
        snapshot = recorder.snapshot()
        recorder.destroy_node()
        rclpy.shutdown()
        stop_process(sim)
        kill_matching_processes("stereo_ground_drone_attitude_error.sdf")
        sim_log.close()

    summary_rows = []
    final_pass = True
    fieldnames = [
        "trial",
        "trial_t_s",
        "gt_forward_m",
        "gt_right_m",
        "gt_down_m",
        "estimated_forward_m",
        "estimated_right_m",
        "estimated_down_m",
        "corrected_forward_m",
        "corrected_right_m",
        "corrected_down_m",
    ]
    for trial_id in range(1, args.trials + 1):
        rows = snapshot.get(trial_id, [])
        write_csv(output_dir / f"trial_{trial_id:02d}.csv", fieldnames, rows)
        result = evaluate_trial(
            rows,
            args.analysis_skip_s,
            args.min_samples,
            args.nonregression_epsilon_m,
        )
        result["trial"] = trial_id
        result["pass"] = "yes" if result["pass"] else "no"
        if result["pass"] != "yes":
            final_pass = False
        summary_rows.append(result)

    summary_fields = [
        "trial",
        "samples",
        "right_estimated_mae_m",
        "right_corrected_mae_m",
        "right_improvement_m",
        "right_estimated_p95_m",
        "right_corrected_p95_m",
        "down_estimated_mae_m",
        "down_corrected_mae_m",
        "down_improvement_m",
        "down_estimated_p95_m",
        "down_corrected_p95_m",
        "forward_estimated_mae_m",
        "forward_corrected_mae_m",
        "forward_improvement_m",
        "forward_estimated_p95_m",
        "forward_corrected_p95_m",
        "pass",
        "failure_reason",
    ]
    formatted_rows = []
    for row in summary_rows:
        formatted = {}
        for field in summary_fields:
            value = row.get(field, "")
            if isinstance(value, float):
                formatted[field] = f"{value:.9g}"
            else:
                formatted[field] = value
        formatted_rows.append(formatted)
    write_csv(output_dir / "summary.csv", summary_fields, formatted_rows)
    (output_dir / "result.txt").write_text(
        f"final_pass={'yes' if final_pass else 'no'}\nsummary={output_dir / 'summary.csv'}\n",
        encoding="utf-8",
    )
    print(f"Report: {output_dir / 'summary.csv'}", flush=True)
    print(f"Final pass: {'yes' if final_pass else 'no'}", flush=True)
    return 0 if final_pass else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
