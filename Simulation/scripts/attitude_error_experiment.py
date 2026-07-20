#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import math
import os
import signal
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


DEFAULT_AXES = ("right_roll_error_deg", "right_pitch_error_deg", "right_yaw_error_deg")
AXIS_CONTRACT = "baseline_center_frd:forward_right_down"
DEFAULT_YOLO_MODEL = "yolov8n.pt"
VALID_ATTITUDE_AXES = {
    "left_roll_error_deg",
    "left_pitch_error_deg",
    "left_yaw_error_deg",
    "right_roll_error_deg",
    "right_pitch_error_deg",
    "right_yaw_error_deg",
}


def parse_axis_range(spec):
    parts = [part.strip() for part in spec.split(":")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "axis range must be AXIS:MIN_DEG:MAX_DEG:STEP_DEG"
        )
    axis, start, stop, step = parts
    if axis not in VALID_ATTITUDE_AXES:
        raise argparse.ArgumentTypeError(
            f"unknown axis {axis!r}; expected one of {', '.join(sorted(VALID_ATTITUDE_AXES))}"
        )
    try:
        start = float(start)
        stop = float(stop)
        step = float(step)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "axis range min/max/step must be numeric"
        ) from exc
    if step <= 0.0:
        raise argparse.ArgumentTypeError("axis range step must be greater than 0")
    if start > stop:
        raise argparse.ArgumentTypeError("axis range min must be less than or equal to max")
    return axis, start, stop, step


def selected_axes(args):
    axes = list(args.axes)
    for axis, *_ in args.axis_range:
        if axis not in axes:
            axes.append(axis)
    return axes


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run repeated stereo distance experiments while varying one camera attitude."
    )
    parser.add_argument("--angle-min-deg", type=float, default=None)
    parser.add_argument("--angle-max-deg", type=float, default=None)
    parser.add_argument("--angle-step-deg", type=float, default=0.25)
    parser.add_argument("--axes", nargs="+", default=list(DEFAULT_AXES))
    parser.add_argument(
        "--axis-range",
        action="append",
        default=[],
        type=parse_axis_range,
        metavar="AXIS:MIN:MAX:STEP",
        help=(
            "Per-axis sweep range in degrees. Repeatable. "
            "Example: --axis-range right_yaw_error_deg:-2:2:0.25"
        ),
    )
    parser.add_argument("--warmup-s", type=float, default=12.0)
    parser.add_argument("--approach-start-distance-m", type=float, default=2000.0)
    parser.add_argument("--approach-end-distance-m", type=float, default=200.0)
    parser.add_argument("--takeoff-altitude-m", type=float, default=10.0)
    parser.add_argument("--takeoff-duration-s", type=float, default=10.0)
    parser.add_argument("--first-hover-duration-s", type=float, default=5.0)
    parser.add_argument("--approach-duration-s", type=float, default=120.0)
    parser.add_argument("--final-hover-duration-s", type=float, default=5.0)
    parser.add_argument("--land-duration-s", type=float, default=10.0)
    parser.add_argument("--sample-tail-s", type=float, default=8.0)
    parser.add_argument("--command-hz", type=float, default=2.0)
    parser.add_argument("--cpu-limit-percent", type=float, default=80.0)
    parser.add_argument("--yolo-model", default=DEFAULT_YOLO_MODEL)
    parser.add_argument("--yolo-class", default="UAV")
    parser.add_argument("--yolo-confidence", type=float, default=0.10)
    parser.add_argument("--yolo-device", default="cuda:0")
    parser.add_argument("--yolo-target-hz", type=float, default=40.0)
    parser.add_argument("--yolo-imgsz", type=int, default=1280)
    parser.add_argument("--yolo-python", default=sys.executable)
    parser.add_argument("--baseline-m", type=float, default=25.0)
    parser.add_argument("--camera-height-m", type=float, default=5.0)
    parser.add_argument("--horizontal-fov-rad", type=float, default=0.154997710698)
    parser.add_argument("--image-width-px", type=int, default=1280)
    parser.add_argument("--image-height-px", type=int, default=720)
    parser.add_argument("--camera-update-rate-hz", type=float, default=60.0)
    parser.add_argument("--camera-clip-near-m", type=float, default=0.1)
    parser.add_argument("--camera-clip-far-m", type=float, default=2500.0)
    parser.add_argument("--target-spawn-x-m", type=float, default=0.0)
    parser.add_argument("--target-spawn-y-m", type=float, default=2000.0)
    parser.add_argument("--target-spawn-z-m", type=float, default=0.75)
    parser.add_argument("--target-spawn-yaw-deg", type=float, default=-90.0)
    parser.add_argument("--output-dir", default="")
    parser.add_argument(
        "--include-negative",
        action="store_true",
        help="Also test -angle-step-deg for each selected axis.",
    )
    parser.add_argument(
        "--keep-gui",
        action="store_true",
        help="Keep Gazebo GUI and RViz enabled. Default is headless to reduce CPU load.",
    )
    parser.add_argument(
        "--allow-existing-sim",
        action="store_true",
        help="Run even if an existing stereo Gazebo/ROS/RViz session is detected.",
    )
    parser.add_argument(
        "--no-stop-existing-sim",
        action="store_true",
        help="Fail instead of stopping an existing attitude_error simulation session.",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Use a short 20 s approach for smoke testing. This does not match the full 1900 m mission timing.",
    )
    return parser.parse_args()


class DistanceRecorder(Node):
    def __init__(self):
        super().__init__("attitude_error_experiment_recorder")
        self.lock = threading.Lock()
        self.started = time.monotonic()
        self.rows = []
        self.latest_actual = None
        self.latest_measured = None
        self.latest_error = None
        self.latest_drone_x = None
        self.latest_drone_y = None
        self.latest_drone_z = None
        self.latest_ground_truth = None
        self.latest_estimated_x = None
        self.latest_estimated_y = None
        self.latest_estimated_z = None
        self.latest_estimated_range = None
        self.create_subscription(Float32, "/ground_truth", self._actual_cb, 10)
        self.create_subscription(Float32, "/target_distance", self._measured_cb, 10)
        self.create_subscription(Float32, "/stereo/distance_error", self._error_cb, 10)
        self.create_subscription(
            Float32MultiArray,
            "/ground_truth_position",
            self._ground_truth_position_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/stereo/estimated_position",
            self._estimated_position_cb,
            10,
        )

    def _append(self, name):
        now = time.monotonic() - self.started
        with self.lock:
            self.rows.append(
                {
                    "t_s": f"{now:.3f}",
                    "axis_contract": AXIS_CONTRACT,
                    "drone_forward_m": "" if self.latest_drone_x is None else f"{self.latest_drone_x:.6f}",
                    "drone_right_m": "" if self.latest_drone_y is None else f"{self.latest_drone_y:.6f}",
                    "drone_down_m": "" if self.latest_drone_z is None else f"{self.latest_drone_z:.6f}",
                    "ground_truth_m": (
                        "" if self.latest_ground_truth is None else f"{self.latest_ground_truth:.6f}"
                    ),
                    "estimated_forward_m": (
                        "" if self.latest_estimated_x is None else f"{self.latest_estimated_x:.6f}"
                    ),
                    "estimated_right_m": (
                        "" if self.latest_estimated_y is None else f"{self.latest_estimated_y:.6f}"
                    ),
                    "estimated_down_m": (
                        "" if self.latest_estimated_z is None else f"{self.latest_estimated_z:.6f}"
                    ),
                    "estimated_range_m": (
                        "" if self.latest_estimated_range is None else f"{self.latest_estimated_range:.6f}"
                    ),
                    "measured_m": "" if self.latest_measured is None else f"{self.latest_measured:.6f}",
                    "error_m": "" if self.latest_error is None else f"{self.latest_error:.6f}",
                    "source": name,
                }
            )

    def _actual_cb(self, msg):
        with self.lock:
            self.latest_actual = float(msg.data)
            if self.latest_ground_truth is None:
                self.latest_ground_truth = self.latest_actual
        self._append("ground_truth")

    def _measured_cb(self, msg):
        with self.lock:
            self.latest_measured = float(msg.data)
        self._append("measured")

    def _error_cb(self, msg):
        with self.lock:
            self.latest_error = float(msg.data)
        self._append("error")

    def _ground_truth_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        with self.lock:
            self.latest_drone_x = float(values[0])
            self.latest_drone_y = float(values[1])
            self.latest_drone_z = float(values[2])
            self.latest_ground_truth = float(values[3])
            self.latest_actual = self.latest_ground_truth
        self._append("ground_truth_position")

    def _estimated_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        with self.lock:
            self.latest_estimated_x = float(values[0])
            self.latest_estimated_y = float(values[1])
            self.latest_estimated_z = float(values[2])
            self.latest_estimated_range = float(values[3])
        self._append("estimated_position")

    def snapshot(self):
        with self.lock:
            return list(self.rows)


def read_proc_stat():
    with open("/proc/stat", "r", encoding="utf-8") as handle:
        fields = handle.readline().split()[1:]
    values = [int(value) for value in fields]
    idle = values[3] + values[4]
    total = sum(values)
    return idle, total


def cpu_sampler(stop_event, rows, interval_s=1.0):
    last_idle, last_total = read_proc_stat()
    start = time.monotonic()
    while not stop_event.wait(interval_s):
        idle, total = read_proc_stat()
        total_delta = total - last_total
        idle_delta = idle - last_idle
        if total_delta > 0:
            usage = 100.0 * (1.0 - (idle_delta / total_delta))
            rows.append({"t_s": f"{time.monotonic() - start:.3f}", "cpu_percent": f"{usage:.3f}"})
        last_idle, last_total = idle, total


def run_process(command, log_path, env):
    log = open(log_path, "w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        preexec_fn=os.setsid,
    )
    return process, log


def stop_process(process, timeout_s=10.0, first_signal=signal.SIGINT):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, first_signal)
        process.wait(timeout=timeout_s)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=timeout_s)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def kill_matching_processes(pattern, timeout_s=5.0):
    result = subprocess.run(
        ["pgrep", "-af", pattern],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    pids = []
    current_pid = os.getpid()
    for line in result.stdout.splitlines():
        parts = line.split(maxsplit=1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        command = parts[1] if len(parts) > 1 else ""
        if pid == current_pid or "pgrep -af" in command:
            continue
        pids.append(pid)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + timeout_s
    while pids and time.monotonic() < deadline:
        pids = [pid for pid in pids if process_exists(pid)]
        if pids:
            time.sleep(0.2)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def process_exists(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def write_csv(path, fieldnames, rows):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def format_float(value, precision=6):
    return f"{float(value):.{precision}f}"


def fov_deg(args):
    return math.degrees(args.horizontal_fov_rad)


def focal_length_px(args):
    return (args.image_width_px * 0.5) / math.tan(args.horizontal_fov_rad * 0.5)


def camera_model_summary(args):
    return {
        "baseline_m": format_float(args.baseline_m),
        "camera_height_m": format_float(args.camera_height_m),
        "horizontal_fov_rad": format_float(args.horizontal_fov_rad, 9),
        "horizontal_fov_deg": format_float(fov_deg(args)),
        "focal_length_px": format_float(focal_length_px(args)),
        "image_width_px": str(args.image_width_px),
        "image_height_px": str(args.image_height_px),
        "camera_update_rate_hz": format_float(args.camera_update_rate_hz),
        "camera_clip_near_m": format_float(args.camera_clip_near_m),
        "camera_clip_far_m": format_float(args.camera_clip_far_m),
    }


def attitudes_from_args(attitude_args):
    left = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    right = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    for name, value in attitude_args.items():
        parts = name.split("_")
        if len(parts) < 3:
            continue
        camera, axis = parts[0], parts[1]
        if camera == "left" and axis in left:
            left[axis] = float(value)
        elif camera == "right" and axis in right:
            right[axis] = float(value)
    return left, right


def write_metadata(output_dir, args, cases):
    lines = [
        "Attitude error experiment metadata",
        f"created_at={dt.datetime.now().isoformat(timespec='seconds')}",
        f"axis_contract={AXIS_CONTRACT}",
        "axis_public_fields=drone_forward_m/drone_right_m/drone_down_m and estimated_forward_m/estimated_right_m/estimated_down_m",
        "axis_aliases=public CSV uses forward/right/down; legacy x/y/z is read only for older files",
        "",
        "[camera]",
    ]
    for key, value in camera_model_summary(args).items():
        lines.append(f"{key}={value}")
    lines.extend(
        [
            "",
            "[sweep]",
            f"axes={' '.join(selected_axes(args))}",
            "axis_ranges="
            + " ".join(
                f"{axis}:{start:g}:{stop:g}:{step:g}"
                for axis, start, stop, step in args.axis_range
            ),
            f"angle_min_deg={'' if args.angle_min_deg is None else format_float(args.angle_min_deg)}",
            f"angle_max_deg={'' if args.angle_max_deg is None else format_float(args.angle_max_deg)}",
            f"angle_step_deg={format_float(args.angle_step_deg)}",
            f"include_negative={str(args.include_negative).lower()}",
            f"quick={str(args.quick).lower()}",
            f"cpu_limit_percent={format_float(args.cpu_limit_percent)}",
            "",
            "[mission]",
            f"warmup_s={format_float(args.warmup_s)}",
            f"takeoff_duration_s={format_float(args.takeoff_duration_s)}",
            f"first_hover_duration_s={format_float(args.first_hover_duration_s)}",
            f"approach_duration_s={format_float(args.approach_duration_s)}",
            f"final_hover_duration_s={format_float(args.final_hover_duration_s)}",
            f"land_duration_s={format_float(args.land_duration_s)}",
            f"sample_tail_s={format_float(args.sample_tail_s)}",
            "",
            "[yolo]",
            f"model={args.yolo_model}",
            f"class={args.yolo_class}",
            f"confidence={format_float(args.yolo_confidence)}",
            f"device={args.yolo_device}",
            f"target_hz={format_float(args.yolo_target_hz)}",
            f"imgsz={args.yolo_imgsz}",
            f"python={args.yolo_python}",
            "",
            "[cases]",
        ]
    )
    for case_name, attitude_args in cases:
        left, right = attitudes_from_args(attitude_args)
        lines.append(
            f"{case_name}: "
            f"left=roll:{left['roll']:+.6f},pitch:{left['pitch']:+.6f},yaw:{left['yaw']:+.6f}; "
            f"right=roll:{right['roll']:+.6f},pitch:{right['pitch']:+.6f},yaw:{right['yaw']:+.6f}"
        )
    (output_dir / "metadata.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize_distance(rows):
    usable = []
    for row in rows:
        ground_truth_value = row.get("ground_truth_m") or row.get("actual_m")
        if not ground_truth_value or not row["measured_m"] or not row["error_m"]:
            continue
        drone_forward = row.get("drone_forward_m") or row.get("drone_x_m") or ""
        drone_right = row.get("drone_right_m") or row.get("drone_y_m") or ""
        drone_down = row.get("drone_down_m") or row.get("drone_z_m") or ""
        estimated_forward = row.get("estimated_forward_m") or row.get("estimated_x_m") or ""
        estimated_right = row.get("estimated_right_m") or row.get("estimated_y_m") or ""
        estimated_down = row.get("estimated_down_m") or row.get("estimated_z_m") or ""
        estimated_range = row.get("estimated_range_m") or ""
        actual = float(ground_truth_value)
        measured = float(row["measured_m"])
        error = float(row["error_m"])
        if not (math.isfinite(actual) and math.isfinite(measured) and math.isfinite(error)):
            continue
        usable.append(
            (
                drone_forward,
                drone_right,
                drone_down,
                estimated_forward,
                estimated_right,
                estimated_down,
                estimated_range,
                actual,
                measured,
                error,
            )
        )
    if not usable:
        return {
            "samples": 0,
            "mean_abs_error_m": "",
            "max_abs_error_m": "",
            "last_drone_forward_m": "",
            "last_drone_right_m": "",
            "last_drone_down_m": "",
            "last_estimated_forward_m": "",
            "last_estimated_right_m": "",
            "last_estimated_down_m": "",
            "last_estimated_range_m": "",
            "last_ground_truth_m": "",
            "last_measured_m": "",
            "last_error_m": "",
        }
    errors = [abs(item[9]) for item in usable]
    (
        last_drone_forward,
        last_drone_right,
        last_drone_down,
        last_estimated_forward,
        last_estimated_right,
        last_estimated_down,
        last_estimated_range,
        last_actual,
        last_measured,
        last_error,
    ) = usable[-1]
    return {
        "samples": len(usable),
        "mean_abs_error_m": f"{statistics.fmean(errors):.6f}",
        "max_abs_error_m": f"{max(errors):.6f}",
        "last_drone_forward_m": last_drone_forward,
        "last_drone_right_m": last_drone_right,
        "last_drone_down_m": last_drone_down,
        "last_estimated_forward_m": last_estimated_forward,
        "last_estimated_right_m": last_estimated_right,
        "last_estimated_down_m": last_estimated_down,
        "last_estimated_range_m": last_estimated_range,
        "last_ground_truth_m": f"{last_actual:.6f}",
        "last_measured_m": f"{last_measured:.6f}",
        "last_error_m": f"{last_error:.6f}",
    }


def summarize_cpu(rows):
    if not rows:
        return {"mean_cpu_percent": "", "max_cpu_percent": ""}
    values = [float(row["cpu_percent"]) for row in rows]
    return {
        "mean_cpu_percent": f"{statistics.fmean(values):.3f}",
        "max_cpu_percent": f"{max(values):.3f}",
    }


def build_cases(args):
    cases = [("baseline", {})]
    axis_ranges = {axis: (start, stop, step) for axis, start, stop, step in args.axis_range}
    axes = selected_axes(args)

    def sweep_values(start, stop, step):
        values = []
        current = start
        epsilon = step * 1e-6
        while current <= stop + epsilon:
            value = 0.0 if abs(current) < 1e-12 else current
            values.append(round(value, 10))
            current += step
        return values

    if args.angle_min_deg is None and args.angle_max_deg is None:
        global_values = [args.angle_step_deg]
        if args.include_negative:
            global_values.insert(0, -args.angle_step_deg)
    else:
        start = args.angle_min_deg if args.angle_min_deg is not None else args.angle_step_deg
        stop = args.angle_max_deg if args.angle_max_deg is not None else start
        if args.angle_step_deg <= 0.0:
            raise SystemExit("--angle-step-deg must be greater than 0.")
        if start > stop:
            raise SystemExit("--angle-min-deg must be less than or equal to --angle-max-deg.")
        global_values = sweep_values(start, stop, args.angle_step_deg)

    for axis in axes:
        if axis in axis_ranges:
            values = sweep_values(*axis_ranges[axis])
        else:
            values = global_values
        for value in values:
            if value == 0.0:
                continue
            angle = f"{value:+.6f}".rstrip("0").rstrip(".")
            cases.append((f"{axis}_{angle}deg", {axis: value}))
    return cases


ATTITUDE_SIM_PATTERNS = (
    "stereo_ground_drone_attitude_error.sdf",
    "ros2 launch stereo_ground_sim stereo_ground_sim.launch.py.*fly_target_in_gazebo:=true",
    "ros2 launch stereo_ground_sim stereo_ground_sim.launch.py.*right_(roll|pitch|yaw)_error_deg:=",
)


def find_existing_attitude_processes():
    lines = []
    seen_pids = set()
    for pattern in ATTITUDE_SIM_PATTERNS:
        result = subprocess.run(
            ["pgrep", "-af", pattern],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
        for line in result.stdout.splitlines():
            parts = line.split(maxsplit=1)
            if not parts:
                continue
            try:
                pid = int(parts[0])
            except ValueError:
                continue
            command = parts[1] if len(parts) > 1 else ""
            if (
                pid == os.getpid()
                or "attitude_error_experiment.py" in command
                or "pgrep -af" in command
                or "codex-linux-sandbox" in command
                or "bwrap " in command
                or "__CODEX_SNAPSHOT" in command
            ):
                continue
            if pid in seen_pids:
                continue
            seen_pids.add(pid)
            lines.append(line)
    return lines


def stop_existing_attitude_processes(lines, timeout_s=10.0):
    pids = []
    for line in lines:
        parts = line.split(maxsplit=1)
        if not parts:
            continue
        try:
            pids.append(int(parts[0]))
        except ValueError:
            continue
    for pid in pids:
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
        except PermissionError:
            continue
        except OSError:
            try:
                os.kill(pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = find_existing_attitude_processes()
        if not remaining:
            return []
        time.sleep(0.25)
    remaining = find_existing_attitude_processes()
    for line in remaining:
        parts = line.split(maxsplit=1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        try:
            os.killpg(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        except OSError:
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
    time.sleep(0.5)
    return find_existing_attitude_processes()


def main():
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    output_dir = Path(args.output_dir) if args.output_dir else root / "reports" / f"attitude_error_{dt.datetime.now():%Y%m%d_%H%M%S}"
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.quick:
        args.takeoff_duration_s = 3.0
        args.first_hover_duration_s = 1.0
        args.approach_duration_s = 20.0
        args.final_hover_duration_s = 1.0
        args.land_duration_s = 3.0

    existing = find_existing_attitude_processes()
    if existing and not args.allow_existing_sim:
        print("Existing attitude_error Gazebo/ROS simulation processes were detected:", file=sys.stderr)
        for line in existing:
            print(f"  {line}", file=sys.stderr)
        if args.no_stop_existing_sim:
            print("Stop them first, or rerun without --no-stop-existing-sim to clean them automatically.", file=sys.stderr)
            return 2
        print("Stopping existing attitude_error simulation processes before starting a new run.", file=sys.stderr)
        remaining = stop_existing_attitude_processes(existing)
        if remaining:
            print("Failed to stop all existing attitude_error simulation processes:", file=sys.stderr)
            for line in remaining:
                print(f"  {line}", file=sys.stderr)
            return 2

    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp/ros2-attitude-experiment-log")
    env.setdefault("GZ_CONFIG_HOME", "/tmp/gz-attitude-experiment-config")
    env["GZ_SIM_RESOURCE_PATH"] = (
        f"{root / 'install/stereo_ground_sim/share/stereo_ground_sim/models'}:"
        + env.get("GZ_SIM_RESOURCE_PATH", "")
    )

    cases = build_cases(args)
    write_metadata(output_dir, args, cases)
    summary_rows = []
    experiment_started = time.monotonic()
    total_duration = (
        args.warmup_s
        + args.takeoff_duration_s
        + args.first_hover_duration_s
        + args.approach_duration_s
        + args.final_hover_duration_s
        + args.land_duration_s
        + args.sample_tail_s
    )

    for case_name, attitude_args in cases:
        case_started = time.monotonic()
        print(f"\n=== {case_name} ===", flush=True)
        launch_args = [
            "ros2",
            "launch",
            "stereo_ground_sim",
            "stereo_ground_sim.launch.py",
            f"gz_gui:={'true' if args.keep_gui else 'false'}",
            f"launch_rviz:={'true' if args.keep_gui else 'false'}",
            "mission_profile:=takeoff_forward_land",
            "fly_target_in_gazebo:=true",
            "actual_distance_source:=gazebo",
            f"approach_start_distance_m:={args.approach_start_distance_m}",
            f"approach_end_distance_m:={args.approach_end_distance_m}",
            f"takeoff_altitude_m:={args.takeoff_altitude_m}",
            f"takeoff_duration_s:={args.takeoff_duration_s}",
            f"first_hover_duration_s:={args.first_hover_duration_s}",
            f"approach_duration_s:={args.approach_duration_s}",
            f"final_hover_duration_s:={args.final_hover_duration_s}",
            f"land_duration_s:={args.land_duration_s}",
            f"target_spawn_x_m:={args.target_spawn_x_m}",
            f"target_spawn_y_m:={args.target_spawn_y_m}",
            f"target_spawn_z_m:={args.target_spawn_z_m}",
            f"target_spawn_yaw_deg:={args.target_spawn_yaw_deg}",
            f"baseline_m:={args.baseline_m}",
            f"camera_height_m:={args.camera_height_m}",
            f"horizontal_fov_rad:={args.horizontal_fov_rad}",
            f"camera_image_width_px:={args.image_width_px}",
            f"camera_image_height_px:={args.image_height_px}",
            f"camera_update_rate_hz:={args.camera_update_rate_hz}",
            f"camera_clip_near_m:={args.camera_clip_near_m}",
            f"camera_clip_far_m:={args.camera_clip_far_m}",
            f"yolo_model_path:={args.yolo_model}",
            f"yolo_class_name:={args.yolo_class}",
            f"yolo_confidence:={args.yolo_confidence}",
            f"yolo_device:={args.yolo_device}",
            f"yolo_target_hz:={args.yolo_target_hz}",
            f"yolo_imgsz:={args.yolo_imgsz}",
            f"yolo_python:={args.yolo_python}",
        ]
        launch_args.extend(f"{name}:={value}" for name, value in attitude_args.items())
        sim, sim_log = run_process(launch_args, output_dir / f"{case_name}_sim.log", env)

        rclpy.init(args=None)
        recorder = DistanceRecorder()
        spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
        spin_thread.start()
        cpu_rows = []
        stop_cpu = threading.Event()
        cpu_thread = threading.Thread(target=cpu_sampler, args=(stop_cpu, cpu_rows), daemon=True)
        cpu_thread.start()

        try:
            time.sleep(args.warmup_s)
            deadline = time.monotonic() + total_duration - args.warmup_s
            while time.monotonic() < deadline:
                if sim.poll() is not None:
                    raise RuntimeError(f"simulation exited early with code {sim.returncode}")
                time.sleep(1.0)
        finally:
            stop_cpu.set()
            cpu_thread.join(timeout=2.0)
            recorder.destroy_node()
            rclpy.shutdown()
            stop_process(sim)
            kill_matching_processes("stereo_ground_drone_attitude_error.sdf")
            sim_log.close()

        distance_rows = recorder.snapshot()
        case_elapsed_s = time.monotonic() - case_started
        write_csv(
            output_dir / f"{case_name}_distance.csv",
            [
                "t_s",
                "axis_contract",
                "drone_forward_m",
                "drone_right_m",
                "drone_down_m",
                "ground_truth_m",
                "estimated_forward_m",
                "estimated_right_m",
                "estimated_down_m",
                "estimated_range_m",
                "measured_m",
                "error_m",
                "source",
            ],
            distance_rows,
        )
        write_csv(output_dir / f"{case_name}_cpu.csv", ["t_s", "cpu_percent"], cpu_rows)
        distance_summary = summarize_distance(distance_rows)
        cpu_summary = summarize_cpu(cpu_rows)
        exceeded = (
            bool(cpu_summary["max_cpu_percent"])
            and float(cpu_summary["max_cpu_percent"]) > args.cpu_limit_percent
        )
        left_attitude, right_attitude = attitudes_from_args(attitude_args)
        summary = {
            "case": case_name,
            "attitude_args": " ".join(f"{key}:={value}" for key, value in attitude_args.items()) or "none",
            "duration_s": f"{case_elapsed_s:.3f}",
            "left_roll_deg": f"{left_attitude['roll']:+.6f}",
            "left_pitch_deg": f"{left_attitude['pitch']:+.6f}",
            "left_yaw_deg": f"{left_attitude['yaw']:+.6f}",
            "right_roll_deg": f"{right_attitude['roll']:+.6f}",
            "right_pitch_deg": f"{right_attitude['pitch']:+.6f}",
            "right_yaw_deg": f"{right_attitude['yaw']:+.6f}",
            "angle_min_deg": "" if args.angle_min_deg is None else format_float(args.angle_min_deg),
            "angle_max_deg": "" if args.angle_max_deg is None else format_float(args.angle_max_deg),
            "angle_step_deg": format_float(args.angle_step_deg),
            "include_negative": str(args.include_negative).lower(),
            **camera_model_summary(args),
            **distance_summary,
            **cpu_summary,
            "cpu_limit_percent": f"{args.cpu_limit_percent:.1f}",
            "cpu_over_limit": "yes" if exceeded else "no",
        }
        summary_rows.append(summary)
        print(
            "samples={samples}, mean_abs_error_m={mean_abs_error_m}, "
            "duration_s={duration_s}, max_cpu_percent={max_cpu_percent}, "
            "cpu_over_limit={cpu_over_limit}".format(**summary),
            flush=True,
        )

    baseline_mean = None
    baseline_max = None
    for row in summary_rows:
        if row["case"] == "baseline":
            baseline_mean = float(row["mean_abs_error_m"]) if row["mean_abs_error_m"] else None
            baseline_max = float(row["max_abs_error_m"]) if row["max_abs_error_m"] else None
            break
    for row in summary_rows:
        if baseline_mean is None or not row["mean_abs_error_m"]:
            row["delta_mean_abs_error_m_vs_baseline"] = ""
        else:
            row["delta_mean_abs_error_m_vs_baseline"] = (
                f"{float(row['mean_abs_error_m']) - baseline_mean:.6f}"
            )
        if baseline_max is None or not row["max_abs_error_m"]:
            row["delta_max_abs_error_m_vs_baseline"] = ""
        else:
            row["delta_max_abs_error_m_vs_baseline"] = (
                f"{float(row['max_abs_error_m']) - baseline_max:.6f}"
            )

    write_csv(
        output_dir / "summary.csv",
        [
            "case",
            "attitude_args",
            "duration_s",
            "left_roll_deg",
            "left_pitch_deg",
            "left_yaw_deg",
            "right_roll_deg",
            "right_pitch_deg",
            "right_yaw_deg",
            "angle_min_deg",
            "angle_max_deg",
            "angle_step_deg",
            "include_negative",
            "baseline_m",
            "camera_height_m",
            "horizontal_fov_rad",
            "horizontal_fov_deg",
            "focal_length_px",
            "image_width_px",
            "image_height_px",
            "camera_update_rate_hz",
            "camera_clip_near_m",
            "camera_clip_far_m",
            "samples",
            "mean_abs_error_m",
            "delta_mean_abs_error_m_vs_baseline",
            "max_abs_error_m",
            "delta_max_abs_error_m_vs_baseline",
            "last_drone_forward_m",
            "last_drone_right_m",
            "last_drone_down_m",
            "last_estimated_forward_m",
            "last_estimated_right_m",
            "last_estimated_down_m",
            "last_estimated_range_m",
            "last_ground_truth_m",
            "last_measured_m",
            "last_error_m",
            "mean_cpu_percent",
            "max_cpu_percent",
            "cpu_limit_percent",
            "cpu_over_limit",
        ],
        summary_rows,
    )
    print(f"\nTotal duration_s={time.monotonic() - experiment_started:.3f}", flush=True)
    print(f"Report: {output_dir / 'summary.csv'}", flush=True)
    return 1 if any(row["cpu_over_limit"] == "yes" for row in summary_rows) else 0


if __name__ == "__main__":
    sys.exit(main())
