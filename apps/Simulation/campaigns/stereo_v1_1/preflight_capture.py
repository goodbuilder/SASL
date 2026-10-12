"""Run a short no-flight lossless capture to validate live image/GT topics."""

from __future__ import annotations

import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from campaigns.stereo_v1_1.camera_frame_contract import optical_rwc_from_entity_quaternion_xyzw


SIM_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = SIM_ROOT.parents[1]
CAMPAIGN_ROOT = Path(__file__).resolve().parent


def _spawn(command: list[str], log_path: Path, env: dict[str, str]):
    handle = log_path.open("x", encoding="utf-8")
    try:
        process = subprocess.Popen(command, cwd=REPO_ROOT, env=env,
                                   stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True, text=True)
    except BaseException:
        handle.close()
        raise
    return process, handle


def _wait_token(process, log_path: Path, token: str, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if log_path.is_file() and token in log_path.read_text(encoding="utf-8", errors="replace"):
            if process.poll() is None:
                return
            raise RuntimeError(f"preflight process exited after {token}: {process.returncode}")
        if process.poll() is not None:
            tail = log_path.read_text(encoding="utf-8", errors="replace")[-1200:]
            raise RuntimeError(f"preflight process exited before {token}: {process.returncode}: {tail}")
        time.sleep(0.25)
    raise TimeoutError(f"preflight process did not report {token}")


def _stop(process, timeout_s: float) -> int | None:
    if process is None:
        return None
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            return process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait(timeout=5)


def _next_attempt(output_root: Path, group: str) -> Path:
    parent = output_root / "preflight"
    parent.mkdir(parents=True, exist_ok=True)
    for number in range(1, 1000):
        target = parent / f"short_capture_{group}_a{number:03d}"
        try:
            target.mkdir()
            return target
        except FileExistsError:
            pass
    raise RuntimeError("preflight short-capture attempt ID space exhausted")


def _quaternion_matrix_xyzw(values) -> list[list[float]]:
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("observed camera quaternion is invalid")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]


def _orientation_error_deg(actual, expected) -> float:
    # trace(actual.T @ expected), computed without relabelling either frame.
    trace = sum(float(actual[k][i]) * float(expected[k][i]) for i in range(3) for k in range(3))
    cosine = (trace - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def _expected_rwc(spec: dict, group: str, side: str) -> list[list[float]]:
    heading_deg = float(spec["camera"]["physical_pose_groups"][group][f"heading_{side}_deg"])
    angle = -math.radians(heading_deg)
    c, s = math.cos(angle), math.sin(angle)
    rz = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    nominal = spec["coordinates"]["R_WC_nominal"]
    return [[sum(rz[i][k] * float(nominal[k][j]) for k in range(3)) for j in range(3)]
            for i in range(3)]


def _write_probe_report(attempt_dir: Path, payload: dict) -> None:
    report_path = attempt_dir / "preflight_report.json"
    with report_path.open("x", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    checksum_path = attempt_dir / "checksums.sha256"
    with checksum_path.open("x", encoding="utf-8") as handle:
        for path in sorted(item for item in attempt_dir.rglob("*")
                           if item.is_file() and item != checksum_path):
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            handle.write(f"{digest}  {path.relative_to(attempt_dir)}\n")
        handle.flush()
        os.fsync(handle.fileno())


def _ros_command(python: Path, script: Path, *args: str) -> list[str]:
    command = [str(python), str(script), *args]
    return ["bash", "-c", 'source "${SASL_ROS_WORKSPACE:-workspace/main}/install/setup.bash"\nexec "$@"', "_", *command]


def run_group(output_root: Path, group: str, spec: dict | None = None,
              workbook_source: dict | None = None) -> dict:
    attempt_dir = _next_attempt(output_root, group)
    env = os.environ.copy()
    env["SIM_CONFIG"] = os.environ.get("SIM_CONFIG", str(SIM_ROOT / "config" / "simulation.env"))
    workbook_environment = {}
    vehicle_model_id = "x500_2m"
    if workbook_source is not None:
        from campaigns.stereo_v1_1.workbook_campaign import scenario_environment, write_scenario_environment
        workbook_environment = scenario_environment(workbook_source["parameters"], workbook_source["sequence"])
        vehicle_model_id = workbook_source["parameters"]["Vehicle"].get("model_id", "x500_2m")
        scenario_path = attempt_dir / "effective_scenario.env"
        write_scenario_environment(scenario_path, workbook_source["parameters"], workbook_source["sequence"])
    for name in tuple(env):
        if name.startswith(("CAMERA_", "CAM_L_", "CAM_R_", "COMMON_OPTICAL_",
                            "RELATIVE_OPTICAL_", "DRONE_", "WAYPOINT_", "YOLO_",
                            "SWEEP_", "AUTO_", "SIM_SCENARIO_", "SIM_TARGET_",
                            "SIM_RANGE_", "SIM_ZOOM_")) or name in workbook_environment or name in {
                                "ENABLE_GPS_BASELINE_TRIANGULATION", "GPS_BASELINE_MAX_AGE_S",
                                "PX4_ATTITUDE_MAX_AGE_S", "PX4_ATTITUDE_PAIR_SKEW_S",
                            }:
            env.pop(name, None)
    env["SIM_CONFIG"] = os.environ.get("SIM_CONFIG", str(SIM_ROOT / "config" / "simulation.env"))
    if workbook_source is not None:
        env["SIM_SCENARIO_CONFIG"] = str(scenario_path)
        env.pop("SIM_GAZEBO_SEED", None)
    else:
        env["SIM_SCENARIO_CONFIG"] = f"config/campaign/v1_1/{group}.env"
        env["SIM_GAZEBO_SEED"] = "0"
    python = Path(os.environ.get("STEREO_V1_1_ROOT_PYTHON", sys.executable)).expanduser()
    operator = state = capture = None
    handles = []
    failure = None
    try:
        operator_command = [str(SIM_ROOT / "run_operator_view.sh"), "--headless", "--no-yolo",
                            "--raw-view", "--skip-px4-build", "--px4-model-name", vehicle_model_id,
                            "observation_mode:=ideal_reference_point", "launch_rviz:=false", "gz_gui:=false"]
        operator, handle = _spawn(operator_command, attempt_dir / "runtime.log", env)
        handles.append(handle)
        _wait_token(operator, attempt_dir / "runtime.log", "Operator view started.", 240)

        state_command = _ros_command(python, CAMPAIGN_ROOT / "state_recorder.py",
                                     "--attempt-dir", str(attempt_dir), "--duration-s", "12")
        state, handle = _spawn(state_command, attempt_dir / "state_recorder.log", env)
        handles.append(handle)
        _wait_token(state, attempt_dir / "state_recorder.log", "STATE_READY", 60)

        capture_command = _ros_command(python, CAMPAIGN_ROOT / "capture_streams.py",
                                       "--attempt-dir", str(attempt_dir), "--duration-s", "5",
                                       "--compression", "zstd")
        capture, handle = _spawn(capture_command, attempt_dir / "capture_runner.log", env)
        handles.append(handle)
        _wait_token(capture, attempt_dir / "capture_runner.log", "CAPTURE_READY", 60)
        capture_code = capture.wait(timeout=90)
        state_code = state.wait(timeout=30)
        if capture_code != 0 or state_code != 0:
            raise RuntimeError(f"short capture process exits: images={capture_code}, state={state_code}")
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
    finally:
        if capture is not None and capture.poll() is None:
            _stop(capture, 20)
        if state is not None and state.poll() is None:
            _stop(state, 20)
        operator_code = _stop(operator, 60)
        for handle in handles:
            handle.close()
    if failure:
        result = {"status": "FAIL", "group": group,
                  "attempt_dir_relative": str(attempt_dir.relative_to(output_root)),
                  "failure": failure, "operator_exit_code": operator_code}
        _write_probe_report(attempt_dir, result)
        return result

    def fail(reason: str) -> dict:
        result = {"status": "FAIL", "group": group,
                  "attempt_dir_relative": str(attempt_dir.relative_to(output_root)),
                  "failure": reason, "operator_exit_code": operator_code}
        _write_probe_report(attempt_dir, result)
        return result

    raw_summary_path = attempt_dir / "raw_capture_summary.json"
    state_summary_path = attempt_dir / "state_capture_summary.json"
    camera_truth_path = attempt_dir / "camera_truth.json"
    try:
        raw_summary = json.loads(raw_summary_path.read_text(encoding="utf-8"))
        state_summary = json.loads(state_summary_path.read_text(encoding="utf-8"))
        camera_payloads = json.loads(camera_truth_path.read_text(encoding="utf-8"))["camera_payloads"]
    except (OSError, ValueError, KeyError) as exc:
        return fail(f"short-capture final artifacts are unreadable: {type(exc).__name__}: {exc}")
    if raw_summary.get("status") != "CAPTURE_COMPLETE" or state_summary.get("status") != "STATE_CAPTURE_COMPLETE":
        return fail(f"short-capture finalization invalid for {group}")
    if float(state_summary.get("gt_observed_hz") or 0.0) < 49.5:
        return fail(f"preflight GT rate below 50 Hz for {group}")
    if set(camera_payloads) != {"left", "right"}:
        return fail(f"camera pose diagnostics missing a side for {group}")
    if workbook_source is None:
        expected_centers = {"left": spec["camera"]["left_center_world_m"],
                            "right": spec["camera"]["right_center_world_m"]}
        expected_rotations = {
            side: _expected_rwc(spec, group, side) for side in ("left", "right")
        }
    else:
        from campaigns.stereo_v1_1.workbook_campaign import _camera_rotation
        geometry = workbook_source["parameters"]["CameraGeometry"]
        pose = workbook_source["parameters"]["RenderedCameraPose"]
        expected_centers = {
            "left": [geometry["left_east_m"], geometry["left_north_m"], geometry["left_up_m"]],
            "right": [geometry["right_east_m"], geometry["right_north_m"], geometry["right_up_m"]],
        }
        expected_rotations = {
            "left": _camera_rotation(pose["left_roll_deg"], pose["left_pitch_deg"], pose["left_heading_deg"]),
            "right": _camera_rotation(pose["right_roll_deg"], pose["right_pitch_deg"], pose["right_heading_deg"]),
        }
    position_errors, orientation_errors = {}, {}
    for side in ("left", "right"):
        payload = camera_payloads[side]
        observed = payload["observed_optical_position_world_enu_m"]
        position_error = math.sqrt(sum((float(observed[i]) - float(expected_centers[side][i])) ** 2
                                       for i in range(3)))
        # The recorded quaternion is world-from-Gazebo-camera-link FLU. Convert
        # its local axes into OpenCV optical RDF before comparing with R_WC.
        actual_rotation = optical_rwc_from_entity_quaternion_xyzw(
            payload["observed_optical_orientation_world_from_entity_xyzw"]
        )
        orientation_error = _orientation_error_deg(actual_rotation, expected_rotations[side])
        position_errors[side] = position_error
        orientation_errors[side] = orientation_error
        if position_error > 1e-4 or orientation_error > 0.05:
            return fail(f"physical {group}/{side} camera pose differs from protocol")
    raw_bytes = sum(path.stat().st_size for path in (attempt_dir / "raw_images_or_lossless_bag").rglob("*")
                    if path.is_file())
    counts = {side: int(raw_summary["streams"][side]["topic_counts"].get(f"/{side}_camera/image", 0))
              for side in ("left", "right")}
    if min(counts.values()) < 50:
        return fail(f"short capture recorded too few frames for {group}: {counts}")
    if operator_code not in (0, 130):
        return fail(f"operator stack stopped unexpectedly after probe: {operator_code}")
    result = {
        "status": "PASS", "group": group,
        "attempt_dir_relative": str(attempt_dir.relative_to(output_root)),
        "duration_s": 5.0, "raw_bytes": raw_bytes,
        "compressed_raw_bytes_per_second": raw_bytes / 5.0,
        "camera_image_message_counts": counts,
        "gt_observed_hz": state_summary["gt_observed_hz"],
        "gt_gap_ge_40ms_count": state_summary["gt_gap_ge_40ms_count"],
        "camera_position_error_m": position_errors,
        "camera_orientation_error_deg": orientation_errors,
        "camera_truth_sha256": hashlib.sha256(camera_truth_path.read_bytes()).hexdigest(),
        "operator_exit_code": operator_code,
    }
    _write_probe_report(attempt_dir, result)
    return result
