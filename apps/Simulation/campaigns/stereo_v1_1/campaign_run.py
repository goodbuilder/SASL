#!/usr/bin/env python3
"""Serial v1.1 SITL campaign runner with immutable attempts and source QC.

Commands: plan, run, all, resume, qc. The runner starts at most one Gazebo/PX4
stack and never invokes the legacy optical-OFAT sweep. V0 source/structure QC
is allowed before method freeze; V0 location-performance evaluation is not.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import importlib.util
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path


SIM_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = SIM_ROOT.parents[1]
CAMPAIGN_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.campaign_status import begin_attempt, finish_attempt
from campaigns.stereo_v1_1.import_yolo_weights import check_owned_model
from campaigns.stereo_v1_1.plan_campaign import build_plan
from campaigns.stereo_v1_1.qc_attempt import validate as validate_attempt
from campaigns.stereo_v1_1.workbook_campaign import (
    reference_from_parameters,
    scenario_environment,
    write_scenario_environment,
)


PILOT_IDS = ("G0-P0-R01", "G0-P4-R01", "G0-V0-R01", "G1-P4-R01")
ROOT_PYTHON = Path(os.environ.get(
    "STEREO_V1_1_ROOT_PYTHON",
    str(REPO_ROOT / ".venv" / "bin" / "python" if (REPO_ROOT / ".venv" / "bin" / "python").is_file() else sys.executable),
)).expanduser()
YOLO_PYTHON = Path(os.environ.get(
    "STEREO_V1_1_YOLO_PYTHON", str(SIM_ROOT / ".venv" / "bin" / "python")
)).expanduser()
FROZEN_FILES = (
    "campaign_run.py", "campaign_status.py", "capture_streams.py", "state_recorder.py",
    "flight_controller.py", "trajectory_contract.py", "flight_gates.py",
    "campaign_setpoints.py", "runtime_observation.py", "ros_observation_bridge.py",
    "bag_index.py", "postprocess_detections.py", "gt_alignment.py", "align_gt.py",
    "qc_attempt.py", "model_manifest.json", "workbook_campaign.py",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_new(path: Path, payload: dict) -> None:
    with path.open("x", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())


def _plan_rows() -> list[dict]:
    plan = build_plan()
    by_id = {row["run_id"]: row for row in plan["planned_runs"]}
    return [by_id[name] for name in PILOT_IDS] + [
        row for row in plan["planned_runs"] if row["run_id"] not in PILOT_IDS
    ]


def _source_manifest(run_id: str) -> dict:
    rows = json.loads((CAMPAIGN_ROOT / "protocol" / "04_run_manifest.json").read_text(encoding="utf-8-sig"))["runs"]
    matches = [row for row in rows if row["run_id"] == run_id]
    if len(matches) != 1:
        raise ValueError(f"run_id not in frozen 39 slots: {run_id}")
    return matches[0]


def _reference(path_id: str | dict) -> dict:
    if isinstance(path_id, dict) and "parameters" in path_id:
        return reference_from_parameters(path_id["parameters"])
    path = CAMPAIGN_ROOT / "protocol" / "05_generate_reference.py"
    spec = importlib.util.spec_from_file_location("stereo_v1_1_reference", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    protocol = json.loads((CAMPAIGN_ROOT / "protocol" / "03_experiment_spec.json").read_text(encoding="utf-8-sig"))
    motion = protocol["motion"]
    return module.make_path(
        protocol["trajectories"][path_id],
        math.radians(protocol["scope"]["current_glide_slope_deg"]),
        motion["cruise_speed_3d_mps"], motion["speed_ramp_s"],
        motion["reference_update_hz"],
    )


def _code_hashes() -> dict[str, str]:
    return {name: _sha256(CAMPAIGN_ROOT / name) for name in FROZEN_FILES}


def _effective_config(source: dict, attempt_id: str, reference_path: Path,
                      development: bool, output_root: Path,
                      scenario_path: Path | None = None,
                      parameter_path: Path | None = None) -> dict:
    if "parameters" in source:
        seed_policy = source.get("seed_policy", "none")
        seed_effective = None
        if seed_policy == "manifest_required":
            seed_effective = {
                "gazebo_cli": True,
                "gazebo_sensor_noise_source": "gz sim --seed via SIM_GAZEBO_SEED",
                "px4_sensor_rng_verified": False,
                "independent_repetition_verified": False,
            }
        scenario_path = scenario_path or reference_path.parent / "effective_scenario.env"
        parameter_path = parameter_path or reference_path.parent / "workbook_parameters.json"
        return {
            "schema_version": "stereo_v1_1_workbook_effective_config_v1",
            "document_version": "workbook_v1_1",
            "sequence": source["sequence"],
            "run_id": source["run_id"],
            "attempt_id": attempt_id,
            "role": source["role"],
            "repetition": source["repetition"],
            "seed_policy": seed_policy,
            "seed_requested": source.get("seed_requested"),
            "seed_effective": seed_effective,
            "seed_retry_policy": source.get("seed_retry_policy"),
            "workbook_sha256": source["workbook_sha256"],
            "environment_report_sha256": (
                _sha256(output_root / "environment_report.json")
                if (output_root / "environment_report.json").is_file() else None
            ),
            "gazebo_world_name": _workbook_environment_value(source, "world_name"),
            "workbook_parameters_sha256": _sha256(parameter_path),
            "scenario_environment_from_attempt": str(scenario_path.relative_to(reference_path.parent)),
            "scenario_environment_sha256": _sha256(scenario_path),
            "reference_sha256": _sha256(reference_path),
            "reference_length_3d_m": json.loads(reference_path.read_text(encoding="utf-8"))["length_3d_m"],
            "reference_duration_s": json.loads(reference_path.read_text(encoding="utf-8"))["science_duration_s"],
            "camera_center_frame": "gazebo_world_enu:east,north,up_m",
            "camera_ray_frame": "opencv_optical_rdf:right,down,forward",
            "flight_control_frame": "px4_local_ned:north,east,down_m",
            "world_to_local_origin_source": "runtime synchronized GT/PX4 observations, not configured spawn",
            "target_model_name": source["parameters"]["Vehicle"].get("model_id", "x500_2m"),
            "vehicle_reference_link": source["parameters"]["Vehicle"].get("reference_link", "vehicle_reference_link"),
            "operator_observation_mode": "ideal_reference_point_visualization_only",
            "actual_yolo_mode": "offline_original_frame",
            "px4_source_from_repo_root": "pinned_runtime_checked_by_preflight",
            "campaign_code_sha256": _code_hashes(),
            "development": development,
        }
    group = source["rig_group"]
    profile = SIM_ROOT / "config" / "campaign" / "v1_1" / f"{group}.env"
    model = json.loads((CAMPAIGN_ROOT / "model_manifest.json").read_text())
    px4_root = Path(os.environ.get("PX4_DIR", SIM_ROOT / "PX4-Autopilot")).expanduser().resolve()
    binary = px4_root / "build" / "px4_sitl_default" / "bin" / "px4"
    if not binary.is_file():
        raise FileNotFoundError("pinned PX4 SITL binary is missing")
    if subprocess.check_output(["git", "-C", str(px4_root), "describe", "--tags", "--exact-match", "HEAD"], text=True).strip() != "v1.17.0":
        raise ValueError("PX4 HEAD differs from v1.17.0")
    base_config = Path(os.environ.get("SIM_CONFIG", SIM_ROOT / "config" / "simulation.env")).expanduser().resolve()
    try:
        base_config_from_sim_root = str(base_config.relative_to(SIM_ROOT))
    except ValueError:
        # External config files are allowed as runtime input, but their absolute
        # locations are not persisted as reopenable campaign dependencies.
        base_config_from_sim_root = "external_machine_local_config"
    environment_report = output_root / "environment_report.json"
    return {
        "schema_version": "stereo_v1_1_effective_config_v1",
        "document_version": "1.1",
        "source_manifest_run_id": source["run_id"],
        "run_id": f"DEVELOPMENT-{group}-{source['path_id']}" if development else source["run_id"],
        "attempt_id": attempt_id,
        "rig_group": group,
        "path_id": source["path_id"],
        "seed_requested": source["seed_requested"],
        "seed_effective": {
            "gazebo_cli": True,
            "gazebo_sensor_noise_source": "gz sim --seed",
            "px4_sensor_rng_verified": False,
            "independent_repetition_verified": False,
        },
        "profile_from_sim_root": str(profile.relative_to(SIM_ROOT)),
        "profile_sha256": _sha256(profile),
        "simulation_config_from_sim_root": base_config_from_sim_root,
        "simulation_config_sha256": _sha256(base_config) if base_config.is_file() else None,
        "environment_report_sha256": _sha256(environment_report) if environment_report.is_file() else None,
        "reference_sha256": _sha256(reference_path),
        "camera_center_frame": "gazebo_world_enu:east,north,up_m",
        "camera_ray_frame": "opencv_optical_rdf:right,down,forward",
        "flight_control_frame": "px4_local_ned:north,east,down_m",
        "world_to_local_origin_source": "20 synchronized runtime GT/PX4 observations, not configured spawn",
        "target_model_name": "x500_2m",
            "vehicle_reference_link": "vehicle_reference_link",
        "operator_observation_mode": "ideal_reference_point_visualization_only",
        "actual_yolo_mode": "offline_original_frame",
        "yolo_model_sha256": model["sha256"],
        "yolo_runtime_version": model["ultralytics_version"],
        "yolo_confidence": model["confidence"],
        "yolo_nms_iou": model["nms_iou"],
        "yolo_imgsz_px": model["inference_imgsz_px"],
        "yolo_device": os.environ.get("STEREO_V1_1_YOLO_DEVICE", "cuda:0"),
        "px4_version": "v1.17.0",
        "px4_source_from_repo_root": (
            str(px4_root.relative_to(REPO_ROOT)) if px4_root.is_relative_to(REPO_ROOT)
            else "external_machine_local_px4_source"
        ),
        "px4_binary_sha256": _sha256(binary),
        "campaign_code_sha256": _code_hashes(),
        "development": development,
    }


def _workbook_environment_value(source: dict, setting_id: str) -> str:
    matches = [row["declared_value"] for row in source.get("environment_contract", [])
               if row.get("setting_id") == setting_id]
    if not matches and setting_id == "world_name":
        return "stereo_ground_drone"
    if len(matches) != 1:
        raise ValueError(f"workbook EnvironmentContract requires exactly one {setting_id!r} row")
    return matches[0]


def _existing_sitl_processes() -> list[int]:
    found = []
    self_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == self_pid:
            continue
        try:
            fields = (entry / "cmdline").read_bytes().split(b"\0")
        except (OSError, PermissionError):
            continue
        command = b" ".join(fields)
        if any(marker in command for marker in (
            b"run_operator_view.sh", b"gz sim -", b"px4_sitl_default/bin/px4",
        )):
            found.append(int(entry.name))
    return found


def _wait_token(process: subprocess.Popen, log_path: Path, token: str,
                timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if token in log_path.read_text(encoding="utf-8", errors="replace"):
            if process.poll() is None:
                return
            raise RuntimeError(f"process exited after {token}: {process.returncode}")
        if process.poll() is not None:
            tail = log_path.read_text(encoding="utf-8", errors="replace")[-1200:]
            raise RuntimeError(f"process exited before {token}: {process.returncode}: {tail}")
        time.sleep(0.5)
    raise TimeoutError(f"process did not report {token} within {timeout_s} seconds")


def _spawn(command: list[str], log_path: Path, *, env=None) -> tuple[subprocess.Popen, object]:
    handle = log_path.open("x", encoding="utf-8")
    try:
        process = subprocess.Popen(command, cwd=REPO_ROOT, env=env,
                                   stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True, text=True)
    except BaseException:
        handle.close()
        raise
    return process, handle


def _ros_command(python: Path, script: Path, *arguments: str) -> list[str]:
    command = [str(python), str(script), *arguments]
    # All values are supplied as argv to subprocess; only the static ROS setup
    # statement is interpreted by bash. No user text is interpolated into it.
    return ["bash", "-c", 'source "${SASL_ROS_WORKSPACE:-workspace/main}/install/setup.bash"\nexec "$@"', "_", *command]


def _stop_process(process: subprocess.Popen | None, timeout_s: float) -> int | None:
    if process is None:
        return None
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            return process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait(timeout=5)


def _run_checked(command: list[str], log_path: Path, timeout_s: float = 900) -> None:
    with log_path.open("x", encoding="utf-8") as handle:
        subprocess.run(command, cwd=REPO_ROOT, stdout=handle,
                       stderr=subprocess.STDOUT, timeout=timeout_s, check=True)


def _copy_verified(source: Path, target: Path) -> None:
    if target.exists():
        raise FileExistsError(target)
    shutil.copyfile(source, target)
    if _sha256(source) != _sha256(target):
        raise ValueError(f"derived artifact copy differs: {target.name}")


def _checksums(attempt_dir: Path) -> Path:
    destination = attempt_dir / "checksums.sha256"
    with destination.open("x", encoding="utf-8") as handle:
        for path in sorted(attempt_dir.rglob("*")):
            if path.is_file() and path != destination and path.name not in ("qc_report.json", "runner_result.json"):
                handle.write(f"{_sha256(path)}  {path.relative_to(attempt_dir)}\n")
        handle.flush()
        os.fsync(handle.fileno())
    return destination


def _append_checksum(attempt_dir: Path, path: Path) -> None:
    with (attempt_dir / "checksums.sha256").open("a", encoding="utf-8") as handle:
        handle.write(f"{_sha256(path)}  {path.relative_to(attempt_dir)}\n")
        handle.flush()
        os.fsync(handle.fileno())


def _attempt_path(output_root: Path, source: dict, development: bool) -> tuple[str, Path]:
    if not development:
        record, path = begin_attempt(output_root, source["run_id"])
        return record["attempt_id"], path
    parent = output_root / "개발시험"
    parent.mkdir(parents=True, exist_ok=True)
    for number in range(1, 100):
        attempt_id = f"a{number:02d}"
        path = parent / f"runner_{source['run_id']}_{attempt_id}"
        if not path.exists():
            path.mkdir()
            return attempt_id, path
    raise RuntimeError("development attempt ID space exhausted")


def _prepare_attempt(output_root: Path, source: dict, development: bool) -> tuple[str, Path, str]:
    attempt_id, attempt_dir = _attempt_path(output_root, source, development)
    workbook_source = "parameters" in source
    run_id = (source["run_id"] if workbook_source else
              f"DEVELOPMENT-{source['rig_group']}-{source['path_id']}" if development else source["run_id"])
    reference_path = attempt_dir / "flight_reference.json"
    _write_new(reference_path, _reference(source if workbook_source else source["path_id"]))
    if workbook_source:
        parameter_path = attempt_dir / "workbook_parameters.json"
        _write_new(parameter_path, source["parameters"])
        parameter_sources_path = attempt_dir / "workbook_parameter_sources.json"
        _write_new(parameter_sources_path, source["parameter_sources"])
        scenario_path = attempt_dir / "effective_scenario.env"
        write_scenario_environment(scenario_path, source["parameters"], source["sequence"])
        manifest = {
            "schema_version": "stereo_v1_1_workbook_run_manifest_v1",
            "sequence": source["sequence"], "run_id": run_id,
            "attempt_id": attempt_id, "repetition": source["repetition"],
            "role": source["role"],
            "seed_requested": source.get("seed_requested"),
            "use_for_calibration": source["use_for_calibration"],
            "use_for_final_evaluation": source["use_for_final_evaluation"],
            "vehicle_model_id": source["parameters"]["Vehicle"].get("model_id", "x500_2m"),
            "vehicle_reference_link": source["parameters"]["Vehicle"].get("reference_link", "vehicle_reference_link"),
            "gazebo_world_name": _workbook_environment_value(source, "world_name"),
            "data_split": source.get("data_split"),
            "manifest_attempt_id": source.get("attempt_id"),
            "path_shape": source["shape"],
            "workbook_sha256": source["workbook_sha256"],
            "workbook_schema_version": source.get("workbook_schema_version"),
            "workbook_parameters_file": "workbook_parameters.json",
            "workbook_parameter_sources_file": "workbook_parameter_sources.json",
            "seed_policy": source.get("seed_policy", "none"),
            "seed_requested": source.get("seed_requested"),
        }
    else:
        manifest = {**source, "source_manifest_run_id": source["run_id"],
                    "run_id": run_id, "attempt_id": attempt_id,
                    "status": "DEVELOPMENT" if development else "RUNNING"}
    _write_new(attempt_dir / "run_manifest.json", manifest)
    _write_new(attempt_dir / "effective_config.json",
               _effective_config(source, attempt_id, reference_path, development, output_root,
                                 scenario_path if workbook_source else None,
                                 parameter_path if workbook_source else None))
    return attempt_id, attempt_dir, run_id


def _postprocess(output_root: Path, attempt_dir: Path, run_id: str,
                 attempt_id: str, source: dict | str) -> None:
    workbook_source = isinstance(source, dict) and "parameters" in source
    parameters = source["parameters"] if workbook_source else None
    capture = parameters["Capture"] if parameters else {}
    index_script = CAMPAIGN_ROOT / "bag_index.py"
    _run_checked(_ros_command(ROOT_PYTHON, index_script,
                              "--attempt-dir", str(attempt_dir),
                              "--run-id", run_id, "--attempt-id", attempt_id,
                              "--pair-tolerance-s", str(capture.get("pair_tolerance_s", 0.001))),
                 attempt_dir / "index_processing.log")
    derived = output_root / "파생_재처리" / run_id / attempt_id
    derived.mkdir(parents=True, exist_ok=False)
    yolo = derived / "yolo-8.4.127"
    yolo_script = CAMPAIGN_ROOT / "postprocess_detections.py"
    _run_checked(_ros_command(YOLO_PYTHON, yolo_script,
                              "--attempt-dir", str(attempt_dir),
                              "--output-dir", str(yolo),
                              "--model", str(SIM_ROOT / "yolov8n_uav.pt"),
                              "--device", os.environ.get("STEREO_V1_1_YOLO_DEVICE", "cuda:0"),
                              "--confidence", str(capture.get("yolo_confidence", 0.1)),
                              "--nms-iou", str(capture.get("yolo_nms_iou", 0.7)),
                              "--imgsz", str(capture.get("yolo_imgsz_px", 736)),
                              "--target-hz", str(capture.get("yolo_target_hz", 20.0)),
                              "--parameter-source", "manifest_xlsx" if workbook_source else "pinned_model_manifest"),
                 attempt_dir / "yolo_processing.log", timeout_s=1800)
    gt = derived / "gt-alignment"
    gt_script = CAMPAIGN_ROOT / "align_gt.py"
    gt_arguments = ["--attempt-dir", str(attempt_dir), "--output-dir", str(gt),
                    "--max-bracketing-gap-s", str(capture.get("max_gt_bracketing_gap_s", 0.04))]
    reveal_tracking = (source["use_for_calibration"] if workbook_source else source != "V0")
    if reveal_tracking:
        gt_arguments.append("--reveal-tracking")
    _run_checked(_ros_command(ROOT_PYTHON, gt_script, *gt_arguments),
                 attempt_dir / "gt_processing.log")
    for filename in ("observations_left.jsonl", "observations_right.jsonl",
                     "paired_observations.jsonl", "observation_summary.json"):
        _copy_verified(yolo / filename, attempt_dir / filename)
    _copy_verified(gt / "gt_alignment_summary.json",
                   attempt_dir / "gt_alignment_summary.json")


def _apply_requested_seed(env: dict[str, str], source: dict,
                          *, workbook_source: bool) -> dict[str, str]:
    """Apply only the seed policy declared by the campaign input."""
    env.pop("SIM_GAZEBO_SEED", None)
    if workbook_source and source.get("seed_policy", "none") == "none":
        return env
    requested = source.get("seed_requested")
    if isinstance(requested, bool) or not isinstance(requested, int) or requested < 0:
        raise ValueError("campaign requires a nonnegative integer seed_requested")
    env["SIM_GAZEBO_SEED"] = str(requested)
    return env


def _run_one(output_root: Path, source: dict, *, development: bool = False) -> dict:
    workbook_source = "parameters" in source
    report_path = output_root / "environment_report.json"
    if not report_path.is_file():
        raise RuntimeError("environment_report.json missing; run campaign_preflight.py before SITL")
    environment_report = json.loads(report_path.read_text(encoding="utf-8"))
    if environment_report.get("status") != "PREFLIGHT_PASS":
        raise RuntimeError("campaign preflight is not passing; refusing to start SITL")
    if (workbook_source
            and environment_report.get("workbook_sha256") != source.get("workbook_sha256")):
        raise RuntimeError("preflight workbook hash differs from the frozen run parameters")
    check_owned_model()
    if not ROOT_PYTHON.is_file() or not YOLO_PYTHON.is_file():
        raise RuntimeError("root and isolated pinned YOLO Python environments are required")
    if shutil.disk_usage(output_root).free < 10_000_000_000:
        raise RuntimeError("less than 10 GB free before a raw-image run")
    active = _existing_sitl_processes()
    if active:
        raise RuntimeError(f"another Gazebo/PX4/operator session is active: {active[:8]}")
    attempt_id, attempt_dir, run_id = _prepare_attempt(output_root, source, development)
    print(f"Starting {run_id}/{attempt_id} ({source['rig_group']} {source['path_id']})", flush=True)
    operator = state = images = controller = None
    handles = []
    reason = None
    controller_code = None
    try:
        env = os.environ.copy()
        # The control panel may be launched from a shell with stale exported
        # values. The campaign's caller-owned .env and per-run G0/G1 profile
        # must be the sole sources for these simulation settings.
        workbook_env = scenario_environment(source["parameters"], source["sequence"]) if workbook_source else {}
        vehicle_model_id = (source["parameters"]["Vehicle"].get("model_id", "x500_2m")
                            if workbook_source else "x500_2m")
        for name in tuple(env):
            if name.startswith(("CAMERA_", "CAM_L_", "CAM_R_", "COMMON_OPTICAL_",
                                "RELATIVE_OPTICAL_", "DRONE_", "WAYPOINT_", "YOLO_",
                                "SWEEP_", "AUTO_", "SIM_SCENARIO_", "SIM_TARGET_",
                                "SIM_RANGE_", "SIM_ZOOM_")) or name in workbook_env or name in {
                                    "ENABLE_GPS_BASELINE_TRIANGULATION", "GPS_BASELINE_MAX_AGE_S",
                                    "PX4_ATTITUDE_MAX_AGE_S", "PX4_ATTITUDE_PAIR_SKEW_S",
                                    "PX4_GZ_WORLD",
                                }:
                env.pop(name, None)
        env["SIM_CONFIG"] = os.environ.get("SIM_CONFIG", str(SIM_ROOT / "config" / "simulation.env"))
        env["SIM_SCENARIO_CONFIG"] = (str(attempt_dir / "effective_scenario.env") if workbook_source
                                      else f"config/campaign/v1_1/{source['rig_group']}.env")
        if workbook_source:
            env["PX4_GZ_WORLD"] = _workbook_environment_value(source, "world_name")
        _apply_requested_seed(env, source, workbook_source=workbook_source)
        operator_command = [
            str(SIM_ROOT / "run_operator_view.sh"), "--headless", "--no-yolo",
            "--raw-view", "--skip-px4-build", "--px4-model-name", vehicle_model_id,
            "observation_mode:=ideal_reference_point", "launch_rviz:=false", "gz_gui:=false",
        ]
        operator, handle = _spawn(operator_command, attempt_dir / "runtime.log", env=env)
        handles.append(handle)
        _wait_token(operator, attempt_dir / "runtime.log", "Operator view started.", 240)

        state_command = _ros_command(ROOT_PYTHON, CAMPAIGN_ROOT / "state_recorder.py",
                                     "--attempt-dir", str(attempt_dir))
        state, handle = _spawn(state_command, attempt_dir / "state_recorder.log")
        handles.append(handle)
        _wait_token(state, attempt_dir / "state_recorder.log", "STATE_READY", 45)

        image_command = _ros_command(ROOT_PYTHON, CAMPAIGN_ROOT / "capture_streams.py",
                                     "--attempt-dir", str(attempt_dir), "--compression", "zstd")
        images, handle = _spawn(image_command, attempt_dir / "capture_runner.log")
        handles.append(handle)
        _wait_token(images, attempt_dir / "capture_runner.log", "CAPTURE_READY", 45)

        controller_command = _ros_command(ROOT_PYTHON, CAMPAIGN_ROOT / "flight_controller.py",
                                          "--run-id", run_id,
                                          "--attempt-id", attempt_id,
                                          "--reference-json", str(attempt_dir / "flight_reference.json"),
                                          "--attempt-dir", str(attempt_dir))
        if workbook_source:
            controller_command.extend(("--run-config-json", str(attempt_dir / "workbook_parameters.json")))
        controller, handle = _spawn(controller_command, attempt_dir / "controller.log")
        handles.append(handle)
        reference_duration = json.loads((attempt_dir / "flight_reference.json").read_text())["science_duration_s"]
        frame_gate = source["parameters"]["FrameGate"] if workbook_source else {}
        entry = source["parameters"]["Entry"] if workbook_source else {}
        deadline_s = max(900.0, reference_duration + float(frame_gate.get("origin_timeout_s", 30.0))
                        + float(entry.get("entry_timeout_s", 300.0))
                        + float(entry.get("science_start_readiness_timeout_s", 120.0))
                        + float(entry.get("landing_timeout_s", 90.0)) + 180.0)
        deadline = time.monotonic() + deadline_s
        last_event = None
        while controller.poll() is None:
            if time.monotonic() > deadline:
                raise TimeoutError(f"flight controller exceeded {deadline_s:g} seconds")
            events_path = attempt_dir / "events.jsonl"
            if events_path.is_file():
                events = events_path.read_text(encoding="utf-8").splitlines()
                if events:
                    current_event = json.loads(events[-1])["event"]
                    if current_event != last_event:
                        print(f"{run_id}/{attempt_id}: {current_event}", flush=True)
                        last_event = current_event
            time.sleep(5)
        controller_code = controller.returncode
        print(f"{run_id}/{attempt_id}: controller exit {controller_code}", flush=True)
    except BaseException as exc:
        reason = f"{type(exc).__name__}: {exc}"
        print(f"{run_id}/{attempt_id}: {reason}", flush=True)
    finally:
        if controller is not None and controller.poll() is None:
            _stop_process(controller, 120)  # lets it request PX4 LAND
        # Preserve GT coverage through the final image bag message.
        image_code = _stop_process(images, 90)
        state_code = _stop_process(state, 30)
        operator_code = _stop_process(operator, 120)
        for handle in handles:
            handle.close()
        print(f"{run_id}/{attempt_id}: capture exits image={image_code} state={state_code}; operator={operator_code}", flush=True)
        if operator is not None and operator_code not in (0, 130) and reason is None:
            reason = f"operator stack exited with status {operator_code}"

    if reason is None and image_code == 0 and state_code == 0:
        try:
            _postprocess(output_root, attempt_dir, run_id, attempt_id,
                         source if workbook_source else source["path_id"])
        except BaseException as exc:
            reason = f"{type(exc).__name__}: postprocessing: {exc}"
            print(f"{run_id}/{attempt_id}: {reason}", flush=True)
    if reason is not None:
        _write_new(attempt_dir / "runner_failure.json", {"reason": reason})
    checksum_path = _checksums(attempt_dir)
    qc = validate_attempt(attempt_dir, run_id, attempt_id,
                          planned_source=source if workbook_source else None)
    if reason is not None and qc["status"] in ("VALID", "VALID_WITH_WARNINGS"):
        qc["status"] = "INFRASTRUCTURE_FAILED"
        qc["counted_in_39"] = False
        qc["counted_in_campaign"] = False
        qc["infrastructure_errors"].append("runner_lifecycle_failure")
    _write_new(attempt_dir / "qc_report.json", qc)
    _append_checksum(attempt_dir, attempt_dir / "qc_report.json")
    _write_new(attempt_dir / "runner_result.json", {
        "run_id": run_id, "attempt_id": attempt_id,
        "qc_status": qc["status"], "controller_exit_code": controller_code,
        "runner_failure_reason": reason,
    })
    _append_checksum(attempt_dir, attempt_dir / "runner_result.json")
    if not development:
        seed_effective = (
            None if workbook_source and source.get("seed_policy") != "manifest_required" else
            {"gazebo_cli": True,
             "gazebo_sensor_noise_source": "gz sim --seed via SIM_GAZEBO_SEED",
             "px4_sensor_rng_verified": False,
             "independent_repetition_verified": False}
        )
        finish_attempt(output_root, run_id, attempt_id, qc, seed_effective)
    print(f"{run_id}/{attempt_id}: QC {qc['status']}", flush=True)
    return qc


def _lock(output_root: Path):
    handle = (output_root / ".campaign.lock").open("a+")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError("another campaign runner holds the output lock")
    return handle


def _audit_valid(output_root: Path) -> dict:
    status = json.loads((output_root / "run_status.json").read_text())
    checked = 0
    for slot in status["run_slots"]:
        if slot["status"] not in ("VALID", "VALID_WITH_WARNINGS"):
            continue
        attempt = slot["attempts"][-1]
        path = output_root / attempt["recording_path"]
        current = validate_attempt(
            path, slot["run_id"], attempt["attempt_id"],
            planned_source=slot if "parameters" in slot else None,
        )
        counted = current.get("counted_in_campaign", current.get("counted_in_39", False))
        if current["status"] != slot["status"] or not counted:
            raise RuntimeError(f"completed run fails fresh file/hash QC: {slot['run_id']}")
        checked += 1
    return {"planned": status["planned_run_count"], "valid_readback": checked,
            "status_counts": json.loads((output_root / "qc_summary.json").read_text())["status_counts"]}


def _reconcile_interrupted(output_root: Path, slot: dict) -> None:
    """Close an abandoned RUNNING attempt while preserving every source file.

    A completed QC/runner result can be committed after an interrupted status
    update. Anything less is a technical failure, never an inferred valid run.
    The caller holds the exclusive campaign lock and has verified no live SITL.
    """
    attempt = slot["attempts"][-1]
    run_id, attempt_id = slot["run_id"], attempt["attempt_id"]
    path = output_root / attempt["recording_path"]
    report_path = path / "qc_report.json"
    result_path = path / "runner_result.json"
    checksum_path = path / "checksums.sha256"
    complete = all(item.is_file() for item in (report_path, result_path, checksum_path))
    if complete:
        listed = set()
        for line in checksum_path.read_text(encoding="utf-8").splitlines():
            parts = line.split("  ", 1)
            if len(parts) == 2:
                listed.add(parts[1])
        complete = {"qc_report.json", "runner_result.json"}.issubset(listed)
    if complete:
        saved = json.loads(report_path.read_text(encoding="utf-8"))
        result = json.loads(result_path.read_text(encoding="utf-8"))
        fresh = validate_attempt(path, run_id, attempt_id,
                                 planned_source=slot if "parameters" in slot else None)
        saved_counted = saved.get("counted_in_campaign", saved.get("counted_in_39"))
        fresh_counted = fresh.get("counted_in_campaign", fresh.get("counted_in_39"))
        complete = (saved["status"] == fresh["status"] == result["qc_status"]
                    and saved_counted == fresh_counted)
    if complete:
        qc = fresh
        reason = "completed_attempt_status_commit_interrupted"
    else:
        qc = {"run_id": run_id, "attempt_id": attempt_id,
              "status": "INFRASTRUCTURE_FAILED", "counted_in_39": False,
              "counted_in_campaign": False,
              "infrastructure_errors": ["runner_interrupted_before_complete_qc"],
              "protocol_errors": [], "warnings": []}
        reason = "incomplete_attempt_preserved_for_audit"
    marker = path / "resume_reconciliation.json"
    if not marker.exists():
        _write_new(marker, {"run_id": run_id, "attempt_id": attempt_id,
                            "reconciliation": reason, "qc_status": qc["status"]})
        if checksum_path.is_file():
            _append_checksum(path, marker)
    effective_path = path / "effective_config.json"
    effective_seed = None
    if effective_path.is_file():
        effective_seed = json.loads(effective_path.read_text(encoding="utf-8")).get("seed_effective")
    finish_attempt(output_root, run_id, attempt_id, qc, effective_seed)
    print(f"{run_id}/{attempt_id}: resume reconciled {qc['status']} ({reason})", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("plan", "run", "all", "resume", "qc"))
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-id")
    parser.add_argument("--development", action="store_true")
    args = parser.parse_args()
    if args.development and args.command != "run":
        parser.error("--development is only supported for one selected run")
    output_root = args.output_root.resolve()
    if not output_root.is_dir():
        parser.error("output root does not exist")
    if args.command == "plan":
        print(json.dumps({"status": "PLAN_ONLY", "order": [row["run_id"] for row in _plan_rows()]},
                         ensure_ascii=False))
        return 0
    if args.command == "qc":
        print(json.dumps(_audit_valid(output_root), ensure_ascii=False))
        return 0
    if not (output_root / "run_status.json").is_file():
        parser.error("run_status.json missing; initialize campaign status first")
    with _lock(output_root):
        if args.command == "run":
            if not args.run_id:
                parser.error("run requires --run-id")
            qc = _run_one(output_root, _source_manifest(args.run_id),
                          development=args.development)
            if args.development:
                unexpected_protocol = [item for item in qc["protocol_errors"]
                                       if item != "run_id_not_in_39_slot_manifest"]
                return 0 if not qc["infrastructure_errors"] and not unexpected_protocol else 1
            return 0 if qc["status"] in ("VALID", "VALID_WITH_WARNINGS") else 1
        _audit_valid(output_root)
        if _existing_sitl_processes():
            raise RuntimeError("Gazebo/PX4 process remains active; stop it before resume")
        rows = _plan_rows()
        current = json.loads((output_root / "run_status.json").read_text())
        by_id = {row["run_id"]: row for row in current["run_slots"]}
        for row in rows:
            slot = by_id[row["run_id"]]
            if slot["status"] in ("VALID", "VALID_WITH_WARNINGS"):
                continue
            if slot["status"] == "RUNNING":
                _reconcile_interrupted(output_root, slot)
                slot = next(item for item in json.loads((output_root / "run_status.json").read_text())["run_slots"]
                            if item["run_id"] == row["run_id"])
                if slot["status"] in ("VALID", "VALID_WITH_WARNINGS"):
                    continue
            qc = _run_one(output_root, _source_manifest(row["run_id"]))
            if qc["status"] not in ("VALID", "VALID_WITH_WARNINGS"):
                raise RuntimeError(f"stopping serial campaign after {row['run_id']} {qc['status']}")
        print(json.dumps(_audit_valid(output_root), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
