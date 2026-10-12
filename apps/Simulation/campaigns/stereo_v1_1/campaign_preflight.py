#!/usr/bin/env python3
"""Non-flying preflight for the frozen v1.1 SITL campaign."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import shutil
import subprocess
import sys
import time
from xml.etree import ElementTree as ET
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = SIM_ROOT.parents[1]
CAMPAIGN_ROOT = Path(__file__).resolve().parent
OUTPUT_PARENT = SIM_ROOT / "recordings" / "stereo_v1_1"
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.plan_campaign import build_plan
from campaigns.stereo_v1_1.campaign_run import _existing_sitl_processes
from campaigns.stereo_v1_1.preflight_capture import run_group


MINIMUM_NEXT_RUN_BYTES = 10_000_000_000


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1].lower()


def _child_text(parent: ET.Element, tag: str) -> str | None:
    child = next((item for item in list(parent) if _local_name(item.tag) == tag), None)
    return None if child is None or child.text is None else child.text.strip()


def _world_environment_snapshot(path: Path) -> dict:
    root = ET.parse(path).getroot()
    worlds = [item for item in root.iter() if _local_name(item.tag) == "world"]
    if len(worlds) != 1:
        raise ValueError("world SDF must contain exactly one world element")
    world = worlds[0]
    scene = next((item for item in list(world) if _local_name(item.tag) == "scene"), None)
    atmosphere = next((item for item in list(world) if _local_name(item.tag) == "atmosphere"), None)
    lights = []
    materials = []
    wind_nodes = []
    weather_nodes = []
    wind_plugins = []
    weather_plugins = []
    for item in world.iter():
        tag = _local_name(item.tag)
        if tag == "light":
            lights.append({
                "name": item.attrib.get("name"),
                "type": item.attrib.get("type"),
                "pose": _child_text(item, "pose"),
                "cast_shadows": _child_text(item, "cast_shadows"),
                "diffuse": _child_text(item, "diffuse"),
                "specular": _child_text(item, "specular"),
                "direction": _child_text(item, "direction"),
            })
        elif tag == "visual":
            material = next((child for child in list(item)
                             if _local_name(child.tag) == "material"), None)
            if material is not None:
                materials.append({
                    "visual": item.attrib.get("name"),
                    "ambient": _child_text(material, "ambient"),
                    "diffuse": _child_text(material, "diffuse"),
                    "specular": _child_text(material, "specular"),
                })
        elif tag == "wind":
            wind_nodes.append(item.attrib.copy())
        elif tag in {"weather", "rain", "snow", "clouds"}:
            weather_nodes.append(tag)
        elif tag == "plugin":
            name = f"{item.attrib.get('name', '')} {item.attrib.get('filename', '')}".lower()
            if "wind" in name:
                wind_plugins.append(name.strip())
            if "weather" in name:
                weather_plugins.append(name.strip())
    return {
        "world_name": world.attrib.get("name"),
        "scene": None if scene is None else {
            "ambient": _child_text(scene, "ambient"),
            "background": _child_text(scene, "background"),
            "shadows": _child_text(scene, "shadows"),
        },
        "lights": lights,
        "visual_materials": materials,
        "atmosphere_type": None if atmosphere is None else atmosphere.attrib.get("type"),
        "wind_nodes": wind_nodes,
        "wind_plugins": wind_plugins,
        "weather_nodes": weather_nodes,
        "weather_plugins": weather_plugins,
    }


def evaluate_environment_contract(contract_rows: list[dict], sim_root: Path,
                                  runtime_world_name: str) -> dict:
    """Compare manifest-declared environment assets and conditions with the active workspace."""
    if not contract_rows:
        return {
            "status": "FAIL", "checks": [{
                "name": "environment_contract_present", "status": "FAIL",
                "detail": {"error": "workbook has no EnvironmentContract rows"},
            }], "observed_world": None,
        }

    by_setting = {row["setting_id"]: row for row in contract_rows}
    source_row = by_setting.get("source_world_asset")
    source_path = (sim_root / source_row["evidence_path_from_sim_root"]).resolve() if source_row else sim_root
    if not source_path.is_relative_to(sim_root.resolve()):
        source_path = sim_root
    try:
        observed_world = _world_environment_snapshot(source_path)
    except (OSError, ET.ParseError, ValueError) as exc:
        observed_world = None
        snapshot_error = str(exc)
    else:
        snapshot_error = None

    checks = []
    for row in contract_rows:
        setting = row["setting_id"]
        evidence_path = (sim_root / row["evidence_path_from_sim_root"]).resolve()
        safe_path = evidence_path.is_relative_to(sim_root.resolve())
        actual_sha256 = _sha256(evidence_path) if safe_path else None
        hash_matches = actual_sha256 == row["expected_sha256"]
        value_matches = True
        if setting == "world_name":
            value_matches = (
                observed_world is not None
                and observed_world["world_name"] == row["declared_value"]
                and runtime_world_name == row["declared_value"]
            )
        elif setting == "lighting_background":
            value_matches = observed_world is not None
        elif setting == "weather":
            expected_atmosphere = row["declared_value"].rsplit("=", 1)[-1]
            value_matches = (
                observed_world is not None
                and not observed_world["weather_nodes"]
                and not observed_world["weather_plugins"]
                and observed_world["atmosphere_type"] == expected_atmosphere
            )
        elif setting == "wind_setting":
            value_matches = (
                observed_world is not None
                and not observed_world["wind_nodes"]
                and not observed_world["wind_plugins"]
            )
        ok = safe_path and hash_matches and value_matches
        checks.append({
            "name": f"environment_contract_{setting}",
            "status": "PASS" if ok else "FAIL",
            "detail": {
                "declared_value": row["declared_value"],
                "evidence_path_from_sim_root": row["evidence_path_from_sim_root"],
                "expected_sha256": row["expected_sha256"],
                "actual_sha256": actual_sha256,
                "hash_matches": hash_matches,
                "runtime_world_name": runtime_world_name if setting == "world_name" else None,
                "snapshot_error": snapshot_error,
            },
        })
    return {
        "status": "PASS" if all(row["status"] == "PASS" for row in checks) else "FAIL",
        "checks": checks,
        "observed_world": observed_world,
    }


def _command_version(command: list[str], timeout_s: float = 10.0) -> dict:
    executable = shutil.which(command[0])
    if executable is None:
        return {"available": False, "version": None}
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=timeout_s, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"available": True, "version": None, "error": str(exc)}
    output = (result.stdout or result.stderr).strip().splitlines()
    return {"available": result.returncode == 0,
            "version": output[0][:240] if output else None,
            "exit_code": result.returncode}


def _reference_geometry() -> dict:
    source = CAMPAIGN_ROOT / "protocol" / "05_generate_reference.py"
    spec = importlib.util.spec_from_file_location("stereo_v1_1_reference", source)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load frozen reference generator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    protocol = json.loads((CAMPAIGN_ROOT / "protocol" / "03_experiment_spec.json").read_text(encoding="utf-8-sig"))
    motion = protocol["motion"]
    results = {}
    for path_id, trajectory in protocol["trajectories"].items():
        reference = module.make_path(
            trajectory,
            math.radians(protocol["scope"]["current_glide_slope_deg"]),
            motion["cruise_speed_3d_mps"], motion["speed_ramp_s"],
            motion["reference_update_hz"],
        )
        samples = reference["samples"]
        peak_acceleration = max(
            sum(float(component) ** 2 for component in row["acceleration_world_enu_mps2"]) ** 0.5
            for row in samples
        )
        if peak_acceleration > protocol["motion"]["preflight_desired_acceleration_limit_mps2"]:
            raise ValueError(f"{path_id} reference acceleration exceeds protocol preflight limit")
        results[path_id] = {
            "sample_count": len(samples),
            "duration_s": reference["science_duration_s"],
            "length_3d_m": reference["length_3d_m"],
            "maximum_acceleration_mps2": peak_acceleration,
            "position_frame": "world_enu:east,north,up_m",
            "sample_rate_hz": motion["reference_update_hz"],
        }
    return results


def collect(output_root: Path, action: str = "preflight",
            workbook_path: Path | None = None) -> dict:
    output_root = output_root.resolve()
    data_root_value = os.environ.get("SASL_WORKSTATION_DATA_ROOT", "").strip()
    data_root = Path(data_root_value).expanduser().resolve() if data_root_value else None
    allowed = output_root.is_relative_to(OUTPUT_PARENT.resolve()) or (
        data_root is not None and output_root.is_relative_to(data_root) and output_root != data_root
    )
    if not allowed:
        raise ValueError("campaign output must be under recordings/stereo_v1_1 or a child of SASL_WORKSTATION_DATA_ROOT")
    output_root.mkdir(parents=True, exist_ok=True)
    workbook_plan = None
    environment_contract_evidence = None
    if workbook_path is not None:
        from campaigns.stereo_v1_1.workbook_campaign import build_workbook_plan
        workbook_plan = build_workbook_plan(workbook_path)
        plan = None
        geometry = {}
    else:
        plan = build_plan()
        geometry = _reference_geometry()
    disk = shutil.disk_usage(output_root)
    ros_workspace = Path(os.environ.get("SASL_ROS_WORKSPACE", REPO_ROOT / "workspace" / "main")).expanduser().resolve()
    px4_root = Path(os.environ.get("PX4_DIR", SIM_ROOT / "PX4-Autopilot")).expanduser().resolve()
    px4_binary = px4_root / "build" / "px4_sitl_default" / "bin" / "px4"
    root_python = Path(os.environ.get("STEREO_V1_1_ROOT_PYTHON", sys.executable)).expanduser()
    yolo_python = Path(os.environ.get(
        "STEREO_V1_1_YOLO_PYTHON", str(SIM_ROOT / ".venv" / "bin" / "python")
    )).expanduser()
    if not yolo_python.is_absolute():
        yolo_python = SIM_ROOT / yolo_python
    yolo_python = Path(os.path.abspath(yolo_python))
    model_manifest = json.loads((CAMPAIGN_ROOT / "model_manifest.json").read_text(encoding="utf-8"))
    model_path = REPO_ROOT / model_manifest["owned_path_from_repo_root"]
    checks = []

    def record(name: str, ok: bool, detail, *, required: bool = True):
        checks.append({"name": name, "status": "PASS" if ok else ("FAIL" if required else "WARN"),
                       "detail": detail, "required": required})

    if workbook_plan is None:
        record("frozen_manifest_and_profiles", plan["planned_run_count"] == 39, {
            "planned_runs": plan["planned_run_count"], "group_counts": plan["group_counts"],
        })
        record("reference_geometry", len(geometry) == 6, geometry)
    else:
        record("numeric_workbook_plan", workbook_plan["planned_run_count"] > 0, {
            "workbook_sha256": workbook_plan["workbook_sha256"],
            "planned_run_count": workbook_plan["planned_run_count"],
            "run_order": [row["run_id"] for row in workbook_plan["planned_runs"]],
        })
        record("workbook_reference_geometry", all(
            row["reference_preview"]["peak_acceleration_mps2"]
            <= row["parameters"]["Motion"]["acceleration_limit_mps2"]
            and row["reference_preview"]["minimum_camera_edge_margin_px"] >= 32
            for row in workbook_plan["planned_runs"]
        ), {
            "run_count": workbook_plan["planned_run_count"],
            "minimum_camera_edge_margin_px": min(
                row["reference_preview"]["minimum_camera_edge_margin_px"]
                for row in workbook_plan["planned_runs"]
            ),
        })
        environment_contract_evidence = evaluate_environment_contract(
            workbook_plan.get("environment_contract", []), SIM_ROOT,
            runtime_world_name=os.environ.get("PX4_GZ_WORLD", "stereo_ground_drone"),
        )
        for check in environment_contract_evidence["checks"]:
            record(check["name"], check["status"] == "PASS", check["detail"])
    record("ros2_humble", Path("/opt/ros/humble/setup.bash").is_file(),
           {"setup": "/opt/ros/humble/setup.bash"})
    record("ros_workspace_overlay", (ros_workspace / "install" / "setup.bash").is_file(),
           {"workspace_from_repo_root": (str(ros_workspace.relative_to(REPO_ROOT))
                                         if ros_workspace.is_relative_to(REPO_ROOT)
                                         else "external_runtime_workspace")})
    record("simulation_overlay", (SIM_ROOT / "install" / "setup.bash").is_file(),
           {"path_from_repo_root": "apps/Simulation/install/setup.bash"})
    controller_python_ok = root_python.is_file()
    if controller_python_ok:
        try:
            result = subprocess.run([str(root_python), "-c",
                                     "import pymavlink, rclpy, px4_msgs, zstandard, numpy, yaml"],
                                    capture_output=True, text=True, timeout=15, check=False)
            controller_python_ok = result.returncode == 0
            controller_python_detail = {"python_version": _command_version([str(root_python), "--version"]),
                                        "required_imports_exit": result.returncode,
                                        "stderr": result.stderr.strip()[-500:]}
        except (OSError, subprocess.TimeoutExpired) as exc:
            controller_python_ok = False
            controller_python_detail = {"error": str(exc)}
    else:
        controller_python_detail = {"error": "controller interpreter does not exist"}
    record("root_python", controller_python_ok, controller_python_detail)
    yolo_runtime_ok = yolo_python.is_file()
    yolo_device = os.environ.get("STEREO_V1_1_YOLO_DEVICE", "cuda:0")
    yolo_backend = os.environ.get("YOLO_TORCH_BACKEND", "cuda")
    yolo_detail = {"path_from_sim_root": (str(yolo_python.relative_to(SIM_ROOT.resolve()))
                                          if yolo_python.is_relative_to(SIM_ROOT.resolve())
                                          else "external_runtime_python")}
    if yolo_runtime_ok:
        try:
            result = subprocess.run([str(yolo_python), "-c",
                                     "import numpy, scipy, zstandard, cv2, torch, ultralytics"],
                                    capture_output=True, text=True, timeout=30, check=False)
            yolo_runtime_ok = result.returncode == 0
            yolo_detail.update({"required_imports_exit": result.returncode,
                                "stderr": result.stderr.strip()[-500:]})
            if yolo_runtime_ok:
                device_check = subprocess.run(
                    [str(yolo_python), "-c",
                     "import sys,torch; d,b=sys.argv[1:3]; i=int(d.split(':',1)[1]) if d.startswith('cuda:') else 0; "
                     "cuda_ok=bool(torch.version.cuda) and torch.cuda.is_available() and i < torch.cuda.device_count(); "
                     "backend_ok=(b=='auto') or (b=='cuda' and bool(torch.version.cuda)) or (b=='cpu' and not torch.version.cuda); "
                     "device_ok=(d=='cpu' and b!='cuda') or (d.startswith('cuda') and cuda_ok and b!='cpu'); "
                     "ok=backend_ok and device_ok; torch.zeros(1,device=d) if ok else None; raise SystemExit(0 if ok else 1)",
                     yolo_device, yolo_backend],
                    capture_output=True, text=True, timeout=30, check=False,
                )
                yolo_runtime_ok = device_check.returncode == 0
                yolo_detail.update({"device": yolo_device,
                                    "torch_backend": yolo_backend,
                                    "selected_device_available": yolo_runtime_ok,
                                    "device_check_stderr": device_check.stderr.strip()[-500:]})
        except (OSError, subprocess.TimeoutExpired) as exc:
            yolo_runtime_ok = False
            yolo_detail["error"] = str(exc)
    record("isolated_yolo_python_and_device", yolo_runtime_ok, yolo_detail)
    px4_tag = _command_version([
        "git", "-C", str(px4_root), "describe", "--tags", "--exact-match", "HEAD",
    ]) if px4_root.is_dir() else {"available": False, "version": None}
    record("pinned_px4_source", px4_tag.get("version") == "v1.17.0" and px4_binary.is_file(), {
        "expected_tag": "v1.17.0", "binary_present": px4_binary.is_file(),
        "observed_tag": px4_tag.get("version"), "binary_sha256": _sha256(px4_binary),
    })
    record("pinned_yolo_model", model_path.is_file() and _sha256(model_path) == model_manifest["sha256"], {
        "path_from_repo_root": model_manifest["owned_path_from_repo_root"],
        "expected_sha256": model_manifest["sha256"], "observed_sha256": _sha256(model_path),
    })
    tools = {
        # `ros2 --version` is not a supported CLI verb in this Humble setup;
        # listing packages verifies the sourced CLI instead.
        "ros2": _command_version(["ros2", "pkg", "list"], timeout_s=30.0),
        "gz": _command_version(["gz", "sim", "--versions"]),
        "colcon": _command_version(["colcon", "--help"]),
        "micro_xrce_agent": {"available": shutil.which("MicroXRCEAgent") is not None},
    }
    record("runtime_commands", all(item.get("available") for item in tools.values()), tools)
    active = _existing_sitl_processes()
    record("no_existing_sitl_stack", not active, {"matching_process_ids": active})
    static_failures = [check["name"] for check in checks
                       if check["required"] and check["status"] != "PASS"]
    live_results = []
    storage_estimate = None
    full_space_required = action in {"preflight", "all", "all_results", "resume"}
    if not static_failures:
        if workbook_plan is None:
            spec = json.loads((CAMPAIGN_ROOT / "protocol" / "03_experiment_spec.json").read_text(encoding="utf-8-sig"))
            probes = [(group, lambda group=group: run_group(output_root, group, spec))
                      for group in ("G0", "G1")]
        else:
            first_run = workbook_plan["planned_runs"][0]
            probes = [(first_run["run_id"],
                       lambda row=first_run: run_group(output_root, row["run_id"],
                                                       workbook_source=row))]
        for group, probe in probes:
            try:
                result = probe()
            except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
                result = {"status": "FAIL", "group": group,
                          "failure": f"{type(exc).__name__}: {exc}"}
            live_results.append(result)
            record(f"live_short_capture_{group}", result.get("status") == "PASS", result)
        if all(result.get("status") == "PASS" for result in live_results):
            measured_bytes_per_second = max(
                float(row["compressed_raw_bytes_per_second"]) for row in live_results
            )
            if workbook_plan is None:
                manifest = json.loads((CAMPAIGN_ROOT / "protocol" / "04_run_manifest.json").read_text(encoding="utf-8-sig"))
                status_path = output_root / "run_status.json"
                pending_ids = {row["run_id"] for row in manifest["runs"]}
                if status_path.is_file():
                    saved_status = json.loads(status_path.read_text(encoding="utf-8"))
                    pending_ids = {row["run_id"] for row in saved_status["run_slots"]
                                   if row["status"] not in ("VALID", "VALID_WITH_WARNINGS")}
                pending_runs = [row for row in manifest["runs"] if row["run_id"] in pending_ids]
                science_seconds = sum(float(geometry[row["path_id"]]["duration_s"]) for row in pending_runs)
            else:
                status_path = output_root / "run_status.json"
                pending_ids = {row["run_id"] for row in workbook_plan["planned_runs"]}
                if status_path.is_file():
                    saved_status = json.loads(status_path.read_text(encoding="utf-8"))
                    pending_ids = {row["run_id"] for row in saved_status["run_slots"]
                                   if row["status"] not in ("VALID", "VALID_WITH_WARNINGS")}
                pending_runs = [row for row in workbook_plan["planned_runs"] if row["run_id"] in pending_ids]
                science_seconds = sum(float(row["reference_preview"]["science_duration_s"])
                                      for row in pending_runs)
                pixel_rate_multiplier = max((
                    float(row["parameters"]["CameraModel"]["image_width_px"])
                    * float(row["parameters"]["CameraModel"]["image_height_px"])
                    * float(row["parameters"]["CameraModel"]["camera_rate_hz"])
                    / (1280.0 * 720.0 * 20.0)
                    for row in pending_runs
                ), default=1.0)
                pixel_rate_multiplier = max(1.0, pixel_rate_multiplier)
            if workbook_plan is None:
                pixel_rate_multiplier = 1.0
            bytes_per_second = measured_bytes_per_second * pixel_rate_multiplier
            overhead_seconds = 60.0 * len(pending_runs)
            projected_bytes = int(math.ceil(bytes_per_second * (science_seconds + overhead_seconds) * 1.25))
            free_after_probe = shutil.disk_usage(output_root).free
            required_for_campaign = projected_bytes + MINIMUM_NEXT_RUN_BYTES
            storage_estimate = {
                "measured_compressed_bytes_per_second": measured_bytes_per_second,
                "estimated_compressed_bytes_per_second_after_workbook_scaling": bytes_per_second,
                "workbook_pixel_rate_multiplier": pixel_rate_multiplier,
                "pending_science_reference_seconds": science_seconds,
                "pending_run_count": len(pending_runs),
                "entry_and_finalization_allowance_seconds": overhead_seconds,
                "safety_factor": 1.25,
                "projected_capture_bytes": projected_bytes,
                "reserved_free_bytes_after_campaign": MINIMUM_NEXT_RUN_BYTES,
                "required_free_bytes_before_all": required_for_campaign,
                "observed_free_bytes_after_short_probes": free_after_probe,
            }
            record("space_for_full_campaign", free_after_probe >= required_for_campaign,
                   storage_estimate, required=full_space_required)
        else:
            record("space_for_full_campaign", False,
                   {"status": "NOT_MEASURED_BECAUSE_SHORT_CAPTURE_FAILED"},
                   required=full_space_required)
    else:
        record("live_short_captures", False,
               {"status": "SKIPPED", "static_blockers": static_failures}, required=False)
        record("space_for_full_campaign", False,
               {"status": "NOT_MEASURED_BECAUSE_STATIC_PREFLIGHT_FAILED"},
               required=full_space_required)

    remaining = shutil.disk_usage(output_root).free
    record("space_for_next_raw_attempt", remaining >= MINIMUM_NEXT_RUN_BYTES, {
        "free_bytes_after_preflight": remaining, "required_bytes": MINIMUM_NEXT_RUN_BYTES,
    })
    active_after = _existing_sitl_processes()
    record("preflight_stack_cleanup", not active_after,
           {"active_sitl_processes_after_probe": active_after})

    required_failures = [check["name"] for check in checks
                         if check["required"] and check["status"] != "PASS"]
    report = {
        "schema_version": "stereo_v1_1_environment_report_v1",
        "status": "PREFLIGHT_PASS" if not required_failures else "PREFLIGHT_BLOCKED",
        "validated_action": action,
        "collected_utc_ns": time.time_ns(),
        "repo_relative_workspace": "apps/Simulation",
        "output_root_identity": (
            {"base": "repo", "path_from_sim_root": str(output_root.relative_to(SIM_ROOT.resolve()))}
            if output_root.is_relative_to(SIM_ROOT.resolve()) else
            {"base": "SASL_WORKSTATION_DATA_ROOT", "path_from_data_root": str(output_root.relative_to(data_root))}
        ),
        "simulation_env": {
            "path_from_sim_root": str(Path(os.environ.get("SIM_CONFIG", SIM_ROOT / "config" / "simulation.env")).resolve().relative_to(SIM_ROOT.resolve()))
            if Path(os.environ.get("SIM_CONFIG", SIM_ROOT / "config" / "simulation.env")).resolve().is_relative_to(SIM_ROOT.resolve())
            else "external_machine_local_config",
            "sha256": _sha256(Path(os.environ.get("SIM_CONFIG", SIM_ROOT / "config" / "simulation.env")).resolve()),
        },
        "protocol_sha256": None if plan is None else plan["protocol_sha256"],
        "workbook_sha256": None if workbook_plan is None else workbook_plan["workbook_sha256"],
        "environment_contract": environment_contract_evidence,
        "frozen_profile_hashes": {} if plan is None else {
            name: profile["sha256"] for name, profile in plan["profiles"].items()
        },
        "short_capture_probes": live_results,
        "storage_estimate": storage_estimate,
        "campaign_code_sha256": {
            name: _sha256(CAMPAIGN_ROOT / name)
            for name in ("campaign_run.py", "trajectory_contract.py", "campaign_setpoints.py",
                         "capture_streams.py", "state_recorder.py", "qc_attempt.py",
                         "preflight_capture.py", "calibration_solver.py", "analyze_campaign.py",
                         "workbook_campaign.py")
        },
        "software": {
            "python": sys.version.split()[0], "python_executable_role": "preflight process",
            "gazebo": tools["gz"], "ros2": tools["ros2"], "colcon": tools["colcon"],
            "yolo_device_requested": yolo_device,
            "px4_expected": "v1.17.0",
        "px4_source_from_repo_root": (
            str(px4_root.relative_to(REPO_ROOT)) if px4_root.is_relative_to(REPO_ROOT)
            else "external_machine_local_px4_source"
        ),
            "ros_workspace_from_repo_root": (
                str(ros_workspace.relative_to(REPO_ROOT)) if ros_workspace.is_relative_to(REPO_ROOT)
                else "external_runtime_workspace"
            ),
        },
        "checks": checks,
        "blocking_checks": required_failures,
        "limitations": [
            ("Short G0/G1 probes start headless SITL but do not arm or fly."
             if workbook_plan is None else
             "The first workbook row starts headless SITL for a short no-flight capture probe."),
            "Target mesh visibility and trajectory tracking are measured on the retained DEVELOPMENT pilot flight.",
            "PX4 sensor RNG independence is not established by this preflight.",
        ],
    }
    report_path = output_root / "environment_report.json"
    if report_path.exists():
        history = output_root / "environment_report_history.jsonl"
        with history.open("a", encoding="utf-8") as handle:
            previous = json.loads(report_path.read_text(encoding="utf-8"))
            handle.write(json.dumps(previous, ensure_ascii=False, sort_keys=True) + "\n")
    temporary = output_root / ".environment_report.json.tmp"
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, report_path)
    return report


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--action", choices=("preflight", "development", "run", "all", "resume", "all_results"), default="preflight")
    parser.add_argument("--workbook", type=Path,
                        help="numeric multi-flight XLSX used for workbook campaign actions")
    args = parser.parse_args()
    try:
        report = collect(args.output_root, action=args.action, workbook_path=args.workbook)
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(json.dumps({"status": "PREFLIGHT_BLOCKED", "error": str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps({"status": report["status"],
                      "blocking_checks": report["blocking_checks"],
                      "workbook_sha256": report["workbook_sha256"],
                      "environment_report": str((args.output_root / "environment_report.json"))},
                     ensure_ascii=False))
    return 0 if report["status"] == "PREFLIGHT_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
