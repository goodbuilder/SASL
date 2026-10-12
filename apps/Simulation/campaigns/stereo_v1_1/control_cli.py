#!/usr/bin/env python3
"""Execute the selected v1.1 action from human-edited Simulation.env."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM_ROOT))
from campaigns.stereo_v1_1.control_config import REPO_ROOT, SIM_ROOT, load_control_config


CAMPAIGN_ROOT = Path(__file__).resolve().parent


def _effective_px4_dir(config) -> str:
    caller_value = os.environ.get("PX4_DIR", "").strip()
    if caller_value:
        return str(Path(caller_value).expanduser().resolve())
    if config.px4_dir_path.is_dir():
        return str(config.px4_dir_path)

    # Older SASL checkouts sometimes keep the exact pinned SITL tree beside the
    # active clone. Discover it only for this process; output manifests mark
    # it as machine-local provenance and never persist its absolute path.
    candidates = [
        REPO_ROOT / ".local" / "legacy" / "Simulation" / "PX4-Autopilot",
        SIM_ROOT.parent / "PX4-Autopilot",
    ]
    candidates.extend(sorted(
        REPO_ROOT.parent.glob("SASL-*/apps/Simulation/PX4-Autopilot"),
        key=lambda path: path.as_posix(), reverse=True,
    ))
    expected_binary = Path("build/px4_sitl_default/bin/px4")
    for candidate in candidates:
        if not (candidate / "Makefile").is_file() or not (candidate / "Tools").is_dir():
            continue
        if not (candidate / expected_binary).is_file():
            continue
        try:
            version = subprocess.run(
                ["git", "-C", str(candidate), "describe", "--tags", "--exact-match", "HEAD"],
                capture_output=True, text=True, timeout=5, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            continue
        if version.returncode == 0 and version.stdout.strip() == "v1.17.0":
            return str(candidate.resolve())
    return str(config.px4_dir_path)


def _analysis_python(config) -> Path:
    caller_value = os.environ.get("STEREO_V1_1_YOLO_PYTHON", "").strip()
    candidate = Path(caller_value).expanduser() if caller_value else None
    candidate_path = (candidate if candidate is None or candidate.is_absolute()
                      else SIM_ROOT / candidate)
    if candidate_path is not None and candidate_path.is_file():
        # Do not dereference a repo-local virtualenv's python symlink. The
        # interpreter uses the venv site-packages through the lexical path.
        return candidate_path if candidate_path.is_absolute() else Path(os.path.abspath(candidate_path))
    return config.yolo_python_path


def _run(command: list[str], *, env: dict[str, str], label: str) -> int:
    print(f"[{label}] {' '.join(command)}", flush=True)
    completed = subprocess.run(command, cwd=SIM_ROOT, env=env, check=False)
    print(f"[{label}] exit={completed.returncode}", flush=True)
    return completed.returncode


def _ensure_status(output_root: Path) -> None:
    output_root.mkdir(parents=True, exist_ok=True)
    status = output_root / "run_status.json"
    companions = tuple(output_root / name for name in (
        "manifest_actual.json", "qc_summary.json", "changes_log.jsonl",
        "checksums.sha256", "schema_mapping.json", "frozen_protocol",
    ))
    if status.is_file():
        missing = [path.name for path in companions if not path.is_file()]
        if missing:
            raise RuntimeError(f"campaign status is incomplete; preserve and inspect it: {missing}")
        return
    if any(path.exists() for path in companions):
        raise RuntimeError("partial campaign state exists without run_status.json; refusing to initialize")
    command = [sys.executable, str(CAMPAIGN_ROOT / "campaign_status.py"),
               "--output-root", str(output_root)]
    code = _run(command, env=os.environ.copy(), label="initialize")
    if code:
        raise RuntimeError(f"campaign status initialization failed with exit {code}")


def _run_campaign_command(config, command: str, *, development: bool = False) -> int:
    arguments = [sys.executable, str(CAMPAIGN_ROOT / "campaign_run.py"), command,
                 "--output-root", str(config.output_root)]
    if command == "run":
        arguments.extend(("--run-id", config.run_id))
    if development:
        arguments.append("--development")
    env = os.environ.copy()
    env["SIM_CONFIG"] = str(config.config_path)
    env["PX4_DIR"] = _effective_px4_dir(config)
    env["STEREO_V1_1_YOLO_PYTHON"] = str(_analysis_python(config))
    env["STEREO_V1_1_YOLO_DEVICE"] = config.yolo_device
    env["YOLO_TORCH_BACKEND"] = config.yolo_torch_backend
    return _run(arguments, env=env, label=f"campaign:{command}")


def _run_analysis(config, *, input_root: Path) -> int:
    if config.mc_trials is None:
        raise ValueError("STEREO_V1_1_MC_TRIALS is only valid for offline-analysis actions")
    arguments = [str(_analysis_python(config)), str(CAMPAIGN_ROOT / "analyze_campaign.py"),
                 "--output-root", str(config.output_root),
                 "--input-root", str(input_root),
                 "--mc-trials", str(config.mc_trials)]
    env = os.environ.copy()
    env["SIM_CONFIG"] = str(config.config_path)
    return _run(arguments, env=env, label="reprocess")


def _legacy_sweep_env(config) -> dict[str, str]:
    env = os.environ.copy()
    env["SIM_CONFIG"] = str(config.config_path)
    env.pop("SIM_SCENARIO_CONFIG", None)
    env.pop("SIM_GAZEBO_SEED", None)
    for name in tuple(env):
        if name.startswith(("CAMERA_", "CAM_L_", "CAM_R_", "COMMON_OPTICAL_",
                            "RELATIVE_OPTICAL_", "DRONE_", "WAYPOINT_", "YOLO_")):
            env.pop(name, None)
    return env


def _campaign_runtime_env(config) -> dict[str, str]:
    env = os.environ.copy()
    env["SIM_CONFIG"] = str(config.config_path)
    env["PX4_DIR"] = _effective_px4_dir(config)
    env["STEREO_V1_1_YOLO_PYTHON"] = str(_analysis_python(config))
    env["STEREO_V1_1_YOLO_DEVICE"] = config.yolo_device
    env["YOLO_TORCH_BACKEND"] = config.yolo_torch_backend
    if config.action in {"xlsx_plan", "xlsx_all", "xlsx_resume"} and config.workbook_world_name:
        env["PX4_GZ_WORLD"] = config.workbook_world_name
    return env


def _run_preflight(config, *, action: str | None = None,
                   workbook_path: Path | None = None) -> int:
    env = _campaign_runtime_env(config)
    if workbook_path is not None:
        from campaigns.stereo_v1_1.workbook_campaign import build_workbook_plan
        plan = build_workbook_plan(workbook_path)
        world_names = [row["declared_value"] for row in plan.get("environment_contract", [])
                       if row.get("setting_id") == "world_name"]
        if len(world_names) == 1:
            env["PX4_GZ_WORLD"] = world_names[0]
    return _run([sys.executable, str(CAMPAIGN_ROOT / "campaign_preflight.py"),
                 "--output-root", str(config.output_root), "--action", action or config.action,
                 *( ["--workbook", str(workbook_path)] if workbook_path else [])],
                env=env, label="preflight")


def _run_workbook_campaign(config, action: str) -> int:
    if action == "xlsx_plan":
        return _run([sys.executable, str(CAMPAIGN_ROOT / "workbook_campaign.py"),
                     "plan", "--workbook", str(config.workbook_path)],
                    env=os.environ.copy(), label="workbook-plan")
    if action == "xlsx_resume" and not (config.output_root / "run_status.json").is_file():
        raise ValueError("xlsx_resume requires an initialized workbook campaign with run_status.json")
    if action == "xlsx_all" and (config.output_root / "run_status.json").is_file():
        saved_status = json.loads((config.output_root / "run_status.json").read_text(encoding="utf-8"))
        if saved_status.get("workbook_sha256") != config.workbook_sha256:
            raise ValueError("output ID belongs to a different campaign or workbook; choose a new output ID")
    workbook_for_preflight = config.workbook_path
    if action == "xlsx_resume":
        frozen_workbook = config.output_root / "frozen_protocol" / "input_parameters.xlsx"
        if not frozen_workbook.is_file():
            raise ValueError("frozen workbook is missing from the existing campaign output")
        workbook_for_preflight = frozen_workbook
    preflight_action = "all" if action == "xlsx_all" else "resume"
    preflight_code = _run_preflight(
        config, action=preflight_action, workbook_path=workbook_for_preflight,
    )
    if preflight_code:
        print("Workbook flight action stopped because preflight is not clear.", flush=True)
        return preflight_code
    runner_action = "run" if action == "xlsx_all" else "resume"
    return _run([sys.executable, str(CAMPAIGN_ROOT / "workbook_campaign.py"),
                 runner_action, "--workbook", str(config.workbook_path),
                 "--output-root", str(config.output_root)],
                env=_campaign_runtime_env(config), label=f"workbook:{runner_action}")


def execute(config_path: Path | str) -> int:
    config = load_control_config(config_path)
    output_root = config.output_root
    print(json.dumps({
        "action": config.action,
        "run_id": config.run_id,
        "output_id": config.output_id,
        "output_base": config.output_base,
        "input_root_setting": config.input_root_setting,
        "workbook_path_from_sim_root": str(config.workbook_path.relative_to(SIM_ROOT)),
        "workbook_sha256": config.workbook_sha256,
        "workbook_world_name": config.workbook_world_name,
        "output_root_relative": (str(output_root.relative_to(SIM_ROOT))
                                  if output_root.is_relative_to(SIM_ROOT)
                                  else f"SASL_WORKSTATION_DATA_ROOT/{config.output_id}"),
        "mc_trials": config.mc_trials,
        "px4_dir_from_repo_root": str(config.px4_dir_path.relative_to(SIM_ROOT.parents[1])),
        "runtime_px4_override_supplied": bool(os.environ.get("PX4_DIR", "").strip()),
        "runtime_px4_resolution": (
            "caller_override" if os.environ.get("PX4_DIR", "").strip() else
            "configured_repo_relative" if config.px4_dir_path.is_dir() else
            "adjacent_pinned_checkout" if Path(_effective_px4_dir(config)) != config.px4_dir_path else
            "configured_path_missing"
        ),
        "yolo_python_from_sim_root": str(config.yolo_python_path.relative_to(SIM_ROOT)),
        "yolo_device": config.yolo_device,
        "yolo_torch_backend": config.yolo_torch_backend,
        "simulation_env_sha256": config.config_sha256,
    }, ensure_ascii=False), flush=True)

    if config.action == "legacy_sweep":
        return _run(["bash", str(SIM_ROOT / "run_auto_attitude_sweep.sh"), "--legacy"],
                    env=_legacy_sweep_env(config), label="legacy-sweep")
    if config.action in {"xlsx_plan", "xlsx_all", "xlsx_resume"}:
        return _run_workbook_campaign(config, config.action)
    if config.action == "plan":
        output_root.mkdir(parents=True, exist_ok=True)
        return _run_campaign_command(config, "plan")
    if config.action == "preflight":
        return _run_preflight(config)
    if config.action == "qc":
        return _run_campaign_command(config, "qc")
    if config.action == "reprocess":
        return _run_analysis(config, input_root=config.input_root)

    if config.action in {"development", "run", "all", "resume", "all_results"}:
        preflight_code = _run_preflight(config)
        if preflight_code:
            print("Flight action stopped because preflight is not clear.", flush=True)
            return preflight_code

    _ensure_status(output_root)
    if config.action == "development":
        return _run_campaign_command(config, "run", development=True)
    if config.action == "run":
        return _run_campaign_command(config, "run")
    if config.action == "all":
        return _run_campaign_command(config, "all")
    if config.action == "resume":
        code = _run_campaign_command(config, "resume")
        if code:
            return code
        status_path = output_root / "run_status.json"
        status = json.loads(status_path.read_text(encoding="utf-8"))
        if status.get("valid_run_count") == 39:
            return _run_analysis(config, input_root=output_root)
        return 0
    if config.action == "all_results":
        code = _run_campaign_command(config, "all")
        if code:
            print("Raw campaign did not finish cleanly; reprocessing was not started.", flush=True)
            return code
        return _run_analysis(config, input_root=output_root)
    raise AssertionError(f"unhandled action: {config.action}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=None,
                        help="Simulation.env; defaults to config/simulation.env")
    args = parser.parse_args()
    config_path = args.config or Path(os.environ.get("SIM_CONFIG", "")) or None
    if config_path is None or str(config_path) == ".":
        from campaigns.stereo_v1_1.control_config import DEFAULT_CONFIG
        config_path = DEFAULT_CONFIG
    try:
        return execute(config_path)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
