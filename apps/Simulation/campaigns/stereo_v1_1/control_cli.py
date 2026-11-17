#!/usr/bin/env python3
"""Execute the selected v1.1 action from human-edited Simulation.env."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM_ROOT))
from campaigns.stereo_v1_1.control_config import REPO_ROOT, SIM_ROOT, load_control_config


CAMPAIGN_ROOT = Path(__file__).resolve().parent
REPORT_V3_BUILDER = (
    SIM_ROOT / "docs" / "changes" / "20261021-report-workbook-supplement"
    / "build_report_workbook_v3.sh"
)
CAMPAIGN_V3_BUILDER = CAMPAIGN_ROOT / "campaign_report_v3.py"
V12_ANALYSIS_AUDITOR = CAMPAIGN_ROOT / "audit_v12_analysis_results.py"


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


def _run_report_v3(config) -> int:
    if config.output_base != "repo" or config.output_id != "paper1_v1_2_001":
        raise ValueError("report_v3 only rebuilds the frozen repo-local paper1_v1_2_001 report")
    return _run(["bash", str(REPORT_V3_BUILDER)], env=os.environ.copy(), label="report-v3")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _current_analysis_id(config) -> str | None:
    """Return the latest analysis only when it still matches this 39-run source."""
    root = config.output_root
    pointer_path = root / "analysis_latest.json"
    status_path = root / "run_status.json"
    if not pointer_path.is_file() or not status_path.is_file():
        return None
    try:
        pointer = json.loads(pointer_path.read_text(encoding="utf-8"))
        status = json.loads(status_path.read_text(encoding="utf-8"))
        summary_path = (root / pointer["analysis_summary"]).resolve()
        if (not summary_path.is_relative_to(root.resolve())
                or summary_path.name != "analysis_summary.json"
                or summary_path.parent.name != pointer.get("analysis_id")):
            return None
        analysis_root = summary_path.parent
        manifest_path = analysis_root / "analysis_run_manifest.json"
        freeze_path = analysis_root / "analysis_freeze.json"
        if not manifest_path.is_file() or not freeze_path.is_file():
            return None
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        freeze = json.loads(freeze_path.read_text(encoding="utf-8"))
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        run_slots = status.get("run_slots", [])
        if (manifest.get("analysis_id") != pointer.get("analysis_id")
                or manifest.get("mc_trial_count") != config.mc_trials
                or manifest.get("source_valid_run_count") != 39
                or summary.get("analysis_id") != pointer.get("analysis_id")
                or summary.get("source_valid_run_count") != 39
                or freeze.get("mc_trial_count") != config.mc_trials
                or pointer.get("analysis_freeze_sha256") != _sha256(freeze_path)
                or summary.get("analysis_freeze_sha256") != pointer.get("analysis_freeze_sha256")
                or status.get("planned_run_count") != 39
                or status.get("valid_run_count") != 39
                or not isinstance(run_slots, list) or len(run_slots) != 39
                or any(not isinstance(slot, dict)
                       or slot.get("status") not in ("VALID", "VALID_WITH_WARNINGS")
                       or not isinstance(slot.get("attempts"), list) or not slot["attempts"]
                       or not isinstance(slot["attempts"][-1], dict)
                       or slot["attempts"][-1].get("status") not in ("VALID", "VALID_WITH_WARNINGS")
                       for slot in run_slots)):
            return None
        for relative, expected in manifest.get("source_campaign_manifest_sha256", {}).items():
            path = (root / relative).resolve()
            if not path.is_relative_to(root.resolve()) or not path.is_file() or _sha256(path) != expected:
                return None
        attempts = {}
        for slot in status.get("run_slots", []):
            if not isinstance(slot, dict) or not slot.get("attempts"):
                return None
            attempt = slot["attempts"][-1]
            relative = Path(slot["recording_path"]) / "checksums.sha256"
            checksums = (root / relative).resolve()
            if not checksums.is_relative_to(root.resolve()) or not checksums.is_file():
                return None
            attempts[f"{slot['run_id']}/{attempt['attempt_id']}"] = _sha256(checksums)
        if attempts != manifest.get("source_attempt_checksums_sha256"):
            return None
        for name, expected in manifest.get("algorithm_sha256", {}).items():
            source = (CAMPAIGN_ROOT / name).resolve()
            if not source.is_relative_to(CAMPAIGN_ROOT.resolve()) or not source.is_file() or _sha256(source) != expected:
                return None
        return pointer["analysis_id"]
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        return None


def _ensure_analysis_audit(config, analysis_id: str) -> tuple[int, Path]:
    audit_dir = config.output_root / "source_evidence" / f"post_analysis_verification_{analysis_id}"
    audit_path = audit_dir / "analysis_results_audit.json"
    if audit_path.is_file():
        result = json.loads(audit_path.read_text(encoding="utf-8"))
        if result.get("status") != "PASS" or result.get("analysis_id") != analysis_id:
            raise RuntimeError(f"existing analysis audit is not a matching PASS: {audit_path}")
        return 0, audit_path
    audit_dir.mkdir(parents=True, exist_ok=True)
    code = _run([
        sys.executable, str(V12_ANALYSIS_AUDITOR),
        "--output-root", str(config.output_root),
        "--source-root", str(config.output_root),
        "--report", str(audit_path),
    ], env=os.environ.copy(), label="analysis-audit")
    if code:
        return code, audit_path
    result = json.loads(audit_path.read_text(encoding="utf-8"))
    if result.get("status") != "PASS" or result.get("analysis_id") != analysis_id:
        return 2, audit_path
    return 0, audit_path


def _run_campaign_analysis_and_report(config) -> int:
    if config.mc_trials != 100:
        raise ValueError("the v1.2 SWEEP report requires 100 Monte Carlo trials")
    analysis_id = _current_analysis_id(config)
    if analysis_id is None:
        code = _run_analysis(config, input_root=config.output_root)
        if code:
            return code
        analysis_id = _current_analysis_id(config)
    if analysis_id is None:
        print("Analysis did not publish a current, source-matched analysis_latest.json.", file=sys.stderr, flush=True)
        return 2
    audit_code, audit_path = _ensure_analysis_audit(config, analysis_id)
    if audit_code:
        return audit_code
    command = [
        str(_analysis_python(config)), str(CAMPAIGN_V3_BUILDER),
        "--output-root", str(config.output_root),
        "--audit-report", str(audit_path),
    ]
    return _run(command, env=_campaign_runtime_env(config), label="report-v3-fresh")


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
    code = _run([sys.executable, str(CAMPAIGN_ROOT / "workbook_campaign.py"),
                 runner_action, "--workbook", str(config.workbook_path),
                 "--output-root", str(config.output_root)],
                env=_campaign_runtime_env(config), label=f"workbook:{runner_action}")
    if code:
        return code
    status_path = config.output_root / "run_status.json"
    if not status_path.is_file():
        raise RuntimeError("workbook campaign completed without run_status.json")
    status = json.loads(status_path.read_text(encoding="utf-8"))
    if status.get("valid_run_count") != 39 or status.get("planned_run_count") != 39:
        print("Campaign is incomplete; analysis and V3 report are deferred until all 39 runs pass source QC.", flush=True)
        return 0
    return _run_campaign_analysis_and_report(config)


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
    if config.action == "report_v3":
        return _run_report_v3(config)
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
