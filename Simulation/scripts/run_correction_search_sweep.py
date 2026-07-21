#!/usr/bin/env python3
"""Run deterministic static Ideal correction-search qualification sweeps.

This is a non-formal, static Gazebo qualification runner.  It does not move a
vehicle, fit a correction profile, or implement train/validation/Test splits.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
from typing import Any, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPOSITORY_ROOT / "Simulation" / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_sweep import (  # noqa: E402
    ATTITUDE_AXES,
    CameraContract,
    S2_SWEEP_SCHEMA_VERSION,
    atomic_write_json,
    build_launch_command,
    build_sweep_plan,
    canonical_json,
    compute_case_statistics,
    compute_spatial_validation,
    next_attempt_directory,
    parse_attitude_axis_range,
    parse_numeric_values,
    sha256_bytes,
    sha256_file,
    validate_r2_artifacts,
    write_case_csv,
    write_plan,
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def command_output(command: Sequence[str], *, cwd: Path) -> str:
    return subprocess.check_output(command, cwd=cwd, text=True).strip()


def verify_git_provenance(
    repository_root: Path,
    *,
    expected_commit_sha: str,
    expected_branch: str,
    repository_name: str,
) -> dict[str, str]:
    root = Path(repository_root).resolve(strict=True)
    if command_output(("git", "rev-parse", "--show-toplevel"), cwd=root) != str(root):
        raise ValueError("script repository root does not match Git toplevel")
    if len(expected_commit_sha) != 40 or any(
        character not in "0123456789abcdef" for character in expected_commit_sha
    ):
        raise ValueError("expected commit SHA must be 40 lowercase hexadecimal characters")
    if not expected_branch.strip() or not repository_name.strip():
        raise ValueError("expected branch and repository name are required")
    actual_sha = command_output(("git", "rev-parse", "HEAD"), cwd=root)
    actual_branch = command_output(("git", "branch", "--show-current"), cwd=root)
    status = command_output(("git", "status", "--short"), cwd=root)
    origin = command_output(("git", "remote", "get-url", "origin"), cwd=root)
    if actual_sha != expected_commit_sha:
        raise ValueError(f"HEAD mismatch: expected {expected_commit_sha}, got {actual_sha}")
    if actual_branch != expected_branch:
        raise ValueError(f"branch mismatch: expected {expected_branch}, got {actual_branch}")
    if status:
        raise ValueError("working tree must be clean")
    return {
        "repository_root": str(root),
        "repository_name": repository_name,
        "branch": actual_branch,
        "commit_sha": actual_sha,
        "git_status": "clean",
        "origin_url": origin,
    }


def validate_output_directory(
    output_dir: Path, *, repository_root: Path, resume: bool
) -> Path:
    candidate = Path(output_dir)
    if not candidate.is_absolute():
        raise ValueError("output directory must be absolute")
    output = candidate.resolve(strict=False)
    repository = Path(repository_root).resolve(strict=True)
    if output == repository or repository in output.parents:
        raise ValueError("output directory must be outside the repository")
    if resume:
        if not output.is_dir() or not (output / "sweep_plan.json").is_file():
            raise ValueError("resume requires an existing sweep directory")
    elif output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError("new sweep output directory must be new or empty")
    output.mkdir(parents=True, exist_ok=True)
    return output


def resolve_gazebo_version() -> str:
    for command in (("gz", "sim", "--versions"), ("gz", "sim", "--version")):
        try:
            value = command_output(command, cwd=REPOSITORY_ROOT).splitlines()[0].strip()
        except (OSError, subprocess.CalledProcessError, IndexError):
            continue
        if value:
            return value
    raise RuntimeError("Gazebo version could not be resolved")


def conflicting_simulation_processes() -> list[dict[str, Any]]:
    result = subprocess.run(
        ("pgrep", "-af", "gz sim|ros2 launch stereo_ground_sim"),
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    conflicts = []
    for line in result.stdout.splitlines():
        parts = line.split(maxsplit=1)
        if len(parts) != 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        command = parts[1]
        if pid == os.getpid() or "pgrep -af" in command or Path(__file__).name in command:
            continue
        conflicts.append({"pid": pid, "command": command})
    return conflicts


class OwnedProcess:
    def __init__(self, process: subprocess.Popen[str], log_handle) -> None:
        self.process = process
        self.log_handle = log_handle
        self.events: list[dict[str, Any]] = []

    @property
    def pid(self) -> int:
        return self.process.pid

    def poll(self):
        return self.process.poll()

    def wait(self, timeout: float):
        return self.process.wait(timeout=timeout)

    def signal_owned(self, signal_number: signal.Signals) -> None:
        # Let ros2 launch deliver the first graceful interrupt exactly once.
        # Broadcasting SIGINT to the whole group makes Python children receive
        # it directly and then receive launch's forwarded SIGINT a second time,
        # which can interrupt recorder finalization.  Escalations still target
        # the complete runner-owned process group.
        if signal_number == signal.SIGINT:
            os.kill(self.process.pid, signal_number)
        else:
            os.killpg(self.process.pid, signal_number)
        self.events.append({"signal": signal_number.name, "utc": utc_now()})

    def process_group_exists(self) -> bool:
        try:
            os.killpg(self.process.pid, 0)
        except ProcessLookupError:
            return False
        return True

    def close_log(self) -> None:
        self.log_handle.close()


class ProcessLauncher:
    def launch(
        self,
        command: Sequence[str],
        *,
        cwd: Path,
        environment: Mapping[str, str],
        log_path: Path,
    ) -> OwnedProcess:
        log_handle = Path(log_path).open("w", encoding="utf-8")
        process = subprocess.Popen(
            list(command),
            cwd=Path(cwd),
            env=dict(environment),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        return OwnedProcess(process, log_handle)


def stop_owned_process(process: Any, *, shutdown_timeout_s: float) -> list[dict[str, Any]]:
    if process.poll() is None:
        for signal_number in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                process.signal_owned(signal_number)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=shutdown_timeout_s)
                break
            except subprocess.TimeoutExpired:
                continue

    # Some Gazebo launch wrappers exit while leaving their server child in the
    # same runner-owned process group.  Recorder finalization is complete once
    # launch has exited, so clean only that residual group before the next case.
    group_exists = getattr(process, "process_group_exists", lambda: False)
    if group_exists():
        try:
            process.signal_owned(signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + shutdown_timeout_s
        while group_exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        if group_exists():
            try:
                process.signal_owned(signal.SIGKILL)
            except ProcessLookupError:
                pass
    return list(getattr(process, "events", ()))


def should_launch_next(*, case_valid: bool, failure_policy: str) -> bool:
    if failure_policy not in {"stop", "continue"}:
        raise ValueError("failure policy must be stop or continue")
    return bool(case_valid) or failure_policy == "continue"


def count_strict_eligible(jsonl_path: Path) -> int:
    if not jsonl_path.is_file():
        return 0
    count = 0
    with jsonl_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                value = json.loads(
                    line,
                    parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
                )
            except (json.JSONDecodeError, ValueError):
                continue
            if type(value.get("sample_eligible")) is bool and value["sample_eligible"]:
                count += 1
    return count


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path.name} must contain an object")
    return value


def validate_resume_plan(
    stored: Mapping[str, Any], regenerated: Mapping[str, Any]
) -> None:
    for field in ("schema_version", "plan_hash", "commit_sha", "branch"):
        if stored.get(field) != regenerated.get(field):
            raise ValueError(f"resume plan mismatch: {field}")


def _numeric_attempts(case_dir: Path) -> list[tuple[int, Path]]:
    attempts = []
    for child in Path(case_dir).glob("attempt-*"):
        try:
            number = int(child.name.removeprefix("attempt-"))
        except ValueError:
            continue
        if child.is_dir():
            attempts.append((number, child))
    return sorted(attempts)


def inspect_sweep_marker_state(output_dir: Path) -> str:
    output = Path(output_dir)
    present = {
        name
        for name in (
            "SWEEP_PLAN_READY",
            "SWEEP_INCOMPLETE",
            "SWEEP_FINALIZED_VALID",
            "SWEEP_FINALIZED_INVALID",
        )
        if (output / name).exists()
    }
    if len(present) > 1:
        raise ValueError("contradictory sweep markers: " + ", ".join(sorted(present)))
    if not present:
        return "EMPTY"
    return {
        "SWEEP_PLAN_READY": "PLAN_READY",
        "SWEEP_INCOMPLETE": "RUNNING",
        "SWEEP_FINALIZED_VALID": "FINAL_VALID",
        "SWEEP_FINALIZED_INVALID": "FINAL_INVALID",
    }[next(iter(present))]


def enter_running_state(output_dir: Path) -> None:
    output = Path(output_dir)
    state = inspect_sweep_marker_state(output)
    incomplete = output / "SWEEP_INCOMPLETE"
    if state == "RUNNING":
        return
    if state == "EMPTY":
        with incomplete.open("x", encoding="utf-8"):
            pass
        return
    source = {
        "PLAN_READY": output / "SWEEP_PLAN_READY",
        "FINAL_VALID": output / "SWEEP_FINALIZED_VALID",
        "FINAL_INVALID": output / "SWEEP_FINALIZED_INVALID",
    }[state]
    os.replace(source, incomplete)


def validate_sweep_summary_marker(output_dir: Path, marker_state: str) -> dict[str, Any] | None:
    if marker_state not in {"FINAL_VALID", "FINAL_INVALID"}:
        return None
    summary_path = Path(output_dir) / "sweep_summary.json"
    if not summary_path.is_file():
        raise ValueError("final sweep marker exists without sweep_summary.json")
    summary = load_json(summary_path)
    expected_valid = marker_state == "FINAL_VALID"
    if summary.get("run_valid") is not expected_valid:
        raise ValueError("sweep marker contradicts summary run_valid")
    if summary.get("finalization_status") != ("VALID" if expected_valid else "INVALID"):
        raise ValueError("sweep marker contradicts summary finalization_status")
    return summary


def _attempt_spatial_validation(
    case: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    *,
    warmup_count: int,
    analysis_count: int,
) -> dict[str, Any]:
    payload = case["payload"]
    spatial = payload["spatial"]
    if payload.get("trajectory_mode") != "static_gazebo_reference_point":
        raise ValueError("case trajectory mode is not static")
    if payload.get("target_physics_mode") != "static_model":
        raise ValueError("case target physics mode is not static_model")
    if payload.get("target_static_for_research") is not True:
        raise ValueError("case target_static_for_research must be true")
    result = compute_spatial_validation(
        records,
        warmup_count=warmup_count,
        analysis_count=analysis_count,
        requested_range_m=spatial["range_m"]["value"],
        requested_bearing_deg=spatial["bearing_deg"]["value"],
        requested_elevation_deg=spatial["elevation_deg"]["value"],
        range_tolerance_m=payload["spatial_range_tolerance_m"],
        angle_tolerance_deg=payload["spatial_angle_tolerance_deg"],
    )
    if not result["spatial_validation_pass"]:
        raise ValueError(
            "spatial_cell_mismatch:"
            f"range_max_abs_error_m={result['spatial_range_error_abs_max_m']},"
            f"bearing_max_abs_error_deg={result['spatial_bearing_error_abs_max_deg']},"
            f"elevation_max_abs_error_deg={result['spatial_elevation_error_abs_max_deg']},"
            f"range_tolerance_m={result['spatial_range_tolerance_m']},"
            f"angle_tolerance_deg={result['spatial_angle_tolerance_deg']}"
        )
    return result


def validated_latest_attempt(
    *,
    case: Mapping[str, Any],
    case_dir: Path,
    provenance: Mapping[str, str],
    sweep_id: str,
    warmup_count: int,
    analysis_count: int,
    plan_hash: str | None = None,
) -> tuple[Path, dict[str, Any]] | None:
    inspection = inspect_latest_attempt(
        case=case,
        case_dir=case_dir,
        provenance=provenance,
        sweep_id=sweep_id,
        warmup_count=warmup_count,
        analysis_count=analysis_count,
        plan_hash=plan_hash,
    )
    if inspection["classification"] != "VALID":
        return None
    return inspection["path"], inspection["state"]


def inspect_latest_attempt(
    *,
    case: Mapping[str, Any],
    case_dir: Path,
    provenance: Mapping[str, str],
    sweep_id: str,
    warmup_count: int,
    analysis_count: int,
    plan_hash: str | None = None,
) -> dict[str, Any]:
    attempts = _numeric_attempts(case_dir)
    if not attempts:
        return {"classification": "MISSING", "path": None, "state": None, "reason": "no_attempt"}
    _number, attempt = attempts[-1]
    state_path = attempt / "attempt_state.json"
    if not state_path.is_file():
        return {
            "classification": "INCOMPLETE",
            "path": attempt,
            "state": None,
            "reason": "attempt_state_missing",
        }
    try:
        state = load_json(state_path)
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        return {
            "classification": "UNKNOWN",
            "path": attempt,
            "state": None,
            "reason": f"attempt_state_unreadable:{type(exc).__name__}:{exc}",
        }
    recording = attempt / "recording"
    marker_names = {
        name
        for name in ("RUN_INCOMPLETE", "RUN_FINALIZED_VALID", "RUN_FINALIZED_INVALID")
        if (recording / name).exists()
    }
    if len(marker_names) > 1:
        return {
            "classification": "UNKNOWN",
            "path": attempt,
            "state": state,
            "reason": "contradictory_recording_markers:" + ",".join(sorted(marker_names)),
        }
    attempt_status = state.get("attempt_status")
    if attempt_status == "RUNNING":
        return {
            "classification": "INCOMPLETE",
            "path": attempt,
            "state": state,
            "reason": "attempt_state_running",
        }
    if attempt_status not in {None, "FINALIZED"}:
        return {
            "classification": "UNKNOWN",
            "path": attempt,
            "state": state,
            "reason": f"unknown_attempt_status:{attempt_status}",
        }
    if "RUN_INCOMPLETE" in marker_names or not marker_names:
        return {
            "classification": "INCOMPLETE",
            "path": attempt,
            "state": state,
            "reason": "recording_not_finalized",
        }
    if "RUN_FINALIZED_INVALID" in marker_names or state.get("case_valid") is False:
        return {
            "classification": "INVALID",
            "path": attempt,
            "state": state,
            "reason": state.get("failure_reason") or "attempt_marked_invalid",
        }
    try:
        if state.get("case_valid") is not True:
            raise ValueError("attempt state is not finalized valid")
        if state.get("case_id") != case["case_id"]:
            raise ValueError("stored attempt case ID mismatch")
        if state.get("case_hash") != case["case_hash"]:
            raise ValueError("stored attempt case hash mismatch")
        if plan_hash is not None and state.get("plan_hash") != plan_hash:
            raise ValueError("stored attempt plan hash mismatch")
        records, _summary = validate_r2_artifacts(
            recording,
            expected_sha=provenance["commit_sha"],
            expected_branch=provenance["branch"],
            expected_run_id=sweep_id,
            expected_case_id=case["case_id"],
            minimum_eligible=warmup_count + analysis_count,
        )
        statistics = compute_case_statistics(
            records,
            warmup_count=warmup_count,
            analysis_count=analysis_count,
        )
        statistics.update(
            _attempt_spatial_validation(
                case,
                records,
                warmup_count=warmup_count,
                analysis_count=analysis_count,
            )
        )
        if state.get("statistics") != statistics:
            raise ValueError("stored case statistics do not match R2 records")
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        return {
            "classification": "INVALID",
            "path": attempt,
            "state": state,
            "reason": f"semantic_or_spatial_validation_failed:{type(exc).__name__}:{exc}",
        }
    return {"classification": "VALID", "path": attempt, "state": state, "reason": ""}


def _case_row(case: Mapping[str, Any], attempt: int, state: Mapping[str, Any]) -> dict[str, Any]:
    payload = case["payload"]
    row = {
        "case_id": case["case_id"],
        "attempt": attempt,
        "case_valid": state.get("case_valid", False),
        "failure_reason": state.get("failure_reason", ""),
        "range_m": payload["spatial"]["range_m"]["value"],
        "bearing_deg": payload["spatial"]["bearing_deg"]["value"],
        "elevation_deg": payload["spatial"]["elevation_deg"]["value"],
    }
    row.update(
        {axis: payload["attitude_source_deg"][axis]["value"] for axis in ATTITUDE_AXES}
    )
    for side in ("left", "right"):
        for index, axis in enumerate(("roll", "pitch", "yaw")):
            row[f"effective_{side}_optical_{axis}_error_deg"] = payload[
                f"effective_{side}_rpy_deg"
            ][index]
    row.update(state.get("statistics", {}))
    return row


def execute_case_attempt(
    *,
    case: Mapping[str, Any],
    output_dir: Path,
    repository_root: Path,
    provenance: Mapping[str, str],
    sweep_id: str,
    gazebo_version: str,
    warmup_count: int,
    analysis_count: int,
    case_timeout_s: float,
    shutdown_timeout_s: float,
    launcher: Any,
    plan_hash: str | None = None,
    predecessor_inspection: Mapping[str, Any] | None = None,
    monotonic=time.monotonic,
    sleeper=time.sleep,
) -> dict[str, Any]:
    attempt_number, attempt_dir = next_attempt_directory(
        Path(output_dir) / "cases" / case["case_id"]
    )
    attempt_dir.mkdir(parents=True, exist_ok=False)
    recording_dir = attempt_dir / "recording"
    log_path = attempt_dir / "simulation.log"
    command = build_launch_command(
        repository_root=repository_root,
        case=case,
        attempt_dir=attempt_dir,
        sweep_id=sweep_id,
        repository_name=provenance["repository_name"],
        branch=provenance["branch"],
        commit_sha=provenance["commit_sha"],
        gazebo_version=gazebo_version,
    )
    environment = dict(os.environ)
    environment["ROS_LOG_DIR"] = str(attempt_dir / "ros-logs")
    state: dict[str, Any] = {
        "schema_version": S2_SWEEP_SCHEMA_VERSION,
        "case_id": case["case_id"],
        "case_hash": case["case_hash"],
        "plan_hash": plan_hash,
        "attempt": attempt_number,
        "attempt_started_utc": utc_now(),
        "attempt_status": "RUNNING",
        "command": command,
        "command_sha256": sha256_bytes(canonical_json(command).encode("utf-8")),
        "cwd": str(Path(repository_root).resolve()),
        "simulation_log": str(log_path),
        "recording_dir": str(recording_dir),
        "case_valid": False,
        "failure_reason": "",
        "signal_escalation": [],
        "resume_predecessor": (
            {
                "classification": predecessor_inspection.get("classification"),
                "path": str(predecessor_inspection.get("path")),
                "reason": predecessor_inspection.get("reason"),
            }
            if predecessor_inspection and predecessor_inspection.get("path") is not None
            else None
        ),
    }
    atomic_write_json(attempt_dir / "attempt_state.json", state)
    process = None
    try:
        process = launcher.launch(
            command,
            cwd=repository_root,
            environment=environment,
            log_path=log_path,
        )
        state["owned_process_pid"] = process.pid
        deadline = monotonic() + case_timeout_s
        target = warmup_count + analysis_count
        while monotonic() < deadline:
            if count_strict_eligible(recording_dir / "pair_records.jsonl") >= target:
                break
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(f"child exited early with return code {return_code}")
            sleeper(0.1)
        else:
            raise TimeoutError("case eligible sample timeout")
        state["signal_escalation"] = stop_owned_process(
            process, shutdown_timeout_s=shutdown_timeout_s
        )
        return_code = process.poll()
        if return_code is None:
            return_code = process.wait(timeout=shutdown_timeout_s)
        state["return_code"] = return_code
        records, summary = validate_r2_artifacts(
            recording_dir,
            expected_sha=provenance["commit_sha"],
            expected_branch=provenance["branch"],
            expected_run_id=sweep_id,
            expected_case_id=case["case_id"],
            minimum_eligible=target,
        )
        statistics = compute_case_statistics(
            records, warmup_count=warmup_count, analysis_count=analysis_count
        )
        statistics.update(
            _attempt_spatial_validation(
                case,
                records,
                warmup_count=warmup_count,
                analysis_count=analysis_count,
            )
        )
        state.update(
            {
                "case_valid": True,
                "statistics": statistics,
                "r2_summary_sha256": sha256_file(recording_dir / "run_summary.json"),
                "r2_terminal_record_count": summary["terminal_record_count"],
                "attempt_finished_utc": utc_now(),
                "attempt_status": "FINALIZED",
            }
        )
    except Exception as exc:
        if process is not None:
            state["signal_escalation"] = stop_owned_process(
                process, shutdown_timeout_s=shutdown_timeout_s
            )
        state.update(
            {
                "case_valid": False,
                "failure_reason": f"{type(exc).__name__}: {exc}",
                "failure_traceback": traceback.format_exc(),
                "attempt_finished_utc": utc_now(),
                "attempt_status": "FINALIZED",
            }
        )
    finally:
        if process is not None:
            process.close_log()
        atomic_write_json(attempt_dir / "attempt_state.json", state)
    return state


def finalize_sweep(
    *,
    output_dir: Path,
    plan: Mapping[str, Any],
    cases: Sequence[Mapping[str, Any]],
    rows: Sequence[Mapping[str, Any]],
    counters: Mapping[str, int],
    case_csv_writer=write_case_csv,
    json_writer=atomic_write_json,
    hash_file=sha256_file,
    marker_replace=os.replace,
) -> dict[str, Any]:
    output = Path(output_dir)
    if inspect_sweep_marker_state(output) != "RUNNING":
        raise ValueError("sweep must be RUNNING before finalization")
    case_csv_writer(output / "sweep_cases.csv", rows)
    valid_ids = {row["case_id"] for row in rows if row.get("case_valid") is True}
    planned_ids = {case["case_id"] for case in cases}
    active_incomplete = []
    for row in rows:
        marker = (
            output
            / "cases"
            / str(row["case_id"])
            / f"attempt-{int(row['attempt']):04d}"
            / "recording"
            / "RUN_INCOMPLETE"
        )
        if marker.is_file():
            active_incomplete.append(str(marker))
    invalid_reasons = []
    if valid_ids != planned_ids:
        invalid_reasons.append("not_all_runnable_cases_have_valid_attempt")
    if active_incomplete:
        invalid_reasons.append("active_incomplete_attempt")
    if len(planned_ids) != len(cases):
        invalid_reasons.append("duplicate_case_id")
    row_ids = [row["case_id"] for row in rows]
    if len(rows) != len(cases):
        invalid_reasons.append("case_summary_row_count_mismatch")
    if len(set(row_ids)) != len(row_ids):
        invalid_reasons.append("duplicate_case_summary_row")
    run_valid = not invalid_reasons
    summary = {
        "schema_version": S2_SWEEP_SCHEMA_VERSION,
        "sweep_id": plan["sweep_id"],
        "plan_hash": plan["plan_hash"],
        "commit_sha": plan["commit_sha"],
        "branch": plan["branch"],
        "finalized_utc": utc_now(),
        "run_valid": run_valid,
        "finalization_status": "VALID" if run_valid else "INVALID",
        "invalid_reasons": invalid_reasons,
        "total_runnable": len(cases),
        "valid_case_count": len(valid_ids),
        "case_summary_row_count": len(rows),
        "active_incomplete_attempts": active_incomplete,
        **dict(counters),
        "target_physics_mode": plan["target_physics_mode"],
        "target_static_for_research": plan["target_static_for_research"],
        "spatial_range_tolerance_m": plan["spatial_range_tolerance_m"],
        "spatial_angle_tolerance_deg": plan["spatial_angle_tolerance_deg"],
        "sweep_plan_sha256": hash_file(output / "sweep_plan.json"),
        "sweep_cases_jsonl_sha256": hash_file(output / "sweep_cases.jsonl"),
        "sweep_cases_csv_sha256": hash_file(output / "sweep_cases.csv"),
    }
    json_writer(output / "sweep_summary.json", summary)
    marker = output / ("SWEEP_FINALIZED_VALID" if run_valid else "SWEEP_FINALIZED_INVALID")
    opposite = output / ("SWEEP_FINALIZED_INVALID" if run_valid else "SWEEP_FINALIZED_VALID")
    if opposite.exists():
        raise ValueError("opposite final marker exists during finalization")
    incomplete = output / "SWEEP_INCOMPLETE"
    marker_replace(incomplete, marker)
    return summary


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--sweep-id", required=True)
    parser.add_argument("--expected-commit-sha", required=True)
    parser.add_argument("--expected-branch", required=True)
    parser.add_argument("--repository-name", required=True)
    parser.add_argument("--range-values-m", required=True)
    parser.add_argument("--bearing-values-deg", required=True)
    parser.add_argument("--elevation-values-deg", required=True)
    parser.add_argument("--attitude-axis-range", action="append", default=[])
    parser.add_argument("--attitude-mode", choices=("ofat", "cartesian"), default="ofat")
    parser.add_argument("--invalid-spatial-policy", choices=("fail", "skip"), default="fail")
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--case-order", choices=("canonical", "seeded"), default="canonical")
    parser.add_argument("--max-cases", type=int, default=5000)
    parser.add_argument("--baseline-m", type=float, default=25.0)
    parser.add_argument("--camera-height-m", type=float, default=5.0)
    parser.add_argument("--horizontal-fov-rad", type=float, default=0.154997710698)
    parser.add_argument("--image-width-px", type=int, default=1280)
    parser.add_argument("--image-height-px", type=int, default=720)
    parser.add_argument("--camera-update-rate-hz", type=float, default=20.0)
    parser.add_argument("--camera-clip-near-m", type=float, default=0.1)
    parser.add_argument("--camera-clip-far-m", type=float, default=2500.0)
    parser.add_argument("--spatial-range-tolerance-m", type=float, default=0.0001)
    parser.add_argument("--spatial-angle-tolerance-deg", type=float, default=0.00001)
    parser.add_argument("--warmup-eligible-samples", type=int, default=20)
    parser.add_argument("--analysis-eligible-samples", type=int, default=60)
    parser.add_argument("--case-timeout-s", type=float, default=60.0)
    parser.add_argument("--shutdown-timeout-s", type=float, default=15.0)
    parser.add_argument("--case-failure-policy", choices=("stop", "continue"), default="stop")
    parser.add_argument("--gazebo-version", default="")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args(argv)


def _build_from_args(args: argparse.Namespace):
    contract = CameraContract(
        baseline_m=args.baseline_m,
        camera_height_m=args.camera_height_m,
        horizontal_fov_rad=args.horizontal_fov_rad,
        image_width_px=args.image_width_px,
        image_height_px=args.image_height_px,
        camera_update_rate_hz=args.camera_update_rate_hz,
        camera_clip_near_m=args.camera_clip_near_m,
        camera_clip_far_m=args.camera_clip_far_m,
    )
    return build_sweep_plan(
        sweep_id=args.sweep_id,
        commit_sha=args.expected_commit_sha,
        branch=args.expected_branch,
        repository_name=args.repository_name,
        attitude_mode=args.attitude_mode,
        axis_ranges=[parse_attitude_axis_range(value) for value in args.attitude_axis_range],
        range_values=parse_numeric_values(args.range_values_m, name="range values"),
        bearing_values=parse_numeric_values(args.bearing_values_deg, name="bearing values"),
        elevation_values=parse_numeric_values(args.elevation_values_deg, name="elevation values"),
        repetitions=args.repetitions,
        seed=args.seed,
        case_order=args.case_order,
        invalid_spatial_policy=args.invalid_spatial_policy,
        max_cases=args.max_cases,
        contract=contract,
        spatial_range_tolerance_m=args.spatial_range_tolerance_m,
        spatial_angle_tolerance_deg=args.spatial_angle_tolerance_deg,
    )


def main(argv: Sequence[str] | None = None, *, launcher: Any | None = None) -> int:
    args = parse_args(argv)
    provenance = verify_git_provenance(
        REPOSITORY_ROOT,
        expected_commit_sha=args.expected_commit_sha,
        expected_branch=args.expected_branch,
        repository_name=args.repository_name,
    )
    output = validate_output_directory(
        args.output_dir, repository_root=REPOSITORY_ROOT, resume=args.resume
    )
    plan, cases = _build_from_args(args)
    if args.warmup_eligible_samples < 0 or args.analysis_eligible_samples <= 0:
        raise ValueError("eligible sample window must be nonnegative/positive")
    if args.case_timeout_s <= 0 or args.shutdown_timeout_s <= 0:
        raise ValueError("timeouts must be positive")
    existing_plan = output / "sweep_plan.json"
    if args.resume:
        marker_state = inspect_sweep_marker_state(output)
        marker_summary = validate_sweep_summary_marker(output, marker_state)
        stored = load_json(existing_plan)
        validate_resume_plan(stored, plan)
        completed = []
        if marker_state == "FINAL_VALID":
            completed = [
                validated_latest_attempt(
                    case=case,
                    case_dir=output / "cases" / case["case_id"],
                    provenance=provenance,
                    sweep_id=args.sweep_id,
                    warmup_count=args.warmup_eligible_samples,
                    analysis_count=args.analysis_eligible_samples,
                    plan_hash=plan["plan_hash"],
                )
                for case in cases
            ]
            stored_summary = marker_summary or load_json(output / "sweep_summary.json")
            summary_valid = (
                stored_summary.get("run_valid") is True
                and stored_summary.get("plan_hash") == plan["plan_hash"]
                and stored_summary.get("case_summary_row_count") == len(cases)
                and stored_summary.get("target_static_for_research") is True
            )
        else:
            summary_valid = False
        if completed and all(item is not None for item in completed) and summary_valid:
            response = {
                "schema_version": S2_SWEEP_SCHEMA_VERSION,
                "sweep_id": args.sweep_id,
                "plan_hash": plan["plan_hash"],
                "skipped_valid_count": len(cases),
                "retried_invalid_count": 0,
                "newly_executed_count": 0,
                "launch_count": 0,
                "run_valid": True,
            }
            print(canonical_json(response, indent=2))
            return 0
    else:
        write_plan(output, plan, cases)
    if args.plan_only:
        if args.resume:
            write_plan(output, plan, cases)
        if inspect_sweep_marker_state(output) == "EMPTY":
            (output / "SWEEP_PLAN_READY").touch(exist_ok=False)
        elif inspect_sweep_marker_state(output) != "PLAN_READY":
            raise ValueError("plan-only output has incompatible marker state")
        print(canonical_json({"plan_hash": plan["plan_hash"], "total_runnable": len(cases)}, indent=2))
        return 0
    conflicts = conflicting_simulation_processes()
    if conflicts:
        raise RuntimeError("conflicting Simulation processes detected: " + canonical_json(conflicts))
    enter_running_state(output)
    state = {
        "schema_version": S2_SWEEP_SCHEMA_VERSION,
        "sweep_id": args.sweep_id,
        "plan_hash": plan["plan_hash"],
        "created_utc": utc_now(),
        "provenance": provenance,
        "case_failure_policy": args.case_failure_policy,
        "warmup_eligible_samples": args.warmup_eligible_samples,
        "analysis_eligible_samples": args.analysis_eligible_samples,
        "case_timeout_s": args.case_timeout_s,
        "shutdown_timeout_s": args.shutdown_timeout_s,
        "target_physics_mode": plan["target_physics_mode"],
        "target_static_for_research": plan["target_static_for_research"],
        "spatial_range_tolerance_m": plan["spatial_range_tolerance_m"],
        "spatial_angle_tolerance_deg": plan["spatial_angle_tolerance_deg"],
    }
    atomic_write_json(output / "sweep_state.json", state)
    gazebo_version = args.gazebo_version or resolve_gazebo_version()
    launcher = launcher or ProcessLauncher()
    rows: list[dict[str, Any]] = []
    counters = {
        "skipped_valid_count": 0,
        "retried_invalid_count": 0,
        "newly_executed_count": 0,
        "launch_count": 0,
    }
    execution_cases = sorted(cases, key=lambda item: item["execution_ordinal"])
    for case in execution_cases:
        case_dir = output / "cases" / case["case_id"]
        attempt_inspection = inspect_latest_attempt(
            case=case,
            case_dir=case_dir,
            provenance=provenance,
            sweep_id=args.sweep_id,
            warmup_count=args.warmup_eligible_samples,
            analysis_count=args.analysis_eligible_samples,
            plan_hash=plan["plan_hash"],
        )
        if attempt_inspection["classification"] == "VALID":
            valid_attempt = attempt_inspection["path"]
            attempt_state = attempt_inspection["state"]
            counters["skipped_valid_count"] += 1
            rows.append(_case_row(case, int(attempt_state["attempt"]), attempt_state))
            continue
        had_attempt = case_dir.exists() and any(case_dir.glob("attempt-*"))
        if had_attempt:
            counters["retried_invalid_count"] += 1
        counters["newly_executed_count"] += 1
        counters["launch_count"] += 1
        attempt_state = execute_case_attempt(
            case=case,
            output_dir=output,
            repository_root=REPOSITORY_ROOT,
            provenance=provenance,
            sweep_id=args.sweep_id,
            gazebo_version=gazebo_version,
            warmup_count=args.warmup_eligible_samples,
            analysis_count=args.analysis_eligible_samples,
            case_timeout_s=args.case_timeout_s,
            shutdown_timeout_s=args.shutdown_timeout_s,
            launcher=launcher,
            plan_hash=plan["plan_hash"],
            predecessor_inspection=attempt_inspection,
        )
        rows.append(_case_row(case, int(attempt_state["attempt"]), attempt_state))
        if not should_launch_next(
            case_valid=attempt_state["case_valid"],
            failure_policy=args.case_failure_policy,
        ):
            break
    summary = finalize_sweep(
        output_dir=output,
        plan=plan,
        cases=cases,
        rows=rows,
        counters=counters,
    )
    print(canonical_json(summary, indent=2))
    return 0 if summary["run_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
