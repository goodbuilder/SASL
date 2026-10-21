"""Read-only campaign observer; isolated post-success solver measurements.

Never edits the campaign or sources, launches a campaign, or signals another job.
"""
from __future__ import annotations

import argparse
import cProfile
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pstats
import resource
import signal
import statistics
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
SIM = HERE.parents[2]
CAMPAIGN = SIM / "recordings/stereo_v1_1/paper1_v1_2_001"
ANALYSIS = CAMPAIGN / "analysis_versions/analysis_9cfd7c6cd10b"
EXECUTION = CAMPAIGN / "source_evidence/full_analysis_execution_002"
VERIFY = CAMPAIGN / "source_evidence/post_analysis_verification_001"


def read(path):
    for attempt in range(5):
        try:
            return json.loads(path.read_text(encoding="utf-8-sig"))
        except json.JSONDecodeError:
            if attempt == 4:
                raise
            time.sleep(0.2)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False,
                      separators=(",", ":"))


def save(path, value):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    tmp.replace(path)


def event(status, **fields):
    with (HERE / "events.jsonl").open("a") as handle:
        handle.write(json.dumps({"observed_ns": time.time_ns(), "status": status,
                                 **fields}, ensure_ascii=False) + "\n")


def sources():
    return {str(p.relative_to(SIM)): digest(p)
            for p in (SIM / "campaigns/stereo_v1_1").glob("*.py")}


def tail(path):
    with path.open("rb") as handle:
        handle.seek(max(0, path.stat().st_size - 65536))
        return handle.read().decode("utf-8", errors="replace").splitlines()


def baseline():
    rows = []
    lines = (EXECUTION / "resource_monitor.jsonl").read_text().splitlines(keepends=True)
    for number, line in enumerate(lines):
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            if number == len(lines) - 1 and not line.endswith("\n"):
                break  # Appender may currently be writing the final record.
            raise
        if "family_cpu_seconds" in row:
            rows.append(row)
    first, last = rows[0], rows[-1]
    elapsed = (last["observed_ns"] - first["observed_ns"]) / 1e9
    cores = ((last["family_cpu_seconds"] - first["family_cpu_seconds"]) / elapsed
             if elapsed else None)
    save(HERE / "baseline.json", {
        "source": str(EXECUTION / "resource_monitor.jsonl"),
        "sample_count": len(rows), "observation_seconds": elapsed,
        "mean_busy_cpu_cores": cores,
        "maximum_sampled_family_rss_bytes": max(r["family_rss_bytes"] for r in rows),
        "first": first, "latest": last,
        "limits": "Sampled RSS, not peak RSS. CPU counter assumes a stable family; "
                  "lost exited workers can invalidate aggregate deltas. "
                  "Calibration log is task submission, not completion. No full-run ETA inferred.",
    })
    return last["observed_ns"]


def completion_ready():
    if not (EXECUTION / "result.json").is_file():
        return False
    if read(EXECUTION / "result.json").get("exit_code") != 0:
        raise RuntimeError("analysis failed; benchmark held")
    if not any('"status": "ANALYSIS_COMPLETE"' in line or '"status":"ANALYSIS_COMPLETE"' in line
               for line in tail(EXECUTION / "analysis.log")):
        raise RuntimeError("terminal success log missing")
    if read(CAMPAIGN / "analysis_latest.json").get("analysis_id") != ANALYSIS.name:
        raise RuntimeError("latest pointer mismatch")
    for name in ("analysis_summary.json", "analysis_freeze.json", "analysis_checksums.sha256"):
        if not (ANALYSIS / name).is_file():
            raise RuntimeError("missing completion artifact: " + name)
    if not (VERIFY / "result.json").is_file():
        return False
    verified = read(VERIFY / "result.json")
    if verified.get("status") != "AUDIT_HEADING_SUMMARY_STAGES_COMPLETE":
        raise RuntimeError("existing post-analysis verification failed")
    if verified.get("analysis_id") != ANALYSIS.name:
        raise RuntimeError("post-analysis receipt version mismatch")
    if read(VERIFY / "analysis_results_audit.json").get("status") != "PASS":
        raise RuntimeError("result auditor did not PASS")
    return True


def reconstruct():
    # Import only after successful completion and unchanged-source checks.
    sys.path.insert(0, str(SIM))
    from campaigns.stereo_v1_1 import analyze_campaign as module
    index = read(ANALYSIS / "calibration_run_index.json")
    slots = read(CAMPAIGN / "run_status.json")["run_slots"]
    spec = read(CAMPAIGN / "frozen_protocol/v1_2_specification.json")
    intrinsics = {key: spec["camera"][key]
                  for key in ("fx_px", "fy_px", "cx_px", "cy_px")}
    selected_runs = ("G0-P0-R01", "G0-P1-R01", "G0-P4-R01", "G1-P4-R01")
    payloads, evidence = [], []
    for run_id in selected_runs:
        slot = next(row for row in slots if row["run_id"] == run_id)
        pairs, _summary = module._image_pairs(CAMPAIGN, slot)
        by_id = {row["pair_id"]: row for row in pairs}
        if len(by_id) != len(pairs):
            raise ValueError("duplicate source pair IDs")
        yaw = "Y0" if run_id.startswith("G0") else "G1_PRIMARY_ZERO"
        for scope in ("all_data", "matched_200"):
            batches = [row for row in index["batches"]
                       if row["run_id"] == run_id and row["trial_index"] == 0
                       and row["yaw_case"] == yaw and row["data_scope"] == scope]
            if len(batches) != 1:
                raise ValueError("missing unique completed calibration batch")
            batch = batches[0]
            case = (ANALYSIS / batch["case_directory"]).resolve()
            if not case.is_relative_to(ANALYSIS.resolve()):
                raise ValueError("case path escape")
            if digest(case / "calibration_results.json") != batch["calibration_results_sha256"]:
                raise ValueError("batch result hash mismatch")
            checks = read(case / "calibration_checksums.json")
            # Producer's checksum file is a direct relative-name -> SHA256 map.
            for name in ("sensor_observations.json", "trial_config.json",
                         "drone_rtk_observations.jsonl", "calibration_results.json"):
                if digest(case / name) != checks[name]:
                    raise ValueError("case input hash mismatch: " + name)
            bundle = read(case / "calibration_results.json")
            stored = [row for row in bundle["results"]
                      if row["method"] in ("A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint")
                      and row["attitude_variant"] in ("RPY_IMU", "YAW_ONLY")]
            if len(stored) != 6:
                raise ValueError("expected six primary method/variant results")
            pair_ids = stored[0]["selected_pair_ids"]
            if len(set(pair_ids)) != len(pair_ids) or not pair_ids:
                raise ValueError("empty/duplicate solver pair IDs")
            if any(row["selected_pair_ids"] != pair_ids for row in stored):
                raise ValueError("primary solver masks differ")
            ordered = [by_id[key] for key in pair_ids]
            rtk_source = module._read_jsonl(case / "drone_rtk_observations.jsonl")
            rtk = [module._interpolate_rtk(rtk_source, pair) for pair in ordered]
            if any(row is None for row in rtk):
                raise ValueError("missing aligned RTK")
            config = read(case / "trial_config.json")
            config = {key: config[key] for key in (
                "initial_yaw_rad", "pixel_sigma_px", "mc_seed", "calibration_run_id",
                "evaluation_run_id", "yaw_case", "trial_index", "sample_scope")}
            payloads.append({"key": run_id + ":" + scope,
                             "sensor_observations": read(case / "sensor_observations.json"),
                             "trial_config": config, "image_pairs": ordered,
                             "intrinsics": intrinsics, "rtk": rtk,
                             "stored_results": stored})
            evidence.append({"key": payloads[-1]["key"], "case": str(case),
                             "selected_pair_ids": pair_ids,
                             "source_result_sha256": batch["calibration_results_sha256"]})
    save(HERE / "input_snapshot.json", {"payloads": payloads, "evidence": evidence})
    return payloads


def solve_task(task):
    sys.path.insert(0, str(SIM))
    from campaigns.stereo_v1_1.calibration_solver import solve_calibration
    began = time.perf_counter()
    rows = []
    for method in ("A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint"):
        for variant in ("RPY_IMU", "YAW_ONLY"):
            kwargs = {key: task[key] for key in (
                "sensor_observations", "trial_config", "image_pairs", "intrinsics")}
            rows.append(solve_calibration(
                method=method, attitude_variant=variant, **kwargs,
                drone_rtk_observations=None if method.startswith("C_") else task["rtk"]))
    for row in rows:
        expected = next(r for r in task["stored_results"]
                        if r["method"] == row["method"]
                        and r["attitude_variant"] == row["attitude_variant"])
        if canonical(row) != canonical({key: expected[key] for key in row}):
            raise ValueError("reconstructed baseline differs from stored result: " + task["key"])
    return {"key": task["key"], "results_sha256": hashlib.sha256(
        canonical(rows).encode()).hexdigest(), "seconds": time.perf_counter() - began,
        "worker_peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024}


def benchmark(workers):
    sys.path.insert(0, str(SIM))
    from campaigns.stereo_v1_1.analysis_execution import ordered_task_results
    tasks = read(HERE / "input_snapshot.json")["payloads"]
    began = time.perf_counter()
    rows = list(ordered_task_results(solve_task, [(t["key"], t) for t in tasks], workers=workers))
    print(json.dumps({"workers": workers, "wall_seconds": time.perf_counter() - began,
                      "tasks": rows}), flush=True)


def profile_solver():
    sys.path.insert(0, str(SIM))
    from campaigns.stereo_v1_1.calibration_solver import solve_calibration
    task = next(t for t in read(HERE / "input_snapshot.json")["payloads"]
                if t["key"] == "G0-P0-R01:matched_200")
    kwargs = {key: task[key] for key in (
        "sensor_observations", "trial_config", "image_pairs", "intrinsics")}
    profiler = cProfile.Profile()
    result = profiler.runcall(solve_calibration, method="B_rtk_joint", attitude_variant="RPY_IMU",
                             **kwargs, drone_rtk_observations=task["rtk"])
    expected = next(r for r in task["stored_results"]
                    if r["method"] == "B_rtk_joint" and r["attitude_variant"] == "RPY_IMU")
    if canonical(result) != canonical({key: expected[key] for key in result}):
        raise ValueError("profile result differs from stored baseline")
    profiler.dump_stats(str(HERE / "solver_profile.pstats"))
    with (HERE / "solver_profile.txt").open("w") as stream:
        pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(50)
    print(json.dumps({"status": "PROFILE_COMPLETE", "key": task["key"]}))


def owned_command(arguments, label):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="1",
               OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    with (HERE / (label + ".stderr.log")).open("w") as err:
        process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), *arguments],
                                   cwd=SIM, env=env, stdout=subprocess.PIPE, stderr=err,
                                   text=True, start_new_session=True)
        try:
            stdout, _ = process.communicate(timeout=3600)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise RuntimeError("isolated measurement timeout; owned process group stopped")
    if process.returncode:
        raise RuntimeError(label + " failed; see stderr log")
    return json.loads(stdout)


def supervise():
    pinned = sources()
    save(HERE / "source_identity.json", pinned)
    event("WAITING_FOR_ANALYSIS_AND_EXISTING_AUDIT", analysis_id=ANALYSIS.name)
    while True:
        last_observed_ns = baseline()
        if completion_ready():
            break
        if not (EXECUTION / "result.json").exists() and time.time_ns() - last_observed_ns > 180e9:
            raise RuntimeError("existing resource monitor stale >180s; completion unknown, inspect")
        time.sleep(30)
    event("V02_V03_PASS")
    if sources() != pinned:
        raise RuntimeError("source changed while waiting; replan required")
    for name, expected in read(ANALYSIS / "analysis_run_manifest.json")["algorithm_sha256"].items():
        if digest(SIM / "campaigns/stereo_v1_1" / name) != expected:
            raise RuntimeError("source differs from completed analysis: " + name)
    with (CAMPAIGN / ".analysis.lock").open("r") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        sys.path.insert(0, str(SIM))
        from campaigns.stereo_v1_1 import analyze_campaign as module
        if module._analysis_dependencies() != read(ANALYSIS / "analysis_run_manifest.json")["dependencies"]:
            raise RuntimeError("dependencies differ from completed analysis")
        reconstruct()
        event("V04_INPUT_SNAPSHOT_READY")
        rounds = []
        reference = None
        for number, workers in enumerate((4, 8, 8, 4, 4, 8), 1):
            if sources() != pinned:
                raise RuntimeError("source changed before benchmark")
            row = owned_command(["--benchmark", str(workers)], f"benchmark_{number:02d}")
            actual = {t["key"]: t["results_sha256"] for t in row["tasks"]}
            if reference is None:
                reference = actual
            if actual != reference:
                raise RuntimeError("result identity changed between worker candidates")
            rounds.append(row)
            save(HERE / "benchmark_rounds.json", rounds)
            event("BENCHMARK_ROUND_PASS", round=number, workers=workers,
                  wall_seconds=row["wall_seconds"])
        medians = {str(w): statistics.median(r["wall_seconds"] for r in rounds if r["workers"] == w)
                   for w in (4, 8)}
        owned_command(["--profile"], "profile")
        save(HERE / "result.json", {
            "status": "BASELINE_AND_WORKER_BENCHMARK_COMPLETE", "medians_seconds": medians,
            "results_identical_across_worker_candidates": True,
            "completed_ns": time.time_ns(),
            "limits": "Eight real trial0 Y0/nominal-zero input tasks only. Not a full campaign timing. "
                      "No optimizer/source/config changed. RSS for original run is sampled in baseline.json. "
                      "Baseline results exactly match stored primary result fields. "
                      "Worker RSS is individual process peak, not total concurrent family peak.",
            "next": "Review profile and select minimal optimization; V06/V07 NOT_RUN."})
        event("V05_COMPLETE_V06_PENDING")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", type=int)
    parser.add_argument("--profile", action="store_true")
    args = parser.parse_args()
    if args.benchmark:
        benchmark(args.benchmark)
    elif args.profile:
        profile_solver()
    else:
        try:
            with (HERE / "supervisor.lock").open("a") as own_lock:
                fcntl.flock(own_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                supervise()
        except Exception as exc:
            event("HOLD", reason=f"{type(exc).__name__}: {exc}")
            save(HERE / "result.json", {"status": "HOLD", "reason": str(exc)})
            raise
