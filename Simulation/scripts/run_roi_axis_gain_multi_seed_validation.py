#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run multi-seed ROI capture/profile validation, ROI silhouette "
            "axis-gain sweeps, and a final multi-report aggregate. This script "
            "is intended for an unsandboxed ROS2/Gazebo-capable environment; "
            "use --dry-run to verify commands without launching simulation."
        )
    )
    parser.add_argument("--seeds", default="20260635,20260636,20260637,20260638,20260639")
    parser.add_argument("--roi-save-stride", type=int, default=3)
    parser.add_argument("--report-prefix", default="reports/roi_reprojection_long_right_att_0p1")
    parser.add_argument("--sweep-prefix", default="reports/roi_silhouette_axis_gain_long_right_att_0p1")
    parser.add_argument(
        "--aggregate-output-dir",
        default=f"reports/roi_silhouette_axis_gain_multi_seed_summary_{dt.datetime.now():%Y%m%d_%H%M%S}",
    )
    parser.add_argument(
        "--include-sweep-dir",
        action="append",
        default=[],
        help="Existing axis-gain sweep directory or candidate summary to include in final aggregate.",
    )
    parser.add_argument("--python", default=sys.executable or "python3")
    parser.add_argument("--setup-file", default="install/setup.bash")
    parser.add_argument("--skip-existing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--continue-on-failure", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--right-roll-error-deg", type=float, default=0.1)
    parser.add_argument("--right-pitch-error-deg", type=float, default=0.1)
    parser.add_argument("--right-yaw-error-deg", type=float, default=0.1)
    parser.add_argument("--validation-trajectory-mode", choices=("random", "screen_coverage"), default="screen_coverage")
    parser.add_argument("--validation-screen-u-fracs", default="0.31,0.50,0.69")
    parser.add_argument("--validation-screen-v-fracs", default="0.31,0.50,0.69")
    parser.add_argument("--validation-ranges-m", default="300,450,650,950,1300")
    parser.add_argument("--validation-altitude-min-m", type=float, default=1.0)
    parser.add_argument("--validation-altitude-max-m", type=float, default=25.0)
    parser.add_argument("--validation-waypoint-repeat", type=int, default=2)
    parser.add_argument("--validation-min-waypoints-per-trial", type=int, default=4)
    parser.add_argument("--profile-min-cell-samples", type=int, default=6)
    parser.add_argument("--profile-disparity-gain", type=float, default=0.125)
    parser.add_argument("--profile-screen-u-gain", type=float, default=0.0)
    parser.add_argument("--profile-screen-v-gain", type=float, default=0.025)
    parser.add_argument("--profile-waypoint-repeat", type=int, default=5)
    parser.add_argument("--right-gains", default="0.0")
    parser.add_argument("--down-gains", default="0.0,0.1,0.25,0.5,0.75,1.0")
    parser.add_argument("--forward-gains", default="0.0,0.25,0.5,0.75,1.0")
    parser.add_argument("--modes", default="centroid,bbox_center,top_center,bottom_center")
    return parser.parse_args()


def parse_seed_list(text: str) -> list[int]:
    seeds: list[int] = []
    for part in str(text).split(","):
        item = part.strip()
        if not item:
            continue
        if "-" in item:
            start_text, end_text = item.split("-", 1)
            start = int(start_text)
            end = int(end_text)
            step = 1 if end >= start else -1
            seeds.extend(range(start, end + step, step))
        else:
            seeds.append(int(item))
    return seeds


def command_text(command: Sequence[str]) -> str:
    return " ".join(shlex.quote(str(part)) for part in command)


def source_command(setup_file: Path, command: Sequence[str]) -> list[str]:
    return ["bash", "-lc", f"source {shlex.quote(str(setup_file))} && exec {command_text(command)}"]


def report_dir(prefix: str, seed: int, roi_save_stride: int) -> Path:
    return ROOT_DIR / f"{prefix}_seed{seed}_stride{roi_save_stride}"


def sweep_dir(prefix: str, seed: int, roi_save_stride: int) -> Path:
    return ROOT_DIR / f"{prefix}_seed{seed}_stride{roi_save_stride}"


def report_complete(path: Path) -> bool:
    return (
        (path / "raw_samples.csv").exists()
        and (path / "normalized" / "trial_06.csv").exists()
        and (path / "validation_gate_summary.csv").exists()
    )


def sweep_complete(path: Path) -> bool:
    return (path / "axis_gain_candidate_summary.csv").exists() and (path / "axis_gain_trial_summary.csv").exists()


def run_command(
    command: Sequence[str],
    *,
    env: dict[str, str] | None = None,
    dry_run: bool,
    label: str,
) -> int:
    print(f"\n[{label}] {command_text(command)}", flush=True)
    if dry_run:
        return 0
    completed = subprocess.run(command, cwd=ROOT_DIR, env=env, check=False)
    return int(completed.returncode)


def validation_command(args: argparse.Namespace, seed: int, out_dir: Path) -> list[str]:
    return [
        args.python,
        str(SCRIPT_DIR / "reprojection_profile_random_validation.py"),
        "--seed",
        str(seed),
        "--output-dir",
        str(out_dir),
        "--right-roll-error-deg",
        f"{args.right_roll_error_deg:g}",
        "--right-pitch-error-deg",
        f"{args.right_pitch_error_deg:g}",
        "--right-yaw-error-deg",
        f"{args.right_yaw_error_deg:g}",
        "--validation-trajectory-mode",
        args.validation_trajectory_mode,
        "--validation-screen-u-fracs",
        args.validation_screen_u_fracs,
        "--validation-screen-v-fracs",
        args.validation_screen_v_fracs,
        "--validation-ranges-m",
        args.validation_ranges_m,
        "--validation-altitude-min-m",
        f"{args.validation_altitude_min_m:g}",
        "--validation-altitude-max-m",
        f"{args.validation_altitude_max_m:g}",
        "--validation-waypoint-repeat",
        str(args.validation_waypoint_repeat),
        "--validation-min-waypoints-per-trial",
        str(args.validation_min_waypoints_per_trial),
        "--profile-min-cell-samples",
        str(args.profile_min_cell_samples),
        "--profile-correction-mode",
        "component_gains",
        "--profile-disparity-gain",
        f"{args.profile_disparity_gain:g}",
        "--profile-screen-u-gain",
        f"{args.profile_screen_u_gain:g}",
        "--profile-screen-v-gain",
        f"{args.profile_screen_v_gain:g}",
        "--profile-output-policy",
        "range_only_forward_fusion",
        "--profile-waypoint-repeat",
        str(args.profile_waypoint_repeat),
        "--save-roi-images",
        "--roi-save-stride",
        str(args.roi_save_stride),
        "--roi-image-format",
        "ppm",
    ]


def sweep_command(args: argparse.Namespace, report_path: Path, out_dir: Path) -> list[str]:
    return [
        args.python,
        str(SCRIPT_DIR / "roi_silhouette_axis_gain_sweep.py"),
        str(report_path),
        "--output-dir",
        str(out_dir),
        "--modes",
        args.modes,
        "--right-gains",
        args.right_gains,
        "--down-gains",
        args.down_gains,
        "--forward-gains",
        args.forward_gains,
    ]


def aggregate_command(args: argparse.Namespace, inputs: Sequence[Path]) -> list[str]:
    return [
        args.python,
        str(SCRIPT_DIR / "roi_silhouette_axis_gain_aggregate.py"),
        *[str(path) for path in inputs],
        "--output-dir",
        str(ROOT_DIR / args.aggregate_output_dir),
    ]


def main() -> int:
    args = parse_args()
    seeds = parse_seed_list(args.seeds)
    setup_file = ROOT_DIR / args.setup_file
    if not setup_file.exists():
        raise SystemExit(f"missing setup file: {setup_file}")

    manifest: dict[str, object] = {
        "created_at": dt.datetime.now().isoformat(timespec="seconds"),
        "dry_run": bool(args.dry_run),
        "seeds": seeds,
        "roi_save_stride": args.roi_save_stride,
        "runs": [],
    }
    aggregate_inputs = [ROOT_DIR / item for item in args.include_sweep_dir]
    failures = 0
    for seed in seeds:
        current_report = report_dir(args.report_prefix, seed, args.roi_save_stride)
        current_sweep = sweep_dir(args.sweep_prefix, seed, args.roi_save_stride)
        run_record: dict[str, object] = {
            "seed": seed,
            "report_dir": str(current_report),
            "sweep_dir": str(current_sweep),
        }
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = f"/tmp/ros2-roi-seed{seed}"
        env["GZ_CONFIG_HOME"] = f"/tmp/gz-roi-seed{seed}"

        if args.skip_existing and report_complete(current_report):
            print(f"\n[seed {seed}] report already complete: {current_report}", flush=True)
            validation_code = 0
            run_record["validation"] = "skipped_existing"
        else:
            validation_code = run_command(
                source_command(setup_file, validation_command(args, seed, current_report)),
                env=env,
                dry_run=bool(args.dry_run),
                label=f"seed {seed} validation",
            )
            run_record["validation"] = f"exit_{validation_code}"
        if validation_code != 0:
            failures += 1
            if not args.continue_on_failure:
                manifest["runs"].append(run_record)
                break
            manifest["runs"].append(run_record)
            continue

        if args.skip_existing and sweep_complete(current_sweep):
            print(f"\n[seed {seed}] sweep already complete: {current_sweep}", flush=True)
            sweep_code = 0
            run_record["sweep"] = "skipped_existing"
        else:
            sweep_code = run_command(
                sweep_command(args, current_report, current_sweep),
                dry_run=bool(args.dry_run),
                label=f"seed {seed} axis_gain_sweep",
            )
            run_record["sweep"] = f"exit_{sweep_code}"
        if sweep_code != 0:
            failures += 1
            if not args.continue_on_failure:
                manifest["runs"].append(run_record)
                break
        if args.dry_run or sweep_complete(current_sweep):
            aggregate_inputs.append(current_sweep)
        manifest["runs"].append(run_record)

    if aggregate_inputs:
        aggregate_code = run_command(
            aggregate_command(args, aggregate_inputs),
            dry_run=bool(args.dry_run),
            label="aggregate",
        )
        manifest["aggregate"] = f"exit_{aggregate_code}"
        if aggregate_code != 0:
            failures += 1
    else:
        print("\n[aggregate] no completed sweep inputs", flush=True)
        manifest["aggregate"] = "no_inputs"
        failures += 1

    manifest_path = ROOT_DIR / args.aggregate_output_dir / "multi_seed_runner_manifest.json"
    if not args.dry_run:
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    else:
        print("\n[dry-run manifest]")
        print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
