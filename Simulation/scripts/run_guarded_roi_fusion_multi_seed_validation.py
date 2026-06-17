#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
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
            "Run multi-seed ROI capture/profile validation, guarded ROI/range "
            "fusion evaluation, multi-report aggregation, bootstrap diagnostics, "
            "and the stability gate. Intended for unsandboxed ROS2/Gazebo; use "
            "--dry-run for command verification."
        )
    )
    parser.add_argument("--seeds", default="20260635,20260636,20260637,20260638,20260639")
    parser.add_argument("--roi-save-stride", type=int, default=3)
    parser.add_argument("--report-prefix", default="reports/roi_reprojection_long_right_att_0p1")
    parser.add_argument("--fusion-prefix", default="reports/roi_guarded_down_range_forward_fusion")
    parser.add_argument(
        "--aggregate-output-dir",
        default=f"reports/roi_guarded_down_range_forward_fusion_multi_seed_{dt.datetime.now():%Y%m%d_%H%M%S}",
    )
    parser.add_argument("--include-fusion-dir", action="append", default=[])
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

    parser.add_argument("--mode", default="top_center")
    parser.add_argument("--down-gain", type=float, default=0.1)
    parser.add_argument("--forward-gain", type=float, default=1.0)
    parser.add_argument("--forward-min-range-m", type=float, default=500.0)
    parser.add_argument("--forward-max-range-m", type=float)
    parser.add_argument("--forward-min-correction-abs-m", type=float)
    parser.add_argument("--disable-down-screen-bins", default="0:1")
    parser.add_argument("--bootstrap-iterations", type=int, default=1000)
    parser.add_argument("--bootstrap-seed", type=int, default=20260609)
    parser.add_argument("--yolo-model", default="")
    parser.add_argument("--yolo-python", default="")
    parser.add_argument("--yolo-pythonpath", default="")
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


def run_command(command: Sequence[str], *, env: dict[str, str] | None = None, dry_run: bool, label: str) -> int:
    print(f"\n[{label}] {command_text(command)}", flush=True)
    if dry_run:
        return 0
    completed = subprocess.run(command, cwd=ROOT_DIR, env=env, check=False)
    return int(completed.returncode)


def report_dir(prefix: str, seed: int, roi_save_stride: int) -> Path:
    return ROOT_DIR / f"{prefix}_seed{seed}_stride{roi_save_stride}"


def fusion_dir(prefix: str, seed: int, roi_save_stride: int) -> Path:
    return ROOT_DIR / f"{prefix}_seed{seed}_stride{roi_save_stride}"


def report_complete(path: Path) -> bool:
    return (
        (path / "raw_samples.csv").exists()
        and (path / "normalized" / "trial_06.csv").exists()
        and (path / "validation_gate_summary.csv").exists()
    )


def fusion_complete(path: Path) -> bool:
    return (
        (path / "axis_gain_candidate_summary.csv").exists()
        and (path / "candidate_sample_diagnostics.csv").exists()
        and (path / "candidate_group_summary.csv").exists()
    )


def validation_command(args: argparse.Namespace, seed: int, out_dir: Path) -> list[str]:
    command = [
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
    if args.yolo_python:
        command.extend(["--yolo-python", args.yolo_python])
    if args.yolo_pythonpath:
        command.extend(["--yolo-pythonpath", args.yolo_pythonpath])
    if args.yolo_model:
        command.extend(["--yolo-model", args.yolo_model])
    return command


def fusion_command(args: argparse.Namespace, report_path: Path, out_dir: Path) -> list[str]:
    command = [
        args.python,
        str(SCRIPT_DIR / "roi_down_range_forward_fusion.py"),
        str(report_path),
        "--output-dir",
        str(out_dir),
        "--modes",
        args.mode,
        "--down-gains",
        f"{args.down_gain:g}",
        "--forward-gains",
        f"{args.forward_gain:g}",
        "--diagnostic-mode",
        args.mode,
        "--diagnostic-down-gain",
        f"{args.down_gain:g}",
        "--diagnostic-forward-gain",
        f"{args.forward_gain:g}",
        "--disable-down-screen-bins",
        args.disable_down_screen_bins,
    ]
    if args.forward_min_range_m is not None:
        command.extend(["--forward-min-range-m", f"{args.forward_min_range_m:g}"])
    if args.forward_max_range_m is not None:
        command.extend(["--forward-max-range-m", f"{args.forward_max_range_m:g}"])
    if args.forward_min_correction_abs_m is not None:
        command.extend(["--forward-min-correction-abs-m", f"{args.forward_min_correction_abs_m:g}"])
    return command


def aggregate_command(args: argparse.Namespace, inputs: Sequence[Path], output_dir: Path) -> list[str]:
    return [
        args.python,
        str(SCRIPT_DIR / "roi_silhouette_axis_gain_aggregate.py"),
        *[str(path) for path in inputs],
        "--output-dir",
        str(output_dir),
    ]


def sample_aggregate_command(args: argparse.Namespace, inputs: Sequence[Path], output_dir: Path) -> list[str]:
    return [
        args.python,
        str(SCRIPT_DIR / "roi_candidate_sample_aggregate.py"),
        *[str(path) for path in inputs],
        "--output-dir",
        str(output_dir),
    ]


def bootstrap_command(args: argparse.Namespace, sample_csv: Path, output_dir: Path) -> list[str]:
    return [
        args.python,
        str(SCRIPT_DIR / "roi_axis_gain_bootstrap_diagnostics.py"),
        str(sample_csv),
        "--output-dir",
        str(output_dir),
        "--iterations",
        str(args.bootstrap_iterations),
        "--seed",
        str(args.bootstrap_seed),
    ]


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def combine_skipped(candidate_skipped: Path, sample_skipped: Path, output_path: Path) -> None:
    rows: list[dict[str, object]] = []
    for source, path in (("candidate_summary", candidate_skipped), ("sample_diagnostics", sample_skipped)):
        for row in read_csv(path):
            combined = dict(row)
            combined["skip_source"] = source
            rows.append(combined)
    write_csv(output_path, rows)


def stability_gate_command(
    args: argparse.Namespace,
    *,
    aggregate_dir: Path,
    sample_aggregate_dir: Path,
    bootstrap_dir: Path,
    gate_dir: Path,
    skipped_path: Path,
    report_paths: Sequence[Path],
) -> list[str]:
    command = [
        args.python,
        str(SCRIPT_DIR / "roi_axis_gain_stability_gate.py"),
        "--aggregate-summary",
        str(aggregate_dir / "axis_gain_multi_report_summary.csv"),
        "--skipped-inputs",
        str(skipped_path),
        "--candidate-mode",
        args.mode,
        "--right-gain",
        "0",
        "--down-gain",
        f"{args.down_gain:g}",
        "--forward-gain",
        f"{args.forward_gain:g}",
        "--candidate-sample-diagnostics",
        str(sample_aggregate_dir / "candidate_sample_diagnostics.csv"),
        "--bootstrap-summary",
        str(bootstrap_dir / "bootstrap_summary.csv"),
        "--output-dir",
        str(gate_dir),
    ]
    for report_path in report_paths:
        profile = report_path / "profile_coverage_summary.json"
        if profile.exists():
            command.extend(["--profile-coverage-summary", str(profile)])
        trajectory = report_path / "trajectory_projection_diagnostic.json"
        if trajectory.exists():
            command.extend(["--trajectory-diagnostics", str(trajectory)])
    return command


def main() -> int:
    args = parse_args()
    seeds = parse_seed_list(args.seeds)
    setup_file = ROOT_DIR / args.setup_file
    if not args.dry_run and seeds and not setup_file.exists():
        raise SystemExit(f"missing setup file: {setup_file}")

    output_root = ROOT_DIR / args.aggregate_output_dir
    aggregate_dir = output_root / "candidate_aggregate"
    sample_aggregate_dir = output_root / "sample_aggregate"
    bootstrap_dir = output_root / "bootstrap"
    gate_dir = output_root / "stability_gate"
    combined_skipped = output_root / "combined_skipped_inputs.csv"
    manifest: dict[str, object] = {
        "created_at": dt.datetime.now().isoformat(timespec="seconds"),
        "dry_run": bool(args.dry_run),
        "seeds": seeds,
        "roi_save_stride": args.roi_save_stride,
        "guarded_candidate": {
            "mode": args.mode,
            "right_gain": 0.0,
            "down_gain": args.down_gain,
            "forward_gain": args.forward_gain,
            "forward_min_range_m": args.forward_min_range_m,
            "forward_max_range_m": args.forward_max_range_m,
            "forward_min_correction_abs_m": args.forward_min_correction_abs_m,
            "disable_down_screen_bins": args.disable_down_screen_bins,
        },
        "runs": [],
    }
    fusion_inputs = [ROOT_DIR / item for item in args.include_fusion_dir]
    report_paths: list[Path] = []
    failures = 0
    for seed in seeds:
        current_report = report_dir(args.report_prefix, seed, args.roi_save_stride)
        current_fusion = fusion_dir(args.fusion_prefix, seed, args.roi_save_stride)
        run_record: dict[str, object] = {
            "seed": seed,
            "report_dir": str(current_report),
            "fusion_dir": str(current_fusion),
        }
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = f"/tmp/ros2-guarded-roi-seed{seed}"
        env["GZ_CONFIG_HOME"] = f"/tmp/gz-guarded-roi-seed{seed}"
        if args.skip_existing and report_complete(current_report):
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
            manifest["runs"].append(run_record)
            if not args.continue_on_failure:
                break
            continue
        report_paths.append(current_report)

        if args.skip_existing and fusion_complete(current_fusion):
            fusion_code = 0
            run_record["fusion"] = "skipped_existing"
        else:
            fusion_code = run_command(
                fusion_command(args, current_report, current_fusion),
                dry_run=bool(args.dry_run),
                label=f"seed {seed} guarded_fusion",
            )
            run_record["fusion"] = f"exit_{fusion_code}"
        if fusion_code != 0:
            failures += 1
            manifest["runs"].append(run_record)
            if not args.continue_on_failure:
                break
            continue
        if args.dry_run or fusion_complete(current_fusion):
            fusion_inputs.append(current_fusion)
        manifest["runs"].append(run_record)

    if not fusion_inputs:
        manifest["aggregate"] = "no_inputs"
        failures += 1
    else:
        for label, command in (
            ("candidate aggregate", aggregate_command(args, fusion_inputs, aggregate_dir)),
            ("sample aggregate", sample_aggregate_command(args, fusion_inputs, sample_aggregate_dir)),
        ):
            code = run_command(command, dry_run=bool(args.dry_run), label=label)
            manifest[label.replace(" ", "_")] = f"exit_{code}"
            if code != 0:
                failures += 1
                if not args.continue_on_failure:
                    break
        if not args.dry_run:
            combine_skipped(
                aggregate_dir / "skipped_inputs.csv",
                sample_aggregate_dir / "skipped_inputs.csv",
                combined_skipped,
            )
        bootstrap_code = run_command(
            bootstrap_command(args, sample_aggregate_dir / "candidate_sample_diagnostics.csv", bootstrap_dir),
            dry_run=bool(args.dry_run),
            label="bootstrap",
        )
        manifest["bootstrap"] = f"exit_{bootstrap_code}"
        if bootstrap_code != 0:
            failures += 1
        gate_code = run_command(
            stability_gate_command(
                args,
                aggregate_dir=aggregate_dir,
                sample_aggregate_dir=sample_aggregate_dir,
                bootstrap_dir=bootstrap_dir,
                gate_dir=gate_dir,
                skipped_path=combined_skipped,
                report_paths=report_paths,
            ),
            dry_run=bool(args.dry_run),
            label="stability gate",
        )
        manifest["stability_gate"] = f"exit_{gate_code}"
        if gate_code != 0:
            failures += 1

    if args.dry_run:
        print("\n[dry-run manifest]")
        print(json.dumps(manifest, ensure_ascii=False, indent=2))
    else:
        output_root.mkdir(parents=True, exist_ok=True)
        (output_root / "guarded_multi_seed_runner_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
