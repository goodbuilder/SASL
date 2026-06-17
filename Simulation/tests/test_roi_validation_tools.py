from __future__ import annotations

import csv
import importlib.util
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_script_module(name: str, relative_path: str):
    spec = importlib.util.spec_from_file_location(name, ROOT_DIR / relative_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load module: {relative_path}")
    module = importlib.util.module_from_spec(spec)
    scripts_dir = str(ROOT_DIR / "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    spec.loader.exec_module(module)
    return module


class RoiValidationToolTests(unittest.TestCase):
    def test_screen_coverage_trajectory_diagnostics_covers_validation_bins(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir) / "trajectory"
            command = [
                sys.executable,
                str(ROOT_DIR / "scripts" / "reprojection_profile_random_validation.py"),
                "--seed",
                "20260635",
                "--output-dir",
                str(output_dir),
                "--right-roll-error-deg",
                "0.1",
                "--right-pitch-error-deg",
                "0.1",
                "--right-yaw-error-deg",
                "0.1",
                "--profile-waypoint-repeat",
                "5",
                "--validation-trajectory-mode",
                "screen_coverage",
                "--validation-screen-u-fracs",
                "0.31,0.50,0.69",
                "--validation-screen-v-fracs",
                "0.31,0.50,0.69",
                "--validation-ranges-m",
                "300,450,650,950,1300",
                "--validation-altitude-min-m",
                "1.0",
                "--validation-altitude-max-m",
                "25.0",
                "--validation-waypoint-repeat",
                "2",
                "--validation-min-waypoints-per-trial",
                "4",
                "--trajectory-diagnostics-only",
            ]
            completed = subprocess.run(command, cwd=ROOT_DIR, text=True, capture_output=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            diagnostic = json.loads((output_dir / "trajectory_projection_diagnostic.json").read_text())
            self.assertEqual(diagnostic["trajectory_mode"], "screen_coverage")
            self.assertEqual(diagnostic["validation_screen_bin_count"], 9)
            self.assertEqual(diagnostic["screen_bin_total"], 9)
            self.assertEqual(diagnostic["validation_screen_bin_coverage_ratio"], 1.0)
            validation_trials = [row for row in diagnostic["trial_rows"] if row["trial"] >= 2]
            self.assertEqual(len(validation_trials), 5)
            self.assertTrue(all(row["projected_count"] >= 4 for row in validation_trials))

    def test_axis_gain_aggregate_skips_partial_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            complete = root / "complete"
            partial = root / "partial"
            output = root / "aggregate"
            complete.mkdir()
            partial.mkdir()
            fieldnames = [
                "mode",
                "right_gain",
                "down_gain",
                "forward_gain",
                "pass_all_nonregression",
                "min_right_rmse_improvement_m",
                "min_down_rmse_improvement_m",
                "min_forward_rmse_improvement_m",
                "min_position_rmse_improvement_m",
                "mean_right_rmse_improvement_m",
                "mean_down_rmse_improvement_m",
                "mean_forward_rmse_improvement_m",
                "mean_position_rmse_improvement_m",
            ]
            with (complete / "axis_gain_candidate_summary.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow(
                    {
                        "mode": "top_center",
                        "right_gain": "0",
                        "down_gain": "0.75",
                        "forward_gain": "0.75",
                        "pass_all_nonregression": "yes",
                        "min_right_rmse_improvement_m": "0",
                        "min_down_rmse_improvement_m": "0.05",
                        "min_forward_rmse_improvement_m": "0.5",
                        "min_position_rmse_improvement_m": "0.5",
                        "mean_right_rmse_improvement_m": "0",
                        "mean_down_rmse_improvement_m": "0.07",
                        "mean_forward_rmse_improvement_m": "3.7",
                        "mean_position_rmse_improvement_m": "3.7",
                    }
                )
            command = [
                sys.executable,
                str(ROOT_DIR / "scripts" / "roi_silhouette_axis_gain_aggregate.py"),
                str(complete),
                str(partial),
                "--output-dir",
                str(output),
            ]
            completed = subprocess.run(command, cwd=ROOT_DIR, text=True, capture_output=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            summary_rows = list(csv.DictReader((output / "axis_gain_multi_report_summary.csv").open()))
            skipped_rows = list(csv.DictReader((output / "skipped_inputs.csv").open()))
            self.assertEqual(len(summary_rows), 1)
            self.assertEqual(summary_rows[0]["mode"], "top_center")
            self.assertEqual(summary_rows[0]["pass_all_reports"], "yes")
            self.assertEqual(summary_rows[0]["report_count"], "1")
            self.assertEqual(len(skipped_rows), 1)
            self.assertEqual(skipped_rows[0]["reason"], "missing_axis_gain_candidate_summary")

    def test_candidate_sample_aggregate_skips_partial_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            complete = root / "complete"
            partial = root / "partial"
            output = root / "sample_aggregate"
            complete.mkdir()
            partial.mkdir()
            with (complete / "candidate_sample_diagnostics.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "fallback",
                        "trial",
                        "raw_error_right_m",
                        "corrected_error_right_m",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "fallback": "none",
                        "trial": "2",
                        "raw_error_right_m": "1.0",
                        "corrected_error_right_m": "0.5",
                    }
                )
            command = [
                sys.executable,
                str(ROOT_DIR / "scripts" / "roi_candidate_sample_aggregate.py"),
                str(complete),
                str(partial),
                "--output-dir",
                str(output),
            ]
            completed = subprocess.run(command, cwd=ROOT_DIR, text=True, capture_output=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            sample_rows = list(csv.DictReader((output / "candidate_sample_diagnostics.csv").open()))
            skipped_rows = list(csv.DictReader((output / "skipped_inputs.csv").open()))
            metadata = json.loads((output / "metadata.json").read_text())
            self.assertEqual(len(sample_rows), 1)
            self.assertEqual(sample_rows[0]["report_id"], "complete")
            self.assertEqual(sample_rows[0]["source_diagnostics"], str(complete / "candidate_sample_diagnostics.csv"))
            self.assertEqual(len(skipped_rows), 1)
            self.assertEqual(skipped_rows[0]["reason"], "missing_candidate_sample_diagnostics")
            self.assertEqual(metadata["loaded_input_count"], 1)
            self.assertEqual(metadata["skipped_input_count"], 1)

    def test_axis_gain_stability_gate_rejects_current_weak_evidence_shape(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            aggregate = root / "axis_gain_multi_report_summary.csv"
            skipped = root / "skipped_inputs.csv"
            samples = root / "candidate_sample_diagnostics.csv"
            bootstrap = root / "bootstrap_summary.csv"
            output = root / "gate"

            with aggregate.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "mode",
                        "right_gain",
                        "down_gain",
                        "forward_gain",
                        "report_count",
                        "required_report_count",
                        "pass_all_reports",
                        "min_right_rmse_improvement_m",
                        "min_down_rmse_improvement_m",
                        "min_forward_rmse_improvement_m",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "mode": "top_center",
                        "right_gain": "0",
                        "down_gain": "0.75",
                        "forward_gain": "0.75",
                        "report_count": "1",
                        "required_report_count": "1",
                        "pass_all_reports": "yes",
                        "min_right_rmse_improvement_m": "0",
                        "min_down_rmse_improvement_m": "0.05",
                        "min_forward_rmse_improvement_m": "0.5",
                    }
                )
            with skipped.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["input", "reason"])
                writer.writeheader()
                writer.writerow({"input": "partial", "reason": "missing_axis_gain_candidate_summary"})
            with samples.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=["fallback", "trial", "screen_u_bin", "screen_v_bin", "range_bin"],
                )
                writer.writeheader()
                for index in range(20):
                    writer.writerow(
                        {
                            "fallback": "none" if index < 3 else "invalid_or_missing_roi",
                            "trial": str(2 + index % 5),
                            "screen_u_bin": str(index % 3),
                            "screen_v_bin": "1",
                            "range_bin": str(index % 3),
                        }
                    )
            with bootstrap.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "group",
                        "bootstrap_pass_fraction",
                        "right_p025_m",
                        "down_p025_m",
                        "forward_p025_m",
                        "trial",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "group": "trial",
                        "bootstrap_pass_fraction": "0.75",
                        "right_p025_m": "0",
                        "down_p025_m": "0.01",
                        "forward_p025_m": "-0.2",
                        "trial": "2",
                    }
                )

            command = [
                sys.executable,
                str(ROOT_DIR / "scripts" / "roi_axis_gain_stability_gate.py"),
                "--aggregate-summary",
                str(aggregate),
                "--skipped-inputs",
                str(skipped),
                "--candidate-mode",
                "top_center",
                "--right-gain",
                "0",
                "--down-gain",
                "0.75",
                "--forward-gain",
                "0.75",
                "--candidate-sample-diagnostics",
                str(samples),
                "--bootstrap-summary",
                str(bootstrap),
                "--output-dir",
                str(output),
            ]
            completed = subprocess.run(command, cwd=ROOT_DIR, text=True, capture_output=True, check=False)
            self.assertNotEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            summary = json.loads((output / "stability_gate_summary.json").read_text())
            self.assertEqual(summary["status"], "fail")
            failed_checks = {row["check"] for row in summary["failed_checks"]}
            self.assertIn("completed_report_count", failed_checks)
            self.assertIn("skipped_input_count", failed_checks)
            self.assertIn("valid_roi_fraction", failed_checks)
            self.assertIn("validation_screen_bin_count", failed_checks)
            self.assertIn("bootstrap_pass_fraction:trial=2", failed_checks)
            self.assertIn("bootstrap_forward_p025:trial=2", failed_checks)

    def test_axis_gain_stability_gate_accepts_strong_synthetic_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            aggregate = root / "axis_gain_multi_report_summary.csv"
            skipped = root / "skipped_inputs.csv"
            samples = root / "candidate_sample_diagnostics.csv"
            bootstrap = root / "bootstrap_summary.csv"
            output = root / "gate"

            with aggregate.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "mode",
                        "right_gain",
                        "down_gain",
                        "forward_gain",
                        "report_count",
                        "required_report_count",
                        "pass_all_reports",
                        "min_right_rmse_improvement_m",
                        "min_down_rmse_improvement_m",
                        "min_forward_rmse_improvement_m",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "mode": "top_center",
                        "right_gain": "0",
                        "down_gain": "0.75",
                        "forward_gain": "0.75",
                        "report_count": "6",
                        "required_report_count": "6",
                        "pass_all_reports": "yes",
                        "min_right_rmse_improvement_m": "0",
                        "min_down_rmse_improvement_m": "0.05",
                        "min_forward_rmse_improvement_m": "0.5",
                    }
                )
            with skipped.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["input", "reason"])
                writer.writeheader()
            with samples.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=["fallback", "trial", "screen_u_bin", "screen_v_bin", "range_bin"],
                )
                writer.writeheader()
                for u_bin in range(3):
                    for v_bin in range(3):
                        for repeat in range(10):
                            writer.writerow(
                                {
                                    "fallback": "none",
                                    "trial": str(2 + repeat % 5),
                                    "screen_u_bin": str(u_bin),
                                    "screen_v_bin": str(v_bin),
                                    "range_bin": str(repeat % 5),
                                }
                            )
            with bootstrap.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "group",
                        "bootstrap_pass_fraction",
                        "right_p025_m",
                        "down_p025_m",
                        "forward_p025_m",
                        "trial",
                        "range_bin",
                        "screen_u_bin",
                        "screen_v_bin",
                    ],
                )
                writer.writeheader()
                rows = [{"group": "all"}]
                rows.extend({"group": "trial", "trial": str(trial)} for trial in range(2, 7))
                rows.extend({"group": "range_bin", "range_bin": str(bin_index)} for bin_index in range(5))
                rows.extend(
                    {"group": "screen_bin", "screen_u_bin": str(u_bin), "screen_v_bin": str(v_bin)}
                    for u_bin in range(3)
                    for v_bin in range(3)
                )
                for row in rows:
                    full_row = {
                        "bootstrap_pass_fraction": "1.0",
                        "right_p025_m": "0",
                        "down_p025_m": "0.01",
                        "forward_p025_m": "0.1",
                        "trial": "",
                        "range_bin": "",
                        "screen_u_bin": "",
                        "screen_v_bin": "",
                    }
                    full_row.update(row)
                    writer.writerow(full_row)

            command = [
                sys.executable,
                str(ROOT_DIR / "scripts" / "roi_axis_gain_stability_gate.py"),
                "--aggregate-summary",
                str(aggregate),
                "--skipped-inputs",
                str(skipped),
                "--candidate-mode",
                "top_center",
                "--right-gain",
                "0",
                "--down-gain",
                "0.75",
                "--forward-gain",
                "0.75",
                "--candidate-sample-diagnostics",
                str(samples),
                "--bootstrap-summary",
                str(bootstrap),
                "--output-dir",
                str(output),
            ]
            completed = subprocess.run(command, cwd=ROOT_DIR, text=True, capture_output=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            summary = json.loads((output / "stability_gate_summary.json").read_text())
            self.assertEqual(summary["status"], "pass")

    def test_guarded_roi_range_fusion_uses_noop_guards(self) -> None:
        module = load_script_module("roi_down_range_forward_fusion", "scripts/roi_down_range_forward_fusion.py")
        sample = SimpleNamespace(raw=(1.0, 2.0, 100.0))
        args = SimpleNamespace(
            disable_down_screen_bins="0:1",
            forward_min_range_m=500.0,
            forward_max_range_m=None,
            forward_min_correction_abs_m=None,
        )
        corrected, forward_fallback = module.corrected_point(
            sample,
            triangulated=(3.0, 10.0, 200.0),
            forward_source=120.0,
            raw_range_m=400.0,
            screen_key=(0, 1),
            down_gain=0.5,
            forward_gain=1.0,
            args=args,
        )
        self.assertEqual(corrected, (1.0, 2.0, 100.0))
        self.assertEqual(forward_fallback, "range_guarded_noop")

        corrected, forward_fallback = module.corrected_point(
            sample,
            triangulated=(3.0, 10.0, 200.0),
            forward_source=120.0,
            raw_range_m=600.0,
            screen_key=(1, 1),
            down_gain=0.5,
            forward_gain=1.0,
            args=args,
        )
        self.assertEqual(corrected, (1.0, 6.0, 120.0))
        self.assertEqual(forward_fallback, "none")

        args.forward_min_correction_abs_m = 0.5
        corrected, forward_fallback = module.corrected_point(
            sample,
            triangulated=(3.0, 10.0, 200.0),
            forward_source=100.25,
            raw_range_m=600.0,
            screen_key=(1, 1),
            down_gain=0.5,
            forward_gain=1.0,
            args=args,
        )
        self.assertEqual(corrected, (1.0, 6.0, 100.0))
        self.assertEqual(forward_fallback, "tiny_correction_guarded_noop")

    def test_guard_policy_sweep_applies_runtime_guards(self) -> None:
        module = load_script_module("roi_guard_policy_sweep", "scripts/roi_guard_policy_sweep.py")
        rows = [
            {
                "fallback": "none",
                "forward_fallback": "none",
                "raw_range_m": "400",
                "screen_u_bin": "0",
                "screen_v_bin": "1",
                "raw_error_right_m": "0",
                "raw_error_down_m": "10",
                "raw_error_forward_m": "20",
                "raw_error_position_m": "22.360679775",
                "corrected_error_right_m": "0",
                "corrected_error_down_m": "2",
                "corrected_error_forward_m": "5",
                "corrected_error_position_m": "5.385164807",
                "trial": "2",
                "range_bin": "0",
            }
        ]
        guarded = module.apply_guard(rows, forward_min_range=500.0, disabled_screen_bins=((0, 1),))
        self.assertEqual(guarded[0]["forward_fallback"], "range_guarded_noop")
        self.assertEqual(guarded[0]["down_guarded_noop"], "yes")
        self.assertEqual(guarded[0]["corrected_error_down_m"], "10")
        self.assertEqual(guarded[0]["corrected_error_forward_m"], "20")

    def test_validation_gate_aggregate_normalizes_seeded_reports(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            output = root / "aggregate"
            for seed, min_right in (("20260625", "0.1"), ("20260626", "0.2")):
                report = root / f"bbox_anchor_global_right_att_0p1_seed{seed}"
                report.mkdir()
                with (report / "validation_gate_summary.csv").open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.DictWriter(
                        handle,
                        fieldnames=[
                            "trial",
                            "sample_count",
                            "evaluated_count",
                            "right_rmse_improvement_m",
                            "down_rmse_improvement_m",
                            "forward_rmse_improvement_m",
                            "position_rmse_improvement_m",
                            "pass",
                        ],
                    )
                    writer.writeheader()
                    writer.writerow(
                        {
                            "trial": "2",
                            "sample_count": "10",
                            "evaluated_count": "10",
                            "right_rmse_improvement_m": min_right,
                            "down_rmse_improvement_m": "0.3",
                            "forward_rmse_improvement_m": "4.0",
                            "position_rmse_improvement_m": "4.1",
                            "pass": "yes",
                        }
                    )
            size_report = root / "bbox_size_range_right_att_0p1_seed20260625"
            size_report.mkdir()
            with (size_report / "validation_gate_summary.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "trial",
                        "size_right_rmse_improvement_m",
                        "size_down_rmse_improvement_m",
                        "size_forward_rmse_improvement_m",
                        "size_position_rmse_improvement_m",
                        "blended_right_rmse_improvement_m",
                        "blended_down_rmse_improvement_m",
                        "blended_forward_rmse_improvement_m",
                        "blended_position_rmse_improvement_m",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "trial": "2",
                        "size_right_rmse_improvement_m": "-1.0",
                        "size_down_rmse_improvement_m": "0.0",
                        "size_forward_rmse_improvement_m": "1.0",
                        "size_position_rmse_improvement_m": "1.0",
                        "blended_right_rmse_improvement_m": "0.0",
                        "blended_down_rmse_improvement_m": "0.1",
                        "blended_forward_rmse_improvement_m": "0.2",
                        "blended_position_rmse_improvement_m": "0.3",
                    }
                )
            command = [
                sys.executable,
                str(ROOT_DIR / "scripts" / "validation_gate_aggregate.py"),
                str(root),
                "--output-dir",
                str(output),
                "--min-reports",
                "2",
            ]
            completed = subprocess.run(command, cwd=ROOT_DIR, text=True, capture_output=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            summary_rows = list(csv.DictReader((output / "validation_gate_method_summary.csv").open()))
            by_key = {row["method_key"]: row for row in summary_rows}
            key = "bbox_anchor_global_right_att_0p1_seed*"
            self.assertIn(key, by_key)
            self.assertEqual(by_key[key]["report_count"], "2")
            self.assertEqual(by_key[key]["pass_all_with_min_reports"], "yes")
            self.assertEqual(by_key[key]["min_right_rmse_improvement_m"], "0.100000000")
            self.assertIn("bbox_size_range_right_att_0p1_seed*::size", by_key)
            self.assertIn("bbox_size_range_right_att_0p1_seed*::blended", by_key)

    def test_validation_gate_aggregate_tracks_strict_rdf_all_axis_pass(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            output = root / "aggregate"
            for seed, right_delta in (("20260601", "0.010"), ("20260602", "0.000")):
                report = root / f"strict_gate_method_seed{seed}"
                report.mkdir()
                with (report / "validation_gate_summary.csv").open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.DictWriter(
                        handle,
                        fieldnames=[
                            "trial",
                            "sample_count",
                            "right_rmse_improvement_m",
                            "down_rmse_improvement_m",
                            "forward_rmse_improvement_m",
                            "position_rmse_improvement_m",
                        ],
                    )
                    writer.writeheader()
                    writer.writerow(
                        {
                            "trial": "2",
                            "sample_count": "4",
                            "right_rmse_improvement_m": right_delta,
                            "down_rmse_improvement_m": "0.020",
                            "forward_rmse_improvement_m": "2.000",
                            "position_rmse_improvement_m": "1.900",
                        }
                    )

            command = [
                sys.executable,
                str(ROOT_DIR / "scripts" / "validation_gate_aggregate.py"),
                str(root),
                "--output-dir",
                str(output),
            ]
            completed = subprocess.run(command, cwd=ROOT_DIR, text=True, capture_output=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            with (output / "validation_gate_trial_rows.csv").open(newline="", encoding="utf-8") as handle:
                trial_rows = list(csv.DictReader(handle))
            with (output / "validation_gate_method_summary.csv").open(newline="", encoding="utf-8") as handle:
                summary_rows = list(csv.DictReader(handle))
            self.assertEqual([row["pass_rdf_nonregression"] for row in trial_rows], ["yes", "yes"])
            self.assertEqual([row["pass_strict_rdf_all_axis"] for row in trial_rows], ["yes", "no"])
            self.assertEqual(len(summary_rows), 1)
            self.assertEqual(summary_rows[0]["pass_all_trials"], "yes")
            self.assertEqual(summary_rows[0]["strict_pass_trial_count"], "1")
            self.assertEqual(summary_rows[0]["strict_pass_all_trials"], "no")

    def test_bbox_anchor_axis_guard_uses_raw_range_noop(self) -> None:
        module = load_script_module("bbox_anchor_axis_guard_sweep", "scripts/bbox_anchor_axis_guard_sweep.py")
        sample = {
            "raw_forward_m": 700.0,
            "raw_error_right": 10.0,
            "anchor_error_right": 1.0,
            "raw_error_down": 20.0,
            "anchor_error_down": 2.0,
            "raw_error_forward": 30.0,
            "anchor_error_forward": 3.0,
        }
        corrected = module.corrected_errors(
            sample,
            right_gain=1.0,
            down_gain=1.0,
            forward_gain=1.0,
            right_min_range_m=1100.0,
            down_min_range_m=500.0,
            forward_min_range_m=None,
        )
        self.assertEqual(corrected["corrected_error_right"], 10.0)
        self.assertEqual(corrected["right_fallback"], "range_guarded_noop")
        self.assertEqual(corrected["corrected_error_down"], 2.0)
        self.assertEqual(corrected["down_fallback"], "none")
        self.assertEqual(corrected["corrected_error_forward"], 3.0)

    def test_bbox_anchor_cell_guard_cross_validation_uses_holdout_policy(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            output = root / "cell_guard"
            fieldnames = [
                "trial_t_s",
                "truth_right_m",
                "truth_down_m",
                "truth_forward_m",
                "raw_right_m",
                "raw_down_m",
                "raw_forward_m",
                "anchor_left_u_px",
                "anchor_left_v_px",
                "anchor_right_u_px",
                "anchor_right_v_px",
                "anchor_cell",
                "anchor_right_m",
                "anchor_down_m",
                "anchor_forward_m",
                "center_rot_right_m",
                "center_rot_down_m",
                "center_rot_forward_m",
                "raw_error_right_m",
                "raw_error_down_m",
                "raw_error_forward_m",
                "anchor_error_right_m",
                "anchor_error_down_m",
                "anchor_error_forward_m",
            ]
            for report_name in ("report_a", "report_b"):
                report = root / report_name
                report.mkdir()
                with (report / "validation_trial_02_anchor_comparison.csv").open(
                    "w", newline="", encoding="utf-8"
                ) as handle:
                    writer = csv.DictWriter(handle, fieldnames=fieldnames)
                    writer.writeheader()
                    base = {
                        "trial_t_s": "0",
                        "truth_right_m": "0",
                        "truth_down_m": "0",
                        "truth_forward_m": "1000",
                        "raw_right_m": "0",
                        "raw_down_m": "0",
                        "raw_forward_m": "1000",
                        "anchor_left_u_px": "640",
                        "anchor_left_v_px": "360",
                        "anchor_right_u_px": "620",
                        "anchor_right_v_px": "360",
                        "anchor_right_m": "0",
                        "anchor_down_m": "0",
                        "anchor_forward_m": "1000",
                        "center_rot_right_m": "0",
                        "center_rot_down_m": "0",
                        "center_rot_forward_m": "1000",
                        "raw_error_right_m": "0",
                        "raw_error_forward_m": "0",
                        "anchor_error_right_m": "0",
                        "anchor_error_forward_m": "0",
                    }
                    good = dict(base)
                    good.update(
                        {
                            "anchor_cell": "good_cell",
                            "raw_error_down_m": "2",
                            "anchor_error_down_m": "1",
                        }
                    )
                    bad = dict(base)
                    bad.update(
                        {
                            "anchor_cell": "bad_cell",
                            "raw_error_down_m": "1",
                            "anchor_error_down_m": "2",
                        }
                    )
                    writer.writerow(good)
                    writer.writerow(bad)

            command = [
                sys.executable,
                str(ROOT_DIR / "scripts" / "bbox_anchor_cell_guard_cross_validation.py"),
                str(root / "report_a"),
                str(root / "report_b"),
                "--output-dir",
                str(output),
                "--validation-trials",
                "2",
                "--feature-sets",
                "anchor_cell",
                "--right-gain",
                "0",
                "--forward-gain",
                "0",
                "--down-gains",
                "1",
                "--train-min-samples",
                "1",
                "--train-min-down-improvements-m",
                "0",
            ]
            completed = subprocess.run(command, cwd=ROOT_DIR, text=True, capture_output=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            summary = list(csv.DictReader((output / "cell_guard_candidate_summary.csv").open()))
            self.assertEqual(len(summary), 1)
            self.assertEqual(summary[0]["pass_all_nonregression"], "yes")
            self.assertEqual(summary[0]["down_applied_count"], "2")
            self.assertGreater(float(summary[0]["min_down_rmse_improvement_m"]), 0.5)
            sample_rows = list(csv.DictReader((output / "candidate_sample_diagnostics.csv").open()))
            by_cell = {row["anchor_cell"]: row["down_fallback"] for row in sample_rows}
            self.assertEqual(by_cell["good_cell"], "none")
            self.assertEqual(by_cell["bad_cell"], "cell_guarded_noop")

            report_trial_output = root / "cell_guard_report_trial"
            report_trial_command = [
                sys.executable,
                str(ROOT_DIR / "scripts" / "bbox_anchor_cell_guard_cross_validation.py"),
                str(root / "report_a"),
                str(root / "report_b"),
                "--output-dir",
                str(report_trial_output),
                "--holdout-mode",
                "report_trial",
                "--validation-trials",
                "2",
                "--feature-sets",
                "anchor_cell",
                "--right-gain",
                "0",
                "--forward-gain",
                "0",
                "--down-gains",
                "1",
                "--train-min-samples",
                "1",
                "--train-min-down-improvements-m",
                "0",
            ]
            completed = subprocess.run(
                report_trial_command, cwd=ROOT_DIR, text=True, capture_output=True, check=False
            )
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            summary = list(csv.DictReader((report_trial_output / "cell_guard_candidate_summary.csv").open()))
            self.assertEqual(len(summary), 1)
            self.assertEqual(summary[0]["holdout_mode"], "report_trial")
            self.assertEqual(summary[0]["pass_all_nonregression"], "yes")
            self.assertEqual(summary[0]["down_applied_count"], "0")
            self.assertEqual(float(summary[0]["min_down_rmse_improvement_m"]), 0.0)
            sample_rows = list(csv.DictReader((report_trial_output / "candidate_sample_diagnostics.csv").open()))
            by_cell = {row["anchor_cell"]: row["down_fallback"] for row in sample_rows}
            self.assertEqual(by_cell["good_cell"], "cell_guarded_noop")
            self.assertEqual(by_cell["bad_cell"], "cell_guarded_noop")
            metadata = json.loads((report_trial_output / "metadata.json").read_text())
            self.assertEqual(metadata["holdout_mode"], "report_trial")


if __name__ == "__main__":
    unittest.main()
