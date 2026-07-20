from __future__ import annotations

import csv
import importlib.util
import json
import subprocess
import sys
import tempfile
from pathlib import Path
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


class ValidationAggregationToolTests(unittest.TestCase):
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
