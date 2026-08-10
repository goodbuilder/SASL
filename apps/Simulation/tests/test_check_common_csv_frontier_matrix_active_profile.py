from __future__ import annotations

import importlib.util
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "check_common_csv_frontier_matrix_active_profile_under_test",
        ROOT_DIR / "scripts" / "check_common_csv_frontier_matrix_active_profile.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_common_csv_frontier_matrix_active_profile.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_profile(path: Path, *, method: str, runtime_inputs: list[str] | None = None) -> None:
    path.write_text(
        json.dumps(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": method,
                "method": method,
                "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "runtime_inputs": runtime_inputs or ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth", "true_disparity"],
                "scale": 1.0,
                "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "translation_rdf_m": [0.0, 0.0, 0.0],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def write_matrix(path: Path, *, selected_method: str = "good_method") -> None:
    path.write_text(
        json.dumps(
            {
                "status": "PASS",
                "selected_method": selected_method,
                "methods": [
                    {
                        "method": "good_method",
                        "evaluated_run_count": 2,
                        "expected_run_count": 2,
                        "pass_count": 2,
                        "pass_all_runs": True,
                        "selected_count": 2,
                        "validation_range_rmse_corrected_mean_m": 1.0,
                        "validation_range_rmse_corrected_max_m": 1.2,
                        "target_max_abs_error_corrected_max_m": 2.0,
                        "common_csv_score_mean": 3.0,
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def write_matrix_with_run_output(root: Path, active: Path, *, active_pass: bool) -> Path:
    run_out = root / "run_out"
    run_out.mkdir()
    comparison = run_out / "common_csv_profile_comparison.csv"
    with comparison.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "candidate",
                "profile_json",
                "method",
                "pass_common_replay",
                "rdf_axis_nonregression_pass",
                "target_max_error_pass",
                "validation_range_rmse_corrected_m",
                "target_max_abs_error_corrected_m",
                "common_csv_score",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "candidate": "current_active",
                "profile_json": str(active),
                "method": "good_method",
                "pass_common_replay": str(active_pass),
                "rdf_axis_nonregression_pass": str(active_pass),
                "target_max_error_pass": "True",
                "validation_range_rmse_corrected_m": "1.0",
                "target_max_abs_error_corrected_m": "2.0",
                "common_csv_score": "3.0",
            }
        )
    matrix = root / "matrix_with_runs.json"
    matrix.write_text(
        json.dumps(
            {
                "status": "PASS",
                "selected_method": "good_method",
                "run_count": 1,
                "runs": [{"scenario": "s", "seed": "seed_1", "output_dir": str(run_out)}],
                "methods": [
                    {
                        "method": "good_method",
                        "evaluated_run_count": 1,
                        "expected_run_count": 1,
                        "pass_count": 1,
                        "pass_all_runs": True,
                        "selected_count": 1,
                        "validation_range_rmse_corrected_mean_m": 1.0,
                        "validation_range_rmse_corrected_max_m": 1.0,
                        "target_max_abs_error_corrected_max_m": 2.0,
                        "common_csv_score_mean": 3.0,
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return matrix


class CheckCommonCsvFrontierMatrixActiveProfileTests(unittest.TestCase):
    def test_passes_when_active_method_matches_matrix_recommendation(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            matrix = root / "matrix.json"
            active = root / "active.json"
            write_matrix(matrix)
            write_profile(active, method="good_method")

            result = module.check_active_profile_against_matrix(
                matrix_summary_json=matrix,
                active_profile_json=active,
                output_dir=root / "out",
            )

            self.assertEqual(result["status"], "PASS")
            self.assertEqual(result["recommendation"], "keep_active_profile")
            self.assertTrue(result["active_matches_matrix_selected_method"])
            self.assertTrue(result["active_profile_runtime_truth_free"])
            self.assertFalse(result["runtime_truth_used"])
            self.assertTrue((root / "out" / "active_profile_matrix_gate.csv").exists())
            self.assertTrue((root / "out" / "active_profile_matrix_gate.json").exists())
            self.assertTrue((root / "out" / "active_profile_matrix_gate.txt").exists())
            txt = (root / "out" / "active_profile_matrix_gate.txt").read_text(encoding="utf-8")
            self.assertIn("stereo-center RDF", txt)
            self.assertIn("runtime_truth_used: False", txt)

    def test_fails_when_active_method_differs_from_matrix_recommendation(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            matrix = root / "matrix.json"
            active = root / "active.json"
            write_matrix(matrix)
            write_profile(active, method="other_method")

            result = module.check_active_profile_against_matrix(
                matrix_summary_json=matrix,
                active_profile_json=active,
                output_dir=root / "out",
            )

            self.assertEqual(result["status"], "FAIL")
            self.assertFalse(result["active_matches_matrix_selected_method"])

    def test_fails_when_method_aggregate_passes_but_actual_active_profile_row_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            active = root / "active.json"
            write_profile(active, method="good_method")
            matrix = write_matrix_with_run_output(root, active, active_pass=False)

            result = module.check_active_profile_against_matrix(
                matrix_summary_json=matrix,
                active_profile_json=active,
                output_dir=root / "out",
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(result["active_matches_matrix_selected_method"])
        self.assertTrue(result["active_method_pass_all_runs"])
        self.assertTrue(result["active_profile_candidate_rows_found"])
        self.assertFalse(result["active_profile_candidate_pass_all_runs"])
        self.assertEqual(result["active_profile_candidate_pass_count"], 0)


if __name__ == "__main__":
    unittest.main()
