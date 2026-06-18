from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "rank_sweep_method_comparison_candidates.py"


def load_module():
    spec = importlib.util.spec_from_file_location("rank_sweep_method_comparison_candidates_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


FIELDNAMES = [
    "method",
    "registration_model",
    "residual_model",
    "regularization_model",
    "runtime_profile_family",
    "selection_note",
    "rdf_axis_contract",
    "model_complexity",
    "validation_range_rmse_m",
    "validation_right_rmse_m",
    "validation_down_rmse_m",
    "validation_forward_rmse_m",
    "validation_position_rmse_m",
    "validation_target_window_count",
    "validation_target_abs_mean_error_m",
    "validation_target_rmse_m",
    "validation_target_max_abs_error_m",
]


def write_method_comparison(path: Path, *, good_rmse: float, bad_axis_rmse: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [
        {
            "method": "raw",
            "registration_model": "none_raw_stereo_rdf",
            "residual_model": "none",
            "regularization_model": "none",
            "runtime_profile_family": "none_raw",
            "selection_note": "diagnostic candidate",
            "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "model_complexity": 0,
            "validation_range_rmse_m": 3.0,
            "validation_right_rmse_m": 10.0,
            "validation_down_rmse_m": 5.0,
            "validation_forward_rmse_m": 3.0,
            "validation_position_rmse_m": 12.0,
            "validation_target_window_count": 2,
            "validation_target_abs_mean_error_m": 5.0,
            "validation_target_rmse_m": 5.5,
            "validation_target_max_abs_error_m": 8.0,
        },
        {
            "method": "good_method",
            "registration_model": "sim3_estimated_to_truth",
            "residual_model": "none",
            "regularization_model": "none",
            "runtime_profile_family": "estimated_to_truth_sim3",
            "selection_note": "similarity registration baseline",
            "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "model_complexity": 7,
            "validation_range_rmse_m": good_rmse,
            "validation_right_rmse_m": 1.0,
            "validation_down_rmse_m": 1.0,
            "validation_forward_rmse_m": good_rmse,
            "validation_position_rmse_m": good_rmse + 0.1,
            "validation_target_window_count": 2,
            "validation_target_abs_mean_error_m": 1.0,
            "validation_target_rmse_m": 1.2,
            "validation_target_max_abs_error_m": 2.0,
        },
        {
            "method": "bad_axis_method",
            "registration_model": "sim3_estimated_to_truth",
            "residual_model": "none",
            "regularization_model": "none",
            "runtime_profile_family": "estimated_to_truth_sim3",
            "selection_note": "axis regression candidate",
            "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "model_complexity": 7,
            "validation_range_rmse_m": 1.0,
            "validation_right_rmse_m": bad_axis_rmse,
            "validation_down_rmse_m": 1.0,
            "validation_forward_rmse_m": 1.0,
            "validation_position_rmse_m": 4.0,
            "validation_target_window_count": 2,
            "validation_target_abs_mean_error_m": 1.0,
            "validation_target_rmse_m": 1.2,
            "validation_target_max_abs_error_m": 2.0,
        },
        {
            "method": "missing_profile_method",
            "registration_model": "sim3_estimated_to_truth",
            "residual_model": "none",
            "regularization_model": "none",
            "runtime_profile_family": "estimated_to_truth_sim3",
            "selection_note": "no runtime profile",
            "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "model_complexity": 7,
            "validation_range_rmse_m": 0.9,
            "validation_right_rmse_m": 1.0,
            "validation_down_rmse_m": 1.0,
            "validation_forward_rmse_m": 0.9,
            "validation_position_rmse_m": 1.1,
            "validation_target_window_count": 2,
            "validation_target_abs_mean_error_m": 1.0,
            "validation_target_rmse_m": 1.2,
            "validation_target_max_abs_error_m": 2.0,
        },
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class RankSweepMethodComparisonCandidatesTests(unittest.TestCase):
    def test_ranks_all_run_profile_backed_axis_clean_candidate(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            for seed, good_rmse in (("seed_1", 1.4), ("seed_2", 1.6)):
                run_dir = root / "scenario_a" / seed
                write_method_comparison(run_dir / "method_comparison.csv", good_rmse=good_rmse, bad_axis_rmse=11.0)
                profiles = run_dir / "runtime_profiles"
                profiles.mkdir()
                (profiles / "good_method.json").write_text("{}\n", encoding="utf-8")
                (profiles / "bad_axis_method.json").write_text("{}\n", encoding="utf-8")

            summary = module.aggregate_candidates(sweep_root=root, target_error_limit_m=10.0)
            module.write_outputs(root / "out", summary)

            methods = summary["methods"]
            output_exists = (root / "out" / "broad_method_candidate_rank.csv").exists()

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["selected_method"], "good_method")
        self.assertEqual(methods[0]["method"], "good_method")
        self.assertTrue(methods[0]["pass_all_runs"])
        bad_axis = next(row for row in methods if row["method"] == "bad_axis_method")
        missing_profile = next(row for row in methods if row["method"] == "missing_profile_method")
        self.assertFalse(bad_axis["pass_all_runs"])
        self.assertEqual(bad_axis["pass_count"], 0)
        self.assertFalse(missing_profile["pass_all_runs"])
        self.assertTrue(output_exists)


if __name__ == "__main__":
    unittest.main()
