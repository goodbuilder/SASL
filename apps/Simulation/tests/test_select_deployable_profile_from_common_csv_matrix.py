from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "select_deployable_profile_from_common_csv_matrix.py"


def load_module():
    spec = importlib.util.spec_from_file_location("select_deployable_profile_from_common_csv_matrix_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
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


def method_row(
    method: str,
    *,
    rmse_mean: float,
    rmse_max: float,
    target_max: float,
    score: float,
    pass_count: int = 2,
    axis_pass_count: int = 2,
    target_pass_count: int = 2,
) -> dict[str, object]:
    return {
        "method": method,
        "evaluated_run_count": 2,
        "expected_run_count": 2,
        "pass_count": pass_count,
        "pass_all_runs": pass_count == 2,
        "axis_pass_count": axis_pass_count,
        "target_pass_count": target_pass_count,
        "validation_range_rmse_corrected_mean_m": rmse_mean,
        "validation_range_rmse_corrected_max_m": rmse_max,
        "validation_position_rmse_corrected_mean_m": rmse_mean + 1.0,
        "validation_position_rmse_corrected_max_m": rmse_max + 1.0,
        "target_max_abs_error_corrected_max_m": target_max,
        "common_csv_score_mean": score,
        "common_csv_score_max": score + 1.0,
    }


def write_matrix(path: Path, profiles: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "status": "PASS",
                "selected_method": "target_best",
                "runtime_profiles_dir": str(profiles),
                "run_count": 2,
                "methods": [
                    method_row("range_best", rmse_mean=1.0, rmse_max=1.4, target_max=3.0, score=5.0),
                    method_row("target_best", rmse_mean=1.2, rmse_max=1.5, target_max=1.0, score=4.0),
                    method_row("axis_fail", rmse_mean=0.9, rmse_max=1.0, target_max=1.0, score=3.0, axis_pass_count=1),
                    method_row("target_fail", rmse_mean=0.8, rmse_max=0.9, target_max=12.0, score=2.0),
                    method_row("truth_runtime", rmse_mean=0.7, rmse_max=0.8, target_max=1.0, score=1.0),
                    method_row("missing_profile", rmse_mean=0.6, rmse_max=0.7, target_max=1.0, score=0.5),
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


class SelectDeployableProfileFromCommonCsvMatrixTests(unittest.TestCase):
    def test_selects_policy_winners_after_deployable_hard_gates(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profiles = root / "profiles"
            profiles.mkdir()
            for method in ("range_best", "target_best", "axis_fail", "target_fail"):
                write_profile(profiles / f"{method}.json", method=method)
            write_profile(
                profiles / "truth_runtime.json",
                method="truth_runtime",
                runtime_inputs=["raw_right_m", "px4_rtk_truth"],
            )
            matrix = root / "matrix.json"
            write_matrix(matrix, profiles)

            summary = module.select_profiles(
                matrix_summary_json=matrix,
                runtime_profiles_dir=None,
                target_error_limit_m=10.0,
                recommended_policy="range_priority",
            )
            module.write_outputs(root / "out", summary)
            policy_csv_exists = (root / "out" / "deployable_profile_policy_selection.csv").exists()
            candidates_csv_exists = (root / "out" / "deployable_profile_candidates.csv").exists()
            txt = (root / "out" / "deployable_profile_policy_selection.txt").read_text(encoding="utf-8")

        policies = {row["policy"]: row for row in summary["policies"]}
        candidates = {row["method"]: row for row in summary["candidates"]}
        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["recommended_method"], "range_best")
        self.assertEqual(policies["range_priority"]["selected_method"], "range_best")
        self.assertEqual(policies["target_priority"]["selected_method"], "target_best")
        self.assertTrue(candidates["range_best"]["hard_gate_pass"])
        self.assertTrue(candidates["target_best"]["hard_gate_pass"])
        self.assertFalse(candidates["axis_fail"]["hard_gate_pass"])
        self.assertFalse(candidates["target_fail"]["hard_gate_pass"])
        self.assertFalse(candidates["truth_runtime"]["hard_gate_pass"])
        self.assertFalse(candidates["missing_profile"]["hard_gate_pass"])
        self.assertFalse(summary["runtime_truth_used"])
        self.assertTrue(policy_csv_exists)
        self.assertTrue(candidates_csv_exists)
        self.assertIn("stereo-center RDF", txt)
        self.assertIn("runtime_truth_used: False", txt)


if __name__ == "__main__":
    unittest.main()
