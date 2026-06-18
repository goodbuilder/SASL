from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "run_common_csv_frontier_profile_matrix_under_test",
        ROOT_DIR / "scripts" / "run_common_csv_frontier_profile_matrix.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load run_common_csv_frontier_profile_matrix.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_sim3_profile(path: Path, *, method: str, scale: float) -> None:
    path.write_text(
        json.dumps(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": method,
                "method": method,
                "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "runtime_inputs": ["right_m", "down_m", "forward_m", "range_m", "disparity_px"],
                "forbidden_runtime_inputs": ["truth_right_m", "truth_down_m", "truth_forward_m"],
                "scale": scale,
                "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "translation_rdf_m": [0.0, 0.0, 0.0],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def write_run(root: Path, scenario: str, seed: str) -> None:
    run_dir = root / scenario / seed
    profiles = run_dir / "runtime_profiles"
    profiles.mkdir(parents=True)
    (run_dir / "calibration_samples.csv").write_text(
        (
            "sample_id,split,truth_right_m,truth_down_m,truth_forward_m,truth_range_m,"
            "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px\n"
            "0,validation,1,2,100,100.025,2,4,200,200.04,100\n"
            "1,validation,2,1,200,200.012,4,2,400,400.025,50\n"
        ),
        encoding="utf-8",
    )
    write_sim3_profile(profiles / "half_scale.json", method="half_scale", scale=0.5)


class RunCommonCsvFrontierProfileMatrixTests(unittest.TestCase):
    def test_aggregates_frontier_replay_across_seed_dirs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_root = root / "sweep"
            write_run(sweep_root, "scenario_a", "seed_1")
            write_run(sweep_root, "scenario_b", "seed_2")
            active = root / "active.json"
            write_sim3_profile(active, method="half_scale", scale=0.5)
            frontier = root / "frontier.csv"
            frontier.write_text(
                (
                    "rank,method,strict_all_scenarios,validation_range_rmse_mean_m,"
                    "target_abs_mean_error_max_m,rdf_axis_nonregression_pass\n"
                    "1,half_scale,True,0.5,0.1,True\n"
                ),
                encoding="utf-8",
            )

            summary = module.run_frontier_profile_matrix(
                frontier_csv=frontier,
                sweep_root=sweep_root,
                output_dir=root / "out",
                active_profile_json=active,
                active_label="active_same_method",
                top_n=1,
                split="validation",
                target_range_m=100.0,
                target_window_m=5.0,
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["selected_method"], "half_scale")
            self.assertEqual(summary["run_count"], 2)
            self.assertFalse(summary["runtime_truth_used"])
            methods = {row["method"]: row for row in summary["methods"]}
            self.assertEqual(methods["half_scale"]["pass_count"], 2)
            self.assertEqual(methods["half_scale"]["evaluated_run_count"], 2)
            self.assertTrue(methods["half_scale"]["pass_all_runs"])
            self.assertTrue((root / "out" / "frontier_profile_matrix_runs.csv").exists())
            self.assertTrue((root / "out" / "frontier_profile_matrix_methods.csv").exists())
            self.assertTrue((root / "out" / "frontier_profile_matrix_summary.json").exists())
            self.assertTrue((root / "out" / "frontier_profile_matrix_summary.txt").exists())
            self.assertTrue((root / "out" / "frontier_profile_matrix_pass_counts.png").exists())
            self.assertTrue((root / "out" / "frontier_profile_matrix_rmse.png").exists())
            self.assertTrue((root / "out" / "frontier_profile_matrix_target_errors.png").exists())
            self.assertTrue((root / "out" / "runs" / "scenario_a" / "seed_1" / "common_csv_profile_comparison.csv").exists())
            txt = (root / "out" / "frontier_profile_matrix_summary.txt").read_text(encoding="utf-8")
            self.assertIn("stereo-center RDF", txt)
            self.assertIn("runtime_truth_used: False", txt)

    def test_uses_fixed_runtime_profiles_dir_for_all_runs_when_provided(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_root = root / "sweep"
            write_run(sweep_root, "scenario_a", "seed_1")
            write_run(sweep_root, "scenario_b", "seed_2")
            fixed_profiles = root / "pooled_profiles"
            fixed_profiles.mkdir()
            write_sim3_profile(fixed_profiles / "half_scale.json", method="half_scale", scale=0.5)
            frontier = root / "frontier.csv"
            frontier.write_text(
                (
                    "rank,method,strict_all_scenarios,validation_range_rmse_mean_m,"
                    "target_abs_mean_error_max_m,rdf_axis_nonregression_pass\n"
                    "1,half_scale,True,0.5,0.1,True\n"
                ),
                encoding="utf-8",
            )

            summary = module.run_frontier_profile_matrix(
                frontier_csv=frontier,
                sweep_root=sweep_root,
                output_dir=root / "out_fixed",
                runtime_profiles_dir=fixed_profiles,
                top_n=1,
                split="validation",
                target_range_m=100.0,
                target_window_m=5.0,
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["runtime_profiles_dir"], str(fixed_profiles))
            for row in summary["runs"]:
                self.assertEqual(row["runtime_profiles_dir"], str(fixed_profiles))


if __name__ == "__main__":
    unittest.main()
