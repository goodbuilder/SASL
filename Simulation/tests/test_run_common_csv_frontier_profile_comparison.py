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
        "run_common_csv_frontier_profile_comparison_under_test",
        ROOT_DIR / "scripts" / "run_common_csv_frontier_profile_comparison.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load run_common_csv_frontier_profile_comparison.py")
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


class RunCommonCsvFrontierProfileComparisonTests(unittest.TestCase):
    def test_selects_existing_frontier_profiles_and_writes_manifest(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profiles = root / "runtime_profiles"
            profiles.mkdir()
            sample_csv = root / "samples.csv"
            sample_csv.write_text(
                (
                    "sample_id,split,truth_right_m,truth_down_m,truth_forward_m,truth_range_m,"
                    "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,"
                    "true_disparity_px,raw_disparity_residual_px\n"
                    "0,validation,1,2,100,100.025,2,4,200,200.04,100,99.5,0.5\n"
                    "1,validation,2,1,200,200.012,4,2,400,400.025,50,49.8,0.2\n"
                ),
                encoding="utf-8",
            )
            active = root / "active.json"
            write_sim3_profile(active, method="active_identity", scale=1.0)
            write_sim3_profile(profiles / "half_scale.json", method="half_scale", scale=0.5)
            frontier = root / "frontier.csv"
            frontier.write_text(
                (
                    "rank,method,strict_all_scenarios,validation_range_rmse_mean_m,"
                    "target_abs_mean_error_max_m,rdf_axis_nonregression_pass\n"
                    "1,missing_method,True,1.0,1.0,True\n"
                    "2,half_scale,True,0.5,0.1,True\n"
                ),
                encoding="utf-8",
            )

            manifest = module.run_frontier_profile_comparison(
                frontier_csv=frontier,
                runtime_profiles_dir=profiles,
                input_csv=sample_csv,
                output_dir=root / "out",
                active_profile_json=active,
                active_label="active",
                top_n=2,
                split="validation",
                target_range_m=100.0,
                target_window_m=5.0,
            )

            self.assertEqual(manifest["selected_method"], "half_scale")
            self.assertEqual(manifest["candidate_count"], 2)
            self.assertEqual(manifest["passing_candidate_count"], 1)
            self.assertFalse(manifest["runtime_truth_used"])
            rows = {row["method"]: row for row in manifest["rows"] if row["method"]}
            self.assertFalse(rows["missing_method"]["profile_found"])
            self.assertFalse(rows["missing_method"]["selected_for_replay"])
            self.assertTrue(rows["half_scale"]["profile_found"])
            self.assertTrue(rows["half_scale"]["selected_for_replay"])
            self.assertTrue((root / "out" / "frontier_profile_candidate_manifest.csv").exists())
            self.assertTrue((root / "out" / "frontier_profile_candidate_manifest.json").exists())
            self.assertTrue((root / "out" / "frontier_profile_candidate_manifest.txt").exists())
            self.assertTrue((root / "out" / "common_csv_profile_comparison.csv").exists())
            self.assertTrue((root / "out" / "common_csv_target_window_stats.json").exists())
            txt = (root / "out" / "frontier_profile_candidate_manifest.txt").read_text(encoding="utf-8")
            self.assertIn("stereo-center RDF", txt)
            self.assertIn("runtime_truth_used: False", txt)

    def test_missing_active_profile_is_recorded_but_not_replayed(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profiles = root / "runtime_profiles"
            profiles.mkdir()
            sample_csv = root / "samples.csv"
            sample_csv.write_text(
                (
                    "sample_id,split,truth_right_m,truth_down_m,truth_forward_m,truth_range_m,"
                    "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px\n"
                    "0,validation,1,2,100,100.025,2,4,200,200.04,100\n"
                ),
                encoding="utf-8",
            )
            write_sim3_profile(profiles / "half_scale.json", method="half_scale", scale=0.5)
            frontier = root / "frontier.csv"
            frontier.write_text(
                (
                    "rank,method,strict_all_scenarios,validation_range_rmse_mean_m,"
                    "target_abs_mean_error_max_m,rdf_axis_nonregression_pass\n"
                    "1,half_scale,True,0.5,0.1,True\n"
                ),
                encoding="utf-8",
            )

            manifest = module.run_frontier_profile_comparison(
                frontier_csv=frontier,
                runtime_profiles_dir=profiles,
                input_csv=sample_csv,
                output_dir=root / "out",
                active_profile_json=root / "missing_active.json",
                active_label="missing_active",
                top_n=1,
                split="validation",
                target_range_m=100.0,
                target_window_m=5.0,
            )

            self.assertEqual(manifest["candidate_count"], 1)
            active_rows = [row for row in manifest["rows"] if row["source"] == "active_profile"]
            self.assertEqual(len(active_rows), 1)
            self.assertFalse(active_rows[0]["profile_found"])
            self.assertFalse(active_rows[0]["selected_for_replay"])


if __name__ == "__main__":
    unittest.main()
