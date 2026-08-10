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
        "compare_stereodistance_profiles_on_common_csv_under_test",
        ROOT_DIR / "scripts" / "compare_stereodistance_profiles_on_common_csv.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load compare_stereodistance_profiles_on_common_csv.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_sim3_profile(path: Path, *, profile_id: str, scale: float) -> None:
    path.write_text(
        json.dumps(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": profile_id,
                "method": profile_id,
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


class CompareStereoDistanceProfilesOnCommonCsvTests(unittest.TestCase):
    def test_selects_profile_on_same_csv_and_writes_artifacts(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
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
            identity = root / "identity.json"
            half = root / "half.json"
            write_sim3_profile(identity, profile_id="identity", scale=1.0)
            write_sim3_profile(half, profile_id="half_scale", scale=0.5)

            summary = module.compare_profiles_on_common_csv(
                input_csv=sample_csv,
                profile_specs=[f"identity:{identity}", f"half:{half}"],
                output_dir=root / "out",
                split="validation",
                target_range_m=100.0,
                target_window_m=5.0,
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["selected_candidate"], "half")
            self.assertEqual(summary["passing_candidate_count"], 1)
            self.assertFalse(summary["runtime_truth_used"])
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)
            rows = {row["candidate"]: row for row in summary["rows"]}
            self.assertTrue(rows["half"]["pass_common_replay"])
            self.assertTrue(rows["half"]["rdf_axis_nonregression_pass"])
            self.assertTrue(rows["half"]["range_rmse_improvement_pass"])
            self.assertFalse(rows["identity"]["pass_common_replay"])
            self.assertTrue((root / "out" / "common_csv_profile_comparison.csv").exists())
            self.assertTrue((root / "out" / "common_csv_profile_predictions.csv").exists())
            self.assertTrue((root / "out" / "common_csv_target_window_stats.csv").exists())
            self.assertTrue((root / "out" / "common_csv_target_window_stats.json").exists())
            self.assertTrue((root / "out" / "common_csv_target_window_stats.txt").exists())
            self.assertTrue((root / "out" / "common_csv_profile_comparison.txt").exists())
            self.assertTrue((root / "out" / "common_csv_profile_range_errors.png").exists())
            self.assertTrue((root / "out" / "common_csv_profile_axis_rmse.png").exists())
            self.assertTrue((root / "out" / "common_csv_residual_disparity_by_range.png").exists())
            target_stats = json.loads(
                (root / "out" / "common_csv_target_window_stats.json").read_text(encoding="utf-8")
            )
            self.assertEqual(target_stats["target_range_m"], 100.0)
            self.assertFalse(target_stats["runtime_truth_used"])
            self.assertEqual(target_stats["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertEqual(len(target_stats["rows"]), 2)
            target_rows = {row["candidate"]: row for row in target_stats["rows"]}
            self.assertFalse(target_rows["half"]["runtime_truth_used"])
            self.assertEqual(target_rows["half"]["target_window_count"], 1)
            self.assertAlmostEqual(target_rows["half"]["corrected_abs_mean_error_m"], 0.0, places=9)
            txt = (root / "out" / "common_csv_profile_comparison.txt").read_text(encoding="utf-8")
            self.assertIn("stereo-center RDF", txt)
            self.assertIn("FRD+range", txt)
            self.assertIn("axis_pass", txt)
            target_txt = (root / "out" / "common_csv_target_window_stats.txt").read_text(encoding="utf-8")
            self.assertIn("1852m target-window", target_txt)
            self.assertIn("runtime_truth_used: False", target_txt)


if __name__ == "__main__":
    unittest.main()
