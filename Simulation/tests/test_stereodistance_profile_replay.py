from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_replay_module():
    spec = importlib.util.spec_from_file_location(
        "stereodistance_profile_replay_under_test",
        ROOT_DIR / "scripts" / "stereodistance_profile_replay.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load stereodistance_profile_replay.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class StereoDistanceProfileReplayTests(unittest.TestCase):
    def test_replay_uses_named_frd_csv_fields_as_internal_rdf(self) -> None:
        module = load_replay_module()
        profile = module.EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-replay",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
            }
        )
        rows = [
            {
                "split": "validation",
                "truth_forward_m": "103.0",
                "truth_right_m": "11.0",
                "truth_down_m": "22.0",
                "estimated_forward_m": "100.0",
                "estimated_right_m": "10.0",
                "estimated_down_m": "20.0",
                "raw_disparity_px": "50.0",
            }
        ]

        predictions, summary = module.replay_rows(
            rows,
            profile,
            split_filter="validation",
            target_range_m=103.0,
            target_window_m=10.0,
        )

        self.assertEqual(len(predictions), 1)
        self.assertEqual(summary["skipped_row_count"], 0)
        self.assertAlmostEqual(float(predictions[0]["corrected_right_m"]), 11.0)
        self.assertAlmostEqual(float(predictions[0]["corrected_down_m"]), 22.0)
        self.assertAlmostEqual(float(predictions[0]["corrected_forward_m"]), 103.0)
        self.assertLess(
            summary["validation_range_rmse_corrected_m"],
            summary["validation_range_rmse_raw_m"],
        )
        self.assertIn("target_max_abs_error_corrected_m", summary)
        self.assertAlmostEqual(summary["target_max_abs_error_corrected_m"], 0.0, places=9)

    def test_replay_accepts_disparity_profile(self) -> None:
        module = load_replay_module()
        profile = module.EstimatedToTruthDisparityProfile.from_json_dict(
            {
                "mode": "estimated_to_truth_disparity_poly",
                "profile_id": "unit-disparity-replay",
                "d_corr_coefficients_high_to_low": [2.0, 0.0],
            }
        )
        rows = [
            {
                "split": "validation",
                "truth_forward_m": "50.0",
                "truth_right_m": "2.0",
                "truth_down_m": "2.5",
                "raw_forward_m": "100.0",
                "raw_right_m": "4.0",
                "raw_down_m": "5.0",
                "raw_disparity_px": "10.0",
            }
        ]

        predictions, summary = module.replay_rows(
            rows,
            profile,
            split_filter="validation",
            target_range_m=50.0,
            target_window_m=10.0,
        )

        self.assertEqual(len(predictions), 1)
        self.assertEqual(summary["skipped_row_count"], 0)
        self.assertAlmostEqual(float(predictions[0]["corrected_right_m"]), 2.0)
        self.assertAlmostEqual(float(predictions[0]["corrected_down_m"]), 2.5)
        self.assertAlmostEqual(float(predictions[0]["corrected_forward_m"]), 50.0)

    def test_replay_passes_left_bbox_center_to_sim3_profile(self) -> None:
        module = load_replay_module()
        profile = module.EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-left-disparity-replay",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "left_disparity_triangulation_model": {
                    "fx_px": 100.0,
                    "fy_px": 100.0,
                    "cx_px": 50.0,
                    "cy_px": 40.0,
                    "baseline_m": 10.0,
                    "disparity_correction_coefficients_high_to_low": [1.0, -2.0],
                },
            }
        )
        rows = [
            {
                "split": "validation",
                "truth_forward_m": "100.0",
                "truth_right_m": "10.0",
                "truth_down_m": "6.0",
                "raw_forward_m": "999.0",
                "raw_right_m": "999.0",
                "raw_down_m": "999.0",
                "raw_disparity_px": "12.0",
                "left_u_px": "65.0",
                "left_v_px": "46.0",
            }
        ]

        predictions, summary = module.replay_rows(
            rows,
            profile,
            split_filter="validation",
            target_range_m=100.7,
            target_window_m=10.0,
        )

        self.assertEqual(len(predictions), 1)
        self.assertEqual(summary["skipped_row_count"], 0)
        self.assertAlmostEqual(float(predictions[0]["corrected_right_m"]), 10.0)
        self.assertAlmostEqual(float(predictions[0]["corrected_down_m"]), 6.0)
        self.assertAlmostEqual(float(predictions[0]["corrected_forward_m"]), 100.0)

    def test_replay_accepts_affine_profile(self) -> None:
        module = load_replay_module()
        profile = module.EstimatedToTruthAffineProfile.from_json_dict(
            {
                "mode": "estimated_to_truth_affine_rdf",
                "profile_id": "unit-affine-replay",
                "affine_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
            }
        )
        rows = [
            {
                "split": "validation",
                "truth_forward_m": "103.0",
                "truth_right_m": "11.0",
                "truth_down_m": "22.0",
                "raw_forward_m": "100.0",
                "raw_right_m": "10.0",
                "raw_down_m": "20.0",
                "raw_disparity_px": "10.0",
            }
        ]

        predictions, summary = module.replay_rows(
            rows,
            profile,
            split_filter="validation",
            target_range_m=103.0,
            target_window_m=10.0,
        )

        self.assertEqual(len(predictions), 1)
        self.assertEqual(summary["profile_mode"], "estimated_to_truth_affine_rdf")
        self.assertAlmostEqual(float(predictions[0]["corrected_right_m"]), 11.0)
        self.assertAlmostEqual(float(predictions[0]["corrected_down_m"]), 22.0)
        self.assertAlmostEqual(float(predictions[0]["corrected_forward_m"]), 103.0)

    def test_replay_rejects_ned_only_truth_without_explicit_rdf(self) -> None:
        module = load_replay_module()
        profile = module.EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-ned-only-replay",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
            }
        )
        rows = [
            {
                "split": "validation",
                "truth_north_m": "100.0",
                "truth_east_m": "10.0",
                "truth_down_ned_m": "2.0",
                "raw_forward_m": "100.0",
                "raw_right_m": "10.0",
                "raw_down_m": "2.0",
                "raw_disparity_px": "10.0",
            }
        ]

        predictions, summary = module.replay_rows(
            rows,
            profile,
            split_filter="validation",
            target_range_m=100.0,
            target_window_m=10.0,
        )

        self.assertEqual(predictions, [])
        self.assertEqual(summary["evaluated_row_count"], 0)
        self.assertEqual(summary["skipped_row_count"], 1)
        self.assertFalse(summary["direct_ned_to_rdf_axis_alias_allowed"])
        self.assertIn("T_source_to_stereo_rdf", summary["truth_rdf_transform_contract"])

    def test_replay_main_writes_outputs(self) -> None:
        module = load_replay_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            input_csv = tmp / "input.csv"
            profile_json = tmp / "profile.json"
            output_dir = tmp / "out"
            input_csv.write_text(
                "split,truth_right_m,truth_down_m,truth_forward_m,"
                "raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px\n"
                "validation,1.0,2.0,13.0,1.0,2.0,10.0,40.0\n",
                encoding="utf-8",
            )
            profile_json.write_text(
                """{
  "mode": "estimated_to_truth_sim3",
  "profile_id": "unit-main",
  "scale": 1.0,
  "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
  "translation_rdf_m": [0.0, 0.0, 3.0]
}
""",
                encoding="utf-8",
            )
            old_argv = sys.argv
            try:
                sys.argv = [
                    "stereodistance_profile_replay.py",
                    "--input-csv",
                    str(input_csv),
                    "--profile-json",
                    str(profile_json),
                    "--output-dir",
                    str(output_dir),
                    "--split",
                    "validation",
                ]
                self.assertEqual(module.main(), 0)
            finally:
                sys.argv = old_argv

            self.assertTrue((output_dir / "profile_replay_predictions.csv").exists())
            self.assertTrue((output_dir / "profile_replay_summary.csv").exists())
            self.assertTrue((output_dir / "profile_replay_summary.txt").exists())


if __name__ == "__main__":
    unittest.main()
