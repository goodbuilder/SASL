from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_check_module():
    spec = importlib.util.spec_from_file_location(
        "stereodistance_profile_promotion_check_under_test",
        ROOT_DIR / "scripts" / "stereodistance_profile_promotion_check.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load stereodistance_profile_promotion_check.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class StereoDistanceProfilePromotionCheckTests(unittest.TestCase):
    def test_promotion_check_loads_profile_through_runtime_and_node_paths(self) -> None:
        module = load_check_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            profile_json = tmp / "profile.json"
            sample_csv = tmp / "samples.csv"
            profile_json.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_sim3",
                        "profile_id": "unit-promotion",
                        "axis_contract": module.RDF_AXIS_CONTRACT,
                        "runtime_inputs": [
                            "raw_right_m",
                            "raw_down_m",
                            "raw_forward_m",
                            "raw_disparity_px",
                        ],
                        "forbidden_runtime_inputs": [
                            "calibration_truth",
                            "px4_rtk_truth",
                            "true_disparity",
                            "true_image_feature_location",
                        ],
                        "scale": 1.0,
                        "rotation_matrix_rdf": [
                            [1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0],
                        ],
                        "translation_rdf_m": [1.0, 2.0, 3.0],
                    }
                ),
                encoding="utf-8",
            )
            sample_csv.write_text(
                "split,raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px\n"
                "validation,10.0,20.0,100.0,50.0\n",
                encoding="utf-8",
            )

            summary = module.run_check(
                profile_json,
                tmp / "out",
                sample_csv=sample_csv,
                split="validation",
                max_samples=5,
            )

        self.assertTrue(summary["pass_promotion_check"])
        self.assertTrue(summary["schema_load_ok"])
        self.assertTrue(summary["node_loader_ok"])
        self.assertFalse(summary["runtime_truth_used"])
        self.assertEqual(summary["sample_apply_count"], 1)

    def test_promotion_check_accepts_disparity_profile(self) -> None:
        module = load_check_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            profile_json = tmp / "disparity_profile.json"
            sample_csv = tmp / "samples.csv"
            profile_json.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_disparity_poly",
                        "profile_id": "unit-disparity-promotion",
                        "axis_contract": module.RDF_AXIS_CONTRACT,
                        "runtime_inputs": [
                            "raw_right_m",
                            "raw_down_m",
                            "raw_forward_m",
                            "raw_disparity_px",
                        ],
                        "forbidden_runtime_inputs": [
                            "calibration_truth",
                            "px4_rtk_truth",
                            "true_disparity",
                            "true_image_feature_location",
                        ],
                        "correction_space": "raw_rdf_ray_scale",
                        "d_corr_coefficients_high_to_low": [1.0, 0.0],
                    }
                ),
                encoding="utf-8",
            )
            sample_csv.write_text(
                "split,raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px\n"
                "validation,10.0,20.0,100.0,50.0\n",
                encoding="utf-8",
            )

            summary = module.run_check(
                profile_json,
                tmp / "out",
                sample_csv=sample_csv,
                split="validation",
                max_samples=5,
            )

        self.assertTrue(summary["pass_promotion_check"])
        self.assertTrue(summary["schema_load_ok"])
        self.assertTrue(summary["node_loader_ok"])
        self.assertEqual(summary["profile_mode"], "estimated_to_truth_disparity_poly")

    def test_promotion_check_accepts_affine_profile(self) -> None:
        module = load_check_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            profile_json = tmp / "affine_profile.json"
            sample_csv = tmp / "samples.csv"
            profile_json.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_affine_rdf",
                        "profile_id": "unit-affine-promotion",
                        "axis_contract": module.RDF_AXIS_CONTRACT,
                        "runtime_inputs": [
                            "raw_right_m",
                            "raw_down_m",
                            "raw_forward_m",
                            "raw_disparity_px",
                        ],
                        "forbidden_runtime_inputs": [
                            "calibration_truth",
                            "px4_rtk_truth",
                            "true_disparity",
                            "true_image_feature_location",
                        ],
                        "affine_matrix_rdf": [
                            [1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0],
                        ],
                        "translation_rdf_m": [1.0, 2.0, 3.0],
                    }
                ),
                encoding="utf-8",
            )
            sample_csv.write_text(
                "split,raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px\n"
                "validation,10.0,20.0,100.0,50.0\n",
                encoding="utf-8",
            )

            summary = module.run_check(
                profile_json,
                tmp / "out",
                sample_csv=sample_csv,
                split="validation",
                max_samples=5,
            )

        self.assertTrue(summary["pass_promotion_check"])
        self.assertTrue(summary["schema_load_ok"])
        self.assertTrue(summary["node_loader_ok"])
        self.assertEqual(summary["profile_mode"], "estimated_to_truth_affine_rdf")

    def test_promotion_check_passes_left_image_geometry_inputs_to_sim3_profile(self) -> None:
        module = load_check_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            profile_json = tmp / "left_geometry_profile.json"
            sample_csv = tmp / "samples.csv"
            profile_json.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_sim3",
                        "profile_id": "unit-left-geometry-promotion",
                        "axis_contract": module.RDF_AXIS_CONTRACT,
                        "runtime_inputs": [
                            "raw_right_m",
                            "raw_down_m",
                            "raw_forward_m",
                            "raw_disparity_px",
                            "left_u_px",
                            "left_v_px",
                        ],
                        "forbidden_runtime_inputs": [
                            "calibration_truth",
                            "px4_rtk_truth",
                            "true_disparity",
                            "true_image_feature_location",
                        ],
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
                            "left_u_offset_px": 0.0,
                            "left_v_offset_px": 0.0,
                            "disparity_correction_coefficients_high_to_low": [1.0, 0.0],
                        },
                    }
                ),
                encoding="utf-8",
            )
            sample_csv.write_text(
                "split,raw_right_m,raw_down_m,raw_forward_m,left_u_px,left_v_px,measured_disparity_px\n"
                "validation,1.0,2.0,100.0,60.0,45.0,20.0\n",
                encoding="utf-8",
            )

            summary = module.run_check(
                profile_json,
                tmp / "out",
                sample_csv=sample_csv,
                split="validation",
                max_samples=5,
            )

            sample_rows = (tmp / "out" / "promotion_check_samples.csv").read_text(encoding="utf-8")

        self.assertTrue(summary["pass_promotion_check"])
        self.assertEqual(summary["sample_apply_count"], 1)
        self.assertEqual(summary["sample_invalid_count"], 0)
        self.assertIn("left_u_px", sample_rows)
        self.assertIn("left_v_px", sample_rows)

    def test_promotion_check_rejects_missing_axis_contract_and_truth_runtime_input(self) -> None:
        module = load_check_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            profile_json = tmp / "bad_profile.json"
            profile_json.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_sim3",
                        "profile_id": "unit-bad-contract",
                        "runtime_inputs": [
                            "raw_right_m",
                            "raw_down_m",
                            "raw_forward_m",
                            "px4_ned_truth_forward_m",
                        ],
                        "forbidden_runtime_inputs": [
                            "calibration_truth",
                            "px4_rtk_truth",
                            "true_disparity",
                            "true_image_feature_location",
                        ],
                        "scale": 1.0,
                        "rotation_matrix_rdf": [
                            [1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0],
                        ],
                        "translation_rdf_m": [0.0, 0.0, 0.0],
                    }
                ),
                encoding="utf-8",
            )

            summary = module.run_check(profile_json, tmp / "out")

        self.assertFalse(summary["pass_promotion_check"])
        self.assertFalse(summary["axis_contract_declared"])
        self.assertTrue(summary["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
