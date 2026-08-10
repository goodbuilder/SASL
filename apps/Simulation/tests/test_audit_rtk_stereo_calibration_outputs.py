from __future__ import annotations

import csv
import struct
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "audit_rtk_stereo_calibration_outputs.py"
SPEC = importlib.util.spec_from_file_location("audit_rtk_stereo_calibration_outputs_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def write_png(path: Path, *, width: int = 800, height: int = 450) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + struct.pack(">I", 13)
        + b"IHDR"
        + struct.pack(">II", width, height)
        + b"\x08\x02\x00\x00\x00"
        + b"\x00\x00\x00\x00"
    )


class AuditRtkStereoCalibrationOutputsTests(unittest.TestCase):
    def write_minimal_result(self, root: Path) -> None:
        common = {
            "rdf_axis_contract": module.RDF_AXIS_CONTRACT,
            "registration_model": "none",
            "residual_model": "none",
            "regularization_model": "none",
            "runtime_profile_family": "estimated_to_truth_sim3",
            "selection_note": "test",
            "model_complexity": 1,
            "train_range_rmse_m": 2.0,
            "validation_right_rmse_m": 0.5,
            "validation_down_rmse_m": 0.4,
            "validation_forward_rmse_m": 0.3,
            "validation_target_window_count": 3,
            "validation_target_max_abs_error_m": 2.0,
            "parameters_json": "{}",
        }
        write_csv(
            root / "method_comparison.csv",
            [
                {
                    **common,
                    "method": "raw",
                    "runtime_profile_family": "none_raw",
                    "validation_range_rmse_m": 3.0,
                    "validation_target_abs_mean_error_m": 8.0,
                },
                {
                    **common,
                    "method": "sim3",
                    "validation_range_rmse_m": 1.0,
                    "validation_target_abs_mean_error_m": 1.5,
                },
            ],
        )
        prediction_common = {
            "sample_id": "v0",
            "split": "validation",
            "rdf_axis_contract": module.RDF_AXIS_CONTRACT,
            "pred_right_m": 1.0,
            "pred_down_m": 2.0,
            "pred_forward_m": 1852.0,
            "pred_range_m": 1852.001,
            "truth_right_m": 1.0,
            "truth_down_m": 2.0,
            "truth_forward_m": 1851.0,
            "truth_range_m": 1851.001,
            "error_right_m": 0.0,
            "error_down_m": 0.0,
            "error_forward_m": 1.0,
            "range_error_m": 1.0,
            "measured_disparity_px": 12.0,
            "true_disparity_px": 12.1,
        }
        write_csv(
            root / "validation_predictions.csv",
            [
                {"method": "raw", **prediction_common},
                {"method": "sim3", **prediction_common},
            ],
        )
        write_csv(
            root / "target_1852m_stats.csv",
            [
                {
                    "method": "raw",
                    "target_range_m": 1852.0,
                    "window_half_width_m": 30.0,
                    "sample_count": 3,
                    "mean_error_m": 8.0,
                    "abs_mean_error_m": 8.0,
                    "rmse_m": 8.0,
                    "max_abs_error_m": 9.0,
                    "pass_abs_mean_within_10m": "yes",
                },
                {
                    "method": "sim3",
                    "target_range_m": 1852.0,
                    "window_half_width_m": 30.0,
                    "sample_count": 3,
                    "mean_error_m": 1.5,
                    "abs_mean_error_m": 1.5,
                    "rmse_m": 1.6,
                    "max_abs_error_m": 2.0,
                    "pass_abs_mean_within_10m": "yes",
                },
            ],
        )
        (root / "summary.txt").write_text(
            "Method performance table\nFinal recommendation: sim3\n",
            encoding="utf-8",
        )
        write_json(
            root / "run_config.json",
            {
                "recommendation": "sim3",
                "sample_count": 10,
                "train_count": 8,
                "validation_count": 2,
                "train_ratio": 0.8,
                "target_range_m": 1852.0,
                "validation_target_window_count": 3,
                "split_strategy": "target_window_stratified_when_target_range_and_window_are_available",
                "contracts": {
                    "rdf": module.RDF_AXIS_CONTRACT,
                    "stereodistance_public_topic": module.PUBLIC_TOPIC_CONTRACT,
                },
            },
        )
        write_json(root / "method_parameters.json", {"raw": {}, "sim3": {}})
        write_csv(
            root / "policy_recommendations.csv",
            [
                {
                    "policy": "balanced",
                    "purpose": "test",
                    "method": "sim3",
                    "is_final_recommendation": "True",
                    "validation_range_rmse_m": 1.0,
                    "validation_position_rmse_m": 1.1,
                    "validation_right_rmse_m": 0.5,
                    "validation_down_rmse_m": 0.4,
                    "validation_forward_rmse_m": 0.3,
                    "validation_target_abs_mean_error_m": 1.5,
                    "validation_target_max_abs_error_m": 2.0,
                    "raw_validation_range_rmse_m": 3.0,
                    "target_tolerance_m": 10.0,
                }
            ],
        )
        write_json(
            root / "policy_recommendations.json",
            {
                "policies": [{"policy": "balanced", "method": "sim3"}],
                "runtime_profile_contract": "estimated_to_truth_sim3 profiles use internal RDF(right,down,forward)",
            },
        )
        write_json(
            root / "runtime_profiles" / "manifest.json",
            [
                {
                    "method": "sim3",
                    "profile_path": "runtime_profiles/sim3.json",
                    "validation_range_rmse_m": 1.0,
                    "target_1852m_abs_mean_error_m": 1.5,
                    "selected": True,
                }
            ],
        )
        selected_profile = {
            "mode": "estimated_to_truth_disparity_poly",
            "method": "sim3",
            "axis_contract": "internal_policy_vector=RDF(right,down,forward); legacy_stereodistance_position_topics=FRD_range(forward,right,down,range)",
            "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "measured_disparity_px"],
            "forbidden_runtime_inputs": ["truth_rdf_m", "px4_local_position_ned", "rtk_truth"],
            "correction_space": "raw_rdf_inverse_depth_ray_scale",
            "inverse_depth_coefficients_high_to_low": [1.0, 0.0],
            "validation_summary": {
                "validation_range_rmse_raw_m": 3.0,
                "validation_range_rmse_corrected_m": 1.0,
                "target_1852m_abs_mean_error_m": 1.5,
            },
        }
        write_json(root / "runtime_profiles" / "sim3.json", selected_profile)
        write_json(root / "sim3_runtime_profile.json", selected_profile)
        policy_profile = dict(selected_profile)
        policy_profile["policy"] = "balanced"
        write_json(
            root / "policy_profiles" / "manifest.json",
            [
                {
                    "policy": "balanced",
                    "method": "sim3",
                    "profile_path": "policy_profiles/balanced.json",
                    "validation_range_rmse_m": 1.0,
                    "target_1852m_abs_mean_error_m": 1.5,
                    "model_complexity": 1,
                }
            ],
        )
        write_json(root / "policy_profiles" / "balanced.json", policy_profile)
        for name in (
            "range_error_before_after.png",
            "residual_disparity_by_distance.png",
            "rdf_axis_rmse.png",
            "pareto_frontier.png",
        ):
            write_png(root / name)

    def test_accepts_complete_result_contract(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)
        self.assertEqual(result["selected_profile_load_status"], "PASS")
        self.assertEqual(result["selected_profile_apply_status"], "PASS")
        self.assertEqual(result["top_level_profile_equivalence_status"], "PASS")
        self.assertEqual(result["runtime_profile_count"], 1)
        self.assertEqual(result["runtime_profile_load_pass_count"], 1)
        self.assertEqual(result["runtime_profile_missing_methods"], ["raw"])
        self.assertEqual(result["policy_profile_count"], 1)
        self.assertEqual(result["policy_profile_load_pass_count"], 1)
        self.assertEqual(
            sorted(result["deliverable_evidence_matrix"].keys()),
            sorted(module.DELIVERABLE_EVIDENCE_ITEMS),
        )
        self.assertTrue(
            all(item["status"] == "PASS" for item in result["deliverable_evidence_matrix"].values())
        )
        self.assertEqual(
            result["deliverable_evidence_matrix"]["runtime_profiles_manifest"]["evidence"]["selected_profile_path"],
            "runtime_profiles/sim3.json",
        )
        self.assertEqual(result["plot_dimensions"]["range_error_plot"], "800x450")
        self.assertFalse(result["runtime_truth_used"])

    def test_rejects_recommendation_that_does_not_improve_raw_validation_rmse(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            columns, rows = module.read_csv_rows(root / "method_comparison.csv")
            rows[1]["validation_range_rmse_m"] = "4.0"
            write_csv(root / "method_comparison.csv", rows)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("recommendation:validation_range_rmse_not_improved_over_raw", result["failures"])

    def test_rejects_recommendation_with_rdf_axis_rmse_regression(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            columns, rows = module.read_csv_rows(root / "method_comparison.csv")
            rows[1]["validation_right_rmse_m"] = "0.6"
            write_csv(root / "method_comparison.csv", rows)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("recommendation:right_validation_rmse_regressed_over_raw", result["failures"])
        self.assertEqual(
            result["deliverable_evidence_matrix"]["rdf_axis_rmse_nonregression"]["status"],
            "FAIL",
        )

    def test_rejects_missing_target_window_samples(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            payload = json.loads((root / "run_config.json").read_text(encoding="utf-8"))
            payload["validation_target_window_count"] = 0
            write_json(root / "run_config.json", payload)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("run_config:no_validation_target_window_samples", result["failures"])

    def test_rejects_non_80_20_train_ratio(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            payload = json.loads((root / "run_config.json").read_text(encoding="utf-8"))
            payload["train_ratio"] = 0.7
            payload["train_count"] = 7
            payload["validation_count"] = 3
            write_json(root / "run_config.json", payload)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("run_config:train_ratio_not_80_20", result["failures"])

    def test_rejects_prediction_rdf_contract_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            columns, rows = module.read_csv_rows(root / "validation_predictions.csv")
            rows[0]["rdf_axis_contract"] = "wrong"
            write_csv(root / "validation_predictions.csv", rows)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("validation_predictions:raw:rdf_contract_mismatch", result["failures"])

    def test_rejects_policy_final_method_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            columns, rows = module.read_csv_rows(root / "policy_recommendations.csv")
            rows[0]["method"] = "raw"
            write_csv(root / "policy_recommendations.csv", rows)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("policy_recommendations:final_method_not_recommendation", result["failures"])

    def test_rejects_selected_profile_with_truth_runtime_input(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            path = root / "runtime_profiles" / "sim3.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["runtime_inputs"].append("truth_rdf_m")
            write_json(path, payload)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("selected_profile:runtime_inputs_include_truth", result["failures"])
        self.assertIn("runtime_profiles:sim3:runtime_inputs_include_truth", result["failures"])

    def test_rejects_selected_profile_metric_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            path = root / "runtime_profiles" / "sim3.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["validation_summary"]["validation_range_rmse_corrected_m"] = 2.0
            write_json(path, payload)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("selected_profile:corrected_validation_rmse_mismatch", result["failures"])

    def test_rejects_selected_profile_runtime_loader_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            path = root / "runtime_profiles" / "sim3.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["inverse_depth_coefficients_high_to_low"] = []
            write_json(path, payload)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("runtime_profiles:sim3:runtime_loader_rejected_profile", result["failures"])
        self.assertIn("runtime_profiles:load_apply_pass_count_mismatch", result["failures"])
        self.assertIn("selected_profile:runtime_loader_rejected_profile", result["failures"])

    def test_rejects_unselected_runtime_profile_loader_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            columns, rows = module.read_csv_rows(root / "method_comparison.csv")
            extra_method = dict(rows[1])
            extra_method["method"] = "sim3_alt"
            rows.append(extra_method)
            write_csv(root / "method_comparison.csv", rows)
            parameters = json.loads((root / "method_parameters.json").read_text(encoding="utf-8"))
            parameters["sim3_alt"] = {}
            write_json(root / "method_parameters.json", parameters)
            columns, prediction_rows = module.read_csv_rows(root / "validation_predictions.csv")
            extra_prediction = dict(prediction_rows[1])
            extra_prediction["method"] = "sim3_alt"
            prediction_rows.append(extra_prediction)
            write_csv(root / "validation_predictions.csv", prediction_rows)
            manifest_path = root / "runtime_profiles" / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest.append(
                {
                    "method": "sim3_alt",
                    "profile_path": "runtime_profiles/sim3_alt.json",
                    "validation_range_rmse_m": 1.0,
                    "target_1852m_abs_mean_error_m": 1.5,
                    "model_complexity": 1,
                    "selected": False,
                }
            )
            write_json(manifest_path, manifest)
            profile = json.loads((root / "runtime_profiles" / "sim3.json").read_text(encoding="utf-8"))
            profile["method"] = "sim3_alt"
            profile["mode"] = "unsupported"
            write_json(root / "runtime_profiles" / "sim3_alt.json", profile)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["runtime_profile_count"], 2)
        self.assertEqual(result["runtime_profile_load_pass_count"], 1)
        self.assertIn("runtime_profiles:sim3_alt:runtime_loader_rejected_profile", result["failures"])
        self.assertIn("runtime_profiles:load_apply_pass_count_mismatch", result["failures"])

    def test_rejects_missing_runtime_profile_for_runtime_family(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            write_json(root / "runtime_profiles" / "manifest.json", [])

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn(
            "runtime_profiles:sim3:missing_profile_for_runtime_family:estimated_to_truth_sim3",
            result["failures"],
        )

    def test_rejects_top_level_profile_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            path = root / "sim3_runtime_profile.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["method"] = "raw"
            write_json(path, payload)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("top_level_runtime_profile:not_equal_selected_profile", result["failures"])
        self.assertIn("top_level_runtime_profile:method_not_recommendation", result["failures"])

    def test_rejects_policy_profile_metric_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            path = root / "policy_profiles" / "manifest.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload[0]["validation_range_rmse_m"] = 9.0
            write_json(path, payload)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("policy_profiles:balanced:validation_rmse_mismatch", result["failures"])

    def test_rejects_policy_profile_runtime_loader_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            path = root / "policy_profiles" / "balanced.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["mode"] = "unsupported"
            write_json(path, payload)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("policy_profiles:balanced:runtime_loader_rejected_profile", result["failures"])

    def test_accepts_multiple_final_policy_rows_when_methods_match_recommendation(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            columns, rows = module.read_csv_rows(root / "policy_recommendations.csv")
            extra = dict(rows[0])
            extra["policy"] = "lowest_complexity_pass"
            rows.append(extra)
            write_csv(root / "policy_recommendations.csv", rows)
            manifest_path = root / "policy_profiles" / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            extra_manifest = dict(manifest[0])
            extra_manifest["policy"] = "lowest_complexity_pass"
            extra_manifest["profile_path"] = "policy_profiles/lowest_complexity_pass.json"
            manifest.append(extra_manifest)
            write_json(manifest_path, manifest)
            profile = json.loads((root / "policy_profiles" / "balanced.json").read_text(encoding="utf-8"))
            profile["policy"] = "lowest_complexity_pass"
            write_json(root / "policy_profiles" / "lowest_complexity_pass.json", profile)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "PASS")

    def test_rejects_invalid_png(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            (root / "rdf_axis_rmse.png").write_bytes(b"not-png")

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("rdf_axis_plot:invalid_png", result["failures"])

    def test_rejects_too_small_plot_png(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_result(root)
            write_png(root / "range_error_before_after.png", width=320, height=200)

            result = module.audit_outputs(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("range_error_plot:png_dimensions_too_small:320x200", result["failures"])
        self.assertIn("deliverable_evidence:range_error_plot_png:not_pass", result["failures"])


if __name__ == "__main__":
    unittest.main()
