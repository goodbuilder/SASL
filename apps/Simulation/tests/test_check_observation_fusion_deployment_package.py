from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_observation_fusion_deployment_package.py"
SPEC = importlib.util.spec_from_file_location("check_observation_fusion_deployment_package_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


class CheckObservationFusionDeploymentPackageTests(unittest.TestCase):
    def write_minimal_package(self, root: Path) -> None:
        false_contract = {"runtime_truth_used": False, "coordinate_contracts_explicit": True}
        profile_payload = {
            "runtime_truth_used": False,
            "coordinate_contracts_explicit": True,
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
        }
        write_json(root / "active_runtime_profile.json", profile_payload)
        write_json(
            root / "observation_fusion_deployment_package_summary.json",
            {
                **false_contract,
                "default_tier": "fresh_common_mode_b7x3_aggregate_best",
                "profile_json": str(root / "active_runtime_profile.json"),
                "packaged_profile_json": str(root / "active_runtime_profile.json"),
            },
        )
        write_json(
            root / "topic_contract" / "observation_fusion_topic_contract_manifest.json",
            {
                **false_contract,
                "runtime_input_interfaces": ["stereo_bbox_observation", "measured_disparity"],
                "frames": {
                    "internal_measurement": "stereo-center RDF right/down/forward",
                    "public_stereodistance": "FRD+range forward/right/down/range",
                },
            },
        )
        write_json(
            root / "observation_fusion_runtime_manifest.json",
            {
                **false_contract,
                "default_recommendation": "fresh_common_mode_b7x3_aggregate_best",
                "tiers": [
                    {
                        "tier": "fresh_common_mode_b7x3_aggregate_best",
                        "selection_scope": "all_scenario_aggregate",
                        "adapter": {
                            "class": "MultiBurstObservationFusion",
                            "min_bursts": 7,
                            "min_observations_per_burst": 3,
                        },
                        "evidence": {
                            "status": "STRICT_PASS",
                            "pass_count": 4,
                            "trial_count": 4,
                            "runtime_truth_used": False,
                        },
                    }
                ],
            },
        )
        write_json(root / "audit" / "observation_fusion_runtime_manifest_audit.json", {**false_contract, "pass_count": 6, "tier_count": 6})
        write_json(
            root / "bbox_conversion_smoke" / "observation_fusion_bbox_conversion_summary.json",
            {**false_contract, "status": "PASS", "input_row_count": 15, "output_row_count": 15, "failure_count": 0},
        )
        for relative in (
            "bbox_conversion_smoke/observation_fusion_bbox_converted_input_schema_validation.json",
            "bbox_conversion_smoke/observation_fusion_bbox_output_schema_validation.json",
            "calibration_csv_bridge_smoke/observation_fusion_calibration_input_schema_validation.json",
            "calibration_csv_bridge_smoke/observation_fusion_calibration_output_schema_validation.json",
            "path_equivalence/observation_fusion_path_equivalence.json",
            "public_topic_payload/observation_fusion_public_topic_payload_summary.json",
            "public_topic_payload/observation_fusion_public_topic_payload_validation.json",
            "public_topic_payload/observation_fusion_public_topic_payload_source_equivalence.json",
            "public_topic_payload/observation_fusion_public_topic_payload_publish_summary.json",
            "profile_apply_smoke/input_schema_validation/observation_fusion_runtime_input_schema_validation.json",
            "profile_apply_smoke/schema_validation/observation_fusion_runtime_output_schema_validation.json",
        ):
            write_json(root / relative, {**false_contract, "status": "PASS", "failure_count": 0})
        write_json(
            root / "calibration_csv_bridge_smoke" / "observation_fusion_calibration_extract_summary.json",
            {
                **false_contract,
                "status": "PASS",
                "input_row_count": 15,
                "output_row_count": 15,
                "dropped_truth_column_count": 3,
                "failure_count": 0,
            },
        )
        write_json(
            root / "calibration_csv_bridge_smoke" / "observation_fusion_calibration_runtime_apply_summary.json",
            {**false_contract, "valid_sample_count": 1},
        )
        write_json(
            root / "calibration_csv_bridge_smoke" / "observation_fusion_calibration_profile_apply_summary.json",
            {**false_contract, "valid_profile_output_count": 1},
        )
        write_json(root / "bbox_conversion_smoke" / "observation_fusion_bbox_runtime_apply_summary.json", {**false_contract, "valid_sample_count": 1})
        write_json(
            root / "bbox_conversion_smoke" / "observation_fusion_bbox_profile_apply_summary.json",
            {**false_contract, "valid_profile_output_count": 1},
        )
        write_json(root / "apply_smoke" / "observation_fusion_runtime_apply_summary.json", {**false_contract, "valid_sample_count": 1})
        write_json(root / "profile_apply_smoke" / "observation_fusion_profile_apply_summary.json", {**false_contract, "valid_profile_output_count": 1})
        write_json(
            root / "operator_view_fusion_launch_check.json",
            {
                **false_contract,
                "status": "PASS",
                "failure_count": 0,
                "observation_fusion_manifest_status": "PASS",
                "launch_contract_status": "PASS",
                "observation_fusion_arg_present": True,
                "bbox_adjustment_arg_present": True,
            },
        )
        write_json(
            root / "operator_view_fusion_tier_override_launch_check.json",
            {
                **false_contract,
                "status": "PASS",
                "failure_count": 0,
                "observation_fusion_manifest_status": "PASS",
                "launch_contract_status": "PASS",
                "observation_fusion_arg_present": True,
                "bbox_adjustment_arg_present": True,
                "observation_fusion_tier": "fresh_independent_n7_low_latency",
                "operator_command_stdout": "--observation-fusion-tier fresh_independent_n7_low_latency",
            },
        )
        write_json(
            root / "operator_view_package_launch_check.json",
            {
                **false_contract,
                "status": "PASS",
                "failure_count": 0,
                "package_type": "lightweight_profile_package",
                "observation_fusion_manifest_status": "PASS",
                "observation_fusion_arg_present": True,
                "resolved_runtime_profile": str(root / "active_runtime_profile.json"),
            },
        )
        write_json(
            root / "stereodistance_observation_fusion_node_smoke.json",
            {
                **false_contract,
                "status": "PASS",
                "failure_count": 0,
                "adapter_class": "MultiBurstObservationFusion",
                "observation_fusion_status": "fused:MultiBurstObservationFusion:21",
            },
        )
        write_json(
            root / "tier_override_smoke" / "observation_fusion_tier_override_smoke.json",
            {
                **false_contract,
                "status": "PASS",
                "tier_count": 2,
                "pass_count": 2,
                "default_tier_count": 1,
                "failure_count": 0,
            },
        )

    def add_residual_profile(self, root: Path) -> None:
        residual_profile = {
            "profile_type": "DisparityResidualRuntimeProfile",
            "profile_id": "unit-residual",
            "feature_space": "log_disparity",
            "correction_space": "ray_scale",
            "degree": 2,
            "feature_mean": 5.3,
            "feature_scale": 0.5,
            "coefficients_low_to_high": [0.01, 0.0, 0.0],
            "correction_clip": 0.5,
            "runtime_inputs": [
                "raw_right_m",
                "raw_down_m",
                "raw_forward_m",
                "measured_disparity_px",
                "left_u_px",
                "left_v_px",
            ],
            "runtime_truth_used": False,
            "coordinate_contracts_explicit": True,
        }
        residual_path = root / "disparity_residual_profile.json"
        write_json(residual_path, residual_profile)
        summary_path = root / "observation_fusion_deployment_package_summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["disparity_residual_profile_json"] = str(residual_path)
        summary["packaged_disparity_residual_profile_json"] = str(residual_path)
        write_json(summary_path, summary)
        node_smoke_path = root / "stereodistance_observation_fusion_node_smoke.json"
        node_smoke = json.loads(node_smoke_path.read_text(encoding="utf-8"))
        node_smoke["disparity_residual_status"] = "applied:unit-residual"
        write_json(node_smoke_path, node_smoke)

    def test_accepts_complete_truth_free_package(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)

            result = module.check_package(root)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)
        self.assertFalse(result["runtime_truth_used"])
        self.assertEqual(result["default_selection_scope"], "all_scenario_aggregate")
        self.assertEqual(result["default_adapter_class"], "MultiBurstObservationFusion")
        self.assertEqual(result["default_evidence_pass"], "4/4")
        self.assertTrue(result["packaged_profile"].endswith("active_runtime_profile.json"))
        self.assertEqual(result["tier_override_smoke_status"], "PASS")
        self.assertEqual(result["tier_override_smoke_pass"], "2/2")
        self.assertEqual(result["operator_view_fusion_tier_override_launch_status"], "PASS")
        self.assertEqual(result["operator_view_fusion_tier_override"], "fresh_independent_n7_low_latency")
        self.assertEqual(result["operator_view_package_launch_status"], "PASS")
        self.assertEqual(result["disparity_residual_profile_status"], "SKIP")

    def test_accepts_truth_free_residual_profile_package(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            self.add_residual_profile(root)

            result = module.check_package(root)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["disparity_residual_profile_status"], "PASS")
        self.assertEqual(result["disparity_residual_profile_id"], "unit-residual")
        self.assertEqual(
            result["stereodistance_node_smoke_disparity_residual_status"],
            "applied:unit-residual",
        )

    def test_rejects_topic_truth_as_runtime_input(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "topic_contract" / "observation_fusion_topic_contract_manifest.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["runtime_input_interfaces"].append("calibration_truth")
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("topic_contract:calibration_truth_marked_runtime_input", result["failures"])

    def test_rejects_output_schema_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "profile_apply_smoke/schema_validation/observation_fusion_runtime_output_schema_validation.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["status"] = "FAIL"
            payload["failure_count"] = 1
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("output_schema:not_pass", result["failures"])

    def test_rejects_bbox_profile_apply_without_valid_output(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "bbox_conversion_smoke/observation_fusion_bbox_profile_apply_summary.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["valid_profile_output_count"] = 0
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("bbox_profile_apply:no_valid_outputs", result["failures"])

    def test_rejects_calibration_extract_that_drops_no_truth(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "calibration_csv_bridge_smoke/observation_fusion_calibration_extract_summary.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["dropped_truth_column_count"] = 0
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("calibration_extract:no_truth_columns_dropped", result["failures"])

    def test_rejects_missing_operator_view_fusion_launch_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            (root / "operator_view_fusion_launch_check.json").unlink()

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(
            any(failure.startswith("missing_required_file:operator_view_fusion_launch") for failure in result["failures"])
        )

    def test_rejects_missing_operator_view_tier_override_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            (root / "operator_view_fusion_tier_override_launch_check.json").unlink()

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(
            any(
                failure.startswith("missing_required_file:operator_view_fusion_tier_override_launch")
                for failure in result["failures"]
            )
        )

    def test_rejects_packaged_profile_with_truth_runtime_input(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "active_runtime_profile.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["runtime_inputs"] = ["raw_right_m", "truth_forward_m"]
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("packaged_profile:runtime_truth_input", result["failures"])

    def test_rejects_node_smoke_that_did_not_fuse(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "stereodistance_observation_fusion_node_smoke.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["observation_fusion_status"] = "warming_up:2/21"
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("stereodistance_node_smoke:not_fused", result["failures"])

    def test_rejects_residual_profile_when_node_smoke_did_not_apply_it(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            self.add_residual_profile(root)
            path = root / "stereodistance_observation_fusion_node_smoke.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["disparity_residual_status"] = "disabled"
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("stereodistance_node_smoke:disparity_residual_not_applied", result["failures"])

    def test_rejects_residual_profile_with_truth_runtime_input(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            self.add_residual_profile(root)
            path = root / "disparity_residual_profile.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["runtime_inputs"].append("px4_truth_forward_m")
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("disparity_residual_profile:runtime_truth_input", result["failures"])

    def test_rejects_default_tier_without_aggregate_scope(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "observation_fusion_runtime_manifest.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["tiers"][0]["selection_scope"] = "single_source"
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("manifest:default_tier_not_all_scenario_aggregate", result["failures"])

    def test_rejects_default_tier_summary_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "observation_fusion_deployment_package_summary.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["default_tier"] = "single_source_tier"
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("manifest:default_tier_summary_mismatch", result["failures"])

    def test_rejects_tier_override_smoke_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "tier_override_smoke" / "observation_fusion_tier_override_smoke.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["pass_count"] = 1
            payload["status"] = "FAIL"
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("tier_override_smoke:not_pass", result["failures"])
        self.assertIn("tier_override_smoke:not_all_tiers_passed", result["failures"])

    def test_rejects_operator_view_tier_override_without_tier_arg(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            path = root / "operator_view_fusion_tier_override_launch_check.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["observation_fusion_tier"] = ""
            payload["operator_command_stdout"] = "--observation-fusion-manifest manifest.json"
            write_json(path, payload)

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("operator_view_fusion_tier_override_launch:tier_missing", result["failures"])
        self.assertIn("operator_view_fusion_tier_override_launch:tier_arg_missing", result["failures"])

    def test_rejects_missing_operator_view_package_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_minimal_package(root)
            (root / "operator_view_package_launch_check.json").unlink()

            result = module.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(
            any(failure.startswith("missing_required_file:operator_view_package_launch") for failure in result["failures"])
        )


if __name__ == "__main__":
    unittest.main()
