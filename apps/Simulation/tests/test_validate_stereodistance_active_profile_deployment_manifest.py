from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "validate_stereodistance_active_profile_deployment_manifest.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "validate_stereodistance_active_profile_deployment_manifest_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load validate_stereodistance_active_profile_deployment_manifest.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ValidateStereoDistanceActiveProfileDeploymentManifestTests(unittest.TestCase):
    def write_valid_fixture(self, root: Path, module) -> Path:
        profile = root / "active_runtime_profile.json"
        profile.write_text(
            json.dumps(
                {
                    "profile_id": "unit-profile",
                    "method": "sim3",
                    "mode": "estimated_to_truth_sim3",
                    "axis_contract": "internal=RDF(right,down,forward); public=FRD+range",
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
                    ],
                }
            ),
            encoding="utf-8",
        )
        promotion = root / "promotion_check.json"
        promotion.write_text(
            json.dumps(
                {
                    "pass_promotion_check": True,
                    "node_loader_ok": True,
                    "axis_contract_declared": True,
                    "runtime_truth_used": False,
                    "sample_apply_count": 4,
                    "sample_invalid_count": 0,
                }
            ),
            encoding="utf-8",
        )
        final_summary = root / "rtk_stereo_acceptance_matrix_summary.json"
        final_summary.write_text(
            json.dumps(
                {
                    "overall_pass": True,
                    "matrix_item_count": 28,
                    "stage_status": {"standalone_profile_promotion": True},
                    "matrix_rows": [
                        {
                            "stage": "single_run_output_audit",
                            "item": "rdf_axis_rmse_nonregression",
                            "status": "PASS",
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        manifest = root / "active_profile_deployment_manifest.json"
        manifest.write_text(
            json.dumps(
                {
                    "mode": "stereodistance_active_profile_deployment_manifest",
                    "status": "PASS",
                    "active_profile": str(profile),
                    "active_profile_sha256": module.sha256_file(profile),
                    "profile_id": "unit-profile",
                    "method": "sim3",
                    "profile_mode": "estimated_to_truth_sim3",
                    "axis_contract": "internal=RDF(right,down,forward); public=FRD+range",
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                    "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth", "true_disparity"],
                    "promotion_check_json": str(promotion),
                    "promotion_pass": True,
                    "promotion_sample_apply_count": 4,
                    "promotion_sample_invalid_count": 0,
                    "final_acceptance_summary": str(final_summary),
                    "final_overall_pass": True,
                    "final_matrix_item_count": 28,
                    "final_rdf_axis_nonregression_status": "PASS",
                    "runtime_truth_used": False,
                    "coordinate_contract": (
                        "runtime consumes stereo-center RDF(right,down,forward) plus measured disparity; "
                        "PX4/RTK/NED truth is calibration/evaluation only; public StereoDistance topics use FRD+range"
                    ),
                }
            ),
            encoding="utf-8",
        )
        return manifest

    def test_valid_manifest_passes(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self.write_valid_fixture(root, module)

            result = module.validate_manifest(manifest)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)
        self.assertEqual(result["promotion_sample_apply_count"], 4)
        self.assertEqual(result["final_matrix_item_count"], 28)
        self.assertEqual(result["final_rdf_axis_nonregression_status"], "PASS")

    def test_hash_mismatch_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self.write_valid_fixture(root, module)
            payload = json.loads(manifest.read_text(encoding="utf-8"))
            payload["active_profile_sha256"] = "bad"
            manifest.write_text(json.dumps(payload), encoding="utf-8")

            result = module.validate_manifest(manifest)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("active_profile:sha256_mismatch", result["failures"])

    def test_runtime_truth_input_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self.write_valid_fixture(root, module)
            payload = json.loads(root.joinpath("active_runtime_profile.json").read_text(encoding="utf-8"))
            payload["runtime_inputs"] = ["raw_right_m", "px4_ned_truth_forward_m"]
            root.joinpath("active_runtime_profile.json").write_text(json.dumps(payload), encoding="utf-8")
            manifest_payload = json.loads(manifest.read_text(encoding="utf-8"))
            manifest_payload["active_profile_sha256"] = module.sha256_file(root / "active_runtime_profile.json")
            manifest.write_text(json.dumps(manifest_payload), encoding="utf-8")

            result = module.validate_manifest(manifest)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("active_profile:runtime_inputs_truth", result["failures"])

    def test_missing_axis_guard_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self.write_valid_fixture(root, module)
            final_path = root / "rtk_stereo_acceptance_matrix_summary.json"
            final_payload = json.loads(final_path.read_text(encoding="utf-8"))
            final_payload["matrix_rows"] = []
            final_path.write_text(json.dumps(final_payload), encoding="utf-8")

            result = module.validate_manifest(manifest)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("final_acceptance:rdf_axis_rmse_nonregression", result["failures"])
        self.assertEqual(result["final_rdf_axis_nonregression_status"], "MISSING")

    def test_manifest_axis_guard_status_mismatch_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self.write_valid_fixture(root, module)
            payload = json.loads(manifest.read_text(encoding="utf-8"))
            payload["final_rdf_axis_nonregression_status"] = "FAIL"
            manifest.write_text(json.dumps(payload), encoding="utf-8")

            result = module.validate_manifest(manifest)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("manifest:final_rdf_axis_nonregression_status", result["failures"])


if __name__ == "__main__":
    unittest.main()
