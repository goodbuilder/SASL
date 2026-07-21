from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
EXPORT_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_active_profile_deployment_package.py"
CHECK_SCRIPT_PATH = ROOT_DIR / "scripts" / "check_stereodistance_deployment_package_portability.py"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckStereoDistanceDeploymentPackagePortabilityTests(unittest.TestCase):
    def make_package(self, root: Path) -> Path:
        exporter = load_module("export_stereodistance_active_profile_deployment_package_for_portability_test", EXPORT_SCRIPT_PATH)
        active_profile = root / "active_runtime_profile.json"
        active_profile.write_text(
            json.dumps(
                {
                    "profile_id": "unit",
                    "method": "sim3",
                    "mode": "estimated_to_truth_sim3",
                    "axis_contract": "RDF(right,down,forward)",
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                    "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
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
        promotion = root / "promotion_check.json"
        promotion.write_text(
            '{"pass_promotion_check": true, "sample_apply_count": 3, '
            '"sample_invalid_count": 0, "runtime_truth_used": false}\n',
            encoding="utf-8",
        )
        final = root / "rtk_stereo_acceptance_matrix_summary.json"
        final.write_text(
            '{"overall_pass": true, "matrix_item_count": 28, "runtime_truth_used": false, '
            '"stage_status": {"standalone_profile_promotion": true}, '
            '"matrix_rows": [{"stage": "single_run_output_audit", '
            '"item": "rdf_axis_rmse_nonregression", "status": "PASS"}]}\n',
            encoding="utf-8",
        )
        manifest = root / "active_profile_deployment_manifest.json"
        manifest.write_text(
            json.dumps(
                {
                    "mode": "stereodistance_active_profile_deployment_manifest",
                    "status": "PASS",
                    "active_profile": str(active_profile),
                    "active_profile_sha256": exporter.sha256_file(active_profile),
                    "profile_id": "unit",
                    "method": "sim3",
                    "profile_mode": "estimated_to_truth_sim3",
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                    "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
                    "promotion_check_json": str(promotion),
                    "final_acceptance_summary": str(final),
                    "runtime_truth_used": False,
                    "final_rdf_axis_nonregression_status": "PASS",
                    "coordinate_contract": (
                        "runtime consumes stereo-center RDF(right,down,forward) plus measured disparity; "
                        "PX4/RTK/NED truth is calibration/evaluation only; public StereoDistance topics use FRD+range"
                    ),
                }
            ),
            encoding="utf-8",
        )
        validation = root / "active_profile_deployment_manifest_validation.json"
        validation.write_text('{"status": "PASS", "failure_count": 0, "runtime_truth_used": false}\n', encoding="utf-8")
        package = root / "package"
        exporter.export_package(manifest_json=manifest, manifest_validation_json=validation, output_dir=package)

        selector_json = root / "runtime_profile_label_selector.json"
        selector_json.write_text(
            '{"mode":"stereodistance_runtime_profile_label_selector",'
            '"axis_contract":"RDF(right,down,forward)",'
            '"public_topic_contract":"FRD_range(forward,right,down,range)",'
            '"selection_input_fields":["deployment_label"]}\n',
            encoding="utf-8",
        )
        frontier_dir = root / "runtime_deployment_frontier"
        frontier_dir.mkdir()
        (frontier_dir / "runtime_deployment_frontier.json").write_text(
            '{"summary":{"status":"PASS","profile_count":1,"deployment_label_count":1,'
            '"runtime_truth_used":false,"truth_free_runtime_selection":true,'
            '"axis_contract":"RDF(right,down,forward)"},'
            '"profiles":[{"profile_label":"active"}],'
            f'"deployment_labels":[{{"deployment_label":"runtime_observation_stress","selected_profile":"{package / "active_runtime_profile.json"}"}}]}}\n',
            encoding="utf-8",
        )
        (frontier_dir / "runtime_deployment_frontier.txt").write_text("status: PASS\n", encoding="utf-8")
        (frontier_dir / "runtime_deployment_frontier_labels.csv").write_text(
            "deployment_label,status,runtime_truth_used\nruntime_observation_stress,PASS,False\n",
            encoding="utf-8",
        )
        (frontier_dir / "runtime_deployment_frontier_profiles.csv").write_text(
            "profile_label,runtime_truth_used,runtime_inputs_truth_free\nactive,False,True\n",
            encoding="utf-8",
        )
        exporter.append_runtime_deployment_evidence(
            package_dir=package,
            selector_json=selector_json,
            frontier_dir=frontier_dir,
        )
        return package

    def test_copied_package_selector_remains_portable(self) -> None:
        checker = load_module("check_stereodistance_deployment_package_portability_under_test", CHECK_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_package(root)

            result = checker.check_portability(
                package_dir=package,
                deployment_label="runtime_observation_stress",
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["selected_label"], "packaged_active")
        self.assertEqual(result["copied_package_check_status"], "PASS")
        self.assertTrue(result["copied_package_dir_ephemeral"])
        self.assertIn("temporary directory", result["copied_package_dir_note"])
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])

    def test_missing_package_selector_fails(self) -> None:
        checker = load_module("check_stereodistance_deployment_package_portability_under_test_missing", CHECK_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_package(root)
            (package / "evidence" / "package_runtime_profile_label_selector.json").unlink()

            result = checker.check_portability(
                package_dir=package,
                deployment_label="runtime_observation_stress",
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("copied_package_selector:missing", result["failures"])


if __name__ == "__main__":
    unittest.main()
