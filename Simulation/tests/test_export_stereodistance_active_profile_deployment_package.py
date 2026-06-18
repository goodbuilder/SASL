from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_active_profile_deployment_package.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "export_stereodistance_active_profile_deployment_package_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load export_stereodistance_active_profile_deployment_package.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportStereoDistanceActiveProfileDeploymentPackageTests(unittest.TestCase):
    def write_fixture(
        self,
        root: Path,
        module,
        *,
        validation_status: str = "PASS",
        include_public_payload: bool = False,
    ) -> tuple[Path, Path]:
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
                }
            ),
            encoding="utf-8",
        )
        promotion = root / "promotion_check.json"
        promotion.write_text('{"pass_promotion_check": true, "sample_apply_count": 3, "sample_invalid_count": 0}\n', encoding="utf-8")
        final = root / "rtk_stereo_acceptance_matrix_summary.json"
        final.write_text('{"overall_pass": true, "matrix_item_count": 28}\n', encoding="utf-8")
        public_payload_summary = root / "public_corrected_position_payload_summary.json"
        public_payload_csv = root / "public_corrected_position_payload.csv"
        public_payload_txt = root / "public_corrected_position_payload_summary.txt"
        if include_public_payload:
            public_payload_summary.write_text(
                '{"status": "PASS", "sample_count": 2, "invalid_count": 0, '
                '"runtime_truth_used": false, '
                '"input_coordinate_contract": "runtime_input:stereo_center_RDF(right,down,forward)", '
                '"public_topic_contract": "std_msgs/Float32MultiArray:data=[forward_m,right_m,down_m,range_m]"}\n',
                encoding="utf-8",
            )
            public_payload_csv.write_text(
                "topic,data_axis_order,data,runtime_truth_used\n"
                '"/stereo/corrected_position","forward_m,right_m,down_m,range_m","[1.0,2.0,3.0,4.0]",False\n'
                '"/stereo/corrected_position","forward_m,right_m,down_m,range_m","[5.0,6.0,7.0,8.0]",False\n',
                encoding="utf-8",
            )
            public_payload_txt.write_text("status: PASS\n", encoding="utf-8")
        payload = {
            "mode": "stereodistance_active_profile_deployment_manifest",
            "status": "PASS",
            "active_profile": str(active_profile),
            "active_profile_sha256": module.sha256_file(active_profile),
            "profile_id": "unit",
            "method": "sim3",
            "profile_mode": "estimated_to_truth_sim3",
            "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
            "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
            "promotion_check_json": str(promotion),
            "final_acceptance_summary": str(final),
            "final_rdf_axis_nonregression_status": "PASS",
            "runtime_truth_used": False,
            "coordinate_contract": (
                "runtime consumes stereo-center RDF(right,down,forward) plus measured disparity; "
                "PX4/RTK/NED truth is calibration/evaluation only; public StereoDistance topics use FRD+range"
            ),
        }
        if include_public_payload:
            payload.update(
                {
                    "public_topic_payload_status": "PASS",
                    "public_topic_payload_sample_count": 2,
                    "public_topic_payload_invalid_count": 0,
                    "public_topic_payload_json": str(public_payload_summary),
                    "public_topic_payload_csv": str(public_payload_csv),
                    "public_topic_payload_txt": str(public_payload_txt),
                }
            )
        manifest = root / "active_profile_deployment_manifest.json"
        manifest.write_text(json.dumps(payload), encoding="utf-8")
        validation = root / "active_profile_deployment_manifest_validation.json"
        validation.write_text(json.dumps({"status": validation_status}), encoding="utf-8")
        return manifest, validation

    def test_exports_deployable_package_and_inventory(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest, validation = self.write_fixture(root, module)
            package = root / "package"

            result = module.export_package(
                manifest_json=manifest,
                manifest_validation_json=validation,
                output_dir=package,
            )
            active_profile_exists = (package / "active_runtime_profile.json").is_file()
            deployment_readme_exists = (package / "DEPLOYMENT_README.txt").is_file()
            package_manifest_exists = (package / "deployment_package_manifest.json").is_file()
            readme_text = (package / "DEPLOYMENT_README.txt").read_text(encoding="utf-8")

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["active_profile"], "active_runtime_profile.json")
        self.assertEqual(result["final_rdf_axis_nonregression_status"], "PASS")
        self.assertEqual(result["file_count"], 6)
        self.assertTrue(active_profile_exists)
        self.assertTrue(deployment_readme_exists)
        self.assertTrue(package_manifest_exists)
        roles = {row["role"] for row in result["files"]}
        self.assertIn("active_profile", roles)
        self.assertIn("deployment_readme", roles)
        self.assertIn("deployment_manifest_validation", roles)
        self.assertIn("run_operator_view_with_calibration_profile.sh --package-dir", readme_text)
        self.assertIn("run_operator_view.sh --bbox-adjustment", readme_text)
        self.assertNotIn("public_corrected_position_payload.csv", readme_text)

    def test_exports_public_payload_evidence_when_manifest_declares_it(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest, validation = self.write_fixture(root, module, include_public_payload=True)
            package = root / "package"

            result = module.export_package(
                manifest_json=manifest,
                manifest_validation_json=validation,
                output_dir=package,
            )
            readme_text = (package / "DEPLOYMENT_README.txt").read_text(encoding="utf-8")

        roles = {row["role"] for row in result["files"]}
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["file_count"], 9)
        self.assertIn("public_topic_payload_summary", roles)
        self.assertIn("public_topic_payload_summary_txt", roles)
        self.assertIn("public_topic_payload_csv", roles)
        self.assertIn("public_corrected_position_payload.csv", readme_text)
        self.assertIn("data=[forward_m,right_m,down_m,range_m]", readme_text)

    def test_export_removes_stale_package_evidence_from_previous_run(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest, validation = self.write_fixture(root, module)
            package = root / "package"
            stale = package / "evidence" / "package_portability_check.json"
            stale.parent.mkdir(parents=True)
            stale.write_text('{"status":"PASS"}\n', encoding="utf-8")

            result = module.export_package(
                manifest_json=manifest,
                manifest_validation_json=validation,
                output_dir=package,
            )
            stale_exists_after_export = stale.exists()

        roles = {row["role"] for row in result["files"]}
        self.assertEqual(result["status"], "PASS")
        self.assertFalse(stale_exists_after_export)
        self.assertNotIn("package_portability_check", roles)

    def test_appends_runtime_deployment_selector_frontier_evidence(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest, validation = self.write_fixture(root, module)
            package = root / "package"
            module.export_package(
                manifest_json=manifest,
                manifest_validation_json=validation,
                output_dir=package,
            )
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
                '"deployment_labels":[{"deployment_label":"nominal"}]}\n',
                encoding="utf-8",
            )
            (frontier_dir / "runtime_deployment_frontier.txt").write_text("status: PASS\n", encoding="utf-8")
            (frontier_dir / "runtime_deployment_frontier_labels.csv").write_text(
                "deployment_label,status,runtime_truth_used\nnominal,PASS,False\n",
                encoding="utf-8",
            )
            (frontier_dir / "runtime_deployment_frontier_profiles.csv").write_text(
                "profile_label,runtime_truth_used,runtime_inputs_truth_free\nactive,False,True\n",
                encoding="utf-8",
            )

            result = module.append_runtime_deployment_evidence(
                package_dir=package,
                selector_json=selector_json,
                frontier_dir=frontier_dir,
            )
            readme_text = (package / "DEPLOYMENT_README.txt").read_text(encoding="utf-8")

        roles = {row["role"] for row in result["files"]}
        self.assertEqual(result["runtime_deployment_frontier_status"], "PASS")
        self.assertIn("runtime_profile_label_selector", roles)
        self.assertIn("package_runtime_profile_label_selector", roles)
        self.assertIn("runtime_deployment_frontier_json", roles)
        self.assertIn("runtime_deployment_frontier_labels_csv", roles)
        self.assertIn("runtime_profile_label_selector.json", readme_text)
        self.assertIn("package_runtime_profile_label_selector.json", readme_text)

    def test_appends_package_portability_evidence_to_manifest(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest, validation = self.write_fixture(root, module)
            package = root / "package"
            module.export_package(
                manifest_json=manifest,
                manifest_validation_json=validation,
                output_dir=package,
            )
            portability_json = package / "evidence" / "package_portability_check.json"
            portability_txt = package / "evidence" / "package_portability_check.txt"
            portability_json.write_text(
                json.dumps(
                    {
                        "status": "PASS",
                        "selected_label": "packaged_active",
                        "copied_package_check_status": "PASS",
                        "copied_package_dir_ephemeral": True,
                        "copied_package_dir_note": "The copied package path is a temporary directory used only during this smoke check.",
                        "runtime_truth_used": False,
                        "coordinate_contract": "RDF(right,down,forward) FRD+range",
                    }
                ),
                encoding="utf-8",
            )
            portability_txt.write_text("status: PASS\n", encoding="utf-8")

            result = module.append_package_portability_evidence(
                package_dir=package,
                portability_json=portability_json,
                portability_txt=portability_txt,
            )
            readme_text = (package / "DEPLOYMENT_README.txt").read_text(encoding="utf-8")

        roles = {row["role"] for row in result["files"]}
        self.assertEqual(result["package_portability_status"], "PASS")
        self.assertEqual(result["package_portability_selected_label"], "packaged_active")
        self.assertIn("package_portability_check", roles)
        self.assertIn("package_portability_check_txt", roles)
        self.assertIn("evidence/package_portability_check.json", readme_text)

    def test_rejects_failed_package_portability_evidence(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest, validation = self.write_fixture(root, module)
            package = root / "package"
            module.export_package(
                manifest_json=manifest,
                manifest_validation_json=validation,
                output_dir=package,
            )
            portability_json = package / "evidence" / "package_portability_check.json"
            portability_json.write_text(
                json.dumps(
                    {
                        "status": "FAIL",
                        "selected_label": "packaged_active",
                        "copied_package_check_status": "PASS",
                        "copied_package_dir_ephemeral": True,
                        "copied_package_dir_note": "The copied package path is a temporary directory used only during this smoke check.",
                        "runtime_truth_used": False,
                        "coordinate_contract": "RDF(right,down,forward) FRD+range",
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaises(ValueError):
                module.append_package_portability_evidence(
                    package_dir=package,
                    portability_json=portability_json,
                )

    def test_rejects_failed_manifest_validation(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest, validation = self.write_fixture(root, module, validation_status="FAIL")

            with self.assertRaises(ValueError):
                module.export_package(
                    manifest_json=manifest,
                    manifest_validation_json=validation,
                    output_dir=root / "package",
                )


if __name__ == "__main__":
    unittest.main()
