from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
EXPORT_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_active_profile_deployment_package.py"
CHECK_SCRIPT_PATH = ROOT_DIR / "scripts" / "check_stereodistance_active_profile_deployment_package.py"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckStereoDistanceActiveProfileDeploymentPackageTests(unittest.TestCase):
    def make_package(self, root: Path):
        exporter = load_module("export_stereodistance_active_profile_deployment_package_for_check_test", EXPORT_SCRIPT_PATH)
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
            json.dumps(
                {
                    "overall_pass": True,
                    "matrix_item_count": 28,
                    "runtime_truth_used": False,
                    "stage_status": {"standalone_profile_promotion": True},
                    "matrix_rows": [
                        {
                            "stage": "single_run_output_audit",
                            "item": "rdf_axis_rmse_nonregression",
                            "status": "PASS",
                            "path": "method_comparison.csv",
                            "evidence_json": "{}",
                        }
                    ],
                }
            )
            + "\n",
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
        validation = root / "active_profile_deployment_manifest_validation.json"
        validation.write_text('{"status": "PASS", "failure_count": 0, "runtime_truth_used": false}\n', encoding="utf-8")
        package = root / "package"
        exporter.export_package(manifest_json=manifest, manifest_validation_json=validation, output_dir=package)
        return package

    def test_valid_package_passes(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test", CHECK_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            package = self.make_package(Path(tmpdir))

            result = checker.check_package(package)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)
        self.assertEqual(result["file_count"], 6)
        self.assertEqual(result["runtime_profile_load_status"], "PASS")
        self.assertEqual(result["node_loader_status"], "PASS")
        self.assertEqual(result["promotion_sample_apply_count"], 3)
        self.assertEqual(result["final_matrix_item_count"], 28)
        self.assertEqual(result["final_rdf_axis_nonregression_status"], "PASS")

    def test_package_with_runtime_frontier_evidence_passes(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test_frontier", CHECK_SCRIPT_PATH)
        exporter = load_module("export_stereodistance_active_profile_deployment_package_for_check_frontier_test", EXPORT_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_package(root)
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
                f'"deployment_labels":[{{"deployment_label":"nominal","selected_profile":"{package / "active_runtime_profile.json"}"}}]}}\n',
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
            exporter.append_runtime_deployment_evidence(
                package_dir=package,
                selector_json=selector_json,
                frontier_dir=frontier_dir,
            )

            result = checker.check_package(package)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["runtime_deployment_frontier_profile_count"], 1)
        self.assertEqual(result["runtime_deployment_frontier_label_count"], 1)
        self.assertEqual(result["package_runtime_selector_supported_label_count"], 1)

    def test_package_with_portability_evidence_passes(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test_portability", CHECK_SCRIPT_PATH)
        exporter = load_module("export_stereodistance_active_profile_deployment_package_for_check_portability_test", EXPORT_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_package(root)
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
                f'"deployment_labels":[{{"deployment_label":"nominal","selected_profile":"{package / "active_runtime_profile.json"}"}}]}}\n',
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
            exporter.append_runtime_deployment_evidence(
                package_dir=package,
                selector_json=selector_json,
                frontier_dir=frontier_dir,
            )
            (package / "evidence" / "package_portability_check.json").write_text(
                json.dumps(
                    {
                        "status": "PASS",
                        "selected_label": "packaged_active",
                        "copied_package_check_status": "PASS",
                        "copied_package_dir_ephemeral": True,
                        "copied_package_dir_note": "The copied package path is a temporary directory used only during this smoke check.",
                        "runtime_truth_used": False,
                        "coordinate_contract": (
                            "portable selector consumes stereo-center RDF(right,down,forward); "
                            "public output remains FRD+range"
                        ),
                    }
                ),
                encoding="utf-8",
            )
            exporter.append_package_portability_evidence(
                package_dir=package,
                portability_json=package / "evidence" / "package_portability_check.json",
            )

            result = checker.check_package(package)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["package_portability_status"], "PASS")
        self.assertEqual(result["package_portability_selected_label"], "packaged_active")
        self.assertEqual(result["package_portability_copied_package_check_status"], "PASS")

    def test_package_portability_failure_fails_if_evidence_is_present(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test_bad_portability", CHECK_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            package = self.make_package(Path(tmpdir))
            portability_path = package / "evidence" / "package_portability_check.json"
            portability_path.write_text(
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
            manifest = json.loads((package / "deployment_package_manifest.json").read_text(encoding="utf-8"))
            manifest["files"].append(
                {
                    "role": "package_portability_check",
                    "path": "evidence/package_portability_check.json",
                    "size_bytes": portability_path.stat().st_size,
                    "sha256": checker.sha256_file(portability_path),
                }
            )
            manifest["file_count"] = len(manifest["files"])
            (package / "deployment_package_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

            result = checker.check_package(package)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("package_portability:status", result["failures"])

    def test_package_portability_manifest_mismatch_fails(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test_portability_manifest_mismatch", CHECK_SCRIPT_PATH)
        exporter = load_module("export_stereodistance_active_profile_deployment_package_for_check_portability_mismatch_test", EXPORT_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            package = self.make_package(Path(tmpdir))
            portability_path = package / "evidence" / "package_portability_check.json"
            portability_path.write_text(
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
            exporter.append_package_portability_evidence(
                package_dir=package,
                portability_json=portability_path,
            )
            manifest_path = package / "deployment_package_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["package_portability_selected_label"] = "wrong_label"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

            result = checker.check_package(package)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("package_portability_manifest:selected_label", result["failures"])

    def test_package_runtime_selector_bad_label_map_fails(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test_bad_package_selector", CHECK_SCRIPT_PATH)
        exporter = load_module("export_stereodistance_active_profile_deployment_package_for_check_bad_selector_test", EXPORT_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_package(root)
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
                f'"deployment_labels":[{{"deployment_label":"nominal","selected_profile":"{package / "active_runtime_profile.json"}"}}]}}\n',
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
            exporter.append_runtime_deployment_evidence(
                package_dir=package,
                selector_json=selector_json,
                frontier_dir=frontier_dir,
            )
            package_selector_path = package / "evidence" / "package_runtime_profile_label_selector.json"
            package_selector = json.loads(package_selector_path.read_text(encoding="utf-8"))
            package_selector["label_profile_map"]["nominal"] = "wrong_profile"
            package_selector_path.write_text(json.dumps(package_selector), encoding="utf-8")
            manifest = json.loads((package / "deployment_package_manifest.json").read_text(encoding="utf-8"))
            for row in manifest["files"]:
                if row["role"] == "package_runtime_profile_label_selector":
                    row["sha256"] = checker.sha256_file(package_selector_path)
                    row["size_bytes"] = package_selector_path.stat().st_size
            (package / "deployment_package_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

            result = checker.check_package(package)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("package_runtime_selector:label:nominal", result["failures"])

    def test_modified_file_fails_checksum(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test_checksum", CHECK_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            package = self.make_package(Path(tmpdir))
            package.joinpath("active_runtime_profile.json").write_text('{"changed": true}\n', encoding="utf-8")

            result = checker.check_package(package)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("files:active_profile:sha256", result["failures"])
        self.assertIn("active_profile:sha256", result["failures"])

    def test_final_acceptance_failure_fails_semantic_check(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test_final", CHECK_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            package = self.make_package(Path(tmpdir))
            final_path = package / "evidence" / "rtk_stereo_acceptance_matrix_summary.json"
            final_path.write_text(
                '{"overall_pass": false, "matrix_item_count": 28, "runtime_truth_used": false, '
                '"stage_status": {"standalone_profile_promotion": true}}\n',
                encoding="utf-8",
            )
            manifest = json.loads((package / "deployment_package_manifest.json").read_text(encoding="utf-8"))
            for row in manifest["files"]:
                if row["role"] == "final_acceptance_summary":
                    row["sha256"] = checker.sha256_file(final_path)
                    row["size_bytes"] = final_path.stat().st_size
            (package / "deployment_package_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

            result = checker.check_package(package)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("final_acceptance:overall_pass", result["failures"])

    def test_final_acceptance_missing_axis_guard_fails_semantic_check(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test_axis_guard", CHECK_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            package = self.make_package(Path(tmpdir))
            final_path = package / "evidence" / "rtk_stereo_acceptance_matrix_summary.json"
            final_payload = json.loads(final_path.read_text(encoding="utf-8"))
            final_payload["matrix_rows"] = []
            final_path.write_text(json.dumps(final_payload), encoding="utf-8")
            manifest = json.loads((package / "deployment_package_manifest.json").read_text(encoding="utf-8"))
            for row in manifest["files"]:
                if row["role"] == "final_acceptance_summary":
                    row["sha256"] = checker.sha256_file(final_path)
                    row["size_bytes"] = final_path.stat().st_size
            (package / "deployment_package_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

            result = checker.check_package(package)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("final_acceptance:rdf_axis_rmse_nonregression", result["failures"])
        self.assertEqual(result["final_rdf_axis_nonregression_status"], "FAIL")

    def test_package_manifest_axis_guard_status_mismatch_fails(self) -> None:
        checker = load_module("check_stereodistance_active_profile_deployment_package_under_test_axis_manifest", CHECK_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            package = self.make_package(Path(tmpdir))
            manifest_path = package / "deployment_package_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["final_rdf_axis_nonregression_status"] = "FAIL"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

            result = checker.check_package(package)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("package_manifest:final_rdf_axis_nonregression_status", result["failures"])


if __name__ == "__main__":
    unittest.main()
