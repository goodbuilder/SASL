from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
EXPORT_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_active_profile_deployment_package.py"
INSTALL_SCRIPT_PATH = ROOT_DIR / "scripts" / "install_stereodistance_active_profile_package.py"
CANDIDATE_EXPORT_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_deployable_profile_candidate_package.py"
CANDIDATE_PACKAGE_TEST_PATH = ROOT_DIR / "tests" / "test_deployable_profile_candidate_package.py"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class InstallStereoDistanceActiveProfilePackageTests(unittest.TestCase):
    def make_package(self, root: Path) -> Path:
        exporter = load_module("export_stereodistance_active_profile_deployment_package_for_install_test", EXPORT_SCRIPT_PATH)
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

    def make_candidate_package(self, root: Path, *, portability_status: str = "PASS") -> Path:
        package_fixture = load_module("candidate_package_fixture_for_install_test", CANDIDATE_PACKAGE_TEST_PATH)
        exporter = load_module("export_deployable_profile_candidate_package_for_install_test", CANDIDATE_EXPORT_SCRIPT_PATH)
        readiness = package_fixture.make_candidate_fixture(root)
        package = root / "candidate_package"
        exporter.export_package(candidate_readiness_json=readiness, output_dir=package)
        (package / "deployable_profile_candidate_package_portability.json").write_text(
            json.dumps(
                {
                    "status": portability_status,
                    "copied_operator_view_status": portability_status,
                    "runtime_truth_used": False,
                }
            ),
            encoding="utf-8",
        )
        return package

    def test_installs_validated_active_profile(self) -> None:
        installer = load_module("install_stereodistance_active_profile_package_under_test", INSTALL_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_package(root)
            output_profile = root / "runtime" / "active_runtime_profile.json"

            result = installer.install_package(package_dir=package, output_profile=output_profile)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["package_type"], "deployment_package")
        self.assertEqual(result["failure_count"], 0)
        self.assertEqual(result["package_check_status"], "PASS")
        self.assertTrue(result["installed_profile_sha256"])
        self.assertEqual(result["installed_profile_sha256"], result["expected_profile_sha256"])
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])
        self.assertIn("FRD+range", result["coordinate_contract"])

    def test_refuses_to_overwrite_without_force(self) -> None:
        installer = load_module("install_stereodistance_active_profile_package_under_test_overwrite", INSTALL_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_package(root)
            output_profile = root / "runtime" / "active_runtime_profile.json"
            output_profile.parent.mkdir(parents=True)
            output_profile.write_text('{"existing": true}\n', encoding="utf-8")

            result = installer.install_package(package_dir=package, output_profile=output_profile)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("output_profile:exists_without_force", result["failures"])

    def test_installs_candidate_package_active_profile(self) -> None:
        installer = load_module("install_stereodistance_active_profile_candidate_package_under_test", INSTALL_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_candidate_package(root)
            output_profile = root / "runtime" / "active_runtime_profile.json"

            result = installer.install_package(package_dir=package, output_profile=output_profile)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["package_type"], "candidate_package")
        self.assertEqual(result["package_check_status"], "PASS")
        self.assertEqual(result["portability_check_status"], "PASS")
        self.assertEqual(result["installed_profile_sha256"], result["expected_profile_sha256"])
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])

    def test_refuses_candidate_package_when_portability_failed(self) -> None:
        installer = load_module("install_stereodistance_active_profile_candidate_package_failed_portability", INSTALL_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_candidate_package(root, portability_status="FAIL")
            output_profile = root / "runtime" / "active_runtime_profile.json"

            result = installer.install_package(package_dir=package, output_profile=output_profile)

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["package_type"], "candidate_package")
        self.assertIn("portability_check:FAIL", result["failures"])


if __name__ == "__main__":
    unittest.main()
