from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
EXPORT_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_active_profile_deployment_package.py"
INSTALL_SCRIPT_PATH = ROOT_DIR / "scripts" / "install_stereodistance_active_profile_package.py"
READINESS_SCRIPT_PATH = ROOT_DIR / "scripts" / "check_rtk_stereo_deployment_readiness.py"
PUBLIC_PAYLOAD_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_public_topic_payload_from_profile.py"
INVENTORY_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_deployment_package_inventory.py"
OPERATOR_LAUNCH_SCRIPT_PATH = ROOT_DIR / "scripts" / "check_stereodistance_operator_view_launch.py"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckRtkStereoDeploymentReadinessTests(unittest.TestCase):
    def make_readiness_fixture(self, root: Path) -> tuple[Path, Path]:
        exporter = load_module("export_stereodistance_active_profile_deployment_package_for_readiness_test", EXPORT_SCRIPT_PATH)
        installer = load_module("install_stereodistance_active_profile_package_for_readiness_test", INSTALL_SCRIPT_PATH)
        public_payload_exporter = load_module(
            "export_stereodistance_public_topic_payload_from_profile_for_readiness_test",
            PUBLIC_PAYLOAD_SCRIPT_PATH,
        )
        inventory_exporter = load_module("export_stereodistance_deployment_package_inventory_for_readiness_test", INVENTORY_SCRIPT_PATH)
        operator_launch_checker = load_module(
            "check_stereodistance_operator_view_launch_for_readiness_test",
            OPERATOR_LAUNCH_SCRIPT_PATH,
        )
        active_profile = root / "active_runtime_profile.json"
        active_profile.write_text(
            json.dumps(
                {
                    "profile_id": "unit",
                    "method": "sim3_binned_residual_disparity_bins11",
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
            '{"pass_promotion_check": true, "node_loader_ok": true, '
            '"axis_contract_declared": true, "runtime_truth_used": false, '
            '"sample_apply_count": 3, "sample_invalid_count": 0}\n',
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
        active_sha = exporter.sha256_file(active_profile)
        manifest.write_text(
            json.dumps(
                {
                    "mode": "stereodistance_active_profile_deployment_manifest",
                    "status": "PASS",
                    "active_profile": str(active_profile),
                    "active_profile_sha256": active_sha,
                    "profile_id": "unit",
                    "method": "sim3_binned_residual_disparity_bins11",
                    "profile_mode": "estimated_to_truth_sim3",
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                    "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
                    "promotion_check_json": str(promotion),
                    "promotion_pass": True,
                    "promotion_sample_apply_count": 3,
                    "promotion_sample_invalid_count": 0,
                    "final_acceptance_summary": str(final),
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
        validation = root / "active_profile_deployment_manifest_validation.json"
        validation.write_text('{"status": "PASS", "failure_count": 0, "runtime_truth_used": false}\n', encoding="utf-8")
        package = root / "deployment_package"
        exporter.export_package(manifest_json=manifest, manifest_validation_json=validation, output_dir=package)
        package_check = root / "deployment_package" / "deployment_package_check.json"
        readiness = load_module("check_rtk_stereo_deployment_readiness_for_fixture_package_check", READINESS_SCRIPT_PATH)
        package_check.write_text(json.dumps(readiness.package_checker.check_package(package)), encoding="utf-8")
        package_inventory = inventory_exporter.export_inventory(package, package / "inventory")
        operator_launch_check = operator_launch_checker.check_operator_view_launch(package_dir=package)
        operator_launch_json = root / "operator_view_launch_check.json"
        operator_launch_json.write_text(json.dumps(operator_launch_check), encoding="utf-8")
        operator_selector_launch_json = root / "operator_view_launch_check_selector.json"
        operator_selector_launch_json.write_text(
            json.dumps(
                {
                    **operator_launch_check,
                    "source_mode": "selector",
                    "selected_label": "stress_active",
                    "label_selection_status": "PASS",
                }
            ),
            encoding="utf-8",
        )
        frontier_dir = root / "runtime_deployment_frontier"
        frontier_dir.mkdir()
        frontier_json = frontier_dir / "runtime_deployment_frontier.json"
        frontier_labels_csv = frontier_dir / "runtime_deployment_frontier_labels.csv"
        frontier_profiles_csv = frontier_dir / "runtime_deployment_frontier_profiles.csv"
        frontier_json.write_text(
            json.dumps(
                {
                    "summary": {
                        "status": "PASS",
                        "profile_count": 1,
                        "deployment_label_count": 2,
                        "runtime_truth_used": False,
                        "truth_free_runtime_selection": True,
                        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                    },
                    "profiles": [{"profile_label": "stress_active"}],
                    "deployment_labels": [{"deployment_label": "runtime_observation_stress"}],
                }
            ),
            encoding="utf-8",
        )
        frontier_labels_csv.write_text(
            "deployment_label,status,runtime_truth_used\nruntime_observation_stress,PASS,False\n",
            encoding="utf-8",
        )
        frontier_profiles_csv.write_text(
            "profile_label,runtime_truth_used,runtime_inputs_truth_free\nstress_active,False,True\n",
            encoding="utf-8",
        )
        portability_json = package / "evidence" / "package_portability_check.json"
        portability_txt = package / "evidence" / "package_portability_check.txt"
        portability_json.write_text(
            json.dumps(
                {
                    "mode": "stereodistance_deployment_package_portability_check",
                    "status": "PASS",
                    "selected_label": "packaged_active",
                    "copied_package_check_status": "PASS",
                    "copied_package_file_count": 15,
                    "copied_package_dir_ephemeral": True,
                    "copied_package_dir_note": "The copied package path is a temporary directory used only during this smoke check.",
                    "runtime_truth_used": False,
                    "coordinate_contract": (
                        "portable package selector consumes deployment/scenario labels only; selected runtime profile "
                        "consumes stereo-center RDF(right,down,forward) plus measured disparity; public output remains FRD+range"
                    ),
                    "failure_count": 0,
                    "failures": [],
                }
            ),
            encoding="utf-8",
        )
        portability_txt.write_text("status: PASS\n", encoding="utf-8")
        exporter.append_package_portability_evidence(
            package_dir=package,
            portability_json=portability_json,
            portability_txt=portability_txt,
        )
        package_check.write_text(json.dumps(readiness.package_checker.check_package(package)), encoding="utf-8")
        package_inventory = inventory_exporter.export_inventory(package, package / "inventory")
        install_profile = root / "install_rehearsal" / "active_runtime_profile.json"
        install_manifest = root / "install_rehearsal" / "install_manifest.json"
        install = installer.install_package(package_dir=package, output_profile=install_profile)
        install_manifest.parent.mkdir(parents=True, exist_ok=True)
        install_manifest.write_text(json.dumps(install), encoding="utf-8")
        samples = root / "samples.csv"
        samples.write_text(
            "\n".join(
                [
                    "split,raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px,left_u_px,left_v_px",
                    "validation,10.0,20.0,30.0,5.0,640.0,360.0",
                ]
            )
            + "\n",
            encoding="utf-8",
        )
        public_payload_dir = root / "public_topic_payload"
        public_payload = public_payload_exporter.export_payloads(
            profile_json=active_profile,
            sample_csv=samples,
            output_dir=public_payload_dir,
            split="validation",
            max_samples=5,
        )
        summary = root / "rtk_stereo_acceptance_dashboard_with_promotion_summary.json"
        summary.write_text(
            json.dumps(
                {
                    "status": "PASS",
                    "output_active_profile": str(active_profile),
                    "promotion_pass": True,
                    "promotion_sample_apply_count": 3,
                    "promotion_sample_invalid_count": 0,
                    "active_profile_hash_stable": True,
                    "active_profile_sha256_after_final_dashboard": active_sha,
                    "final_overall_pass": True,
                    "final_matrix_item_count": 28,
                    "final_rdf_axis_nonregression_status": "PASS",
                    "runtime_truth_used": False,
                    "coordinate_contracts_explicit": True,
                    "deployment_manifest_json": str(manifest),
                    "deployment_manifest_validation_status": "PASS",
                    "deployment_package_dir": str(package),
                    "deployment_package_manifest_json": str(package / "deployment_package_manifest.json"),
                    "deployment_package_check_status": "PASS",
                    "deployment_package_check_json": str(package_check),
                    "deployment_package_inventory_status": package_inventory["status"],
                    "deployment_package_inventory_json": str(package / "inventory" / "stereodistance_deployment_package_inventory.json"),
                    "deployment_package_inventory_file_count": package_inventory["file_count"],
                    "operator_view_launch_check_status": operator_launch_check["status"],
                    "operator_view_launch_check_json": str(operator_launch_json),
                    "operator_view_selector_launch_check_status": "PASS",
                    "operator_view_selector_launch_check_json": str(operator_selector_launch_json),
                    "operator_view_selector_selected_label": "stress_active",
                    "runtime_deployment_frontier_status": "PASS",
                    "runtime_deployment_frontier_json": str(frontier_json),
                    "runtime_deployment_frontier_labels_csv": str(frontier_labels_csv),
                    "runtime_deployment_frontier_profiles_csv": str(frontier_profiles_csv),
                    "deployment_package_portability_status": "PASS",
                    "deployment_package_portability_json": str(portability_json),
                    "deployment_package_portability_txt": str(portability_txt),
                    "deployment_package_portability_selected_label": "packaged_active",
                    "deployment_package_portability_copied_package_check_status": "PASS",
                    "install_rehearsal_status": "PASS",
                    "install_rehearsal_profile": str(install_profile),
                    "install_rehearsal_json": str(install_manifest),
                    "install_rehearsal_sha256": active_sha,
                    "public_topic_payload_status": public_payload["status"],
                    "public_topic_payload_node_equivalence_status": public_payload[
                        "node_public_payload_equivalence_status"
                    ],
                    "public_topic_payload_json": str(public_payload_dir / "public_corrected_position_payload_summary.json"),
                    "public_topic_payload_csv": str(public_payload_dir / "public_corrected_position_payload.csv"),
                }
            ),
            encoding="utf-8",
        )
        return summary, install_profile

    def write_matrix_active_profile_gate(self, path: Path, summary: Path, *, status: str = "PASS") -> Path:
        summary_payload = json.loads(summary.read_text(encoding="utf-8"))
        active_profile = Path(summary_payload["output_active_profile"])
        active_sha = summary_payload["active_profile_sha256_after_final_dashboard"]
        active_payload = json.loads(active_profile.read_text(encoding="utf-8"))
        method = active_payload["method"]
        path.write_text(
            json.dumps(
                {
                    "status": status,
                    "recommendation": "keep_active_profile" if status == "PASS" else "do_not_promote_without_review",
                    "active_profile_sha256": active_sha,
                    "matrix_selected_method": method,
                    "active_profile_method": method,
                    "active_matches_matrix_selected_method": status == "PASS",
                    "active_method_pass_all_runs": status == "PASS",
                    "active_method_pass_count": 6,
                    "active_method_expected_run_count": 6,
                    "active_profile_candidate_rows_found": True,
                    "active_profile_candidate_pass_count": 6,
                    "active_profile_candidate_expected_run_count": 6,
                    "active_profile_candidate_pass_all_runs": True,
                    "active_method_validation_range_rmse_corrected_mean_m": 1.5,
                    "active_method_validation_range_rmse_corrected_max_m": 2.2,
                    "active_method_target_max_abs_error_corrected_max_m": 4.0,
                    "active_profile_runtime_truth_free": True,
                    "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                    "public_topic_contract": "public StereoDistance topic publishes FRD+range",
                    "runtime_input_contract": (
                        "active profile matrix gate reads offline common-CSV frontier evidence only; runtime apply "
                        "consumes stereo-center RDF(right,down,forward) plus measured disparity and publishes FRD+range"
                    ),
                    "runtime_truth_used": False,
                }
            ),
            encoding="utf-8",
        )
        return path

    def test_valid_readiness_evidence_passes(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)
        self.assertEqual(result["method"], "sim3_binned_residual_disparity_bins11")
        self.assertEqual(result["deployment_manifest_validation_status"], "PASS")
        self.assertEqual(result["deployment_package_check_status"], "PASS")
        self.assertEqual(result["deployment_package_inventory_status"], "PASS")
        self.assertGreater(int(result["deployment_package_inventory_file_count"]), 0)
        self.assertEqual(result["operator_view_launch_check_status"], "PASS")
        self.assertEqual(result["operator_view_selector_launch_check_status"], "PASS")
        self.assertEqual(result["operator_view_selector_selected_label"], "stress_active")
        self.assertEqual(result["runtime_deployment_frontier_status"], "PASS")
        self.assertEqual(result["runtime_deployment_frontier_profile_count"], 1)
        self.assertEqual(result["runtime_deployment_frontier_label_count"], 2)
        self.assertEqual(result["deployment_package_portability_status"], "PASS")
        self.assertEqual(result["deployment_package_portability_selected_label"], "packaged_active")
        self.assertEqual(result["deployment_package_portability_copied_package_check_status"], "PASS")
        self.assertEqual(result["final_rdf_axis_nonregression_status"], "PASS")
        self.assertEqual(result["install_rehearsal_status"], "PASS")
        self.assertEqual(result["public_topic_payload_status"], "PASS")
        self.assertEqual(result["public_topic_payload_sample_count"], 1)
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])
        self.assertIn("FRD+range", result["coordinate_contract"])

    def test_matrix_active_profile_gate_pass_is_reported_when_provided(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_matrix_gate", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            summary, _install_profile = self.make_readiness_fixture(root)
            gate = self.write_matrix_active_profile_gate(root / "active_profile_matrix_gate.json", summary)

            result = module.check_readiness(summary, gate)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["matrix_active_profile_gate_status"], "PASS")
        self.assertEqual(result["matrix_active_profile_gate_recommendation"], "keep_active_profile")
        self.assertEqual(result["matrix_active_profile_selected_method"], "sim3_binned_residual_disparity_bins11")
        self.assertEqual(result["matrix_active_profile_pass_count"], 6)
        self.assertEqual(result["matrix_active_profile_expected_run_count"], 6)
        self.assertEqual(result["matrix_active_profile_candidate_pass_count"], 6)
        self.assertEqual(result["matrix_active_profile_candidate_expected_run_count"], 6)
        self.assertTrue(result["matrix_active_profile_candidate_pass_all_runs"])

    def test_matrix_active_profile_gate_failure_fails_readiness_when_provided(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_matrix_gate", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            summary, _install_profile = self.make_readiness_fixture(root)
            gate = self.write_matrix_active_profile_gate(root / "active_profile_matrix_gate.json", summary)
            gate_payload = json.loads(gate.read_text(encoding="utf-8"))
            gate_payload["matrix_selected_method"] = "different_method"
            gate.write_text(json.dumps(gate_payload), encoding="utf-8")

            result = module.check_readiness(summary, gate)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("matrix_active_profile_gate:matrix_selected_method", result["failures"])

    def test_matrix_active_profile_candidate_failure_fails_readiness_when_provided(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_matrix_candidate", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            summary, _install_profile = self.make_readiness_fixture(root)
            gate = self.write_matrix_active_profile_gate(root / "active_profile_matrix_gate.json", summary)
            gate_payload = json.loads(gate.read_text(encoding="utf-8"))
            gate_payload["active_profile_candidate_pass_all_runs"] = False
            gate_payload["active_profile_candidate_pass_count"] = 2
            gate.write_text(json.dumps(gate_payload), encoding="utf-8")

            result = module.check_readiness(summary, gate)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("matrix_active_profile_gate:active_profile_candidate_pass_all_runs", result["failures"])

    def test_modified_install_profile_fails_sha_check(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_install", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, install_profile = self.make_readiness_fixture(Path(tmpdir))
            install_profile.write_text('{"changed": true}\n', encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("install_profile:sha256", result["failures"])

    def test_public_payload_axis_order_failure_fails(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_public_payload", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            public_csv = Path(summary_payload["public_topic_payload_csv"])
            with public_csv.open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            rows[0]["data_axis_order"] = "right_m,down_m,forward_m,range_m"
            with public_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("public_topic_payload_csv:axis_order", result["failures"])

    def test_missing_wrapper_axis_guard_status_fails(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_missing_axis_guard", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            summary_payload.pop("final_rdf_axis_nonregression_status")
            summary.write_text(json.dumps(summary_payload), encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("wrapper_summary:final_rdf_axis_nonregression_status", result["failures"])

    def test_selector_launch_failure_fails_when_selector_evidence_is_present(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_selector", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            selector_json = Path(summary_payload["operator_view_selector_launch_check_json"])
            selector_payload = json.loads(selector_json.read_text(encoding="utf-8"))
            selector_payload["status"] = "FAIL"
            selector_payload["failures"] = ["unit_failure"]
            selector_json.write_text(json.dumps(selector_payload), encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("operator_view_selector_launch:status", result["failures"])

    def test_selector_launch_profile_sha_mismatch_fails(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_selector_sha", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            selector_json = Path(summary_payload["operator_view_selector_launch_check_json"])
            selector_payload = json.loads(selector_json.read_text(encoding="utf-8"))
            selector_payload["resolved_runtime_profile_sha256"] = "0" * 64
            selector_json.write_text(json.dumps(selector_payload), encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("operator_view_selector_launch:profile_sha256", result["failures"])

    def test_frontier_failure_fails_when_frontier_evidence_is_present(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_frontier", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            frontier_json = Path(summary_payload["runtime_deployment_frontier_json"])
            frontier_payload = json.loads(frontier_json.read_text(encoding="utf-8"))
            frontier_payload["summary"]["status"] = "FAIL"
            frontier_json.write_text(json.dumps(frontier_payload), encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("runtime_deployment_frontier:status", result["failures"])

    def test_portability_failure_fails_when_portability_evidence_is_present(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_portability", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            portability_json = Path(summary_payload["deployment_package_portability_json"])
            portability_payload = json.loads(portability_json.read_text(encoding="utf-8"))
            portability_payload["status"] = "FAIL"
            portability_payload["failures"] = ["unit_failure"]
            portability_json.write_text(json.dumps(portability_payload), encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("deployment_package_portability:status", result["failures"])

    def test_package_check_portability_mismatch_fails(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_package_portability", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            package_check_json = Path(summary_payload["deployment_package_check_json"])
            package_check_payload = json.loads(package_check_json.read_text(encoding="utf-8"))
            package_check_payload["package_portability_selected_label"] = "wrong_label"
            package_check_json.write_text(json.dumps(package_check_payload), encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("deployment_package_check_file:package_portability_selected_label", result["failures"])

    def test_package_check_axis_guard_mismatch_fails(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_package_axis_guard", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            package_check_json = Path(summary_payload["deployment_package_check_json"])
            package_check_payload = json.loads(package_check_json.read_text(encoding="utf-8"))
            package_check_payload["final_rdf_axis_nonregression_status"] = "FAIL"
            package_check_json.write_text(json.dumps(package_check_payload), encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("deployment_package_check_file:final_rdf_axis_nonregression_status", result["failures"])

    def test_package_manifest_axis_guard_mismatch_fails(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_package_manifest_axis", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            package_manifest_json = Path(summary_payload["deployment_package_manifest_json"])
            package_manifest = json.loads(package_manifest_json.read_text(encoding="utf-8"))
            package_manifest["final_rdf_axis_nonregression_status"] = "FAIL"
            package_manifest_json.write_text(json.dumps(package_manifest), encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("deployment_package_manifest:final_rdf_axis_nonregression_status", result["failures"])

    def test_summary_portability_mismatch_fails(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_bad_summary_portability", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            summary, _install_profile = self.make_readiness_fixture(Path(tmpdir))
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            summary_payload["deployment_package_portability_selected_label"] = "wrong_label"
            summary.write_text(json.dumps(summary_payload), encoding="utf-8")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("wrapper_summary:deployment_package_portability_selected_label_match", result["failures"])

    def test_run_writes_json_txt_and_csv(self) -> None:
        module = load_module("check_rtk_stereo_deployment_readiness_under_test_outputs", READINESS_SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            summary, _install_profile = self.make_readiness_fixture(root)
            output_json = root / "readiness.json"
            output_txt = root / "readiness.txt"
            output_csv = root / "readiness.csv"

            result = module.run(
                [
                    "--wrapper-summary-json",
                    str(summary),
                    "--matrix-active-profile-gate-json",
                    str(self.write_matrix_active_profile_gate(root / "active_profile_matrix_gate.json", summary)),
                    "--output-json",
                    str(output_json),
                    "--output-txt",
                    str(output_txt),
                    "--output-csv",
                    str(output_csv),
                ]
            )
            with output_csv.open(encoding="utf-8") as handle:
                csv_rows = list(csv.DictReader(handle))
            output_json_exists = output_json.is_file()
            output_txt_exists = output_txt.is_file()
            output_csv_exists = output_csv.is_file()

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(output_json_exists)
        self.assertTrue(output_txt_exists)
        self.assertTrue(output_csv_exists)
        self.assertEqual(len(csv_rows), 1)
        self.assertEqual(csv_rows[0]["status"], "PASS")
        self.assertEqual(csv_rows[0]["method"], "sim3_binned_residual_disparity_bins11")
        self.assertEqual(csv_rows[0]["matrix_active_profile_gate_status"], "PASS")
        self.assertEqual(csv_rows[0]["runtime_truth_used"], "False")
        self.assertIn("RDF(right,down,forward)", csv_rows[0]["coordinate_contract"])


if __name__ == "__main__":
    unittest.main()
