from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_rtk_stereo_acceptance_dashboard_with_promotion.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "run_rtk_stereo_acceptance_dashboard_with_promotion_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load run_rtk_stereo_acceptance_dashboard_with_promotion.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunRtkStereoAcceptanceDashboardWithPromotionTests(unittest.TestCase):
    def test_wrapper_exports_profile_promotes_then_rebuilds_final_dashboard(self) -> None:
        module = load_module()
        calls: list[str] = []

        def fake_acceptance_run(**kwargs):
            calls.append("acceptance_with_promotion" if kwargs.get("promotion_check_json") else "acceptance_initial")
            Path(kwargs["output_active_profile"]).parent.mkdir(parents=True, exist_ok=True)
            Path(kwargs["output_active_profile"]).write_text(
                '{"profile_id":"unit","method":"sim3","mode":"estimated_to_truth_sim3",'
                '"axis_contract":"RDF(right,down,forward)",'
                '"runtime_inputs":["raw_right_m","raw_down_m","raw_forward_m","raw_disparity_px"],'
                '"forbidden_runtime_inputs":["calibration_truth","px4_rtk_truth"],'
                '"scale":1.0,'
                '"rotation_matrix_rdf":[[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],'
                '"translation_rdf_m":[0.0,0.0,0.0]}\n',
                encoding="utf-8",
            )
            Path(kwargs["output_dir"]).mkdir(parents=True, exist_ok=True)
            matrix_item_count = 29 if kwargs.get("promotion_check_json") else 27
            Path(kwargs["output_dir"]).joinpath("rtk_stereo_acceptance_matrix_summary.json").write_text(
                json.dumps(
                    {
                        "overall_pass": True,
                        "matrix_item_count": matrix_item_count,
                        "runtime_truth_used": False,
                        "stage_status": {"standalone_profile_promotion": bool(kwargs.get("promotion_check_json"))},
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
            return {
                "overall_pass": True,
                "matrix_item_count": matrix_item_count,
                "stage_status": {"standalone_profile_promotion": bool(kwargs.get("promotion_check_json"))},
                "matrix_rows": [
                    {
                        "stage": "single_run_output_audit",
                        "item": "rdf_axis_rmse_nonregression",
                        "status": "PASS",
                    }
                ],
            }

        def fake_promotion_run_check(profile_json, output_dir, *, sample_csv=None, split="validation", max_samples=50):
            calls.append("promotion")
            output_dir.mkdir(parents=True, exist_ok=True)
            output_dir.joinpath("promotion_check.json").write_text(
                '{"pass_promotion_check": true, "node_loader_ok": true, '
                '"axis_contract_declared": true, "runtime_truth_used": false, '
                '"sample_apply_count": 3, "sample_invalid_count": 0}\n',
                encoding="utf-8",
            )
            return {
                "pass_promotion_check": True,
                "sample_apply_count": 3,
                "sample_invalid_count": 0,
            }

        def fake_install_package(*, package_dir, output_profile, force=False):
            calls.append("install_rehearsal")
            output_profile.parent.mkdir(parents=True, exist_ok=True)
            output_profile.write_text('{"profile_id":"unit"}\n', encoding="utf-8")
            return {
                "status": "PASS",
                "installed_profile": str(output_profile),
                "installed_profile_sha256": "abc123",
                "expected_profile_sha256": "abc123",
                "runtime_truth_used": False,
                "coordinate_contract": "installed profile consumes stereo-center RDF(right,down,forward); public StereoDistance alias is FRD+range",
                "failure_count": 0,
                "failures": [],
            }

        def fake_readiness_check(wrapper_summary_json, matrix_active_profile_gate_json=None):
            calls.append("deployment_readiness")
            self.assertIsNotNone(matrix_active_profile_gate_json)
            return {
                "status": "PASS",
                "method": "sim3",
                "profile_mode": "estimated_to_truth_sim3",
                "active_profile_sha256": "abc123",
                "matrix_active_profile_gate_status": "PASS",
                "matrix_active_profile_gate_recommendation": "keep_active_profile",
                "deployment_manifest_validation_status": "PASS",
                "deployment_package_check_status": "PASS",
                "install_rehearsal_status": "PASS",
                "promotion_sample_apply_count": 3,
                "promotion_sample_invalid_count": 0,
                "final_matrix_item_count": 28,
                "runtime_truth_used": False,
                "coordinate_contract": "RDF(right,down,forward) FRD+range truth is calibration/evaluation only",
                "failure_count": 0,
                "failures": [],
            }

        def fake_public_payload_export(*, profile_json, sample_csv, output_dir, split="validation", max_samples=50):
            calls.append("public_payload")
            output_dir.mkdir(parents=True, exist_ok=True)
            output_dir.joinpath("public_corrected_position_payload.csv").write_text(
                "topic,data_axis_order,data,runtime_truth_used,node_public_payload_equivalent\n"
                '"/stereo/corrected_position","forward_m,right_m,down_m,range_m","[1.0,2.0,3.0,4.0]",False,True\n',
                encoding="utf-8",
            )
            output_dir.joinpath("public_corrected_position_payload_summary.json").write_text(
                '{"status": "PASS", "sample_count": 1, "invalid_count": 0, '
                '"node_public_payload_equivalence_status": "PASS", '
                '"runtime_truth_used": false, '
                '"input_coordinate_contract": "runtime_input:stereo_center_RDF(right,down,forward)", '
                '"public_topic_contract": "std_msgs/Float32MultiArray:data=[forward_m,right_m,down_m,range_m]"}\n',
                encoding="utf-8",
            )
            return {
                "status": "PASS",
                "sample_count": 1,
                "invalid_count": 0,
                "node_public_payload_equivalence_status": "PASS",
                "runtime_truth_used": False,
                "input_coordinate_contract": "runtime_input:stereo_center_RDF(right,down,forward)",
                "public_topic_contract": "std_msgs/Float32MultiArray:data=[forward_m,right_m,down_m,range_m]",
                "payload_csv": str(output_dir / "public_corrected_position_payload.csv"),
            }

        def fake_package_inventory(package_dir, output_dir):
            calls.append("package_inventory")
            output_dir.mkdir(parents=True, exist_ok=True)
            output_dir.joinpath("stereodistance_deployment_package_inventory.json").write_text(
                '{"status": "PASS", "file_count": 13, "runtime_truth_used": false, "files": []}\n',
                encoding="utf-8",
            )
            output_dir.joinpath("stereodistance_deployment_package_inventory.csv").write_text(
                "relative_path,size_bytes,sha256\n",
                encoding="utf-8",
            )
            output_dir.joinpath("stereodistance_deployment_package_inventory.txt").write_text("status: PASS\n", encoding="utf-8")
            return {"status": "PASS", "file_count": 13, "runtime_truth_used": False}

        def fake_operator_view_launch_check(
            *,
            package_dir=None,
            profile_json=None,
            selector_json=None,
            deployment_label="",
            scenario_label="",
            scenario_name="",
        ):
            if selector_json is not None:
                calls.append("operator_view_selector_launch")
                if "package_runtime_profile_label_selector" in str(selector_json):
                    return {
                        "status": "PASS",
                        "source_mode": "selector",
                        "selected_label": "packaged_active",
                        "resolved_runtime_profile": str(root / "out" / "deployment_package" / "active_runtime_profile.json"),
                        "resolved_runtime_profile_sha256": "abc123",
                        "runtime_truth_used": False,
                        "coordinate_contract": "RDF(right,down,forward) FRD+range",
                        "failure_count": 0,
                        "failures": [],
                    }
                return {
                    "status": "PASS",
                    "source_mode": "selector",
                    "selected_label": "stress_active",
                    "resolved_runtime_profile": str(root / "stress_package" / "active_runtime_profile.json"),
                    "runtime_truth_used": False,
                    "coordinate_contract": "RDF(right,down,forward) FRD+range",
                    "failure_count": 0,
                    "failures": [],
                }
            calls.append("operator_view_package_launch")
            return {
                "status": "PASS",
                "source_mode": "package",
                "resolved_runtime_profile": str(Path(package_dir) / "active_runtime_profile.json"),
                "runtime_truth_used": False,
                "coordinate_contract": "RDF(right,down,forward) FRD+range",
                "failure_count": 0,
                "failures": [],
            }

        def fake_runtime_frontier_export(*, selector_json, output_dir):
            calls.append("runtime_frontier")
            output_dir.mkdir(parents=True, exist_ok=True)
            output_dir.joinpath("runtime_deployment_frontier.json").write_text(
                '{"summary":{"status":"PASS","profile_count":1,"deployment_label_count":1,'
                '"runtime_truth_used":false,"truth_free_runtime_selection":true,'
                '"axis_contract":"RDF(right,down,forward)"},'
                '"profiles":[{"profile_label":"stress_active"}],'
                '"deployment_labels":[{"deployment_label":"runtime_observation_stress"}]}\n',
                encoding="utf-8",
            )
            output_dir.joinpath("runtime_deployment_frontier_labels.csv").write_text(
                "deployment_label,status,runtime_truth_used\nruntime_observation_stress,PASS,False\n",
                encoding="utf-8",
            )
            output_dir.joinpath("runtime_deployment_frontier_profiles.csv").write_text(
                "profile_label,runtime_truth_used,runtime_inputs_truth_free\nstress_active,False,True\n",
                encoding="utf-8",
            )
            output_dir.joinpath("runtime_deployment_frontier.txt").write_text("status: PASS\n", encoding="utf-8")
            return {"status": "PASS", "profile_count": 2, "deployment_label_count": 3}

        def fake_package_portability_check(*, package_dir, deployment_label="", scenario_label="", scenario_name=""):
            calls.append("package_portability")
            return {
                "status": "PASS",
                "selected_label": "packaged_active",
                "copied_package_check_status": "PASS",
                "copied_package_file_count": 15,
                "copied_package_dir_ephemeral": True,
                "copied_package_dir_note": "The copied package path is a temporary directory used only during this smoke check.",
                "runtime_truth_used": False,
                "coordinate_contract": "RDF(right,down,forward) FRD+range",
                "failure_count": 0,
                "failures": [],
            }

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            (root / "runtime_profile_label_selector.json").write_text(
                '{"mode":"stereodistance_runtime_profile_label_selector",'
                '"axis_contract":"RDF(right,down,forward)",'
                '"public_topic_contract":"FRD_range(forward,right,down,range)",'
                '"selection_input_fields":["deployment_label"]}\n',
                encoding="utf-8",
            )
            matrix_gate_json = root / "active_profile_matrix_gate.json"
            matrix_gate_json.write_text(
                '{"status":"PASS","recommendation":"keep_active_profile","runtime_truth_used":false}\n',
                encoding="utf-8",
            )
            with mock.patch.object(module.acceptance_summary, "run", side_effect=fake_acceptance_run):
                with mock.patch.object(module.promotion_check, "run_check", side_effect=fake_promotion_run_check):
                    with mock.patch.object(module.active_profile_installer, "install_package", side_effect=fake_install_package):
                        with mock.patch.object(module.public_payload_exporter, "export_payloads", side_effect=fake_public_payload_export):
                            with mock.patch.object(module.package_inventory_exporter, "export_inventory", side_effect=fake_package_inventory):
                                with mock.patch.object(module.operator_view_launch_checker, "check_operator_view_launch", side_effect=fake_operator_view_launch_check):
                                    with mock.patch.object(module.runtime_frontier_exporter, "export_frontier", side_effect=fake_runtime_frontier_export):
                                        with mock.patch.object(module.package_portability_checker, "check_portability", side_effect=fake_package_portability_check):
                                            with mock.patch.object(
                                                module.deployment_readiness_checker,
                                                "check_readiness",
                                                side_effect=fake_readiness_check,
                                            ):
                                                result = module.run(
                                                    [
                                                        "--single-audit-json",
                                                        str(root / "single.json"),
                                                        "--multi-seed-summary-json",
                                                        str(root / "multi.json"),
                                                        "--robustness-summary-json",
                                                        str(root / "robust.json"),
                                                        "--deployment-strategy-json",
                                                        str(root / "deployment.json"),
                                                        "--output-active-profile",
                                                        str(root / "active_runtime_profile.json"),
                                                        "--calibration-samples-csv",
                                                        str(root / "calibration_samples.csv"),
                                                        "--output-dir",
                                                        str(root / "out"),
                                                        "--runtime-profile-selector-json",
                                                        str(root / "runtime_profile_label_selector.json"),
                                                        "--runtime-profile-selector-deployment-label",
                                                        "runtime_observation_stress",
                                                        "--matrix-active-profile-gate-json",
                                                        str(matrix_gate_json),
                                                        "--install-rehearsal-profile",
                                                        str(root / "install_rehearsal" / "active_runtime_profile.json"),
                                                    ]
                                                )

            summary_json = root / "out" / "rtk_stereo_acceptance_dashboard_with_promotion_summary.json"
            manifest_json = root / "out" / "active_profile_deployment_manifest.json"
            manifest_validation_json = root / "out" / "active_profile_deployment_manifest_validation.json"
            package_manifest_json = root / "out" / "deployment_package" / "deployment_package_manifest.json"
            package_check_json = root / "out" / "deployment_package" / "deployment_package_check.json"
            install_manifest_json = root / "out" / "install_rehearsal" / "install_manifest.json"
            install_profile = root / "install_rehearsal" / "active_runtime_profile.json"
            readiness_json = root / "out" / "deployment_readiness.json"
            readiness_csv = root / "out" / "deployment_readiness.csv"
            public_payload_csv = root / "out" / "public_topic_payload" / "public_corrected_position_payload.csv"
            inventory_json = root / "out" / "deployment_package" / "inventory" / "stereodistance_deployment_package_inventory.json"
            selector_launch_json = root / "out" / "operator_view_launch_check_selector.json"
            frontier_json = root / "out" / "runtime_deployment_frontier" / "runtime_deployment_frontier.json"
            portability_json = root / "out" / "deployment_package" / "evidence" / "package_portability_check.json"
            summary_exists = summary_json.is_file()
            manifest_exists = manifest_json.is_file()
            manifest_validation_exists = manifest_validation_json.is_file()
            package_manifest_exists = package_manifest_json.is_file()
            package_manifest_file_count = json.loads(package_manifest_json.read_text(encoding="utf-8"))["file_count"]
            package_check_exists = package_check_json.is_file()
            install_manifest_exists = install_manifest_json.is_file()
            install_profile_exists = install_profile.is_file()
            readiness_json_exists = readiness_json.is_file()
            readiness_csv_exists = readiness_csv.is_file()
            public_payload_csv_exists = public_payload_csv.is_file()
            inventory_json_exists = inventory_json.is_file()
            selector_launch_json_exists = selector_launch_json.is_file()
            frontier_json_exists = frontier_json.is_file()
            portability_json_exists = portability_json.is_file()

        self.assertEqual(
            calls,
            [
                "acceptance_initial",
                "promotion",
                "acceptance_with_promotion",
                "public_payload",
                "operator_view_package_launch",
                "operator_view_selector_launch",
                "runtime_frontier",
                "package_portability",
                "operator_view_selector_launch",
                "package_inventory",
                "install_rehearsal",
                "deployment_readiness",
            ],
        )
        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["initial_overall_pass"])
        self.assertTrue(result["promotion_pass"])
        self.assertTrue(result["final_overall_pass"])
        self.assertTrue(result["active_profile_hash_stable"])
        self.assertEqual(result["final_matrix_item_count"], 29)
        self.assertEqual(result["final_rdf_axis_nonregression_status"], "PASS")
        self.assertTrue(summary_exists)
        self.assertTrue(manifest_exists)
        self.assertTrue(manifest_validation_exists)
        self.assertTrue(package_manifest_exists)
        self.assertTrue(package_check_exists)
        self.assertTrue(install_manifest_exists)
        self.assertTrue(install_profile_exists)
        self.assertEqual(result["deployment_manifest_json"], str(manifest_json))
        self.assertEqual(result["deployment_manifest_validation_status"], "PASS")
        self.assertEqual(result["deployment_package_file_count"], package_manifest_file_count)
        self.assertEqual(result["deployment_package_check_status"], "PASS")
        self.assertEqual(result["install_rehearsal_status"], "PASS")
        self.assertEqual(result["install_rehearsal_profile"], str(install_profile))
        self.assertEqual(result["install_rehearsal_sha256"], "abc123")
        self.assertTrue(readiness_json_exists)
        self.assertTrue(readiness_csv_exists)
        self.assertTrue(public_payload_csv_exists)
        self.assertTrue(inventory_json_exists)
        self.assertTrue(selector_launch_json_exists)
        self.assertTrue(frontier_json_exists)
        self.assertTrue(portability_json_exists)
        self.assertEqual(result["deployment_readiness_status"], "PASS")
        self.assertEqual(result["deployment_readiness_json"], str(readiness_json))
        self.assertEqual(result["deployment_readiness_csv"], str(readiness_csv))
        self.assertEqual(result["matrix_active_profile_gate_json"], str(matrix_gate_json))
        self.assertEqual(result["matrix_active_profile_gate_status"], "PASS")
        self.assertEqual(result["matrix_active_profile_gate_recommendation"], "keep_active_profile")
        self.assertEqual(result["public_topic_payload_status"], "PASS")
        self.assertEqual(result["public_topic_payload_csv"], str(public_payload_csv))
        self.assertEqual(result["deployment_package_inventory_status"], "PASS")
        self.assertEqual(result["deployment_package_inventory_file_count"], 13)
        self.assertEqual(result["operator_view_selector_launch_check_status"], "PASS")
        self.assertEqual(result["operator_view_selector_launch_check_json"], str(selector_launch_json))
        self.assertEqual(result["operator_view_selector_selected_label"], "packaged_active")
        self.assertEqual(result["runtime_deployment_frontier_status"], "PASS")
        self.assertEqual(result["runtime_deployment_frontier_json"], str(frontier_json))
        self.assertEqual(result["deployment_package_portability_status"], "PASS")
        self.assertEqual(result["deployment_package_portability_json"], str(portability_json))
        self.assertEqual(result["deployment_package_portability_selected_label"], "packaged_active")

    def test_wrapper_fails_when_final_dashboard_fails(self) -> None:
        module = load_module()

        def fake_acceptance_run(**kwargs):
            Path(kwargs["output_active_profile"]).parent.mkdir(parents=True, exist_ok=True)
            Path(kwargs["output_active_profile"]).write_text(
                '{"profile_id":"unit","method":"sim3","mode":"estimated_to_truth_sim3"}\n',
                encoding="utf-8",
            )
            Path(kwargs["output_dir"]).mkdir(parents=True, exist_ok=True)
            Path(kwargs["output_dir"]).joinpath("rtk_stereo_acceptance_matrix_summary.json").write_text(
                '{"overall_pass": false, "matrix_item_count": 28, '
                '"stage_status": {"standalone_profile_promotion": false}}\n',
                encoding="utf-8",
            )
            return {
                "overall_pass": not bool(kwargs.get("promotion_check_json")),
                "matrix_item_count": 28 if kwargs.get("promotion_check_json") else 27,
                "stage_status": {"standalone_profile_promotion": False},
            }

        def fake_promotion_run_check(profile_json, output_dir, *, sample_csv=None, split="validation", max_samples=50):
            output_dir.mkdir(parents=True, exist_ok=True)
            output_dir.joinpath("promotion_check.json").write_text('{"pass_promotion_check": false}\n', encoding="utf-8")
            return {
                "pass_promotion_check": False,
                "sample_apply_count": 0,
                "sample_invalid_count": 0,
            }

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            with mock.patch.object(module.acceptance_summary, "run", side_effect=fake_acceptance_run):
                with mock.patch.object(module.promotion_check, "run_check", side_effect=fake_promotion_run_check):
                    result = module.run(
                        [
                            "--single-audit-json",
                            str(root / "single.json"),
                            "--multi-seed-summary-json",
                            str(root / "multi.json"),
                            "--robustness-summary-json",
                            str(root / "robust.json"),
                            "--deployment-strategy-json",
                            str(root / "deployment.json"),
                            "--output-active-profile",
                            str(root / "active_runtime_profile.json"),
                            "--calibration-samples-csv",
                            str(root / "calibration_samples.csv"),
                            "--output-dir",
                            str(root / "out"),
                        ]
                    )

        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["promotion_pass"])
        self.assertFalse(result["final_overall_pass"])

    def test_wrapper_fails_when_final_export_changes_promoted_profile(self) -> None:
        module = load_module()

        def fake_acceptance_run(**kwargs):
            output_profile = Path(kwargs["output_active_profile"])
            output_profile.parent.mkdir(parents=True, exist_ok=True)
            if kwargs.get("promotion_check_json"):
                output_profile.write_text(
                    '{"profile_id":"unit","method":"changed","mode":"estimated_to_truth_sim3"}\n',
                    encoding="utf-8",
                )
            else:
                output_profile.write_text(
                    '{"profile_id":"unit","method":"promoted","mode":"estimated_to_truth_sim3"}\n',
                    encoding="utf-8",
                )
            Path(kwargs["output_dir"]).mkdir(parents=True, exist_ok=True)
            Path(kwargs["output_dir"]).joinpath("rtk_stereo_acceptance_matrix_summary.json").write_text(
                '{"overall_pass": true, "matrix_item_count": 28, '
                '"stage_status": {"standalone_profile_promotion": true}}\n',
                encoding="utf-8",
            )
            return {
                "overall_pass": True,
                "matrix_item_count": 28 if kwargs.get("promotion_check_json") else 27,
                "stage_status": {"standalone_profile_promotion": bool(kwargs.get("promotion_check_json"))},
            }

        def fake_promotion_run_check(profile_json, output_dir, *, sample_csv=None, split="validation", max_samples=50):
            output_dir.mkdir(parents=True, exist_ok=True)
            output_dir.joinpath("promotion_check.json").write_text('{"pass_promotion_check": true}\n', encoding="utf-8")
            return {
                "pass_promotion_check": True,
                "sample_apply_count": 3,
                "sample_invalid_count": 0,
            }

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            with mock.patch.object(module.acceptance_summary, "run", side_effect=fake_acceptance_run):
                with mock.patch.object(module.promotion_check, "run_check", side_effect=fake_promotion_run_check):
                    result = module.run(
                        [
                            "--single-audit-json",
                            str(root / "single.json"),
                            "--multi-seed-summary-json",
                            str(root / "multi.json"),
                            "--robustness-summary-json",
                            str(root / "robust.json"),
                            "--deployment-strategy-json",
                            str(root / "deployment.json"),
                            "--output-active-profile",
                            str(root / "active_runtime_profile.json"),
                            "--calibration-samples-csv",
                            str(root / "calibration_samples.csv"),
                            "--output-dir",
                            str(root / "out"),
                        ]
                    )

        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(result["final_overall_pass"])
        self.assertFalse(result["active_profile_hash_stable"])

    def test_wrapper_fails_when_install_rehearsal_fails(self) -> None:
        module = load_module()

        def fake_acceptance_run(**kwargs):
            output_profile = Path(kwargs["output_active_profile"])
            output_profile.parent.mkdir(parents=True, exist_ok=True)
            output_profile.write_text(
                '{"profile_id":"unit","method":"sim3","mode":"estimated_to_truth_sim3",'
                '"axis_contract":"RDF(right,down,forward)",'
                '"runtime_inputs":["raw_right_m","raw_down_m","raw_forward_m","raw_disparity_px"],'
                '"forbidden_runtime_inputs":["calibration_truth","px4_rtk_truth"],'
                '"scale":1.0,'
                '"rotation_matrix_rdf":[[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],'
                '"translation_rdf_m":[0.0,0.0,0.0]}\n',
                encoding="utf-8",
            )
            Path(kwargs["output_dir"]).mkdir(parents=True, exist_ok=True)
            matrix_item_count = 29 if kwargs.get("promotion_check_json") else 27
            Path(kwargs["output_dir"]).joinpath("rtk_stereo_acceptance_matrix_summary.json").write_text(
                json.dumps(
                    {
                        "overall_pass": True,
                        "matrix_item_count": matrix_item_count,
                        "runtime_truth_used": False,
                        "stage_status": {"standalone_profile_promotion": bool(kwargs.get("promotion_check_json"))},
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
            return {
                "overall_pass": True,
                "matrix_item_count": matrix_item_count,
                "stage_status": {"standalone_profile_promotion": bool(kwargs.get("promotion_check_json"))},
                "matrix_rows": [
                    {
                        "stage": "single_run_output_audit",
                        "item": "rdf_axis_rmse_nonregression",
                        "status": "PASS",
                    }
                ],
            }

        def fake_promotion_run_check(profile_json, output_dir, *, sample_csv=None, split="validation", max_samples=50):
            output_dir.mkdir(parents=True, exist_ok=True)
            output_dir.joinpath("promotion_check.json").write_text(
                '{"pass_promotion_check": true, "node_loader_ok": true, '
                '"axis_contract_declared": true, "runtime_truth_used": false, '
                '"sample_apply_count": 3, "sample_invalid_count": 0}\n',
                encoding="utf-8",
            )
            return {
                "pass_promotion_check": True,
                "sample_apply_count": 3,
                "sample_invalid_count": 0,
            }

        def fake_install_package(*, package_dir, output_profile, force=False):
            return {
                "status": "FAIL",
                "installed_profile": str(output_profile),
                "installed_profile_sha256": "",
                "expected_profile_sha256": "abc123",
                "runtime_truth_used": False,
                "coordinate_contract": "RDF(right,down,forward) FRD+range",
                "failure_count": 1,
                "failures": ["output_profile:exists_without_force"],
            }

        def fake_public_payload_export(*, profile_json, sample_csv, output_dir, split="validation", max_samples=50):
            output_dir.mkdir(parents=True, exist_ok=True)
            output_dir.joinpath("public_corrected_position_payload.csv").write_text(
                "topic,data_axis_order,data,runtime_truth_used,node_public_payload_equivalent\n"
                '"/stereo/corrected_position","forward_m,right_m,down_m,range_m","[1.0,2.0,3.0,4.0]",False,True\n',
                encoding="utf-8",
            )
            output_dir.joinpath("public_corrected_position_payload_summary.json").write_text(
                '{"status": "PASS", "sample_count": 1, "invalid_count": 0, '
                '"node_public_payload_equivalence_status": "PASS", '
                '"runtime_truth_used": false, '
                '"input_coordinate_contract": "runtime_input:stereo_center_RDF(right,down,forward)", '
                '"public_topic_contract": "std_msgs/Float32MultiArray:data=[forward_m,right_m,down_m,range_m]"}\n',
                encoding="utf-8",
            )
            return {
                "status": "PASS",
                "sample_count": 1,
                "invalid_count": 0,
                "node_public_payload_equivalence_status": "PASS",
                "runtime_truth_used": False,
            }

        def fake_package_inventory(package_dir, output_dir):
            output_dir.mkdir(parents=True, exist_ok=True)
            output_dir.joinpath("stereodistance_deployment_package_inventory.json").write_text(
                '{"status": "PASS", "file_count": 13, "runtime_truth_used": false, "files": []}\n',
                encoding="utf-8",
            )
            return {"status": "PASS", "file_count": 13, "runtime_truth_used": False}

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            with mock.patch.object(module.acceptance_summary, "run", side_effect=fake_acceptance_run):
                with mock.patch.object(module.promotion_check, "run_check", side_effect=fake_promotion_run_check):
                    with mock.patch.object(module.active_profile_installer, "install_package", side_effect=fake_install_package):
                        with mock.patch.object(module.public_payload_exporter, "export_payloads", side_effect=fake_public_payload_export):
                            with mock.patch.object(module.package_inventory_exporter, "export_inventory", side_effect=fake_package_inventory):
                                result = module.run(
                                    [
                                        "--single-audit-json",
                                        str(root / "single.json"),
                                        "--multi-seed-summary-json",
                                        str(root / "multi.json"),
                                        "--robustness-summary-json",
                                        str(root / "robust.json"),
                                        "--deployment-strategy-json",
                                        str(root / "deployment.json"),
                                        "--output-active-profile",
                                        str(root / "active_runtime_profile.json"),
                                        "--calibration-samples-csv",
                                        str(root / "calibration_samples.csv"),
                                        "--output-dir",
                                        str(root / "out"),
                                        "--install-rehearsal-profile",
                                        str(root / "install_rehearsal" / "active_runtime_profile.json"),
                                    ]
                                )

            install_manifest_json = root / "out" / "install_rehearsal" / "install_manifest.json"
            install_manifest_exists = install_manifest_json.is_file()

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["install_rehearsal_status"], "FAIL")
        self.assertTrue(install_manifest_exists)


if __name__ == "__main__":
    unittest.main()
