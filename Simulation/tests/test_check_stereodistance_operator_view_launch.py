from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_stereodistance_operator_view_launch.py"
EXPORT_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_active_profile_deployment_package.py"
CANDIDATE_EXPORT_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_deployable_profile_candidate_package.py"
PUBLIC_PAYLOAD_SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_public_topic_payload_from_profile.py"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckStereoDistanceOperatorViewLaunchTests(unittest.TestCase):
    def write_profile(self, path: Path) -> None:
        path.write_text(
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

    def write_observation_fusion_manifest(self, path: Path) -> None:
        path.write_text(
            json.dumps(
                {
                    "default_recommendation": "unit_repeated",
                    "tiers": [
                        {
                            "tier": "unit_repeated",
                            "adapter": {
                                "class": "RepeatedObservationFusion",
                                "min_observations": 3,
                                "mode": "median",
                                "trim_fraction": 0.0,
                            },
                        }
                    ],
                    "runtime_input_contract": "image left_u_px,left_v_px,measured_disparity_px only",
                    "output_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                }
            )
            + "\n",
            encoding="utf-8",
        )

    def write_residual_profile(self, path: Path, profile_id: str = "residual_unit") -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                {
                    "profile_type": "DisparityResidualRuntimeProfile",
                    "profile_id": profile_id,
                    "method": f"{profile_id}_method",
                    "feature_space": "log_disparity",
                    "correction_space": "ray_scale",
                    "degree": 1,
                    "feature_mean": 1.0,
                    "feature_scale": 1.0,
                    "coefficients_low_to_high": [0.0, 0.1],
                    "correction_clip": 0.5,
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "measured_disparity_px"],
                    "runtime_truth_used": False,
                    "coordinate_contracts_explicit": True,
                    "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                    "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                }
            )
            + "\n",
            encoding="utf-8",
        )

    def write_residual_decision(self, path: Path, residual_profile: Path, *, allowed: bool = True) -> None:
        path.write_text(
            json.dumps(
                {
                    "mode": "operator_telemetry_selector_decision",
                    "status": "PASS" if allowed else "FAIL",
                    "apply_residual_allowed": allowed,
                    "recommended_residual_label": "unit" if allowed else "",
                    "recommended_residual_profile": str(residual_profile) if allowed else "",
                    "selector_label_stable": allowed,
                    "selected_label_counts": {"unit": 1} if allowed else {},
                    "distance_margin_min": 1.0 if allowed else 0.0,
                    "require_stable_label": True,
                    "min_distance_margin": 0.5,
                    "runtime_truth_used": False,
                    "decision_contract": (
                        "apply_residual_allowed is true only when the truth-free telemetry export and selector replay gates pass"
                    ),
                    "coordinate_contract": (
                        "operator telemetry input is public FRD; selector observation output is stereo-center RDF"
                    ),
                    "failure_count": 0 if allowed else 1,
                    "failures": [] if allowed else ["selector:unstable_label"],
                }
            )
            + "\n",
            encoding="utf-8",
        )

    def make_package(self, root: Path) -> Path:
        exporter = load_module("export_stereodistance_package_for_operator_launch_test", EXPORT_SCRIPT_PATH)
        active_profile = root / "active_runtime_profile.json"
        self.write_profile(active_profile)
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
        package = root / "deployment_package"
        exporter.export_package(manifest_json=manifest, manifest_validation_json=validation, output_dir=package)
        return package

    def make_candidate_package(self, root: Path) -> Path:
        candidate_exporter = load_module(
            "export_deployable_profile_candidate_package_for_operator_launch_test",
            CANDIDATE_EXPORT_SCRIPT_PATH,
        )
        public_payload_exporter = load_module(
            "export_public_payload_for_candidate_operator_launch_test",
            PUBLIC_PAYLOAD_SCRIPT_PATH,
        )
        active_profile = root / "candidate_source" / "active_runtime_profile.json"
        active_profile.parent.mkdir()
        self.write_profile(active_profile)
        active_sha = candidate_exporter.sha256_file(active_profile)
        policy = root / "candidate_source" / "policy.json"
        policy.write_text(
            json.dumps(
                {
                    "status": "PASS",
                    "recommended_policy": "range_priority",
                    "recommended_method": "sim3",
                    "recommended_profile_sha256": active_sha,
                    "runtime_truth_used": False,
                }
            ),
            encoding="utf-8",
        )
        promotion = root / "candidate_source" / "promotion_check.json"
        promotion.write_text(
            json.dumps(
                {
                    "pass_promotion_check": True,
                    "sample_apply_count": 1,
                    "sample_invalid_count": 0,
                    "runtime_truth_used": False,
                }
            ),
            encoding="utf-8",
        )
        samples = root / "candidate_source" / "samples.csv"
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
        public_payload = public_payload_exporter.export_payloads(
            profile_json=active_profile,
            sample_csv=samples,
            output_dir=root / "candidate_source" / "public_payload",
            split="validation",
            max_samples=1,
        )
        candidate = root / "candidate_source" / "candidate_export.json"
        candidate.write_text(
            json.dumps(
                {
                    "mode": "deployable_profile_candidate_export",
                    "status": "PASS",
                    "recommendation": "use_as_next_acceptance_dashboard_active_profile",
                    "policy_selection_json": str(policy),
                    "recommended_policy": "range_priority",
                    "method": "sim3",
                    "output_active_profile": str(active_profile),
                    "active_profile_sha256": active_sha,
                    "profile_sha256_matches_policy": True,
                    "candidate_pass_count": 1,
                    "candidate_expected_run_count": 1,
                    "candidate_axis_pass_count": 1,
                    "candidate_target_pass_count": 1,
                    "promotion_check_json": str(promotion),
                    "promotion_check_pass": True,
                    "promotion_sample_apply_count": 1,
                    "promotion_sample_invalid_count": 0,
                    "public_topic_payload_json": str(root / "candidate_source" / "public_payload" / "public_corrected_position_payload_summary.json"),
                    "public_topic_payload_csv": str(root / "candidate_source" / "public_payload" / "public_corrected_position_payload.csv"),
                    "public_topic_payload_status": public_payload["status"],
                    "public_topic_payload_sample_count": public_payload["sample_count"],
                    "public_topic_payload_invalid_count": public_payload["invalid_count"],
                    "public_topic_payload_node_equivalence_status": public_payload["node_public_payload_equivalence_status"],
                    "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                    "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                    "runtime_input_contract": "exported candidate consumes stereo-center RDF(right,down,forward) plus measured disparity",
                    "runtime_truth_used": False,
                }
            ),
            encoding="utf-8",
        )
        readiness = root / "candidate_source" / "candidate_readiness.json"
        readiness.write_text(
            json.dumps(
                {
                    "mode": "deployable_profile_candidate_readiness",
                    "status": "PASS",
                    "candidate_export_json": str(candidate),
                    "method": "sim3",
                    "recommended_policy": "range_priority",
                    "active_profile_sha256": active_sha,
                    "candidate_pass_count": 1,
                    "candidate_expected_run_count": 1,
                    "candidate_axis_pass_count": 1,
                    "candidate_target_pass_count": 1,
                    "promotion_check_pass": True,
                    "promotion_sample_apply_count": 1,
                    "promotion_sample_invalid_count": 0,
                    "public_topic_payload_status": "PASS",
                    "public_topic_payload_sample_count": 1,
                    "public_topic_payload_node_equivalence_status": "PASS",
                    "runtime_truth_used": False,
                }
            ),
            encoding="utf-8",
        )
        package = root / "candidate_package"
        candidate_exporter.export_package(candidate_readiness_json=readiness, output_dir=package)
        return package

    def test_profile_json_resolves_operator_view_command(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_profile", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            profile = Path(tmpdir) / "active_runtime_profile.json"
            self.write_profile(profile)

            result = module.check_operator_view_launch(profile_json=profile)
            expected_sha = module.sha256_file(profile)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["profile_mode"], "estimated_to_truth_sim3")
        self.assertTrue(result["bbox_adjustment_arg_present"])
        self.assertEqual(result["resolved_runtime_profile"], str(profile.resolve()))
        self.assertEqual(result["resolved_runtime_profile_sha256"], expected_sha)
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])

    def test_profile_json_with_observation_fusion_manifest_resolves_command(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_profile_fusion", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            manifest = root / "observation_fusion_runtime_manifest.json"
            self.write_profile(profile)
            self.write_observation_fusion_manifest(manifest)

            result = module.check_operator_view_launch(
                profile_json=profile,
                observation_fusion_manifest=manifest,
                observation_fusion_tier="unit_repeated",
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["observation_fusion_manifest_status"], "PASS")
        self.assertEqual(result["resolved_observation_fusion_manifest"], str(manifest.resolve()))
        self.assertTrue(result["observation_fusion_arg_present"])
        self.assertIn("--observation-fusion-manifest", result["operator_command_stdout"])
        self.assertIn("--observation-fusion-tier", result["operator_command_stdout"])
        self.assertIn("left_u/left_v/disparity", result["coordinate_contract"])

    def test_profile_json_with_residual_feature_selector_resolves_command(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_residual_feature_selector", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            residual_profile = root / "profiles" / "unit.json"
            catalog = root / "disparity_residual_profile_label_selector.json"
            model = root / "disparity_residual_feature_selector_model.json"
            feature_csv = root / "features.csv"
            self.write_profile(profile)
            self.write_residual_profile(residual_profile)
            catalog.write_text(
                json.dumps({"profile_paths": {"unit": "profiles/unit.json"}}) + "\n",
                encoding="utf-8",
            )
            model.write_text(
                json.dumps(
                    {
                        "feature_names": ["measured_disparity_px_mean", "observation_forward_m_mean"],
                        "mean": [0.0, 0.0],
                        "scale": [1.0, 1.0],
                        "centroids": {"unit": [1.0, 2.0]},
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            feature_csv.write_text(
                "measured_disparity_px_mean,observation_forward_m_mean\n1.0,2.1\n",
                encoding="utf-8",
            )

            result = module.check_operator_view_launch(
                profile_json=profile,
                disparity_residual_feature_selector_json=model,
                disparity_residual_feature_catalog_json=catalog,
                disparity_residual_feature_csv=feature_csv,
                disparity_residual_feature_row_index=0,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["residual_feature_selection_status"], "PASS")
        self.assertEqual(result["residual_feature_selected_label"], "unit")
        self.assertEqual(result["resolved_disparity_residual_profile"], str(residual_profile.resolve()))
        self.assertTrue(result["disparity_residual_arg_present"])
        self.assertIn("Residual feature selected label: unit", result["operator_command_stdout"])
        self.assertIn("image/disparity/RDF", result["coordinate_contract"])

    def test_profile_json_with_residual_decision_resolves_command(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_residual_decision", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            residual_profile = root / "profiles" / "unit.json"
            decision = root / "operator_telemetry_selector_decision.json"
            self.write_profile(profile)
            self.write_residual_profile(residual_profile, profile_id="unit")
            self.write_residual_decision(decision, residual_profile)

            result = module.check_operator_view_launch(
                profile_json=profile,
                disparity_residual_decision_json=decision,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["residual_decision_selected_label"], "unit")
        self.assertEqual(result["disparity_residual_decision_status"], "PASS")
        self.assertTrue(result["disparity_residual_decision_apply_allowed"])
        self.assertEqual(result["resolved_disparity_residual_decision"], str(decision.resolve()))
        self.assertEqual(result["resolved_disparity_residual_decision_label"], "unit")
        self.assertEqual(result["resolved_disparity_residual_profile"], str(residual_profile.resolve()))
        self.assertTrue(result["disparity_residual_arg_present"])
        self.assertIn("Residual decision selected label: unit", result["operator_command_stdout"])
        self.assertIn("--disparity-residual-profile", result["operator_command_stdout"])

    def test_profile_json_with_disallowed_residual_decision_fails(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_residual_decision_fail", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            residual_profile = root / "profiles" / "unit.json"
            decision = root / "operator_telemetry_selector_decision.json"
            self.write_profile(profile)
            self.write_residual_profile(residual_profile, profile_id="unit")
            self.write_residual_decision(decision, residual_profile, allowed=False)

            result = module.check_operator_view_launch(
                profile_json=profile,
                disparity_residual_decision_json=decision,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("residual_decision:status", result["failures"])
        self.assertIn("residual_decision:apply_not_allowed", result["failures"])
        self.assertFalse(result["disparity_residual_arg_present"])

    def test_profile_json_with_node_residual_feature_selector_resolves_command(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_node_residual_feature_selector", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            residual_profile = root / "profiles" / "unit.json"
            catalog = root / "disparity_residual_profile_label_selector.json"
            model = root / "disparity_residual_feature_selector_model.json"
            self.write_profile(profile)
            self.write_residual_profile(residual_profile)
            catalog.write_text(
                json.dumps({"profile_paths": {"unit": "profiles/unit.json"}}) + "\n",
                encoding="utf-8",
            )
            model.write_text(
                json.dumps(
                    {
                        "feature_names": ["measured_disparity_px_mean", "observation_forward_m_mean"],
                        "mean": [0.0, 0.0],
                        "scale": [1.0, 1.0],
                        "centroids": {"unit": [1.0, 2.0]},
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = module.check_operator_view_launch(
                profile_json=profile,
                node_disparity_residual_feature_selector_json=model,
                node_disparity_residual_feature_catalog_json=catalog,
                node_disparity_residual_feature_window_size=21,
                node_disparity_residual_feature_min_samples=7,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["resolved_node_disparity_residual_feature_selector"], str(model.resolve()))
        self.assertEqual(result["resolved_node_disparity_residual_feature_catalog"], str(catalog.resolve()))
        self.assertEqual(result["node_disparity_residual_feature_count"], 2)
        self.assertEqual(result["node_disparity_residual_catalog_profile_count"], 1)
        self.assertEqual(result["node_disparity_residual_feature_window_size"], 21)
        self.assertEqual(result["node_disparity_residual_feature_min_samples"], 7)
        self.assertTrue(result["node_disparity_residual_feature_selector_arg_present"])
        self.assertTrue(result["node_disparity_residual_feature_catalog_arg_present"])
        self.assertTrue(result["node_disparity_residual_feature_window_size_arg_present"])
        self.assertTrue(result["node_disparity_residual_feature_min_samples_arg_present"])
        self.assertIn("--disparity-residual-feature-selector", result["operator_command_stdout"])
        self.assertIn("--disparity-residual-feature-catalog", result["operator_command_stdout"])
        self.assertIn("--disparity-residual-feature-window-size", result["operator_command_stdout"])
        self.assertIn("--disparity-residual-feature-min-samples", result["operator_command_stdout"])

    def test_package_dir_runs_package_check_and_resolves_command(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_package", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            package = self.make_package(Path(tmpdir))

            result = module.check_operator_view_launch(package_dir=package)
            expected_sha = module.sha256_file(package / "active_runtime_profile.json")

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["package_check_status"], "PASS")
        self.assertEqual(result["resolved_runtime_profile"], str((package / "active_runtime_profile.json").resolve()))
        self.assertEqual(result["resolved_runtime_profile_sha256"], expected_sha)
        self.assertIn("--bbox-adjustment", result["operator_command_stdout"])

    def test_candidate_package_dir_runs_candidate_package_check_and_resolves_command(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_candidate_package", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            package = self.make_candidate_package(Path(tmpdir))

            result = module.check_operator_view_launch(package_dir=package)
            expected_sha = module.sha256_file(package / "active_runtime_profile.json")

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["package_type"], "candidate_package")
        self.assertEqual(result["package_check_status"], "PASS")
        self.assertEqual(result["resolved_runtime_profile"], str((package / "active_runtime_profile.json").resolve()))
        self.assertEqual(result["resolved_runtime_profile_sha256"], expected_sha)
        self.assertIn("--bbox-adjustment", result["operator_command_stdout"])

    def test_selector_json_resolves_package_and_operator_view_command(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_selector", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = self.make_package(root)
            evidence = root / "selector_evidence.txt"
            selector_json = root / "runtime_profile_label_selector.json"
            evidence.write_text("PASS\n", encoding="utf-8")
            selector_json.write_text(
                json.dumps(
                    {
                        "mode": "stereodistance_runtime_profile_label_selector",
                        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                        "selection_input_fields": ["deployment_label", "scenario_label", "scenario_name"],
                        "forbidden_runtime_fields": ["truth", "px4", "rtk", "ned"],
                        "baseline_selector": "current_active",
                        "label_profile_map": {"runtime_observation_stress": "stress_active"},
                        "profile_paths": {
                            "stress_active": str(package / "active_runtime_profile.json"),
                            "current_active": str(package / "active_runtime_profile.json"),
                        },
                        "package_paths": {
                            "stress_active": str(package),
                            "current_active": str(package),
                        },
                        "validated_evidence": {"unit": str(evidence)},
                        "selection_contract": "truth-free deployment/scenario label selection only",
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = module.check_operator_view_launch(
                selector_json=selector_json,
                deployment_label="runtime_observation_stress",
            )
            expected_sha = module.sha256_file(package / "active_runtime_profile.json")

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["source_mode"], "selector")
        self.assertEqual(result["selected_label"], "stress_active")
        self.assertEqual(result["label_selection_status"], "PASS")
        self.assertEqual(result["package_check_status"], "PASS")
        self.assertEqual(result["resolved_runtime_profile"], str((package / "active_runtime_profile.json").resolve()))
        self.assertEqual(result["resolved_runtime_profile_sha256"], expected_sha)
        self.assertIn(f"Selector: {selector_json}", result["operator_command_stdout"])
        self.assertIn("Selected label: stress_active", result["operator_command_stdout"])
        self.assertIn("--bbox-adjustment", result["operator_command_stdout"])

    def test_run_writes_json_and_text(self) -> None:
        module = load_module("check_stereodistance_operator_view_launch_under_test_run", SCRIPT_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            output_json = root / "operator_launch_check.json"
            output_txt = root / "operator_launch_check.txt"
            self.write_profile(profile)

            result = module.run(
                [
                    "--profile-json",
                    str(profile),
                    "--output-json",
                    str(output_json),
                    "--output-txt",
                    str(output_txt),
                ]
            )
            output_json_exists = output_json.is_file()
            output_txt_text = output_txt.read_text(encoding="utf-8")

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(output_json_exists)
        self.assertIn("status: PASS", output_txt_text)
        self.assertIn("bbox_adjustment_arg_present: True", output_txt_text)


if __name__ == "__main__":
    unittest.main()
