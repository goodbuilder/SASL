from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_selector_module():
    spec = importlib.util.spec_from_file_location(
        "select_stereodistance_profile_from_catalog_under_test",
        ROOT_DIR / "scripts" / "select_stereodistance_profile_from_catalog.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load select_stereodistance_profile_from_catalog.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SelectStereoDistanceProfileFromCatalogTests(unittest.TestCase):
    def test_load_catalog_accepts_rows_based_sweep_summary(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_json = root / "scenario_profile_catalog.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "catalog_profile_count": 1,
                        "rows": [
                            {
                                "scenario": "nominal",
                                "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            catalog = module.load_catalog(catalog_json)

            self.assertEqual(catalog["mode"], "scenario_profile_catalog")
            self.assertEqual(catalog["profiles"][0]["scenario"], "nominal")

    def test_select_profile_copies_validated_scenario_profile(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            source_profile = catalog_dir / "nominal_runtime_profile.json"
            source_profile.write_text('{"mode": "estimated_to_truth_sim3"}\n', encoding="utf-8")
            catalog = {
                "mode": "scenario_profile_catalog",
                "selection_contract": "operator selects by environment",
                "profiles": [
                    {
                        "scenario": "nominal",
                        "method": "sim3",
                        "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                        "profile_exists": True,
                        "pass_cross_seed_replay": True,
                        "validation_range_rmse_mean_m": 1.0,
                        "target_abs_mean_error_max_m": 0.5,
                    }
                ],
            }
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(json.dumps(catalog), encoding="utf-8")
            output_profile = root / "active_runtime_profile.json"

            summary = module.select_profile(
                catalog_json=catalog_json,
                scenario="nominal",
                output_profile=output_profile,
            )

            self.assertTrue(output_profile.is_file())
            self.assertEqual(output_profile.read_text(encoding="utf-8"), source_profile.read_text(encoding="utf-8"))
            self.assertEqual(summary["scenario"], "nominal")
            self.assertEqual(summary["method"], "sim3")
            self.assertEqual(summary["registration_model"], "sim3_estimated_to_truth")
            self.assertEqual(summary["runtime_profile_family"], "estimated_to_truth_sim3")
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_select_profile_rejects_unvalidated_profile_by_default(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            (catalog_dir / "nominal_runtime_profile.json").write_text("{}", encoding="utf-8")
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "nominal",
                                "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": False,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaises(ValueError):
                module.select_profile(
                    catalog_json=catalog_json,
                    scenario="nominal",
                    output_profile=root / "active.json",
                )

    def test_find_profile_row_reports_known_scenarios(self) -> None:
        module = load_selector_module()

        with self.assertRaisesRegex(ValueError, "nominal"):
            module.find_profile_row(
                {
                    "profiles": [
                        {
                            "scenario": "nominal",
                        }
                    ]
                },
                "missing",
            )

    def test_deployment_strategy_selects_single_runtime_profile_without_scenario(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile_dir = root / "robust_policy_profiles"
            profile_dir.mkdir()
            source_profile = profile_dir / "balanced_runtime_profile.json"
            source_profile.write_text('{"mode": "estimated_to_truth_sim3"}\n', encoding="utf-8")
            strategy_json = root / "deployment_strategy_recommendation.json"
            strategy_json.write_text(
                json.dumps(
                    {
                        "deployment_strategy": "single_runtime_profile",
                        "selected_policy": "balanced",
                        "best_transfer_method": "sim3_binned_residual_disparity",
                        "best_transfer_pass_count": 4,
                        "best_transfer_total_count": 4,
                        "selection_contract": "no runtime truth",
                    }
                ),
                encoding="utf-8",
            )
            output_profile = root / "active_runtime_profile.json"

            summary = module.select_profile_from_deployment_strategy(
                deployment_strategy_json=strategy_json,
                output_profile=output_profile,
            )

            self.assertTrue(output_profile.is_file())
            self.assertEqual(output_profile.read_text(encoding="utf-8"), source_profile.read_text(encoding="utf-8"))
            self.assertEqual(summary["selection_mode"], "deployment_strategy_single_runtime_profile")
            self.assertEqual(summary["policy"], "balanced")
            self.assertEqual(summary["residual_model"], "forward_disparity_lut")
            self.assertEqual(summary["runtime_profile_family"], "estimated_to_truth_sim3")
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_method_contract_fields_classify_piecewise_sim3_runtime_profile(self) -> None:
        module = load_selector_module()

        summary = module.method_contract_fields("sim3_piecewise_raw_forward_bins5")

        self.assertEqual(summary["registration_model"], "piecewise_raw_forward_sim3_estimated_to_truth")
        self.assertEqual(summary["residual_model"], "none")
        self.assertIn("raw_forward_piecewise_registration", summary["regularization_model"])
        self.assertEqual(summary["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertIn("raw RDF forward", summary["selection_note"])

    def test_method_contract_fields_classify_inverse_depth_ray_profile(self) -> None:
        module = load_selector_module()

        summary = module.method_contract_fields("global_inverse_depth_linear")
        sim3_summary = module.method_contract_fields("target_weighted_inverse_depth_linear_then_sim3")
        shrunk_summary = module.method_contract_fields("target_shrunk_global_inverse_depth_linear_shrink050")

        self.assertEqual(summary["registration_model"], "raw_rdf_inverse_depth_ray_scale")
        self.assertEqual(summary["residual_model"], "corrected_inverse_depth_linear")
        self.assertEqual(summary["runtime_profile_family"], "estimated_to_truth_disparity_poly")
        self.assertEqual(
            sim3_summary["registration_model"],
            "inverse_depth_ray_scale_then_sim3_estimated_to_truth",
        )
        self.assertEqual(sim3_summary["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertIn("target_range_weighting", sim3_summary["regularization_model"])
        self.assertEqual(shrunk_summary["runtime_profile_family"], "estimated_to_truth_disparity_poly")
        self.assertIn("target_weight_shrinkage", shrunk_summary["regularization_model"])

    def test_deployment_strategy_catalog_requires_explicit_scenario(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            strategy_json = root / "deployment_strategy_recommendation.json"
            strategy_json.write_text(
                json.dumps({"deployment_strategy": "scenario_profile_catalog"}),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "requires --scenario"):
                module.select_profile_from_deployment_strategy(
                    deployment_strategy_json=strategy_json,
                    output_profile=root / "active_runtime_profile.json",
                )

    def test_deployment_strategy_catalog_selects_explicit_scenario_profile(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            source_profile = catalog_dir / "nominal_runtime_profile.json"
            source_profile.write_text('{"mode": "estimated_to_truth_sim3"}\n', encoding="utf-8")
            (catalog_dir / "manifest.json").write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "nominal",
                                "method": "sim3_binned_residual_disparity",
                                "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            strategy_json = root / "deployment_strategy_recommendation.json"
            strategy_json.write_text(
                json.dumps({"deployment_strategy": "scenario_profile_catalog"}),
                encoding="utf-8",
            )
            output_profile = root / "active_runtime_profile.json"

            summary = module.select_profile_from_deployment_strategy(
                deployment_strategy_json=strategy_json,
                scenario="nominal",
                output_profile=output_profile,
            )

            self.assertTrue(output_profile.is_file())
            self.assertEqual(summary["selection_mode"], "deployment_strategy_scenario_profile_catalog")
            self.assertEqual(summary["scenario"], "nominal")
            self.assertEqual(summary["deployment_strategy"], "scenario_profile_catalog")

    def test_deployment_strategy_selects_single_pooled_runtime_profile(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile_dir = root / "pooled_scenario_profile" / "pipeline" / "runtime_profiles"
            profile_dir.mkdir(parents=True)
            source_profile = profile_dir / "target_shrunk_global_inverse_depth_linear_shrink025.json"
            source_profile.write_text('{"mode": "estimated_to_truth_disparity_poly"}\n', encoding="utf-8")
            strategy_json = root / "deployment_strategy_recommendation.json"
            strategy_json.write_text(
                json.dumps(
                    {
                        "deployment_strategy": "single_pooled_runtime_profile",
                        "pooled_profile_method": "target_shrunk_global_inverse_depth_linear_shrink025",
                        "pooled_profile_path": (
                            "pooled_scenario_profile/pipeline/runtime_profiles/"
                            "target_shrunk_global_inverse_depth_linear_shrink025.json"
                        ),
                        "pooled_profile_transfer_pass_count": 6,
                        "pooled_profile_transfer_total_count": 6,
                        "pooled_profile_promotion_pass": True,
                        "selection_contract": "no runtime truth",
                    }
                ),
                encoding="utf-8",
            )
            output_profile = root / "active_runtime_profile.json"

            summary = module.select_profile_from_deployment_strategy(
                deployment_strategy_json=strategy_json,
                output_profile=output_profile,
            )

            self.assertEqual(output_profile.read_text(encoding="utf-8"), source_profile.read_text(encoding="utf-8"))
            self.assertEqual(summary["selection_mode"], "deployment_strategy_single_pooled_runtime_profile")
            self.assertEqual(summary["deployment_strategy"], "single_pooled_runtime_profile")
            self.assertEqual(summary["runtime_profile_family"], "estimated_to_truth_disparity_poly")
            self.assertEqual(summary["transfer_pass_count"], 6)

    def test_deployment_strategy_selects_single_near_tie_runtime_profile(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            source_profile = root / "scenario_near_tie_best_runtime_profile.json"
            source_profile.write_text('{"mode": "estimated_to_truth_sim3"}\n', encoding="utf-8")
            strategy_json = root / "deployment_strategy_recommendation.json"
            strategy_json.write_text(
                json.dumps(
                    {
                        "deployment_strategy": "single_near_tie_runtime_profile",
                        "best_near_tie_method": "sim3_piecewise_raw_forward_bins3",
                        "best_near_tie_profile_path": "scenario_near_tie_best_runtime_profile.json",
                        "best_near_tie_transfer_pass_count": 6,
                        "best_near_tie_transfer_total_count": 6,
                        "best_near_tie_promotion_pass": True,
                    }
                ),
                encoding="utf-8",
            )
            output_profile = root / "active_runtime_profile.json"

            summary = module.select_profile_from_deployment_strategy(
                deployment_strategy_json=strategy_json,
                output_profile=output_profile,
            )

            self.assertEqual(output_profile.read_text(encoding="utf-8"), source_profile.read_text(encoding="utf-8"))
            self.assertEqual(summary["selection_mode"], "deployment_strategy_single_near_tie_runtime_profile")
            self.assertEqual(summary["deployment_strategy"], "single_near_tie_runtime_profile")
            self.assertEqual(summary["runtime_profile_family"], "estimated_to_truth_sim3")

    def test_minimal_set_maps_requested_scenario_to_covering_source_profile(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            nominal_profile = catalog_dir / "nominal_runtime_profile.json"
            nominal_profile.write_text('{"profile": "nominal"}\n', encoding="utf-8")
            intrinsic_profile = catalog_dir / "intrinsic_scale_stress_runtime_profile.json"
            intrinsic_profile.write_text('{"profile": "intrinsic"}\n', encoding="utf-8")
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "nominal",
                                "method": "sim3_binned_residual_disparity",
                                "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                            {
                                "scenario": "intrinsic_scale_stress",
                                "method": "sim3_binned_residual_disparity",
                                "catalog_profile_path": "scenario_profile_catalog/intrinsic_scale_stress_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            minimal_set_json = root / "scenario_profile_catalog_minimal_set.json"
            minimal_set_json.write_text(
                json.dumps(
                    {
                        "pass_minimal_catalog_profile_set": True,
                        "selected_profile_count": 2,
                        "covered_target_scenarios": ["combined_sensor_stress", "intrinsic_scale_stress"],
                        "selection_contract": "offline set cover; no runtime truth",
                        "rows": [
                            {
                                "source_scenario": "nominal",
                                "newly_covered_target_scenarios": "combined_sensor_stress",
                                "all_pass_target_scenarios": "combined_sensor_stress",
                            },
                            {
                                "source_scenario": "intrinsic_scale_stress",
                                "newly_covered_target_scenarios": "intrinsic_scale_stress",
                                "all_pass_target_scenarios": "combined_sensor_stress,intrinsic_scale_stress",
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            output_profile = root / "active_runtime_profile.json"

            summary = module.select_profile_from_minimal_set(
                catalog_json=catalog_json,
                minimal_set_json=minimal_set_json,
                scenario="combined_sensor_stress",
                output_profile=output_profile,
            )

            self.assertEqual(output_profile.read_text(encoding="utf-8"), nominal_profile.read_text(encoding="utf-8"))
            self.assertEqual(summary["requested_scenario"], "combined_sensor_stress")
            self.assertEqual(summary["scenario"], "nominal")
            self.assertEqual(summary["selection_mode"], "scenario_profile_catalog_minimal_set")
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_deployment_strategy_catalog_uses_minimal_set_when_provided(self) -> None:
        module = load_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            source_profile = catalog_dir / "intrinsic_scale_stress_runtime_profile.json"
            source_profile.write_text('{"profile": "intrinsic"}\n', encoding="utf-8")
            (catalog_dir / "manifest.json").write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "intrinsic_scale_stress",
                                "method": "sim3_binned_residual_disparity",
                                "catalog_profile_path": "scenario_profile_catalog/intrinsic_scale_stress_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            minimal_set_json = root / "scenario_profile_catalog_minimal_set.json"
            minimal_set_json.write_text(
                json.dumps(
                    {
                        "pass_minimal_catalog_profile_set": True,
                        "selected_profile_count": 1,
                        "covered_target_scenarios": ["intrinsic_scale_stress"],
                        "rows": [
                            {
                                "source_scenario": "intrinsic_scale_stress",
                                "newly_covered_target_scenarios": "intrinsic_scale_stress",
                                "all_pass_target_scenarios": "intrinsic_scale_stress",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            strategy_json = root / "deployment_strategy_recommendation.json"
            strategy_json.write_text(
                json.dumps({"deployment_strategy": "scenario_profile_catalog"}),
                encoding="utf-8",
            )
            output_profile = root / "active_runtime_profile.json"

            summary = module.select_profile_from_deployment_strategy(
                deployment_strategy_json=strategy_json,
                minimal_set_json=minimal_set_json,
                scenario="intrinsic_scale_stress",
                output_profile=output_profile,
            )

            self.assertEqual(output_profile.read_text(encoding="utf-8"), source_profile.read_text(encoding="utf-8"))
            self.assertEqual(summary["selection_mode"], "deployment_strategy_scenario_profile_catalog_minimal_set")
            self.assertEqual(summary["requested_scenario"], "intrinsic_scale_stress")
            self.assertEqual(summary["deployment_strategy"], "scenario_profile_catalog")


if __name__ == "__main__":
    unittest.main()
