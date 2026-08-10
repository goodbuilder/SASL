from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_blend_module():
    spec = importlib.util.spec_from_file_location(
        "explore_stereodistance_profile_blends_under_test",
        ROOT_DIR / "scripts" / "explore_stereodistance_profile_blends.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load explore_stereodistance_profile_blends.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExploreStereoDistanceProfileBlendsTests(unittest.TestCase):
    def test_blend_sim3_profile_exports_runtime_contract_metadata(self) -> None:
        module = load_blend_module()
        base_profile = {
            "mode": "estimated_to_truth_sim3",
            "profile_id": "a",
            "method": "sim3",
            "scale": 1.0,
            "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
            "translation_rdf_m": [0.0, 0.0, 0.0],
        }
        second_profile = {
            **base_profile,
            "profile_id": "b",
            "translation_rdf_m": [2.0, 4.0, 6.0],
        }

        blended = module.blend_sim3_profiles(
            base_profile,
            second_profile,
            alpha=0.25,
            profile_id="blend-a-b",
        )

        self.assertEqual(blended["mode"], "estimated_to_truth_sim3")
        self.assertEqual(blended["profile_family"], "estimated_to_truth_sim3")
        self.assertEqual(blended["rdf_axis_contract"], module.RDF_AXIS_CONTRACT)
        self.assertEqual(blended["public_topic_contract"], module.PUBLIC_TOPIC_CONTRACT)
        self.assertFalse(blended["runtime_truth_used"])
        self.assertEqual(blended["blend_source_profile_a_id"], "a")
        self.assertEqual(blended["blend_source_profile_b_id"], "b")
        self.assertEqual(blended["blend_alpha_second_profile"], 0.25)

    def test_blend_inverse_depth_profile_interpolates_coefficients_and_contract(self) -> None:
        module = load_blend_module()
        base_profile = {
            "mode": "estimated_to_truth_disparity_poly",
            "profile_id": "a",
            "method": "huber_global_inverse_depth_linear",
            "correction_space": "raw_rdf_inverse_depth_ray_scale",
            "inverse_depth_coefficients_high_to_low": [1.0, 2.0],
        }
        second_profile = {
            **base_profile,
            "profile_id": "b",
            "inverse_depth_coefficients_high_to_low": [3.0, 6.0],
        }

        blended = module.blend_inverse_depth_profiles(
            base_profile,
            second_profile,
            alpha=0.25,
            profile_id="blend-a-b",
        )

        self.assertEqual(blended["mode"], "estimated_to_truth_disparity_poly")
        self.assertEqual(blended["profile_family"], "estimated_to_truth_disparity_poly")
        self.assertEqual(blended["inverse_depth_coefficients_high_to_low"], [1.5, 3.0])
        self.assertEqual(blended["rdf_axis_contract"], module.RDF_AXIS_CONTRACT)
        self.assertEqual(blended["public_topic_contract"], module.PUBLIC_TOPIC_CONTRACT)
        self.assertFalse(blended["runtime_truth_used"])
        self.assertIn("px4_rtk_truth", blended["forbidden_runtime_inputs"])

    def test_pareto_frontier_keeps_rmse_target_tradeoff(self) -> None:
        module = load_blend_module()
        rows = [
            {
                "pass_replay": True,
                "alpha_second_profile": 0.0,
                "validation_range_rmse_corrected_m": 1.0,
                "target_abs_mean_error_corrected_m": 0.9,
                "stability_min_range_improvement_m": 1.0,
            },
            {
                "pass_replay": True,
                "alpha_second_profile": 0.2,
                "validation_range_rmse_corrected_m": 1.2,
                "target_abs_mean_error_corrected_m": 0.2,
                "stability_min_range_improvement_m": 0.9,
            },
            {
                "pass_replay": True,
                "alpha_second_profile": 0.4,
                "validation_range_rmse_corrected_m": 1.4,
                "target_abs_mean_error_corrected_m": 0.6,
                "stability_min_range_improvement_m": 0.8,
            },
        ]

        frontier = module.pareto_frontier(rows)

        self.assertEqual([row["alpha_second_profile"] for row in frontier], [0.0, 0.2])

    def test_policy_rows_expose_balanced_validation_and_target_choices(self) -> None:
        module = load_blend_module()
        rows = [
            {
                "pass_replay": True,
                "alpha_second_profile": 0.0,
                "validation_range_rmse_corrected_m": 1.0,
                "target_abs_mean_error_corrected_m": 0.9,
                "stability_min_range_improvement_m": 1.0,
                "replay_selection_score": 2.0,
            },
            {
                "pass_replay": True,
                "alpha_second_profile": 0.2,
                "validation_range_rmse_corrected_m": 1.2,
                "target_abs_mean_error_corrected_m": 0.2,
                "stability_min_range_improvement_m": 0.9,
                "replay_selection_score": 1.8,
            },
        ]

        policies = module.choose_policy_rows(rows)

        self.assertEqual(policies["balanced"]["alpha_second_profile"], 0.2)
        self.assertEqual(policies["validation_best"]["alpha_second_profile"], 0.0)
        self.assertEqual(policies["target_guard"]["alpha_second_profile"], 0.2)
        self.assertEqual(policies["simple_stable"]["alpha_second_profile"], 0.0)

    def test_explore_blends_all_catalog_mode_uses_profiles_outside_minimal_set(self) -> None:
        module = load_blend_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()

            def write_profile(name: str, translation_forward_m: float) -> None:
                (catalog_dir / f"{name}_runtime_profile.json").write_text(
                    json.dumps(
                        {
                            "mode": "estimated_to_truth_sim3",
                            "profile_id": name,
                            "method": "sim3",
                            "scale": 1.0,
                            "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                            "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                            "translation_rdf_m": [0.0, 0.0, translation_forward_m],
                        }
                    ),
                    encoding="utf-8",
                )

            for index, name in enumerate(("nominal", "intrinsic_scale_stress", "disparity_noise_stress")):
                write_profile(name, float(index))
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": name,
                                "method": "sim3",
                                "catalog_profile_path": f"scenario_profile_catalog/{name}_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            }
                            for name in ("nominal", "intrinsic_scale_stress", "disparity_noise_stress")
                        ]
                    }
                ),
                encoding="utf-8",
            )
            minimal_set_json = root / "scenario_profile_catalog_minimal_set.json"
            minimal_set_json.write_text(
                json.dumps(
                    {
                        "rows": [
                            {"source_scenario": "nominal"},
                            {"source_scenario": "intrinsic_scale_stress"},
                        ]
                    }
                ),
                encoding="utf-8",
            )
            input_csv = root / "samples.csv"
            input_csv.write_text("split\nvalidation\n", encoding="utf-8")

            seen_profiles: list[str] = []

            def fake_replay_profile_summary(**kwargs):
                profile_path = Path(str(kwargs["profile_path"]))
                seen_profiles.append(profile_path.name)
                uses_disparity_noise = "disparity_noise_stress" in profile_path.name
                return {
                    "pass_replay": True,
                    "pass_reason": "pass",
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": 0.5 if uses_disparity_noise else 1.5,
                    "validation_range_rmse_improvement_m": 2.5 if uses_disparity_noise else 1.5,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": 0.2 if uses_disparity_noise else 0.8,
                    "target_max_abs_error_corrected_m": 0.4 if uses_disparity_noise else 1.2,
                    "stability_min_range_improvement_m": 1.0,
                    "stability_max_target_abs_mean_error_m": 0.4,
                    "model_complexity_estimate": 16,
                    "replay_selection_score": 1.0 if uses_disparity_noise else 2.0,
                }

            with patch.object(module, "replay_profile_summary", side_effect=fake_replay_profile_summary):
                summary = module.explore_blends(
                    catalog_json=catalog_json,
                    input_csv=input_csv,
                    output_dir=root / "blend",
                    output_profile=root / "active_blend.json",
                    minimal_set_json=minimal_set_json,
                    split="validation",
                    stability_splits=["validation"],
                    target_range_m=1852.0,
                    target_window_m=30.0,
                    target_error_limit_m=10.0,
                    alphas=[0.0, 1.0],
                    candidate_source_mode="all_catalog",
                )

            self.assertEqual(summary["candidate_source_mode"], "all_catalog")
            self.assertIn("disparity_noise_stress", summary["candidate_source_scenarios"])
            self.assertTrue(any("disparity_noise_stress" in name for name in seen_profiles))
            self.assertIn("disparity_noise_stress", {summary["source_a"], summary["source_b"]})

    def test_explore_blends_exports_requested_policy_profile(self) -> None:
        module = load_blend_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()

            def write_profile(name: str, translation_forward_m: float) -> None:
                (catalog_dir / f"{name}_runtime_profile.json").write_text(
                    json.dumps(
                        {
                            "mode": "estimated_to_truth_sim3",
                            "profile_id": name,
                            "method": "sim3",
                            "scale": 1.0,
                            "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                            "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                            "translation_rdf_m": [0.0, 0.0, translation_forward_m],
                        }
                    ),
                    encoding="utf-8",
                )

            write_profile("a", 0.0)
            write_profile("b", 10.0)
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": name,
                                "method": "sim3",
                                "catalog_profile_path": f"scenario_profile_catalog/{name}_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            }
                            for name in ("a", "b")
                        ],
                    }
                ),
                encoding="utf-8",
            )
            input_csv = root / "samples.csv"
            input_csv.write_text("split\nvalidation\n", encoding="utf-8")

            def fake_replay_profile_summary(**kwargs):
                payload = json.loads(Path(str(kwargs["profile_path"])).read_text(encoding="utf-8"))
                alpha = float(payload["blend_alpha_second_profile"])
                target = abs(alpha - 1.0)
                return {
                    "pass_replay": True,
                    "pass_reason": "pass",
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": 0.5 + alpha,
                    "validation_range_rmse_improvement_m": 2.0,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": target,
                    "target_max_abs_error_corrected_m": target,
                    "stability_min_range_improvement_m": 1.0,
                    "stability_max_target_abs_mean_error_m": target,
                    "model_complexity_estimate": 16,
                    "replay_selection_score": 0.5 + alpha,
                }

            output_profile = root / "active_target_guard_blend.json"
            with patch.object(module, "replay_profile_summary", side_effect=fake_replay_profile_summary):
                summary = module.explore_blends(
                    catalog_json=catalog_json,
                    input_csv=input_csv,
                    output_dir=root / "blend",
                    output_profile=output_profile,
                    minimal_set_json=None,
                    split="validation",
                    stability_splits=["validation"],
                    target_range_m=1852.0,
                    target_window_m=30.0,
                    target_error_limit_m=10.0,
                    alphas=[0.0, 1.0],
                    selection_policy="target_guard",
                )

            exported = json.loads(output_profile.read_text(encoding="utf-8"))
            self.assertEqual(summary["selection_policy"], "target_guard")
            self.assertEqual(summary["alpha_second_profile"], 1.0)
            self.assertEqual(exported["blend_alpha_second_profile"], 1.0)


if __name__ == "__main__":
    unittest.main()
