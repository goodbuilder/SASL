from __future__ import annotations

import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))


def load_module():
    spec = importlib.util.spec_from_file_location(
        "runtime_deployment_selector_under_test",
        ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "runtime_deployment_selector.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load runtime_deployment_selector.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RuntimeDeploymentSelectorTests(unittest.TestCase):
    def test_public_frd_range_adapter_reorders_to_internal_rdf(self) -> None:
        module = load_module()

        sample = module.RuntimeMeasurementSample.from_public_frd_range(
            [1000.0, 8.0, 4.0, 1000.04],
            measured_disparity_px=12.0,
            left_u_px=640.0,
            left_v_px=360.0,
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.raw_right_m, 8.0)
        self.assertAlmostEqual(sample.raw_down_m, 4.0)
        self.assertAlmostEqual(sample.raw_forward_m, 1000.0)
        self.assertAlmostEqual(sample.raw_range_m, 1000.04)
        self.assertAlmostEqual(sample.measured_disparity_px, 12.0)
        self.assertAlmostEqual(sample.left_u_px, 640.0)
        self.assertAlmostEqual(sample.left_v_px, 360.0)

    def test_mapping_adapter_accepts_named_public_frd_range(self) -> None:
        module = load_module()
        row = {
            "estimated_frd_range": [200.0, -3.0, 1.5, 200.03],
            "raw_disparity_px": 14.0,
            "left_cx_px": 641.0,
            "left_cy_px": 363.0,
        }

        sample = module.RuntimeMeasurementSample.from_mapping(row)

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.raw_right_m, -3.0)
        self.assertAlmostEqual(sample.raw_down_m, 1.5)
        self.assertAlmostEqual(sample.raw_forward_m, 200.0)
        self.assertAlmostEqual(sample.measured_disparity_px, 14.0)

    def test_feature_window_computes_raw_rdf_disparity_features(self) -> None:
        module = load_module()
        window = module.RuntimeFeatureWindow(max_samples=10)
        window.extend_from_rows(
            [
                {
                    "raw_right_m": 1.0,
                    "raw_down_m": 2.0,
                    "raw_forward_m": 100.0,
                    "raw_range_m": 100.03,
                    "measured_disparity_px": 10.0,
                    "left_u_px": 640.0,
                    "left_v_px": 360.0,
                },
                {
                    "raw_right_m": 2.0,
                    "raw_down_m": 3.0,
                    "raw_forward_m": 200.0,
                    "raw_range_m": 200.03,
                    "measured_disparity_px": 5.0,
                    "left_u_px": 642.0,
                    "left_v_px": 364.0,
                },
            ]
        )

        features = window.features()

        self.assertEqual(window.sample_count, 2)
        self.assertAlmostEqual(features["raw_forward_mean_m"], 150.0)
        self.assertAlmostEqual(features["left_v_std_px"], 2.0)
        self.assertTrue(math.isfinite(features["disparity_forward_product_ratio_to_nominal"]))

    def test_disparity_residual_profile_can_use_raw_forward_feature(self) -> None:
        module = load_module()
        sample = module.RuntimeMeasurementSample(
            raw_right_m=2.0,
            raw_down_m=1.0,
            raw_forward_m=100.0,
            measured_disparity_px=12.0,
            raw_range_m=100.03,
            left_u_px=640.0,
            left_v_px=360.0,
        )

        profile = module.DisparityResidualRuntimeProfile(
            feature_space="raw_forward",
            correction_space="forward_add",
            degree=1,
            feature_mean=100.0,
            feature_scale=10.0,
            coefficients_low_to_high=(5.0, 0.0),
            correction_clip=10.0,
        )

        corrected = profile.apply(sample)

        self.assertIsNotNone(corrected)
        assert corrected is not None
        self.assertAlmostEqual(corrected.raw_right_m, 2.0)
        self.assertAlmostEqual(corrected.raw_down_m, 1.0)
        self.assertAlmostEqual(corrected.raw_forward_m, 105.0)
        self.assertAlmostEqual(corrected.measured_disparity_px, 12.0)

    def test_disparity_residual_profile_can_use_disparity_forward_product_feature(self) -> None:
        module = load_module()
        sample = module.RuntimeMeasurementSample(
            raw_right_m=2.0,
            raw_down_m=1.0,
            raw_forward_m=100.0,
            measured_disparity_px=12.0,
            raw_range_m=100.03,
            left_u_px=640.0,
            left_v_px=360.0,
        )

        profile = module.DisparityResidualRuntimeProfile(
            feature_space="disparity_forward_product",
            correction_space="ray_scale",
            degree=1,
            feature_mean=1200.0,
            feature_scale=100.0,
            coefficients_low_to_high=(0.1, 0.0),
            correction_clip=0.5,
        )

        corrected = profile.apply(sample)

        self.assertIsNotNone(corrected)
        assert corrected is not None
        self.assertAlmostEqual(corrected.raw_right_m, 2.2)
        self.assertAlmostEqual(corrected.raw_down_m, 1.1)
        self.assertAlmostEqual(corrected.raw_forward_m, 110.0)

    def test_disparity_residual_profile_can_use_forward_disparity_ratio_features(self) -> None:
        module = load_module()
        sample = module.RuntimeMeasurementSample(
            raw_right_m=2.0,
            raw_down_m=1.0,
            raw_forward_m=100.0,
            measured_disparity_px=10.0,
            raw_range_m=100.03,
            left_u_px=640.0,
            left_v_px=360.0,
        )

        profile = module.DisparityResidualRuntimeProfile(
            feature_space="forward_over_disparity",
            correction_space="forward_add",
            degree=1,
            feature_mean=10.0,
            feature_scale=1.0,
            coefficients_low_to_high=(3.0, 0.0),
            correction_clip=5.0,
        )
        corrected = profile.apply(sample)

        self.assertIsNotNone(corrected)
        assert corrected is not None
        self.assertAlmostEqual(corrected.raw_forward_m, 103.0)

        inverse_profile = module.DisparityResidualRuntimeProfile(
            feature_space="disparity_over_forward",
            correction_space="forward_add",
            degree=1,
            feature_mean=0.1,
            feature_scale=0.01,
            coefficients_low_to_high=(-2.0, 0.0),
            correction_clip=5.0,
        )
        inverse_corrected = inverse_profile.apply(sample)

        self.assertIsNotNone(inverse_corrected)
        assert inverse_corrected is not None
        self.assertAlmostEqual(inverse_corrected.raw_forward_m, 98.0)

    def test_runtime_image_observation_mapping_accepts_bbox_aliases(self) -> None:
        module = load_module()

        observation = module.RuntimeImageObservation.from_mapping(
            {
                "left_cx_px": 650.0,
                "left_cy_px": 355.0,
                "raw_disparity_px": 12.5,
                "truth_forward_m": 1000.0,
            }
        )

        self.assertIsNotNone(observation)
        assert observation is not None
        self.assertAlmostEqual(observation.left_u_px, 650.0)
        self.assertAlmostEqual(observation.left_v_px, 355.0)
        self.assertAlmostEqual(observation.measured_disparity_px, 12.5)

    def test_repeated_observation_median_fuses_to_internal_rdf(self) -> None:
        module = load_module()
        camera = module.RuntimeStereoCameraContract()
        fusion = module.RepeatedObservationFusion(camera=camera, min_observations=3, mode="median")
        forward_m = 1000.0
        right_m = 5.0
        down_m = -2.0
        disparity_px = camera.fx_px * camera.baseline_m / forward_m
        left_u_px = camera.cx_px + (right_m + camera.baseline_m * 0.5) * camera.fx_px / forward_m
        left_v_px = camera.cy_px + down_m * camera.fy_px / forward_m

        sample = fusion.fuse_rows(
            [
                {
                    "left_u_px": left_u_px,
                    "left_v_px": left_v_px,
                    "measured_disparity_px": disparity_px,
                },
                {
                    "left_u_px": left_u_px + 0.1,
                    "left_v_px": left_v_px - 0.1,
                    "measured_disparity_px": disparity_px,
                },
                {
                    "left_u_px": 1200.0,
                    "left_v_px": 40.0,
                    "measured_disparity_px": disparity_px * 0.2,
                },
            ]
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        expected_right = (left_u_px + 0.1 - camera.cx_px) * forward_m / camera.fx_px - camera.baseline_m * 0.5
        expected_down = (left_v_px - 0.1 - camera.cy_px) * forward_m / camera.fy_px
        self.assertAlmostEqual(sample.raw_forward_m, forward_m)
        self.assertAlmostEqual(sample.raw_right_m, expected_right, places=6)
        self.assertAlmostEqual(sample.raw_down_m, expected_down, places=6)
        self.assertEqual(sample.raw_range_m, math.sqrt(sample.raw_right_m**2 + sample.raw_down_m**2 + sample.raw_forward_m**2))

    def test_repeated_observation_trimmed_mean_requires_minimum_observations(self) -> None:
        module = load_module()
        camera = module.RuntimeStereoCameraContract()
        fusion = module.RepeatedObservationFusion(
            camera=camera,
            min_observations=5,
            mode="trimmed_mean",
            trim_fraction=0.2,
        )
        forward_m = 500.0
        disparity_px = camera.fx_px * camera.baseline_m / forward_m
        center = {
            "left_u_px": camera.cx_px + camera.baseline_m * 0.5 * camera.fx_px / forward_m,
            "left_v_px": camera.cy_px,
            "measured_disparity_px": disparity_px,
        }

        self.assertIsNone(fusion.fuse_rows([center, center, center]))

        sample = fusion.fuse_rows(
            [
                center,
                {**center, "left_u_px": center["left_u_px"] + 1.0},
                {**center, "left_u_px": center["left_u_px"] - 1.0},
                {**center, "left_u_px": center["left_u_px"] + 40.0},
                {**center, "left_u_px": center["left_u_px"] - 40.0},
            ]
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.raw_forward_m, forward_m)
        self.assertAlmostEqual(sample.raw_right_m, 0.0, places=6)

    def test_multiburst_observation_fusion_reconstructs_internal_rdf(self) -> None:
        module = load_module()
        camera = module.RuntimeStereoCameraContract()
        fusion = module.MultiBurstObservationFusion(
            camera=camera,
            min_bursts=3,
            min_observations_per_burst=5,
            inner_mode="trimmed_mean",
            outer_mode="median",
            trim_fraction=0.2,
        )
        forward_m = 800.0
        right_m = 4.0
        down_m = 1.5
        disparity_px = camera.fx_px * camera.baseline_m / forward_m
        left_u_px = camera.cx_px + (right_m + camera.baseline_m * 0.5) * camera.fx_px / forward_m
        left_v_px = camera.cy_px + down_m * camera.fy_px / forward_m

        def burst(common_u_px: float) -> list[dict[str, float]]:
            base = {
                "left_u_px": left_u_px + common_u_px,
                "left_v_px": left_v_px,
                "measured_disparity_px": disparity_px,
            }
            return [
                base,
                {**base, "left_u_px": base["left_u_px"] + 0.1},
                {**base, "left_u_px": base["left_u_px"] - 0.1},
                {**base, "left_u_px": base["left_u_px"] + 90.0},
                {**base, "left_u_px": base["left_u_px"] - 90.0},
            ]

        sample = fusion.fuse_row_groups([burst(-2.0), burst(0.0), burst(2.0)])

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.raw_forward_m, forward_m)
        self.assertAlmostEqual(sample.raw_right_m, right_m, places=6)
        self.assertAlmostEqual(sample.raw_down_m, down_m, places=6)

    def test_multiburst_observation_fusion_requires_enough_bursts(self) -> None:
        module = load_module()
        camera = module.RuntimeStereoCameraContract()
        fusion = module.MultiBurstObservationFusion(camera=camera, min_bursts=3, min_observations_per_burst=2)
        disparity_px = camera.fx_px * camera.baseline_m / 500.0
        row = {
            "left_u_px": camera.cx_px + camera.baseline_m * 0.5 * camera.fx_px / 500.0,
            "left_v_px": camera.cy_px,
            "measured_disparity_px": disparity_px,
        }

        self.assertIsNone(fusion.fuse_row_groups([[row, row], [row, row]]))

    def test_observation_fusion_adapter_factory_builds_repeated_adapter(self) -> None:
        module = load_module()

        adapter = module.build_observation_fusion_adapter(
            {
                "class": "RepeatedObservationFusion",
                "min_observations": 15,
                "mode": "trimmed_mean",
                "trim_fraction": 0.2,
            }
        )

        self.assertIsInstance(adapter, module.RepeatedObservationFusion)
        self.assertEqual(adapter.min_observations, 15)
        self.assertEqual(adapter.mode, "trimmed_mean")

    def test_observation_fusion_manifest_loader_uses_default_tier(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "manifest.json"
            path.write_text(
                json.dumps(
                    {
                        "default_recommendation": "common_mode_target_gate",
                        "runtime_truth_used": False,
                        "tiers": [
                            {
                                "tier": "balanced_independent",
                                "adapter": {
                                    "class": "RepeatedObservationFusion",
                                    "min_observations": 15,
                                    "mode": "trimmed_mean",
                                    "trim_fraction": 0.2,
                                },
                            },
                            {
                                "tier": "common_mode_target_gate",
                                "adapter": {
                                    "class": "MultiBurstObservationFusion",
                                    "min_bursts": 3,
                                    "min_observations_per_burst": 5,
                                    "inner_mode": "trimmed_mean",
                                    "outer_mode": "median",
                                    "trim_fraction": 0.2,
                                },
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )

            adapter, tier = module.load_observation_fusion_adapter_from_manifest(path)
            repeated, repeated_tier = module.load_observation_fusion_adapter_from_manifest(
                path,
                tier="balanced_independent",
            )

        self.assertIsInstance(adapter, module.MultiBurstObservationFusion)
        self.assertEqual(tier["tier"], "common_mode_target_gate")
        self.assertEqual(adapter.min_bursts, 3)
        self.assertEqual(adapter.min_observations_per_burst, 5)
        self.assertIsInstance(repeated, module.RepeatedObservationFusion)
        self.assertEqual(repeated_tier["tier"], "balanced_independent")

    def test_packaged_selector_applies_override_without_truth(self) -> None:
        module = load_module()
        selector = module.PackagedRuntimeDeploymentSelector(
            {
                "baseline_selector": "catalog_guard",
                "override_profile_label": "single_pooled",
                "runtime_rule": "use catalog_guard unless feature_left_v_std_px ge 10, then use single_pooled",
                "selection_contract": "raw features only",
            }
        )

        selected = selector.select({"left_v_std_px": 12.0})
        baseline = selector.select({"left_v_std_px": 8.0})

        self.assertEqual(selected["selected_label"], "single_pooled")
        self.assertTrue(selected["override_applied"])
        self.assertEqual(baseline["selected_label"], "catalog_guard")
        self.assertFalse(baseline["override_applied"])
        self.assertEqual(selected["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_packaged_selector_applies_raw_forward_guard_without_truth(self) -> None:
        module = load_module()
        selector = module.PackagedRuntimeDeploymentSelector(
            {
                "baseline_selector": "log_disparity_forward_add",
                "override_profile_label": "product_ray_scale_intrinsic_fallback",
                "runtime_rule": (
                    "use log_disparity_forward_add unless "
                    "feature_raw_forward_mean_m gt 1771.0, "
                    "then use product_ray_scale_intrinsic_fallback"
                ),
                "selection_contract": (
                    "runtime guard uses stereo-center RDF raw_forward_mean_m; "
                    "scenario labels and truth are forbidden"
                ),
            }
        )

        intrinsic_like = selector.select({"raw_forward_mean_m": 1773.2})
        nominal_like = selector.select({"raw_forward_mean_m": 1770.6})

        self.assertEqual(intrinsic_like["selected_label"], "product_ray_scale_intrinsic_fallback")
        self.assertTrue(intrinsic_like["override_applied"])
        self.assertEqual(nominal_like["selected_label"], "log_disparity_forward_add")
        self.assertFalse(nominal_like["override_applied"])
        self.assertEqual(intrinsic_like["axis_contract"], module.RDF_AXIS_CONTRACT)
        self.assertNotIn("truth", intrinsic_like["override_rule"]["feature_name"])

    def test_packaged_selector_loads_from_json_file(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "selector.json"
            path.write_text(
                json.dumps(
                    {
                        "baseline_selector": "catalog_guard",
                        "override_profile_label": "single_pooled",
                        "runtime_rule": "use catalog_guard unless feature_left_v_std_px ge 10, then use single_pooled",
                    }
                ),
                encoding="utf-8",
            )

            selector = module.PackagedRuntimeDeploymentSelector.from_json_file(path)

            self.assertEqual(selector.select({"left_v_std_px": 11.0})["selected_label"], "single_pooled")

    def test_packaged_selector_blocks_override_outside_validated_geometry(self) -> None:
        module = load_module()
        selector = module.PackagedRuntimeDeploymentSelector(
            {
                "baseline_selector": "catalog_guard",
                "override_profile_label": "small_drone_profile",
                "runtime_rule": "use catalog_guard unless feature_left_v_std_px ge 10, then use small_drone_profile",
                "validated_envelope": {
                    "baseline_m": [1.5, 2.5],
                    "camera_height_m": [1.0, 2.0],
                    "target_size_m": [0.3, 0.8],
                    "range_m": [20.0, 120.0],
                    "zoom_index": 4,
                },
            }
        )

        selected = selector.select(
            {
                "left_v_std_px": 12.0,
                "baseline_m": 2.0,
                "camera_height_m": 1.5,
                "target_size_m": 0.5,
                "raw_range_mean_m": 60.0,
                "zoom_index": 4,
            }
        )
        rejected = selector.select(
            {
                "left_v_std_px": 12.0,
                "baseline_m": 25.0,
                "camera_height_m": 10.0,
                "target_size_m": 15.0,
                "raw_range_mean_m": 1800.0,
                "zoom_index": 8,
            }
        )

        self.assertEqual(selected["selected_label"], "small_drone_profile")
        self.assertTrue(selected["scenario_envelope_ok"])
        self.assertEqual(rejected["selected_label"], "catalog_guard")
        self.assertFalse(rejected["override_applied"])
        self.assertFalse(rejected["scenario_envelope_ok"])
        self.assertIn("baseline_m_outside_envelope", rejected["scenario_envelope_failures"])

    def test_packaged_selector_uses_explicit_deployment_label_without_truth(self) -> None:
        module = load_module()
        selector = module.PackagedRuntimeDeploymentSelector(
            {
                "baseline_selector": "current_active",
                "override_profile_label": "feature_stress_profile",
                "runtime_rule": "use current_active unless feature_left_v_std_px ge 10, then use feature_stress_profile",
                "label_profile_map": {
                    "runtime_observation_stress": "stress_active",
                    "nominal": "current_active",
                },
                "selection_contract": "explicit deployment labels are operator/deployment metadata only; runtime truth is forbidden",
            }
        )

        selected = selector.select(
            {
                "deployment_label": "runtime_observation_stress",
                "left_v_std_px": 1.0,
                "truth_forward_m": 1852.0,
            }
        )
        fallback = selector.select(
            {
                "deployment_label": "unknown",
                "left_v_std_px": 1.0,
            }
        )

        self.assertEqual(selected["selected_label"], "stress_active")
        self.assertTrue(selected["label_selection_applied"])
        self.assertFalse(selected["override_applied"])
        self.assertEqual(selected["deployment_label"], "runtime_observation_stress")
        self.assertEqual(fallback["selected_label"], "current_active")
        self.assertFalse(fallback["label_selection_applied"])
        self.assertIn("operator/deployment metadata", selected["selection_contract"])

    def test_packaged_selector_blocks_label_selection_outside_validated_geometry(self) -> None:
        module = load_module()
        selector = module.PackagedRuntimeDeploymentSelector(
            {
                "baseline_selector": "current_active",
                "label_profile_map": {"small_drone_close": "small_drone_profile"},
                "validated_envelope": {
                    "baseline_m": [1.5, 2.5],
                    "range_m": [20.0, 120.0],
                },
            }
        )

        rejected = selector.select(
            {
                "deployment_label": "small_drone_close",
                "baseline_m": 25.0,
                "raw_range_mean_m": 1852.0,
            }
        )

        self.assertEqual(rejected["selected_label"], "current_active")
        self.assertFalse(rejected["label_selection_applied"])
        self.assertFalse(rejected["scenario_envelope_ok"])
        self.assertIn("baseline_m_outside_envelope", rejected["scenario_envelope_failures"])


if __name__ == "__main__":
    unittest.main()
