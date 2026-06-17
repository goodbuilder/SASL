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


if __name__ == "__main__":
    unittest.main()
