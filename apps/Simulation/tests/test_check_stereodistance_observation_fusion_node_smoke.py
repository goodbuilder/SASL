from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_stereodistance_observation_fusion_node_smoke.py"
SPEC = importlib.util.spec_from_file_location("check_stereodistance_observation_fusion_node_smoke_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class CheckStereoDistanceObservationFusionNodeSmokeTests(unittest.TestCase):
    def write_manifest(self, path: Path) -> None:
        path.write_text(
            json.dumps(
                {
                    "default_recommendation": "unit_multiburst",
                    "tiers": [
                        {
                            "tier": "unit_multiburst",
                            "adapter": {
                                "class": "MultiBurstObservationFusion",
                                "min_bursts": 2,
                                "min_observations_per_burst": 3,
                                "inner_mode": "median",
                                "outer_mode": "median",
                                "trim_fraction": 0.0,
                            },
                        }
                    ],
                    "runtime_truth_used": False,
                }
            )
            + "\n",
            encoding="utf-8",
        )

    def write_anchor_profile(self, path: Path) -> None:
        path.write_text(
            json.dumps(
                {
                    "mode": "origin_bbox_anchor",
                    "anchor": {
                        "left_x_fraction": 0.5,
                        "right_x_fraction": 0.5,
                        "left_y_fraction": 0.5,
                        "right_y_fraction": 0.5,
                    },
                }
            )
            + "\n",
            encoding="utf-8",
        )

    def write_residual_profile(self, path: Path) -> None:
        path.write_text(
            json.dumps(
                {
                    "profile_type": "DisparityResidualRuntimeProfile",
                    "profile_id": "unit_residual",
                    "feature_space": "disparity",
                    "correction_space": "ray_scale",
                    "degree": 0,
                    "feature_mean": 0.0,
                    "feature_scale": 1.0,
                    "coefficients_low_to_high": [0.01],
                    "correction_clip": 0.5,
                }
            )
            + "\n",
            encoding="utf-8",
        )

    def test_node_smoke_fuses_multiburst_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            manifest = Path(tmpdir) / "observation_fusion_runtime_manifest.json"
            self.write_manifest(manifest)

            result = module.run_smoke(manifest_json=manifest)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["adapter_class"], "MultiBurstObservationFusion")
        self.assertEqual(result["observation_count"], 6)
        self.assertEqual(result["observation_fusion_status"], "fused:MultiBurstObservationFusion:6")
        self.assertFalse(result["runtime_truth_used"])
        self.assertEqual(result["profile_status"]["observation_fusion_buffer_count"], 6)
        self.assertEqual(result["profile_status"]["observation_fusion_required_count"], 6)
        self.assertEqual(result["disparity_residual_feature_summary_status"], "ready:current_sample")
        self.assertIn("raw_forward_m_mean", result["disparity_residual_feature_summary"])
        self.assertEqual(
            result["profile_status"]["disparity_residual_feature_summary_status"],
            "ready:current_sample",
        )

    def test_node_smoke_applies_base_profile_then_disparity_residual(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = root / "observation_fusion_runtime_manifest.json"
            base_profile = root / "anchor_profile.json"
            residual_profile = root / "residual_profile.json"
            self.write_manifest(manifest)
            self.write_anchor_profile(base_profile)
            self.write_residual_profile(residual_profile)

            result = module.run_smoke(
                manifest_json=manifest,
                profile_json=base_profile,
                disparity_residual_profile_json=residual_profile,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["observation_fusion_status"], "fused:MultiBurstObservationFusion:6")
        self.assertTrue(result["disparity_residual_status"].startswith("applied:unit_residual"))
        self.assertTrue(result["profile_status"]["disparity_residual_status"].startswith("applied:unit_residual"))
        self.assertEqual(result["disparity_residual_feature_summary_status"], "ready:current_sample")
        self.assertFalse(result["runtime_truth_used"])

    def test_node_smoke_selects_residual_from_feature_selector(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = root / "observation_fusion_runtime_manifest.json"
            base_profile = root / "anchor_profile.json"
            residual_profile = root / "profiles" / "unit.json"
            selector_model = root / "selector_model.json"
            selector_catalog = root / "selector_catalog.json"
            self.write_manifest(manifest)
            self.write_anchor_profile(base_profile)
            residual_profile.parent.mkdir()
            self.write_residual_profile(residual_profile)
            selector_catalog.write_text(
                module.json.dumps({"profile_paths": {"unit": "profiles/unit.json"}}) + "\n",
                encoding="utf-8",
            )
            selector_model.write_text(
                module.json.dumps(
                    {
                        "feature_names": ["raw_forward_m_mean"],
                        "mean": [0.0],
                        "scale": [1.0],
                        "centroids": {"unit": [800.0]},
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = module.run_smoke(
                manifest_json=manifest,
                profile_json=base_profile,
                disparity_residual_feature_selector_json=selector_model,
                disparity_residual_feature_catalog_json=selector_catalog,
                disparity_residual_feature_selector_window_size=6,
                disparity_residual_feature_selector_min_samples=4,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["disparity_residual_feature_selector_summary_status"], "ready:runtime_window:6")
        self.assertEqual(result["disparity_residual_feature_selector_window_count"], 6)
        self.assertEqual(result["disparity_residual_feature_selector_status"], "selected:unit")
        self.assertEqual(result["disparity_residual_feature_selected_label"], "unit")
        self.assertTrue(result["disparity_residual_status"].startswith("applied:unit_residual"))
        self.assertEqual(result["profile_status"]["disparity_residual_feature_selector_status"], "selected:unit")
        self.assertEqual(result["profile_status"]["disparity_residual_feature_selector_summary_status"], "ready:runtime_window:6")

    def test_run_writes_json_and_text(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = root / "observation_fusion_runtime_manifest.json"
            output_json = root / "node_smoke.json"
            output_txt = root / "node_smoke.txt"
            self.write_manifest(manifest)

            result = module.run(
                [
                    "--manifest-json",
                    str(manifest),
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


if __name__ == "__main__":
    unittest.main()
