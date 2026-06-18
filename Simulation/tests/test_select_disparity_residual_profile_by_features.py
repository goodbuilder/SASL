from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "select_disparity_residual_profile_by_features.py"
SPEC = importlib.util.spec_from_file_location("select_disparity_residual_profile_by_features_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def write_profile(path: Path, profile_id: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "profile_type": "DisparityResidualRuntimeProfile",
                "profile_id": profile_id,
                "method": f"{profile_id}_method",
                "feature_space": "log_disparity",
                "correction_space": "ray_scale",
                "degree": 2,
                "feature_mean": 5.0,
                "feature_scale": 1.0,
                "coefficients_low_to_high": [0.0, 0.01, 0.0],
                "correction_clip": 0.5,
                "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "measured_disparity_px"],
                "runtime_truth_used": False,
            }
        ),
        encoding="utf-8",
    )


class SelectDisparityResidualProfileByFeaturesTests(unittest.TestCase):
    def test_selects_catalog_profile_from_truth_free_features(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile_a = root / "profiles" / "a.json"
            profile_b = root / "profiles" / "b.json"
            write_profile(profile_a, "a")
            write_profile(profile_b, "b")
            catalog = root / "catalog.json"
            catalog.write_text(
                json.dumps({"profile_paths": {"a": "profiles/a.json", "b": "profiles/b.json"}}),
                encoding="utf-8",
            )
            model = root / "model.json"
            model.write_text(
                json.dumps(
                    {
                        "feature_names": ["measured_disparity_px_mean", "observation_forward_m_mean"],
                        "mean": [0.0, 0.0],
                        "scale": [1.0, 1.0],
                        "centroids": {"a": [1.0, 1.0], "b": [10.0, 10.0]},
                    }
                ),
                encoding="utf-8",
            )

            result = module.select_residual_profile_by_features(
                selector_model_json=model,
                catalog_json=catalog,
                feature_row={"measured_disparity_px_mean": 1.2, "observation_forward_m_mean": 0.8},
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["selected_label"], "a")
        self.assertEqual(result["profile_id"], "a")
        self.assertFalse(result["runtime_truth_used"])

    def test_rejects_truth_named_selector_feature(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "profiles" / "a.json"
            write_profile(profile, "a")
            catalog = root / "catalog.json"
            catalog.write_text(json.dumps({"profile_paths": {"a": "profiles/a.json"}}), encoding="utf-8")
            model = root / "model.json"
            model.write_text(
                json.dumps(
                    {
                        "feature_names": ["px4_truth_range_m"],
                        "mean": [0.0],
                        "scale": [1.0],
                        "centroids": {"a": [0.0]},
                    }
                ),
                encoding="utf-8",
            )

            result = module.select_residual_profile_by_features(
                selector_model_json=model,
                catalog_json=catalog,
                feature_row={"px4_truth_range_m": 0.0},
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("selector_model:feature_names_truth", result["failures"])


if __name__ == "__main__":
    unittest.main()
