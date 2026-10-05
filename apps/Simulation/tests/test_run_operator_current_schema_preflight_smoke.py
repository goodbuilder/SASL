from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_operator_current_schema_preflight_smoke.py"


def load_module():
    spec = importlib.util.spec_from_file_location("run_operator_current_schema_preflight_smoke_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunOperatorCurrentSchemaPreflightSmokeTests(unittest.TestCase):
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
                    "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                    "translation_rdf_m": [0.0, 0.0, 0.0],
                }
            )
            + "\n",
            encoding="utf-8",
        )

    def write_selector(self, root: Path) -> tuple[Path, Path]:
        profile = root / "profiles" / "unit.json"
        profile.parent.mkdir(parents=True, exist_ok=True)
        profile.write_text(
            json.dumps(
                {
                    "profile_type": "DisparityResidualRuntimeProfile",
                    "profile_id": "unit",
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "measured_disparity_px"],
                    "runtime_truth_used": False,
                    "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                    "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                }
            )
            + "\n",
            encoding="utf-8",
        )
        model = root / "selector_model.json"
        model.write_text(
            json.dumps(
                {
                    "feature_names": ["raw_forward_m_mean"],
                    "mean": [0.0],
                    "scale": [1.0],
                    "centroids": {"unit": [403.4]},
                }
            )
            + "\n",
            encoding="utf-8",
        )
        catalog = root / "catalog.json"
        catalog.write_text(json.dumps({"profile_paths": {"unit": "profiles/unit.json"}}) + "\n", encoding="utf-8")
        return model, catalog

    def test_current_schema_smoke_runs_preflight_and_readiness(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            active_profile = root / "active_runtime_profile.json"
            output_dir = root / "out"
            self.write_profile(active_profile)
            model, catalog = self.write_selector(root)

            result = module.run(
                [
                    "--profile-json",
                    str(active_profile),
                    "--selector-model-json",
                    str(model),
                    "--catalog-json",
                    str(catalog),
                    "--output-dir",
                    str(output_dir),
                    "--expected-label",
                    "unit",
                    "--require-stable-label",
                    "--min-distance-margin",
                    "0.1",
                ]
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["preflight_status"], "PASS")
        self.assertEqual(result["readiness_status"], "PASS")
        self.assertTrue(result["apply_residual_allowed"])
        self.assertEqual(result["recommended_residual_label"], "unit")
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("public FRD+range", result["coordinate_contract"])


if __name__ == "__main__":
    unittest.main()
