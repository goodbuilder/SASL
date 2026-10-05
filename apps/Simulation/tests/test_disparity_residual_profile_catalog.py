from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
EXPORT_PATH = ROOT_DIR / "scripts" / "export_disparity_residual_profile_catalog.py"
SELECT_PATH = ROOT_DIR / "scripts" / "select_disparity_residual_profile_by_label.py"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


exporter = load_module("export_disparity_residual_profile_catalog_under_test", EXPORT_PATH)
selector = load_module("select_disparity_residual_profile_by_label_under_test", SELECT_PATH)


def write_profile(path: Path, profile_id: str, runtime_inputs: list[str] | None = None) -> None:
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
                "runtime_inputs": runtime_inputs
                or ["raw_right_m", "raw_down_m", "raw_forward_m", "measured_disparity_px"],
                "runtime_truth_used": False,
                "coordinate_contracts_explicit": True,
                "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            }
        ),
        encoding="utf-8",
    )


class DisparityResidualProfileCatalogTests(unittest.TestCase):
    def test_exports_and_selects_truth_free_residual_profile(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile_a = root / "a.json"
            profile_b = root / "b.json"
            write_profile(profile_a, "a")
            write_profile(profile_b, "b")

            result = exporter.export_catalog(
                output_dir=root / "catalog",
                default_label="a",
                profiles=[("a", profile_a), ("b", profile_b)],
                deployment_maps=["stress_a=a", "stress_b=b"],
                evidence=[],
            )
            selection = selector.select_residual_profile_by_label(
                selector_json=Path(result["selector_json"]),
                deployment_label="stress_b",
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(selection["status"], "PASS")
        self.assertEqual(selection["selected_label"], "b")
        self.assertEqual(selection["profile_id"], "b")
        self.assertFalse(selection["runtime_truth_used"])

    def test_export_rejects_truth_runtime_input(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "bad.json"
            write_profile(profile, "bad", runtime_inputs=["raw_forward_m", "px4_truth_forward_m"])

            result = exporter.export_catalog(
                output_dir=root / "catalog",
                default_label="bad",
                profiles=[("bad", profile)],
                deployment_maps=[],
                evidence=[],
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("profile:bad:runtime_inputs_truth", result["failures"])


if __name__ == "__main__":
    unittest.main()
