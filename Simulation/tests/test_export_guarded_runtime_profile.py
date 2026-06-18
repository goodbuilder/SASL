from __future__ import annotations

import importlib.util
import json
import math
import sys
import tempfile
import unittest
from argparse import Namespace
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_guarded_runtime_profile.py"


def load_module():
    spec = importlib.util.spec_from_file_location("export_guarded_runtime_profile_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load export_guarded_runtime_profile.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def base_args(input_profile: Path) -> Namespace:
    return Namespace(
        input_profile=input_profile,
        output_profile=Path("out.json"),
        output_summary=None,
        guard_mode="target_disparity_range_delta_cap",
        profile_id_suffix="guarded",
        target_range_m=1852.0,
        target_disparity_px=None,
        half_width_px=0.35,
        range_delta_cap_m=1.25,
        alpha_inside=0.75,
        alpha_outside=1.0,
        range_window_m=70.0,
        alpha_away=0.85,
        width_px=1280.0,
        horizontal_fov_rad=0.154997710698,
        baseline_m=25.0,
        baseline_scale_error=0.006,
        focal_scale_error=-0.004,
    )


class ExportGuardedRuntimeProfileTests(unittest.TestCase):
    def test_target_disparity_uses_measured_camera_contract(self) -> None:
        module = load_module()

        target_d = module.target_disparity_px(
            width_px=1280.0,
            horizontal_fov_rad=0.154997710698,
            focal_scale_error=-0.004,
            baseline_m=25.0,
            baseline_scale_error=0.006,
            target_range_m=1852.0,
        )

        fx_true = (1280.0 * 0.5) / math.tan(0.154997710698 * 0.5)
        self.assertAlmostEqual(target_d, fx_true * 0.996 * 25.0 * 1.006 / 1852.0)

    def test_guarded_profile_adds_cap_guard_without_truth_fields(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            path.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_sim3",
                        "profile_id": "base",
                        "scale": 1.0,
                        "rotation_matrix_rdf": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                        "translation_rdf_m": [0, 0, 0],
                    }
                ),
                encoding="utf-8",
            )

            payload = module.guarded_profile(base_args(path))

            self.assertEqual(payload["profile_id"], "base_guarded")
            guard = payload["target_disparity_range_delta_cap_guard"]
            self.assertAlmostEqual(guard["range_delta_cap_m"], 1.25)
            self.assertIn("measured disparity", payload["guard_export_contract"])
            self.assertNotIn("truth_right_m", payload)
            self.assertNotIn("truth_north_m", payload)

    def test_rejects_non_sim3_profile_for_current_guard_exporter(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            path.write_text(json.dumps({"mode": "estimated_to_truth_affine_rdf"}), encoding="utf-8")

            with self.assertRaises(ValueError):
                module.guarded_profile(base_args(path))


if __name__ == "__main__":
    unittest.main()
