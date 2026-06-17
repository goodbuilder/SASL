from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "audit_runtime_profile_fresh_seed_replay.py"
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import rtk_stereo_calibration_pipeline as pipe  # noqa: E402


def load_module():
    spec = importlib.util.spec_from_file_location("audit_runtime_profile_fresh_seed_replay_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_runtime_profile_fresh_seed_replay.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditRuntimeProfileFreshSeedReplayTests(unittest.TestCase):
    def test_camera_and_injection_for_scenario_applies_stress_contract(self) -> None:
        module = load_module()

        camera, injection = module.camera_and_injection_for_scenario("combined_sensor_stress")

        self.assertAlmostEqual(camera.baseline_scale_error, 0.012)
        self.assertAlmostEqual(camera.focal_scale_error, -0.008)
        self.assertAlmostEqual(injection.yaw_deg, 0.85)
        self.assertAlmostEqual(injection.disparity_bias_px, 0.60)
        self.assertAlmostEqual(injection.bbox_noise_px, 0.30)

    def test_sample_payload_contains_runtime_rdf_and_disparity_only(self) -> None:
        module = load_module()
        sample = pipe.CalibrationSample(
            sample_id=1,
            t_s=0.2,
            split="validation",
            truth_ned_m=np.array([100.0, 2.0, 3.0], dtype=float),
            truth_rdf_m=np.array([2.0, 3.0, 100.0], dtype=float),
            raw_rdf_m=np.array([2.5, 3.5, 101.0], dtype=float),
            left_u_px=640.0,
            left_v_px=360.0,
            measured_disparity_px=12.0,
            true_disparity_px=11.5,
        )

        payload = module.sample_payload(sample)

        self.assertEqual(payload["right_m"], 2.5)
        self.assertEqual(payload["down_m"], 3.5)
        self.assertEqual(payload["forward_m"], 101.0)
        self.assertEqual(payload["disparity_px"], 12.0)
        self.assertEqual(payload["left_u_px"], 640.0)
        self.assertEqual(payload["left_v_px"], 360.0)
        self.assertNotIn("truth_right_m", payload)
        self.assertNotIn("truth_north_m", payload)
        self.assertNotIn("true_disparity_px", payload)


if __name__ == "__main__":
    unittest.main()
