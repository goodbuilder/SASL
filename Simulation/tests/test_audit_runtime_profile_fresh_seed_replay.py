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

    def test_trailing_temporal_filter_preserves_rdf_bearing_and_replaces_range_outlier(self) -> None:
        module = load_module()
        points = np.array(
            [
                [0.0, 0.0, 100.0],
                [0.0, 0.0, 101.0],
                [0.0, 0.0, 102.0],
                [0.0, 0.0, 150.0],
            ],
            dtype=float,
        )

        filtered, changed = module.apply_trailing_temporal_range_filter(
            points,
            window=3,
            threshold_m=10.0,
        )

        self.assertEqual(changed, 1)
        self.assertAlmostEqual(float(np.linalg.norm(filtered[3])), 103.0)
        self.assertAlmostEqual(filtered[3, 0], 0.0)
        self.assertAlmostEqual(filtered[3, 1], 0.0)
        self.assertGreater(filtered[3, 2], 0.0)

    def test_trailing_temporal_filter_disabled_by_default_contract(self) -> None:
        module = load_module()
        points = np.array([[0.0, 0.0, 100.0], [0.0, 0.0, 150.0]], dtype=float)

        filtered, changed = module.apply_trailing_temporal_range_filter(
            points,
            window=1,
            threshold_m=0.0,
        )

        self.assertEqual(changed, 0)
        np.testing.assert_allclose(filtered, points)

    def test_disparity_prefilter_replaces_outlier_and_rescales_raw_rdf_bearing(self) -> None:
        module = load_module()
        raw = np.array(
            [
                [0.0, 0.0, 100.0],
                [0.0, 0.0, 101.0],
                [0.0, 0.0, 102.0],
                [0.0, 0.0, 80.0],
            ],
            dtype=float,
        )
        disparities = np.array([10.0, 9.9, 9.8, 12.5], dtype=float)

        filtered_raw, filtered_disparity, changed = module.apply_trailing_disparity_prefilter(
            raw,
            disparities,
            window=3,
            threshold_px=0.5,
        )

        self.assertEqual(changed, 1)
        self.assertAlmostEqual(filtered_disparity[3], 9.7)
        self.assertAlmostEqual(filtered_raw[3, 0], 0.0)
        self.assertAlmostEqual(filtered_raw[3, 1], 0.0)
        self.assertAlmostEqual(filtered_raw[3, 2], 80.0 * 12.5 / 9.7)

    def test_disparity_prefilter_disabled_by_default_contract(self) -> None:
        module = load_module()
        raw = np.array([[0.0, 0.0, 100.0], [0.0, 0.0, 80.0]], dtype=float)
        disparities = np.array([10.0, 12.0], dtype=float)

        filtered_raw, filtered_disparity, changed = module.apply_trailing_disparity_prefilter(
            raw,
            disparities,
            window=1,
            threshold_px=0.0,
        )

        self.assertEqual(changed, 0)
        np.testing.assert_allclose(filtered_raw, raw)
        np.testing.assert_allclose(filtered_disparity, disparities)

    def test_disparity_prefilter_raw_range_window_limits_prefilter_scope(self) -> None:
        module = load_module()
        raw = np.array(
            [
                [0.0, 0.0, 100.0],
                [0.0, 0.0, 101.0],
                [0.0, 0.0, 102.0],
                [0.0, 0.0, 80.0],
            ],
            dtype=float,
        )
        disparities = np.array([10.0, 9.9, 9.8, 12.5], dtype=float)

        filtered_raw, filtered_disparity, changed = module.apply_trailing_disparity_prefilter(
            raw,
            disparities,
            window=3,
            threshold_px=0.5,
            target_range_m=100.0,
            raw_range_window_m=5.0,
        )

        self.assertEqual(changed, 0)
        np.testing.assert_allclose(filtered_raw, raw)
        np.testing.assert_allclose(filtered_disparity, disparities)

    def test_raw_range_prefilter_replaces_range_outlier_with_delta_median_prediction(self) -> None:
        module = load_module()
        raw = np.array(
            [
                [0.0, 0.0, 100.0],
                [0.0, 0.0, 110.0],
                [0.0, 0.0, 120.0],
                [0.0, 0.0, 130.0],
                [0.0, 0.0, 90.0],
            ],
            dtype=float,
        )

        filtered, changed = module.apply_trailing_raw_range_prefilter(
            raw,
            window=4,
            threshold_m=15.0,
        )

        self.assertEqual(changed, 1)
        self.assertAlmostEqual(float(np.linalg.norm(filtered[4])), 140.0)
        self.assertAlmostEqual(filtered[4, 0], 0.0)
        self.assertAlmostEqual(filtered[4, 1], 0.0)
        self.assertGreater(filtered[4, 2], 0.0)

    def test_raw_range_prefilter_target_window_limits_scope(self) -> None:
        module = load_module()
        raw = np.array(
            [
                [0.0, 0.0, 100.0],
                [0.0, 0.0, 110.0],
                [0.0, 0.0, 120.0],
                [0.0, 0.0, 130.0],
                [0.0, 0.0, 90.0],
            ],
            dtype=float,
        )

        filtered, changed = module.apply_trailing_raw_range_prefilter(
            raw,
            window=4,
            threshold_m=15.0,
            target_range_m=200.0,
            target_window_m=20.0,
        )

        self.assertEqual(changed, 0)
        np.testing.assert_allclose(filtered, raw)

    def test_alpha_beta_range_prefilter_replaces_outlier_and_preserves_bearing(self) -> None:
        module = load_module()
        raw = np.array(
            [
                [0.0, 0.0, 100.0],
                [0.0, 0.0, 110.0],
                [0.0, 0.0, 120.0],
                [0.0, 0.0, 130.0],
                [0.0, 0.0, 90.0],
            ],
            dtype=float,
        )
        times = np.arange(raw.shape[0], dtype=float)

        filtered, changed = module.apply_alpha_beta_range_prefilter(
            raw,
            times,
            gate_m=15.0,
            alpha=1.0,
            beta=1.0,
            min_history=3,
        )

        self.assertEqual(changed, 1)
        self.assertAlmostEqual(float(np.linalg.norm(filtered[4])), 140.0)
        self.assertAlmostEqual(filtered[4, 0], 0.0)
        self.assertAlmostEqual(filtered[4, 1], 0.0)
        self.assertGreater(filtered[4, 2], 0.0)

    def test_alpha_beta_range_prefilter_disabled_by_default_contract(self) -> None:
        module = load_module()
        raw = np.array([[0.0, 0.0, 100.0], [0.0, 0.0, 150.0]], dtype=float)
        times = np.array([0.0, 1.0], dtype=float)

        filtered, changed = module.apply_alpha_beta_range_prefilter(
            raw,
            times,
            gate_m=0.0,
            alpha=0.35,
            beta=0.08,
            min_history=5,
        )

        self.assertEqual(changed, 0)
        np.testing.assert_allclose(filtered, raw)


if __name__ == "__main__":
    unittest.main()
