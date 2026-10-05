from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "rtk_stereo_calibration_pipeline.py"


def load_module():
    spec = importlib.util.spec_from_file_location("rtk_stereo_calibration_pipeline_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RtkStereoCalibrationPipelineLocalSweepTests(unittest.TestCase):
    def test_local_sweep_config_adds_native_method(self) -> None:
        module = load_module()
        camera = module.CameraModel()
        source = module.SyntheticCalibrationDataSource(
            camera=camera,
            injection=module.ErrorInjection(),
            sample_count=96,
            seed=20260618,
            target_range_m=1852.0,
            target_window_m=30.0,
            target_oversample_count=8,
        )
        samples = source.load_samples()
        train, _ = module.split_samples(samples, train_ratio=0.8, seed=20260635, target_range_m=1852.0, target_window_m=30.0)
        config = {
            "mode": "rtk_stereo_local_sweep_candidate",
            "method": "sim3_inverse_depth_monotone_binned_residual_raw_forward_bins11_test",
            "registration_model": "sim3",
            "residual_basis": "inverse_depth",
            "bin_count": 11,
            "prior_strength_samples": 8.0,
            "residual_gain": 1.0,
            "target_weight": 0.0,
            "robust_loss": "none",
            "monotone": True,
            "forward_key": "raw_forward",
            "runtime_truth_used": False,
        }
        status_rows: list[dict[str, object]] = []
        methods = module.fit_methods(
            train,
            camera=camera,
            target_range_m=1852.0,
            local_sweep_configs=[config],
            local_sweep_status_rows=status_rows,
        )

        self.assertIn(config["method"], {method.name for method in methods})
        self.assertEqual(status_rows[0]["status"], "ADDED")
        method = next(method for method in methods if method.name == config["method"])
        profile = module.runtime_profile_for_method(method, experiment_id="unit")
        self.assertIsNotNone(profile)
        self.assertIn("raw_right_m", profile["runtime_inputs"])
        self.assertIn("raw_down_m", profile["runtime_inputs"])
        self.assertIn("raw_forward_m", profile["runtime_inputs"])
        self.assertIn("px4_rtk_truth", profile["forbidden_runtime_inputs"])

    def test_local_sweep_target_weight_config_adds_native_method(self) -> None:
        module = load_module()
        camera = module.CameraModel()
        source = module.SyntheticCalibrationDataSource(
            camera=camera,
            injection=module.ErrorInjection(),
            sample_count=96,
            seed=20260619,
            target_range_m=1852.0,
            target_window_m=30.0,
            target_oversample_count=8,
        )
        samples = source.load_samples()
        train, _ = module.split_samples(samples, train_ratio=0.8, seed=20260636, target_range_m=1852.0, target_window_m=30.0)
        config = {
            "mode": "rtk_stereo_local_sweep_candidate",
            "method": "sim3_inverse_depth_targetw0p25_binned_residual_raw_forward_bins11_test",
            "registration_model": "sim3",
            "residual_basis": "inverse_depth",
            "bin_count": 11,
            "prior_strength_samples": 8.0,
            "residual_gain": 1.0,
            "target_weight": 0.25,
            "robust_loss": "none",
            "monotone": False,
            "forward_key": "raw_forward",
            "runtime_truth_used": False,
        }
        status_rows: list[dict[str, object]] = []
        methods = module.fit_methods(
            train,
            camera=camera,
            target_range_m=1852.0,
            local_sweep_configs=[config],
            local_sweep_status_rows=status_rows,
        )

        self.assertIn(config["method"], {method.name for method in methods})
        self.assertEqual(status_rows[0]["status"], "ADDED")
        method = next(method for method in methods if method.name == config["method"])
        self.assertEqual(method.parameters["local_sweep_target_weight"], 0.25)
    
    def test_local_sweep_robust_loss_config_adds_native_method(self) -> None:
        module = load_module()
        camera = module.CameraModel()
        source = module.SyntheticCalibrationDataSource(
            camera=camera,
            injection=module.ErrorInjection(disparity_outlier_rate=0.05, disparity_outlier_px=0.4),
            sample_count=96,
            seed=20260620,
            target_range_m=1852.0,
            target_window_m=30.0,
            target_oversample_count=8,
        )
        samples = source.load_samples()
        train, _ = module.split_samples(samples, train_ratio=0.8, seed=20260637, target_range_m=1852.0, target_window_m=30.0)
        config = {
            "mode": "rtk_stereo_local_sweep_candidate",
            "method": "sim3_inverse_depth_huber_binned_residual_raw_forward_bins11_test",
            "registration_model": "sim3",
            "residual_basis": "inverse_depth",
            "bin_count": 11,
            "prior_strength_samples": 8.0,
            "residual_gain": 1.0,
            "target_weight": 0.0,
            "robust_loss": "huber",
            "monotone": False,
            "forward_key": "raw_forward",
            "runtime_truth_used": False,
        }
        status_rows: list[dict[str, object]] = []
        methods = module.fit_methods(
            train,
            camera=camera,
            target_range_m=1852.0,
            local_sweep_configs=[config],
            local_sweep_status_rows=status_rows,
        )

        self.assertIn(config["method"], {method.name for method in methods})
        self.assertEqual(status_rows[0]["status"], "ADDED")
        method = next(method for method in methods if method.name == config["method"])
        self.assertEqual(method.parameters["local_sweep_robust_fit"]["robust_loss"], "huber")

    def test_local_sweep_base_forward_profile_exports_query_basis(self) -> None:
        module = load_module()
        camera = module.CameraModel()
        source = module.SyntheticCalibrationDataSource(
            camera=camera,
            injection=module.ErrorInjection(),
            sample_count=96,
            seed=20260621,
            target_range_m=1852.0,
            target_window_m=30.0,
            target_oversample_count=8,
        )
        samples = source.load_samples()
        train, _ = module.split_samples(samples, train_ratio=0.8, seed=20260638, target_range_m=1852.0, target_window_m=30.0)
        config = {
            "mode": "rtk_stereo_local_sweep_candidate",
            "method": "sim3_inverse_depth_binned_residual_base_forward_bins11_test",
            "registration_model": "sim3",
            "residual_basis": "inverse_depth",
            "bin_count": 11,
            "prior_strength_samples": 8.0,
            "residual_gain": 1.0,
            "target_weight": 0.0,
            "robust_loss": "none",
            "monotone": False,
            "forward_key": "base_forward",
            "runtime_truth_used": False,
        }
        status_rows: list[dict[str, object]] = []
        methods = module.fit_methods(
            train,
            camera=camera,
            target_range_m=1852.0,
            local_sweep_configs=[config],
            local_sweep_status_rows=status_rows,
        )
        method = next(method for method in methods if method.name == config["method"])
        profile = module.runtime_profile_for_method(method, experiment_id="unit")

        self.assertEqual(status_rows[0]["status"], "ADDED")
        self.assertEqual(profile["forward_residual_raw_forward_query"], "base_forward")

    def test_local_sweep_monotone_range_and_blended_configs_add_native_methods(self) -> None:
        module = load_module()
        camera = module.CameraModel()
        source = module.SyntheticCalibrationDataSource(
            camera=camera,
            injection=module.ErrorInjection(),
            sample_count=96,
            seed=20260622,
            target_range_m=1852.0,
            target_window_m=30.0,
            target_oversample_count=8,
        )
        samples = source.load_samples()
        train, _ = module.split_samples(samples, train_ratio=0.8, seed=20260639, target_range_m=1852.0, target_window_m=30.0)
        configs = [
            {
                "mode": "rtk_stereo_local_sweep_candidate",
                "method": "sim3_range_normalized_monotone_binned_residual_raw_forward_bins11_test",
                "registration_model": "sim3",
                "residual_basis": "range_normalized",
                "bin_count": 11,
                "prior_strength_samples": 8.0,
                "residual_gain": 1.0,
                "target_weight": 0.0,
                "robust_loss": "none",
                "monotone": True,
                "forward_key": "raw_forward",
                "runtime_truth_used": False,
            },
            {
                "mode": "rtk_stereo_local_sweep_candidate",
                "method": "sim3_blended_disp_invdepth_monotone_huber_binned_residual_raw_forward_bins11_test",
                "registration_model": "sim3",
                "residual_basis": "blended_disparity_inverse_depth",
                "bin_count": 11,
                "prior_strength_samples": 8.0,
                "residual_gain": 1.0,
                "target_weight": 0.0,
                "robust_loss": "huber",
                "monotone": True,
                "forward_key": "raw_forward",
                "runtime_truth_used": False,
            },
        ]
        status_rows: list[dict[str, object]] = []
        methods = module.fit_methods(
            train,
            camera=camera,
            target_range_m=1852.0,
            local_sweep_configs=configs,
            local_sweep_status_rows=status_rows,
        )
        method_names = {method.name for method in methods}

        self.assertEqual([row["status"] for row in status_rows], ["ADDED", "ADDED"])
        self.assertIn(configs[0]["method"], method_names)
        self.assertIn(configs[1]["method"], method_names)

    def test_local_sweep_loader_reads_candidate_configs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            config_dir = Path(tmpdir)
            (config_dir / "001_candidate.json").write_text(
                json.dumps(
                    {
                        "mode": "rtk_stereo_local_sweep_candidate",
                        "method": "candidate",
                        "runtime_truth_used": False,
                    }
                ),
                encoding="utf-8",
            )
            configs = module.load_local_sweep_configs(config_dir)

        self.assertEqual(len(configs), 1)
        self.assertEqual(configs[0]["method"], "candidate")
        self.assertIn("_config_path", configs[0])


if __name__ == "__main__":
    unittest.main()
