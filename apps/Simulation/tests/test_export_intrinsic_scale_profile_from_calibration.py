from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_export_module():
    spec = importlib.util.spec_from_file_location(
        "export_intrinsic_scale_profile_from_calibration_under_test",
        ROOT_DIR / "scripts" / "export_intrinsic_scale_profile_from_calibration.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load export_intrinsic_scale_profile_from_calibration.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportIntrinsicScaleProfileFromCalibrationTests(unittest.TestCase):
    def test_skips_when_product_ratio_guard_does_not_detect_gap(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "calibration.csv"
            input_csv.write_text(
                "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,left_u_px,left_v_px,truth_right_m,truth_down_m,truth_forward_m\n"
                "1,2,1000,1000,20,640,360,1,2,1000\n",
                encoding="utf-8",
            )
            args = type(
                "Args",
                (),
                {
                    "input_csv": input_csv,
                    "output_dir": root / "out",
                    "output_profile": root / "active.json",
                    "target_range_m": 1852.0,
                    "target_window_m": 30.0,
                    "target_error_limit_m": 10.0,
                    "train_ratio": 0.8,
                    "seed": 1,
                    "product_ratio_p10_threshold": 999.0,
                    "force": False,
                    "pipeline_script": root / "pipeline.py",
                },
            )()

            with patch.object(module, "run_pipeline") as run_pipeline:
                summary = module.export_intrinsic_scale_profile(args)

            run_pipeline.assert_not_called()
            self.assertFalse(summary["selected"])
            self.assertEqual(summary["skip_reason"], "intrinsic_scale_gap_not_detected")
            self.assertTrue((args.output_dir / "intrinsic_scale_profile_export_summary.json").is_file())

    def test_runs_pipeline_and_copies_selected_runtime_profile_when_gap_is_detected(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "calibration.csv"
            input_csv.write_text(
                "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,left_u_px,left_v_px,truth_right_m,truth_down_m,truth_forward_m\n"
                "1,2,1000,1000,20,640,360,1,2,1000\n",
                encoding="utf-8",
            )
            output_profile = root / "active.json"
            args = type(
                "Args",
                (),
                {
                    "input_csv": input_csv,
                    "output_dir": root / "out",
                    "output_profile": output_profile,
                    "target_range_m": 1852.0,
                    "target_window_m": 30.0,
                    "target_error_limit_m": 10.0,
                    "train_ratio": 0.8,
                    "seed": 1,
                    "product_ratio_p10_threshold": 0.01,
                    "force": False,
                    "pipeline_script": root / "pipeline.py",
                },
            )()

            def fake_run_pipeline(**kwargs):
                fit_dir = kwargs["fit_dir"]
                profile = fit_dir / "sim3_runtime_profile.json"
                profile.parent.mkdir(parents=True)
                profile.write_text('{"profile":"fit"}\n', encoding="utf-8")
                (fit_dir / "run_config.json").write_text(
                    json.dumps({"recommendation": "sim3", "recommendation_reason": "pass"}),
                    encoding="utf-8",
                )
                return type("Completed", (), {"stdout": "ok", "stderr": ""})()

            with patch.object(module, "run_pipeline", side_effect=fake_run_pipeline) as run_pipeline:
                summary = module.export_intrinsic_scale_profile(args)

            run_pipeline.assert_called_once()
            self.assertTrue(summary["selected"])
            self.assertEqual(summary["recommended_method"], "sim3")
            self.assertEqual(output_profile.read_text(encoding="utf-8"), '{"profile":"fit"}\n')


if __name__ == "__main__":
    unittest.main()
