from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_mode_module():
    spec = importlib.util.spec_from_file_location(
        "audit_stereodistance_selector_operating_modes_under_test",
        ROOT_DIR / "scripts" / "audit_stereodistance_selector_operating_modes.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_stereodistance_selector_operating_modes.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditStereoDistanceSelectorOperatingModesTests(unittest.TestCase):
    def test_audit_operating_modes_joins_runtime_and_calibration_rows(self) -> None:
        module = load_mode_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            runtime_csv = root / "runtime.csv"
            runtime_csv.write_text(
                "target_scenario,target_seed,selected_source_scenario,selected_method,"
                "pass_runtime_measurement_guard_replay,validation_range_rmse_corrected_m,"
                "target_abs_mean_error_corrected_m,target_max_abs_error_corrected_m\n"
                "nominal,seed_1,bbox,sim3,True,1.2,0.8,2.0\n",
                encoding="utf-8",
            )
            calibration_csv = root / "calibration.csv"
            calibration_csv.write_text(
                "target_scenario,seed,calibration_source_scenario,calibration_pass,"
                "calibration_validation_range_rmse_corrected_m,calibration_target_abs_mean_error_corrected_m\n"
                "nominal,1,nominal,True,1.0,0.5\n",
                encoding="utf-8",
            )
            args = type(
                "Args",
                (),
                {
                    "runtime_guard_audit_csv": runtime_csv,
                    "calibration_replay_runs_csv": calibration_csv,
                    "output_dir": root / "out",
                },
            )()

            summary = module.audit_operating_modes(args)

            self.assertEqual(summary["row_count"], 1)
            self.assertEqual(summary["runtime_pass_count"], 1)
            self.assertEqual(summary["calibration_pass_count"], 1)
            self.assertEqual(summary["calibration_lower_rmse_count"], 1)
            self.assertTrue((args.output_dir / "selector_operating_mode_comparison.csv").is_file())
            self.assertTrue((args.output_dir / "selector_operating_mode_recommendation.json").is_file())
            self.assertTrue((args.output_dir / "selector_operating_mode_comparison.png").is_file())


if __name__ == "__main__":
    unittest.main()
