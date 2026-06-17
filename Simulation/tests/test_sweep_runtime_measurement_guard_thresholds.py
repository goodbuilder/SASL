from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_sweep_module():
    spec = importlib.util.spec_from_file_location(
        "sweep_runtime_measurement_guard_thresholds_under_test",
        ROOT_DIR / "scripts" / "sweep_runtime_measurement_guard_thresholds.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load sweep_runtime_measurement_guard_thresholds.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SweepRuntimeMeasurementGuardThresholdTests(unittest.TestCase):
    def test_parse_float_list_accepts_comma_values(self) -> None:
        module = load_sweep_module()

        self.assertEqual(module.parse_float_list("1.0, 2.5,3"), [1.0, 2.5, 3.0])

    def test_sweep_thresholds_writes_ranked_summary(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)

            def fake_run_guard(args):
                self.assertTrue(args.exclude_target_scenario_from_library)
                args.output_dir.mkdir(parents=True, exist_ok=True)
                passed = args.transfer_low_risk_rmse_limit_m >= 1.0 and args.transfer_low_risk_target_limit_m >= 2.0
                args.output_dir.joinpath("runtime_measurement_guard_audit.csv").write_text(
                    "transfer_confusion_cluster,fallback_reason\n"
                    "nominal,because\n",
                    encoding="utf-8",
                )
                return {
                    "audit_count": 2,
                    "pass_count": 2 if passed else 1,
                    "pass_all": passed,
                    "validation_range_rmse_corrected_mean_m": 1.0 if passed else 2.0,
                    "validation_range_rmse_corrected_max_m": 1.4 if passed else 2.5,
                    "target_abs_mean_error_mean_m": 0.8,
                    "target_abs_mean_error_max_m": 1.2,
                    "target_max_abs_error_max_m": 2.0 if passed else 12.0,
                    "transfer_low_risk_rmse_limit_m": args.transfer_low_risk_rmse_limit_m,
                    "transfer_low_risk_target_limit_m": args.transfer_low_risk_target_limit_m,
                }

            args = type(
                "Args",
                (),
                {
                    "sweep_dir": root / "sweep",
                    "catalog_json": root / "catalog.json",
                    "output_dir": root / "out",
                    "minimal_set_json": None,
                    "transfer_matrix_csv": None,
                    "rmse_limits_m": [0.9, 1.1],
                    "target_limits_m": [1.5, 2.5],
                    "candidate_source_mode": "all_catalog",
                    "split": "validation",
                    "target_range_m": 1852.0,
                    "target_window_m": 30.0,
                    "target_error_limit_m": 10.0,
                    "transfer_risk_target_weight": 0.10,
                    "include_target_in_centroids": False,
                    "exclude_target_scenario_from_library": True,
                    "allow_unvalidated": False,
                },
            )()

            with patch.object(module, "run_guard", side_effect=fake_run_guard):
                summary = module.sweep_thresholds(args)

            self.assertEqual(summary["recommended_rmse_limit_m"], 1.1)
            self.assertEqual(summary["recommended_target_limit_m"], 2.5)
            self.assertTrue(summary["exclude_target_scenario_from_library"])
            self.assertTrue((args.output_dir / "runtime_measurement_guard_threshold_sweep.csv").is_file())
            self.assertTrue((args.output_dir / "runtime_measurement_guard_threshold_sweep_summary.txt").is_file())
            self.assertTrue((args.output_dir / "runtime_measurement_guard_threshold_sweep_heatmap.png").is_file())


if __name__ == "__main__":
    unittest.main()
