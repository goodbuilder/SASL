from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_weight_module():
    spec = importlib.util.spec_from_file_location(
        "sweep_runtime_measurement_guard_target_weight_under_test",
        ROOT_DIR / "scripts" / "sweep_runtime_measurement_guard_target_weight.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load sweep_runtime_measurement_guard_target_weight.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SweepRuntimeMeasurementGuardTargetWeightTests(unittest.TestCase):
    def test_sweep_target_weights_ranks_lower_regret_passing_weight(self) -> None:
        module = load_weight_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)

            def fake_run_guard(args):
                args.output_dir.mkdir(parents=True, exist_ok=True)
                args.output_dir.joinpath("runtime_measurement_guard_audit.csv").write_text(
                    "input_csv\nplaceholder.csv\n",
                    encoding="utf-8",
                )
                return {
                    "pass_count": 2,
                    "audit_count": 2,
                    "validation_range_rmse_corrected_mean_m": 1.0,
                    "validation_range_rmse_corrected_max_m": 1.2,
                    "target_abs_mean_error_max_m": 1.0,
                    "target_max_abs_error_max_m": 2.0,
                }

            def fake_audit_oracle_regret(args):
                better = "0p200" in str(args.output_dir)
                args.output_dir.mkdir(parents=True, exist_ok=True)
                return {
                    "selected_oracle_match_count": 1 if better else 0,
                    "rmse_regret_mean_m": 0.0,
                    "rmse_regret_max_m": 0.0,
                    "target_abs_regret_mean_m": 0.1 if better else 0.3,
                    "target_abs_regret_max_m": 0.2 if better else 0.6,
                    "target_max_regret_mean_m": 0.1 if better else 0.3,
                    "target_max_regret_max_m": 0.2 if better else 0.6,
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
                    "weights": [0.1, 0.2],
                    "candidate_source_mode": "all_catalog",
                    "split": "validation",
                    "target_range_m": 1852.0,
                    "target_window_m": 30.0,
                    "target_error_limit_m": 10.0,
                    "transfer_low_risk_rmse_limit_m": 1.3,
                    "transfer_low_risk_target_limit_m": 2.5,
                    "include_target_in_centroids": False,
                    "allow_unvalidated": False,
                },
            )()

            with patch.object(module, "run_guard", side_effect=fake_run_guard), patch.object(
                module, "audit_oracle_regret", side_effect=fake_audit_oracle_regret
            ):
                summary = module.sweep_target_weights(args)

            self.assertEqual(summary["recommended_transfer_risk_target_weight"], 0.2)
            self.assertTrue((args.output_dir / "runtime_measurement_guard_target_weight_sweep.csv").is_file())
            self.assertTrue((args.output_dir / "runtime_measurement_guard_target_weight_sweep_summary.txt").is_file())
            self.assertTrue((args.output_dir / "runtime_measurement_guard_target_weight_sweep.png").is_file())


if __name__ == "__main__":
    unittest.main()
