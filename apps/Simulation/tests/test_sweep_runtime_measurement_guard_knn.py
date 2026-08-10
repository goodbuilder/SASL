from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_knn_sweep_module():
    spec = importlib.util.spec_from_file_location(
        "sweep_runtime_measurement_guard_knn_under_test",
        ROOT_DIR / "scripts" / "sweep_runtime_measurement_guard_knn.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load sweep_runtime_measurement_guard_knn.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SweepRuntimeMeasurementGuardKnnTests(unittest.TestCase):
    def test_sweep_knn_ranks_passing_lower_rmse_neighbor_count(self) -> None:
        module = load_knn_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)

            def fake_run_guard(args):
                args.output_dir.mkdir(parents=True, exist_ok=True)
                args.output_dir.joinpath("runtime_measurement_guard_audit.csv").write_text(
                    "fallback_reason\nfallback\n\n",
                    encoding="utf-8",
                )
                better = args.knn_neighbor_count == 5
                return {
                    "audit_count": 3,
                    "pass_count": 3 if better else 2,
                    "pass_all": better,
                    "validation_range_rmse_corrected_mean_m": 1.0 if better else 2.0,
                    "validation_range_rmse_corrected_max_m": 1.5 if better else 2.5,
                    "target_abs_mean_error_mean_m": 0.8,
                    "target_abs_mean_error_max_m": 1.2,
                    "target_max_abs_error_max_m": 2.0 if better else 11.0,
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
                    "neighbor_counts": [3, 5],
                    "candidate_source_mode": "all_catalog",
                    "split": "validation",
                    "target_range_m": 1852.0,
                    "target_window_m": 30.0,
                    "target_error_limit_m": 10.0,
                    "transfer_low_risk_rmse_limit_m": 1.3,
                    "transfer_low_risk_target_limit_m": 2.5,
                    "transfer_risk_target_weight": 0.10,
                    "include_target_in_centroids": False,
                    "allow_unvalidated": False,
                },
            )()

            with patch.object(module, "run_guard", side_effect=fake_run_guard):
                summary = module.sweep_knn(args)

            self.assertEqual(summary["recommended_knn_neighbor_count"], 5)
            self.assertTrue((args.output_dir / "runtime_measurement_guard_knn_sweep.csv").is_file())
            self.assertTrue((args.output_dir / "runtime_measurement_guard_knn_sweep_summary.txt").is_file())
            self.assertTrue((args.output_dir / "runtime_measurement_guard_knn_sweep.png").is_file())


if __name__ == "__main__":
    unittest.main()
