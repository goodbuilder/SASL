from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_audit_module():
    spec = importlib.util.spec_from_file_location(
        "estimated_to_truth_result_audit_under_test",
        ROOT_DIR / "scripts" / "estimated_to_truth_result_audit.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load estimated_to_truth_result_audit.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def touch(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("ok\n", encoding="utf-8")


class EstimatedToTruthResultAuditTests(unittest.TestCase):
    def test_audit_passes_complete_result_directory(self) -> None:
        module = load_audit_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            result_dir = Path(tmpdir)
            for relative in module.CORE_ARTIFACTS + module.RUNTIME_ARTIFACTS + module.CSV_PREFLIGHT_ARTIFACTS:
                touch(result_dir / relative)
            (result_dir / "metrics.json").write_text(
                json.dumps(
                    {
                        "status": "STRICT_PASS",
                        "scenario": "recorded_calibration_csv_px4_ned_to_stereo_rdf",
                        "method": "sim3",
                        "validation_range_rmse_raw_m": 5.0,
                        "validation_range_rmse_corrected_m": 1.0,
                        "validation_position_rmse_raw_m": 6.0,
                        "validation_position_rmse_corrected_m": 1.2,
                        "target_1852m_abs_mean_error_m": 0.5,
                        "target_1852m_tolerance_m": 10.0,
                        "target_1852m_sample_count": 4,
                        "target_1852m_min_sample_count": 3,
                        "runtime_truth_used": False,
                        "coordinate_contracts_explicit": True,
                        "alias_contracts_explicit": True,
                        "runtime_profile_exportable": True,
                        "stereodistance_applicable": True,
                        "preflight": {"pass_preflight": True},
                        "promotion_check": {"pass_promotion_check": True},
                        "gate_result": {"verdict": "STRICT_PASS"},
                    }
                ),
                encoding="utf-8",
            )

            audit = module.audit_result_dir(result_dir, require_csv_preflight=True)

        self.assertTrue(audit["pass_audit"])
        self.assertEqual(audit["missing_artifacts"], [])
        self.assertEqual(audit["failed_metric_checks"], [])

    def test_audit_fails_missing_promotion_check(self) -> None:
        module = load_audit_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            result_dir = Path(tmpdir)
            for relative in module.CORE_ARTIFACTS:
                touch(result_dir / relative)
            (result_dir / "metrics.json").write_text(
                json.dumps(
                    {
                        "status": "STRICT_PASS",
                        "scenario": "synthetic_rtk_px4_ned_to_stereo_rdf_1852m",
                        "validation_range_rmse_raw_m": 5.0,
                        "validation_range_rmse_corrected_m": 1.0,
                        "validation_position_rmse_raw_m": 6.0,
                        "validation_position_rmse_corrected_m": 1.2,
                        "target_1852m_abs_mean_error_m": 0.5,
                        "target_1852m_sample_count": 4,
                        "runtime_truth_used": False,
                        "coordinate_contracts_explicit": True,
                        "alias_contracts_explicit": True,
                        "runtime_profile_exportable": True,
                        "stereodistance_applicable": True,
                        "promotion_check": {},
                        "gate_result": {"verdict": "STRICT_PASS"},
                    }
                ),
                encoding="utf-8",
            )

            audit = module.audit_result_dir(result_dir)

        self.assertFalse(audit["pass_audit"])
        self.assertIn("promotion_check/promotion_check.json", audit["missing_artifacts"])
        self.assertIn("promotion_check_passed", audit["failed_metric_checks"])


if __name__ == "__main__":
    unittest.main()
