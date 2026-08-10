from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "summarize_runtime_calibration_required_gaps_under_test",
        ROOT_DIR / "scripts" / "summarize_runtime_calibration_required_gaps.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load summarize_runtime_calibration_required_gaps.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SummarizeRuntimeCalibrationRequiredGapsTests(unittest.TestCase):
    def test_flags_required_unavailable_and_unsafe_correction(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            audit_csv = root / "runtime_measurement_guard_audit.csv"
            audit_csv.write_text(
                "\n".join(
                    [
                        "target_scenario,target_seed,eligible_source_scenarios,selected_source_scenario,feature_disparity_forward_product_ratio_p10_to_nominal,pass_runtime_measurement_guard_replay,validation_only_pass_runtime_measurement_guard_replay,validation_range_rmse_corrected_m,validation_range_rmse_improvement_m,target_max_abs_error_corrected_m,fallback_reason,pass_reason",
                        "intrinsic_scale_stress,seed_1,nominal,nominal,1.0040,False,False,2.0,-0.1,5.0,,validation:no_positive_range_rmse_improvement",
                        "nominal,seed_1,nominal,intrinsic_scale_stress,1.0000,True,True,0.5,1.0,1.0,,pass",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            summary = module.summarize_calibration_required_gaps(
                audit_csv,
                root / "out",
                product_ratio_p10_threshold=1.0037,
                required_source_scenario="intrinsic_scale_stress",
                target_error_limit_m=10.0,
            )

            self.assertEqual(summary["calibration_required_count"], 1)
            self.assertEqual(summary["unsafe_correction_count"], 1)
            self.assertEqual(summary["calibration_required_target_within_limit_count"], 1)
            rows = (root / "out" / "runtime_calibration_required_gap_rows.csv").read_text(encoding="utf-8")
            self.assertIn("calibration_required_detected", rows)
            summary_json = json.loads(
                (root / "out" / "runtime_calibration_required_gap_summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(summary_json["axis_contract"], module.RDF_AXIS_CONTRACT)


if __name__ == "__main__":
    unittest.main()
