from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import pytest


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_installed_operator_recorder_csv_smoke.py"


def load_module():
    spec = importlib.util.spec_from_file_location("run_installed_operator_recorder_csv_smoke_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.usefixtures("isolated_operator_overlay_root")
class RunInstalledOperatorRecorderCsvSmokeTests(unittest.TestCase):
    def test_installed_recorder_writes_preflight_compatible_public_frd_csv(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir) / "smoke"

            result = module.run(
                [
                    "--root-dir",
                    str(self.isolated_operator_overlay_root),
                    "--output-dir",
                    str(output_dir),
                ]
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["installed_axis_contract"], module.PUBLIC_AXIS_CONTRACT)
        self.assertEqual(result["schema_audit_status"], "PASS")
        self.assertEqual(result["schema_audit_valid_row_count"], 1)
        self.assertFalse(result["first_row_has_axis_contract"])
        self.assertEqual(result["first_row_raw_forward_m"], "1000")
        self.assertEqual(result["first_row_corrected_forward_m"], "999")
        self.assertEqual(result["first_row_raw_gps_baseline_forward_m"], "1002")
        self.assertEqual(result["first_row_global_position_baseline_forward_m"], "999")
        self.assertEqual(result["first_row_raw_minus_global_baseline_delta_m"], "3.64005494")
        # The public recorder derives raw disparity from the recorded BBox centres
        # (700 - 690), rather than preserving an earlier standalone disparity sample.
        self.assertEqual(result["first_row_raw_disparity_px"], "10")
        self.assertTrue(
            Path(result["installed_module_file"]).is_relative_to(self.isolated_operator_overlay_root)
        )
        self.assertFalse(result["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
