from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


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


class RunInstalledOperatorRecorderCsvSmokeTests(unittest.TestCase):
    def test_installed_recorder_writes_preflight_compatible_public_frd_csv(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir) / "smoke"

            result = module.run(["--output-dir", str(output_dir)])

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["installed_axis_contract"], module.PUBLIC_AXIS_CONTRACT)
        self.assertEqual(result["schema_audit_status"], "PASS")
        self.assertEqual(result["schema_audit_valid_row_count"], 1)
        self.assertEqual(result["first_row_axis_contract"], module.PUBLIC_AXIS_CONTRACT)
        self.assertEqual(result["first_row_raw_forward_m"], "1000")
        self.assertEqual(result["first_row_corrected_forward_m"], "999")
        self.assertEqual(result["first_row_raw_disparity_px"], "200")
        self.assertFalse(result["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
