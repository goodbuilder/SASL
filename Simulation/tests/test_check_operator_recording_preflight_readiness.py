from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_operator_recording_preflight_readiness.py"


def load_module():
    spec = importlib.util.spec_from_file_location("check_operator_recording_preflight_readiness_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckOperatorRecordingPreflightReadinessTests(unittest.TestCase):
    def test_current_operator_recording_path_is_preflight_ready(self) -> None:
        module = load_module()

        result = module.check_readiness(ROOT_DIR)

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["record_lite_supported"])
        self.assertTrue(result["headless_supported"])
        self.assertTrue(result["operator_recorder_entrypoint_declared"])
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("--record-lite", result["recommended_record_command"])
        self.assertIn("public FRD+range", result["coordinate_contract"])
        schema = result["recorder_schema"]
        self.assertEqual(schema["axis_contract"], module.PUBLIC_AXIS_CONTRACT)
        for field in ("nominal_pose_forward_m", "nominal_pose_right_m", "nominal_pose_down_m"):
            self.assertIn(field, schema["estimated_position_fields"])
        for field in ("corrected_forward_m", "corrected_right_m", "corrected_down_m"):
            self.assertIn(field, schema["corrected_position_fields"])
        self.assertEqual(schema["ambiguous_fields"], [])


if __name__ == "__main__":
    unittest.main()
