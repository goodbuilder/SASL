from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import pytest


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_operator_recording_install_overlay_readiness.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "check_operator_recording_install_overlay_readiness_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.usefixtures("isolated_operator_overlay_root")
class CheckOperatorRecordingInstallOverlayReadinessTests(unittest.TestCase):
    def test_installed_operator_recorder_schema_matches_current_public_frd_contract(self) -> None:
        module = load_module()

        result = module.check_readiness(self.isolated_operator_overlay_root)

        self.assertEqual(result["status"], "PASS")
        self.assertFalse(result["runtime_truth_used"])
        self.assertEqual(result["installed_axis_contract"], module.PUBLIC_AXIS_CONTRACT)
        self.assertIn("public FRD+range", result["coordinate_contract"])
        recorder = result["installed_operator_recorder"]
        self.assertTrue(Path(recorder["module_file"]).is_relative_to(self.isolated_operator_overlay_root))
        installed_fields = set(recorder["estimated_position_fields"])
        installed_fields.update(recorder["corrected_position_fields"])
        installed_fields.update(recorder["detection_fields"])
        for field in module.REQUIRED_FIELDS:
            self.assertIn(field, installed_fields)


if __name__ == "__main__":
    unittest.main()
