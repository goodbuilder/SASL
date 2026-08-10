from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "validate_observation_fusion_runtime_output_schema.py"
SPEC = importlib.util.spec_from_file_location("validate_observation_fusion_runtime_output_schema_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ValidateObservationFusionRuntimeOutputSchemaTests(unittest.TestCase):
    def valid_row(self) -> dict[str, object]:
        raw_right, raw_down, raw_forward = 4.0, 1.5, 800.0
        corrected_right, corrected_down, corrected_forward = 5.0, 2.0, 900.0
        raw_range = (raw_right**2 + raw_down**2 + raw_forward**2) ** 0.5
        corrected_range = (corrected_right**2 + corrected_down**2 + corrected_forward**2) ** 0.5
        return {
            "runtime_truth_used": "False",
            "rdf_axis_contract": module.RDF_AXIS_CONTRACT,
            "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
            "raw_right_m": raw_right,
            "raw_down_m": raw_down,
            "raw_forward_m": raw_forward,
            "raw_range_m": raw_range,
            "raw_public_forward_m": raw_forward,
            "raw_public_right_m": raw_right,
            "raw_public_down_m": raw_down,
            "raw_public_range_m": raw_range,
            "corrected_right_m": corrected_right,
            "corrected_down_m": corrected_down,
            "corrected_forward_m": corrected_forward,
            "corrected_range_m": corrected_range,
            "corrected_public_forward_m": corrected_forward,
            "corrected_public_right_m": corrected_right,
            "corrected_public_down_m": corrected_down,
            "corrected_public_range_m": corrected_range,
        }

    def test_validate_row_accepts_explicit_rdf_and_public_frd_aliases(self) -> None:
        failures = module.validate_row(self.valid_row(), row_index=0, tolerance_m=1.0e-6)

        self.assertEqual(failures, [])

    def test_validate_row_rejects_swapped_public_axes(self) -> None:
        row = self.valid_row()
        row["corrected_public_forward_m"] = row["corrected_right_m"]

        failures = module.validate_row(row, row_index=0, tolerance_m=1.0e-6)

        self.assertIn("row0:corrected_public_forward_m_axis_or_range_mismatch", failures)

    def test_validate_row_rejects_truth_runtime_flag(self) -> None:
        row = self.valid_row()
        row["runtime_truth_used"] = "True"

        failures = module.validate_row(row, row_index=0, tolerance_m=1.0e-6)

        self.assertIn("row0:runtime_truth_used_not_false", failures)


if __name__ == "__main__":
    unittest.main()
