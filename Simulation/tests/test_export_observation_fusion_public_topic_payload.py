from __future__ import annotations

import importlib.util
import json
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_observation_fusion_public_topic_payload.py"
SPEC = importlib.util.spec_from_file_location("export_observation_fusion_public_topic_payload_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ExportObservationFusionPublicTopicPayloadTests(unittest.TestCase):
    def valid_row(self) -> dict[str, object]:
        raw_range = (4.0**2 + 1.5**2 + 800.0**2) ** 0.5
        corrected_range = (6.0**2 + 2.0**2 + 799.0**2) ** 0.5
        return {
            "group_id": "g0",
            "runtime_truth_used": "False",
            "rdf_axis_contract": module.output_validator.RDF_AXIS_CONTRACT,
            "public_topic_contract": module.output_validator.PUBLIC_TOPIC_CONTRACT,
            "raw_right_m": 4.0,
            "raw_down_m": 1.5,
            "raw_forward_m": 800.0,
            "raw_range_m": raw_range,
            "raw_public_forward_m": 800.0,
            "raw_public_right_m": 4.0,
            "raw_public_down_m": 1.5,
            "raw_public_range_m": raw_range,
            "corrected_right_m": 6.0,
            "corrected_down_m": 2.0,
            "corrected_forward_m": 799.0,
            "corrected_range_m": corrected_range,
            "corrected_public_forward_m": 799.0,
            "corrected_public_right_m": 6.0,
            "corrected_public_down_m": 2.0,
            "corrected_public_range_m": corrected_range,
        }

    def test_exports_float32_multi_array_payload_in_frd_order(self) -> None:
        rows, failures = module.payload_rows([self.valid_row()], tolerance_m=1.0e-6)

        self.assertEqual(failures, [])
        self.assertEqual(rows[0]["axis_order"], "forward,right,down,range")
        self.assertEqual(rows[0]["topic"], "/stereo/corrected_position")
        self.assertEqual(json.loads(rows[0]["float32_multi_array_data_json"])[:3], [799.0, 6.0, 2.0])
        self.assertFalse(rows[0]["runtime_truth_used"])

    def test_rejects_swapped_public_axis(self) -> None:
        row = self.valid_row()
        row["corrected_public_forward_m"] = row["corrected_right_m"]

        rows, failures = module.payload_rows([row], tolerance_m=1.0e-6)

        self.assertEqual(rows, [])
        self.assertTrue(any("corrected_public_forward_m" in failure for failure in failures))


if __name__ == "__main__":
    unittest.main()
