from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "validate_observation_fusion_public_topic_payload.py"
SPEC = importlib.util.spec_from_file_location("validate_observation_fusion_public_topic_payload_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ValidateObservationFusionPublicTopicPayloadTests(unittest.TestCase):
    def valid_row(self) -> dict[str, object]:
        return {
            "group_id": "g0",
            "topic": "/stereo/corrected_position",
            "message_type": "std_msgs/msg/Float32MultiArray",
            "axis_order": "forward,right,down,range",
            "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
            "forward_m": 3.0,
            "right_m": 4.0,
            "down_m": 12.0,
            "range_m": 13.0,
            "float32_multi_array_data_json": "[3.0,4.0,12.0,13.0]",
            "runtime_truth_used": "False",
        }

    def test_validate_row_accepts_public_frd_payload(self) -> None:
        failures = module.validate_row(self.valid_row(), row_index=0, tolerance_m=1.0e-6)

        self.assertEqual(failures, [])

    def test_validate_row_rejects_payload_order_mismatch(self) -> None:
        row = self.valid_row()
        row["float32_multi_array_data_json"] = "[4.0,3.0,12.0,13.0]"

        failures = module.validate_row(row, row_index=0, tolerance_m=1.0e-6)

        self.assertIn("row0:payload_order_or_value_mismatch", failures)

    def test_validate_row_rejects_range_mismatch(self) -> None:
        row = self.valid_row()
        row["range_m"] = 99.0

        failures = module.validate_row(row, row_index=0, tolerance_m=1.0e-6)

        self.assertIn("row0:range_m_mismatch", failures)

    def write_csv(self, path: Path, rows: list[dict[str, object]]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    def test_validate_against_source_accepts_matching_payload(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            payload_csv = root / "payload.csv"
            source_csv = root / "source.csv"
            self.write_csv(payload_csv, [self.valid_row()])
            self.write_csv(
                source_csv,
                [
                    {
                        "group_id": "g0",
                        "corrected_public_forward_m": 3.0,
                        "corrected_public_right_m": 4.0,
                        "corrected_public_down_m": 12.0,
                        "corrected_public_range_m": 13.0,
                    }
                ],
            )

            result = module.validate_against_source(
                payload_csv=payload_csv,
                source_csv=source_csv,
                tolerance_m=1.0e-6,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)

    def test_validate_against_source_rejects_stale_payload(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            payload_csv = root / "payload.csv"
            source_csv = root / "source.csv"
            self.write_csv(payload_csv, [self.valid_row()])
            self.write_csv(
                source_csv,
                [
                    {
                        "group_id": "g0",
                        "corrected_public_forward_m": 30.0,
                        "corrected_public_right_m": 4.0,
                        "corrected_public_down_m": 12.0,
                        "corrected_public_range_m": 13.0,
                    }
                ],
            )

            result = module.validate_against_source(
                payload_csv=payload_csv,
                source_csv=source_csv,
                tolerance_m=1.0e-6,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("row0:forward_m_source_mismatch", result["failures"])


if __name__ == "__main__":
    unittest.main()
