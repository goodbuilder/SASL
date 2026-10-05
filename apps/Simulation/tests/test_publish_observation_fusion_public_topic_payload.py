from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "publish_observation_fusion_public_topic_payload.py"
SPEC = importlib.util.spec_from_file_location("publish_observation_fusion_public_topic_payload_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class PublishObservationFusionPublicTopicPayloadTests(unittest.TestCase):
    def write_payload_csv(self, path: Path, payload_json: str = "[3.0,4.0,12.0,13.0]") -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "group_id",
                    "topic",
                    "message_type",
                    "axis_order",
                    "public_topic_contract",
                    "forward_m",
                    "right_m",
                    "down_m",
                    "range_m",
                    "float32_multi_array_data_json",
                    "runtime_truth_used",
                ],
            )
            writer.writeheader()
            writer.writerow(
                {
                    "group_id": "g0",
                    "topic": "/stereo/corrected_position",
                    "message_type": "std_msgs/msg/Float32MultiArray",
                    "axis_order": "forward,right,down,range",
                    "public_topic_contract": module.payload_validator.PUBLIC_TOPIC_CONTRACT,
                    "forward_m": "3.0",
                    "right_m": "4.0",
                    "down_m": "12.0",
                    "range_m": "13.0",
                    "float32_multi_array_data_json": payload_json,
                    "runtime_truth_used": "False",
                }
            )

    def test_dry_run_replays_valid_payload(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "payload.csv"
            self.write_payload_csv(input_csv)

            summary = module.run(
                input_csv=input_csv,
                output_dir=root / "out",
                topic="/stereo/corrected_position",
                rate_hz=10.0,
                once=True,
                dry_run=True,
            )

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["payload_count"], 1)
        self.assertEqual(summary["payloads"][0], [3.0, 4.0, 12.0, 13.0])
        self.assertFalse(summary["runtime_truth_used"])

    def test_dry_run_rejects_invalid_payload_order(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "payload.csv"
            self.write_payload_csv(input_csv, payload_json="[4.0,3.0,12.0,13.0]")

            summary = module.run(
                input_csv=input_csv,
                output_dir=root / "out",
                topic="/stereo/corrected_position",
                rate_hz=10.0,
                once=True,
                dry_run=True,
            )

        self.assertEqual(summary["status"], "FAIL")
        self.assertIn("row0:payload_order_or_value_mismatch", summary["failures"])


if __name__ == "__main__":
    unittest.main()
