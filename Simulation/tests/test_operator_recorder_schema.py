from __future__ import annotations

import csv
import importlib.util
import io
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "operator_recorder.py"
AUDIT_SCRIPT_PATH = ROOT_DIR / "scripts" / "audit_operator_telemetry_residual_preflight_schema.py"


def load_module():
    spec = importlib.util.spec_from_file_location("operator_recorder_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_audit_module():
    spec = importlib.util.spec_from_file_location("operator_telemetry_schema_audit_under_test", AUDIT_SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {AUDIT_SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeClockNow:
    @property
    def nanoseconds(self) -> int:
        return 123_456_789


class FakeClock:
    def now(self) -> FakeClockNow:
        return FakeClockNow()


class FakeMsg:
    def __init__(self, data):
        self.data = data


class OperatorRecorderSchemaTests(unittest.TestCase):
    def test_recorder_declares_current_public_frd_schema(self) -> None:
        module = load_module()

        self.assertEqual(module.AXIS_CONTRACT, "stereodistance_public_frd:forward,right,down,range")
        for field in (
            "raw_forward_m",
            "raw_right_m",
            "raw_down_m",
            "estimated_forward_m",
            "estimated_right_m",
            "estimated_down_m",
            "corrected_forward_m",
            "corrected_right_m",
            "corrected_down_m",
        ):
            self.assertIn(field, module.ESTIMATED_POSITION_FIELDS + module.CORRECTED_POSITION_FIELDS)

    def test_callbacks_and_csv_row_emit_preflight_required_fields(self) -> None:
        module = load_module()
        recorder = object.__new__(module.OperatorRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.start_monotonic = time.monotonic()
        recorder.get_clock = lambda: FakeClock()
        output = io.StringIO()
        fields = (
            ["time_sec", "ros_time_sec", "axis_contract"]
            + list(module.NUMERIC_TOPICS.values())
            + module.COMPARISON_FIELDS
            + module.GROUND_TRUTH_POSITION_FIELDS
            + module.ESTIMATED_POSITION_FIELDS
            + module.CORRECTED_POSITION_FIELDS
            + module.BBOX_ADJUSTMENT_FIELDS
            + module.DETECTION_FIELDS
            + module.PROFILE_STATUS_FIELDS
        )
        recorder.csv_file = output
        recorder.csv_writer = csv.DictWriter(output, fieldnames=fields)
        recorder.csv_writer.writeheader()

        recorder._estimated_position_cb(FakeMsg([1000.0, 7.0, 4.0, 1000.0325]))
        recorder._corrected_position_cb(FakeMsg([999.0, 6.5, 3.5, 999.0274]))
        recorder._float_cb(FakeMsg(200.0), "disparity_px", "/stereo/disparity")
        recorder._detections_cb(FakeMsg([1, 690, 380, 710, 400, 0.9, 700, 390, 1, 490, 380, 510, 400, 0.8, 500, 390]))
        recorder._profile_status_cb(
            FakeMsg(json.dumps({"profile_id": "unit", "profile_mode": "sim3", "fallback_level": 2}))
        )
        recorder._write_csv_row()
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))

        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row["axis_contract"], module.AXIS_CONTRACT)
        self.assertEqual(row["raw_forward_m"], "1000")
        self.assertEqual(row["raw_right_m"], "7")
        self.assertEqual(row["raw_down_m"], "4")
        self.assertEqual(row["estimated_forward_m"], "1000")
        self.assertEqual(row["corrected_forward_m"], "999")
        self.assertEqual(row["corrected_right_m"], "6.5")
        self.assertEqual(row["corrected_down_m"], "3.5")
        self.assertEqual(row["raw_disparity_px"], "200")
        self.assertEqual(row["right_valid"], "1")
        self.assertEqual(row["right_x1"], "490")
        self.assertEqual(row["right_x2"], "510")
        self.assertEqual(row["right_cx"], "500")
        self.assertEqual(row["right_width_px"], "20")
        self.assertEqual(row["profile_id"], "unit")
        self.assertEqual(row["profile_mode"], "sim3")
        self.assertEqual(row["profile_fallback_level"], "2")

    def test_recorder_csv_row_passes_residual_preflight_schema_audit(self) -> None:
        module = load_module()
        audit_module = load_audit_module()
        recorder = object.__new__(module.OperatorRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.start_monotonic = time.monotonic()
        recorder.get_clock = lambda: FakeClock()
        fields = (
            ["time_sec", "ros_time_sec", "axis_contract"]
            + list(module.NUMERIC_TOPICS.values())
            + module.COMPARISON_FIELDS
            + module.GROUND_TRUTH_POSITION_FIELDS
            + module.ESTIMATED_POSITION_FIELDS
            + module.CORRECTED_POSITION_FIELDS
            + module.BBOX_ADJUSTMENT_FIELDS
            + module.DETECTION_FIELDS
            + module.PROFILE_STATUS_FIELDS
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            telemetry = Path(tmpdir) / "telemetry.csv"
            with telemetry.open("w", newline="", encoding="utf-8") as handle:
                recorder.csv_file = handle
                recorder.csv_writer = csv.DictWriter(handle, fieldnames=fields)
                recorder.csv_writer.writeheader()
                recorder._estimated_position_cb(FakeMsg([1000.0, 7.0, 4.0, 1000.0325]))
                recorder._corrected_position_cb(FakeMsg([999.0, 6.5, 3.5, 999.0274]))
                recorder._float_cb(FakeMsg(200.0), "disparity_px", "/stereo/disparity")
                recorder._detections_cb(
                    FakeMsg([1, 690, 380, 710, 400, 0.9, 700, 390, 1, 490, 380, 510, 400, 0.8, 500, 390])
                )
                recorder._write_csv_row()

            payload = audit_module.audit_schema(input_csv=telemetry)

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(payload["valid_row_count"], 1)
        self.assertFalse(payload["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
