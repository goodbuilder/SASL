from __future__ import annotations

import csv
import io
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stereo_ground_sim"))

from stereo_ground_sim import operator_recorder  # noqa: E402


class TimestampPrecisionTests(unittest.TestCase):
    def test_ros_stamp_ns_is_preserved_as_int(self):
        stamp = SimpleNamespace(sec=1_700_000_000, nanosec=123_456_789)
        self.assertEqual(operator_recorder._stamp_to_ns(stamp), 1_700_000_000_123_456_789)
        self.assertEqual(
            operator_recorder._timestamp_sec_from_ns(1_700_000_000_123_456_789),
            "1700000000.123456789",
        )

    def test_csv_timestamp_columns_use_fixed_precision_and_ns_columns(self):
        module = operator_recorder
        recorder = object.__new__(module.OperatorRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.counts = {field: 0 for field in module.COUNTER_FIELDS}
        recorder._lock = module.threading.RLock()
        recorder.start_monotonic = time.monotonic()
        recorder.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_700_000_000_123_456_789)
        )

        fields = list(
            dict.fromkeys(
                ["time_sec", "ros_time_sec", "axis_contract"]
                + list(module.NUMERIC_TOPICS.values())
                + module.COMPARISON_FIELDS
                + module.GROUND_TRUTH_POSITION_FIELDS
                + module.ESTIMATED_POSITION_FIELDS
                + module.CORRECTED_POSITION_FIELDS
                + module.BBOX_ADJUSTMENT_FIELDS
                + module.DETECTION_FIELDS
                + module.PROFILE_STATUS_FIELDS
                + module.RESEARCH_FIELDS
                + module.RECEIVE_TIME_FIELDS
                + module.RECEIVE_TIME_NS_FIELDS
                + module.RECEIVE_MONOTONIC_NS_FIELDS
                + module.SOURCE_STATE_FIELDS
                + module.SOURCE_RECEIVE_AGE_FIELDS
                + module.FLIGHT_CONTRACT_FIELDS
            )
        )
        output = io.StringIO()
        recorder.csv_file = output
        recorder.csv_writer = csv.DictWriter(output, fieldnames=fields)
        recorder.csv_writer.writeheader()

        message = SimpleNamespace(data=[10.0, 2.0, 3.0, 10.6])
        recorder._ground_truth_position_cb(message)
        recorder.latest["disparity_px"] = 12.0
        recorder.latest["analytic_disparity_px"] = 10.0
        recorder._write_csv_row()
        row = next(csv.DictReader(io.StringIO(output.getvalue())))

        self.assertEqual(row["ground_truth_receive_ros_time_ns"], "1700000000123456789")
        self.assertEqual(row["ground_truth_receive_ros_time_sec"], "1700000000.123456789")
        self.assertNotEqual(row["ground_truth_receive_monotonic_ns"], "")
        self.assertEqual(row["ground_truth_state"], "VALID")
        self.assertNotEqual(row["ground_truth_receive_age_sec"], "")
        self.assertEqual(row["disparity_difference_px"], "")
        self.assertEqual(row["latest_unsynchronized_disparity_difference_px"], "2")
        self.assertEqual(len(row["ros_time_sec"].split(".")[1]), 9)

    def test_image_header_stamp_ns_is_separate_from_receive_time(self):
        module = operator_recorder

        class Writer:
            def isOpened(self):
                return True

            def write(self, _image):
                return None

        class Cv2:
            @staticmethod
            def VideoWriter_fourcc(*_args):
                return 0

            @staticmethod
            def VideoWriter(*_args):
                return Writer()

        recorder = object.__new__(module.OperatorRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.counts = {field: 0 for field in module.COUNTER_FIELDS}
        recorder._lock = module.threading.RLock()
        recorder.video_writers = {}
        recorder.video_fps = 20.0
        recorder.session_dir = Path(tempfile.mkdtemp())
        recorder.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_700_000_000_987_654_321)
        )
        recorder.get_logger = lambda: SimpleNamespace(error=lambda *_a, **_k: None)
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1_700_000_000, nanosec=987_654_321)),
            encoding="bgr8",
            width=1,
            height=1,
            step=3,
            data=bytes((1, 2, 3)),
        )
        previous = module.cv2
        module.cv2 = Cv2
        try:
            recorder._image_cb("left", message, "left_overlay")
        finally:
            module.cv2 = previous

        self.assertEqual(recorder.latest["left_image_header_time_ns"], 1_700_000_000_987_654_321)
        self.assertEqual(recorder.latest["left_image_header_time_sec"], 1_700_000_000.9876543)
        self.assertEqual(recorder.last_stamp["/left_camera/detection_image"], 1_700_000_000_987_654_321)

    def test_source_age_uses_monotonic_when_sim_time_pauses_or_resets(self):
        module = operator_recorder
        recorder = object.__new__(module.OperatorRecorder)
        recorder.last_monotonic_stamp = {"/ground_truth_position": 10_000_000_000}
        latest = {"ground_truth": [1.0, 2.0, 3.0, 4.0]}
        stamps = {"/ground_truth_position": 0}

        states, ages = recorder._source_state_snapshot(
            latest,
            stamps,
            now_ns=0,
            monotonic_stamps=recorder.last_monotonic_stamp,
            now_monotonic_ns=13_000_000_000,
        )
        self.assertEqual(states["ground_truth"], "stale")
        self.assertEqual(ages["ground_truth"], 3.0)

        # A ROS/simulation reset cannot make a receipt fresh; monotonic
        # ordering still rejects a now-before-receipt observation.
        states, ages = recorder._source_state_snapshot(
            latest,
            {"/ground_truth_position": 5_000_000_000},
            now_ns=5_000_000_000,
            monotonic_stamps=recorder.last_monotonic_stamp,
            now_monotonic_ns=9_000_000_000,
        )
        self.assertEqual(states["ground_truth"], "stale")
        self.assertIsNone(ages["ground_truth"])


if __name__ == "__main__":
    unittest.main()
