import json
import math
import unittest

from pose_simulation.sources import (
    RTK_STATUS_INVALID_FIX,
    RTK_STATUS_VALID_FIXED,
    RTK_STATUS_VALID_NON_FIXED,
    RtkSample,
    SampleParseError,
    pose_source_from_camera_diagnostic_json,
)


class FakeGps:
    timestamp_sample = 1234
    latitude_deg = 37.0
    longitude_deg = 127.0
    altitude_msl_m = 100.0
    altitude_ellipsoid_m = 102.0
    eph = 0.03
    epv = 0.05
    fix_type = 6
    satellites_used = 20


class SourcesTests(unittest.TestCase):
    def test_camera_parser_consumes_observed_optical_pose(self):
        payload = {
            "gazebo_stamp_ns": 20,
            "observed_optical_position_world_enu_m": [1.0, 2.0, 3.0],
            "observed_optical_orientation_world_from_entity_xyzw": [0.0, 0.0, 0.0, 1.0],
            "desired_optical_position_world_enu_m": [99.0, 99.0, 99.0],
        }
        sample = pose_source_from_camera_diagnostic_json(
            json.dumps(payload), source="left", receive_timestamp_ns=100
        )
        self.assertEqual(sample.position_world_enu_m, (1.0, 2.0, 3.0))
        self.assertEqual(sample.source_timestamp_ns, 20)
        # Identity Gazebo link orientation still maps RDF forward to link +X.
        self.assertEqual(
            tuple(round(value, 12) for value in sample.rotation_world_from_frame[0]),
            (0.0, 0.0, 1.0),
        )

    def test_camera_parser_rejects_desired_only_payload(self):
        with self.assertRaises(SampleParseError):
            pose_source_from_camera_diagnostic_json(
                {"gazebo_stamp_ns": 20, "desired_optical_position_world_enu_m": [0.0, 0.0, 0.0]},
                source="left",
                receive_timestamp_ns=100,
            )

    def test_rtk_fix_states_are_explicit(self):
        fixed = RtkSample("/rtk", "FakeGps", 1, 1, 37.0, 127.0, 100.0, 102.0, 0.03, 0.05, 6, 20)
        floating = RtkSample("/rtk", "FakeGps", 1, 1, 37.0, 127.0, 100.0, 102.0, 0.03, 0.05, 5, 20)
        invalid = RtkSample("/rtk", "FakeGps", 1, 1, 37.0, 127.0, 100.0, 102.0, 0.03, 0.05, 3, 20)
        self.assertEqual(fixed.status(), RTK_STATUS_VALID_FIXED)
        self.assertEqual(floating.status(), RTK_STATUS_VALID_NON_FIXED)
        self.assertEqual(invalid.status(), RTK_STATUS_INVALID_FIX)

    def test_rtk_nonfinite_accuracy_is_not_silent(self):
        invalid = RtkSample("/rtk", "FakeGps", 1, 1, 37.0, 127.0, 100.0, 102.0, math.nan, 0.05, 6, 20)
        self.assertNotEqual(invalid.status(), RTK_STATUS_VALID_FIXED)


if __name__ == "__main__":
    unittest.main()
