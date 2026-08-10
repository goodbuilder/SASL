import math
import unittest

from pose_simulation.gazebo_pose_adapter import (
    GazeboPosePayloadError,
    parse_pose_v_payload,
    parse_world_pose_v_payload,
)


class GazeboPoseAdapterTests(unittest.TestCase):
    def test_scoped_link_is_composed_into_world_enu(self):
        half = math.sin(math.pi / 4.0)
        payload = {
            "header": {"stamp": {"sec": 2, "nsec": 3}},
            "pose": [
                {
                    "name": "camera_model",
                    "position": {"x": 10.0, "y": 20.0, "z": 30.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": half, "w": half},
                },
                {
                    "name": "camera_model::optical_link",
                    "position": {"x": 1.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            ],
        }
        record = parse_world_pose_v_payload(
            payload,
            entity_name="camera_model::optical_link",
            parent_model_name="camera_model",
        )
        self.assertEqual(record.stamp_ns, 2_000_000_003)
        self.assertAlmostEqual(record.position_world_enu_m[0], 10.0, delta=1.0e-12)
        self.assertAlmostEqual(record.position_world_enu_m[1], 21.0, delta=1.0e-12)
        self.assertAlmostEqual(record.position_world_enu_m[2], 30.0, delta=1.0e-12)

    def test_missing_entity_and_bad_quaternion_fail(self):
        payload = {
            "header": {"stamp": {"sec": 1, "nsec": 0}},
            "pose": [
                {
                    "name": "model::link",
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
                }
            ],
        }
        with self.assertRaises(GazeboPosePayloadError):
            parse_pose_v_payload(payload, entity_name="missing")
        with self.assertRaises(ValueError):
            parse_pose_v_payload(payload, entity_name="model::link")


if __name__ == "__main__":
    unittest.main()
