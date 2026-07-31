import unittest
import math

from pose_simulation.visualization_marker_factory import (
    VisualizationConfig,
    build_visualization_marker_specs,
)
from test_visualization_snapshot import _status, _valid_snapshot


WORLD = "gazebo_world_enu"


class VisualizationMarkerFactoryTests(unittest.TestCase):
    def test_all_markers_use_world_frame_and_deterministic_ids(self):
        snapshot = _valid_snapshot()
        config = VisualizationConfig()
        first = build_visualization_marker_specs(snapshot, config)
        second = build_visualization_marker_specs(snapshot, config)
        self.assertEqual(
            [(marker.namespace, marker.marker_id, marker.action) for marker in first],
            [(marker.namespace, marker.marker_id, marker.action) for marker in second],
        )
        self.assertTrue(all(marker.frame_id == WORLD for marker in first))

    def test_camera_optical_forward_is_positive_z_for_identity_pose(self):
        snapshot = _valid_snapshot()
        specs = build_visualization_marker_specs(snapshot, VisualizationConfig())
        left_forward = next(
            marker
            for marker in specs
            if marker.namespace == "pose_sim/actual/left"
            and marker.marker_id == 1
            and marker.action == "ADD"
        )
        self.assertEqual(left_forward.points[0], (-12.5, 0.0, 0.0))
        self.assertEqual(left_forward.points[1], (-12.5, 0.0, 5.0))

    def test_normalized_ray_has_configured_length_not_target_distance(self):
        snapshot = _valid_snapshot()
        specs = build_visualization_marker_specs(
            snapshot,
            VisualizationConfig(normalized_ray_length_m=50.0),
        )
        ray = next(
            marker
            for marker in specs
            if marker.namespace == "pose_sim/rays/gt_los"
            and marker.marker_id == 0
            and marker.action == "ADD"
        )
        self.assertEqual(ray.points[0], (-12.5, 0.0, 0.0))
        length = math.sqrt(sum((ray.points[1][index] - ray.points[0][index]) ** 2 for index in range(3)))
        self.assertAlmostEqual(length, 50.0)
        self.assertGreater(ray.points[1][1], 49.0)

    def test_invalid_snapshot_deletes_geometry_and_keeps_no_normal_marker(self):
        invalid = _valid_snapshot()
        invalid = invalid.__class__(
            evaluation_id=invalid.evaluation_id,
            evaluation_stamp_ns=invalid.evaluation_stamp_ns,
            world_frame=invalid.world_frame,
            valid=False,
            state="EVALUATION_MISMATCH",
            invalid_reason="Source evaluations do not match",
        )
        specs = build_visualization_marker_specs(invalid, VisualizationConfig())
        self.assertTrue(specs)
        self.assertTrue(all(marker.action == "DELETE" for marker in specs if marker.namespace != "pose_sim/status"))
        self.assertFalse(any(marker.action == "ADD" and marker.namespace == "pose_sim/actual/left" for marker in specs))

    def test_rtk_markers_are_absent_when_source_declares_rtk_unimplemented(self):
        specs = build_visualization_marker_specs(_valid_snapshot(), VisualizationConfig(show_rtk_ideal=True))
        self.assertFalse(any(marker.action == "ADD" and marker.namespace.startswith("pose_sim/ideal_rtk") for marker in specs))
        self.assertFalse(any(marker.action == "ADD" and marker.namespace == "pose_sim/target/rtk" for marker in specs))

    def test_invalid_config_is_rejected(self):
        with self.assertRaises(ValueError):
            VisualizationConfig(normalized_ray_length_m=0.0).validate()


if __name__ == "__main__":
    unittest.main()
