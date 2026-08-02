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

    def test_camera_and_drone_mesh_markers_use_stl_resources(self):
        specs = build_visualization_marker_specs(_valid_snapshot(), VisualizationConfig())
        camera_mesh = next(
            marker
            for marker in specs
            if marker.namespace == "pose_sim/actual/left"
            and marker.marker_id == 5
            and marker.action == "ADD"
        )
        drone_mesh = next(
            marker
            for marker in specs
            if marker.namespace == "pose_sim/target/gt"
            and marker.marker_id == 2
            and marker.action == "ADD"
        )
        self.assertEqual(camera_mesh.marker_type, "mesh")
        self.assertTrue(camera_mesh.mesh_resource.endswith("WIRIS.stl"))
        self.assertEqual(camera_mesh.scale, (0.001, 0.001, 0.001))
        self.assertEqual(drone_mesh.marker_type, "mesh")
        self.assertTrue(drone_mesh.mesh_resource.endswith("x500_15m.stl"))
        self.assertEqual(drone_mesh.scale, (22.0, 22.0, 22.0))

    def test_marker_and_ray_visual_thickness_scales_are_applied(self):
        specs = build_visualization_marker_specs(
            _valid_snapshot(),
            VisualizationConfig(
                marker_visual_scale=2.0,
                ray_shaft_width_m=2.0,
                ray_head_width_m=2.0,
                ray_line_width_m=2.0,
            ),
        )
        optical_marker = next(
            marker
            for marker in specs
            if marker.namespace == "pose_sim/actual/left"
            and marker.marker_id == 0
            and marker.action == "ADD"
        )
        gt_ray = next(
            marker
            for marker in specs
            if marker.namespace == "pose_sim/rays/gt_los"
            and marker.marker_id == 0
            and marker.action == "ADD"
        )
        self.assertEqual(optical_marker.scale, (6.0, 6.0, 6.0))
        self.assertEqual(gt_ray.scale, (2.0, 2.0, 2.0))

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
        self.assertFalse(any(marker.action == "ADD" and marker.namespace == "pose_sim/status" for marker in specs))

    def test_rtk_markers_are_absent_when_source_declares_rtk_unimplemented(self):
        specs = build_visualization_marker_specs(_valid_snapshot(), VisualizationConfig(show_rtk_ideal=True))
        self.assertFalse(any(marker.action == "ADD" and marker.namespace.startswith("pose_sim/ideal_rtk") for marker in specs))
        self.assertFalse(any(marker.action == "ADD" and marker.namespace == "pose_sim/target/rtk" for marker in specs))

    def test_invalid_config_is_rejected(self):
        with self.assertRaises(ValueError):
            VisualizationConfig(normalized_ray_length_m=0.0).validate()

    def test_deduplicated_snapshot_can_use_persistent_markers(self):
        config = VisualizationConfig(marker_lifetime_s=0.0).validate()
        specs = build_visualization_marker_specs(_valid_snapshot(), config)
        self.assertTrue(any(marker.action == "ADD" and marker.lifetime_s == 0.0 for marker in specs))
        with self.assertRaises(ValueError):
            VisualizationConfig(marker_lifetime_s=-1.0).validate()


if __name__ == "__main__":
    unittest.main()
