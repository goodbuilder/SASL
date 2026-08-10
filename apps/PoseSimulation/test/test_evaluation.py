import math
import unittest

from pose_simulation.coordinate_adapter import Wgs84ReferenceOrigin
from pose_simulation.evaluation import (
    DEFAULT_RIG_FROM_OPTICAL_RDF,
    NominalStereoExtrinsics,
    evaluate_pose_snapshot,
)
from pose_simulation.pose_math import (
    Pose3D,
    compute_look_at_rig_rotation,
    derive_camera_optical_pose,
)
from pose_simulation.sources import PoseSourceSample


def sample(source, pose, stamp=10):
    return PoseSourceSample(source, pose.frame_id, pose, stamp, stamp, source)


class EvaluationTests(unittest.TestCase):
    def test_rtk_ideal_vs_gt_ideal_is_atan2_one_over_thousand(self):
        extrinsics = NominalStereoExtrinsics(
            left_rotation_rig_from_optical=DEFAULT_RIG_FROM_OPTICAL_RDF,
            right_rotation_rig_from_optical=DEFAULT_RIG_FROM_OPTICAL_RDF,
        )
        gt_target = Pose3D((0.0, 1000.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), "gazebo_world_enu")
        # The initial actual camera pair is the GT ideal pair.  Their midpoint
        # is therefore the rig center used by the evaluation.
        gt_look = compute_look_at_rig_rotation((0.0, 0.0, 0.0), (0.0, 1000.0, 0.0))
        left_pose = derive_camera_optical_pose(
            (0.0, 0.0, 0.0), gt_look.rotation_world_from_rig, extrinsics.left_translation_rig_m, extrinsics.left_rotation_rig_from_optical
        )
        right_pose = derive_camera_optical_pose(
            (0.0, 0.0, 0.0), gt_look.rotation_world_from_rig, extrinsics.right_translation_rig_m, extrinsics.right_rotation_rig_from_optical
        )
        result = evaluate_pose_snapshot(
            ground_truth_target=sample("gt", gt_target),
            left_actual=sample("left", left_pose),
            right_actual=sample("right", right_pose),
            rtk_target_world_enu_m=(1.0, 1000.0, 0.0),
            rtk_valid=True,
            rtk_status="VALID_RTK_FIXED",
            extrinsics=extrinsics,
            evaluation_timestamp_ns=10,
            ground_truth_source="TARGET_REFERENCE_LINK",
            target_entity_name="target_reference_link",
            rtk_source_topic="/fmu/out/vehicle_gps_position",
            reference_origin=Wgs84ReferenceOrigin(0.0, 0.0, 0.0),
            gazebo_origin_in_reference_enu_m=(0.0, 0.0, 0.0),
            altitude_datum="ellipsoid",
        )
        expected = math.degrees(math.atan2(1.0, 1000.0))
        actual = result.pose_error["rtk_ideal_vs_gt_ideal"]["yaw_error_deg"]
        self.assertAlmostEqual(actual, expected, delta=1.0e-12)
        self.assertAlmostEqual(
            result.pose_error["left_actual_vs_gt_ideal"]["rotation_error_deg"], 0.0, delta=1.0e-12
        )
        self.assertEqual(result.status["rig_center_source"], "actual_optical_midpoint")
        self.assertEqual(result.stereo_geometry["baseline"]["midpoint_error_status"], "not_available")

    def test_invalid_rtk_keeps_ground_truth_comparison_valid(self):
        extrinsics = NominalStereoExtrinsics()
        look = compute_look_at_rig_rotation((0.0, 0.0, 0.0), (0.0, 100.0, 0.0))
        left = derive_camera_optical_pose(
            (0.0, 0.0, 0.0), look.rotation_world_from_rig,
            extrinsics.left_translation_rig_m, extrinsics.left_rotation_rig_from_optical,
        )
        right = derive_camera_optical_pose(
            (0.0, 0.0, 0.0), look.rotation_world_from_rig,
            extrinsics.right_translation_rig_m, extrinsics.right_rotation_rig_from_optical,
        )
        target = Pose3D((0.0, 100.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), "gazebo_world_enu")
        result = evaluate_pose_snapshot(
            ground_truth_target=sample("gt", target),
            left_actual=sample("left", left),
            right_actual=sample("right", right),
            rtk_target_world_enu_m=None,
            rtk_valid=False,
            rtk_status="INVALID_FIX",
            extrinsics=extrinsics,
            evaluation_timestamp_ns=10,
            ground_truth_source="TARGET_REFERENCE_LINK",
            target_entity_name="target_reference_link",
            rtk_source_topic="/fmu/out/vehicle_gps_position",
            reference_origin=Wgs84ReferenceOrigin(0.0, 0.0, 0.0),
            gazebo_origin_in_reference_enu_m=(0.0, 0.0, 0.0),
            altitude_datum="ellipsoid",
        )
        self.assertTrue(result.status["validity"]["ground_truth_ideal"])
        self.assertTrue(result.status["validity"]["actual_vs_gt"])
        self.assertFalse(result.status["validity"]["actual_vs_rtk"])
        self.assertFalse(result.pose_error["rtk_ideal_vs_gt_ideal"]["valid"])


if __name__ == "__main__":
    unittest.main()
