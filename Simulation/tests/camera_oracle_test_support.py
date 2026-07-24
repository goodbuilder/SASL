from __future__ import annotations

import time

from stereo_ground_sim.stereo_distance_node import ObservedCameraOraclePose


TEST_SDF_SHA256 = "a" * 64


def seed_observed_camera_oracle(node, *, left_pose=None, right_pose=None) -> None:
    """Install test-only validated observations from explicitly chosen poses."""

    node.camera_oracle_generated_sdf_sha256 = TEST_SDF_SHA256
    node.camera_oracle_max_age_s = 2.5
    epoch = getattr(getattr(node, "simulation_epoch", None), "epoch", 0)
    for label, pose, stamp_ns in (
        ("left", left_pose or node.desired_left_camera, 101),
        ("right", right_pose or node.desired_right_camera, 202),
    ):
        model = f"{label}_stereo_camera"
        observation = ObservedCameraOraclePose(
            camera_label=label,
            pose=pose,
            gazebo_stamp_ns=stamp_ns,
            generated_sdf_sha256=TEST_SDF_SHA256,
            base_entity_name=f"{model}::base_link",
            base_entity_id=-1,
            optical_entity_name=f"{model}::optical_link",
            optical_entity_id=-1,
            optical_position_world_enu_m=pose.center_world_enu_m,
            optical_orientation_world_from_entity_xyzw=(0.0, 0.0, 0.0, 1.0),
            simulation_epoch=epoch,
            received_monotonic_s=time.monotonic(),
        )
        setattr(node, f"observed_{label}_camera", observation)
