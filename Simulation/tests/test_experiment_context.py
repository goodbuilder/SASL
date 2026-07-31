from __future__ import annotations

import math
import unittest

from stereo_ground_sim.coordinate_frames import nominal_camera_poses
from stereo_ground_sim.experiment_context import (
    build_context_payload,
    camera_pose_quaternion_xyzw,
    canonical_context_json,
    context_id_sha256,
)


class SimulationExperimentContextTests(unittest.TestCase):
    def _payload(self):
        left, right = nominal_camera_poses(25.0, 5.0)
        return build_context_payload(
            source_application="Simulation",
            run_id="run",
            case_id="case",
            scenario_name="fixture",
            simulation_epoch=0,
            simulation_epoch_valid=False,
            simulation_epoch_source="unavailable",
            repository="repo",
            branch="branch",
            commit_sha="sha",
            git_status="",
            generated_sdf_sha256="sdf",
            nominal_baseline_m=25.0,
            zoom_index=0.0,
            zoom_index_valid=False,
            zoom_index_source="unresolved",
            camera_image_width_px=1280,
            camera_image_height_px=720,
            camera_update_rate_hz=20.0,
            camera_horizontal_fov_rad=0.154997710698,
            observation_mode="fixture",
            nominal_left_position_world_enu_m=left.center_world_enu_m,
            nominal_left_orientation_xyzw=camera_pose_quaternion_xyzw(left),
            nominal_right_position_world_enu_m=right.center_world_enu_m,
            nominal_right_orientation_xyzw=camera_pose_quaternion_xyzw(right),
            left_direct_rpy_deg=(0.0, 0.0, 0.0),
            right_direct_rpy_deg=(0.0, 0.0, 0.0),
            common_rpy_deg=(0.0, 0.0, 0.3),
            relative_rpy_deg=(0.0, 0.0, 0.2),
        )

    def test_effective_rpy_and_nominal_quaternions_are_contract_values(self):
        payload = self._payload()
        self.assertEqual(payload["left_effective_rpy_deg"], [0.0, 0.0, 0.19999999999999998])
        self.assertEqual(payload["right_effective_rpy_deg"], [0.0, 0.0, 0.4])
        for pose_key in ("nominal_left_optical_pose_world_enu", "nominal_right_optical_pose_world_enu"):
            quaternion = payload[pose_key]["orientation_xyzw"]
            self.assertAlmostEqual(math.sqrt(sum(value * value for value in quaternion)), 1.0, delta=1.0e-15)

    def test_canonical_hash_is_stable_and_rejects_nonfinite(self):
        payload = self._payload()
        self.assertEqual(context_id_sha256(payload), context_id_sha256(payload))
        self.assertEqual(canonical_context_json(payload), canonical_context_json(payload))
        payload["left_effective_rpy_deg"][2] = float("nan")
        with self.assertRaises(ValueError):
            context_id_sha256(payload)


if __name__ == "__main__":
    unittest.main()
