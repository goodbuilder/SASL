from __future__ import annotations

import math

import numpy as np

from campaigns.stereo_v1_1.camera_frame_contract import (
    optical_rwc_from_entity_quaternion_xyzw,
)
from campaigns.stereo_v1_1.calibration_solver import R_WC_NOMINAL
from campaigns.stereo_v1_1.preflight_capture import _expected_rwc, _orientation_error_deg


def test_recorded_gazebo_camera_link_flu_quaternion_converts_to_optical_rdf_rwc():
    # World-from-SDF-link FLU for nominal yaw=90deg and 20deg camera elevation.
    half = math.sqrt(0.5)
    half_pitch_sin = math.sin(math.radians(10.0))
    half_pitch_cos = math.cos(math.radians(10.0))
    link_quaternion_xyzw = [
        half * half_pitch_sin,
        -half * half_pitch_sin,
        half * half_pitch_cos,
        half * half_pitch_cos,
    ]
    optical_rwc = optical_rwc_from_entity_quaternion_xyzw(link_quaternion_xyzw)
    np.testing.assert_allclose(optical_rwc, R_WC_NOMINAL, atol=1e-12)
    np.testing.assert_allclose(optical_rwc[:, 0], [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(
        optical_rwc[:, 2],
        [0.0, math.cos(math.radians(20.0)), math.sin(math.radians(20.0))],
        atol=1e-12,
    )
    np.testing.assert_allclose(_expected_rwc({
        "coordinates": {"R_WC_nominal": R_WC_NOMINAL.tolist()},
        "camera": {"physical_pose_groups": {"G0": {
            "heading_left_deg": 0.0, "heading_right_deg": 0.0,
        }}},
    }, "G0", "left"), optical_rwc, atol=1e-12)
    assert _orientation_error_deg(optical_rwc.tolist(), R_WC_NOMINAL.tolist()) < 1e-10

