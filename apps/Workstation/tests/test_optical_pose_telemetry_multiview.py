from __future__ import annotations

import math
import time

import pytest

from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.workstation_core.optical_pose import CameraPoseNed
from Workstation.workstation_core.pixhawk_attitude import CameraExtrinsicState
from Workstation.workstation_core.rtk_geometry import GpsPositionState, ned_delta_m

from test_optical_pose_controller_integration import _panel


def _gps(
    camera: str,
    *,
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    timestamp_s: float,
) -> GpsPositionState:
    return GpsPositionState(
        camera=camera,
        namespace=f"{camera}-pixhawk",
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        fix_type=6,
        satellites_used=20,
    )


def _extrinsic(camera: str, timestamp_s: float, *, yaw_deg: float) -> CameraExtrinsicState:
    return CameraExtrinsicState(
        camera=camera,
        namespace=f"{camera}-pixhawk",
        roll_deg=2.0,
        pitch_deg=-1.0,
        yaw_deg=yaw_deg,
        timestamp_s=timestamp_s,
        topic="mavlink/ATTITUDE_QUATERNION",
        q=(1.0, 0.0, 0.0, 0.0),
    )


def _ray_for_target(pose: CameraPoseNed, target: tuple[float, float, float]) -> tuple[float, float, float]:
    world = tuple(target[index] - pose.center_ned_m[index] for index in range(3))
    norm = math.sqrt(sum(value * value for value in world))
    world = tuple(value / norm for value in world)
    rotation = pose.rotation_ned_from_camera_rdf
    return tuple(
        sum(rotation[row][column] * world[row] for row in range(3))
        for column in range(3)
    )


def test_telemetry_initialization_builds_reviewable_ned_candidate(tmp_path) -> None:
    panel = _panel(tmp_path)
    now = time.monotonic()
    panel.latest_gps = {
        "left": _gps("left", latitude_deg=37.0, longitude_deg=127.0, altitude_m=100.0, timestamp_s=now),
        "right": _gps(
            "right",
            latitude_deg=37.00001,
            longitude_deg=127.00002,
            altitude_m=100.4,
            timestamp_s=now,
        ),
        "drone": _gps(
            "drone",
            latitude_deg=37.00015,
            longitude_deg=127.00010,
            altitude_m=115.0,
            timestamp_s=now,
        ),
    }
    panel.latest_extrinsics = {
        "left": _extrinsic("left", now, yaw_deg=12.0),
        "right": _extrinsic("right", now, yaw_deg=14.0),
    }
    controller = OpticalPoseController(panel)

    snapshot = controller.initialize_workstation_optical_pose_telemetry({})

    candidate = snapshot["candidates"][-1]
    assert candidate["method"] == "telemetry-initial-pose"
    assert set(candidate["observable_dof"]) == {
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "north_m",
        "east_m",
        "down_m",
    }
    expected_right = ned_delta_m(panel.latest_gps["left"], panel.latest_gps["right"])
    assert candidate["left_pose"]["center_ned_m"] == pytest.approx([0.0, 0.0, 0.0])
    assert candidate["right_pose"]["center_ned_m"] == pytest.approx(expected_right)
    assert candidate["quality_metrics"]["target_distance_m"] > 0.0
    assert "reviewed candidate" in " ".join(candidate["warnings"])


def test_telemetry_initialization_rejects_stale_drone_position(tmp_path) -> None:
    panel = _panel(tmp_path)
    now = time.monotonic()
    panel.latest_gps = {
        "left": _gps("left", latitude_deg=37.0, longitude_deg=127.0, altitude_m=100.0, timestamp_s=now),
        "right": _gps("right", latitude_deg=37.0, longitude_deg=127.00002, altitude_m=100.0, timestamp_s=now),
        "drone": _gps(
            "drone",
            latitude_deg=37.00015,
            longitude_deg=127.00010,
            altitude_m=115.0,
            timestamp_s=now - 3.0,
        ),
    }

    with pytest.raises(RuntimeError, match="stale drone"):
        OpticalPoseController(panel).initialize_workstation_optical_pose_telemetry({})


def test_drone_rtk_multiview_solves_camera_center_and_rotation(tmp_path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    controller.capture_workstation_optical_pose_burst({})

    true_poses = {
        "left": CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=(0.35, -0.25, 0.15),
            roll_deg=2.0,
            pitch_deg=-1.0,
            yaw_deg=12.0,
        ),
        "right": CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=(0.35, 1.95, 0.25),
            roll_deg=2.5,
            pitch_deg=-1.4,
            yaw_deg=15.0,
        ),
    }
    targets = (
        (22.0, -5.0, 1.0),
        (24.0, 4.0, -2.0),
        (30.0, 1.0, 5.0),
        (19.0, -2.0, -4.0),
        (27.0, 7.0, 3.0),
        (18.0, 3.0, 0.0),
    )
    observations = {
        side: [
            {
                "observation_id": f"frame-{index}",
                "observed_ray_camera_rdf": list(_ray_for_target(pose, target)),
                "target_point_ned_m": list(target),
                "timestamp_alignment_verified": True,
            }
            for index, target in enumerate(targets)
        ]
        for side, pose in true_poses.items()
    }

    snapshot = controller.solve_workstation_optical_pose_drone_rtk_multiview(
        {
            "drone_rtk_multiview": {
                "camera_centers_ned_m": {
                    "left": [0.0, 0.0, 0.0],
                    "right": [0.0, 2.0, 0.0],
                },
                "observations": observations,
            }
        }
    )

    candidate = snapshot["candidates"][-1]
    assert candidate["method"] == "drone-rtk-multiview-r-t"
    assert candidate["observable_dof"] == {
        "roll_deg": True,
        "pitch_deg": True,
        "yaw_deg": True,
        "north_m": True,
        "east_m": True,
        "down_m": True,
    }
    assert candidate["left_pose"]["center_ned_m"] == pytest.approx(
        true_poses["left"].center_ned_m,
        abs=1.0e-4,
    )
    assert candidate["right_pose"]["center_ned_m"] == pytest.approx(
        true_poses["right"].center_ned_m,
        abs=1.0e-4,
    )
    assert candidate["quality_metrics"]["left_angular_rms_chord"] < 1.0e-6
    assert candidate["quality_metrics"]["right_angular_rms_chord"] < 1.0e-6


def test_drone_rtk_multiview_requires_multiple_observations(tmp_path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    controller.capture_workstation_optical_pose_burst({})

    with pytest.raises(ValueError, match="at least four observations"):
        controller.solve_workstation_optical_pose_drone_rtk_multiview(
            {
                "drone_rtk_multiview": {
                    "observations": {
                        "left": [{"observed_ray_camera_rdf": [0.0, 0.0, 1.0], "target_point_ned_m": [10.0, 0.0, 0.0]}],
                        "right": [{"observed_ray_camera_rdf": [0.0, 0.0, 1.0], "target_point_ned_m": [10.0, 0.0, 0.0]}],
                    }
                }
            }
        )
