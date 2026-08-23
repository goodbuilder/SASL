from __future__ import annotations

import math

import pytest
from test_optical_pose_controller_integration import _gps_state, _panel
from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.workstation_core.optical_pose import CameraPoseNed
from Workstation.workstation_core.rtk_geometry import EARTH_RADIUS_M, ned_delta_m


def _ray_for_target(pose: CameraPoseNed, target: tuple[float, float, float]) -> tuple[float, float, float]:
    world = tuple(target[index] - pose.center_ned_m[index] for index in range(3))
    norm = math.sqrt(sum(value * value for value in world))
    world = tuple(value / norm for value in world)
    rotation = pose.rotation_ned_from_camera_rdf
    return tuple(
        sum(rotation[row][column] * world[row] for row in range(3))
        for column in range(3)
    )


def _gps_for_ned(
    role: str,
    target_ned_m: tuple[float, float, float],
    *,
    timestamp_s: float = 100.0,
):
    latitude_origin_deg = 37.0
    longitude_origin_deg = 127.0
    altitude_origin_m = 100.0
    north_m, east_m, down_m = target_ned_m
    latitude_deg = latitude_origin_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude_deg = longitude_origin_deg + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(latitude_origin_deg)))
    )
    return _gps_state(
        role,
        timestamp_s=timestamp_s,
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_origin_m - down_m,
        altitude_ellipsoid_m=altitude_origin_m - down_m,
    )


def _target_annotations(snapshot, pixels_by_side):
    return {
        side: {
            "frame_id": snapshot["frozen_frame_artifacts"][side]["frame_id"],
            "image_sha256": snapshot["frozen_frame_artifacts"][side]["sha256"],
            "source_width_px": 640,
            "source_height_px": 480,
            "horizon_line": [],
            "ridge_polyline": [],
            "target_point": [
                {"u_px": pixels_by_side[side][0], "v_px": pixels_by_side[side][1]}
            ],
        }
        for side in ("left", "right")
    }


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


def test_accumulated_frozen_targets_solve_without_json_payload(tmp_path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    true_poses = {
        "left": CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=(0.3, -0.2, 0.1),
            roll_deg=1.0,
            pitch_deg=-0.5,
            yaw_deg=0.0,
        ),
        "right": CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=(0.3, 1.8, 0.15),
            roll_deg=1.5,
            pitch_deg=-0.8,
            yaw_deg=0.0,
        ),
    }
    requested_targets = (
        (20.0, -2.0, 0.0),
        (22.0, 2.0, -2.0),
        (25.0, 0.0, 3.0),
        (18.0, 4.0, -3.0),
        (28.0, -1.0, 4.0),
        (21.0, 5.0, 1.0),
    )
    origin = _gps_for_ned("left", (0.0, 0.0, 0.0))
    right = _gps_for_ned("right", (0.0, 2.0, 0.0))

    for requested_target in requested_targets:
        drone = _gps_for_ned("drone", requested_target)
        panel.latest_gps = {"left": origin, "right": right, "drone": drone}
        frozen = controller.capture_workstation_optical_pose_burst({})
        actual_target = ned_delta_m(origin, drone)
        assert actual_target is not None
        assert actual_target == pytest.approx(requested_target, abs=1.0e-4)
        pixels = {}
        for side, pose in true_poses.items():
            ray = _ray_for_target(pose, actual_target)
            pixels[side] = (
                800.0 * ray[0] / ray[2] + 320.0,
                800.0 * ray[1] / ray[2] + 240.0,
            )
        snapshot = controller.record_workstation_optical_pose_annotation(
            {
                "annotation_mode": "target",
                "annotations": _target_annotations(frozen, pixels),
            }
        )
        assert snapshot["drone_rtk_multiview_samples"][-1]["state"] == "READY"

    solved = controller.solve_workstation_optical_pose_drone_rtk_multiview({})

    assert len(solved["drone_rtk_multiview_samples"]) == len(requested_targets)
    candidate = solved["candidates"][-1]
    assert candidate["method"] == "drone-rtk-multiview-r-t"
    assert candidate["quality_metrics"]["left_observation_count"] == 6.0
    assert candidate["quality_metrics"]["right_observation_count"] == 6.0
    assert "drone_rtk_multiview_collection_sha256" in candidate["input_hashes"]
    assert "drone_rtk_multiview_json_sha256" not in candidate["input_hashes"]
    assert candidate["left_pose"]["center_ned_m"] == pytest.approx(
        true_poses["left"].center_ned_m,
        abs=1.0e-3,
    )
    assert candidate["right_pose"]["center_ned_m"] == pytest.approx(
        true_poses["right"].center_ned_m,
        abs=1.0e-3,
    )
    with pytest.raises(ValueError, match="owner-derived"):
        controller.apply_workstation_optical_pose_candidate(
            {
                "candidate_id": candidate["candidate_id"],
                "dof": {"roll": True, "pitch": True, "yaw": True},
            }
        )

    applied = controller.apply_workstation_optical_pose_candidate(
        {"candidate_id": candidate["candidate_id"]}
    )
    assert all(applied["selected_dof"].values())
    assert applied["left_pose"]["center_ned_m"] == pytest.approx(
        true_poses["left"].center_ned_m,
        abs=1.0e-3,
    )
    assert applied["right_pose"]["center_ned_m"] == pytest.approx(
        true_poses["right"].center_ned_m,
        abs=1.0e-3,
    )


def test_accumulated_multiview_ignores_blocked_sync_and_revises_same_burst(tmp_path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    panel.latest_gps = {
        "left": _gps_for_ned("left", (0.0, 0.0, 0.0)),
        "right": _gps_for_ned("right", (0.0, 2.0, 0.0)),
        "drone": _gps_for_ned("drone", (20.0, 0.0, 0.0), timestamp_s=101.0),
    }
    frozen = controller.capture_workstation_optical_pose_burst({})
    annotations = _target_annotations(
        frozen,
        {"left": (320.0, 240.0), "right": (240.0, 240.0)},
    )
    right_target = annotations["right"]["target_point"]
    annotations["right"]["target_point"] = []
    partial = controller.record_workstation_optical_pose_annotation(
        {"annotation_mode": "target", "annotations": annotations}
    )
    assert partial["drone_rtk_multiview_samples"] == []
    annotations["right"]["target_point"] = right_target

    first = controller.record_workstation_optical_pose_annotation(
        {"annotation_mode": "target", "annotations": annotations}
    )
    sample = first["drone_rtk_multiview_samples"][0]
    assert sample["state"] == "BLOCKED"
    assert sample["revision"] == 1
    assert "synchronization" in " ".join(sample["reasons"]).lower() or "delta" in " ".join(
        sample["reasons"]
    ).lower()

    annotations["left"]["target_point"][0]["u_px"] = 321.0
    revised = controller.record_workstation_optical_pose_annotation(
        {"annotation_mode": "target", "annotations": annotations}
    )
    assert len(revised["drone_rtk_multiview_samples"]) == 1
    assert revised["drone_rtk_multiview_samples"][0]["revision"] == 2
    assert revised["drone_rtk_multiview_samples"][0]["target_pixels"]["left"]["u_px"] == 321.0

    with pytest.raises(ValueError, match=r"ready=0, blocked=1"):
        controller.solve_workstation_optical_pose_drone_rtk_multiview({})

    controller.reset_workstation_optical_pose_session()
    assert controller.workstation_optical_pose_snapshot()[
        "drone_rtk_multiview_samples"
    ] == []
