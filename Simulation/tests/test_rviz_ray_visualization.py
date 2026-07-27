from __future__ import annotations

import sys
from pathlib import Path

import pytest


ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import build_baseline_center_frd, nominal_camera_poses
from sasl_common.stereo_geometry import PinholeIntrinsics
from stereo_ground_sim.stereo_distance_node import (
    dkl_ideal_pixels_from_world,
    ray_visualization_world_enu_from_triangulation,
)


def test_raw_ray_visualization_converts_solver_frd_endpoints_back_to_world_enu() -> None:
    left, right = nominal_camera_poses(25.0, 5.0)
    frame = build_baseline_center_frd(
        left.center_world_enu_m,
        right.center_world_enu_m,
        left.forward_world_enu,
        right.forward_world_enu,
    )
    left_point_frd = (1000.0, -3.0, 2.0)
    right_point_frd = (1000.0, -2.0, 2.5)
    midpoint_frd = (1000.0, -2.5, 2.25)

    geometry = ray_visualization_world_enu_from_triangulation(
        frame,
        left,
        right,
        left_point_frd,
        right_point_frd,
        midpoint_frd,
    )

    assert geometry is not None
    assert geometry["left_origin_world_enu_m"] == pytest.approx(left.center_world_enu_m)
    assert geometry["right_origin_world_enu_m"] == pytest.approx(right.center_world_enu_m)
    assert geometry["left_target_world_enu_m"] == pytest.approx(frame.point_to_world(left_point_frd))
    assert geometry["right_target_world_enu_m"] == pytest.approx(frame.point_to_world(right_point_frd))
    assert geometry["midpoint_world_enu_m"] == pytest.approx(frame.point_to_world(midpoint_frd))


def test_dkl_ideal_pixels_project_world_enu_reference_through_nominal_cameras() -> None:
    left, right = nominal_camera_poses(25.0, 10.0)
    intrinsics = PinholeIntrinsics(fx_px=500.0, fy_px=500.0, cx_px=640.0, cy_px=360.0)

    pixels = dkl_ideal_pixels_from_world(
        (0.0, 1000.0, 10.0), left, right, intrinsics, width=1280, height=720
    )

    assert pixels is not None
    assert all(value == pytest.approx(round(value)) for value in pixels)
    assert 0.0 <= pixels[0] < 1280.0
    assert 0.0 <= pixels[1] < 720.0
    assert 0.0 <= pixels[2] < 1280.0
    assert 0.0 <= pixels[3] < 720.0


def test_rviz_rays_are_enabled_by_default_and_use_a_world_enu_marker_array() -> None:
    launcher = (ROOT_DIR / "run_operator_view.sh").read_text(encoding="utf-8")
    launch = (ROOT_DIR / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py").read_text(
        encoding="utf-8"
    )
    rviz = (ROOT_DIR / "stereo_ground_sim" / "rviz" / "stereo_ground_sim.rviz").read_text(
        encoding="utf-8"
    )

    assert 'launch_args+=("launch_rviz:=true")' in launcher
    assert 'launch_args+=("launch_rviz:=false")' in launcher  # --headless keeps all GUIs off.
    assert 'launch_args+=("enable_rviz_rays:=true")' in launcher
    assert 'DeclareLaunchArgument("enable_rviz_rays", default_value="true")' in launch
    assert 'executable="static_transform_publisher"' in launch
    assert '"world", "gazebo_world_enu"' in launch
    assert "/stereo/ray_markers" in rviz
    distance_display = rviz.split("Name: Distance Text", 1)[0].rsplit("Class: rviz_default_plugins/Marker", 1)[1]
    assert "Topic:" in distance_display
    assert "Marker Topic:" not in distance_display
    assert "/stereo/distance_marker" in distance_display
    assert "Enabled: false" in distance_display
    ray_display = rviz.split("Name: Stereo Rays — Nominal / Optical / Ideal (World ENU)", 1)[0].rsplit("Class: rviz_default_plugins/MarkerArray", 1)[1]
    assert "Topic:" in ray_display
    assert "Marker Topic:" not in ray_display
    assert "Durability Policy: Transient Local" in ray_display
    assert "Reliability Policy: Reliable" in ray_display
    assert "gazebo_world_enu" in rviz
    assert "Cell Size: 100" in rviz
    assert "Distance: 2800" in rviz
    assert "Y: 1000" in rviz
    assert "Yaw: 1.57" in rviz
    assert "stereo_rays_nominal: true" in rviz
    assert "stereo_rays_optical: true" in rviz
    assert "stereo_rays_ideal: true" in rviz
    assert "stereo_cameras: true" in rviz
    assert "stereo_actual_drone: true" in rviz


def test_marker_source_contains_nominal_optical_and_dkl_ideal_ray_groups() -> None:
    source = (PACKAGE_ROOT / "stereo_ground_sim" / "stereo_distance_node.py").read_text(encoding="utf-8")

    assert "self._publish_ray_markers(nominal=raw, optical=optical)" in source
    assert 'self.create_publisher(MarkerArray, "/stereo/ray_markers", ray_marker_qos)' in source
    assert "durability=DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert 'self._publish_ray_markers(ideal=ideal)' in source
    assert 'label="nominal"' in source
    assert 'label="optical"' in source
    assert 'label="ideal"' in source
    assert 'camera.ns = "stereo_cameras"' in source
    assert "camera.type = Marker.CUBE" in source
    assert "camera_result = optical if optical is not None else nominal" in source
    assert 'marker.ns = "stereo_actual_drone"' in source
    assert "marker.type = Marker.MESH_RESOURCE" in source
    assert "marker.mesh_use_embedded_materials = False" in source
    assert 'DRONE_MESH_RESOURCE = "package://stereo_ground_sim/models/x500_15m/meshes/NXP-HGD-CF.dae"' in source
    assert 'namespace = f"stereo_rays_{label}"' in source
    assert 'f"left_{label}_ray"' in source
    assert 'left_target, 2.0)' in source
    assert 'right_target, 2.0)' in source
    assert 'left_target, right_target, 1.5)' in source


def test_rviz_ray_colors_match_the_nominal_optical_and_ideal_rqt_series() -> None:
    source = (PACKAGE_ROOT / "stereo_ground_sim" / "stereo_distance_node.py").read_text(encoding="utf-8")

    assert source.count('left_color=(220.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0)') == 1
    assert source.count('right_color=(220.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0)') == 1
    assert source.count('midpoint_color=(220.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0)') == 1
    assert source.count('left_color=(0.0, 170.0 / 255.0, 170.0 / 255.0)') == 1
    assert source.count('right_color=(0.0, 170.0 / 255.0, 170.0 / 255.0)') == 1
    assert source.count('midpoint_color=(0.0, 170.0 / 255.0, 170.0 / 255.0)') == 1
    assert source.count('left_color=(32.0 / 255.0, 86.0 / 255.0, 1.0)') == 1
    assert source.count('right_color=(32.0 / 255.0, 86.0 / 255.0, 1.0)') == 1
    assert source.count('midpoint_color=(32.0 / 255.0, 86.0 / 255.0, 1.0)') == 1
