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
    RAY_DISPLAY_NAMES,
    dkl_ideal_pixels_from_world,
    reference_ray_world_enu_from_point,
    ray_visualization_world_enu_from_triangulation,
    uam_gps_ray_visualization_world_enu_from_point,
)


def test_ray_display_names_keep_internal_keys_and_use_contract_labels() -> None:
    assert {key: RAY_DISPLAY_NAMES[key] for key in (
        "nominal_pose", "optical_pose", "ideal", "gps_baseline", "global_baseline"
    )} == {
        "nominal_pose": "Nominal Pose",
        "optical_pose": "Optical Pose",
        "ideal": "Ideal",
        "gps_baseline": "GPS Baseline",
        "global_baseline": "Global Baseline",
    }

    source = (PACKAGE_ROOT / "stereo_ground_sim" / "operator_panel.py").read_text(encoding="utf-8")
    assert '"nominal_pose": "Nominal Pose"' in source
    assert '"optical_pose": "Optical Pose"' in source
    assert '"ideal": "Ideal"' in source
    assert '"gps_baseline": "GPS Baseline"' in source
    assert '"global_baseline": "Global Baseline"' in source


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


def test_rviz_reference_ray_can_display_points_outside_dkl_pixel_bounds() -> None:
    left, right = nominal_camera_poses(25.0, 5.0)
    frame = build_baseline_center_frd(
        left.center_world_enu_m,
        right.center_world_enu_m,
        left.forward_world_enu,
        right.forward_world_enu,
    )
    point_world = frame.point_to_world((1000.0, 0.0, -120.0))
    intrinsics = PinholeIntrinsics(fx_px=6400.0, fy_px=6400.0, cx_px=640.0, cy_px=360.0)

    assert dkl_ideal_pixels_from_world(point_world, left, right, intrinsics, 1280, 720) is None
    geometry = reference_ray_world_enu_from_point(frame, left, right, point_world)

    assert geometry is not None
    assert geometry["computation_status"] == "computed"
    assert geometry["ray_visualization_world_enu"]["left_target_world_enu_m"] == pytest.approx(
        point_world
    )
    assert geometry["ray_visualization_world_enu"]["right_target_world_enu_m"] == pytest.approx(
        point_world
    )


def test_uam_gps_ray_uses_baseline_center_origin_and_world_enu_target() -> None:
    left, right = nominal_camera_poses(25.0, 5.0)
    frame = build_baseline_center_frd(
        left.center_world_enu_m,
        right.center_world_enu_m,
        left.forward_world_enu,
        right.forward_world_enu,
    )
    point_world = frame.point_to_world((1000.0, 4.0, -2.0))

    ray = uam_gps_ray_visualization_world_enu_from_point(frame, point_world)

    assert ray is not None
    assert ray["computation_status"] == "computed"
    assert ray["output_frame"] == "baseline_center_frd"
    assert ray["origin_world_enu_m"] == pytest.approx(frame.origin_world_enu_m)
    assert ray["target_world_enu_m"] == pytest.approx(point_world)
    assert ray["point_baseline_center_frd_m"] == pytest.approx((1000.0, 4.0, -2.0))


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
    assert "/stereo/fov_markers" in rviz
    assert "/stereo/camera_markers" in rviz
    assert "Name: Stereo Cameras (World ENU)" in rviz
    assert "/stereo/ground_truth_markers" in rviz
    assert "Name: Gazebo Ground-Truth Drone (World ENU)" in rviz
    distance_display = rviz.split("Name: Distance Text", 1)[0].rsplit("Class: rviz_default_plugins/Marker", 1)[1]
    assert "Topic:" in distance_display
    assert "Marker Topic:" not in distance_display
    assert "/stereo/distance_marker" in distance_display
    assert "Enabled: false" in distance_display
    ray_display = rviz.split("Name: Stereo Rays — Nominal Pose / Optical Pose / Ideal / GPS Baseline / Global Baseline / Attitude / Operational / UAM GPS Ray (World ENU)", 1)[0].rsplit("Class: rviz_default_plugins/MarkerArray", 1)[1]
    assert "Topic:" in ray_display
    assert "Marker Topic:" not in ray_display
    assert "Durability Policy: Transient Local" in ray_display
    assert "Reliability Policy: Reliable" in ray_display
    assert "gazebo_world_enu" in rviz
    assert "Cell Size: 100" in rviz
    assert "Distance: 2000" in rviz
    assert "Y: 1000" in rviz
    assert "Yaw: 1.57" in rviz
    assert "stereo_rays_nominal_pose: true" in rviz
    assert "stereo_rays_optical_pose: true" in rviz
    assert "stereo_rays_ideal: true" in rviz
    assert "stereo_rays_gps_baseline: true" in rviz
    assert "stereo_rays_global_baseline: true" in rviz
    assert "stereo_rays_attitude: true" in rviz
    assert "stereo_rays_operational: true" in rviz
    assert "stereo_rays_uam_gps: true" in rviz
    assert "stereo_cameras: true" in rviz
    assert "Name: Stereo Camera FOV (World ENU)" in rviz
    assert "- Class: rviz_default_plugins/MarkerArray\n      Enabled: false" in rviz
    assert "Name: Stereo Camera FOV (World ENU)\n      Namespaces:\n        stereo_camera_fov: true" in rviz
    assert "stereo_actual_drone: true" in rviz


def test_marker_source_contains_nominal_optical_ideal_gps_and_uam_ray_groups() -> None:
    source = (PACKAGE_ROOT / "stereo_ground_sim" / "stereo_distance_node.py").read_text(encoding="utf-8")

    assert "_rviz_live_worker_loop" in source
    assert "_latest_rviz_ideal_ray" in source
    assert "uam_gps_ray_visualization_world_enu_from_point" in source
    assert '"/stereo/live_detections"' in source
    assert "self.live_detections_pub.publish(self._detections_to_msg(left, right))" in source
    assert "rviz_live_latest_only" in source
    assert "rviz_live_ray_hz" in source
    assert 'diagnostic_rays = computed.get("diagnostic_rays", {})' in source
    assert "self._publish_ray_metrics(\n                        pair_id=pair.pair_id,\n                        nominal_pose=raw," in source
    assert "if not self._rviz_latest_only_markers_enabled()" in source
    assert 'worker = getattr(self, "rviz_live_thread", None)' in source
    assert "and worker is not None" in source
    assert "and worker.is_alive()" in source
    assert 'self.create_publisher(MarkerArray, "/stereo/ray_markers", ray_marker_qos)' in source
    assert 'self.create_publisher(MarkerArray, "/stereo/fov_markers", ray_marker_qos)' in source
    assert 'self.create_publisher(MarkerArray, "/stereo/camera_markers", ray_marker_qos)' in source
    assert "durability=DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "nominal_pose=metrics.get(\"_raw_ray\")" in source
    assert "ideal=ideal" in source
    assert "self.lock = threading.RLock()" in source
    assert "self.live_detections_pub.publish(detection_message)" in source
    assert "ideal=ideal" in source
    assert 'label="nominal_pose"' in source
    assert 'label="optical_pose"' in source
    assert 'label="ideal"' in source
    assert 'label="gps_baseline"' in source
    assert '"UAM GPS Ray"' in source
    assert 'namespace = "stereo_rays_uam_gps"' in source
    assert "uam_gps_ray=uam_gps_ray" in source
    assert 'label="global_baseline"' in source
    assert 'display_name = RAY_DISPLAY_NAMES[label]' in source
    assert 'f"left_{display_name}_ray"' in source
    assert 'f"right_{display_name}_ray"' in source
    assert 'f"{display_name}_closest_ray_gap"' in source
    assert "gazebo_observed_optical_pose" in source
    assert 'gps_baseline=computed.get("gps_baselines", {}).get("gps_baseline")' in source
    assert 'global_baseline=computed.get("gps_baselines", {}).get("global_baseline")' in source
    assert 'camera.ns = "stereo_cameras"' in source
    assert "camera.type = Marker.CUBE" in source
    assert "Keep configuration-only FOV geometry separate from live ray results." in source
    assert "faces.type = Marker.TRIANGLE_LIST" in source
    assert "edges.type = Marker.LINE_LIST" in source
    assert 'faces.ns = "stereo_camera_fov"' in source
    assert "self.create_timer(1.0, self._publish_camera_fov_markers)" in source
    assert "def _publish_camera_fov_markers(self):" in source
    assert "Publish the configured camera frustums without requiring a ray result." in source
    assert "def _camera_position_markers(self, stamp):" in source
    assert "Render the two configured camera origins in ``gazebo_world_enu`` metres." in source
    assert 'publisher = getattr(self, "fov_marker_pub", None)' in source
    assert "markers = self._camera_fov_markers(stamp)" in source
    assert "depth_m = min(max(1.0, float(getattr(self, \"camera_clip_far_m\", 2500.0))), 2500.0)" in source
    assert "faces.color.a = 0.20" in source
    assert "edges.scale.x = 3.0" in source
    assert 'marker.ns = "stereo_actual_drone"' in source
    assert "marker.type = Marker.MESH_RESOURCE" in source
    assert "marker.mesh_use_embedded_materials = False" in source
    assert "RViz vehicle rendering is published directly by" in source
    assert 'DRONE_MESH_RESOURCE = "package://stereo_ground_sim/models/x500_15m/meshes/NXP-HGD-CF.dae"' in source
    assert 'namespace = f"stereo_rays_{label}"' in source
    assert 'f"left_{display_name}_ray"' in source
    assert 'left_target, 2.0)' in source
    assert 'right_target, 2.0)' in source
    assert 'left_target, right_target, 1.5)' in source


def test_launch_exposes_latest_only_rviz_ray_controls() -> None:
    launch = (ROOT_DIR / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("rviz_live_latest_only", default_value="true")' in launch
    assert 'DeclareLaunchArgument("rviz_live_ray_hz", default_value="20.0")' in launch
    assert '"rviz_live_latest_only"' in launch
    assert '"rviz_live_ray_hz"' in launch


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
    for marker_id, label in (
        (0, "nominal_pose"),
        (10, "optical_pose"),
        (20, "ideal"),
        (30, "gps_baseline"),
        (40, "global_baseline"),
        (50, "attitude"),
        (70, "operational"),
    ):
        assert f'label="{label}",' in source
        assert f"marker_id={marker_id}," in source
    assert "line(80, namespace, \"UAM GPS Ray\"" in source
    for color in (
        "(0.65, 0.25, 0.95)",
        "(1.0, 0.62, 0.0)",
        "(1.0, 0.1, 0.75)",
        "(0.95, 0.95, 0.95)",
    ):
        assert f"left_color={color}" in source
