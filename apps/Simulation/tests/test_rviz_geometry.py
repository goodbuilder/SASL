from pathlib import Path
import sys
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.rviz_geometry import camera_frustum_geometry, ray_line_geometry


def test_frustum_output_is_world_enu_with_expected_forward_depth():
    camera = SimpleNamespace(
        center_world_enu_m=(1.0, 2.0, 3.0),
        right_world_enu=(1.0, 0.0, 0.0),
        down_world_enu=(0.0, 0.0, -1.0),
        forward_world_enu=(0.0, 1.0, 0.0),
    )
    geometry = camera_frustum_geometry(
        camera, width_px=4, height_px=2,
        horizontal_fov_rad=0.5, depth_m=10,
    )
    assert len(geometry["triangles_world_enu_m"]) == 18
    assert len(geometry["lines_world_enu_m"]) == 16
    far_points = geometry["triangles_world_enu_m"]
    assert any(point[1] == 12.0 for point in far_points)


def test_ray_geometry_accepts_only_computed_world_enu_primitives():
    assert ray_line_geometry({"computation_status": "invalid_input"}) is None
    geometry = {
        "left_origin_world_enu_m": (0, 0, 0),
        "left_target_world_enu_m": (1, 0, 0),
        "right_origin_world_enu_m": (0, 1, 0),
        "right_target_world_enu_m": (1, 1, 0),
        "midpoint_world_enu_m": (1, 0.5, 0),
    }
    assert ray_line_geometry(
        {"computation_status": "computed", "ray_visualization_world_enu": geometry}
    )["midpoint_world_enu_m"] == (1.0, 0.5, 0.0)
