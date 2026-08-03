from pathlib import Path
import sys
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from camera_oracle_test_support import seed_observed_camera_oracle


def _node():
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    node.synthetic_detection_bbox_size_px = 32.0
    node._ensure_camera_frames()
    seed_observed_camera_oracle(node)
    return node


def test_ideal_projection_rejects_out_of_fov_and_accepts_valid_reference_points():
    node = _node()
    out_of_fov = node.baseline_center_frd.point_to_world((120.0, -3.0, 2.0))
    in_fov = node.baseline_center_frd.point_to_world((500.0, -3.0, 2.0))

    rejected = node._ideal_detection_pair_from_reference_world(out_of_fov, 1280, 720)
    accepted = node._ideal_detection_pair_from_reference_world(in_fov, 1280, 720)

    assert rejected == (None, None)
    assert accepted[0] is not None
    assert accepted[1] is not None
    assert 0.0 <= accepted[0]["cx"] < 1280.0
    assert 0.0 <= accepted[1]["cx"] < 1280.0


def test_rviz_ideal_ray_falls_back_to_reference_point_outside_fov():
    node = _node()
    out_of_fov = node.baseline_center_frd.point_to_world((120.0, -3.0, 2.0))
    node.cached_ground_truth_position = {
        "ground_truth_world_enu_m": out_of_fov,
    }
    pair = SimpleNamespace(left_message=SimpleNamespace(width=1280, height=720))

    ray = node._latest_rviz_ideal_ray(pair)

    assert ray is not None
    assert ray["computation_status"] == "computed"
    assert ray["ray_visualization_world_enu"]["midpoint_world_enu_m"] == out_of_fov
