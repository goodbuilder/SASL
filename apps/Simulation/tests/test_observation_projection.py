from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from sasl_common.stereo_geometry import PinholeIntrinsics
from stereo_ground_sim.coordinate_frames import nominal_camera_poses
from stereo_ground_sim.observation_projection import detection_pair_from_world


def test_world_enu_point_projects_to_opencv_uv_bbox_contract():
    left, right = nominal_camera_poses(2.0, 0.0)
    detections = detection_pair_from_world(
        (0.0, 10.0, 0.0),
        left_camera=left, right_camera=right,
        intrinsics=PinholeIntrinsics(100, 100, 50, 40),
        width_px=100, height_px=80, bbox_size_px=10,
        enforce_fov_bounds=True,
    )
    assert all(item is not None for item in detections)
    for detection in detections:
        assert detection["bbox"] == (
            detection["cx"] - 5, detection["cy"] - 5,
            detection["cx"] + 5, detection["cy"] + 5,
        )


def test_out_of_fov_reference_is_rejected_only_when_gate_enabled():
    left, right = nominal_camera_poses(2.0, 0.0)
    kwargs = dict(
        left_camera=left, right_camera=right,
        intrinsics=PinholeIntrinsics(100, 100, 50, 40),
        width_px=100, height_px=80, bbox_size_px=10,
    )
    assert detection_pair_from_world((100, 10, 0), enforce_fov_bounds=True, **kwargs) == (None, None)
    assert all(
        item is not None
        for item in detection_pair_from_world((100, 10, 0), enforce_fov_bounds=False, **kwargs)
    )
