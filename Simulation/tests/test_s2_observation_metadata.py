from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME
from stereo_ground_sim.coordinate_frames import VEHICLE_REFERENCE_LINK
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


@pytest.mark.parametrize(
    ("mode", "expected"),
    (
        (
            "ideal_reference_point",
            {
                "observation_source": "gazebo_reference_point_projection",
                "measurement_gt_independent": False,
                "reference_source": VEHICLE_REFERENCE_LINK,
                "projection_camera_pose_source": "actual",
            },
        ),
        (
            "bbox_yolo",
            {
                "observation_source": "yolo_bbox_center",
                "measurement_gt_independent": True,
                "reference_source": "not_applicable",
                "projection_camera_pose_source": "rendered_image_observation",
            },
        ),
    ),
)
def test_actual_observation_metadata_dict_is_isolated_by_mode(mode, expected):
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.observation_mode = mode

    metadata = node._observation_metadata()

    assert metadata["observation_mode"] == mode
    assert metadata["observation_source"] == expected["observation_source"]
    assert metadata["measurement_gt_independent"] is expected["measurement_gt_independent"]
    assert metadata["reference_source"] == expected["reference_source"]
    assert metadata["projection_camera_pose_source"] == expected["projection_camera_pose_source"]
    assert metadata["raw_interpretation_pose_source"] == "nominal"
    assert metadata["corrected_interpretation_pose_source"] == "actual"
    assert metadata["correction_source"] == "injected_attitude_oracle"
    assert metadata["evaluation_gt_source"] == "gazebo_reference_link_world_pose"
    assert metadata["output_frame"] == BASELINE_CENTER_FRD_FRAME


def test_bbox_metadata_does_not_reuse_ideal_observation_reference():
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.observation_mode = "bbox_yolo"

    metadata = node._observation_metadata()

    assert metadata["reference_source"] != VEHICLE_REFERENCE_LINK
    assert metadata["evaluation_gt_source"] == "gazebo_reference_link_world_pose"
