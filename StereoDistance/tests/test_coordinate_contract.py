from wiris_stereo_distance import coordinate_contract as contract
from wiris_stereo_distance import gui, triangulation, uam_telemetry


def test_coordinate_contract_canonical_values() -> None:
    assert contract.WORLD_FRAME == "ned"
    assert contract.BODY_FRAME == "px4_frd"
    assert contract.CAMERA_FRAME == "opencv_optical"
    assert contract.CAMERA_MOUNT_PROFILE == "front_camera_z_to_body_x_x_to_body_y"
    assert contract.MOUNT_ASSUMPTION_MODEL == "nominal-shared-rig-v1"
    assert contract.MAVLINK_PRIORITY_ROLE == "readiness-control-rtcm-recovery"
    assert contract.ROS2_PRIORITY_ROLE == "structured-uorb-telemetry-source-identity"
    assert contract.MEASUREMENT_FRAME == "stereo_center_opencv_optical"
    assert contract.MEASUREMENT_AXIS_CONTRACT == "x/right,y/down,z/forward"
    assert contract.FRD_COMPARISON_AXIS_CONTRACT == "stereo_center_frd:forward_right_down"
    assert contract.TRUTH_FRAME == "px4_local_ned"
    assert contract.TRUTH_AXIS_CONTRACT == "n/north,e/east,d/down"
    assert contract.MIXED_FRAME == "mixed-frame"


def test_coordinate_contract_is_shared_by_modules() -> None:
    assert triangulation.WORLD_FRAME == contract.WORLD_FRAME
    assert triangulation.BODY_FRAME == contract.BODY_FRAME
    assert triangulation.CAMERA_FRAME == contract.CAMERA_FRAME
    assert triangulation.CAMERA_MOUNT_PROFILE == contract.CAMERA_MOUNT_PROFILE
    assert uam_telemetry.MEASUREMENT_FRAME == contract.MEASUREMENT_FRAME
    assert uam_telemetry.MEASUREMENT_AXIS_CONTRACT == contract.MEASUREMENT_AXIS_CONTRACT
    assert uam_telemetry.TRUTH_FRAME == contract.TRUTH_FRAME
    assert uam_telemetry.TRUTH_AXIS_CONTRACT == contract.TRUTH_AXIS_CONTRACT
    assert gui.MEASUREMENT_FRAME == contract.MEASUREMENT_FRAME
    assert gui.MEASUREMENT_AXIS_CONTRACT == contract.MEASUREMENT_AXIS_CONTRACT
    assert gui.TRUTH_FRAME == contract.TRUTH_FRAME
    assert gui.TRUTH_AXIS_CONTRACT == contract.TRUTH_AXIS_CONTRACT
