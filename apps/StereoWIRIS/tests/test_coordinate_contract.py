from wiris_stereo_distance import coordinate_contract as contract
from wiris_stereo_distance import triangulation


def test_coordinate_contract_canonical_values() -> None:
    assert contract.WORLD_FRAME == "ned"
    assert contract.BODY_FRAME == "px4_frd"
    assert contract.CAMERA_FRAME == "camera_optical_rdf"
    assert contract.CAMERA_MOUNT_PROFILE == "front_camera_z_to_body_x_x_to_body_y"
    assert contract.BASELINE_TRANSLATION_FRAME == "left_camera_rdf"
    assert contract.BASELINE_TRANSLATION_AXIS_CONTRACT == "left_camera_rdf:right_down_forward"
    assert contract.MEASUREMENT_FRAME == "baseline_center_frd"
    assert contract.MEASUREMENT_AXIS_CONTRACT == "baseline_center_frd:forward_right_down"
    assert contract.PUBLIC_MEASUREMENT_FRAME == "baseline_center_frd"
    assert contract.PUBLIC_MEASUREMENT_VECTOR_CONTRACT == "baseline_center_frd:[forward,right,down,range]"
    assert contract.LEGACY_XYZ_ALIAS_CONTRACT == "not_emitted"


def test_coordinate_contract_is_shared_by_modules() -> None:
    assert triangulation.WORLD_FRAME == contract.WORLD_FRAME
    assert triangulation.BODY_FRAME == contract.BODY_FRAME
    assert triangulation.CAMERA_FRAME == contract.CAMERA_FRAME
    assert triangulation.CAMERA_MOUNT_PROFILE == contract.CAMERA_MOUNT_PROFILE
