from wiris_stereo_distance import coordinate_contract as contract
from wiris_stereo_distance import gui, triangulation


def test_coordinate_contract_canonical_values() -> None:
    assert contract.WORLD_FRAME == "ned"
    assert contract.BODY_FRAME == "px4_frd"
    assert contract.CAMERA_FRAME == "camera_optical_rdf"
    assert contract.CAMERA_MOUNT_PROFILE == "front_camera_z_to_body_x_x_to_body_y"
    assert contract.MEASUREMENT_FRAME == "left_camera_rdf"
    assert contract.MEASUREMENT_AXIS_CONTRACT == "left_camera_rdf:right_down_forward"
    assert contract.PUBLIC_MEASUREMENT_FRAME == "left_camera_rdf"
    assert contract.PUBLIC_MEASUREMENT_VECTOR_CONTRACT == "left_camera_rdf:[right,down,forward,range]"
    assert contract.LEGACY_XYZ_ALIAS_CONTRACT == "x=right,y=down,z=forward"


def test_coordinate_contract_is_shared_by_modules() -> None:
    assert triangulation.WORLD_FRAME == contract.WORLD_FRAME
    assert triangulation.BODY_FRAME == contract.BODY_FRAME
    assert triangulation.CAMERA_FRAME == contract.CAMERA_FRAME
    assert triangulation.CAMERA_MOUNT_PROFILE == contract.CAMERA_MOUNT_PROFILE
    assert gui.MEASUREMENT_FRAME == contract.MEASUREMENT_FRAME
    assert gui.MEASUREMENT_AXIS_CONTRACT == contract.MEASUREMENT_AXIS_CONTRACT
