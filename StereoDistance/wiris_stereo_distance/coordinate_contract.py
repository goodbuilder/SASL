"""Canonical coordinate-frame and axis contracts for StereoDistance."""

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_FRAME,
    CAMERA_OPTICAL_FRAME,
    STEREO_CENTER_OPTICAL_FRAME,
)

WORLD_FRAME = "ned"
BODY_FRAME = "px4_frd"
CAMERA_FRAME = CAMERA_OPTICAL_FRAME
CAMERA_MOUNT_PROFILE = "front_camera_z_to_body_x_x_to_body_y"

MEASUREMENT_FRAME = STEREO_CENTER_OPTICAL_FRAME
MEASUREMENT_AXIS_CONTRACT = "x/right,y/down,z/forward"
PUBLIC_MEASUREMENT_FRAME = BASELINE_CENTER_FRD_FRAME
FRD_COMPARISON_AXIS_CONTRACT = "baseline_center_frd:forward_right_down"

TRUTH_FRAME = "px4_local_ned"
TRUTH_AXIS_CONTRACT = "n/north,e/east,d/down"

FINAL_ESTIMATE_FRAME = TRUTH_FRAME
FINAL_ESTIMATE_AXIS_CONTRACT = TRUTH_AXIS_CONTRACT

MIXED_FRAME = "mixed-frame"

CAMERA_RIGHT_FIELD = "measurement_right_m"
CAMERA_DOWN_FIELD = "measurement_down_m"
CAMERA_FORWARD_FIELD = "measurement_forward_m"

TRUTH_RDF_RIGHT_FIELD = "truth_rdf_right_m"
TRUTH_RDF_DOWN_FIELD = "truth_rdf_down_m"
TRUTH_RDF_FORWARD_FIELD = "truth_rdf_forward_m"

ERROR_RDF_RIGHT_FIELD = "error_rdf_right_m"
ERROR_RDF_DOWN_FIELD = "error_rdf_down_m"
ERROR_RDF_FORWARD_FIELD = "error_rdf_forward_m"

TRUTH_FRD_FORWARD_FIELD = "truth_frd_forward_m"
TRUTH_FRD_RIGHT_FIELD = "truth_frd_right_m"
TRUTH_FRD_DOWN_FIELD = "truth_frd_down_m"

ERROR_FRD_FORWARD_FIELD = "error_frd_forward_m"
ERROR_FRD_RIGHT_FIELD = "error_frd_right_m"
ERROR_FRD_DOWN_FIELD = "error_frd_down_m"

ESTIMATED_NORTH_FIELD = "estimated_north_m"
ESTIMATED_EAST_FIELD = "estimated_east_m"
ESTIMATED_DOWN_FIELD = "estimated_down_m"

TRUTH_NORTH_FIELD = "truth_north_m"
TRUTH_EAST_FIELD = "truth_east_m"
TRUTH_DOWN_FIELD = "truth_down_m"

FINAL_ERROR_NORTH_FIELD = "error_north_m"
FINAL_ERROR_EAST_FIELD = "error_east_m"
FINAL_ERROR_DOWN_FIELD = "error_down_m"
FINAL_POSITION_ERROR_FIELD = "position_error_m"
