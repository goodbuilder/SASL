"""Canonical coordinate-frame and axis contracts for StereoDistance."""

WORLD_FRAME = "ned"
BODY_FRAME = "px4_frd"
CAMERA_FRAME = "opencv_optical"
CAMERA_MOUNT_PROFILE = "front_camera_z_to_body_x_x_to_body_y"

MOUNT_ASSUMPTION_MODEL = "nominal-shared-rig-v1"
MOUNT_ASSUMPTION_STATUS = "nominal shared rigid rig"
MOUNT_ASSUMPTION_DETAIL = "rtk_gnss_baseline_used_as_effective_stereo_translation"

MEASUREMENT_FRAME = "stereo_center_opencv_optical"
MEASUREMENT_AXIS_CONTRACT = "x/right,y/down,z/forward"
FRD_COMPARISON_AXIS_CONTRACT = "stereo_center_frd:forward_right_down"

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

SYSTEM_CLAIM = (
    "field stereo measurement workstation that records input provenance, "
    "coordinate contracts, and post-flight validation evidence"
)
CLAIM_BOUNDARY = (
    "does not guarantee hardware accuracy, all-axis correction improvement, "
    "or equivalence between simulation PASS and hardware PASS"
)

CONNECTION_POLICY = (
    "prefer simultaneous MAVLink and ROS2/uXRCE connections; MAVLink is the "
    "readiness, control, RTCM, parameter, shell, reboot, and recovery authority"
)
MAVLINK_PRIORITY_ROLE = "readiness-control-rtcm-recovery"
ROS2_PRIORITY_ROLE = "structured-uorb-telemetry-source-identity"
