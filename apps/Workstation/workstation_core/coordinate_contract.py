"""Canonical coordinate-frame and axis contracts for StereoWIRIS."""

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_AXIS_CONTRACT,
    BASELINE_CENTER_FRD_FRAME,
    BASELINE_CENTER_FRD_VECTOR_CONTRACT,
    CAMERA_OPTICAL_RDF_FRAME,
    LEFT_CAMERA_RDF_AXIS_CONTRACT,
    LEFT_CAMERA_RDF_FRAME,
)

WORLD_FRAME = "ned"
BODY_FRAME = "px4_frd"
CAMERA_FRAME = CAMERA_OPTICAL_RDF_FRAME
CAMERA_MOUNT_PROFILE = "front_camera_z_to_body_x_x_to_body_y"

BASELINE_TRANSLATION_FRAME = LEFT_CAMERA_RDF_FRAME
BASELINE_TRANSLATION_AXIS_CONTRACT = LEFT_CAMERA_RDF_AXIS_CONTRACT
MEASUREMENT_FRAME = BASELINE_CENTER_FRD_FRAME
MEASUREMENT_AXIS_CONTRACT = BASELINE_CENTER_FRD_AXIS_CONTRACT
PUBLIC_MEASUREMENT_FRAME = BASELINE_CENTER_FRD_FRAME
PUBLIC_MEASUREMENT_VECTOR_CONTRACT = BASELINE_CENTER_FRD_VECTOR_CONTRACT
LEGACY_XYZ_ALIAS_CONTRACT = "not_emitted"

# Read-only GNSS antenna-midpoint reference. This must never alias the public
# optical-center/camera-axis ``baseline_center_frd`` measurement frame above.
GNSS_BASELINE_CENTER_FRD_FRAME = "gnss_baseline_center_local_level_frd"
GNSS_BASELINE_CENTER_FRD_AXIS_CONTRACT = (
    "gnss_baseline_center_local_level_frd:forward_right_down"
)
GNSS_BASELINE_CENTER_FRD_VECTOR_CONTRACT = (
    "gnss_baseline_center_local_level_frd:[forward,right,down,range]"
)
