"""Stable PX4 camera-attitude and fixed-extrinsic provenance constants."""

PX4_CAMERA_EXTRINSIC_SOURCE_FILE = (
    "apps/Simulation/stereo_ground_sim/worlds/stereo_ground_drone.sdf"
)
PX4_CAMERA_EXTRINSIC_SOURCE_TYPE = "sdf_fixed_joint_axis_contract"
PX4_CAMERA_EXTRINSIC_PARENT_LINK = "base_link"
PX4_CAMERA_EXTRINSIC_CHILD_LINK = "optical_link"
PX4_ATTITUDE_MODEL_VERSION = "configured_independent_body_attitude_v2"
PX4_ATTITUDE_REFERENCE_SOURCE = "configured_px4_attitude_rpy"
PX4_ATTITUDE_FRAME = "body_frd"
PX4_ATTITUDE_COMPOSITION = "canonical_rig_heading_plus_configured_rpy"
