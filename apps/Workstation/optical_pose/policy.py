"""Existing optical-pose DOF masks and capture evidence thresholds.

Angles are degrees, translations NED metres; synchronization deltas are
host-monotonic seconds. Values and advisory/apply policies are unchanged.
"""

from Workstation.workstation_core.optical_pose import PoseDofMask, POSE_DOF_NAMES

_ALL_DOF = PoseDofMask.from_names(POSE_DOF_NAMES)

_HORIZON_DOF = PoseDofMask(roll_deg=True, pitch_deg=True, yaw_deg=False)

DRONE_RTK_FRAME_PAIR_MAX_DELTA_S = 0.350

DRONE_RTK_FRAME_GNSS_MAX_ABS_DELTA_S = 0.350

_YAW_DOF = PoseDofMask(roll_deg=False, pitch_deg=False, yaw_deg=True)

_ROTATION_DOF = PoseDofMask(roll_deg=True, pitch_deg=True, yaw_deg=True)

CLOUD_FRAME_PAIR_MAX_DELTA_S = DRONE_RTK_FRAME_PAIR_MAX_DELTA_S

_DRONE_ROTATION_DOF = PoseDofMask(roll_deg=True, pitch_deg=True, yaw_deg=True)

_TRANSLATION_DOF = PoseDofMask(roll_deg=False, pitch_deg=False, yaw_deg=False, north_m=True, east_m=True, down_m=True)

_DRONE_RTK_MULTIVIEW_MIN_OBSERVATIONS = 4

_MOUNTAIN_MINIMUM_GNSS_FIX_TYPE = 3
