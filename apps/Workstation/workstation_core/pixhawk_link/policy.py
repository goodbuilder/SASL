"""Existing Pixhawk runtime constants and stock stream contract.

These values retain their previous units, retry limits and source identities.
No device is opened when the policy is imported.
"""

from __future__ import annotations

import re
from Workstation.workstation_core.error_throttle import DEFAULT_ERROR_LOG_THROTTLE_S


MAVLINK_MESSAGE_IDS = {
    "ATTITUDE": 30,
    "SYS_STATUS": 1,
    "GPS_RAW_INT": 24,
    "RAW_IMU": 27,
    "LOCAL_POSITION_NED": 32,
    "GLOBAL_POSITION_INT": 33,
    "VFR_HUD": 74,
    "GPS_RTK": 127,
    "GPS2_RAW": 124,
    "BATTERY_STATUS": 147,
    "GPS2_RTK": 128,
    "SCALED_IMU": 26,
    "SCALED_IMU2": 116,
    "SCALED_IMU3": 129,
    "DEBUG_VECT": 250,
    "SASL_CAN_MAG": 180,
}


MAVLINK_STOCK_STREAM_REQUESTS = (
    ("ATTITUDE", 30.0),
    ("SYS_STATUS", 1.0),
    ("GPS_RAW_INT", 5.0),
    ("GPS2_RAW", 5.0),
    ("GPS_RTK", 5.0),
    ("GPS2_RTK", 5.0),
    ("SCALED_IMU", 20.0),
    ("SCALED_IMU2", 20.0),
    ("SCALED_IMU3", 20.0),
    ("DEBUG_VECT", 20.0),
    ("RAW_IMU", 20.0),
    ("LOCAL_POSITION_NED", 10.0),
    ("GLOBAL_POSITION_INT", 10.0),
    ("VFR_HUD", 5.0),
    ("BATTERY_STATUS", 2.0),
)


MAVLINK_STOCK_STREAM_CONSUMERS = {
    "ATTITUDE": ("extrinsic/status/measurement",),
    "SYS_STATUS": ("optional timeline", "battery evidence"),
    "GPS_RAW_INT": ("GNSS position/source selection/measurement",),
    "GPS2_RAW": ("GNSS position/source selection/measurement",),
    "GPS_RTK": ("GNSS relative baseline",),
    "GPS2_RTK": ("GNSS relative baseline",),
    "SCALED_IMU": ("mag sensor presentation/evidence",),
    "SCALED_IMU2": ("mag sensor presentation/evidence",),
    "SCALED_IMU3": ("mag sensor presentation/evidence",),
    "DEBUG_VECT": ("CAN-mag compatibility presentation/evidence",),
    "RAW_IMU": ("optional timeline/session evidence",),
    "LOCAL_POSITION_NED": ("optional timeline/session evidence",),
    "GLOBAL_POSITION_INT": ("optional timeline/session evidence",),
    "VFR_HUD": ("optional timeline/session evidence",),
    "BATTERY_STATUS": ("optional timeline/battery evidence",),
}


MAVLINK_RM3100_STREAM_REQUESTS = MAVLINK_STOCK_STREAM_REQUESTS


MAVLINK_MAX_MESSAGES_PER_TICK = 256


MAVLINK_MAX_PROCESS_TIME_S = 0.020


MAVLINK_RECEIVE_ERROR_INTERVAL_S = DEFAULT_ERROR_LOG_THROTTLE_S


MAVLINK_STREAM_REQUEST_MAX_ATTEMPTS = 3


MAVLINK_STREAM_REQUEST_RETRY_S = 2.0


MAVLINK_DUPLICATE_TTL_S = 0.250


MAVLINK_DUPLICATE_CACHE_SIZE = 512


MAVLINK_ATTITUDE_STALE_S = 2.0


MAVLINK_ATTITUDE_RECOVERY_COOLDOWN_S = 10.0


MAVLINK_IMU_STREAM_STALE_S = 1.0


MAVLINK_IMU_STREAM_RECOVERY_COOLDOWN_S = 10.0


MAVLINK_GCS_SOURCE_SYSTEM = 255


MAVLINK_GCS_SOURCE_COMPONENT = 190


MAVLINK_GCS_HEARTBEAT_INTERVAL_S = 1.0


MAVLINK_COMMAND_RECEIVE_POLL_S = 0.05


MAVLINK_SHELL_OUTPUT_IDLE_S = 0.5


_NSH_PROMPT_TAIL_RE = re.compile(r"nsh>\s*(?:\x1b?\[[0-9;]*[A-Za-z]\s*)*$")


MAVLINK_RTCM_STATUS_INTERVAL_S = 5.0


MAVLINK_EXPECTED_SYSTEM_IDS = {"left": 1, "right": 2, "drone": 3}


DEFAULT_USB_MAVLINK_BAUDRATES = (57600, 115200, 921600)


USB_MAVLINK_IDENTITY_MAX_AGE_S = 30.0


_MAVLINK_FLOAT_PARAM_TYPES = (9, 10)
