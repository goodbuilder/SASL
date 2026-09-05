"""Read-only magnetometer yaw projections used by the Pixhawk UI.

The raw projection intentionally keeps the producer sensor frame.  It is a
diagnostic azimuth, not a replacement for PX4's estimator output.  The PX4
projection is only returned when the row's physical device identity matches
the active vehicle_magnetometer identity and the existing ATTITUDE sample is
fresh.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from math import atan2, degrees, hypot, isfinite


RAW_MAGNETOMETER_YAW_CONTRACT = (
    "RAW sensor-axis azimuth = atan2(-y, x); no CAL_MAG, sensor rotation, "
    "board rotation, or tilt correction"
)
DEFAULT_ATTITUDE_STALE_S = 2.0
MIN_HORIZONTAL_VECTOR = 1.0e-9

# PX4's Rotation enum (rotation.h) maps sensor-frame vectors into the vehicle
# body frame.  The table is kept here so the read-only Workstation projection
# follows the same CAL_MAG rotation contract without importing pymavlink.
_PX4_ROTATION_EULERS_DEG: dict[int, tuple[float, float, float]] = {
    0: (0.0, 0.0, 0.0),
    1: (0.0, 0.0, 45.0),
    2: (0.0, 0.0, 90.0),
    3: (0.0, 0.0, 135.0),
    4: (0.0, 0.0, 180.0),
    5: (0.0, 0.0, 225.0),
    6: (0.0, 0.0, 270.0),
    7: (0.0, 0.0, 315.0),
    8: (180.0, 0.0, 0.0),
    9: (180.0, 0.0, 45.0),
    10: (180.0, 0.0, 90.0),
    11: (180.0, 0.0, 135.0),
    12: (0.0, 180.0, 0.0),
    13: (180.0, 0.0, 225.0),
    14: (180.0, 0.0, 270.0),
    15: (180.0, 0.0, 315.0),
    16: (90.0, 0.0, 0.0),
    17: (90.0, 0.0, 45.0),
    18: (90.0, 0.0, 90.0),
    19: (90.0, 0.0, 135.0),
    20: (270.0, 0.0, 0.0),
    21: (270.0, 0.0, 45.0),
    22: (270.0, 0.0, 90.0),
    23: (270.0, 0.0, 135.0),
    24: (0.0, 90.0, 0.0),
    25: (0.0, 270.0, 0.0),
    26: (0.0, 180.0, 90.0),
    27: (0.0, 180.0, 270.0),
    28: (90.0, 90.0, 0.0),
    29: (180.0, 90.0, 0.0),
    30: (270.0, 90.0, 0.0),
    31: (90.0, 180.0, 0.0),
    32: (270.0, 180.0, 0.0),
    33: (90.0, 270.0, 0.0),
    34: (180.0, 270.0, 0.0),
    35: (270.0, 270.0, 0.0),
    36: (90.0, 180.0, 90.0),
    37: (90.0, 0.0, 270.0),
    38: (90.0, 68.0, 293.0),
    39: (0.0, 315.0, 0.0),
    40: (90.0, 315.0, 0.0),
}


def wrap_signed_yaw_deg(value: float) -> float:
    """Return a finite yaw in the half-open interval ``[-180, 180)``."""

    angle = float(value)
    if not isfinite(angle):
        raise ValueError("yaw must be finite")
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return -180.0 if wrapped == 180.0 else wrapped


def raw_magnetometer_yaw_deg(x: object, y: object) -> float | None:
    """Compute a diagnostic yaw from the raw producer-axis horizontal vector.

    The sign follows PX4's body-FRD heading convention when the producer axes
    happen to be aligned with body forward/right.  The caller must still label
    the result as raw sensor-axis data because no alignment is assumed here.
    """

    try:
        x_value = float(x)
        y_value = float(y)
    except (TypeError, ValueError):
        return None
    if not isfinite(x_value) or not isfinite(y_value):
        return None
    if hypot(x_value, y_value) <= MIN_HORIZONTAL_VECTOR:
        return None
    return wrap_signed_yaw_deg(degrees(atan2(-y_value, x_value)))


def _finite_param(params: Mapping[str, object], name: str) -> float | None:
    value = params.get(name)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if isfinite(number) else None


def _mat_mul(left: tuple[tuple[float, float, float], ...], right: tuple[tuple[float, float, float], ...]):
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def _mat_vec(matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]):
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3))


def _rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Return PX4-compatible ZYX Euler rotation for a sensor-to-body vector."""

    from math import cos, radians, sin

    roll, pitch, yaw = radians(roll_deg), radians(pitch_deg), radians(yaw_deg)
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    return _mat_mul(
        (
            (cy, -sy, 0.0),
            (sy, cy, 0.0),
            (0.0, 0.0, 1.0),
        ),
        _mat_mul(
            (
                (cp, 0.0, sp),
                (0.0, 1.0, 0.0),
                (-sp, 0.0, cp),
            ),
            (
                (1.0, 0.0, 0.0),
                (0.0, cr, -sr),
                (0.0, sr, cr),
            ),
        ),
    )


def calibrated_magnetometer_yaw_deg(
    *,
    x: object,
    y: object,
    z: object,
    calibration_params: Mapping[str, object] | None,
    calibration_slot: int | None,
    board_params: Mapping[str, object] | None = None,
    internal_sensor: bool = False,
) -> dict[str, object]:
    """Apply PX4 CAL_MAG corrections and return a body-frame corrected yaw.

    PX4's Magnetometer::Correct() is ``R * S * (data - offset)``. Power/current
    compensation is deliberately rejected here because sensor_mag readback has
    no contemporaneous power/current sample. The caller can display the state
    instead of silently presenting a partially corrected value.
    """

    values = tuple(_finite_param({"v": value}, "v") for value in (x, y, z))
    if any(value is None for value in values):
        return {
            "corrected_yaw_deg": None,
            "corrected_yaw_state": "WAITING",
            "corrected_yaw_reason": "magnetometer vector incomplete",
        }
    if calibration_params is None or calibration_slot is None:
        return {
            "corrected_yaw_deg": None,
            "corrected_yaw_state": "PARAM WAITING",
            "corrected_yaw_reason": "CAL_MAG readback required",
        }

    prefix = f"CAL_MAG{int(calibration_slot)}_"
    offset = tuple(_finite_param(calibration_params, prefix + suffix) for suffix in ("XOFF", "YOFF", "ZOFF"))
    diagonal = tuple(_finite_param(calibration_params, prefix + suffix) for suffix in ("XSCALE", "YSCALE", "ZSCALE"))
    offdiagonal = tuple(
        _finite_param(calibration_params, prefix + suffix)
        for suffix in ("XODIAG", "YODIAG", "ZODIAG")
    )
    if any(value is None for value in (*offset, *diagonal, *offdiagonal)):
        return {
            "corrected_yaw_deg": None,
            "corrected_yaw_state": "PARAM INCOMPLETE",
            "corrected_yaw_reason": f"{prefix[:-1]} calibration fields incomplete",
        }

    compensation = tuple(
        _finite_param(calibration_params, prefix + suffix) or 0.0
        for suffix in ("XCOMP", "YCOMP", "ZCOMP")
    )
    if any(abs(value) > 1.0e-12 for value in compensation):
        return {
            "corrected_yaw_deg": None,
            "corrected_yaw_state": "POWER COMP PENDING",
            "corrected_yaw_reason": "CAL_MAG power/current compensation needs a contemporaneous power sample",
        }

    scale_matrix = (
        (diagonal[0], offdiagonal[0], offdiagonal[1]),
        (offdiagonal[0], diagonal[1], offdiagonal[2]),
        (offdiagonal[1], offdiagonal[2], diagonal[2]),
    )
    corrected_sensor = _mat_vec(
        scale_matrix,
        (values[0] - offset[0], values[1] - offset[1], values[2] - offset[2]),
    )

    rotation_code = _finite_param(calibration_params, prefix + "ROT")
    custom_euler = tuple(
        _finite_param(calibration_params, prefix + suffix) or 0.0
        for suffix in ("ROLL", "PITCH", "YAW")
    )
    if internal_sensor:
        board_rotation = _finite_param(board_params or {}, "SENS_BOARD_ROT")
        if board_rotation is None:
            return {
                "corrected_yaw_deg": None,
                "corrected_yaw_state": "BOARD ROTATION WAITING",
                "corrected_yaw_reason": "SENS_BOARD_ROT readback required for internal sensor",
            }
        board_euler = _PX4_ROTATION_EULERS_DEG.get(int(board_rotation))
        if board_euler is None:
            return {
                "corrected_yaw_deg": None,
                "corrected_yaw_state": "BOARD ROTATION UNKNOWN",
                "corrected_yaw_reason": f"unsupported SENS_BOARD_ROT={int(board_rotation)}",
            }
        rotation = _rotation_matrix(*board_euler)
    elif rotation_code is not None and int(rotation_code) == 100:
        rotation = _rotation_matrix(*custom_euler)
    else:
        euler = _PX4_ROTATION_EULERS_DEG.get(0 if rotation_code is None else int(rotation_code))
        if euler is None:
            return {
                "corrected_yaw_deg": None,
                "corrected_yaw_state": "SENSOR ROTATION UNKNOWN",
                "corrected_yaw_reason": f"unsupported CAL_MAG rotation={rotation_code}",
            }
        rotation = _rotation_matrix(*euler)

    corrected_body = _mat_vec(rotation, corrected_sensor)
    if hypot(corrected_body[0], corrected_body[1]) <= MIN_HORIZONTAL_VECTOR:
        return {
            "corrected_yaw_deg": None,
            "corrected_yaw_state": "INVALID VECTOR",
            "corrected_yaw_reason": "corrected horizontal vector is near zero",
        }
    return {
        "corrected_yaw_deg": wrap_signed_yaw_deg(degrees(atan2(-corrected_body[1], corrected_body[0]))),
        "corrected_yaw_state": "CALIBRATED",
        "corrected_yaw_reason": "CAL_MAG offset/scale/misalignment/rotation applied",
        "corrected_vector": corrected_body,
    }


@dataclass(frozen=True)
class MagnetometerYawProjection:
    """One row's raw yaw and safely correlated PX4 yaw reference."""

    raw_yaw_deg: float | None
    px4_yaw_deg: float | None
    delta_yaw_deg: float | None
    px4_yaw_state: str
    px4_yaw_reason: str

    def to_mapping(self) -> dict[str, object]:
        return {
            "raw_yaw_deg": self.raw_yaw_deg,
            "px4_yaw_deg": self.px4_yaw_deg,
            "delta_yaw_deg": self.delta_yaw_deg,
            "px4_yaw_state": self.px4_yaw_state,
            "px4_yaw_reason": self.px4_yaw_reason,
            "raw_yaw_contract": RAW_MAGNETOMETER_YAW_CONTRACT,
        }


def project_magnetometer_yaw(
    *,
    x: object,
    y: object,
    sensor_device_id: int | None,
    active_device_id: int | None,
    px4_yaw_deg: object,
    px4_age_s: object,
    px4_stale_s: float = DEFAULT_ATTITUDE_STALE_S,
) -> MagnetometerYawProjection:
    """Project one row without guessing identity or estimator semantics."""

    raw_yaw = raw_magnetometer_yaw_deg(x, y)
    try:
        stale_limit = float(px4_stale_s)
    except (TypeError, ValueError) as exc:
        raise ValueError("px4_stale_s must be finite and positive") from exc
    if not isfinite(stale_limit) or stale_limit <= 0.0:
        raise ValueError("px4_stale_s must be finite and positive")

    try:
        px4_yaw = None if px4_yaw_deg is None else wrap_signed_yaw_deg(float(px4_yaw_deg))
    except (TypeError, ValueError):
        px4_yaw = None
    if px4_yaw is None:
        return MagnetometerYawProjection(
            raw_yaw_deg=raw_yaw,
            px4_yaw_deg=None,
            delta_yaw_deg=None,
            px4_yaw_state="WAITING",
            px4_yaw_reason="PX4 ATTITUDE yaw unavailable",
        )

    try:
        age = None if px4_age_s is None else float(px4_age_s)
    except (TypeError, ValueError):
        age = None
    if age is None or not isfinite(age):
        return MagnetometerYawProjection(
            raw_yaw_deg=raw_yaw,
            px4_yaw_deg=None,
            delta_yaw_deg=None,
            px4_yaw_state="WAITING",
            px4_yaw_reason="PX4 ATTITUDE freshness unavailable",
        )
    if age > stale_limit:
        return MagnetometerYawProjection(
            raw_yaw_deg=raw_yaw,
            px4_yaw_deg=None,
            delta_yaw_deg=None,
            px4_yaw_state="STALE",
            px4_yaw_reason=f"PX4 ATTITUDE stale ({age:.2f}s)",
        )
    if active_device_id is None:
        return MagnetometerYawProjection(
            raw_yaw_deg=raw_yaw,
            px4_yaw_deg=None,
            delta_yaw_deg=None,
            px4_yaw_state="ACTIVE ID WAITING",
            px4_yaw_reason="active vehicle_magnetometer device ID unavailable",
        )
    if sensor_device_id is None:
        return MagnetometerYawProjection(
            raw_yaw_deg=raw_yaw,
            px4_yaw_deg=None,
            delta_yaw_deg=None,
            px4_yaw_state="IDENTITY UNKNOWN",
            px4_yaw_reason="row device ID unavailable; PX4 yaw not copied",
        )
    if int(sensor_device_id) != int(active_device_id):
        return MagnetometerYawProjection(
            raw_yaw_deg=raw_yaw,
            px4_yaw_deg=None,
            delta_yaw_deg=None,
            px4_yaw_state="NOT SELECTED",
            px4_yaw_reason=(
                f"row device {int(sensor_device_id)} != active device {int(active_device_id)}"
            ),
        )

    delta = None if raw_yaw is None else wrap_signed_yaw_deg(px4_yaw - raw_yaw)
    return MagnetometerYawProjection(
        raw_yaw_deg=raw_yaw,
        px4_yaw_deg=px4_yaw,
        delta_yaw_deg=delta,
        px4_yaw_state="SELECTED",
        px4_yaw_reason="active device matched; fresh PX4 ATTITUDE reference",
    )
