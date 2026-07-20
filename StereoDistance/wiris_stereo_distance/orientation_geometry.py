from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2, copysign, degrees, pi, sqrt
from typing import Iterable


Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class RelativeAttitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


def normalize_quaternion(q: Iterable[float]) -> Quaternion:
    w, x, y, z = (float(value) for value in q)
    norm = sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        raise ValueError("zero-length quaternion")
    return (w / norm, x / norm, y / norm, z / norm)


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    w, x, y, z = q
    return (w, -x, -y, -z)


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return normalize_quaternion(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )
    )


def quaternion_to_euler_deg(q: Iterable[float]) -> RelativeAttitude:
    w, x, y, z = normalize_quaternion(q)
    roll = degrees(atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = (
        degrees(copysign(pi / 2.0, sin_pitch))
        if abs(sin_pitch) >= 1.0
        else degrees(asin(sin_pitch))
    )
    yaw = degrees(atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return RelativeAttitude(roll, pitch, yaw)


def relative_attitude(left_q: Iterable[float], right_q: Iterable[float]) -> RelativeAttitude:
    left = normalize_quaternion(left_q)
    right = normalize_quaternion(right_q)
    return quaternion_to_euler_deg(quaternion_multiply(right, quaternion_conjugate(left)))
