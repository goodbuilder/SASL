from __future__ import annotations

from dataclasses import dataclass
from math import acos, asin, atan2, copysign, degrees, isfinite, pi, sqrt
from typing import Iterable


Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class RelativeAttitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


def normalize_quaternion(q: Iterable[float]) -> Quaternion:
    w, x, y, z = (float(value) for value in q)
    if not all(isfinite(value) for value in (w, x, y, z)):
        raise ValueError("quaternion must contain four finite values")
    norm = sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1.0e-12:
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


def _quaternion_multiply_raw(
    a: Quaternion,
    b: Quaternion,
) -> Quaternion:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def rotate_vector_by_quaternion(
    vector: tuple[float, float, float],
    q: Iterable[float],
) -> tuple[float, float, float]:
    """Rotate a vector with a PX4-style ``q(w,x,y,z)`` quaternion.

    The quaternion is interpreted as a rotation from the source frame to the
    destination frame.  The vector magnitude is preserved; the public
    ``quaternion_multiply`` helper intentionally normalizes its result and is
    therefore not used for the intermediate pure-vector products here.
    """

    quat = normalize_quaternion(q)
    pure = (0.0, float(vector[0]), float(vector[1]), float(vector[2]))
    rotated = _quaternion_multiply_raw(
        _quaternion_multiply_raw(quat, pure),
        quaternion_conjugate(quat),
    )
    return rotated[1], rotated[2], rotated[3]


def world_ned_vector_to_body_frd(
    world_vector: tuple[float, float, float],
    body_to_world_q: Iterable[float],
) -> tuple[float, float, float]:
    """Express a world-NED vector in the PX4 body-FRD frame.

    PX4 attitude quaternions rotate body FRD components into world NED, so the
    inverse rotation (the conjugate) is required for world NED → body FRD.
    """

    return rotate_vector_by_quaternion(
        world_vector,
        quaternion_conjugate(normalize_quaternion(body_to_world_q)),
    )


def quaternion_to_euler_deg(q: Iterable[float]) -> RelativeAttitude:
    w, x, y, z = normalize_quaternion(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = degrees(atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = degrees(copysign(pi / 2.0, sinp))
    else:
        pitch = degrees(asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = degrees(atan2(siny_cosp, cosy_cosp))
    return RelativeAttitude(roll, pitch, yaw)


def relative_attitude(left_q: Iterable[float], right_q: Iterable[float]) -> RelativeAttitude:
    left = normalize_quaternion(left_q)
    right = normalize_quaternion(right_q)
    relative_q = quaternion_multiply(right, quaternion_conjugate(left))
    return quaternion_to_euler_deg(relative_q)


def relative_rotation_angle_deg(reference_q: Iterable[float], current_q: Iterable[float]) -> float:
    """Return the shortest SO(3) rotation from reference to current in degrees."""

    reference = normalize_quaternion(reference_q)
    current = normalize_quaternion(current_q)
    dot = abs(sum(left * right for left, right in zip(reference, current)))
    return degrees(2.0 * acos(max(-1.0, min(1.0, dot))))
