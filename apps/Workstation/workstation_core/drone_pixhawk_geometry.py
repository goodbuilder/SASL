"""Relative Drone/Pixhawk geometry used by the Workstation Pixhawk view.

The public vectors point from each named reference to the Drone:
``midpoint -> Drone``, ``P1 -> Drone``, and ``P2 -> Drone``.  Vector values
retain Forward/Right/Down and metre units in the frame named by the caller.
The two triangle angles are deliberately different: they are the horizontal
internal angles at P1 and P2, so altitude/Down and vehicle yaw are ignored.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees, hypot, isfinite, sqrt
from typing import Iterable


Vector3 = tuple[float, float, float]
HORIZONTAL_VECTOR_EPSILON_M = 1.0e-9


@dataclass(frozen=True)
class HorizontalTriangleAngles:
    """Horizontal internal angles for the triangle Dr/P1/P2."""

    at_p1_deg: float | None
    at_p2_deg: float | None


def _vector3(values: Iterable[float], *, name: str) -> Vector3:
    parsed = tuple(float(value) for value in values)
    if len(parsed) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    if not all(isfinite(value) for value in parsed):
        raise ValueError(f"{name} must contain finite metres")
    return parsed[0], parsed[1], parsed[2]


def subtract_vectors(left: Iterable[float], right: Iterable[float]) -> Vector3:
    """Return the finite 3D vector ``left - right``."""

    left_vector = _vector3(left, name="left")
    right_vector = _vector3(right, name="right")
    return tuple(
        left_vector[index] - right_vector[index]
        for index in range(3)
    )  # type: ignore[return-value]


def negate_vector(vector: Iterable[float]) -> Vector3:
    """Return the finite 3D vector with every component negated."""

    parsed = _vector3(vector, name="vector")
    return -parsed[0], -parsed[1], -parsed[2]


def vector_range_m(vector: Iterable[float]) -> float:
    """Return the Euclidean range of a finite metre vector."""

    forward_m, right_m, down_m = _vector3(vector, name="vector")
    return sqrt(forward_m * forward_m + right_m * right_m + down_m * down_m)


def horizontal_angle_deg(
    first_vector: Iterable[float],
    second_vector: Iterable[float],
) -> float | None:
    """Return the unsigned horizontal angle between two FRD/NED vectors.

    Only the first two components are used by contract.  The input order is
    preserved for readability, while the absolute 2D cross product makes the
    result an internal angle in ``[0, 180]`` rather than a signed bearing.
    ``None`` means one horizontal ray has no measurable length.
    """

    first = _vector3(first_vector, name="first_vector")
    second = _vector3(second_vector, name="second_vector")
    first_length = hypot(first[0], first[1])
    second_length = hypot(second[0], second[1])
    if (
        first_length <= HORIZONTAL_VECTOR_EPSILON_M
        or second_length <= HORIZONTAL_VECTOR_EPSILON_M
    ):
        return None
    dot = first[0] * second[0] + first[1] * second[1]
    cross_abs = abs(first[0] * second[1] - first[1] * second[0])
    return degrees(atan2(cross_abs, dot))


def calculate_horizontal_triangle_angles(
    p1_position_ned_m: Iterable[float],
    p2_position_ned_m: Iterable[float],
    drone_position_ned_m: Iterable[float],
) -> HorizontalTriangleAngles:
    """Calculate the two requested horizontal Dr/P1/P2 internal angles.

    ``p1_position_ned_m``, ``p2_position_ned_m``, and
    ``drone_position_ned_m`` share one local NED origin.  North/East define
    the horizontal plane; Down is validated for contract completeness but is
    intentionally ignored by the angle calculation.
    """

    p1 = _vector3(p1_position_ned_m, name="p1_position_ned_m")
    p2 = _vector3(p2_position_ned_m, name="p2_position_ned_m")
    drone = _vector3(drone_position_ned_m, name="drone_position_ned_m")
    p1_to_p2 = subtract_vectors(p2, p1)
    p1_to_drone = subtract_vectors(drone, p1)
    p2_to_p1 = negate_vector(p1_to_p2)
    p2_to_drone = subtract_vectors(drone, p2)
    return HorizontalTriangleAngles(
        at_p1_deg=horizontal_angle_deg(p1_to_drone, p1_to_p2),
        at_p2_deg=horizontal_angle_deg(p2_to_drone, p2_to_p1),
    )


__all__ = [
    "HORIZONTAL_VECTOR_EPSILON_M",
    "HorizontalTriangleAngles",
    "calculate_horizontal_triangle_angles",
    "horizontal_angle_deg",
    "negate_vector",
    "subtract_vectors",
    "vector_range_m",
]
