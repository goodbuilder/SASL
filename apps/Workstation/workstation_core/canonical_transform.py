"""Canonical stereo transform used by every Measurement method.

Every method, whatever it derives R,t from, is stored as

    X_right = R_right_from_left_rdf * X_left + T_right_from_left_rdf_m

with both cameras in OpenCV optical RDF (Right, Down, Forward). This is the
OpenCV ``stereoCalibrate`` direction. The triangulation kernel instead wants
the right ray rotated *into* the left frame (``R_left_from_right``) and the
right optical center expressed in the left frame (``C_right_in_left``):

    R_left_from_right = R_right_from_left^T
    C_right_in_left   = -R_right_from_left^T * T_right_from_left

``T`` and ``C`` are different vectors and are never aliased. The conversions
here are the exact functions the live solver uses, so a workbook that stores
K, D, u, v, R, T reproduces the live arithmetic bit for bit.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence

from Workstation.workstation_core.triangulation import (
    canonical_transform_from_kernel_inputs,
    invert_left_to_right_camera_transform,
    pixhawk_canonical_transform,
    rotation_left_from_right_from_body_quaternions,
)

Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


def right_from_left_from_kernel_inputs(
    rotation_left_from_right: Sequence[Sequence[float]],
    right_origin_in_left_m: Iterable[float],
) -> tuple[Matrix3, Vector3]:
    """Kernel inputs (R_left_from_right, C_right_in_left) -> canonical (R, T)."""

    rotation, translation = canonical_transform_from_kernel_inputs(
        [list(row) for row in rotation_left_from_right],
        tuple(float(value) for value in right_origin_in_left_m),  # type: ignore[arg-type]
    )
    return tuple(tuple(row) for row in rotation), translation  # type: ignore[return-value]


def kernel_inputs_from_right_from_left(
    rotation_right_from_left: Sequence[Sequence[float]],
    translation_right_from_left_m: Iterable[float],
) -> tuple[Matrix3, Vector3]:
    """Canonical (R, T) -> kernel inputs (R_left_from_right, C_right_in_left)."""

    rotation, origin = invert_left_to_right_camera_transform(
        [list(row) for row in rotation_right_from_left],
        translation_right_from_left_m,
    )
    return tuple(tuple(row) for row in rotation), origin  # type: ignore[return-value]


def flatten_rotation(rotation: Sequence[Sequence[float]]) -> dict[str, float]:
    if len(rotation) != 3 or any(len(row) != 3 for row in rotation):
        raise ValueError("rotation must be a 3x3 matrix")
    return {
        f"R{row + 1}{column + 1}": float(rotation[row][column])
        for row in range(3)
        for column in range(3)
    }


def flatten_translation(translation: Iterable[float]) -> dict[str, float]:
    t = tuple(float(value) for value in translation)
    if len(t) != 3:
        raise ValueError("translation must contain 3 values")
    return {"T_R": t[0], "T_D": t[1], "T_F": t[2]}


def rotation_from_flat(values: dict[str, float]) -> Matrix3:
    return tuple(
        tuple(float(values[f"R{row + 1}{column + 1}"]) for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def translation_from_flat(values: dict[str, float]) -> Vector3:
    return (float(values["T_R"]), float(values["T_D"]), float(values["T_F"]))


__all__ = [
    "flatten_rotation",
    "flatten_translation",
    "kernel_inputs_from_right_from_left",
    "pixhawk_canonical_transform",
    "right_from_left_from_kernel_inputs",
    "rotation_from_flat",
    "rotation_left_from_right_from_body_quaternions",
    "translation_from_flat",
]
