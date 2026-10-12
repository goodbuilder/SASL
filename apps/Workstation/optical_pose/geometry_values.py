"""Existing optical pose matrix and evidence-value helpers.

R_NED_from_optical_RDF and centers in NED metres retain their existing
contracts; these functions do not read UI, clocks, profiles or devices.
"""

from __future__ import annotations

from Workstation.workstation_core.optical_pose import PoseDofMask

import hashlib
import math
from typing import Any, Iterable, Mapping, Sequence
from Workstation.workstation_core.run_evidence import canonical_json_bytes
from Workstation.workstation_core.optical_pose import CameraPoseNed, POSE_DOF_NAMES, pose_parameters_from_camera_pose


def _mat_transpose(matrix: Sequence[Sequence[float]]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(float(matrix[row][column]) for row in range(3)) for column in range(3))


def _matvec(
    matrix: Sequence[Sequence[float]],
    vector: Sequence[float],
) -> tuple[float, float, float]:
    return tuple(sum(float(matrix[row][index]) * float(vector[index]) for index in range(3)) for row in range(3))


def _payload_sha256(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _pose_mapping(pose: CameraPoseNed) -> dict[str, Any]:
    parameters = pose_parameters_from_camera_pose(pose)
    return {
        "rotation_ned_from_camera_rdf": [list(row) for row in pose.rotation_ned_from_camera_rdf],
        "center_ned_m": list(pose.center_ned_m),
        "virtual_frd_rpy_deg": [parameters.roll_deg, parameters.pitch_deg, parameters.yaw_deg],
        "coordinate_contract": "X_NED=R_NED<-optical_RDF*X_RDF+C_NED; center [north,east,down] metres",
    }


def _finite_vector(values: Iterable[Any], *, length: int, name: str) -> tuple[float, ...]:
    parsed = tuple(float(value) for value in values)
    if len(parsed) != length or not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{name} must contain {length} finite values")
    return parsed


def _mask_observability(mask: PoseDofMask) -> tuple[tuple[str, bool], ...]:
    return tuple((name, bool(getattr(mask, name))) for name in POSE_DOF_NAMES)


def _matmul(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(float(left[row][index]) * float(right[index][column]) for index in range(3)) for column in range(3))
        for row in range(3)
    )


def _subtract(left: Sequence[float], right: Sequence[float]) -> tuple[float, float, float]:
    return tuple(float(a) - float(b) for a, b in zip(left, right))  # type: ignore[return-value]


def _vector_value(value: Any, *, name: str) -> tuple[float, float, float]:
    if isinstance(value, Mapping):
        value = value.get("ned_m", value.get("vector_m", value))
        if isinstance(value, Mapping):
            value = (value.get("north_m"), value.get("east_m"), value.get("down_m"))
    parsed = _finite_vector(value or (), length=3, name=name)
    return parsed  # type: ignore[return-value]
