"""Reproduce Forward/Right/Down/Range from a Measurement workbook alone.

Inputs are exactly the standalone contract: K,D per side (03_Calibration or
91_Exact), the actual solver pixel u,v, and the canonical R,T of the method.
The arithmetic path is the live one (``triangulate_pixels_with_relative_pose_attempt``
after ``invert_left_to_right_camera_transform``), so on the same runtime the
values are bit-identical; across runtimes use ``math.isclose``.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from Workstation.workstation_core.canonical_transform import (
    kernel_inputs_from_right_from_left,
)
from Workstation.workstation_core.measurement_xlsx_read import read_workbook, table_rows, table_rows_with_parts
from Workstation.workstation_core.triangulation import (
    triangulate_pixels_with_relative_pose_attempt,
)

NA = "N/A"


@dataclass(frozen=True)
class ReproducedResult:
    pair_id: str
    method_id: str
    stored: tuple[float | None, float | None, float | None, float | None]
    reproduced: tuple[float | None, float | None, float | None, float | None]
    reason: str

    @property
    def bit_equal(self) -> bool:
        return all(
            (a is None and b is None) or (a is not None and b is not None and a == b)
            for a, b in zip(self.stored, self.reproduced)
        )

    def close(self, rel_tol: float = 1e-9, abs_tol: float = 1e-9) -> bool:
        return all(
            (a is None and b is None)
            or (a is not None and b is not None and math.isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol))
            for a, b in zip(self.stored, self.reproduced)
        )


def _number(value: object) -> float | None:
    if value is None or value == NA or isinstance(value, bool):
        return None
    try:
        parsed = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _intrinsics(matrix: Sequence[Sequence[float]], distortion: Sequence[float], width: int, height: int) -> SimpleNamespace:
    return SimpleNamespace(
        camera_matrix=[[float(value) for value in row] for row in matrix],
        distortion_coefficients=tuple(float(value) for value in distortion),
        width=int(width),
        height=int(height),
    )


def reproduce_pair_geometry(
    *,
    left_camera_matrix: Sequence[Sequence[float]],
    left_distortion: Sequence[float],
    right_camera_matrix: Sequence[Sequence[float]],
    right_distortion: Sequence[float],
    left_uv: tuple[float, float],
    right_uv: tuple[float, float],
    rotation_right_from_left: Sequence[Sequence[float]],
    translation_right_from_left_m: Sequence[float],
    image_size: tuple[int, int] = (0, 0),
) -> tuple[tuple[float, float, float, float] | None, str]:
    """Return ((Forward, Right, Down, Range), reason) or (None, reason)."""

    rotation_left_from_right, right_origin_in_left = kernel_inputs_from_right_from_left(
        rotation_right_from_left,
        translation_right_from_left_m,
    )
    attempt = triangulate_pixels_with_relative_pose_attempt(
        left_pixel=(float(left_uv[0]), float(left_uv[1])),
        right_pixel=(float(right_uv[0]), float(right_uv[1])),
        left_intrinsics=_intrinsics(left_camera_matrix, left_distortion, *image_size),
        right_intrinsics=_intrinsics(right_camera_matrix, right_distortion, *image_size),
        rotation_right_to_left=[list(row) for row in rotation_left_from_right],
        translation_right_in_left_m=right_origin_in_left,
    )
    if attempt.result is None:
        return None, attempt.reason
    forward, right, down = attempt.result.point_m
    return (forward, right, down, attempt.result.range_m), attempt.reason


def _calibration_from_sheet(rows: Sequence[Mapping[str, object]]) -> dict[str, dict[str, Any]]:
    calibration: dict[str, dict[str, Any]] = {}
    for row in rows:
        side = str(row.get("side") or "")
        if side in calibration or side not in {"left", "right"}:
            continue
        matrix = [[_number(row.get(f"K{r + 1}{c + 1}")) for c in range(3)] for r in range(3)]
        if any(value is None for line in matrix for value in line):
            continue
        count = int(_number(row.get("D_count")) or 0)
        distortion = [_number(row.get(f"D{index + 1}")) for index in range(count)]
        if any(value is None for value in distortion):
            continue
        calibration[side] = {
            "camera_matrix": matrix,
            "distortion_coefficients": distortion,
            "width": int(_number(row.get("image_width")) or 0),
            "height": int(_number(row.get("image_height")) or 0),
        }
    return calibration


def reproduce_workbook(path: Path | str) -> list[ReproducedResult]:
    """Reproduce every attempted (pair, method) row of ``90_Flat`` from decimal cells."""

    sheets = read_workbook(path)
    calibration = _calibration_from_sheet(table_rows(sheets["03_Calibration"]))
    results: list[ReproducedResult] = []
    for row in table_rows_with_parts(sheets, "90_Flat"):
        if str(row.get("Attempted")) != "YES":
            continue
        stored = (
            _number(row.get("Forward")),
            _number(row.get("Right")),
            _number(row.get("Down")),
            _number(row.get("Range")),
        )
        rotation = [[_number(row.get(f"R{r + 1}{c + 1}")) for c in range(3)] for r in range(3)]
        translation = [_number(row.get(name)) for name in ("T_R", "T_D", "T_F")]
        left_uv = (_number(row.get("L_u")), _number(row.get("L_v")))
        right_uv = (_number(row.get("R_u")), _number(row.get("R_v")))
        missing = (
            any(value is None for line in rotation for value in line)
            or any(value is None for value in translation)
            or any(value is None for value in (*left_uv, *right_uv))
            or "left" not in calibration
            or "right" not in calibration
        )
        if missing:
            results.append(
                ReproducedResult(
                    pair_id=str(row.get("pair_id")),
                    method_id=str(row.get("method_id")),
                    stored=stored,
                    reproduced=(None, None, None, None),
                    reason="missing_input",
                )
            )
            continue
        reproduced, reason = reproduce_pair_geometry(
            left_camera_matrix=calibration["left"]["camera_matrix"],
            left_distortion=calibration["left"]["distortion_coefficients"],
            right_camera_matrix=calibration["right"]["camera_matrix"],
            right_distortion=calibration["right"]["distortion_coefficients"],
            left_uv=left_uv,  # type: ignore[arg-type]
            right_uv=right_uv,  # type: ignore[arg-type]
            rotation_right_from_left=rotation,  # type: ignore[arg-type]
            translation_right_from_left_m=translation,  # type: ignore[arg-type]
            image_size=(calibration["left"]["width"], calibration["left"]["height"]),
        )
        results.append(
            ReproducedResult(
                pair_id=str(row.get("pair_id")),
                method_id=str(row.get("method_id")),
                stored=stored,
                reproduced=(None, None, None, None) if reproduced is None else reproduced,
                reason=reason,
            )
        )
    return results


def reproduce_workbook_exact(path: Path | str) -> list[ReproducedResult]:
    """Reproduce from ``91_Exact`` hex values; bit-exact on the same runtime."""

    sheets = read_workbook(path)
    exact: dict[tuple[str, str], dict[str, float]] = {}
    for row in table_rows_with_parts(sheets, "91_Exact"):
        key = (str(row.get("pair_id") or ""), str(row.get("method_id") or ""))
        text = row.get("value_hex")
        if not isinstance(text, str):
            continue
        try:
            exact.setdefault(key, {})[str(row.get("field"))] = float.fromhex(text)
        except ValueError:
            continue
    results: list[ReproducedResult] = []
    for (pair_id, method_id), values in exact.items():
        if method_id == "calibration":
            continue
        try:
            left_k = [[values[f"left.K{r + 1}{c + 1}"] for c in range(3)] for r in range(3)]
            right_k = [[values[f"right.K{r + 1}{c + 1}"] for c in range(3)] for r in range(3)]
            rotation = [[values[f"R{r + 1}{c + 1}"] for c in range(3)] for r in range(3)]
            translation = [values["T_R"], values["T_D"], values["T_F"]]
            left_uv = (values["left.u"], values["left.v"])
            right_uv = (values["right.u"], values["right.v"])
        except KeyError:
            continue
        left_d = [values[name] for name in sorted((key for key in values if key.startswith("left.D")), key=lambda item: int(item[6:]))]
        right_d = [values[name] for name in sorted((key for key in values if key.startswith("right.D")), key=lambda item: int(item[7:]))]
        stored = tuple(values.get(name) for name in ("Forward", "Right", "Down", "Range"))
        reproduced, reason = reproduce_pair_geometry(
            left_camera_matrix=left_k,
            left_distortion=left_d,
            right_camera_matrix=right_k,
            right_distortion=right_d,
            left_uv=left_uv,
            right_uv=right_uv,
            rotation_right_from_left=rotation,
            translation_right_from_left_m=translation,
        )
        results.append(
            ReproducedResult(
                pair_id=pair_id,
                method_id=method_id,
                stored=stored,  # type: ignore[arg-type]
                reproduced=(None, None, None, None) if reproduced is None else reproduced,
                reason=reason,
            )
        )
    return results


def reproduce_target_coordinates_exact(path: Path | str) -> list[dict[str, Any]]:
    """Recompute target geolocation from exact K,D,R,T,u,v and world anchor.

    Old workbooks without target fields yield no entries. Existing FRD replay
    APIs retain their original return contract.
    """
    import json
    from Workstation.workstation_core.target_coordinates import TARGET_COORDINATE_FIELDS, target_wgs84_from_attempt

    sheets = read_workbook(path)
    groups: dict[tuple[str, str], dict[str, float]] = {}
    for row in table_rows_with_parts(sheets, "91_Exact"):
        try:
            value = float.fromhex(str(row.get("value_hex")))
        except ValueError:
            continue
        groups.setdefault((str(row.get("pair_id") or ""), str(row.get("method_id") or "")), {})[str(row.get("field"))] = value
    results = []
    for (pair_id, method_id), v in groups.items():
        if not any(name in v for name in TARGET_COORDINATE_FIELDS):
            continue
        stored = {name: v.get(name) for name in TARGET_COORDINATE_FIELDS}
        try:
            origin = [v[f"target.origin_wgs84.{i}"] for i in range(3)]
            cameras = {}
            for side in ("left", "right"):
                matrix = [[v[f"{side}.K{r+1}{c+1}"] for c in range(3)] for r in range(3)]
                names = sorted((key for key in v if key.startswith(f"{side}.D")), key=lambda name: int(name.split('.D')[1]))
                cameras[side] = _intrinsics(matrix, [v[name] for name in names], 0, 0)
            rotation, center = kernel_inputs_from_right_from_left(
                [[v[f"R{r+1}{c+1}"] for c in range(3)] for r in range(3)],
                [v[axis] for axis in ("T_R", "T_D", "T_F")],
            )
            geometry = triangulate_pixels_with_relative_pose_attempt(
                left_pixel=(v["left.u"], v["left.v"]), right_pixel=(v["right.u"], v["right.v"]),
                left_intrinsics=cameras["left"], right_intrinsics=cameras["right"],
                rotation_right_to_left=rotation, translation_right_in_left_m=center,
            )
            attempt = {"left_gps": {"latitude_deg": origin[0], "longitude_deg": origin[1],
                       "altitude_ellipsoid_m": origin[2], "fix_type": 3}}
            if "target.body_to_ned_quaternion_wxyz.0" in v:
                attempt.update(triangulation_source="pixhawk", geometry_left_quaternion_wxyz=[v[f"target.body_to_ned_quaternion_wxyz.{i}"] for i in range(4)])
            else:
                attempt.update(triangulation_source="checkerboard", measurement_pose_evidence_json=json.dumps({
                    "target_origin_model": "P1_GNSS_as_left_optical_center",
                    "left_pose": {"rotation_ned_from_camera_rdf": [[v[f"target.rotation.{r}.{c}"] for c in range(3)] for r in range(3)]},
                }))
            result = target_wgs84_from_attempt(attempt, None if geometry.result is None else geometry.result.source_point_left_rdf_m)
            reason = result["target_coordinate_evidence"]["reason"]
        except (KeyError, TypeError, ValueError) as exc:
            result, reason = {}, f"missing_or_invalid_input: {exc}"
        reproduced = {name: result.get(name) for name in TARGET_COORDINATE_FIELDS}
        results.append({"pair_id": pair_id, "method_id": method_id, "stored": stored,
                        "reproduced": reproduced, "reason": reason})
    return results


__all__ = [
    "reproduce_target_coordinates_exact",
    "ReproducedResult",
    "reproduce_pair_geometry",
    "reproduce_workbook",
    "reproduce_workbook_exact",
]
