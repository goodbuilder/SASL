#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
from scipy.spatial.transform import Rotation


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
TRANSFORM_CONTRACT = (
    "T_source_to_stereo_rdf: truth_rdf = R_source_to_stereo_rdf * "
    "(truth_source - origin_source) + translation_rdf; "
    "direct source-frame axis aliasing is forbidden"
)
FORBIDDEN_NED_TO_RDF_ALIAS_MATRIX = np.array(
    [
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=float,
)


@dataclass(frozen=True)
class ValidatedTransform:
    source_frame: str
    source_axis_order: tuple[str, str, str]
    source_fields: tuple[str, str, str]
    origin_source_m: np.ndarray
    translation_rdf_m: np.ndarray
    rotation: Rotation

    def summary(self, *, mode: str) -> dict[str, object]:
        return {
            "mode": mode,
            "source_frame": self.source_frame,
            "source_axis_order": ",".join(self.source_axis_order),
            "source_fields": ",".join(self.source_fields),
            "rdf_axis_contract": RDF_AXIS_CONTRACT,
            "truth_rdf_transform_contract": TRANSFORM_CONTRACT,
            "direct_axis_alias_allowed": False,
            "origin_source_m": self.origin_source_m.tolist(),
            "translation_rdf_m": self.translation_rdf_m.tolist(),
            "rotation_matrix_source_to_stereo_rdf": self.rotation.as_matrix().tolist(),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert calibration/evaluation truth from an explicit source frame into "
            "stereo-center RDF truth fields for the RTK stereo calibration pipeline."
        )
    )
    parser.add_argument("--input-csv", type=Path)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--transform-json", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--summary-txt", type=Path)
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Validate the transform JSON and write summaries without reading or writing a CSV.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Fail if any row lacks finite source truth fields.",
    )
    return parser.parse_args()


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def vector_from_row(row: Mapping[str, str], fields: Sequence[str]) -> np.ndarray | None:
    values = [finite_float(row.get(field)) for field in fields]
    if all(value is not None for value in values):
        return np.array([float(value) for value in values if value is not None], dtype=float)
    return None


def resolve_source_fields(row: Mapping[str, str], configured_fields: Sequence[str]) -> tuple[tuple[str, str, str], np.ndarray | None]:
    fields = tuple(str(field) for field in configured_fields)
    if len(fields) != 3:
        raise ValueError("source_fields must contain exactly three field names")
    return fields, vector_from_row(row, fields)


def load_rotation(payload: Mapping[str, object]) -> Rotation:
    matrix = payload.get("rotation_matrix_source_to_stereo_rdf")
    euler = payload.get("rotation_euler_zyx_deg_source_to_stereo_rdf")
    if matrix is not None and euler is not None:
        raise ValueError(
            "provide exactly one rotation representation: "
            "rotation_matrix_source_to_stereo_rdf or rotation_euler_zyx_deg_source_to_stereo_rdf"
        )
    if matrix is not None:
        arr = np.asarray(matrix, dtype=float)
        if arr.shape != (3, 3):
            raise ValueError("rotation_matrix_source_to_stereo_rdf must be 3x3")
        return Rotation.from_matrix(arr)
    if euler is not None:
        values = np.asarray(euler, dtype=float)
        if values.shape != (3,):
            raise ValueError("rotation_euler_zyx_deg_source_to_stereo_rdf must contain three values")
        return Rotation.from_euler("zyx", values, degrees=True)
    raise ValueError("transform JSON requires rotation_matrix_source_to_stereo_rdf or rotation_euler_zyx_deg_source_to_stereo_rdf")


def load_vector(payload: Mapping[str, object], key: str, default: Sequence[float]) -> np.ndarray:
    values = np.asarray(payload.get(key, default), dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError(f"{key} must contain three finite values")
    return values


def source_axis_order(payload: Mapping[str, object]) -> tuple[str, str, str] | None:
    raw = payload.get("source_axis_order")
    if raw is None:
        return None
    if not isinstance(raw, list) or len(raw) != 3:
        raise ValueError("source_axis_order must be a list of three axis names")
    return tuple(str(value).strip().lower() for value in raw)


def require_source_contract(payload: Mapping[str, object]) -> tuple[str, tuple[str, str, str]]:
    source_frame = str(payload.get("source_frame", "")).strip()
    if not source_frame:
        raise ValueError("source_frame is required")
    axes = source_axis_order(payload)
    if axes is None:
        raise ValueError("source_axis_order is required")
    if any(not axis for axis in axes):
        raise ValueError("source_axis_order entries must be non-empty")
    return source_frame, axes


def reject_forbidden_direct_alias(payload: Mapping[str, object], rotation: Rotation) -> None:
    axes = source_axis_order(payload)
    source_frame = str(payload.get("source_frame", "")).strip().lower()
    if axes != ("north", "east", "down") and "ned" not in source_frame:
        return
    matrix = rotation.as_matrix()
    if np.allclose(matrix, FORBIDDEN_NED_TO_RDF_ALIAS_MATRIX, atol=1.0e-9, rtol=0.0):
        raise ValueError(
            "forbidden direct NED-to-RDF axis alias transform: "
            "rdf_right=east, rdf_down=down, rdf_forward=north"
        )


def transform_point(source_point: np.ndarray, *, rotation: Rotation, origin_source_m: np.ndarray, translation_rdf_m: np.ndarray) -> np.ndarray:
    return rotation.apply(source_point - origin_source_m) + translation_rdf_m


def range_m(point: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in point))


def validated_transform(transform: Mapping[str, object]) -> ValidatedTransform:
    source_frame, axes = require_source_contract(transform)
    rotation = load_rotation(transform)
    reject_forbidden_direct_alias(transform, rotation)
    origin_source_m = load_vector(transform, "origin_source_m", (0.0, 0.0, 0.0))
    translation_rdf_m = load_vector(transform, "translation_rdf_m", (0.0, 0.0, 0.0))
    configured_fields = transform.get("source_fields")
    if configured_fields is None:
        raise ValueError("source_fields is required")
    if not isinstance(configured_fields, list) or len(configured_fields) != 3:
        raise ValueError("source_fields must be a list of three field names")
    if any(not str(field).strip() for field in configured_fields):
        raise ValueError("source_fields entries must be non-empty")
    return ValidatedTransform(
        source_frame=source_frame,
        source_axis_order=axes,
        source_fields=tuple(str(field).strip() for field in configured_fields),
        origin_source_m=origin_source_m,
        translation_rdf_m=translation_rdf_m,
        rotation=rotation,
    )


def validate_transform(transform: Mapping[str, object]) -> dict[str, object]:
    validated = validated_transform(transform)
    summary = validated.summary(mode="truth_source_to_stereo_rdf_transform_validation")
    summary.update(
        {
            "valid": True,
        }
    )
    return summary


def convert_rows(
    rows: Sequence[Mapping[str, str]],
    transform: Mapping[str, object],
    *,
    strict: bool = False,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    validation = validated_transform(transform)

    converted: list[dict[str, object]] = []
    converted_count = 0
    skipped_count = 0
    source_field_names: set[str] = set()
    for row_index, row in enumerate(rows):
        out = dict(row)
        fields, source_point = resolve_source_fields(row, validation.source_fields)
        if source_point is None:
            skipped_count += 1
            if strict:
                raise ValueError(f"row {row_index} lacks finite source truth fields")
            out.setdefault("truth_rdf_conversion_status", "SKIP_MISSING_SOURCE")
            converted.append(out)
            continue
        source_field_names.update(fields or ())
        truth_rdf = transform_point(
            source_point,
            rotation=validation.rotation,
            origin_source_m=validation.origin_source_m,
            translation_rdf_m=validation.translation_rdf_m,
        )
        out.update(
            {
                "truth_right_m": f"{truth_rdf[0]:.9f}",
                "truth_down_m": f"{truth_rdf[1]:.9f}",
                "truth_forward_m": f"{truth_rdf[2]:.9f}",
                "truth_range_m": f"{range_m(truth_rdf):.9f}",
                "truth_source_frame": validation.source_frame,
                "truth_source_axis_order": ",".join(validation.source_axis_order),
                "truth_source_fields": ",".join(fields or ()),
                "truth_rdf_axis_contract": RDF_AXIS_CONTRACT,
                "truth_rdf_transform_contract": TRANSFORM_CONTRACT,
                "truth_source_origin_m": ",".join(f"{value:.9f}" for value in validation.origin_source_m),
                "truth_source_to_stereo_rdf_translation_m": ",".join(
                    f"{value:.9f}" for value in validation.translation_rdf_m
                ),
                "truth_source_to_stereo_rdf_rotation_matrix": json.dumps(
                    validation.rotation.as_matrix().tolist(),
                    separators=(",", ":"),
                ),
                "truth_rdf_conversion_status": "CONVERTED",
            }
        )
        converted_count += 1
        converted.append(out)

    summary = {
        "mode": "truth_source_to_stereo_rdf_conversion",
        "row_count": len(rows),
        "converted_row_count": converted_count,
        "skipped_row_count": skipped_count,
        "source_frame": validation.source_frame,
        "source_axis_order": ",".join(validation.source_axis_order),
        "source_fields_seen": ",".join(sorted(source_field_names)),
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "truth_rdf_transform_contract": TRANSFORM_CONTRACT,
        "direct_axis_alias_allowed": False,
        "origin_source_m": validation.origin_source_m.tolist(),
        "translation_rdf_m": validation.translation_rdf_m.tolist(),
        "rotation_matrix_source_to_stereo_rdf": validation.rotation.as_matrix().tolist(),
    }
    return converted, summary


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary_txt(path: Path, summary: Mapping[str, object]) -> None:
    lines = ["Truth source to stereo RDF conversion", ""]
    for key, value in summary.items():
        if isinstance(value, list):
            value_text = json.dumps(value, ensure_ascii=True)
        else:
            value_text = str(value)
        lines.append(f"{key}: {value_text}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    transform = json.loads(args.transform_json.read_text(encoding="utf-8"))
    if args.validate_only:
        summary = validate_transform(transform)
        summary_json = args.summary_json or args.transform_json.with_suffix(".validation.json")
        summary_txt = args.summary_txt or args.transform_json.with_suffix(".validation.txt")
        summary_json.parent.mkdir(parents=True, exist_ok=True)
        summary_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        write_summary_txt(summary_txt, summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0
    if args.input_csv is None or args.output_csv is None:
        raise SystemExit("--input-csv and --output-csv are required unless --validate-only is set")
    with args.input_csv.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    converted, summary = convert_rows(rows, transform, strict=args.strict)
    write_csv(args.output_csv, converted)

    summary_json = args.summary_json or args.output_csv.with_suffix(".summary.json")
    summary_txt = args.summary_txt or args.output_csv.with_suffix(".summary.txt")
    summary_json.parent.mkdir(parents=True, exist_ok=True)
    summary_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_summary_txt(summary_txt, summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["converted_row_count"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
