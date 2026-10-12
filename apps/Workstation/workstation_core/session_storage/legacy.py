"""Session legacy responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable
from . import canonical as _storage_canonical
from . import generation as _storage_generation
from . import manifest as _storage_manifest
from . import paths as _storage_paths
from . import projection as _storage_projection
from . import schema as _storage_schema
from . import writers as _storage_writers
from .models import SavedSessionFiles
from .spool import SpoolRows


def _save_session_files_legacy(
    output_dir: Path,
    *,
    metadata: dict[str, object],
    pair_rows: Iterable[dict[str, object]],
    camera_rows: dict[str, Iterable[dict[str, object]]] | None = None,
    px4_rows: dict[str, Iterable[dict[str, object]]] | None = None,
    gnss_base_rows: Iterable[dict[str, object]] | None = None,
    extrinsics_comparison_rows: Iterable[dict[str, object]] | None = None,
) -> SavedSessionFiles:
    output_dir.mkdir(parents=True, exist_ok=True)
    session_id = Path(str(metadata.get("session_id") or output_dir.name)).name
    rows = _storage_projection._project_pair_typed_fields(_storage_projection._normalized_pair_rows(pair_rows))
    camera_rows = camera_rows or {}
    px4_rows = px4_rows or {}
    gnss_base_full_rows = _storage_projection._normalized_generic_rows(gnss_base_rows or [])
    extrinsics_comparison_full_rows = _storage_projection._normalized_generic_rows(extrinsics_comparison_rows or [])
    camera_full_rows = {
        side: _storage_projection._normalized_generic_rows(camera_rows.get(side, []))
        for side in ("left", "right")
    }
    px4_sides = ("left", "right", *(("drone",) if "drone" in px4_rows else ()))
    px4_full_rows = {
        side: _storage_projection._normalized_generic_rows(px4_rows.get(side, []))
        for side in px4_sides
    }
    compact_distance_rows = _storage_projection.compact_distance_measurement_rows(rows, metadata)
    compact_camera_rows = {
        side: _storage_projection.compact_camera_timeline_rows(camera_full_rows[side], metadata)
        for side in ("left", "right")
    }
    compact_px4_rows = {
        side: _storage_projection.compact_px4_timeline_rows(px4_full_rows[side], metadata)
        for side in ("left", "right")
    }
    compact_rtk_rows = _storage_writers._compact_rtk_rows(px4_full_rows, metadata)
    compact_gnss_base_row_list = _storage_projection.compact_gnss_base_rows(gnss_base_full_rows, metadata)
    static_output = output_dir / f"{session_id}_session_static.json"
    distance_csv_output = output_dir / f"{session_id}_distance_measurements.csv"
    distance_parquet_output = output_dir / f"{session_id}_distance_measurements.parquet"
    xlsx_output = output_dir / f"{session_id}.xlsx"
    camera_left_output = output_dir / f"{session_id}_camera_left.csv"
    camera_left_parquet_output = output_dir / f"{session_id}_camera_left.parquet"
    camera_right_output = output_dir / f"{session_id}_camera_right.csv"
    camera_right_parquet_output = output_dir / f"{session_id}_camera_right.parquet"
    px4_left_output = output_dir / f"{session_id}_px4_left.csv"
    px4_left_parquet_output = output_dir / f"{session_id}_px4_left.parquet"
    px4_right_output = output_dir / f"{session_id}_px4_right.csv"
    px4_right_parquet_output = output_dir / f"{session_id}_px4_right.parquet"
    px4_left_xlsx_output = output_dir / f"{session_id}_px4_left.xlsx"
    px4_right_xlsx_output = output_dir / f"{session_id}_px4_right.xlsx"
    session_rtk_xlsx_output = output_dir / f"{session_id}_session_rtk.xlsx"
    gnss_base_output = output_dir / f"{session_id}_gnss_base.csv"
    gnss_base_parquet_output = output_dir / f"{session_id}_gnss_base.parquet"
    extrinsics_comparison_output = output_dir / f"{session_id}_extrinsics_comparison.csv"
    extrinsics_comparison_parquet_output = output_dir / f"{session_id}_extrinsics_comparison.parquet"
    raw_pair_output = output_dir / f"{session_id}_raw_pair_rows.jsonl"
    raw_camera_left_output = output_dir / f"{session_id}_raw_camera_left_rows.jsonl"
    raw_camera_right_output = output_dir / f"{session_id}_raw_camera_right_rows.jsonl"
    raw_px4_left_output = output_dir / f"{session_id}_raw_px4_left_rows.jsonl"
    raw_px4_right_output = output_dir / f"{session_id}_raw_px4_right_rows.jsonl"
    raw_px4_drone_output = output_dir / f"{session_id}_raw_px4_drone_rows.jsonl"
    raw_gnss_base_output = output_dir / f"{session_id}_raw_gnss_base_rows.jsonl"
    raw_extrinsics_comparison_output = output_dir / f"{session_id}_raw_extrinsics_comparison_rows.jsonl"
    manifest_output = output_dir / "session_manifest.json"
    resource_files = {
        "session_static": static_output.name,
        "distance_measurements_csv": distance_csv_output.name,
        "distance_measurements_parquet": distance_parquet_output.name,
        "camera_left_csv": camera_left_output.name,
        "camera_left_parquet": camera_left_parquet_output.name,
        "camera_right_csv": camera_right_output.name,
        "camera_right_parquet": camera_right_parquet_output.name,
        "px4_left_csv": px4_left_output.name,
        "px4_left_parquet": px4_left_parquet_output.name,
        "px4_right_csv": px4_right_output.name,
        "px4_right_parquet": px4_right_parquet_output.name,
        "px4_left_xlsx": px4_left_xlsx_output.name,
        "px4_right_xlsx": px4_right_xlsx_output.name,
        "session_rtk_xlsx": session_rtk_xlsx_output.name,
        "gnss_base_csv": gnss_base_output.name,
        "gnss_base_parquet": gnss_base_parquet_output.name,
        "extrinsics_comparison_csv": extrinsics_comparison_output.name,
        "extrinsics_comparison_parquet": extrinsics_comparison_parquet_output.name,
        "raw_pair_rows_jsonl": raw_pair_output.name,
        "raw_camera_left_rows_jsonl": raw_camera_left_output.name,
        "raw_camera_right_rows_jsonl": raw_camera_right_output.name,
        "raw_px4_left_rows_jsonl": raw_px4_left_output.name,
        "raw_px4_right_rows_jsonl": raw_px4_right_output.name,
        "raw_gnss_base_rows_jsonl": raw_gnss_base_output.name,
        "raw_extrinsics_comparison_rows_jsonl": raw_extrinsics_comparison_output.name,
        "workbook": xlsx_output.name,
    }
    if "drone" in px4_full_rows:
        resource_files["raw_px4_drone_rows_jsonl"] = raw_px4_drone_output.name
    if not _storage_schema.WRITE_PARQUET_ARTIFACTS:
        for key in (
            "distance_measurements_parquet",
            "camera_left_parquet",
            "camera_right_parquet",
            "px4_left_parquet",
            "px4_right_parquet",
            "gnss_base_parquet",
            "extrinsics_comparison_parquet",
        ):
            resource_files.pop(key, None)
    _storage_generation._cleanup_finalize_orphans(output_dir)
    generation_id = _storage_paths.session_id_now()
    generations_root = output_dir / ".generations"
    generations_root.mkdir(parents=True, exist_ok=True)
    staging_dir = generations_root / f"{generation_id}.preparing"
    staging_dir.mkdir()
    csv_names_and_rows = (
        (distance_csv_output.name, compact_distance_rows, _storage_schema.DISTANCE_MEASUREMENT_COLUMNS),
        (camera_left_output.name, compact_camera_rows["left"], _storage_schema.CAMERA_TIMELINE_COMPACT_COLUMNS),
        (camera_right_output.name, compact_camera_rows["right"], _storage_schema.CAMERA_TIMELINE_COMPACT_COLUMNS),
        (px4_left_output.name, compact_px4_rows["left"], _storage_schema.PX4_TIMELINE_COMPACT_COLUMNS),
        (px4_right_output.name, compact_px4_rows["right"], _storage_schema.PX4_TIMELINE_COMPACT_COLUMNS),
        (gnss_base_output.name, compact_gnss_base_row_list, _storage_schema.GNSS_BASE_COMPACT_COLUMNS),
        (
            extrinsics_comparison_output.name,
            extrinsics_comparison_full_rows,
            _storage_schema.EXTRINSICS_COMPARISON_COMPACT_COLUMNS,
        ),
    )
    parquet_names_and_rows = (
        (
            (distance_parquet_output.name, compact_distance_rows, _storage_schema.DISTANCE_MEASUREMENT_COLUMNS),
            (camera_left_parquet_output.name, compact_camera_rows["left"], _storage_schema.CAMERA_TIMELINE_COMPACT_COLUMNS),
            (camera_right_parquet_output.name, compact_camera_rows["right"], _storage_schema.CAMERA_TIMELINE_COMPACT_COLUMNS),
            (px4_left_parquet_output.name, compact_px4_rows["left"], _storage_schema.PX4_TIMELINE_COMPACT_COLUMNS),
            (px4_right_parquet_output.name, compact_px4_rows["right"], _storage_schema.PX4_TIMELINE_COMPACT_COLUMNS),
            (gnss_base_parquet_output.name, compact_gnss_base_row_list, _storage_schema.GNSS_BASE_COMPACT_COLUMNS),
            (
                extrinsics_comparison_parquet_output.name,
                extrinsics_comparison_full_rows,
                _storage_schema.EXTRINSICS_COMPARISON_COMPACT_COLUMNS,
            ),
        )
        if _storage_schema.WRITE_PARQUET_ARTIFACTS
        else ()
    )
    raw_jsonl_names_and_rows = [
        (
            raw_pair_output.name,
            rows,
            "raw_pair_rows_jsonl",
            "_normalized_pair_rows",
        ),
        (
            raw_camera_left_output.name,
            camera_full_rows["left"],
            "raw_camera_left_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_camera_right_output.name,
            camera_full_rows["right"],
            "raw_camera_right_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_px4_left_output.name,
            px4_full_rows["left"],
            "raw_px4_left_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_px4_right_output.name,
            px4_full_rows["right"],
            "raw_px4_right_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_gnss_base_output.name,
            gnss_base_full_rows,
            "raw_gnss_base_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_extrinsics_comparison_output.name,
            extrinsics_comparison_full_rows,
            "raw_extrinsics_comparison_rows_jsonl",
            "_normalized_generic_rows",
        ),
    ]
    if "drone" in px4_full_rows:
        raw_jsonl_names_and_rows.append(
            (
                raw_px4_drone_output.name,
                px4_full_rows["drone"],
                "raw_px4_drone_rows_jsonl",
                "_normalized_generic_rows",
            )
        )
    manifest: dict[str, object] = {
        "session_id": session_id,
        "status": "preparing",
        "resource_files_authoritative": True,
        "resource_files": resource_files,
        "time_axis": "recording_time_ns",
        "artifacts": {},
        "artifact_details": {},
        "authoritative_raw_artifacts_verified": False,
        "artifact_roles": {
            static_output.name: "session_static_metadata",
            distance_csv_output.name: "distance_measurements_csv",
            distance_parquet_output.name: "distance_measurements_parquet",
            camera_left_output.name: "camera_timeline_csv",
            camera_left_parquet_output.name: "camera_timeline_parquet",
            camera_right_output.name: "camera_timeline_csv",
            camera_right_parquet_output.name: "camera_timeline_parquet",
            px4_left_output.name: "px4_timeline_csv",
            px4_left_parquet_output.name: "px4_timeline_parquet",
            px4_right_output.name: "px4_timeline_csv",
            px4_right_parquet_output.name: "px4_timeline_parquet",
            px4_left_xlsx_output.name: "px4_left_workbook",
            px4_right_xlsx_output.name: "px4_right_workbook",
            session_rtk_xlsx_output.name: "session_rtk_workbook",
            gnss_base_output.name: "gnss_base_timeline_csv",
            gnss_base_parquet_output.name: "gnss_base_timeline_parquet",
            extrinsics_comparison_output.name: "extrinsics_comparison_csv",
            extrinsics_comparison_parquet_output.name: "extrinsics_comparison_parquet",
            raw_pair_output.name: "raw_pair_rows_jsonl",
            raw_camera_left_output.name: "raw_camera_left_rows_jsonl",
            raw_camera_right_output.name: "raw_camera_right_rows_jsonl",
            raw_px4_left_output.name: "raw_px4_left_rows_jsonl",
            raw_px4_right_output.name: "raw_px4_right_rows_jsonl",
            raw_gnss_base_output.name: "raw_gnss_base_rows_jsonl",
            raw_extrinsics_comparison_output.name: "raw_extrinsics_comparison_rows_jsonl",
            xlsx_output.name: "integrated_review_workbook",
        },
    }
    if "drone" in px4_full_rows:
        manifest["artifact_roles"][raw_px4_drone_output.name] = "raw_px4_drone_rows_jsonl"  # type: ignore[index]
    if not _storage_schema.WRITE_PARQUET_ARTIFACTS:
        artifact_roles = manifest["artifact_roles"]  # type: ignore[assignment]
        for name in (
            distance_parquet_output.name,
            camera_left_parquet_output.name,
            camera_right_parquet_output.name,
            px4_left_parquet_output.name,
            px4_right_parquet_output.name,
            gnss_base_parquet_output.name,
            extrinsics_comparison_parquet_output.name,
        ):
            artifact_roles.pop(name, None)  # type: ignore[union-attr]
    authoritative_raw_artifacts_verified = False
    try:
        _storage_writers._write_session_static_json(
            staging_dir / static_output.name,
            metadata,
            resource_files=resource_files,
        )
        manifest["artifacts"][static_output.name] = "prepared"  # type: ignore[index]
        for name, source_rows, role, normalization in raw_jsonl_names_and_rows:
            staged_path = staging_dir / name
            _storage_canonical._write_authoritative_jsonl(staged_path, source_rows)
            detail = _storage_canonical._validate_authoritative_jsonl(
                staged_path,
                source_rows,
                role=role,
                artifact_path=name,
                normalization=normalization,
            )
            manifest["artifacts"][name] = "prepared"  # type: ignore[index]
            manifest["artifact_details"][name] = detail  # type: ignore[index]
        authoritative_raw_artifacts_verified = True
        manifest["authoritative_raw_artifacts_verified"] = True
        for name, source_rows, required in csv_names_and_rows:
            staged_path = staging_dir / name
            _storage_writers.write_dynamic_csv(staged_path, source_rows, required_columns=required)
            manifest["artifacts"][name] = "prepared"  # type: ignore[index]
        for name, source_rows, columns in parquet_names_and_rows:
            staged_path = staging_dir / name
            _storage_writers.write_parquet(staged_path, source_rows, columns=columns)
            manifest["artifacts"][name] = "prepared"  # type: ignore[index]
        _storage_writers._write_px4_workbook(staging_dir / px4_left_xlsx_output.name, compact_px4_rows["left"])
        _storage_writers._write_px4_workbook(staging_dir / px4_right_xlsx_output.name, compact_px4_rows["right"])
        _storage_writers._write_rtk_workbook(staging_dir / session_rtk_xlsx_output.name, compact_rtk_rows)
        for name in (px4_left_xlsx_output.name, px4_right_xlsx_output.name, session_rtk_xlsx_output.name):
            manifest["artifacts"][name] = "prepared"  # type: ignore[index]
    except Exception as exc:
        _storage_canonical._scope_unpublished_artifact_details(manifest, artifact_root=staging_dir)
        manifest.update(
            status="failed",
            phase="prepare_resource_files",
            error=f"resource file preparation failed: {exc}",
            retryable=True,
            staging_dir=str(staging_dir),
        )
        _storage_manifest._write_manifest(manifest_output, manifest)
        raise

    manifest.update(status="prepared", phase="prepare_xlsx", generation_id=generation_id)
    _storage_manifest._write_manifest(manifest_output, manifest)
    error = ""
    try:
        _storage_writers._write_xlsx(
            staging_dir / xlsx_output.name,
            metadata,
            compact_distance_rows,
            camera_rows=compact_camera_rows,
            px4_rows=compact_px4_rows,
            gnss_base_rows=compact_gnss_base_row_list,
            extrinsics_comparison_rows=extrinsics_comparison_full_rows,
        )
        manifest["artifacts"][xlsx_output.name] = "prepared"  # type: ignore[index]
    except Exception as exc:
        error = f"XLSX finalization failed: {exc}"
        manifest["artifacts"][xlsx_output.name] = "failed"  # type: ignore[index]
    generation_dir = generations_root / generation_id
    generation_manifest = dict(manifest)
    generation_manifest.update(
        status="prepared", phase="publish", error=error, retryable=bool(error)
    )
    _storage_manifest._write_manifest(staging_dir / "generation_manifest.json", generation_manifest)
    os.replace(staging_dir, generation_dir)
    try:
        _storage_generation._publish_generation(
            output_dir,
            generation_dir,
            [static_output.name]
            + [name for name, _rows, _required in csv_names_and_rows]
            + [name for name, _rows, _columns in parquet_names_and_rows]
            + [name for name, _rows, _role, _normalization in raw_jsonl_names_and_rows]
            + [px4_left_xlsx_output.name, px4_right_xlsx_output.name, session_rtk_xlsx_output.name]
            + [xlsx_output.name],
        )
    except Exception as exc:
        _storage_canonical._scope_unpublished_artifact_details(manifest, artifact_root=generation_dir)
        manifest.update(
            status="failed", phase="publish", error=f"generation publish failed: {exc}",
            retryable=True, generation_id=generation_id,
        )
        _storage_manifest._write_manifest(manifest_output, manifest)
        raise
    manifest.update(
        status="failed" if error else "finalized",
        phase="published",
        error=error,
        retryable=bool(error),
        generation_id=generation_id,
        published_pointer="published",
        publish_contract="all stable artifact links resolve through one atomically replaced generation pointer",
    )
    _storage_manifest._write_manifest(manifest_output, manifest)
    saved = SavedSessionFiles(
        context_json=static_output,
        measurement_pairs_parquet=distance_parquet_output,
        camera_timeline_parquet=camera_left_parquet_output,
        pixhawk_timeline_parquet=px4_left_parquet_output,
        gnss_base_timeline_parquet=gnss_base_parquet_output,
        geometry_comparison_parquet=extrinsics_comparison_parquet_output,
        review_xlsx=xlsx_output,
        manifest=manifest_output,
        finalized=not error,
        error=error,
    )
    if saved.finalized and authoritative_raw_artifacts_verified:
        for source_rows in (
            pair_rows,
            *camera_rows.values(),
            *px4_rows.values(),
            gnss_base_rows,
            extrinsics_comparison_rows,
        ):
            if isinstance(source_rows, SpoolRows):
                source_rows.discard_spool()
        recovery_metadata = output_dir / ".spool" / "session_metadata.json"
        recovery_metadata.unlink(missing_ok=True)
        try:
            recovery_metadata.parent.rmdir()
        except OSError:
            pass
        _storage_generation._cleanup_finalize_orphans(output_dir)
    return saved
