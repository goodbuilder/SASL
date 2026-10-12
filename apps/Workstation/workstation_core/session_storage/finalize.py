"""Session finalize responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from . import canonical as _storage_canonical
from . import generation as _storage_generation
from . import manifest as _storage_manifest
from . import paths as _storage_paths
from . import projection as _storage_projection
from . import review as _storage_review
from . import schema as _storage_schema
from .models import SavedSessionFiles
from .spool import SpoolRows
from .disk_index import disk_index, DiskRowTable
from .indexed_records import FinalizationIndexes


def save_session_files(
    output_dir: Path,
    *,
    metadata: dict[str, object],
    pair_rows: Iterable[dict[str, object]],
    camera_rows: dict[str, Iterable[dict[str, object]]] | None = None,
    px4_rows: dict[str, Iterable[dict[str, object]]] | None = None,
    gnss_base_rows: Iterable[dict[str, object]] | None = None,
    extrinsics_comparison_rows: Iterable[dict[str, object]] | None = None,
    method_evaluation_bundle_rows: Iterable[dict[str, object]] | None = None,
) -> SavedSessionFiles:
    """Finalize canonical JSONL plus derived Parquet in one generation.

    Each source is consumed once into canonical JSONL. Source byte receipts
    are verified from disk before bounded derived Parquet writers read rows.
    The generation is published atomically, verified again through the
    ``current`` pointer, and only then may the retry spool be discarded.
    """

    from Workstation.workstation_core.session_parquet import (
        CAMERA_TIMELINE_FILENAME,
        GEOMETRY_COMPARISON_FILENAME,
        GNSS_BASE_TIMELINE_FILENAME,
        MEASUREMENT_PAIRS_FILENAME,
        PX4_TIMELINE_FILENAME,
        finalize_session_parquet_artifacts,
    )
    output_dir = Path(output_dir)
    session_id, measurement_id = _storage_generation._validate_measurement_storage_identity(
        output_dir, metadata
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    # Mirror the legacy path's start-of-save orphan sweep: a prior failed save
    # attempt can leave a ``.preparing`` staging directory (or any other
    # non-current generation) behind under ``.workstation/publish/generations``.
    # Without this, repeated failures leak disk space indefinitely because the
    # success-only cleanup at the end of this function never runs for them.
    _storage_generation._cleanup_v2_generations(output_dir)
    camera_sources = camera_rows or {}
    px4_sources = px4_rows or {}
    jsonl_streams = []
    jsonl_details = {}
    disk_rows = {}

    def write_stream(artifact_key, rows, normalization):
        role, relative_path = _storage_schema.AUTHORITATIVE_JSONL_STREAMS[artifact_key]
        path = staging_dir / relative_path
        receipt = _storage_canonical._write_authoritative_jsonl(path, rows)
        detail = _storage_canonical._validate_authoritative_jsonl(
            path, receipt, role=role, artifact_path=relative_path, normalization=normalization,
        )
        jsonl_streams.append((artifact_key, role, relative_path, receipt, normalization))
        jsonl_details[artifact_key] = detail
        disk_rows[artifact_key] = _storage_canonical.JsonlRows(path, receipt)

    manifest_output = output_dir / "measurement.json"
    generation_id = _storage_paths.session_id_now()
    publish_root = _storage_generation._v2_publish_root(output_dir)
    generations_root = publish_root / "generations"
    generations_root.mkdir(parents=True, exist_ok=True)
    staging_dir = generations_root / f"{generation_id}.preparing"
    staging_dir.mkdir()
    data_dir = staging_dir / "data"
    data_dir.mkdir()
    previous_target: Path | None = None
    generation_dir: Path | None = None

    try:
        previous_target = _storage_generation._v2_current_target(output_dir)
        with disk_index(staging_dir) as connection:
            indexes = FinalizationIndexes(
                connection, metadata=metadata, session_id=session_id,
                measurement_id=measurement_id,
            )
            for side in ("left", "right"):
                write_stream(f"raw_camera_{side}", indexes.camera_rows(side, camera_sources.get(side, ())), "_normalized_generic_rows")
            write_stream("raw_stereo_measurements", indexes.pair_rows(pair_rows), "_normalized_pair_rows")
            for source in dict.fromkeys(("left", "right", *px4_sources)):
                # Preserve extra PX4 partition support in Parquet, but only
                # the canonical catalog's left/right/drone files are published.
                key = f"raw_px4_{source}"
                if key in _storage_schema.AUTHORITATIVE_JSONL_STREAMS:
                    write_stream(key, indexes.generic_rows(px4_sources.get(source, ())), "_normalized_generic_rows")
                else:
                    path = staging_dir / f".px4-{len(disk_rows)}.jsonl"
                    receipt = _storage_canonical._write_authoritative_jsonl(path, indexes.generic_rows(px4_sources[source]))
                    _storage_canonical._validate_authoritative_jsonl(path, receipt, role=key, artifact_path=path.name, normalization="_normalized_generic_rows")
                    disk_rows[key] = _storage_canonical.JsonlRows(path, receipt)
            write_stream("raw_gnss_base", indexes.generic_rows(gnss_base_rows or ()), "_normalized_generic_rows")
            write_stream("geometry_contexts", indexes.contexts, "geometry_context_record_v1")
            write_stream("time_sync_epochs", indexes.epochs, "time_sync_epoch_record_v2")
            method_tables = {method_id: DiskRowTable(connection, f"method_{index}") for index, method_id in enumerate(MEASUREMENT_METHOD_IDS)}
            write_stream("method_evaluation_bundles", indexes.method_bundle_rows(method_evaluation_bundle_rows or (), method_tables), "workstation-method-evaluation-bundle-v1")
            for method_id, rows in method_tables.items():
                write_stream(f"method_result_{method_id}", rows, "workstation-method-evaluation-v1")
            write_stream("raw_extrinsics_comparison", indexes.generic_rows(extrinsics_comparison_rows or ()), "_normalized_generic_rows")
            if not disk_rows["raw_extrinsics_comparison"]:
                disk_rows["raw_extrinsics_comparison"].path.unlink()
                jsonl_streams = [stream for stream in jsonl_streams if stream[0] != "raw_extrinsics_comparison"]
                jsonl_details.pop("raw_extrinsics_comparison")
                disk_rows.pop("raw_extrinsics_comparison")
            # Manifest/resource order follows the historical stream catalog.
            order = ("raw_stereo_measurements", "raw_camera_left", "raw_camera_right", "raw_px4_left", "raw_px4_right", "raw_gnss_base", "geometry_contexts", "time_sync_epochs", "method_evaluation_bundles", *(f"method_result_{method_id}" for method_id in MEASUREMENT_METHOD_IDS), "raw_px4_drone", "raw_extrinsics_comparison")
            jsonl_streams.sort(key=lambda stream: order.index(stream[0]))
            jsonl_details = {key: jsonl_details[key] for key in order if key in jsonl_details}
            artifacts = finalize_session_parquet_artifacts(
                data_dir,
                measurement_pairs=_storage_projection._iter_project_pair_typed_fields(disk_rows["raw_stereo_measurements"]),
                camera_rows={side: _storage_projection._iter_project_camera_typed_fields(disk_rows[f"raw_camera_{side}"]) for side in ("left", "right")},
                px4_rows={source: _storage_projection._iter_project_px4_typed_fields(disk_rows[f"raw_px4_{source}"]) for source in dict.fromkeys((*px4_sources, "left", "right"))},
                gnss_base_rows=disk_rows["raw_gnss_base"],
                geometry_comparison_rows=disk_rows.get("raw_extrinsics_comparison", ()),
            )
            for key, rows in disk_rows.items():
                if key not in _storage_schema.AUTHORITATIVE_JSONL_STREAMS:
                    rows.path.unlink()
        resource_files = {
            **{
                artifact_key: relative_path
                for artifact_key, _role, relative_path, _rows, _normalization in jsonl_streams
            },
            "measurement_pairs_parquet": f"data/{MEASUREMENT_PAIRS_FILENAME}",
            "camera_timeline_parquet": f"data/{CAMERA_TIMELINE_FILENAME}",
            "pixhawk_timeline_parquet": f"data/{PX4_TIMELINE_FILENAME}",
            "gnss_base_timeline_parquet": f"data/{GNSS_BASE_TIMELINE_FILENAME}",
            "geometry_comparison_parquet": f"data/{GEOMETRY_COMPARISON_FILENAME}",
            "context": "data/context.json",
        }
        context_payload = {
            "file_type": "workstation_measurement_context",
            "schema_version": "workstation-measurement-context-v2",
            "session_id": session_id,
            "measurement_id": measurement_id,
            "measurement_package_id": measurement_id,
            "time_axis": {
                "primary": "recording_time_ns",
                "definition": "nanoseconds elapsed from measurement recording start",
            },
            "resource_files": resource_files,
            "geometry_context_registry": resource_files["geometry_contexts"],
            "time_sync_epoch_registry": resource_files["time_sync_epochs"],
            "metadata": metadata,
        }
        context_path = data_dir / "context.json"
        _storage_manifest._write_manifest(context_path, context_payload)
        parquet_details = artifacts.to_mapping(relative_to=staging_dir)
        parquet_derivations = {
            "measurement_pairs": ("raw_stereo_measurements",),
            "camera_timeline": ("raw_camera_left", "raw_camera_right"),
            "pixhawk_timeline": (
                "raw_px4_left",
                "raw_px4_right",
                "raw_px4_drone",
            ),
            "gnss_base_timeline": ("raw_gnss_base",),
            "geometry_comparison": ("raw_extrinsics_comparison",),
        }
        for artifact_key, source_keys in parquet_derivations.items():
            parquet_details[artifact_key]["derived_from"] = [
                dict(jsonl_details[source_key])
                for source_key in source_keys
                if source_key in jsonl_details
            ]
        context_detail = {
            "role": "measurement_context",
            "media_type": "application/json",
            "relative_path": "data/context.json",
            "size_bytes": context_path.stat().st_size,
            "sha256": _storage_manifest._sha256_path(context_path),
            "row_count": None,
            "schema_version": "workstation-measurement-context-v2",
            "authoritative": True,
            "derived_from": [],
            "verified": True,
        }
        required_jsonl_roles = [
            _storage_schema.AUTHORITATIVE_JSONL_STREAMS[key][0]
            for key in (
                "raw_stereo_measurements",
                "raw_camera_left",
                "raw_camera_right",
                "raw_px4_left",
                "raw_px4_right",
                "raw_gnss_base",
                "geometry_contexts",
                "time_sync_epochs",
                "method_evaluation_bundles",
                *(
                    f"method_result_{method_id}"
                    for method_id in MEASUREMENT_METHOD_IDS
                ),
            )
        ]
        generation_manifest = {
            "schema": _storage_schema.MEASUREMENT_STORAGE_SCHEMA,
            "schema_version": _storage_schema.MEASUREMENT_STORAGE_SCHEMA_VERSION,
            "session_id": session_id,
            "measurement_id": measurement_id,
            "measurement_package_id": measurement_id,
            "status": "prepared",
            "generation_id": generation_id,
            "authority_contract": {
                "structured_records": "canonical_jsonl",
                "binary_evidence": "native_bytes",
                "static_metadata": "json",
                "parquet": "derived_analysis",
                "xlsx": "derived_review",
            },
            "resource_files_authoritative": False,
            "resource_files": resource_files,
            "artifact_details": {
                **jsonl_details,
                **parquet_details,
                "context": context_detail,
            },
            "authoritative_jsonl_artifacts_verified": True,
            "parquet_artifacts_verified": True,
            "authoritative_parquet_artifacts_verified": False,
            "completeness": {
                "complete": True,
                "required_authoritative_roles": required_jsonl_roles,
                "missing_required_authoritative_roles": [],
            },
            "time_axis": "recording_time_ns",
        }
        manifest = {
            **generation_manifest,
            "status": "finalized",
            "phase": "published",
            "retryable": False,
            "published_pointer": ".workstation/publish/current",
            "data_view": "data",
            "publish_contract": (
                "data and measurement.json resolve through one atomically replaced generation pointer"
            ),
        }
        _storage_manifest._write_manifest(staging_dir / "generation_manifest.json", generation_manifest)
        _storage_manifest._write_manifest(staging_dir / "measurement.json", manifest)
        _storage_manifest._fsync_directory_path(data_dir)
        _storage_manifest._fsync_directory_path(staging_dir)
        generation_dir = generations_root / generation_id
        os.replace(staging_dir, generation_dir)
        _storage_manifest._fsync_directory_path(generations_root)
        _storage_generation._publish_v2_generation(output_dir, generation_dir)
        _storage_generation._validate_published_v2_generation(
            output_dir,
            generation_id=generation_id,
            expected_manifest=manifest,
            jsonl_streams=jsonl_streams,
        )
    except Exception as exc:
        rollback_error = ""
        if generation_dir is not None:
            try:
                current_target = _storage_generation._v2_current_target(output_dir)
                if current_target == Path("generations") / generation_dir.name:
                    _storage_generation._restore_v2_current_target(
                        output_dir,
                        failed_generation=generation_dir,
                        previous_target=previous_target,
                    )
            except Exception as rollback_exc:
                rollback_error = str(rollback_exc)
        failure = {
            "schema": _storage_schema.MEASUREMENT_STORAGE_SCHEMA,
            "schema_version": _storage_schema.MEASUREMENT_STORAGE_SCHEMA_VERSION,
            "session_id": session_id,
            "measurement_id": measurement_id,
            "status": "failed",
            "phase": "prepare_validate_or_publish_generation",
            "error": (
                str(exc)
                if not rollback_error
                else f"{exc}; generation pointer rollback failed: {rollback_error}"
            ),
            "retryable": True,
            "generation_id": generation_id,
        }
        failure_path = output_dir / ".workstation" / "save_failure.json"
        _storage_manifest._write_manifest(failure_path, failure)
        _storage_manifest._fsync_directory_path(failure_path.parent)
        if rollback_error:
            raise OSError(failure["error"]) from exc
        raise
    failure_path = output_dir / ".workstation" / "save_failure.json"
    failure_path.unlink(missing_ok=True)
    _storage_manifest._fsync_directory_path(failure_path.parent)

    session_root = _storage_generation._owning_session_root(output_dir)
    review_path = session_root / "review.xlsx"
    review_status_path = session_root / ".workstation" / "review.json"
    review_error = ""
    review_pending = output_dir.parent.name == "measurements"
    if review_pending:
        _storage_manifest._write_manifest(
            review_status_path,
            {
                "status": "pending_session_close",
                "path": "review.xlsx",
                "regenerable": True,
            },
        )
        _storage_manifest._fsync_directory_path(review_status_path.parent)
    else:
        try:
            review_path = _storage_review.rebuild_session_review(session_root, session_id)
        except Exception as exc:
            # The workbook is a reproducible operator view, never the
            # measurement source of truth. Preserve an older review and keep
            # typed evidence finalized when regeneration fails.
            review_error = str(exc)

    data_view = output_dir / "data"
    saved = SavedSessionFiles(
        context_json=data_view / "context.json",
        measurement_pairs_parquet=data_view / MEASUREMENT_PAIRS_FILENAME,
        camera_timeline_parquet=data_view / CAMERA_TIMELINE_FILENAME,
        pixhawk_timeline_parquet=data_view / PX4_TIMELINE_FILENAME,
        gnss_base_timeline_parquet=data_view / GNSS_BASE_TIMELINE_FILENAME,
        geometry_comparison_parquet=data_view / GEOMETRY_COMPARISON_FILENAME,
        review_xlsx=review_path,
        manifest=manifest_output,
        authoritative_jsonl_files=tuple(
            output_dir / relative_path
            for _key, _role, relative_path, _rows, _normalization in jsonl_streams
        ),
        finalized=True,
        review_error=review_error,
        review_pending=review_pending,
    )
    for source_rows in (
        pair_rows,
        *camera_sources.values(),
        *px4_sources.values(),
        gnss_base_rows,
        extrinsics_comparison_rows,
        method_evaluation_bundle_rows,
    ):
        if isinstance(source_rows, SpoolRows):
            source_rows.discard_spool()
    recovery_metadata = output_dir / ".spool" / "session_metadata.json"
    try:
        recovery_metadata.unlink(missing_ok=True)
    except OSError:
        pass
    try:
        recovery_metadata.parent.rmdir()
    except OSError:
        pass
    _storage_generation._cleanup_v2_generations(output_dir)
    return saved
