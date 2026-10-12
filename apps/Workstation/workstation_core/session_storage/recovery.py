"""Session recovery responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import json
from pathlib import Path
from . import manifest as _storage_manifest
from .models import RetryableSession
from .spool import SpoolRows


def write_recovery_metadata(output_dir: Path, metadata: dict[str, object]) -> Path:
    path = output_dir / ".spool" / "session_metadata.json"
    _storage_manifest._write_manifest(path, metadata)
    return path


def discover_retryable_sessions(recordings_root: Path) -> list[Path]:
    """Find retryable v2 sessions and legacy ``sessions/`` descendants."""

    retryable: list[Path] = []
    candidates: list[Path] = []
    roots: list[Path] = []
    direct_roots: dict[str, Path] = {}
    if recordings_root.is_dir():
        direct_roots = {
            child.name: child
            for child in recordings_root.iterdir()
            if child.name != "sessions"
            and not child.name.startswith(".")
            and child.is_dir()
            and not child.is_symlink()
        }
        roots.extend(direct_roots.values())
    legacy_root = recordings_root / "sessions"
    if legacy_root.is_dir() and not legacy_root.is_symlink():
        legacy_roots = {
            child.name: child
            for child in legacy_root.iterdir()
            if child.is_dir() and not child.is_symlink()
        }

        def has_spool(root: Path) -> bool:
            if (root / ".spool").is_dir():
                return True
            measurements = root / "measurements"
            return measurements.is_dir() and any(
                (child / ".spool").is_dir()
                for child in measurements.iterdir()
                if child.is_dir() and not child.is_symlink()
            )

        conflicts = sorted(
            name
            for name in set(direct_roots) & set(legacy_roots)
            if has_spool(direct_roots[name]) or has_spool(legacy_roots[name])
        )
        if conflicts:
            raise FileExistsError(
                "direct/legacy retry session ID collision: "
                + ", ".join(conflicts)
            )
        roots.extend(legacy_roots.values())
    for session_root in roots:
        candidates.append(session_root)
        measurements_root = session_root / "measurements"
        if measurements_root.is_dir() and not measurements_root.is_symlink():
            candidates.extend(
                child
                for child in measurements_root.iterdir()
                if child.is_dir() and not child.is_symlink()
            )
    for output_dir in candidates:
        spool_dir = output_dir / ".spool"
        if (
            not spool_dir.is_dir()
            or not (spool_dir / "session_metadata.json").is_file()
        ):
            continue
        status = ""
        failure_path = output_dir / ".workstation" / "save_failure.json"
        manifest_path = output_dir / "measurement.json"
        if failure_path.is_file():
            status = "failed"
        elif not manifest_path.is_file():
            manifest_path = output_dir / "session_manifest.json"
        if not status and manifest_path.is_file():
            try:
                status = str(json.loads(manifest_path.read_text(encoding="utf-8")).get("status", ""))
            except (OSError, ValueError):
                status = "failed"
        if status != "finalized":
            retryable.append(output_dir)
    return sorted(
        retryable,
        key=lambda path: (path.stat().st_mtime_ns, path.as_posix()),
        reverse=True,
    )


def load_retryable_session(output_dir: Path) -> RetryableSession:
    spool_dir = output_dir / ".spool"
    metadata_path = spool_dir / "session_metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    pair_rows = SpoolRows(supports_updates=True)
    pair_rows.open_existing(spool_dir / "pair_rows.jsonl")
    camera_rows: dict[str, SpoolRows] = {}
    px4_rows: dict[str, SpoolRows] = {}
    for side in ("left", "right"):
        camera_rows[side] = SpoolRows()
        camera_rows[side].open_existing(spool_dir / f"camera_{side}.jsonl")
        px4_rows[side] = SpoolRows()
        px4_rows[side].open_existing(spool_dir / f"px4_{side}.jsonl")
    drone_spool = spool_dir / "px4_drone.jsonl"
    if drone_spool.exists():
        px4_rows["drone"] = SpoolRows()
        px4_rows["drone"].open_existing(drone_spool)
    gnss_base_rows = SpoolRows()
    # Sessions recorded before this field existed have no gnss_base.jsonl;
    # open_existing() tolerates a missing path (empty rows, fresh append handle).
    gnss_base_rows.open_existing(spool_dir / "gnss_base.jsonl")
    extrinsics_comparison_rows = SpoolRows()
    # Sessions recorded before this field existed have no extrinsics_comparison.jsonl;
    # open_existing() tolerates a missing path (empty rows, fresh append handle).
    extrinsics_comparison_rows.open_existing(spool_dir / "extrinsics_comparison.jsonl")
    method_evaluation_bundle_rows = SpoolRows()
    # Pre multi-method sessions have no bundle spool; an empty stream remains
    # a valid recovery input and is materialized into eight empty method files.
    method_evaluation_bundle_rows.open_existing(
        spool_dir / "method_evaluation_bundles.jsonl"
    )
    recovered_spools = {
        "pair": pair_rows,
        "camera_left": camera_rows["left"],
        "camera_right": camera_rows["right"],
        "px4_left": px4_rows["left"],
        "px4_right": px4_rows["right"],
        "gnss_base": gnss_base_rows,
        "extrinsics_comparison": extrinsics_comparison_rows,
        "method_evaluation_bundles": method_evaluation_bundle_rows,
    }
    if "drone" in px4_rows:
        recovered_spools["px4_drone"] = px4_rows["drone"]
    recovery_warnings = [
        {"stream": stream, "warning": warning}
        for stream, rows in recovered_spools.items()
        for warning in rows.recovery_warnings
    ]
    recovery_artifacts = [
        {
            "stream": stream,
            "path": artifact.relative_to(output_dir).as_posix(),
        }
        for stream, rows in recovered_spools.items()
        for artifact in rows.recovery_artifacts
    ]
    if recovery_warnings or recovery_artifacts:
        metadata["spool_recovery"] = {
            "status": "RECOVERED_WITH_WARNINGS",
            "warnings": recovery_warnings,
            "artifacts": recovery_artifacts,
        }
    return RetryableSession(
        output_dir,
        metadata,
        pair_rows,
        camera_rows,
        px4_rows,
        gnss_base_rows,
        extrinsics_comparison_rows,
        method_evaluation_bundle_rows,
    )
