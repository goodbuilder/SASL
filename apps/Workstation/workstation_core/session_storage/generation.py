"""Session generation responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import json
import os
import shutil
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Iterable
from . import canonical as _storage_canonical
from . import manifest as _storage_manifest
from . import paths as _storage_paths


def _cleanup_finalize_orphans(output_dir: Path) -> None:
    for path in output_dir.glob(".finalize-*"):
        if path.is_dir():
            shutil.rmtree(path, ignore_errors=True)
    generations = output_dir / ".generations"
    if generations.is_dir():
        published_name = ""
        published = output_dir / "published"
        if published.is_symlink():
            published_name = Path(os.readlink(published)).name
        for path in generations.glob("*.preparing"):
            shutil.rmtree(path, ignore_errors=True)
        for path in generations.iterdir():
            if path.is_dir() and not path.name.endswith(".preparing") and path.name != published_name:
                shutil.rmtree(path, ignore_errors=True)


def _ensure_stable_artifact_links(output_dir: Path, names: Iterable[str]) -> None:
    names = list(names)
    regular_paths = [output_dir / name for name in names if (output_dir / name).exists() and not (output_dir / name).is_symlink()]
    if regular_paths:
        generation_id = f"legacy-{_storage_paths.session_id_now()}"
        legacy_dir = output_dir / ".generations" / generation_id
        legacy_dir.mkdir(parents=True, exist_ok=False)
        for path in regular_paths:
            shutil.copy2(path, legacy_dir / path.name)
        pointer_tmp = output_dir / f".published-{generation_id}.tmp"
        pointer_tmp.symlink_to(Path(".generations") / generation_id)
        os.replace(pointer_tmp, output_dir / "published")
        for path in regular_paths:
            link_tmp = output_dir / f".{path.name}.link.tmp"
            link_tmp.unlink(missing_ok=True)
            link_tmp.symlink_to(Path("published") / path.name)
            os.replace(link_tmp, path)
    for name in names:
        path = output_dir / name
        if os.path.lexists(path):
            if path.is_symlink():
                continue
            raise OSError(f"stable artifact path is not a symlink: {path}")
        path.symlink_to(Path("published") / name)


def _publish_generation(output_dir: Path, generation_dir: Path, names: Iterable[str]) -> None:
    _ensure_stable_artifact_links(output_dir, names)
    temp_link = output_dir / f".published-{generation_dir.name}.tmp"
    temp_link.unlink(missing_ok=True)
    temp_link.symlink_to(Path(".generations") / generation_dir.name)
    try:
        os.replace(temp_link, output_dir / "published")
    finally:
        temp_link.unlink(missing_ok=True)


def _v2_publish_root(output_dir: Path) -> Path:
    return output_dir / ".workstation" / "publish"


def _ensure_v2_data_link(output_dir: Path) -> None:
    """Expose one data link and one generation-bound measurement manifest."""

    links = {
        output_dir / "data": Path(".workstation") / "publish" / "current" / "data",
        output_dir / "measurement.json": (
            Path(".workstation") / "publish" / "current" / "measurement.json"
        ),
    }
    changed = False
    for link, expected in links.items():
        if os.path.lexists(link):
            if not link.is_symlink() or Path(os.readlink(link)) != expected:
                raise OSError(f"measurement v2 path is not a managed link: {link}")
            continue
        link.symlink_to(expected)
        changed = True
    if changed:
        _storage_manifest._fsync_directory_path(output_dir)


def _publish_v2_generation(output_dir: Path, generation_dir: Path) -> None:
    publish_root = _v2_publish_root(output_dir)
    _ensure_v2_data_link(output_dir)
    current = publish_root / "current"
    temporary = publish_root / f".current-{generation_dir.name}.tmp"
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(Path("generations") / generation_dir.name)
    try:
        os.replace(temporary, current)
        _storage_manifest._fsync_directory_path(publish_root)
    finally:
        temporary.unlink(missing_ok=True)


def _v2_current_target(output_dir: Path) -> Path | None:
    current = _v2_publish_root(output_dir) / "current"
    if not os.path.lexists(current):
        return None
    if not current.is_symlink():
        raise OSError(f"measurement generation pointer is not a symlink: {current}")
    target = Path(os.readlink(current))
    if target.is_absolute() or len(target.parts) != 2 or target.parts[0] != "generations":
        raise OSError(f"measurement generation pointer target is invalid: {target}")
    if target.parts[1] in {"", ".", ".."}:
        raise OSError(f"measurement generation pointer target is invalid: {target}")
    return target


def _restore_v2_current_target(
    output_dir: Path,
    *,
    failed_generation: Path,
    previous_target: Path | None,
) -> None:
    """Roll back only the pointer published by the failed finalization."""

    publish_root = _v2_publish_root(output_dir)
    current = publish_root / "current"
    expected_failed_target = Path("generations") / failed_generation.name
    if _v2_current_target(output_dir) != expected_failed_target:
        raise OSError("measurement generation pointer changed during validation")
    if previous_target is None:
        current.unlink()
        _storage_manifest._fsync_directory_path(publish_root)
        return
    temporary = publish_root / f".current-rollback-{failed_generation.name}.tmp"
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(previous_target)
    try:
        os.replace(temporary, current)
        _storage_manifest._fsync_directory_path(publish_root)
    finally:
        temporary.unlink(missing_ok=True)


def _strict_json_object(path: Path) -> dict[str, object]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant is forbidden: {value}")

    payload = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=reject_constant,
    )
    if not isinstance(payload, dict):
        raise OSError(f"JSON artifact is not an object: {path}")
    return payload


def _validate_published_v2_generation(
    output_dir: Path,
    *,
    generation_id: str,
    expected_manifest: Mapping[str, object],
    jsonl_streams: Sequence[
        tuple[str, str, str, Sequence[dict[str, object]] | _storage_canonical.JsonlReceipt, str]
    ],
) -> None:
    """Verify the bytes reached through the atomically published pointer."""

    from Workstation.workstation_core.session_parquet import verify_parquet_artifact

    publish_root = _v2_publish_root(output_dir)
    current = publish_root / "current"
    expected_target = Path("generations") / generation_id
    if _v2_current_target(output_dir) != expected_target:
        raise OSError("published measurement generation pointer is stale")
    expected_root = (publish_root / expected_target).resolve(strict=True)
    published_root = current.resolve(strict=True)
    if published_root != expected_root or not published_root.is_dir():
        raise OSError("published measurement generation resolves outside its target")

    published_manifest = _strict_json_object(current / "measurement.json")
    if published_manifest != dict(expected_manifest):
        raise OSError("published measurement manifest changed after staging")
    artifact_details = published_manifest.get("artifact_details")
    if not isinstance(artifact_details, Mapping):
        raise OSError("published measurement artifact_details is missing")

    for artifact_key, role, relative_path, rows, normalization in jsonl_streams:
        declared = artifact_details.get(artifact_key)
        if not isinstance(declared, Mapping):
            raise OSError(f"published JSONL descriptor is missing: {artifact_key}")
        verified = _storage_canonical._validate_authoritative_jsonl(
            current / relative_path,
            rows,
            role=role,
            artifact_path=relative_path,
            normalization=normalization,
        )
        if verified != dict(declared):
            raise OSError(f"published JSONL descriptor mismatch: {artifact_key}")

    for artifact_key in (
        "measurement_pairs",
        "camera_timeline",
        "pixhawk_timeline",
        "gnss_base_timeline",
        "geometry_comparison",
    ):
        declared = artifact_details.get(artifact_key)
        if not isinstance(declared, Mapping):
            raise OSError(f"published Parquet descriptor is missing: {artifact_key}")
        relative_path = str(declared.get("relative_path") or "")
        verified = verify_parquet_artifact(
            current / relative_path,
            artifact_key,
        ).to_mapping(relative_to=current)
        derived_from = declared.get("derived_from")
        if not isinstance(derived_from, list):
            raise OSError(f"Parquet derived_from is invalid: {artifact_key}")
        for source_descriptor in derived_from:
            if not isinstance(source_descriptor, Mapping) or not any(
                dict(source_descriptor) == dict(candidate)
                and candidate.get("authoritative") is True
                and candidate.get("media_type") == "application/x-ndjson"
                for candidate in artifact_details.values()
                if isinstance(candidate, Mapping)
            ):
                raise OSError(
                    f"Parquet derived_from does not resolve: {artifact_key}"
                )
        verified["derived_from"] = [dict(value) for value in derived_from]
        if verified != dict(declared):
            raise OSError(f"published Parquet descriptor mismatch: {artifact_key}")

    context = artifact_details.get("context")
    if not isinstance(context, Mapping):
        raise OSError("published context descriptor is missing")
    context_path = current / str(context.get("relative_path") or "")
    if (
        not context_path.is_file()
        or context_path.stat().st_size != int(context.get("size_bytes") or -1)
        or _storage_manifest._sha256_path(context_path) != str(context.get("sha256") or "")
    ):
        raise OSError("published context bytes do not match their descriptor")


def _cleanup_v2_generations(output_dir: Path) -> None:
    publish_root = _v2_publish_root(output_dir)
    generations = publish_root / "generations"
    if not generations.is_dir():
        return
    current_name = ""
    current = publish_root / "current"
    if current.is_symlink():
        current_name = Path(os.readlink(current)).name
    for candidate in generations.iterdir():
        if candidate.is_dir() and candidate.name != current_name:
            shutil.rmtree(candidate, ignore_errors=True)


def _owning_session_root(output_dir: Path) -> Path:
    if output_dir.parent.name == "measurements":
        return output_dir.parent.parent
    return output_dir


def _validate_measurement_storage_identity(
    output_dir: Path,
    metadata: Mapping[str, object],
) -> tuple[str, str]:
    raw_session_id = str(metadata.get("session_id") or output_dir.name)
    # Reuse the public session path contract without creating the path.
    session_id = _storage_paths.session_dir(Path("."), raw_session_id).name
    raw_measurement_id = str(metadata.get("measurement_id") or output_dir.name)
    if (
        not raw_measurement_id
        or raw_measurement_id in {".", ".."}
        or raw_measurement_id.startswith(".")
        or Path(raw_measurement_id).name != raw_measurement_id
        or "\\" in raw_measurement_id
        or "\x00" in raw_measurement_id
    ):
        raise ValueError("measurement_id must be one safe non-hidden path component")
    if output_dir.parent.name == "measurements":
        physical_session_id = output_dir.parent.parent.name
        if session_id != physical_session_id:
            raise ValueError(
                f"metadata session_id {session_id!r} does not own {output_dir}"
            )
        if raw_measurement_id != output_dir.name:
            raise ValueError(
                f"metadata measurement_id {raw_measurement_id!r} does not own {output_dir}"
            )
    elif session_id != output_dir.name:
        raise ValueError(
            f"metadata session_id {session_id!r} does not own {output_dir}"
        )
    return session_id, raw_measurement_id
