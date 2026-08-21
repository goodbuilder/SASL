"""Filesystem ownership for the standalone Workstation runtime.

The default directory is deliberately repository-relative so a checkout can
be continued on another machine.  Operators may place runtime state on a
different volume with ``SASL_WORKSTATION_DATA_ROOT``; relative overrides are
still anchored to the repository instead of the launcher's current directory.
"""

from __future__ import annotations

import hashlib
import os
import tempfile
from collections.abc import Iterable
from pathlib import Path, PureWindowsPath


REPO_ROOT = Path(__file__).resolve().parents[2]


def _workstation_data_root() -> Path:
    configured = os.environ.get("SASL_WORKSTATION_DATA_ROOT", "").strip()
    if not configured:
        return REPO_ROOT / ".local" / "workstation_data"
    root = Path(configured).expanduser()
    if not root.is_absolute():
        root = REPO_ROOT / root
    return root.resolve(strict=False)


WORKSTATION_DATA_ROOT = _workstation_data_root()
WORKSTATION_SETTINGS_PATH = WORKSTATION_DATA_ROOT / "stereo_distance_settings.json"
WORKSTATION_RECORDINGS_ROOT = WORKSTATION_DATA_ROOT / "recordings"
WORKSTATION_CALIBRATION_ROOT = WORKSTATION_DATA_ROOT / "calibration"
WORKSTATION_MODELS_ROOT = WORKSTATION_DATA_ROOT / "models"

PORTABLE_SETTINGS_PATH_FIELDS = (
    "intrinsics_path",
    "yolo_weights_path",
    "calibration_root",
    "calibration_validation_root",
    "calibration_validation_json",
)


def resolve_workstation_data_path(value: str | os.PathLike[str]) -> Path:
    """Resolve runtime-relative paths against the canonical data root, not CWD."""

    path = Path(value).expanduser()
    return path if path.is_absolute() else (WORKSTATION_DATA_ROOT / path).resolve(strict=False)


def portable_workstation_data_path_text(
    value: str | os.PathLike[str],
    *,
    data_root: str | os.PathLike[str] = WORKSTATION_DATA_ROOT,
    field_name: str = "path",
    allow_empty: bool = False,
) -> str:
    """Return one normalized data-root-relative path or fail closed.

    Persistent Workstation asset settings must survive a checkout move and a
    different user home.  Existing absolute paths are accepted only when they
    already resolve beneath ``data_root``; paths outside that ownership boundary
    must be imported explicitly before they can be saved.
    """

    text = str(value or "").strip()
    if not text:
        if allow_empty:
            return ""
        raise ValueError(f"{field_name} is empty")
    if PureWindowsPath(text).is_absolute() and not Path(text).is_absolute():
        raise ValueError(f"{field_name} must not use a Windows absolute path: {text}")
    root = Path(data_root).expanduser().resolve(strict=False)
    candidate = Path(text).expanduser()
    resolved = (
        candidate.resolve(strict=False)
        if candidate.is_absolute()
        else (root / candidate).resolve(strict=False)
    )
    try:
        relative = resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError(
            f"{field_name} must stay under SASL_WORKSTATION_DATA_ROOT ({root}): {resolved}"
        ) from exc
    if str(relative) in {"", "."}:
        raise ValueError(f"{field_name} must name a child path beneath {root}")
    return relative.as_posix()


def workstation_settings_portability_errors(
    settings: object,
    *,
    data_root: str | os.PathLike[str] = WORKSTATION_DATA_ROOT,
    fields: Iterable[str] = PORTABLE_SETTINGS_PATH_FIELDS,
) -> list[str]:
    """Validate only portable asset fields, not device or remote paths."""

    errors: list[str] = []
    for field_name in fields:
        if not hasattr(settings, field_name):
            continue
        value = getattr(settings, field_name)
        try:
            portable_workstation_data_path_text(
                value,
                data_root=data_root,
                field_name=field_name,
                allow_empty=field_name == "calibration_validation_json",
            )
        except ValueError as exc:
            errors.append(str(exc))
    return errors


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def import_workstation_asset_file(
    source: str | os.PathLike[str],
    *,
    category: str,
    data_root: str | os.PathLike[str] = WORKSTATION_DATA_ROOT,
) -> Path:
    """Copy one external asset into a content-addressed data-root directory.

    The copy is flushed, atomically published, and independently hashed before
    the portable path may be persisted.  Files already owned by the data root
    are returned unchanged.
    """

    root = Path(data_root).expanduser().resolve(strict=False)
    source_path = Path(source).expanduser().resolve(strict=True)
    if not source_path.is_file():
        raise ValueError(f"asset is not a regular file: {source_path}")
    try:
        source_path.relative_to(root)
        return source_path
    except ValueError:
        pass
    category_text = str(category or "").strip().strip("/")
    if not category_text or Path(category_text).is_absolute() or ".." in Path(category_text).parts:
        raise ValueError(f"invalid portable asset category: {category!r}")

    digest = _sha256_file(source_path)
    target_dir = root / category_text / "imported" / digest
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / source_path.name
    if target.exists():
        if target.is_file() and target.stat().st_size == source_path.stat().st_size:
            if _sha256_file(target) == digest:
                return target
        raise ValueError(f"portable asset target conflicts with verified source: {target}")

    descriptor, temp_name = tempfile.mkstemp(
        dir=target_dir,
        prefix=f".{source_path.name}.",
        suffix=".tmp",
    )
    temp_path = Path(temp_name)
    try:
        copied_digest = hashlib.sha256()
        with source_path.open("rb") as source_handle, os.fdopen(descriptor, "wb") as target_handle:
            for chunk in iter(lambda: source_handle.read(1024 * 1024), b""):
                target_handle.write(chunk)
                copied_digest.update(chunk)
            target_handle.flush()
            os.fsync(target_handle.fileno())
        if copied_digest.hexdigest() != digest or temp_path.stat().st_size != source_path.stat().st_size:
            raise OSError(f"portable asset copy verification failed: {source_path}")
        os.replace(temp_path, target)
        if _sha256_file(target) != digest:
            raise OSError(f"portable asset published hash mismatch: {target}")
        return target
    finally:
        temp_path.unlink(missing_ok=True)

__all__ = [
    "REPO_ROOT",
    "WORKSTATION_DATA_ROOT",
    "WORKSTATION_SETTINGS_PATH",
    "WORKSTATION_RECORDINGS_ROOT",
    "WORKSTATION_CALIBRATION_ROOT",
    "WORKSTATION_MODELS_ROOT",
    "PORTABLE_SETTINGS_PATH_FIELDS",
    "portable_workstation_data_path_text",
    "workstation_settings_portability_errors",
    "import_workstation_asset_file",
    "resolve_workstation_data_path",
]
