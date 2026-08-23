"""Portable calibration photo-selection sidecar helpers.

Selection paths are owned by the photo-set root. Persist them relative to that
root so a cloned checkout can reopen the same files. Legacy absolute paths are
rebased only when their suffix can be proven to remain inside the current root.
"""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

CALIBRATION_SELECTION_SCHEMA = "workstation-calibration-file-selection-v2"
LEGACY_CALIBRATION_SELECTION_SCHEMA = "workstation-calibration-file-selection-v1"
CALIBRATION_SELECTION_STATES = frozenset({"ABSENT", "VALID", "INVALID"})
CALIBRATION_IMAGE_SUFFIXES = frozenset(
    {".png", ".jpg", ".jpeg", ".tif", ".tiff", ".bmp"}
)

_SELECTION_LIST_FIELDS = (
    "included_keys",
    "left_paths",
    "right_paths",
    "excluded_left_paths",
    "excluded_right_paths",
    "excluded_pair_keys",
)


def empty_calibration_selection() -> dict[str, object]:
    return {
        "schema": CALIBRATION_SELECTION_SCHEMA,
        "root": ".",
        "manual_mode": False,
        "included_keys": [],
        "left_paths": [],
        "right_paths": [],
        "excluded_left_paths": [],
        "excluded_right_paths": [],
        "excluded_pair_keys": [],
    }


@dataclass(frozen=True)
class CalibrationSelectionDocument:
    state: str
    canonical_json: str
    errors: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.state not in CALIBRATION_SELECTION_STATES:
            raise ValueError("unsupported calibration selection document state")
        payload = json.loads(self.canonical_json)
        if not isinstance(payload, dict):
            raise ValueError("calibration selection canonical_json must contain an object")
        if self.state == "INVALID" and not self.errors:
            raise ValueError("invalid calibration selection requires an error")
        if self.state != "INVALID" and self.errors:
            raise ValueError("valid/absent calibration selection cannot contain errors")

    @property
    def selection(self) -> dict[str, object]:
        return json.loads(self.canonical_json)

    def to_mapping(self) -> dict[str, object]:
        return {
            **self.selection,
            "_sidecar_state": self.state,
            "_sidecar_errors": list(self.errors),
        }


def _canonical_selection_json(payload: Mapping[str, object]) -> str:
    return json.dumps(
        dict(payload),
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    )


def _selection_key_error(value: str, field: str) -> str | None:
    if not value or value in {".", ".."}:
        return f"{field} contains an empty/invalid pair identity"
    if "/" in value or "\\" in value or "\x00" in value:
        return f"{field} contains a path-like pair identity: {value!r}"
    return None


def _validate_selection_payload(payload: object) -> tuple[dict[str, object], tuple[str, ...]]:
    if not isinstance(payload, Mapping):
        return empty_calibration_selection(), ("selection JSON must contain an object",)
    schema = payload.get("schema")
    if schema not in {
        CALIBRATION_SELECTION_SCHEMA,
        LEGACY_CALIBRATION_SELECTION_SCHEMA,
    }:
        return empty_calibration_selection(), (
            f"unsupported calibration selection schema: {schema!r}",
        )
    errors: list[str] = []
    root = payload.get("root", ".")
    if not isinstance(root, str) or not root.strip():
        errors.append("selection root must be a non-empty string")
        root = "."
    if schema == CALIBRATION_SELECTION_SCHEMA and root != ".":
        errors.append("v2 selection root must be '.'")
    manual_mode = payload.get("manual_mode", False)
    if type(manual_mode) is not bool:
        errors.append("manual_mode must be a JSON boolean")
        manual_mode = False

    normalized: dict[str, list[str]] = {}
    for field in _SELECTION_LIST_FIELDS:
        raw = payload.get(field, [])
        if not isinstance(raw, list):
            errors.append(f"{field} must be a JSON list")
            normalized[field] = []
            continue
        values: list[str] = []
        for index, item in enumerate(raw):
            if not isinstance(item, str) or not item.strip():
                errors.append(f"{field}[{index}] must be a non-empty string")
                continue
            value = item.strip()
            if value in values:
                errors.append(f"{field} contains duplicate value: {value!r}")
                continue
            values.append(value)
        normalized[field] = values

    for field in ("included_keys", "excluded_pair_keys"):
        for value in normalized[field]:
            error = _selection_key_error(value, field)
            if error:
                errors.append(error)

    if schema == CALIBRATION_SELECTION_SCHEMA:
        path_sides = {
            "left_paths": "left",
            "right_paths": "right",
            "excluded_left_paths": "left",
            "excluded_right_paths": "right",
        }
        for field, side in path_sides.items():
            for value in normalized[field]:
                path = Path(value)
                if path.is_absolute() or ".." in path.parts:
                    errors.append(f"{field} must contain a portable relative path: {value!r}")
                elif not path.parts or path.parts[0].lower() != side:
                    errors.append(f"{field} path must remain under {side}/: {value!r}")
                elif path.suffix.lower() not in CALIBRATION_IMAGE_SUFFIXES:
                    errors.append(f"{field} path has unsupported image suffix: {value!r}")

    if manual_mode and not (
        normalized["left_paths"]
        or normalized["right_paths"]
        or normalized["included_keys"]
    ):
        errors.append("manual selection contains no files or included pair identities")

    result: dict[str, object] = {
        "schema": str(schema),
        "root": str(root),
        "manual_mode": bool(manual_mode),
        **normalized,
    }
    return result, tuple(dict.fromkeys(errors))


def load_calibration_selection_sidecar(path: Path) -> CalibrationSelectionDocument:
    sidecar = Path(path)
    empty = empty_calibration_selection()
    if not sidecar.exists():
        return CalibrationSelectionDocument("ABSENT", _canonical_selection_json(empty))
    try:
        payload = json.loads(sidecar.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError) as exc:
        return CalibrationSelectionDocument(
            "INVALID",
            _canonical_selection_json(empty),
            (f"cannot parse calibration selection sidecar: {exc}",),
        )
    normalized, errors = _validate_selection_payload(payload)
    return CalibrationSelectionDocument(
        "INVALID" if errors else "VALID",
        _canonical_selection_json(normalized),
        errors,
    )


def calibration_selection_path_text(root: Path, path: Path) -> str:
    """Return one normalized photo path relative to ``root``.

    Paths outside the selected photo set are rejected rather than serialized as
    a machine-local dependency.
    """

    resolved_root = Path(root).expanduser().resolve(strict=False)
    resolved_path = Path(path).expanduser().resolve(strict=False)
    try:
        relative = resolved_path.relative_to(resolved_root)
    except ValueError as exc:
        raise ValueError(
            f"calibration selection path must stay under photo-set root: {resolved_path}"
        ) from exc
    if not relative.parts or relative == Path("."):
        raise ValueError("calibration selection path must identify an image file")
    return relative.as_posix()


def resolve_calibration_selection_paths(
    raw_paths: object,
    root: Path,
    *,
    stored_root: object = None,
) -> tuple[list[Path], tuple[str, ...]]:
    """Resolve current and legacy sidecar paths without escaping ``root``.

    Returns both valid files and unresolved raw entries. Callers deciding a
    calculation input set must fail closed when the latter is non-empty.
    """

    if not isinstance(raw_paths, (list, tuple)):
        return [], ()
    resolved_root = Path(root).expanduser().resolve(strict=False)
    legacy_root_text = str(stored_root or "").strip()
    legacy_root = (
        Path(legacy_root_text).expanduser()
        if legacy_root_text and legacy_root_text != "."
        else None
    )
    valid: list[Path] = []
    errors: list[str] = []
    for raw in raw_paths:
        text = str(raw or "").strip()
        if not text:
            continue
        source = Path(text).expanduser()
        candidates: list[Path] = []
        if source.is_absolute():
            candidates.append(source)
            if legacy_root is not None:
                try:
                    candidates.append(resolved_root / source.relative_to(legacy_root))
                except ValueError:
                    pass
            # Older sidecars always stored root/left/name or root/right/name.
            # This bounded suffix fallback recovers them even if their stored
            # root field was omitted, while still forbidding arbitrary paths.
            if source.parent.name.lower() in {"left", "right"}:
                candidates.append(
                    resolved_root / source.parent.name.lower() / source.name
                )
        else:
            candidates.append(resolved_root / source)

        selected: Path | None = None
        for candidate in candidates:
            resolved = candidate.resolve(strict=False)
            try:
                resolved.relative_to(resolved_root)
            except ValueError:
                continue
            if (
                resolved.is_file()
                and resolved.suffix.lower() in CALIBRATION_IMAGE_SUFFIXES
            ):
                selected = resolved
                break
        if selected is None:
            errors.append(text)
        elif selected not in valid:
            valid.append(selected)
    return valid, tuple(errors)


def portable_calibration_selection_paths(
    root: Path,
    paths: Sequence[Path],
) -> list[str]:
    return [calibration_selection_path_text(root, Path(path)) for path in paths]


__all__ = [
    "CALIBRATION_IMAGE_SUFFIXES",
    "CALIBRATION_SELECTION_SCHEMA",
    "CALIBRATION_SELECTION_STATES",
    "CalibrationSelectionDocument",
    "LEGACY_CALIBRATION_SELECTION_SCHEMA",
    "calibration_selection_path_text",
    "empty_calibration_selection",
    "load_calibration_selection_sidecar",
    "portable_calibration_selection_paths",
    "resolve_calibration_selection_paths",
]
