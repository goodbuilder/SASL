"""Immutable identity for one Workstation checkerboard solver run."""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Mapping, Sequence

from Workstation.workstation_core.strict_jsonl import sha256_file as _sha256_file


CALIBRATION_INPUT_SNAPSHOT_SCHEMA = "workstation-calibration-input-snapshot-v1"


def _canonical_bytes(payload: Mapping[str, object]) -> bytes:
    return json.dumps(
        dict(payload),
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


@dataclass(frozen=True)
class CalibrationInputFile:
    relative_path: str
    size_bytes: int
    sha256: str

    @classmethod
    def from_path(cls, root: Path, path: Path) -> "CalibrationInputFile":
        resolved_root = Path(root).resolve(strict=True)
        resolved = Path(path).resolve(strict=True)
        relative = resolved.relative_to(resolved_root)
        if not resolved.is_file():
            raise ValueError(f"calibration input is not a regular file: {resolved}")
        return cls(relative.as_posix(), resolved.stat().st_size, _sha256_file(resolved))

    def __post_init__(self) -> None:
        path = Path(self.relative_path)
        if path.is_absolute() or not path.parts or ".." in path.parts:
            raise ValueError("calibration input relative_path must stay below its root")
        if int(self.size_bytes) <= 0:
            raise ValueError("calibration input file must be non-empty")
        if len(self.sha256) != 64 or any(
            character not in "0123456789abcdef" for character in self.sha256
        ):
            raise ValueError("calibration input sha256 must be lowercase hex")


@dataclass(frozen=True)
class CalibrationInputPair:
    pair_key: str
    left_relative_path: str
    right_relative_path: str


@dataclass(frozen=True)
class CalibrationInputSnapshot:
    root: str
    mode: str
    left_files: tuple[CalibrationInputFile, ...]
    right_files: tuple[CalibrationInputFile, ...]
    stereo_pairs: tuple[CalibrationInputPair, ...]
    included_pair_keys: tuple[str, ...]
    excluded_pair_keys: tuple[str, ...]
    excluded_left_paths: tuple[str, ...]
    excluded_right_paths: tuple[str, ...]
    selection_policy: str
    pair_limit: int
    sidecar_state: str
    sidecar_schema: str
    sidecar_sha256: str
    zoom_evidence: tuple[CalibrationInputFile, ...] = ()
    timing_evidence_classes: tuple[str, ...] = ()
    precision_timing_qualified: bool = False
    actual_exposure_verified: bool = False
    schema: str = CALIBRATION_INPUT_SNAPSHOT_SCHEMA

    def __post_init__(self) -> None:
        if self.schema != CALIBRATION_INPUT_SNAPSHOT_SCHEMA:
            raise ValueError("unsupported calibration input snapshot schema")
        root = Path(self.root)
        if not root.is_absolute():
            raise ValueError("calibration input snapshot root must be resolved absolute path")
        if self.mode not in {"pixhawk", "checkerboard"}:
            raise ValueError("calibration input snapshot mode is invalid")
        if self.selection_policy not in {"automatic_even_spread", "manual_explicit"}:
            raise ValueError("calibration input selection policy is invalid")
        if int(self.pair_limit) < 0:
            raise ValueError("calibration input pair_limit must be non-negative")
        left = {item.relative_path for item in self.left_files}
        right = {item.relative_path for item in self.right_files}
        if len(left) != len(self.left_files) or len(right) != len(self.right_files):
            raise ValueError("calibration input files must be unique per side")
        pair_keys: set[str] = set()
        for pair in self.stereo_pairs:
            if not pair.pair_key or pair.pair_key in pair_keys:
                raise ValueError("calibration stereo pair identity must be non-empty and unique")
            pair_keys.add(pair.pair_key)
            if pair.left_relative_path not in left or pair.right_relative_path not in right:
                raise ValueError("calibration pair must reference snapshotted left/right files")
        if self.mode == "checkerboard" and len(self.stereo_pairs) < 3:
            raise ValueError("checkerboard input snapshot requires at least three stereo pairs")
        if self.sidecar_state not in {"ABSENT", "VALID"}:
            raise ValueError("invalid selection sidecar cannot produce an input snapshot")
        if self.sidecar_sha256 and (
            len(self.sidecar_sha256) != 64
            or any(character not in "0123456789abcdef" for character in self.sidecar_sha256)
        ):
            raise ValueError("selection sidecar sha256 must be lowercase hex")
        if self.actual_exposure_verified and not self.precision_timing_qualified:
            raise ValueError("exposure verification requires precision timing qualification")
        if (
            {"HOST_DISPATCH_PROXY", "HOST_RECEIVE_PROXY"}
            & set(self.timing_evidence_classes)
        ) and (
            self.precision_timing_qualified or self.actual_exposure_verified
        ):
            raise ValueError("host timing proxy cannot precision-qualify a snapshot")

    @property
    def root_path(self) -> Path:
        return Path(self.root)

    @property
    def left_paths(self) -> tuple[Path, ...]:
        return tuple(self.root_path / item.relative_path for item in self.left_files)

    @property
    def right_paths(self) -> tuple[Path, ...]:
        return tuple(self.root_path / item.relative_path for item in self.right_files)

    @property
    def pair_paths(self) -> tuple[tuple[str, Path, Path], ...]:
        return tuple(
            (
                pair.pair_key,
                self.root_path / pair.left_relative_path,
                self.root_path / pair.right_relative_path,
            )
            for pair in self.stereo_pairs
        )

    def verify_files(self) -> None:
        """Recheck exact bytes at the worker boundary before OpenCV reads them."""

        for item in (*self.left_files, *self.right_files, *self.zoom_evidence):
            path = (self.root_path / item.relative_path).resolve(strict=True)
            path.relative_to(self.root_path)
            if not path.is_file() or path.stat().st_size != item.size_bytes:
                raise ValueError(f"calibration input size changed after snapshot: {item.relative_path}")
            if _sha256_file(path) != item.sha256:
                raise ValueError(f"calibration input hash changed after snapshot: {item.relative_path}")
        sidecar = self.root_path / "calibration_pair_selection.json"
        if self.sidecar_state == "VALID":
            if not sidecar.is_file() or _sha256_file(sidecar) != self.sidecar_sha256:
                raise ValueError("calibration selection sidecar changed after snapshot")

    def canonical_payload(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "root": self.root,
            "mode": self.mode,
            "left_files": [asdict(item) for item in self.left_files],
            "right_files": [asdict(item) for item in self.right_files],
            "stereo_pairs": [asdict(item) for item in self.stereo_pairs],
            "included_pair_keys": list(self.included_pair_keys),
            "excluded_pair_keys": list(self.excluded_pair_keys),
            "excluded_left_paths": list(self.excluded_left_paths),
            "excluded_right_paths": list(self.excluded_right_paths),
            "selection_policy": self.selection_policy,
            "pair_limit": int(self.pair_limit),
            "sidecar_state": self.sidecar_state,
            "sidecar_schema": self.sidecar_schema,
            "sidecar_sha256": self.sidecar_sha256,
            "zoom_evidence": [asdict(item) for item in self.zoom_evidence],
            "timing_evidence_classes": list(self.timing_evidence_classes),
            "precision_timing_qualified": self.precision_timing_qualified,
            "actual_exposure_verified": self.actual_exposure_verified,
        }

    @property
    def snapshot_sha256(self) -> str:
        return hashlib.sha256(_canonical_bytes(self.canonical_payload())).hexdigest()

    def to_mapping(self) -> dict[str, object]:
        return {**self.canonical_payload(), "snapshot_sha256": self.snapshot_sha256}

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> "CalibrationInputSnapshot":
        def files(name: str) -> tuple[CalibrationInputFile, ...]:
            raw = payload.get(name)
            if not isinstance(raw, (list, tuple)):
                raise ValueError(f"{name} must be a list")
            return tuple(CalibrationInputFile(**dict(item)) for item in raw)

        raw_pairs = payload.get("stereo_pairs")
        if not isinstance(raw_pairs, (list, tuple)):
            raise ValueError("stereo_pairs must be a list")
        snapshot = cls(
            schema=str(payload.get("schema") or ""),
            root=str(payload.get("root") or ""),
            mode=str(payload.get("mode") or ""),
            left_files=files("left_files"),
            right_files=files("right_files"),
            stereo_pairs=tuple(
                CalibrationInputPair(**dict(item)) for item in raw_pairs
            ),
            included_pair_keys=tuple(
                str(value) for value in payload.get("included_pair_keys", ()) or ()
            ),
            excluded_pair_keys=tuple(
                str(value) for value in payload.get("excluded_pair_keys", ()) or ()
            ),
            excluded_left_paths=tuple(
                str(value) for value in payload.get("excluded_left_paths", ()) or ()
            ),
            excluded_right_paths=tuple(
                str(value) for value in payload.get("excluded_right_paths", ()) or ()
            ),
            selection_policy=str(payload.get("selection_policy") or ""),
            pair_limit=int(payload.get("pair_limit") or 0),
            sidecar_state=str(payload.get("sidecar_state") or ""),
            sidecar_schema=str(payload.get("sidecar_schema") or ""),
            sidecar_sha256=str(payload.get("sidecar_sha256") or ""),
            zoom_evidence=files("zoom_evidence"),
            timing_evidence_classes=tuple(
                str(value) for value in payload.get("timing_evidence_classes", ()) or ()
            ),
            precision_timing_qualified=bool(
                payload.get("precision_timing_qualified")
            ),
            actual_exposure_verified=bool(payload.get("actual_exposure_verified")),
        )
        supplied = str(payload.get("snapshot_sha256") or "")
        if supplied and supplied != snapshot.snapshot_sha256:
            raise ValueError("calibration input snapshot hash mismatch")
        return snapshot


def build_calibration_input_snapshot(
    *,
    root: Path,
    mode: str,
    left_paths: Sequence[Path],
    right_paths: Sequence[Path],
    stereo_pairs: Sequence[tuple[str, Path, Path]],
    included_pair_keys: Sequence[str],
    excluded_pair_keys: Sequence[str],
    excluded_left_paths: Sequence[Path],
    excluded_right_paths: Sequence[Path],
    selection_policy: str,
    pair_limit: int,
    sidecar_state: str,
    sidecar_schema: str,
    sidecar_path: Path,
) -> CalibrationInputSnapshot:
    resolved_root = Path(root).resolve(strict=True)
    left_files = tuple(CalibrationInputFile.from_path(resolved_root, path) for path in left_paths)
    right_files = tuple(CalibrationInputFile.from_path(resolved_root, path) for path in right_paths)
    left_relative = {str(Path(path).resolve(strict=True)): item.relative_path for path, item in zip(left_paths, left_files, strict=True)}
    right_relative = {str(Path(path).resolve(strict=True)): item.relative_path for path, item in zip(right_paths, right_files, strict=True)}
    pairs = tuple(
        CalibrationInputPair(
            str(key),
            left_relative[str(Path(left).resolve(strict=True))],
            right_relative[str(Path(right).resolve(strict=True))],
        )
        for key, left, right in stereo_pairs
    )
    capture_keys = {
        path.stem.split("_", 1)[1]
        for path in (*left_paths, *right_paths)
        if "_" in path.stem
    }
    zoom_paths = [
        resolved_root / "capture_metadata" / f"capture_{key}.json"
        for key in sorted(capture_keys)
    ]
    zoom_evidence = tuple(
        CalibrationInputFile.from_path(resolved_root, path)
        for path in zoom_paths
        if path.is_file()
    )
    stereo_metadata_paths: list[Path] = []
    for key, left_path, right_path in stereo_pairs:
        left_key = Path(left_path).stem.removeprefix("left_")
        right_key = Path(right_path).stem.removeprefix("right_")
        capture_key = left_key if left_key and left_key == right_key else str(key)
        stereo_metadata_paths.append(
            resolved_root / "capture_metadata" / f"capture_{capture_key}.json"
        )
    timing_rows: list[dict[str, object]] = []
    for path in stereo_metadata_paths:
        if not path.is_file():
            continue
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, TypeError, ValueError):
            payload = {}
        evidence = payload.get("calibration_timing_evidence") if isinstance(payload, dict) else None
        if isinstance(evidence, Mapping):
            timing_rows.append(dict(evidence))
            continue
        basis = str(payload.get("pair_time_basis") or "") if isinstance(payload, dict) else ""
        timing_rows.append(
            {
                "evidence_class": (
                    "HOST_DISPATCH_PROXY"
                    if basis.startswith("ftp-") and "dispatch" in basis
                    else "HOST_RECEIVE_PROXY"
                    if basis == "app-receive-monotonic"
                    else "CAMERA_TIMESTAMP_VERIFIED"
                    if basis == "rtcp-ntp"
                    else "UNKNOWN"
                ),
                "precision_qualified": bool(
                    basis == "rtcp-ntp" and payload.get("stereo_pair_saved")
                ) if isinstance(payload, dict) else False,
                "actual_exposure_verified": False,
            }
        )
    timing_classes = tuple(
        sorted({str(row.get("evidence_class") or "UNKNOWN") for row in timing_rows})
    )
    expected_timing_rows = len(stereo_pairs)
    precision_timing_qualified = bool(
        mode == "checkerboard"
        and expected_timing_rows > 0
        and len(timing_rows) == expected_timing_rows
        and all(bool(row.get("precision_qualified")) for row in timing_rows)
    )
    actual_exposure_verified = bool(
        precision_timing_qualified
        and all(bool(row.get("actual_exposure_verified")) for row in timing_rows)
    )
    sidecar = Path(sidecar_path)
    sidecar_sha256 = _sha256_file(sidecar) if sidecar.is_file() else ""
    return CalibrationInputSnapshot(
        root=str(resolved_root),
        mode=str(mode),
        left_files=left_files,
        right_files=right_files,
        stereo_pairs=pairs,
        included_pair_keys=tuple(str(value) for value in included_pair_keys),
        excluded_pair_keys=tuple(str(value) for value in excluded_pair_keys),
        excluded_left_paths=tuple(
            Path(path).resolve(strict=True).relative_to(resolved_root).as_posix()
            for path in excluded_left_paths
        ),
        excluded_right_paths=tuple(
            Path(path).resolve(strict=True).relative_to(resolved_root).as_posix()
            for path in excluded_right_paths
        ),
        selection_policy=str(selection_policy),
        pair_limit=int(pair_limit),
        sidecar_state=str(sidecar_state),
        sidecar_schema=str(sidecar_schema),
        sidecar_sha256=sidecar_sha256,
        zoom_evidence=zoom_evidence,
        timing_evidence_classes=timing_classes,
        precision_timing_qualified=precision_timing_qualified,
        actual_exposure_verified=actual_exposure_verified,
    )


__all__ = [
    "CALIBRATION_INPUT_SNAPSHOT_SCHEMA",
    "CalibrationInputFile",
    "CalibrationInputPair",
    "CalibrationInputSnapshot",
    "build_calibration_input_snapshot",
]
