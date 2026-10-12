"""Calibration file selection and hash snapshots, independent of Qt and live UI.

Per-camera K,D membership stays independent; stereo R,t additionally applies the
existing timing eligibility contract. No input root is read from a live widget.
"""
from __future__ import annotations
from pathlib import Path
import json
from typing import Mapping
from Workstation.workstation_core.calibration import StereoIntrinsicsResult
from Workstation.workstation_core.calibration_input import CalibrationInputSnapshot, build_calibration_input_snapshot
from Workstation.workstation_core.calibration_timing import load_calibration_pair_usage
from Workstation.workstation_core.calibration_selection import CALIBRATION_IMAGE_SUFFIXES, load_calibration_selection_sidecar, resolve_calibration_selection_paths


class CalibrationInputs:
    def __init__(self, report_errors=lambda _errors: None):
        self.report_errors = report_errors

    def _calibration_image_paths_for_root(self, root: Path, side: str) -> list[Path]:
        directory = root / side
        if not directory.is_dir():
            return []
        return sorted(
            path
            for path in directory.iterdir()
            if path.is_file() and path.suffix.lower() in CALIBRATION_IMAGE_SUFFIXES
        )


    def _calibration_stereo_pair_paths_for_root(self, root: Path) -> list[tuple[str, Path, Path]]:
        def indexed(paths: list[Path], side: str) -> dict[str, Path]:
            result: dict[str, Path] = {}
            for path in paths:
                suffix = path.stem.replace(f"{side}_", "", 1)
                result[suffix] = path
            return result

        left_by_id = indexed(self._calibration_image_paths_for_root(root, "left"), "left")
        right_by_id = indexed(self._calibration_image_paths_for_root(root, "right"), "right")
        common_keys = sorted(set(left_by_id) & set(right_by_id))
        if common_keys:
            return [
                (key, left_by_id[key], right_by_id[key])
                for key in common_keys
                if self._calibration_pair_stereo_eligible(
                    root,
                    left_by_id[key],
                    right_by_id[key],
                )
            ]
        left_paths = self._calibration_image_paths_for_root(root, "left")
        right_paths = self._calibration_image_paths_for_root(root, "right")
        if len(left_paths) == len(right_paths):
            return [
                (f"ordered-{index:06d}", left, right)
                for index, (left, right) in enumerate(zip(left_paths, right_paths, strict=False))
                if self._calibration_pair_stereo_eligible(root, left, right)
            ]
        return []


    @staticmethod
    def _calibration_pair_stereo_eligible(
        root: Path,
        left_path: Path,
        right_path: Path,
    ) -> bool:
        left_key = left_path.stem.removeprefix("left_")
        right_key = right_path.stem.removeprefix("right_")
        left_usage = load_calibration_pair_usage(
            Path(root) / "capture_metadata" / f"capture_{left_key}.json"
        )
        if not left_usage.stereo_eligible:
            return False
        if left_key == right_key:
            return True
        right_usage = load_calibration_pair_usage(
            Path(root) / "capture_metadata" / f"capture_{right_key}.json"
        )
        return right_usage.stereo_eligible


    @staticmethod
    def _evenly_sampled_calibration_items(candidates: list, limit: int) -> list:
        """Evenly sample a deterministic, bounded subset of ``candidates``.

        A deterministic spread keeps early/late items (capture pairs or
        single-camera files) represented instead of silently taking the first
        N, while a non-positive or oversized limit returns everything. Shared
        by ``_select_calibration_pairs`` and ``_select_calibration_image_paths``,
        which differ only in the element type being sampled.
        """

        items = list(candidates)
        if limit <= 0 or limit >= len(items):
            return items
        limit = max(3, limit)
        if limit >= len(items):
            return items
        indices = sorted(
            {
                round(index * (len(items) - 1) / (limit - 1))
                for index in range(limit)
            }
        )
        return [items[index] for index in indices]


    def _select_calibration_pairs(
        self,
        pairs: list[tuple[str, Path, Path]],
        *,
        pair_limit: int | None = None,
    ) -> list[tuple[str, Path, Path]]:
        """Select evenly distributed capture pairs for calibration input."""

        limit = (
            0
            if pair_limit is None
            else max(0, int(pair_limit))
        )
        return self._evenly_sampled_calibration_items(pairs, limit)


    @staticmethod
    def _select_calibration_image_paths(
        paths: list[Path],
        limit: int,
    ) -> list[Path]:
        """Evenly sample one camera's files when no stereo IDs are available."""

        return CalibrationInputs._evenly_sampled_calibration_items(paths, limit)


    @staticmethod
    def _resolve_calibration_selection_paths(
        raw_paths: object,
        root: Path,
        *,
        stored_root: object = None,
    ) -> list[Path]:
        paths, _errors = resolve_calibration_selection_paths(
            raw_paths,
            root,
            stored_root=stored_root,
        )
        return paths


    def _selected_calibration_file_lists(
        self,
        root: Path,
        fallback_left: list[Path],
        fallback_right: list[Path],
        *,
        selection: Mapping[str, object] | None = None,
    ) -> tuple[list[Path], list[Path], bool]:
        selection = (
            dict(selection or {})
        )
        if str(selection.get("_sidecar_state") or "") == "INVALID":
            errors = tuple(str(value) for value in selection.get("_sidecar_errors") or ())
            self.report_errors(errors)
            return [], [], True
        stored_root = selection.get("root")
        excluded_left_paths, excluded_left_errors = resolve_calibration_selection_paths(
            selection.get("excluded_left_paths"), root, stored_root=stored_root
        )
        excluded_right_paths, excluded_right_errors = resolve_calibration_selection_paths(
            selection.get("excluded_right_paths"), root, stored_root=stored_root
        )
        left_paths, left_errors = resolve_calibration_selection_paths(
            selection.get("left_paths"), root, stored_root=stored_root
        )
        right_paths, right_errors = resolve_calibration_selection_paths(
            selection.get("right_paths"), root, stored_root=stored_root
        )
        errors = (
            *excluded_left_errors,
            *excluded_right_errors,
            *left_errors,
            *right_errors,
        )
        if errors:
            self.report_errors(errors)
            return [], [], True
        excluded_left = set(excluded_left_paths)
        excluded_right = set(excluded_right_paths)
        if not selection.get("manual_mode"):
            return (
                [path for path in fallback_left if path not in excluded_left],
                [path for path in fallback_right if path not in excluded_right],
                False,
            )
        if not left_paths and not right_paths:
            included = {str(key) for key in selection.get("included_keys") or ()}
            if included:
                left_paths = [
                    path
                    for path in fallback_left
                    if path.stem.removeprefix("left_") in included
                ]
                right_paths = [
                    path
                    for path in fallback_right
                    if path.stem.removeprefix("right_") in included
                ]
            else:
                self.report_errors(
                    ("manual selection contains no recoverable files or pair keys",)
                )
                return [], [], True
        left = [path for path in left_paths if path not in excluded_left]
        right = [path for path in right_paths if path not in excluded_right]
        return left, right, True


    @staticmethod
    def _selected_calibration_pairs(
        left_paths: list[Path],
        right_paths: list[Path],
    ) -> list[tuple[str, Path, Path]]:
        """Pair explicit selections by row, with key matching for unequal lists."""

        def key(path: Path, side: str) -> str:
            return path.stem.removeprefix(f"{side}_")

        right_by_key: dict[str, list[Path]] = {}
        for path in right_paths:
            right_by_key.setdefault(key(path, "right"), []).append(path)
        common_pairs: list[tuple[str, Path, Path]] = []
        for left in left_paths:
            capture_key = key(left, "left")
            candidates = right_by_key.get(capture_key) or []
            if candidates:
                common_pairs.append((capture_key, left, candidates.pop(0)))
        if common_pairs:
            return common_pairs
        if len(left_paths) == len(right_paths):
            return [
                (f"selected-{index:04d}", left, right)
                for index, (left, right) in enumerate(zip(left_paths, right_paths, strict=False))
            ]
        # No shared capture keys and unequal list lengths: the key-matching
        # pass above (which produced the empty ``common_pairs``) is the only
        # matching strategy available here, so there is nothing left to pair.
        return common_pairs


    def _effective_calibration_inputs(
        self,
        root: Path,
        fallback_left: list[Path],
        fallback_right: list[Path],
        scanned_pairs: list[tuple[str, Path, Path]],
        *,
        mode: str,
        selection: Mapping[str, object] | None = None,
        pair_limit: int | None = None,
    ) -> tuple[list[Path], list[Path], list[tuple[str, Path, Path]], bool]:
        """Resolve one immutable calculation input set from the selection sidecar.

        Intrinsic selection belongs to each camera independently. Stereo R,t
        receives only ordered pairs whose left and right files both survived
        those independent selections. This prevents excluding a left outlier
        from silently deleting its otherwise useful right-camera observation.
        """

        left_paths, right_paths, explicit_selection = self._selected_calibration_file_lists(
            root,
            fallback_left,
            fallback_right,
            selection=selection,
        )
        left_members = set(left_paths)
        right_members = set(right_paths)
        pair_candidates = [
            pair
            for pair in scanned_pairs
            if pair[1] in left_members and pair[2] in right_members
        ]
        # ``scanned_pairs`` is already stereo-eligibility filtered by
        # ``_calibration_stereo_pair_paths_for_root`` (both its common-key and
        # ordered-fallback branches apply ``_calibration_pair_stereo_eligible``),
        # so ``pair_candidates`` -- a subset of it -- needs no second pass here.
        selected_pairs = self._choose_calibration_pairs(
            pair_candidates,
            selection=selection,
            root=root,
            pair_limit=pair_limit,
        )
        return left_paths, right_paths, selected_pairs, explicit_selection


    @staticmethod
    def _solver_effective_selection(
        result: StereoIntrinsicsResult,
    ) -> tuple[tuple[Path, ...], tuple[Path, ...], tuple[tuple[str, Path, Path], ...]] | None:
        """Read the exact post-deduplication input set emitted by the solver."""

        diagnostics = result.solver_diagnostics
        if not isinstance(diagnostics, dict):
            return None
        payload = diagnostics.get("effective_selection")
        if not isinstance(payload, dict):
            return None
        raw_left = payload.get("left_intrinsic_paths")
        raw_right = payload.get("right_intrinsic_paths")
        raw_pairs = payload.get("stereo_pairs")
        if not isinstance(raw_left, (list, tuple)) or not isinstance(
            raw_right, (list, tuple)
        ) or not isinstance(raw_pairs, (list, tuple)):
            return None
        pairs: list[tuple[str, Path, Path]] = []
        for row in raw_pairs:
            if not isinstance(row, (list, tuple)) or len(row) != 3:
                return None
            pairs.append((str(row[0]), Path(str(row[1])), Path(str(row[2]))))
        return (
            tuple(Path(str(path)) for path in raw_left),
            tuple(Path(str(path)) for path in raw_right),
            tuple(pairs),
        )


    def _choose_calibration_pairs(
        self,
        pairs: list[tuple[str, Path, Path]],
        *,
        selection: Mapping[str, object] | None = None,
        root: Path | None = None,
        pair_limit: int | None = None,
    ) -> list[tuple[str, Path, Path]]:
        """Manual picker override of ``_select_calibration_pairs``.

        Falls back to the existing even-distribution auto-selection unless
        the operator has switched on manual selection in the picker (which
        writes ``calibration_pair_selection.json`` -- the picker lives on the
        visible workspace panel, calculation runs on this hidden one, so a
        JSON sidecar is the handoff instead of a cross-panel command).
        """

        selection = (
            dict(selection or {})
        )
        if str(selection.get("_sidecar_state") or "") == "INVALID":
            errors = tuple(str(value) for value in selection.get("_sidecar_errors") or ())
            self.report_errors(errors)
            return []
        root = Path(root)
        stored_root = selection.get("root")
        explicit_left, left_errors = resolve_calibration_selection_paths(
            selection.get("left_paths"),
            root,
            stored_root=stored_root,
        )
        explicit_right, right_errors = resolve_calibration_selection_paths(
            selection.get("right_paths"),
            root,
            stored_root=stored_root,
        )
        excluded_left_paths, excluded_left_errors = resolve_calibration_selection_paths(
            selection.get("excluded_left_paths"), root, stored_root=stored_root
        )
        excluded_right_paths, excluded_right_errors = resolve_calibration_selection_paths(
            selection.get("excluded_right_paths"), root, stored_root=stored_root
        )
        errors = (
            *left_errors,
            *right_errors,
            *excluded_left_errors,
            *excluded_right_errors,
        )
        if errors:
            self.report_errors(errors)
            return []
        manual_mode = bool(selection.get("manual_mode"))
        if manual_mode and (explicit_left or explicit_right):
            candidates = self._selected_calibration_pairs(explicit_left, explicit_right)
        elif not manual_mode:
            candidates = list(pairs)
        else:
            included = set(selection.get("included_keys") or ())
            candidates = [pair for pair in pairs if pair[0] in included]
        excluded_left = set(excluded_left_paths)
        excluded_right = set(excluded_right_paths)
        excluded_keys = {str(key) for key in selection.get("excluded_pair_keys") or ()}
        # ``candidates`` from the manual-explicit-paths branch above is
        # rebuilt straight from the operator's raw left/right selection via
        # ``_selected_calibration_pairs`` -- it never consults ``pairs``
        # (already stereo-eligibility filtered by
        # ``_calibration_stereo_pair_paths_for_root``), so an
        # intrinsics-only/timing-rejected capture the operator explicitly
        # kept for K,D purposes could otherwise slip back into the stereo
        # pair set here. Re-checking eligibility is a no-op for pairs that
        # came from the already-filtered ``pairs`` list.
        eligible = [
            pair
            for pair in candidates
            if pair[0] not in excluded_keys
            and pair[1] not in excluded_left
            and pair[2] not in excluded_right
            and self._calibration_pair_stereo_eligible(root, pair[1], pair[2])
        ]
        # ``calibration_calculation_pair_count`` is a stereo-pair limit. Apply
        # it once, after exclusions and side membership have established the
        # usable intersection. Sampling before that intersection can discard
        # many otherwise valid pairs when only one camera has an exclusion.
        return (
            self._select_calibration_pairs(eligible, pair_limit=pair_limit)
            if not manual_mode
            else eligible
        )


    def snapshot(self, root: Path, *, mode: str, pair_limit: int):
        if mode not in {"pixhawk", "checkerboard"}:
            raise ValueError(f"unsupported calibration calculation mode: {mode}")
        fallback_left = self._calibration_image_paths_for_root(root, "left")
        fallback_right = self._calibration_image_paths_for_root(root, "right")
        scanned_pairs = self._calibration_stereo_pair_paths_for_root(root)
        sidecar_path = root / "calibration_pair_selection.json"
        document = load_calibration_selection_sidecar(sidecar_path)
        selection = document.to_mapping()
        if document.state == "INVALID":
            self.report_errors(document.errors)
            raise ValueError("; ".join(document.errors))
        left_paths, right_paths, selected_pairs, explicit = (
            self._effective_calibration_inputs(
                root,
                fallback_left,
                fallback_right,
                scanned_pairs,
                mode=mode,
                selection=selection,
                pair_limit=pair_limit,
            )
        )
        if not left_paths or not right_paths:
            raise ValueError(
                f"left/right calibration images are required: left={len(left_paths)}, "
                f"right={len(right_paths)}"
            )
        if mode == "checkerboard" and len(selected_pairs) < 3:
            raise ValueError(
                f"checkerboard calibration requires at least 3 stereo pairs; "
                f"found {len(selected_pairs)}"
            )
        stored_root = selection.get("root")
        excluded_left, excluded_left_errors = resolve_calibration_selection_paths(
            selection.get("excluded_left_paths"), root, stored_root=stored_root
        )
        excluded_right, excluded_right_errors = resolve_calibration_selection_paths(
            selection.get("excluded_right_paths"), root, stored_root=stored_root
        )
        if excluded_left_errors or excluded_right_errors:
            errors = (*excluded_left_errors, *excluded_right_errors)
            self.report_errors(errors)
            raise ValueError("; ".join(errors))
        snapshot = build_calibration_input_snapshot(
            root=root,
            mode=mode,
            left_paths=left_paths,
            right_paths=right_paths,
            stereo_pairs=selected_pairs,
            included_pair_keys=tuple(selection.get("included_keys") or ()),
            excluded_pair_keys=tuple(selection.get("excluded_pair_keys") or ()),
            excluded_left_paths=excluded_left,
            excluded_right_paths=excluded_right,
            selection_policy=("manual_explicit" if explicit else "automatic_even_spread"),
            pair_limit=pair_limit,
            sidecar_state=document.state,
            sidecar_schema=str(selection.get("schema") or ""),
            sidecar_path=sidecar_path,
        )
        return snapshot, len(scanned_pairs), explicit




    @staticmethod
    def _zoom_index(value: object) -> int | None:
        try:
            level = int(value)
        except (TypeError, ValueError):
            return None
        return level if level >= 0 else None


    def _capture_zoom_level_for_paths(
        self,
        root: Path,
        paths: list[Path] | tuple[Path, ...],
    ) -> tuple[int | None, str]:
        """Resolve one common zoom from the capture metadata selected for a run."""

        capture_ids = {
            path.stem.split("_", 1)[1]
            for path in paths
            if "_" in path.stem
        }
        if not capture_ids:
            return None, "capture IDs are missing from the calibration image names"
        levels: dict[str, int] = {}
        missing: list[str] = []
        metadata_root = root / "capture_metadata"
        for capture_id in sorted(capture_ids):
            metadata_path = metadata_root / f"capture_{capture_id}.json"
            try:
                payload = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                missing.append(capture_id)
                continue
            level = self._zoom_index(payload.get("capture_zoom_level"))
            if level is None:
                left_level = self._zoom_index(payload.get("capture_left_zoom_level"))
                right_level = self._zoom_index(payload.get("capture_right_zoom_level"))
                if left_level is not None and left_level == right_level:
                    level = left_level
            if level is None:
                missing.append(capture_id)
            else:
                levels[capture_id] = level
        if missing:
            return (
                None,
                "capture-time zoom readback is missing for "
                + ", ".join(missing[:5])
                + (" …" if len(missing) > 5 else ""),
            )
        unique_levels = sorted(set(levels.values()))
        if len(unique_levels) != 1:
            return None, f"mixed capture zoom levels: {', '.join(str(level) for level in unique_levels)}"
        return unique_levels[0], f"capture-time zoom={unique_levels[0]}"

