"""Single owner of automatic proposals, explicit acceptance and annotation provenance.

Native image u/v, source SHA/dimensions and candidate DOF meanings do not change.
The owner sees only frozen artifacts and emits already-decided evidence rows.
"""

from __future__ import annotations
import copy
import json
import math
import time
from typing import Any, Mapping
from Workstation.workstation_core.optical_pose_auto_select import (
    HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION,
    validate_horizon_suggestion_points,
    validate_mountain_suggestion_points,
)
from .observations import annotation_points, validate_annotation_identity
from .geometry_values import _payload_sha256


class OpticalReviewState:
    _MOUNTAIN_RIDGE_MARGIN_DEFAULT = 0.06
    _MOUNTAIN_RIDGE_MARGIN_MIN = 0.02
    _MOUNTAIN_RIDGE_MARGIN_MAX = 0.15
    _MOUNTAIN_MAX_POINTS_DEFAULT = 0
    _MOUNTAIN_MAX_POINTS_MIN = 4
    _MOUNTAIN_MAX_POINTS_MAX = 500
    _HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION = HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION

    def __init__(self, emit, persist_accepted):
        self._append_event = lambda name, value: emit(name, copy.deepcopy(value))
        self._persist_accepted_suggestion_alignment_row = lambda value: persist_accepted(copy.deepcopy(value))
        self._auto_suggestions = {}
        self._last_accepted_auto_suggestion = None
        self._annotation_review = None
        self._mountain_ridge_margin_fraction = self._MOUNTAIN_RIDGE_MARGIN_DEFAULT
        self._mountain_max_points = self._MOUNTAIN_MAX_POINTS_DEFAULT
        self._burst_generation = 0

    def reset_burst(self):
        self._burst_generation += 1
        self._auto_suggestions.clear()
        self._last_accepted_auto_suggestion = None
        self._annotation_review = None

    @staticmethod
    def _validate_annotation_identity(artifacts, side, annotation):
        validate_annotation_identity(artifacts.get(side), side, annotation)

    _annotation_points = staticmethod(annotation_points)

    def record(
        self,
        suggestion: Mapping[str, Any],
    ) -> dict[str, Any]:
        value = copy.deepcopy(dict(suggestion))
        suggestion_id = str(value.get("suggestion_id") or "")
        if not suggestion_id:
            raise ValueError("automatic suggestion_id is required")
        self._auto_suggestions[suggestion_id] = value
        self._append_event(
            "optical_pose_auto_suggestion_created",
            {
                "schema": "optical-pose-auto-suggestion-v1",
                "suggestion_id": suggestion_id,
                "mode": value.get("mode"),
                "state": value.get("state"),
                "detection_method": value.get("detection_method"),
                "confidence": value.get("confidence"),
                "inlier_count": value.get("inlier_count"),
                "residual_px": value.get("residual_px"),
                "source_frames": value.get("source_frames"),
                "input_hashes": value.get("input_hashes"),
                "failure_reason": value.get("failure_reason", ""),
                "eligible": value.get("eligible", False),
                "copy_eligible": value.get("copy_eligible", False),
                "copy_block_reason": value.get("copy_block_reason", ""),
                # Preserve the complete native-point proposal in the
                # append-only event, not only its review summary.
                "suggestion": value,
            },
        )
        return None

    def set_ridge_margin(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Set the Mountain ridge-boundary match margin for later Auto-select runs.

        A fraction of image height (see ``detect_mountain``'s
        ``ridge_margin_fraction``); does not itself re-run detection.
        """

        raw = payload.get("ridge_margin_fraction")
        try:
            fraction = float(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("ridge_margin_fraction must be a finite number") from exc
        if not math.isfinite(fraction):
            raise ValueError("ridge_margin_fraction must be a finite number")
        clamped = min(
            self._MOUNTAIN_RIDGE_MARGIN_MAX,
            max(self._MOUNTAIN_RIDGE_MARGIN_MIN, fraction),
        )
        self._mountain_ridge_margin_fraction = clamped
        return None

    def set_max_points(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Set how many spread correspondences Mountain/Cloud keep for later Auto-select runs.

        ``0`` keeps every RANSAC inlier (the default); a positive value caps
        the spread selection. See ``detect_mountain``'s ``max_points``; does
        not itself re-run detection.
        """

        raw = payload.get("max_points")
        try:
            count = int(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("max_points must be an integer") from exc
        if count <= 0:
            clamped = 0
        else:
            clamped = min(
                self._MOUNTAIN_MAX_POINTS_MAX,
                max(self._MOUNTAIN_MAX_POINTS_MIN, count),
            )
        self._mountain_max_points = clamped
        return None

    def clear(self, payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """Discard automatic Horizon/Mountain suggestions.

        Mirrors the UI's "Clear" action: an operator clearing the points
        they are looking at should also discard whatever automatic
        suggestion produced them, not just the manual overlay copy, or a
        stale suggestion keeps reappearing in the snapshot after the manual
        points are gone. With no ``mode`` (or an empty one) every suggestion
        is discarded; with a ``mode`` ("horizon" or "mountain") only that
        mode's suggestions are.
        """

        values = dict(payload or {})
        mode = str(values.get("mode") or "").strip().lower()
        if mode:
            # Mountain and Cloud suggestions share the ridge annotation, so
            # clearing either mode discards both.
            modes = {"mountain", "cloud"} if mode in {"mountain", "cloud"} else {mode}
            self._auto_suggestions = {
                suggestion_id: value
                for suggestion_id, value in self._auto_suggestions.items()
                if str(value.get("mode") or "") not in modes
            }
            if (
                self._last_accepted_auto_suggestion is not None
                and str(self._last_accepted_auto_suggestion.get("mode") or "") in modes
            ):
                self._last_accepted_auto_suggestion = None
        else:
            self._auto_suggestions.clear()
            self._last_accepted_auto_suggestion = None
        return None

    def commit(
        self,
        suggestion: Mapping[str, Any],
        *,
        artifacts,
    ) -> dict[str, Any]:
        """Commit a completed background result on the owner/UI thread only."""

        value = copy.deepcopy(dict(suggestion))
        source_frames = value.get("source_frames")
        if not isinstance(source_frames, Mapping):
            raise ValueError("auto-selection result is missing source frame identities")
        for side in ("left", "right"):
            artifact = artifacts.get(side)
            expected = source_frames.get(side)
            if not isinstance(expected, Mapping) or artifact is None:
                raise RuntimeError("auto-selection result is stale; capture a new frozen burst")
            if str(expected.get("frame_id") or "") != str(getattr(artifact, "frame_id", "")) or str(
                expected.get("image_sha256") or ""
            ) != str(getattr(artifact, "sha256", "")):
                raise RuntimeError("auto-selection result is stale; capture a new frozen burst")
        return self.record(value)

    @staticmethod
    def _append_auto_copy_quality_warning(
        accepted: dict[str, Any],
        *,
        mode_label: str,
    ) -> None:
        quality_state = str(accepted.get("state") or "UNKNOWN").upper()
        if quality_state == "GOOD":
            return
        warning = (
            f"operator copied a structurally usable {mode_label} suggestion with quality state "
            f"{quality_state}; detector confidence remains warning evidence"
        )
        raw_warnings = accepted.get("warnings")
        warnings = [str(value) for value in raw_warnings] if isinstance(raw_warnings, (tuple, list)) else []
        if warning not in warnings:
            warnings.append(warning)
        accepted["warnings"] = warnings

    def _validated_horizon_suggestion_for_acceptance(
        self,
        suggestion: Mapping[str, Any],
        *,
        artifacts,
    ) -> dict[str, Any]:
        """Return a current-frame, structurally validated Horizon copy payload."""

        sides = suggestion.get("sides")
        if not isinstance(sides, Mapping):
            raise ValueError("Horizon suggestion is missing left/right endpoints")
        parsed: dict[str, tuple[tuple[float, float], ...]] = {}
        image_sizes: dict[str, tuple[int, int]] = {}
        for side in ("left", "right"):
            row = sides.get(side)
            if not isinstance(row, Mapping):
                raise ValueError(f"Horizon suggestion is missing {side} endpoints")
            self._validate_annotation_identity(artifacts, side, row)
            annotation = dict(row)
            annotation["horizon_line"] = row.get("points", row.get("horizon_line", ()))
            parsed[side] = self._annotation_points(annotation, "horizon_line")
            image_sizes[side] = (
                int(annotation["source_width_px"]),
                int(annotation["source_height_px"]),
            )

        left_points, right_points = validate_horizon_suggestion_points(
            parsed["left"],
            parsed["right"],
            left_image_size=image_sizes["left"],
            right_image_size=image_sizes["right"],
            minimum_separation_fraction=self._HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION,
        )
        accepted = json.loads(json.dumps(suggestion, ensure_ascii=False))
        accepted_sides = accepted.get("sides")
        if not isinstance(accepted_sides, dict):
            raise ValueError("Horizon suggestion is missing left/right endpoints")
        for side, points in (("left", left_points), ("right", right_points)):
            row = accepted_sides.get(side)
            if not isinstance(row, dict):
                raise ValueError(f"Horizon suggestion is missing {side} endpoints")
            normalized = [{"u_px": u, "v_px": v} for u, v in points]
            row["points"] = normalized
            row["horizon_line"] = list(normalized)
            row["original_point_count"] = 2
            row["retained_point_count"] = 2
        accepted["copy_eligible"] = True
        accepted["copy_block_reason"] = ""
        accepted["original_point_count"] = 2
        accepted["retained_point_count"] = 2
        accepted["validated_input_sha256"] = _payload_sha256(
            {
                "coordinate_contract": "native OpenCV pixels; u right, v down",
                "left": accepted_sides["left"]["points"],
                "right": accepted_sides["right"]["points"],
            }
        )
        self._append_auto_copy_quality_warning(accepted, mode_label="Horizon")
        return accepted

    def _validated_mountain_suggestion_for_acceptance(
        self,
        suggestion: Mapping[str, Any],
        payload: Mapping[str, Any],
        *,
        artifacts,
    ) -> dict[str, Any]:
        """Return an immutable, structurally validated Mountain copy payload."""

        sides = suggestion.get("sides")
        if not isinstance(sides, Mapping):
            raise ValueError("Mountain suggestion is missing paired left/right points")
        parsed: dict[str, tuple[tuple[float, float], ...]] = {}
        for side in ("left", "right"):
            row = sides.get(side)
            if not isinstance(row, Mapping):
                raise ValueError(f"Mountain suggestion is missing {side} points")
            self._validate_annotation_identity(artifacts, side, row)
            annotation = dict(row)
            annotation["ridge_polyline"] = row.get("points", row.get("ridge_polyline", ()))
            parsed[side] = self._annotation_points(annotation, "ridge_polyline")

        raw_exclusions = payload.get("excluded_point_indices", ())
        if raw_exclusions is None:
            raw_exclusions = ()
        if not isinstance(raw_exclusions, (tuple, list)):
            raise ValueError("Mountain excluded point indices must be a list")
        left_points, right_points, excluded = validate_mountain_suggestion_points(
            parsed["left"],
            parsed["right"],
            excluded_indices=raw_exclusions,
        )

        accepted = json.loads(json.dumps(suggestion, ensure_ascii=False))
        accepted_sides = accepted.get("sides")
        if not isinstance(accepted_sides, dict):
            raise ValueError("Mountain suggestion is missing paired left/right points")
        excluded_set = set(excluded)
        original_count = len(parsed["left"])
        for side, points in (("left", left_points), ("right", right_points)):
            row = accepted_sides.get(side)
            if not isinstance(row, dict):
                raise ValueError(f"Mountain suggestion is missing {side} points")
            filtered = [{"u_px": u, "v_px": v} for u, v in points]
            row["points"] = filtered
            row["ridge_polyline"] = list(filtered)
            confidences = row.get("point_confidences")
            if isinstance(confidences, list) and len(confidences) == original_count:
                row["point_confidences"] = [
                    value for index, value in enumerate(confidences) if index not in excluded_set
                ]
            elif confidences is not None:
                row["point_confidences"] = []
            row["original_point_count"] = original_count
            row["retained_point_count"] = len(points)

        accepted["copy_eligible"] = True
        accepted["copy_block_reason"] = ""
        accepted["excluded_point_indices"] = list(excluded)
        accepted["original_point_count"] = original_count
        accepted["retained_point_count"] = len(left_points)
        accepted["filtered_input_sha256"] = _payload_sha256(
            {
                "coordinate_contract": "same-order native OpenCV pixels; u right, v down",
                "excluded_point_indices": list(excluded),
                "left": accepted_sides["left"]["points"],
                "right": accepted_sides["right"]["points"],
            }
        )
        self._append_auto_copy_quality_warning(accepted, mode_label="Mountain")
        return accepted

    def accept(
        self,
        payload: Mapping[str, Any],
        *,
        artifacts,
    ) -> dict[str, Any]:
        """Record explicit review approval; the UI then copies points locally."""

        generation = self._burst_generation
        suggestion_id = str(payload.get("suggestion_id") or "")
        if not suggestion_id:
            raise ValueError("Use suggestion requires an explicit suggestion_id")
        suggestion = self._auto_suggestions.get(suggestion_id)
        if suggestion is None:
            raise ValueError(f"unknown automatic optical-pose suggestion: {suggestion_id}")
        mode = str(suggestion.get("mode") or "").lower()
        if mode in {"mountain", "cloud"}:
            accepted = self._validated_mountain_suggestion_for_acceptance(suggestion, payload, artifacts=artifacts)
        elif mode == "horizon":
            accepted = self._validated_horizon_suggestion_for_acceptance(suggestion, artifacts=artifacts)
        else:
            raise ValueError(f"unsupported automatic optical-pose suggestion mode: {mode!r}")
        accepted["provenance_state"] = "AUTO_ACCEPTED"
        accepted["accepted_utc_ns"] = time.time_ns()
        self._last_accepted_auto_suggestion = accepted
        accepted_annotations = self._annotation_mappings_from_suggestion(accepted)
        if accepted_annotations:
            self._annotation_review = self._build_annotation_review(
                suggestion_id=suggestion_id,
                provenance_state="AUTO_ACCEPTED",
                annotation_mode=str(accepted.get("mode") or ""),
                annotations=accepted_annotations,
                artifacts=artifacts,
            )
        self._append_event(
            "optical_pose_auto_suggestion_accepted",
            {
                "schema": "optical-pose-auto-suggestion-acceptance-v1",
                "suggestion_id": suggestion_id,
                "mode": accepted.get("mode"),
                "provenance_state": "AUTO_ACCEPTED",
                "source_frames": accepted.get("source_frames"),
                "input_hashes": accepted.get("input_hashes"),
                "suggestion": accepted,
                "annotation_review": self._annotation_review,
            },
        )
        if generation == self._burst_generation:
            self._persist_accepted_suggestion_alignment_row(accepted)
        return None

    @staticmethod
    def _annotation_mappings_from_suggestion(
        suggestion: Mapping[str, Any],
    ) -> dict[str, dict[str, Any]]:
        """Convert accepted suggestion sides to the existing annotation schema."""

        mode = str(suggestion.get("mode") or "").lower()
        annotation_key = {
            "horizon": "horizon_line",
            "mountain": "ridge_polyline",
            "cloud": "ridge_polyline",
        }.get(mode)
        if annotation_key is None:
            return {}
        sides = suggestion.get("sides")
        if not isinstance(sides, Mapping):
            return {}
        annotations: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            row = sides.get(side)
            if not isinstance(row, Mapping):
                continue
            points = row.get("points", row.get(annotation_key, ()))
            if not isinstance(points, (tuple, list)):
                points = []
            annotation: dict[str, Any] = {
                "frame_id": str(row.get("frame_id") or ""),
                "image_sha256": str(row.get("image_sha256") or ""),
                "source_width_px": row.get("source_width_px"),
                "source_height_px": row.get("source_height_px"),
                "horizon_line": [],
                "ridge_polyline": [],
                "target_point": [],
            }
            annotation[annotation_key] = list(points)
            annotations[side] = annotation
        return annotations

    def _build_annotation_review(
        self,
        *,
        suggestion_id: str,
        provenance_state: str,
        annotation_mode: str,
        annotations: Mapping[str, Mapping[str, Any]],
        artifacts,
    ) -> dict[str, Any]:
        normalized: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            annotation = annotations.get(side)
            if not isinstance(annotation, Mapping):
                continue
            self._validate_annotation_identity(artifacts, side, annotation)
            normalized[side] = json.loads(json.dumps(dict(annotation), ensure_ascii=False))
        if not normalized:
            raise ValueError("optical-pose annotation review requires a left or right annotation")
        return {
            "schema": "optical-pose-annotation-review-v1",
            "suggestion_id": str(suggestion_id or ""),
            "provenance_state": str(provenance_state or "MANUAL").upper(),
            "annotation_mode": str(annotation_mode or ""),
            "annotations": normalized,
            "annotation_hashes": {side: _payload_sha256(annotation) for side, annotation in normalized.items()},
            "updated_utc_ns": time.time_ns(),
        }

    def record_annotation(
        self,
        payload: Mapping[str, Any],
        *,
        artifacts,
    ) -> dict[str, Any]:
        """Persist the current committed annotation and its provenance state."""

        values = dict(payload or {})
        annotations = self._annotation_root(values)
        if not annotations:
            raise ValueError("optical-pose annotation update requires at least one annotation")
        provenance_state = str(values.get("provenance_state") or "MANUAL").upper()
        if provenance_state not in {"MANUAL", "AUTO_ACCEPTED", "MANUAL_EDITED"}:
            raise ValueError(f"unsupported optical-pose annotation provenance: {provenance_state}")
        review = self._build_annotation_review(
            suggestion_id=str(values.get("suggestion_id") or ""),
            provenance_state=provenance_state,
            annotation_mode=str(values.get("annotation_mode") or ""),
            annotations=annotations,
            artifacts=artifacts,
        )
        self._annotation_review = review
        self._append_event("optical_pose_annotations_updated", review)
        return review

    @staticmethod
    def _annotation_root(payload: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
        raw = payload.get("annotations")
        raw = raw if isinstance(raw, Mapping) else {}
        result: dict[str, Mapping[str, Any]] = {}
        for side in ("left", "right"):
            value = raw.get(side)
            if value is None:
                value = payload.get(f"{side}_annotation")
            to_mapping = getattr(value, "to_mapping", None)
            if callable(to_mapping):
                value = to_mapping()
            if isinstance(value, Mapping):
                result[side] = dict(value)
        return result
