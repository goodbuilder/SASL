"""Project selected intrinsic and checkerboard calibration provenance.

Calibration source identity and explicit optical-frame contracts are preserved."""
from __future__ import annotations
import json
from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
from .ports import CalibrationFieldsPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .formatting import Formatting


class CalibrationFields:
    def __init__(self, *,
        boundary: CalibrationFieldsPort,
    ) -> None:
        self.boundary = boundary

    def bind_collaborators(self, *,
        formatting: "Formatting",
    ) -> None:
        self.formatting = formatting

    def _record_current_intrinsic_parameters(self) -> dict[str, str]:
        intrinsics_source_zoom = getattr(
            self.boundary, "_active_intrinsics_source_zoom_level", None
        )
        extrinsics_source_zoom = getattr(
            self.boundary, "_active_extrinsics_source_zoom_level", None
        )
        row: dict[str, str] = {
            "intrinsics_active": "true" if self.boundary.active_intrinsics is not None else "false",
            "intrinsics_path": str(
                self.boundary._workstation_setting("intrinsics_path", self.boundary.settings.intrinsics_path)
            ),
            "intrinsics_zoom_level": str(self.boundary.zoom_level_spin.value()),
            "intrinsics_zoom_path": str(self.boundary._intrinsics_storage_path()),
            "intrinsics_latest_path": str(self.boundary._intrinsics_storage_base_path()),
            "intrinsics_profile_path": str(
                getattr(self.boundary, "_active_intrinsics_profile_path", "") or ""
            ),
            "intrinsics_profile_sha256": str(
                getattr(self.boundary, "_active_intrinsics_profile_hash", "") or ""
            ),
            "intrinsics_source_zoom_level": (
                "" if intrinsics_source_zoom is None else str(intrinsics_source_zoom)
            ),
            "extrinsics_profile_path": str(
                getattr(self.boundary, "_active_extrinsics_profile_path", "") or ""
            ),
            "extrinsics_profile_sha256": str(
                getattr(self.boundary, "_active_extrinsics_profile_hash", "") or ""
            ),
            "extrinsics_source_zoom_level": (
                "" if extrinsics_source_zoom is None else str(extrinsics_source_zoom)
            ),
            "calibration_selection_mode": str(
                getattr(self.boundary, "_active_calibration_selection_mode", "") or ""
            ),
            "calibration_validation_gate": "disabled",
            "cross_zoom_selection": self.formatting._fmt_record_bool(
                intrinsics_source_zoom is not None
                and extrinsics_source_zoom is not None
                and int(intrinsics_source_zoom) != int(extrinsics_source_zoom)
            ),
            "actual_visible_zoom": json.dumps(
                getattr(self.boundary, "actual_visible_zoom", {}) or {},
                separators=(",", ":"),
            ),
            "manual_focal_override": self.formatting._fmt_record_bool(self.boundary.manual_focal_checkbox.isChecked()),
            "runtime_focal_px": self.formatting._fmt_record_float(self.boundary.focal_spin.value(), 3),
        }
        if self.boundary.active_intrinsics is None:
            return row
        for prefix, intrinsics in (("left", self.boundary.active_intrinsics.left), ("right", self.boundary.active_intrinsics.right)):
            matrix = intrinsics.camera_matrix
            row.update(
                {
                    f"{prefix}_intr_width": str(intrinsics.width),
                    f"{prefix}_intr_height": str(intrinsics.height),
                    f"{prefix}_intr_fx": self.formatting._fmt_record_float(matrix[0][0], 6),
                    f"{prefix}_intr_fy": self.formatting._fmt_record_float(matrix[1][1], 6),
                    f"{prefix}_intr_cx": self.formatting._fmt_record_float(matrix[0][2], 6),
                    f"{prefix}_intr_cy": self.formatting._fmt_record_float(matrix[1][2], 6),
                    f"{prefix}_intr_distortion": json.dumps(intrinsics.distortion_coefficients, separators=(",", ":")),
                    f"{prefix}_intr_reprojection_error_px": self.formatting._fmt_record_float(
                        intrinsics.reprojection_error_px, 6
                    ),
                    f"{prefix}_intr_image_count": str(intrinsics.image_count),
                }
            )
        return row

    def _record_intrinsic_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, str]:
        intrinsics_snapshot = measurement_attempt.intrinsics
        row: dict[str, str] = {
            "intrinsics_active": self.formatting._fmt_record_bool(intrinsics_snapshot is not None),
            "intrinsics_path": "" if intrinsics_snapshot is None else intrinsics_snapshot.selected_profile_path,
            "intrinsics_zoom_level": "" if intrinsics_snapshot is None else str(intrinsics_snapshot.active_zoom_level),
            "intrinsics_zoom_path": "" if intrinsics_snapshot is None else intrinsics_snapshot.selected_profile_path,
            "intrinsics_latest_path": "" if intrinsics_snapshot is None else intrinsics_snapshot.selected_profile_path,
            "intrinsics_content_sha256": "" if intrinsics_snapshot is None else intrinsics_snapshot.content_sha256,
            "intrinsics_profile_sha256": (
                "" if intrinsics_snapshot is None else intrinsics_snapshot.intrinsics_content_sha256
            ),
            "intrinsics_source_zoom_level": (
                ""
                if intrinsics_snapshot is None
                else str(intrinsics_snapshot.intrinsics_source_zoom_level)
            ),
            "extrinsics_profile_path": (
                ""
                if intrinsics_snapshot is None
                else intrinsics_snapshot.selected_extrinsics_profile_path
            ),
            "extrinsics_profile_sha256": (
                ""
                if intrinsics_snapshot is None
                else intrinsics_snapshot.extrinsics_content_sha256
            ),
            "extrinsics_source_zoom_level": (
                ""
                if intrinsics_snapshot is None
                or intrinsics_snapshot.extrinsics_source_zoom_level is None
                else str(intrinsics_snapshot.extrinsics_source_zoom_level)
            ),
            "calibration_selection_mode": (
                "" if intrinsics_snapshot is None else intrinsics_snapshot.selection_mode
            ),
            "calibration_validation_gate": (
                "" if intrinsics_snapshot is None else intrinsics_snapshot.validation_gate
            ),
            "cross_zoom_selection": self.formatting._fmt_record_bool(
                intrinsics_snapshot is not None
                and intrinsics_snapshot.extrinsics_source_zoom_level is not None
                and intrinsics_snapshot.intrinsics_source_zoom_level
                != intrinsics_snapshot.extrinsics_source_zoom_level
            ),
            "manual_focal_override": self.formatting._fmt_record_bool(
                measurement_attempt.geometry_config.manual_focal_override
            ),
            "runtime_focal_px": self.formatting._fmt_record_float(
                measurement_attempt.geometry_config.focal_length_px, 3
            ),
        }
        if intrinsics_snapshot is None:
            return row
        for prefix, intrinsics in (
            ("left", intrinsics_snapshot.left),
            ("right", intrinsics_snapshot.right),
        ):
            matrix = intrinsics.camera_matrix
            row.update(
                {
                    f"{prefix}_intr_width": str(intrinsics.width),
                    f"{prefix}_intr_height": str(intrinsics.height),
                    f"{prefix}_intr_fx": self.formatting._fmt_record_float(matrix[0][0], 6),
                    f"{prefix}_intr_fy": self.formatting._fmt_record_float(matrix[1][1], 6),
                    f"{prefix}_intr_cx": self.formatting._fmt_record_float(matrix[0][2], 6),
                    f"{prefix}_intr_cy": self.formatting._fmt_record_float(matrix[1][2], 6),
                    f"{prefix}_intr_distortion": json.dumps(
                        intrinsics.distortion_coefficients, separators=(",", ":")
                    ),
                    f"{prefix}_intr_reprojection_error_px": self.formatting._fmt_record_float(
                        intrinsics.reprojection_error_px, 6
                    ),
                    f"{prefix}_intr_image_count": str(intrinsics.image_count),
                }
            )
        return row

    def _record_current_checkerboard_extrinsics_parameters(self) -> dict[str, object]:
        calibration = self.boundary.active_intrinsics
        extrinsics = None if calibration is None else calibration.extrinsics
        row: dict[str, object] = {
            "active": extrinsics is not None,
            "source": "independent_profile_composed_with_intrinsics",
            "profile_path": str(
                getattr(self.boundary, "_active_extrinsics_profile_path", "") or ""
            ),
            "profile_sha256": str(
                getattr(self.boundary, "_active_extrinsics_profile_hash", "") or ""
            ),
            "source_zoom_level": getattr(
                self.boundary, "_active_extrinsics_source_zoom_level", None
            ),
            "validation_gate": "disabled",
            "frame_contract": "opencv_optical:RDF(right,down,forward); X_right = R_left_to_right * X_left + T_left_to_right_m",
        }
        if extrinsics is None:
            return row
        right_origin = self.boundary._checkerboard_translation_right_in_left_m()
        row.update(
            {
                "pair_count": int(extrinsics.pair_count),
                "reprojection_error_px": float(extrinsics.reprojection_error_px),
                "rotation_left_to_right": [list(map(float, values)) for values in extrinsics.rotation_left_to_right],
                "translation_left_to_right_m": [float(value) for value in extrinsics.translation_left_to_right_m],
                "right_camera_origin_in_left_rdf_m": (
                    None if right_origin is None else [float(value) for value in right_origin]
                ),
            }
        )
        return row

    def _record_checkerboard_extrinsics_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        calibration = measurement_attempt.intrinsics
        extrinsics = None if calibration is None else calibration.checkerboard_extrinsics
        row: dict[str, object] = {
            "active": extrinsics is not None,
            "source": "measurement_attempt_independent_profile_bundle",
            "profile_path": (
                ""
                if calibration is None
                else calibration.selected_extrinsics_profile_path
            ),
            "profile_sha256": (
                "" if calibration is None else calibration.extrinsics_content_sha256
            ),
            "source_zoom_level": (
                None if calibration is None else calibration.extrinsics_source_zoom_level
            ),
            "validation_gate": (
                "disabled" if calibration is None else calibration.validation_gate
            ),
            "frame_contract": "opencv_optical:RDF(right,down,forward); X_right = R_left_to_right * X_left + T_left_to_right_m",
        }
        if extrinsics is None:
            return row
        right_origin = measurement_attempt.translation_right_in_left_m
        row.update(
            {
                "pair_count": extrinsics.pair_count,
                "reprojection_error_px": extrinsics.reprojection_error_px,
                "rotation_left_to_right": [list(values) for values in extrinsics.rotation_left_to_right],
                "translation_left_to_right_m": list(extrinsics.translation_left_to_right_m),
                "right_camera_origin_in_left_rdf_m": (
                    None if right_origin is None else list(right_origin)
                ),
            }
        )
        return row
