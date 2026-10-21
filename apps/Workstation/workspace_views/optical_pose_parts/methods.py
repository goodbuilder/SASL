"""Method tabs and readiness display from explicit annotation/candidate/observation values.

Solver eligibility remains backend-owned; local text is an affordance projection."""

from __future__ import annotations

from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets
from .layouts import load_component_form, bind_component_form



class MethodControls(QtCore.QObject):
    information_changed = QtCore.pyqtSignal()
    mountain_lines_changed = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None, *, make_solver_button, create_combine_button, populate_combine_page):
        super().__init__(parent)
        for callback in (make_solver_button, create_combine_button, populate_combine_page):
            if not callable(callback):
                raise TypeError("Method controls require explicit button and combine-content ports")
        load_component_form("optical_pose_readiness.ui", self, "methods")
        self.method_tabs = load_component_form(
            "optical_pose_methods.ui", self, "methods", make_solver_button
        )
        # The candidate owner retains Combine state and callback lifecycle.
        create_combine_button()
        combine_page = self.method_tabs.widget(self.method_tabs.count() - 1)
        populate_combine_page(combine_page.layout())
        bind_component_form(combine_page, self, "methods")
        self.mountain_hide_auto_lines_checkbox.toggled.connect(self.mountain_lines_changed.emit)
        self.mountain_lines_changed.emit(True)
        self.destroyed.connect(self.readiness_group.deleteLater)
        self.destroyed.connect(self.method_tabs.deleteLater)

    def set_readiness(self, readiness: Mapping[str, object]) -> None:
        for key, widget in self.readiness_values.items():
            raw = readiness.get(key)
            if isinstance(raw, Mapping):
                display = raw.get("value", raw.get("state", "--"))
                source = str(raw.get("source", "optical pose owner snapshot") or "")
                frame = str(raw.get("frame", "") or "")
                valid = raw.get("valid")
                valid = valid if isinstance(valid, bool) else None
            else:
                display = "--" if raw in (None, "") else raw
                source = "optical pose owner snapshot"
                frame = ""
                valid = None
            widget.set_value(display, source=source, frame=frame, valid=valid)
        self.information_changed.emit()

    def refresh_information(self, annotation_payloads, candidate_rows, observation_count, multiview_samples) -> None:
        annotation_counts: dict[str, dict[str, int]] = {
            "left": {"horizon": 0, "ridge": 0, "target": 0},
            "right": {"horizon": 0, "ridge": 0, "target": 0},
        }
        for side, payload in annotation_payloads.items():
            if payload is None:
                continue
            annotation_counts[side] = {
                "horizon": len(payload.horizon_line),
                "ridge": len(payload.ridge_polyline),
                "target": len(payload.target_point),
            }

        candidate_state_by_method: dict[str, str] = {}
        candidate_by_method: dict[str, Mapping[str, object]] = {}
        method_categories = {
            "flat-horizon-roll-pitch": "Horizon",
            "spherical-horizon-roll-pitch": "Horizon",
            "gnss-constrained-mountain-uv": "Mountain + GNSS",
            "terrain-pnp-ensemble": "Terrain PnP",
            "drone-rtk-target-yaw": "Drone Rotation",
            "drone-rtk-multiview-r-t": "Drone Multi-view",
            "combined-independent-axis-evidence": "Combine",
        }
        for candidate in candidate_rows:
            method = str(candidate.get("method", "") or "")
            category = method_categories.get(method)
            if category is not None:
                candidate_by_method[category] = candidate
                candidate_state_by_method[category] = str(
                    candidate.get("state", candidate.get("quality", "CANDIDATE")) or "CANDIDATE"
                )

        left = annotation_counts["left"]
        right = annotation_counts["right"]
        terrain_state = self.readiness_values["terrain"].value.text()
        multiview_sample_count = len(multiview_samples)
        multiview_ready_count = sum(str(sample.get("state") or "").upper() == "READY" for sample in multiview_samples)
        candidate_count = len(candidate_rows)
        information = {
            "Horizon": (
                f"Solve readiness · horizon points L/R={left['horizon']}/{right['horizon']} "
                f"(2 required each) · candidate={candidate_state_by_method.get('Horizon', '--')} · "
                "Measurement also requires fresh Pixhawk Yaw and usable GNSS t"
            ),
            "Mountain + GNSS": (
                f"Solve readiness · paired ridge points L/R={left['ridge']}/{right['ridge']} "
                f"(≥4, equal order) · candidate={candidate_state_by_method.get('Mountain + GNSS', '--')} · "
                "GNSS Yaw90 + P1 R/P at capture request; existing Base R,t is not required"
            ),
            "Terrain PnP": (
                f"Solve readiness · ridge points L/R={left['ridge']}/{right['ridge']} · "
                f"terrain={terrain_state or '--'} · candidate={candidate_state_by_method.get('Terrain PnP', '--')}"
            ),
            "Drone Rotation": (
                f"Solve readiness · target points L/R={left['target']}/{right['target']} · "
                f"captured observations={observation_count} · candidate={candidate_state_by_method.get('Drone Rotation', '--')}"
            ),
            "Drone Multi-view": (
                f"Solve readiness · accumulated READY/total={multiview_ready_count}/{multiview_sample_count} "
                f"(≥4 READY required) · captured evidence={observation_count} · "
                f"candidate={candidate_state_by_method.get('Drone Multi-view', '--')}"
            ),
            "Combine": (
                f"Combine readiness · candidates={candidate_count} · "
                f"combined={candidate_state_by_method.get('Combine', '--')}"
            ),
        }

        def candidate_metric(category: str, key: str, default: object = "--") -> object:
            row = candidate_by_method.get(category, {})
            metrics = row.get("quality_metrics")
            metrics = metrics if isinstance(metrics, Mapping) else {}
            return metrics.get(key, default)

        def candidate_input(category: str, key: str, default: object = "--") -> object:
            row = candidate_by_method.get(category, {})
            hashes = row.get("input_hashes")
            hashes = hashes if isinstance(hashes, Mapping) else {}
            return hashes.get(key, default)

        details = {
            "Horizon": (
                f"model={candidate_input('Horizon', 'horizon_model')} · "
                f"L/R RMS px={candidate_metric('Horizon', 'left_rms_px_equivalent')}/"
                f"{candidate_metric('Horizon', 'right_rms_px_equivalent')} · "
                f"sensitivity R/P={candidate_metric('Horizon', 'left_roll_sensitivity_1px_deg')}/"
                f"{candidate_metric('Horizon', 'left_pitch_sensitivity_1px_deg')} deg"
            ),
            "Mountain + GNSS": (
                f"pairs={candidate_metric('Mountain + GNSS', 'mountain_uv_pair_count')} · "
                f"inlier ratio={candidate_metric('Mountain + GNSS', 'inlier_ratio')} · "
                f"Sampson RMS px={candidate_metric('Mountain + GNSS', 'sampson_rms_px_equivalent')} · "
                f"baseline={candidate_metric('Mountain + GNSS', 'gnss_baseline_m')} m · "
                f"lever-arm={candidate_input('Mountain + GNSS', 'lever_arm_status')}"
            ),
            "Terrain PnP": (
                f"L/R reprojection RMSE={candidate_metric('Terrain PnP', 'left_reprojection_rmse_px')}/"
                f"{candidate_metric('Terrain PnP', 'right_reprojection_rmse_px')} px · "
                f"positive depth={candidate_metric('Terrain PnP', 'left_positive_depth_ratio')}/"
                f"{candidate_metric('Terrain PnP', 'right_positive_depth_ratio')}"
            ),
            "Drone Rotation": (
                f"observations L/R={candidate_metric('Drone Rotation', 'left_observation_count')}/"
                f"{candidate_metric('Drone Rotation', 'right_observation_count')} · "
                f"angular RMS={candidate_metric('Drone Rotation', 'left_rotation_rms_chord_residual')} · "
                "Translation=pair-frozen GNSS"
            ),
            "Drone Multi-view": (
                f"observations L/R={candidate_metric('Drone Multi-view', 'left_observation_count')}/"
                f"{candidate_metric('Drone Multi-view', 'right_observation_count')} · "
                f"angular RMS={candidate_metric('Drone Multi-view', 'left_angular_rms_chord')} / "
                f"{candidate_metric('Drone Multi-view', 'right_angular_rms_chord')} · "
                f"collection={candidate_input('Drone Multi-view', 'drone_rtk_multiview_collection_sha256')} · "
                f"JSON fallback={candidate_input('Drone Multi-view', 'drone_rtk_multiview_json_sha256')}"
            ),
            "Combine": (
                f"sources={len(candidate_by_method.get('Combine', {}).get('source_observation_ids', ())) or '--'} · "
                "axes must be non-overlapping and observable on both sides"
            ),
        }
        for label, widget in self.method_information_labels.items():
            widget.setText(information.get(label, "Solve readiness: --"))
        for label, widget in self.method_detail_labels.items():
            widget.setText(details.get(label, "Evidence: --"))
