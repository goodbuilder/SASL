"""Method tabs and readiness display from explicit annotation/candidate/observation values.

Solver eligibility remains backend-owned; local text is an affordance projection."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Callable, Sequence
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import EngineeringValue

from .layouts import _CurrentPageTabWidget


class MethodControls(QtCore.QObject):
    information_changed = QtCore.pyqtSignal()
    mountain_lines_changed = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None, *, make_solver_button, create_combine_button, populate_combine_page):
        super().__init__(parent)
        for callback in (make_solver_button, create_combine_button, populate_combine_page):
            if not callable(callback):
                raise TypeError("Method controls require explicit button and combine-content ports")
        command_button = make_solver_button
        method_buttons = {}

        def make_solver_button(command_id, label):
            button = command_button(command_id, label)
            method_buttons[command_id] = button
            return button

        readiness_group = QtWidgets.QGroupBox("Readiness")
        self.readiness_group = readiness_group
        readiness_layout = QtWidgets.QGridLayout(readiness_group)
        readiness_layout.setContentsMargins(6, 6, 6, 6)
        self.readiness_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("state", "Pose state"),
                ("operating_zoom", "Operating zoom L/R"),
                ("intrinsics", "Exact K,D"),
                ("base_extrinsics", "Base R,t"),
                ("mountain_uv", "Mountain u,v"),
                ("terrain", "Terrain GeoTIFF"),
                ("target_reference", "Target reference"),
                ("yaw90_reference", "Mountain/Cloud 기준"),
            )
        ):
            value = EngineeringValue(label)
            value.set_value("--", source="optical pose owner snapshot")
            self.readiness_values[key] = value
            readiness_layout.addWidget(value, index // 2, index % 2)
        multiview_button = make_solver_button(
            "alignment.pose.solve_drone_rtk_multiview",
            "Solve Drone RTK Multi-view R,t",
        )
        multiview_button.setToolTip(
            "Use at least four READY Frozen target pairs accumulated since the last Alignment Reset, together "
            "with their capture-time Drone RTK positions, to solve full 6-DOF R,t for both cameras."
        )
        workflow_note = QtWidgets.QLabel(
            "Each solver below creates an independent candidate and shares the same review/apply path."
        )
        workflow_note.setObjectName("compactContract")
        workflow_note.setWordWrap(True)
        readiness_layout.addWidget(workflow_note, 4, 0, 1, 2)

        self.method_tabs = _CurrentPageTabWidget()
        self.method_tabs.setObjectName("alignmentOperationTabs")
        self.method_information_labels: dict[str, QtWidgets.QLabel] = {}
        self.method_detail_labels: dict[str, QtWidgets.QLabel] = {}
        method_contracts = {
            "Horizon": "Observed: Roll/Pitch · Measurement fill: Yaw=pair-frozen Pixhawk, t=pair-frozen GNSS",
            "Mountain + GNSS": "Observed: relative R from paired u,v · t: frozen GNSS baseline · no terrain height",
            "Terrain PnP": "Observed: full 6-DOF · requires GeoTIFF vertical datum and matched 2D/3D points",
            "Drone Rotation": "1 target: Yaw · 2+ targets: Roll/Pitch/Yaw · Measurement t: pair-frozen GNSS",
            "Drone Multi-view": "Observed: full 6-DOF R,t · requires at least four READY accumulated Frozen target pairs",
            "Combine": "Compose only non-overlapping axes observable on both camera sides with shared prior/K,D",
        }
        method_details = {
            "Horizon": "Evidence: horizon point count, model fit, Roll/Pitch sensitivity, prior Yaw/Translation source",
            "Mountain + GNSS": "Evidence: paired point count, inliers/outliers, Sampson residual, GNSS baseline, lever-arm status",
            "Terrain PnP": "Evidence: GeoTIFF/vertical datum, 2D↔3D counts, reprojection error, depth ratio, rank/condition",
            "Drone Rotation": "Evidence: target count, Yaw/full-R mode, Drone/P1/P2 RTK state, sync delta, angular residual",
            "Drone Multi-view": "Evidence: JSON hash, per-side observation count, timestamp verification, 3D spread, full-rank status",
            "Combine": "Evidence: selected candidates, per-side observability, DOF ownership, overlap and shared-prior checks",
        }

        def add_method_tab(
            tab_label: str,
            description: str,
            buttons: Sequence[QtWidgets.QWidget],
            content: Callable[[QtWidgets.QVBoxLayout], None] | None = None,
        ) -> None:
            page = QtWidgets.QWidget()
            page_layout = QtWidgets.QVBoxLayout(page)
            page_layout.setContentsMargins(6, 6, 6, 6)
            note = QtWidgets.QLabel(description)
            note.setObjectName("compactContract")
            note.setWordWrap(True)
            page_layout.addWidget(note)
            contract_label = QtWidgets.QLabel(method_contracts.get(tab_label, ""))
            contract_label.setObjectName("compactContract")
            contract_label.setWordWrap(True)
            page_layout.addWidget(contract_label)
            information_label = QtWidgets.QLabel("Solve readiness: waiting for owner snapshot")
            information_label.setObjectName("statusStrip")
            information_label.setWordWrap(True)
            self.method_information_labels[tab_label] = information_label
            page_layout.addWidget(information_label)
            detail_label = QtWidgets.QLabel(method_details.get(tab_label, ""))
            detail_label.setObjectName("compactContract")
            detail_label.setWordWrap(True)
            self.method_detail_labels[tab_label] = detail_label
            page_layout.addWidget(detail_label)
            if content is not None:
                content(page_layout)
            action_layout = QtWidgets.QHBoxLayout()
            for button in buttons:
                action_layout.addWidget(button)
            action_layout.addStretch(1)
            page_layout.addLayout(action_layout)
            page_layout.addStretch(1)
            index = self.method_tabs.addTab(page, tab_label)
            self.method_tabs.setTabToolTip(index, description)

        add_method_tab(
            "Horizon",
            "Two horizon points constrain Roll/Pitch only. Yaw and camera position remain at the prior pose.",
            (make_solver_button("alignment.pose.solve_horizon", "Solve Horizon Roll/Pitch"),),
        )

        def build_mountain_content(page_layout: QtWidgets.QVBoxLayout) -> None:
            self.mountain_hide_auto_lines_checkbox = QtWidgets.QCheckBox("Hide Auto-select connecting lines")
            self.mountain_hide_auto_lines_checkbox.setObjectName("mountainHideAutoSuggestionLines")
            self.mountain_hide_auto_lines_checkbox.setChecked(True)
            self.mountain_hide_auto_lines_checkbox.setToolTip(
                "Hide only the line joining Mountain Auto-select points. Right-click an "
                "automatic point to exclude its corresponding left/right pair from the overlay "
                "and from the accepted solver input."
            )
            self.mountain_hide_auto_lines_checkbox.toggled.connect(self.mountain_lines_changed.emit)
            page_layout.addWidget(self.mountain_hide_auto_lines_checkbox)

        add_method_tab(
            "Mountain + GNSS",
            "Click corresponding Mountain features in the same order on both frames. Uses paired u,v and the numeric GNSS baseline; no GeoTIFF or terrain height.",
            (
                make_solver_button("alignment.pose.solve_gnss_mountain", "Solve GNSS + Mountain R,t"),
                make_solver_button("alignment.pose.solve_cloud_rotation", "Solve Cloud rotation"),
            ),
            content=build_mountain_content,
        )
        self.mountain_lines_changed.emit(True)

        def build_terrain_content(page_layout: QtWidgets.QVBoxLayout) -> None:
            terrain_note = QtWidgets.QLabel(
                "Uses the same Ridge points as Mountain + GNSS -- switch to Ridge mode on that "
                'tab to click them. "Solve Terrain PnP" will prompt for a correspondences JSON '
                "matching each side's clicked points 1:1 with object_points_ned_m; see the "
                "Readiness card above for GeoTIFF/vertical-datum status."
            )
            terrain_note.setObjectName("compactContract")
            terrain_note.setWordWrap(True)
            page_layout.addWidget(terrain_note)

        add_method_tab(
            "Terrain PnP",
            "Full 6-DOF pose from 2D/3D correspondences against an imported GeoTIFF -- the only "
            "method that does not inherit any axis from the prior pose when it succeeds.",
            (
                make_solver_button("alignment.pose.import_terrain", "Import GeoTIFF"),
                make_solver_button("alignment.pose.solve_terrain", "Solve Terrain PnP"),
            ),
            content=build_terrain_content,
        )
        add_method_tab(
            "Drone Rotation",
            "One synchronized target constrains Yaw; two or more non-degenerate target/ray observations "
            "constrain Roll/Pitch/Yaw. Missing Translation is supplied by live GNSS in Measurement.",
            (make_solver_button("alignment.pose.solve_drone_yaw", "Solve Drone RTK Rotation"),),
        )
        add_method_tab(
            "Drone Multi-view",
            "Accumulate at least four synchronized Frozen target pairs in Drone Rotation, then solve "
            "full six-DOF camera pose without a separate JSON file. Translation is solved from the "
            "observations instead of inherited from live GNSS.",
            (multiview_button,),
        )
        combine_button = create_combine_button()

        add_method_tab(
            "Combine",
            "Compose explicitly selected non-overlapping candidate axes into one immutable candidate.",
            (combine_button,),
            content=populate_combine_page,
        )
        method_buttons["alignment.pose.solve_horizon"].setToolTip(
            "Automatically fit Roll/Pitch from the manually marked horizon line. The horizon cannot observe Yaw."
        )
        method_buttons["alignment.pose.solve_gnss_mountain"].setToolTip(
            "Build metric R,t from at least four same-order paired Mountain u,v clicks "
            "on frozen left/right frames and the numeric left/right GNSS baseline. "
            "No GeoTIFF or terrain height is used. Fix quality is recorded as evidence, "
            "not used as a hard gate. Assumes camera optical center equals GNSS antenna."
        )
        method_buttons["alignment.pose.solve_cloud_rotation"].setToolTip(
            "Rotation-only solve from the same paired Ridge points treated as far-field "
            "sky/cloud correspondences (Auto-select Cloud). Translation is never applied by "
            "this candidate; the Cloud measurement method fills N/E/D from live GNSS. Both "
            "frozen frames must be simultaneous within the burst pairing tolerance."
        )
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
