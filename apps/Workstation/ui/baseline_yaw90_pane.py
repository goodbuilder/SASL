"""Display-only live Baseline Yaw90 installation surface."""
from __future__ import annotations

import math
import time
from copy import deepcopy
from collections.abc import Mapping

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workspace_views.camera_preview import CameraPreview
from Workstation.ui.designer import load_form


def number(value: object, *, signed: bool = False, digits: int = 2) -> str:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(parsed):
        return "--"
    return format(parsed, f"{'+' if signed else ''}.{digits}f")


class BaselineYaw90Pane(QtWidgets.QWidget):
    inspection_requested = QtCore.pyqtSignal(object)
    open_measurement = QtCore.pyqtSignal()

    def __init__(self, backend: object, invoke_command=None, parent=None):
        super().__init__(parent)
        self.backend = backend
        self._snapshot = {}
        self._last_good_snapshot = None
        self._last_good_snapshot_s = None
        self._dirty = False
        self._feedback = ""
        self._camera_status_text = {}
        self._preview_viewport = None
        self._preview_fit_timer = QtCore.QTimer(self)
        self._preview_fit_timer.setSingleShot(True)
        self._preview_fit_timer.setInterval(0)
        self._preview_fit_timer.timeout.connect(self._fit_previews)
        load_form("baseline_yaw90.ui", self)
        for widget in self.findChildren(QtWidgets.QWidget):
            legacy_name = widget.property("legacyObjectName")
            if legacy_name:
                widget.setObjectName(str(legacy_name))
        self.preview_button.setEnabled(callable(invoke_command))
        self.preview_button.clicked.connect(lambda: invoke_command("camera.toggle"))
        self.measurement_button.clicked.connect(self.open_measurement.emit)
        self.previews = {}
        for side in ("left", "right"):
            slot = getattr(self, "previews_" + side)
            preview = CameraPreview(
                side, crop_to_fill=False, lock_chrome_height=True, wheel_zoom=True
            )
            preview.setMinimumWidth(0)
            preview.setFixedHeight(230)
            preview.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            # Replace the Designer slot directly: an extra QWidget/layout layer
            # delays size-hint propagation when the viewport first becomes visible.
            self.layout().replaceWidget(slot, preview)
            slot.hide()
            slot.deleteLater()
            self.previews[side] = preview
        self.center_line_checkbox.toggled.connect(self._center_line_toggled)
        for table in (self.attitude_table, self.angle_table, self.reference_table):
            self._configure_table(table)
        for table, widths in ((self.attitude_table, (62, 62, 92, 56)), (self.angle_table, (66, 92, 56))):
            for column, width in enumerate(widths, start=1):
                table.horizontalHeader().resizeSection(column, width)
        self.attitude_table.clicked.connect(lambda index: self._inspect("attitudes", index.row()))
        self.angle_table.clicked.connect(lambda index: self._inspect("angles", index.row()))
        self.reverse_checkbox.toggled.connect(self._edited)
        self.apply_button.clicked.connect(lambda: self._apply(False))
        self.save_button.clicked.connect(lambda: self._apply(True))
        self.reference_table.hide()
        self.reference_toggle.toggled.connect(self.reference_table.setVisible)
        connector = getattr(backend, "connect_baseline_yaw90_preview", None)
        if callable(connector):
            connector(self)
        self.apply_snapshot({})

    @staticmethod
    def _configure_table(table):
        """Initialize dynamic cells and header sizing on the Designer table."""
        table.verticalHeader().setDefaultSectionSize(24)
        table.horizontalHeader().setFixedHeight(28)
        header = table.horizontalHeader()
        header.setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        for col in range(1, table.columnCount()):
            header.resizeSection(col, 74)
        for row in range(table.rowCount()):
            for col in range(table.columnCount()):
                table.setItem(row, col, QtWidgets.QTableWidgetItem("--"))

    @staticmethod
    def _row(table, index, values, detail):
        for col, value in enumerate(values):
            item = table.item(index, col)
            item.setText(str(value))
            item.setToolTip(detail)
            try:
                float(value)
                numeric = col > 0
            except (TypeError, ValueError):
                numeric = False
            item.setTextAlignment((QtCore.Qt.AlignRight if numeric else QtCore.Qt.AlignLeft) | QtCore.Qt.AlignVCenter)

    @staticmethod
    def _bounded(label, text):
        label.setToolTip(text)
        label.setText(label.fontMetrics().elidedText(text, QtCore.Qt.ElideRight, max(100, label.width())))

    def _center_line_toggled(self, checked):
        for preview in self.previews.values():
            preview.set_center_line_visible(checked)

    def set_frame(self, side: str, image: QtGui.QImage, metadata: object):
        if side in self.previews:
            self.previews[side].set_frame(image, metadata)
            if side in self._camera_status_text:
                self.previews[side].meta.setText(self._camera_status_text[side])

    def clear_previews(self):
        for preview in self.previews.values():
            preview.clear()

    def refresh(self, statuses=None):
        reader = getattr(self.backend, "baseline_yaw90_snapshot", None)
        try:
            snapshot = reader() if callable(reader) else {}
        except Exception as exc:
            snapshot = {"error": f"{type(exc).__name__}: {exc}"}
        self.apply_snapshot(snapshot)
        if isinstance(statuses, Mapping):
            cameras = [statuses.get(f"camera.{side}") for side in ("left", "right")]
            for side, state in zip(("left", "right"), cameras):
                self._camera_status_text[side] = str(getattr(state, "state", "WAITING"))
                self.previews[side].meta.setText(self._camera_status_text[side])
            text = " · ".join(
                f"{label}: {getattr(state, 'state', 'WAITING')}"
                for label, state in zip(("CAM L", "CAM R"), cameras)
            )
            self._bounded(self.status, text + " · " + self.status.toolTip())
            live = any(getattr(state, "state", "") in {"LIVE", "STARTING"} for state in cameras)
            self.preview_button.setText("Stop preview" if live else "Preview")

    def apply_snapshot(self, data):
        now = time.monotonic()
        try:
            data = self._validated_snapshot(data)
            if data.get("error"):
                raise RuntimeError(str(data["error"]))
        except Exception as exc:
            data = self._query_failure_snapshot(str(exc), now)
        else:
            # This is display history only, never a Measurement input cache.
            self._last_good_snapshot = deepcopy(data)
            self._last_good_snapshot_s = now
        self._render_snapshot(data)

    @staticmethod
    def _validated_snapshot(data):
        """Check container contracts before replacing any visible rows."""
        if not isinstance(data, Mapping):
            raise TypeError("Baseline Yaw90 snapshot must be a mapping")
        data = deepcopy(dict(data))
        for key in ("baseline", "drone_geometry", "midpoint", "branch_preview"):
            value = data.get(key)
            if value is not None and not isinstance(value, Mapping):
                raise TypeError(f"Baseline Yaw90 {key} must be a mapping")
            data[key] = dict(value or {})
        for container, key in ((data, "attitudes"), (data["drone_geometry"], "angles")):
            rows = container.get(key)
            if rows is None:
                rows = []
            if not isinstance(rows, (tuple, list)) or any(not isinstance(row, Mapping) for row in rows):
                raise TypeError(f"Baseline Yaw90 {key} must contain mapping rows")
            container[key] = [dict(row) for row in rows]
        ages = data.get("reference_ages_s")
        if ages is not None and not isinstance(ages, (tuple, list)):
            raise TypeError("Baseline Yaw90 reference ages must be a sequence")
        for key in ("ages_s", "fix_labels"):
            value = data["midpoint"].get(key)
            if value is not None and not isinstance(value, Mapping):
                raise TypeError(f"Baseline Yaw90 midpoint {key} must be a mapping")
        return data

    def _query_failure_snapshot(self, error, now):
        """Age a copy of the last confirmed display; never alter source caches."""
        data = deepcopy(self._last_good_snapshot or {})
        elapsed = max(0.0, now - self._last_good_snapshot_s) if self._last_good_snapshot_s is not None else 0.0

        def age(value):
            try:
                value = float(value)
            except (TypeError, ValueError):
                return None
            return value + elapsed if math.isfinite(value) else None

        reason = f"화면 갱신 오류: {error} · 마지막 확인값"
        attitudes = list(data.get("attitudes") or [])
        for index, label in enumerate(("P1", "P2")):
            if index >= len(attitudes):
                attitudes.append({"label": label})
            row = attitudes[index]
            row.update(state="QUERY_ERROR", age_s=age(row.get("age_s")), reason=reason)
        data["attitudes"] = attitudes
        baseline = data.setdefault("baseline", {})
        baseline.update(baseline_state="QUERY_ERROR", reason=reason)
        for key in ("baseline_age_s", "p1_gps_age_s", "p2_gps_age_s"):
            baseline[key] = age(baseline.get(key))
        drone = data.setdefault("drone_geometry", {})
        old_angles = {row.get("key"): row for row in drone.get("angles", [])}
        drone["angles"] = []
        for key in ("dr_p1_p2", "dr_p2_p1"):
            row = old_angles.get(key, {"key": key})
            row.update(state="QUERY_ERROR", age_s=age(row.get("age_s")), reason=reason)
            drone["angles"].append(row)
        midpoint = data.setdefault("midpoint", {})
        midpoint.update(state="QUERY_ERROR", reason=reason, maximum_age_s=age(midpoint.get("maximum_age_s")))
        midpoint["ages_s"] = {key: age(value) for key, value in (midpoint.get("ages_s") or {}).items()}
        data["reference_ages_s"] = tuple(age(value) for value in data.get("reference_ages_s") or ())
        data.update(error=error, display_query_failed=True, reverse_editable=False)
        return data

    @staticmethod
    def _state_text(state):
        return "조회 오류" if state == "QUERY_ERROR" else str(state)

    def _render_snapshot(self, data):
        self._snapshot = data
        attitudes = list(data.get("attitudes") or [])
        self._attitude_rows = []
        for index, label in enumerate(("P1", "P2")):
            row = {"label": label, **(attitudes[index] if index < len(attitudes) else {})}
            self._attitude_rows.append(row)
            self._row(self.attitude_table, index, (
                label, number(row.get("roll_deg"), signed=True), number(row.get("pitch_deg"), signed=True),
                self._state_text(row.get("state", "WAITING")), number(row.get("age_s")),
            ), str(row))
        baseline = data.get("baseline") or {}
        drone = data.get("drone_geometry") or {}
        angles = list(drone.get("angles") or [])
        rows = [{"label": "P1→P2 azimuth", "angle_deg": baseline.get("baseline_azimuth_deg"),
                 "state": baseline.get("baseline_state", "WAITING"), "age_s": baseline.get("baseline_age_s"),
                 "source": "selected P1/P2 GNSS", "frame": "horizontal NED; North=0°, clockwise", **{
                     key: baseline.get(key) for key in ("p1_gps_source", "p2_gps_source", "gps_pair_delta_ms", "p1_fix_type", "p2_fix_type")}}]
        for key, label in (("dr_p1_p2", "∠Dr–P1–P2"), ("dr_p2_p1", "∠Dr–P2–P1")):
            row = dict(next((r for r in angles if r.get("key") == key), {}))
            row["label"] = label
            if not data.get("display_query_failed") and (drone.get("drone_position_frozen") or drone.get("installation_position_frozen")):
                row["state"] = "FROZEN"
                row["reason"] = drone.get("reason")
            rows.append(row)
        self._angle_rows = rows
        for index, row in enumerate(rows):
            angle = row.get("angle_deg")
            if index == 0 and isinstance(angle, (float, int)) and math.isfinite(angle):
                angle = round(angle, 2) % 360.0
            self._row(self.angle_table, index, (row["label"], number(angle),
                self._state_text(row.get("state", "WAITING")), number(row.get("age_s"))), str(row))
        midpoint = data.get("midpoint") or {}
        for index, (key, label, digits) in enumerate((
            ("forward_m", "Forward m", 3), ("right_m", "Right m", 3), ("down_m", "Down m", 3),
            ("range_m", "Range m", 3), ("baseline_drone_angle_deg", "중앙점 baseline 선각°", 2),
            ("baseline_drone_angle_error_deg", "중앙점 선각의 90° 편차", 2),
        )):
            self._row(self.reference_table, index, (label, number(midpoint.get(key), digits=digits),
                self._state_text(midpoint.get("state", "WAITING"))), str(midpoint))
        branch = data.get("branch_preview") or {}
        text = (f"Yaw preview: {number(branch.get('selected_yaw_deg'))}° · "
                f"−90°: {number(branch.get('candidate_minus90_deg'))}° / +90°: {number(branch.get('candidate_plus90_deg'))}° · "
                f"{branch.get('mode', 'WAITING')}")
        if branch.get("low_margin"):
            text += " · low margin"
        if branch.get("consensus") is False:
            text += " · P1/P2 disagree"
        if data.get("branch_error"):
            text += " · " + str(data["branch_error"])
        if branch.get("ready") is False and branch.get("reason"):
            text += " · " + str(branch["reason"])
        reference_ages = data.get("reference_ages_s") or ()
        if any(isinstance(age, (int, float)) and age > 2.0 for age in reference_ages):
            text += " · STALE yaw reference"
        if data.get("display_query_failed"):
            text = "조회 오류 · 마지막 확인값 · " + text
        self._bounded(self.branch_status, text)
        self.branch_status.setToolTip(text + "\n" + str(branch) + "\nReference ages: " + str(data.get("reference_ages_s")))
        editable = bool(data.get("reverse_editable", False))
        if not self._dirty and not data.get("display_query_failed"):
            blocked = self.reverse_checkbox.blockSignals(True)
            self.reverse_checkbox.setChecked(bool(data.get("reverse", False)))
            self.reverse_checkbox.blockSignals(blocked)
        for widget in (self.reverse_checkbox, self.apply_button, self.save_button):
            widget.setEnabled(editable)
        self._bounded(self.settings_status, self._feedback or ("미적용 변경" if self._dirty else ""))
        status = ("조회 오류 · 마지막 확인값 · " + str(data["error"]) if data.get("display_query_failed") else
                  f"Live installation · Measurement {data.get('run_state', 'IDLE')}" +
                  (" · Reverse locked" if not editable else ""))
        self._bounded(self.status, status)

    def _edited(self, _checked):
        self._dirty = True
        self._feedback = "미적용 변경"
        self._bounded(self.settings_status, self._feedback)

    def _apply(self, persist):
        try:
            result = self.backend.apply_baseline_yaw90_settings(
                {"measurement_baseline_yaw_reverse": self.reverse_checkbox.isChecked()}, persist=persist)
            if not isinstance(result, Mapping) or result.get("reverse") != self.reverse_checkbox.isChecked():
                raise RuntimeError("Reverse apply was not confirmed")
            if persist and not result.get("saved"):
                raise RuntimeError("Settings save was not confirmed")
        except Exception as exc:
            self._feedback = ("저장 실패 (적용 상태 확인 필요): " if persist else "적용 실패: ") + str(exc)
            self._dirty = True
        else:
            self._dirty = False
            self._feedback = "저장 완료" if persist else "적용 완료 · 저장 전"
        self.refresh()

    def _inspect(self, kind, row):
        details = self._selection_details(kind, row)
        if details:
            details["_resolver"] = lambda: self._selection_details(kind, row)
            self.inspection_requested.emit(details)

    def _selection_details(self, kind, row):
        rows = self._attitude_rows if kind == "attitudes" else self._angle_rows
        if 0 <= row < len(rows):
            value = rows[row]
            text = (f"Roll {number(value.get('roll_deg'), signed=True)}° / Pitch {number(value.get('pitch_deg'), signed=True)}°"
                    if kind == "attitudes" else number(value.get("angle_deg")) + "°")
            source = str(value.get("source", "GNSS / ATTITUDE"))
            if kind == "angles":
                base = self._snapshot.get("baseline") or {}
                midpoint = self._snapshot.get("midpoint") or {}
                source += (f" · P1 {base.get('p1_gps_source', '--')} / P2 {base.get('p2_gps_source', '--')}"
                           f" · P1/P2 Δt {number(base.get('gps_pair_delta_ms'))} ms")
                if row > 0:
                    source += f" · 3-GNSS Δt {number(midpoint.get('timestamp_spread_ms'))} ms · Fix {midpoint.get('fix_labels', {})}"
            return {"selection_owner": "baseline_yaw90", "selection": value.get("label", ""), "value": text,
                "freshness": f"{self._state_text(value.get('state', 'WAITING'))} · {number(value.get('age_s'))} s",
                "source": source,
                "contract": value.get("frame", value.get("frame_contract", "")),
                "verdict": (f"화면 갱신 오류: {self._snapshot['error']} · 마지막 확인값"
                            if self._snapshot.get("display_query_failed") else
                            value.get("reason") or "Read-only live installation evidence")}
        return {}

    def showEvent(self, event):
        super().showEvent(event)
        self._watch_preview_viewport()
        self.refresh()
        self._queue_preview_fit()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._queue_preview_fit()

    def hideEvent(self, event):
        self._preview_fit_timer.stop()
        super().hideEvent(event)

    def _watch_preview_viewport(self):
        parent = self.parentWidget()
        while parent is not None and not isinstance(parent, QtWidgets.QScrollArea):
            parent = parent.parentWidget()
        viewport = parent.viewport() if parent is not None else None
        if viewport is self._preview_viewport:
            return
        if self._preview_viewport is not None:
            try:
                self._preview_viewport.removeEventFilter(self)
            except RuntimeError:
                pass  # The former host may have been destroyed after reparenting.
        self._preview_viewport = viewport
        if viewport is not None:
            viewport.installEventFilter(self)

    def eventFilter(self, watched, event):
        if watched is self._preview_viewport and event.type() == QtCore.QEvent.Resize:
            self._queue_preview_fit()
        return super().eventFilter(watched, event)

    def _queue_preview_fit(self):
        if self.isVisible() and not self._preview_fit_timer.isActive():
            self._preview_fit_timer.start()

    def _fit_previews(self):
        if not self.isVisible():
            return
        available = self._preview_viewport.height() if self._preview_viewport is not None else 650
        height = max(120, min(320, available - 290))
        for preview in self.previews.values():
            if preview.height() != height:
                preview.setFixedHeight(height)
