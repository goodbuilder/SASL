"""Live relative R,t table, independent of frozen observation and candidate state.

R maps left optical RDF to right optical RDF; t is [right,down,forward] metres.
The supplied method labels are a presentation contract, not a solver dependency."""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets
from .layouts import load_component_form


class LiveRelativePose(QtCore.QObject):
    def __init__(self, methods, parent=None):
        super().__init__(parent)
        self._methods = tuple(methods)
        load_component_form("optical_pose_live_reference.ui", self, "live_reference")
        self._live_rt_rows: dict[str, dict[str, object]] = {}
        self.set_live_rt_snapshot({"rows": []})
        self.destroyed.connect(self.live_rt_group.deleteLater)

    def set_live_rt_snapshot(self, snapshot: Mapping[str, object], *, error: str = "") -> None:
        """Update fixed live rows in-place, keeping solved candidate selection intact."""
        now = time.monotonic()
        if not error:
            rows = snapshot.get("rows")
            if not isinstance(rows, (tuple, list)):
                raise ValueError("live R,t snapshot rows missing")
            checked = {}
            for row in rows:
                if not isinstance(row, Mapping):
                    raise ValueError("invalid live R,t row")
                rotation = row.get("rotation_right_from_left_rdf")
                translation = row.get("translation_right_from_left_rdf_m")
                if rotation is not None and (
                    not isinstance(rotation, (tuple, list))
                    or len(rotation) != 3
                    or any(
                        not isinstance(line, (tuple, list))
                        or len(line) != 3
                        or any(not math.isfinite(float(v)) for v in line)
                        for line in rotation
                    )
                ):
                    raise ValueError("invalid live R matrix")
                if translation is not None and (
                    not isinstance(translation, (tuple, list))
                    or len(translation) != 3
                    or any(not math.isfinite(float(v)) for v in translation)
                ):
                    raise ValueError("invalid live translation")
                timestamp = row.get("oldest_sample_s")
                if timestamp is not None and not math.isfinite(float(timestamp)):
                    raise ValueError("invalid live sample timestamp")
                checked[str(row.get("method_id"))] = dict(row)
            self._live_rt_rows = checked
        for index, (method, label) in enumerate(self._methods):
            row = self._live_rt_rows.get(method, {})
            rotation = row.get("rotation_right_from_left_rdf")
            translation = row.get("translation_right_from_left_rdf_m")
            r_text = (
                "--"
                if rotation is None
                else "\n".join("[" + ", ".join(f"{float(v):+.5f}" for v in line) + "]" for line in rotation)
            )
            t_text = (
                "--"
                if translation is None
                else "\n".join(f"{axis} {float(v):+.6f}" for axis, v in zip(("R", "D", "F"), translation))
            )
            timestamp = row.get("oldest_sample_s")
            age = max(0.0, now - float(timestamp)) if isinstance(timestamp, (float, int)) else None
            state = "ERROR" if error else str(row.get("state", "WAITING"))
            tooltip = str(error or row.get("reason") or "")
            tooltip += f"\nR:\n{r_text}\nt [R,D,F] m:\n{t_text}"
            tooltip += f"\nP1/P2 attitude, GNSS ages: {row.get('input_ages_s', '--')} s"
            if row.get("yaw_selection"):
                yaw = row["yaw_selection"]
                tooltip += f"\nYaw={yaw.get('selected_yaw_deg')} deg; Reverse={yaw.get('reverse')}"
            for column, text in enumerate((label, r_text, t_text, state, "--" if age is None else f"{age:.2f}")):
                item = self.live_rt_table.item(index, column)
                if item.text() != text:
                    item.setText(text)
                item.setToolTip(tooltip)
