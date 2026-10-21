"""Magnetometer tables, readback controls and PX4 calibration parameter views."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from PyQt5 import QtCore, QtWidgets
from Workstation.application.status_model import RuntimeStatus
from .designer import load_pixhawk_form
from ..shared import _display_number, _live_inspection, _sync_table_rows, _table_model

import math
from Workstation.workstation_core.px4_calibration_groups import PX4_CALIBRATION_GROUPS
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    MAG_PRIORITY_KEYS,
    MAG_SLOTS,
    RM3100_BACKUP_IDS,
    RM3100_PRIMARY_IDS,
    mag_priority_slot_key,
    mag_priority_display,
)
from ..px4_widgets import _px4_priority_combo_value, _set_px4_priority_combo_value
from .contracts import MagnetometerInputs
from .formatting import (
    _display_signed_deg,
    _display_magnetometer_corrected_yaw,
    _display_magnetometer_px4_yaw,
    _magnetometer_yaw_summary,
    _format_px4_parameter_value,
)


class MagnetometerView(QtCore.QObject):
    inspection_requested = QtCore.pyqtSignal(object)
    MAG_STATUS_AUTO_REFRESH_RETRY_S = 10.0
    MAG_STATUS_AUTO_REFRESH_MAX_ATTEMPTS = 3

    def __init__(
        self,
        *,
        read_inputs: Callable[[], MagnetometerInputs],
        read_cal_mag,
        read_can,
        read_legacy,
        read_preview,
        read_verification=None,
        log=None,
        invoke_command=None,
        parent=None,
    ):
        super().__init__(parent)
        if not all(callable(reader) for reader in (read_inputs, read_cal_mag, read_can, read_legacy, read_preview)):
            raise TypeError("Magnetometer view requires readback and telemetry readers")
        self._read_inputs = read_inputs
        self._read_cal_mag = read_cal_mag
        self._read_can = read_can
        self._read_legacy = read_legacy
        self._read_preview = read_preview
        self._read_verification = read_verification
        self._log = log
        self._invoke_command = invoke_command
        self._invoke_command_available = invoke_command is not None
        self._recording_locked = False
        self._designer_form = load_pixhawk_form("pixhawk_magnetometers.ui")
        form = self._designer_form
        self.magnetometer_table = form.magnetometer_table
        self.magnetometer_group = form.magnetometer_group
        self.mag_priority_combos = {
            side: {key: getattr(form, f"mag_priority_combos_{side}_{key}") for key in MAG_PRIORITY_KEYS}
            for side in ("left", "right")
        }
        for collection in (
            "mag_priority_status_labels", "mag_priority_draft_labels", "mag_priority_preview_labels",
            "mag_priority_warning_labels", "mag_priority_refresh_buttons", "mag_priority_discard_buttons",
            "mag_priority_apply_buttons", "px4_parameter_status_labels", "px4_parameter_refresh_buttons",
        ):
            setattr(self, collection, {side: getattr(form, f"{collection}_{side}") for side in ("left", "right")})
        for collection in ("px4_parameter_tables", "px4_parameter_group_status_labels", "px4_parameter_reset_buttons"):
            setattr(self, collection, {
                side: {group.key: getattr(form, f"{collection}_{side}_{group.key}") for group in PX4_CALIBRATION_GROUPS}
                for side in ("left", "right")
            })
        self.mag_priority_group = form.mag_priority_group
        self.can_magnetometer_table = form.can_magnetometer_table
        self.can_magnetometer_group = form.can_magnetometer_group
        self.parameter_group = form.parameter_group
        self.magnetometer_model = _table_model(self, ('Pixhawk', 'CAL_MAG', 'Sensor', 'Device ID', 'State', 'xmag', 'ymag', 'zmag', 'Norm', 'RAW yaw°', 'Corrected yaw°', 'PX4 yaw°', 'Δyaw°', 'Priority', 'Unit', 'Age', 'Topic'))
        self.magnetometer_table.setModel(self.magnetometer_model)
        header = form.magnetometer_table.horizontalHeader()
        for column, (width, mode) in enumerate(((72, 0), (76, 0), (190, 0), (100, 0), (72, 0), (72, 0), (72, 0), (72, 0), (72, 0), (88, 0), (96, 0), (88, 0), (88, 0), (88, 0), (52, 0), (72, 0), (190, 0))):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeMode(mode))
            header.resizeSection(column, width)
        for column in (12,):
            header.setSectionHidden(column, True)
        self.can_magnetometer_model = _table_model(self, ('Pixhawk', 'CAN port', 'Node ID', 'Device ID', 'State', 'xmag', 'ymag', 'zmag', 'Norm', 'RAW yaw°', 'Corrected yaw°', 'PX4 yaw°', 'Δyaw°', 'Unit', 'Age', 'Topic'))
        self.can_magnetometer_table.setModel(self.can_magnetometer_model)
        header = form.can_magnetometer_table.horizontalHeader()
        for column, (width, mode) in enumerate(((72, 0), (72, 0), (72, 0), (112, 0), (72, 0), (72, 0), (72, 0), (72, 0), (72, 0), (88, 0), (96, 0), (88, 0), (88, 0), (52, 0), (72, 0), (220, 0))):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeMode(mode))
            header.resizeSection(column, width)
        for column in (12,):
            header.setSectionHidden(column, True)
        self.px4_parameter_models = {"left": {}, "right": {}}
        for side, tables in self.px4_parameter_tables.items():
            for key, table in tables.items():
                model = _table_model(self, ("Parameter", "Current", "Reset target", "Contract"))
                self.px4_parameter_models[side][key] = model
                table.setModel(model)
                for column in range(3):
                    table.horizontalHeader().setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeToContents)
        self.px4_parameter_group_models = self.px4_parameter_models
        self._mag_status_auto_refresh_attempts: dict[str, tuple[int, float]] = {}
        self._board_rotation_auto_refresh_attempts: dict[str, tuple[int, float]] = {}
        self._mag_priority_dirty: dict[str, set[str]] = {"left": set(), "right": set()}
        self._mag_priority_apply_pending = {"left": False, "right": False}
        self._mag_priority_voter_gap_instances: dict[str, tuple[int, ...]] = {"left": (), "right": ()}
        self._mag_priority_voter_gap_unknown_instances: dict[str, tuple[int, ...]] = {"left": (), "right": ()}
        self.magnetometer_table.clicked.connect(self._inspect_magnetometer_row)
        self.can_magnetometer_table.clicked.connect(self._inspect_can_magnetometer_row)
        for button in form.findChildren(QtWidgets.QPushButton):
            command = button.property("commandId")
            if command:
                button.setEnabled(invoke_command is not None)
                if invoke_command is not None:
                    if str(command).startswith("pixhawk.mag.apply."):
                        side = "left" if str(command).endswith(".p1") else "right"
                        button.clicked.connect(
                            lambda _checked=False, side=side, command=command: self._invoke_mag_priority_apply(side, command)
                        )
                    else:
                        button.clicked.connect(lambda _checked=False, command=command: invoke_command(command))
        for side, combos in self.mag_priority_combos.items():
            self.mag_priority_discard_buttons[side].clicked.connect(
                lambda _checked=False, side=side: self._discard_mag_priority_draft(side)
            )
            for slot_key, combo in combos.items():
                combo.currentIndexChanged.connect(
                    lambda _index, combo=combo: combo.setToolTip(
                        f"PX4 priority: {mag_priority_display(combo.currentData())}"
                    )
                )
                combo.currentIndexChanged.connect(
                    lambda _index, side=side, slot_key=slot_key: self._on_mag_priority_combo_changed(side, slot_key)
                )
        if isinstance(parent, QtWidgets.QWidget):
            for group in (
                self.magnetometer_group,
                self.mag_priority_group,
                self.can_magnetometer_group,
                self.parameter_group,
            ):
                group.setParent(parent)

    @staticmethod
    def _mag_role_for_device_id(device_id: int | None) -> str | None:
        if device_id == BOARD_INTERNAL_MAG_ID:
            return "BOARD"
        if device_id in RM3100_PRIMARY_IDS:
            return "CAN1"
        if device_id in RM3100_BACKUP_IDS:
            return "CAN2"
        if device_id == GPS1_MAG_ID:
            return "GPS1_MAG"
        if device_id == GPS2_MAG_ID:
            return "GPS2_MAG"
        return None


    def _invoke_mag_priority_apply(self, side: str, command_id: str) -> bool:
        if self._invoke_command is None:
            return False
        self._mag_priority_apply_pending[side] = True
        self._set_mag_priority_side_busy(side, True)
        started = bool(self._invoke_command(command_id))
        if not started:
            self._mag_priority_apply_pending[side] = False
            self._set_mag_priority_side_busy(side, False)
        return started

    def _set_mag_priority_side_busy(self, side: str, busy: bool) -> None:
        for combo in self.mag_priority_combos.get(side, {}).values():
            combo.setEnabled(not busy)
        for buttons in (
            self.mag_priority_refresh_buttons,
            self.mag_priority_discard_buttons,
            self.mag_priority_apply_buttons,
        ):
            button = buttons.get(side)
            if button is not None:
                button.setEnabled(not busy)

    @staticmethod
    def _priority_readback(status: object, slot_key: str) -> int | None:
        text = str(slot_key)
        if not text.startswith("CAL_MAG"):
            return None
        try:
            slot_number = int(text[len("CAL_MAG"):])
        except (TypeError, ValueError):
            return None
        for slot in getattr(status, "slots", ()) or ():
            if getattr(slot, "slot", None) == slot_number:
                value = getattr(slot, "priority", None)
                return None if value is None else int(value)
        params = getattr(status, "params", {})
        if isinstance(params, Mapping):
            value = params.get(f"CAL_MAG{slot_number}_PRIO")
            return None if value is None else int(value)
        return None

    @staticmethod
    def _set_priority_from_readback(combo: QtWidgets.QComboBox, value: int | float | None) -> None:
        # Programmatic readback must not be mistaken for an operator edit.
        previous = combo.blockSignals(True)
        try:
            _set_px4_priority_combo_value(combo, value)
        finally:
            combo.blockSignals(previous)

    def _on_mag_priority_combo_changed(self, side: str, slot_key: str) -> None:
        combo = self.mag_priority_combos.get(side, {}).get(slot_key)
        if combo is None:
            return
        inputs = self._read_inputs()
        status = inputs.status_cache.get(side) if isinstance(inputs.status_cache, Mapping) else None
        actual = self._priority_readback(status, slot_key) if status is not None else None
        dirty = self._mag_priority_dirty.setdefault(side, set())
        if actual is not None and actual == _px4_priority_combo_value(combo):
            dirty.discard(slot_key)
        else:
            dirty.add(slot_key)
        self._refresh_mag_priority_preview(side)
        self._render_mag_priority_draft_status(side)

    def _render_mag_priority_draft_status(self, side: str) -> None:
        label = self.mag_priority_draft_labels.get(side)
        if label is None:
            return
        dirty = self._mag_priority_dirty.get(side, set())
        if not dirty:
            label.setText(" ")
            label.setToolTip("미적용 Priority 편집 없음")
            return
        rendered = ", ".join(
            f"{key}={_px4_priority_combo_value(self.mag_priority_combos[side][key])}"
            for key in MAG_PRIORITY_KEYS
            if key in dirty
        )
        label.setText(f"Draft pending Apply · {len(dirty)} slot(s)")
        label.setToolTip(f"Apply to send: {rendered}")

    def _discard_mag_priority_draft(self, side: str) -> None:
        inputs = self._read_inputs()
        status = inputs.status_cache.get(side) if isinstance(inputs.status_cache, Mapping) else None
        if status is None:
            label = self.mag_priority_status_labels.get(side)
            if label is not None:
                side_label = "P1" if side == "left" else "P2"
                label.setText(f"{side_label}: PARAM readback required before Discard")
            return
        controls = self.mag_priority_combos.get(side, {})
        discarded: set[str] = set()
        for slot_key, combo in controls.items():
            value = self._priority_readback(status, slot_key)
            if value is None:
                continue
            self._set_priority_from_readback(combo, value)
            discarded.add(slot_key)
        if not discarded:
            return
        self._mag_priority_dirty.setdefault(side, set()).difference_update(discarded)
        self._refresh_mag_priority_preview(side)
        self._refresh_mag_priority_group({})

    def mag_priority_payload(self, side: str) -> dict[str, int]:
        controls = self.mag_priority_combos.get(str(side), {})
        return {slot_key: _px4_priority_combo_value(combo) for slot_key, combo in controls.items()}

    @staticmethod
    def _mag_preview_role_label(role: str | None) -> str:
        return str(role or "UNKNOWN")

    def _refresh_mag_priority_preview(self, side: str) -> None:
        label = self.mag_priority_preview_labels.get(str(side))
        if label is None:
            return
        state, preview = self._read_preview(str(side), self.mag_priority_payload(str(side)))
        warning = self.mag_priority_warning_labels.get(str(side))
        if state == "TOPOLOGY WAITING":
            self._mag_priority_voter_gap_instances[str(side)] = ()
            self._mag_priority_voter_gap_unknown_instances[str(side)] = ()
            label.setText("Selection preview: topology readback 대기 중")
            if warning is not None:
                warning.setText("Gap check waiting for topology")
                warning.setToolTip("Topology readback이 없어 PX4 v1.17 voter gap을 검사하지 못했습니다.")
            return
        if state == "PARAM WAITING":
            self._mag_priority_voter_gap_instances[str(side)] = ()
            self._mag_priority_voter_gap_unknown_instances[str(side)] = ()
            label.setText("Selection preview: CAL_MAG PARAM readback 대기 중")
            if warning is not None:
                warning.setText("Gap check waiting for CAL_MAG PARAM")
                warning.setToolTip("CAL_MAG ID/priority readback이 없어 PX4 v1.17 voter gap을 검사하지 못했습니다.")
            return
        voter_gaps = tuple(getattr(preview, "priority_voter_gap_instances", ()) or ())
        self._mag_priority_voter_gap_instances[str(side)] = voter_gaps
        unknown_gaps = tuple(getattr(preview, "priority_voter_gap_unknown_instances", ()) or ())
        self._mag_priority_voter_gap_unknown_instances[str(side)] = unknown_gaps
        topology_text = " · ".join(
            f"i{candidate.instance}:{candidate.role} id={candidate.device_id} "
            f"CAL_MAG{candidate.cal_slot if candidate.cal_slot is not None else '-'}"
            for candidate in preview.candidates
        )
        if preview.predicted_device_id is None:
            candidate_text = "N/A"
        else:
            predicted_slot = next(
                (
                    candidate.cal_slot
                    for candidate in preview.candidates
                    if candidate.device_id == preview.predicted_device_id
                ),
                None,
            )
            candidate_text = (
                f"{self._mag_preview_role_label(preview.predicted_role)} "
                f"id={preview.predicted_device_id} inst={preview.predicted_instance} "
                f"CAL_MAG{predicted_slot if predicted_slot is not None else '-'} "
                f"prio={preview.predicted_priority}"
            )
            if preview.priority_tie:
                candidate_text += " (tie)"
        actual_text = (
            "N/A"
            if preview.actual_device_id is None
            else f"{self._mag_preview_role_label(preview.actual_role)} id={preview.actual_device_id}"
        )
        label.setText(f"Topology: {topology_text}\nPriority candidate: {candidate_text}\nActual PX4: {actual_text}")
        if warning is not None:
            if voter_gaps:
                slots_by_instance = {candidate.instance: candidate.cal_slot for candidate in preview.candidates}
                instances = ", ".join(
                    f"i{instance}/MAG{slots_by_instance[instance]}"
                    if slots_by_instance.get(instance) is not None
                    else f"i{instance}"
                    for instance in voter_gaps
                )
                warning.setText(f"PX4 1.17 gap: {instances} off; set Min (1)")
                warning.setToolTip(
                    "활성 후보보다 낮은 sensor_mag instance가 priority 0입니다. "
                    "해당 MAG를 Min (1) 이상으로 설정하면 voter index가 유지됩니다."
                )
            elif unknown_gaps:
                instances = ", ".join(f"i{instance}" for instance in unknown_gaps)
                warning.setText(f"Gap check incomplete: {instances}; refresh topology")
                warning.setToolTip(
                    "후보보다 낮은 sensor_mag instance readback이 없습니다. "
                    "Topology refresh 후 voter guard를 다시 확인하세요. Apply는 기존 gates로 진행합니다."
                )
            else:
                warning.setText("PX4 1.17 voter gap: none detected")
                warning.setToolTip("현재 topology와 staged priority에서 알려진 sparse-voter gap이 없습니다.")

    @staticmethod
    def _board_rotation_known(side: str, attitude_cache: object, rotation_cache: object) -> bool:
        """True when SENS_BOARD_ROT is available from either readback path."""

        for cache in (attitude_cache, rotation_cache):
            if not isinstance(cache, Mapping):
                continue
            entry = cache.get(side)
            params = getattr(entry, "params", entry)
            if not isinstance(params, Mapping):
                continue
            value = params.get("SENS_BOARD_ROT")
            try:
                if value is not None and math.isfinite(float(value)):
                    return True
            except (TypeError, ValueError):
                continue
        return False

    def _refresh_mag_priority_group(self, snapshot: dict[str, RuntimeStatus]) -> None:
        inputs = self._read_inputs()
        status_cache = inputs.status_cache
        connected = inputs.connected
        # The automatic readback waits for a verified heartbeat. The PARAM
        # worker shares the serial link with the sensor_mag shell poller, and
        # a request fired during the CONNECTING window is exactly what failed
        # in the field ("device reports readiness to read but returned no
        # data"), leaving the CAL_MAG-slot table empty for the whole session.
        verified = inputs.verified
        parameter_threads = inputs.parameter_sides
        incomplete_cache = inputs.incomplete_cache
        attitude_cache = inputs.attitude_cache
        rotation_cache = inputs.rotation_cache
        now_s = inputs.now_s
        max_attempts = self.MAG_STATUS_AUTO_REFRESH_MAX_ATTEMPTS
        for side, label, code in (("left", "P1", "p1"), ("right", "P2", "p2")):
            status = status_cache.get(side) if isinstance(status_cache, Mapping) else None
            missing_params = (
                tuple(incomplete_cache.get(side) or ())
                if isinstance(incomplete_cache, Mapping) and status is not None
                else ()
            )
            controls = self.mag_priority_combos.get(side, {})
            if side not in connected:
                self._mag_status_auto_refresh_attempts.pop(side, None)
                self._board_rotation_auto_refresh_attempts.pop(side, None)
            elif (
                (status is None or missing_params)
                and self._invoke_command_available
                and side in verified
                and side not in parameter_threads
            ):
                # The CAL_MAG-slot table and priority combos need a live
                # CAL_MAGn_ID/_PRIO param readback to know which physical
                # sensor sits in which slot. Request it automatically so the
                # table doesn't sit empty until the operator presses
                # "Refresh MAG", and retry a bounded number of times when a
                # readback fails (no status lands and the worker is gone).
                attempts, last_request_s = self._mag_status_auto_refresh_attempts.get(side, (0, float("-inf")))
                if attempts < max_attempts and now_s - last_request_s >= self.MAG_STATUS_AUTO_REFRESH_RETRY_S:
                    self._mag_status_auto_refresh_attempts[side] = (attempts + 1, now_s)
                    self._invoke_command(f"pixhawk.mag.refresh.{code}")
                    logger = self._log
                    if callable(logger):
                        reason = f"; {len(missing_params)}개 PARAM 미수신" if missing_params else ""
                        logger(
                            f"[PX4 MAG] {label} CAL_MAG PARAM 자동 readback 요청 "
                            f"({attempts + 1}/{max_attempts}{reason})"
                        )
            elif (
                status is not None
                and self._invoke_command_available
                and side in verified
                and side not in parameter_threads
                and not self._board_rotation_known(side, attitude_cache, rotation_cache)
            ):
                # Internal (board/GPS) MAG rows cannot show a CAL_MAG-corrected
                # yaw until SENS_BOARD_ROT is known. The full attitude-cal
                # readback is operator-driven, so request the single PARAM here
                # with the same bounded cadence as the CAL_MAG readback.
                attempts, last_request_s = self._board_rotation_auto_refresh_attempts.get(side, (0, float("-inf")))
                if attempts < max_attempts and now_s - last_request_s >= self.MAG_STATUS_AUTO_REFRESH_RETRY_S:
                    self._board_rotation_auto_refresh_attempts[side] = (attempts + 1, now_s)
                    self._invoke_command(f"pixhawk.board_rotation.refresh.{code}")
                    logger = self._log
                    if callable(logger):
                        logger(f"[PX4 MAG] {label} SENS_BOARD_ROT 자동 readback 요청 ({attempts + 1}/{max_attempts})")
            active_operation = side in parameter_threads
            if self._mag_priority_apply_pending.get(side) and not active_operation:
                self._mag_priority_apply_pending[side] = False
            busy = active_operation or self._mag_priority_apply_pending.get(side, False)
            if status is not None:
                for slot_key, combo in controls.items():
                    actual_priority = self._priority_readback(status, slot_key)
                    if actual_priority is None:
                        continue
                    dirty = self._mag_priority_dirty.setdefault(side, set())
                    if slot_key in dirty:
                        if busy or _px4_priority_combo_value(combo) != actual_priority:
                            continue
                        dirty.discard(slot_key)
                    self._set_priority_from_readback(combo, actual_priority)
            verification_reader = self._read_verification
            verification = {}
            if callable(verification_reader):
                try:
                    value = verification_reader(side)
                    verification = dict(value) if isinstance(value, Mapping) else {}
                except (RuntimeError, TypeError, ValueError):
                    verification = {}
            if verification.get("verification_started_s") is not None:
                status_text = (
                    f"{label}: {verification.get('state', 'VERIFYING')} · "
                    f"expected={verification.get('expected_device_id', '--')} · "
                    f"actual={verification.get('actual_device_id', '--')} · "
                    f"ATT={_display_number(verification.get('attitude_age_s'), digits=2, suffix='s')}\n"
                    f"{verification.get('reason', '')}"
                )
            elif status is None:
                attempts, _last_request_s = self._mag_status_auto_refresh_attempts.get(side, (0, 0.0))
                if attempts >= max_attempts:
                    status_text = (
                        f"{label}: PARAM readback 자동 시도 {attempts}회 실패 · "
                        "Refresh MAG를 눌러 CAL_MAG PARAM을 다시 읽으세요"
                    )
                elif attempts:
                    status_text = f"{label}: PARAM readback 대기 중 (자동 시도 {attempts}/{max_attempts})"
                else:
                    status_text = f"{label}: PARAM readback 대기 중"
            elif missing_params:
                attempts, _last_request_s = self._mag_status_auto_refresh_attempts.get(side, (0, 0.0))
                if attempts >= max_attempts:
                    status_text = (
                        f"{label}: PARAM readback 불완전 ({len(missing_params)}개 미수신) · "
                        f"자동 시도 {attempts}회 소진 · Refresh MAG를 눌러 다시 읽으세요"
                    )
                else:
                    status_text = (
                        f"{label}: PARAM readback 불완전 ({len(missing_params)}개 미수신) · "
                        f"자동 재시도 대기 ({attempts}/{max_attempts})"
                    )
            else:
                active = inputs.active_ids.get(side)
                status_text = (
                    f"{label}: PARAM VERIFIED · actual={active if active is not None else '--'} · "
                    "Apply & Verify를 눌러 active selection을 확인하세요."
                )
            self.mag_priority_status_labels[side].setText(status_text)
            self._render_mag_priority_draft_status(side)
            self._refresh_mag_priority_preview(side)
            latest = self._read_inputs()
            active_operation = side in latest.parameter_sides
            if self._mag_priority_apply_pending.get(side) and not active_operation:
                self._mag_priority_apply_pending[side] = False
            busy = active_operation or self._mag_priority_apply_pending.get(side, False)
            armed = side in latest.armed_sides
            enabled = (
                self._invoke_command_available
                and side in connected
                and not busy
                and not self._recording_locked
            )
            self.mag_priority_refresh_buttons[side].setEnabled(enabled)
            self.mag_priority_discard_buttons[side].setEnabled(
                bool(self._mag_priority_dirty.get(side)) and status is not None and not busy
            )
            for combo in controls.values():
                combo.setEnabled(not busy)
            voter_gap_instances = self._mag_priority_voter_gap_instances.get(side, ())
            self.mag_priority_apply_buttons[side].setEnabled(
                enabled and not armed and not voter_gap_instances
            )
            if armed:
                tooltip = "Disarm required"
            elif voter_gap_instances:
                tooltip = "PX4 v1.17 voter gap: lower sensor priorities를 Min (1) 이상으로 바꾸세요."
            else:
                tooltip = ""
            self.mag_priority_apply_buttons[side].setToolTip(tooltip or "이 side만 Priority를 적용하고 검증합니다.")


    def _refresh_px4_parameter_group(self, snapshot: dict[str, RuntimeStatus]) -> None:
        inputs = self._read_inputs()
        mag_cache = inputs.status_cache
        attitude_cache = inputs.attitude_cache
        errors = inputs.parameter_errors
        reset_errors = inputs.reset_errors
        for side, label in (("left", "P1"), ("right", "P2")):
            mag_status = mag_cache.get(side) if isinstance(mag_cache, Mapping) else None
            attitude_status = attitude_cache.get(side) if isinstance(attitude_cache, Mapping) else None
            params: dict[str, object] = {}
            if attitude_status is not None:
                params.update(getattr(attitude_status, "params", {}) or {})
            if mag_status is not None:
                params.update(getattr(mag_status, "params", {}) or {})
            button = self.px4_parameter_refresh_buttons[side]
            state = snapshot.get(f"pixhawk.{side}")
            connected = state is not None and str(getattr(state, "state", "")).upper() in {
                "LIVE",
                "CONNECTED",
                "STARTING",
            }
            connected = connected or side in inputs.connected
            active = side in inputs.parameter_sides or side in inputs.shell_sides
            button.setEnabled(
                self._invoke_command_available and connected and not active and not self._recording_locked
            )
            if side in errors:
                status_text = f"{label}: PARAM readback failed · {errors[side]}"
            elif mag_status is None and attitude_status is None:
                status_text = f"{label}: PARAM readback 대기 중"
            else:
                status_text = (
                    f"{label}: QGC calibration 결과 표시 중 · "
                    "Accelerometer/Gyroscope/Compass/Level groups · PX4 sensor axes"
                )
            self.px4_parameter_status_labels[side].setText(status_text)
            for calibration in PX4_CALIBRATION_GROUPS:
                group_params = tuple(params.get(name) for name in calibration.parameter_names)
                rows = tuple(
                    (
                        name,
                        _format_px4_parameter_value(name, params.get(name)),
                        "PX4 firmware default" if calibration.resettable else "Manual / no reset",
                        calibration.contract,
                    )
                    for name in calibration.parameter_names
                )
                _sync_table_rows(self.px4_parameter_group_models[side][calibration.key], rows)
                reset_button = self.px4_parameter_reset_buttons[side][calibration.key]
                disarmed = side not in inputs.armed_sides
                reset_button.setEnabled(
                    calibration.resettable
                    and self._invoke_command_available
                    and connected
                    and disarmed
                    and not active
                    and not self._recording_locked
                )
                group_error = reset_errors.get(f"{side}:{calibration.key}")
                if group_error:
                    group_status = f"{label}: reset failed · {group_error}"
                elif not params or all(value is None for value in group_params):
                    group_status = f"{label}: PARAM readback 대기 중"
                elif calibration.resettable:
                    group_status = (
                        f"{label}: {calibration.label} readback 완료 · Reset은 PX4 firmware default를 사용합니다."
                    )
                else:
                    group_status = f"{label}: {calibration.label}은 수동 파라미터입니다."
                self.px4_parameter_group_status_labels[side][calibration.key].setText(group_status)

    def _inspect_magnetometer_row(self, index: QtCore.QModelIndex) -> None:
        if index.row() < 0 or index.row() >= self.magnetometer_model.rowCount():
            return
        rows_per_side = len(MAG_SLOTS)
        side = "left" if index.row() < rows_per_side else "right"
        view_index = index.row() % rows_per_side
        rows = self._read_cal_mag(side)
        if view_index >= len(rows):
            return
        row = rows[view_index]
        selection = f"{row.get('pixhawk', '--')} {row.get('cal_mag', '--')} · {row.get('sensor', 'Magnetometer')}"
        refresh = lambda side=side, source_index=view_index: self._cal_mag_magnetometer_inspection(side, source_index)
        fallback_source = "PX4 CAL_MAG slot readback"
        self.inspection_requested.emit(
            _live_inspection(
                {
                    "selection": selection,
                    "value": (
                        f"x/y/z={_display_number(row.get('xmag'))}/"
                        f"{_display_number(row.get('ymag'))}/"
                        f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                        + (
                            f"node={row.get('node_id', '--')} device={row.get('device_id', '--')} · "
                            if row.get("node_id") is not None
                            else f"device={row.get('device_id', '--')} · "
                        )
                        + f"norm={_display_number(row.get('norm'))} · "
                        + f"{_magnetometer_yaw_summary(row)}"
                    ),
                    "freshness": (
                        f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                        f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
                    ),
                    "contract": (
                        f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                        f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                        f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
                    ),
                    "verdict": (
                        f"{row.get('state', 'WAITING')} · Corrected {row.get('corrected_yaw_state', 'WAITING')}"
                    ),
                    "source": str(row.get("source", fallback_source)),
                },
                refresh,
            )
        )

    def _cal_mag_magnetometer_inspection(self, side: str, slot_index: int) -> dict[str, object]:
        rows = self._read_cal_mag(side)
        if slot_index < 0 or slot_index >= len(rows):
            return {"selection": "CAL_MAG", "verdict": "WAITING"}
        row = rows[slot_index]
        return {
            "selection": (
                f"{row.get('pixhawk', '--')} {row.get('cal_mag', '--')} · {row.get('sensor', 'Magnetometer')}"
            ),
            "value": (
                f"x/y/z={_display_number(row.get('xmag'))}/"
                f"{_display_number(row.get('ymag'))}/"
                f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                f"device={row.get('device_id', '--')} · "
                f"{_magnetometer_yaw_summary(row)}"
            ),
            "freshness": (
                f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                f"Priority={mag_priority_display(row.get('priority'))} · "
                f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
            ),
            "contract": (
                f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
            ),
            "verdict": (f"{row.get('state', 'WAITING')} · Corrected {row.get('corrected_yaw_state', 'WAITING')}"),
            "source": str(row.get("source", "PX4 CAL_MAG slot readback")),
        }

    def _inspect_can_magnetometer_row(self, index: QtCore.QModelIndex) -> None:
        if index.row() < 0 or index.row() >= self.can_magnetometer_model.rowCount():
            return
        side = "left" if index.row() < 2 else "right"
        port_index = index.row() % 2
        rows = self._read_can(side)
        if port_index >= len(rows):
            return
        row = rows[port_index]
        self.inspection_requested.emit(
            _live_inspection(
                {
                    "selection": f"{row.get('pixhawk', '--')} {row.get('can_port', '--')} magnetometer",
                    "value": (
                        f"x/y/z={_display_number(row.get('xmag'))}/"
                        f"{_display_number(row.get('ymag'))}/"
                        f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                        f"node={row.get('node_id', '--')} device={row.get('device_id', '--')} · "
                        f"{_magnetometer_yaw_summary(row)}"
                    ),
                    "freshness": (
                        f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                        f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
                    ),
                    "contract": (
                        f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                        f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                        f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
                    ),
                    "verdict": (
                        f"{row.get('state', 'WAITING')} · Corrected {row.get('corrected_yaw_state', 'WAITING')}"
                    ),
                    "source": str(row.get("source", "PX4 DEBUG_VECT CAN_MAG compatibility telemetry")),
                },
                lambda side=side, port_index=port_index: self._can_magnetometer_inspection(side, port_index),
            )
        )

    def _can_magnetometer_inspection(self, side: str, port_index: int) -> dict[str, object]:
        rows = self._read_can(side)
        if port_index < 0 or port_index >= len(rows):
            return {"selection": "CAN magnetometer", "verdict": "WAITING"}
        row = rows[port_index]
        return {
            "selection": f"{row.get('pixhawk', '--')} {row.get('can_port', '--')} magnetometer",
            "value": (
                f"x/y/z={_display_number(row.get('xmag'))}/"
                f"{_display_number(row.get('ymag'))}/"
                f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                f"{_magnetometer_yaw_summary(row)}"
            ),
            "freshness": (
                f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
            ),
            "contract": (
                f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
            ),
            "verdict": (f"{row.get('state', 'WAITING')} · Corrected {row.get('corrected_yaw_state', 'WAITING')}"),
            "source": str(row.get("source", "PX4 DEBUG_VECT CAN_MAG compatibility telemetry")),
        }

    def _magnetometer_inspection(self, side: str, stream_index: int) -> dict[str, object]:
        rows = self._read_legacy(side)
        if stream_index < 0 or stream_index >= len(rows):
            return {"selection": "Magnetometer", "verdict": "WAITING"}
        row = rows[stream_index]
        return {
            "selection": f"{row.get('pixhawk', '--')} {row.get('imu', '--')} magnetometer",
            "value": (
                f"x/y/z={_display_number(row.get('xmag'))}/"
                f"{_display_number(row.get('ymag'))}/"
                f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                f"{_magnetometer_yaw_summary(row)}"
            ),
            "freshness": (
                f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
            ),
            "contract": (
                f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
            ),
            "verdict": (f"{row.get('state', 'WAITING')} · Corrected {row.get('corrected_yaw_state', 'WAITING')}"),
            "source": str(row.get("source", "MAVLink SCALED_IMU telemetry")),
        }

    def refresh(self, snapshot: dict[str, RuntimeStatus]):
        recording = snapshot.get("recording")
        self._recording_locked = recording is not None and recording.state in {"ACTIVE", "FINALIZING"}
        magnetometer_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            for row in self._read_cal_mag(side):
                magnetometer_rows.append(
                    (
                        row.get("pixhawk", "--"),
                        row.get("cal_mag", "--"),
                        row.get("sensor", "--"),
                        row.get("device_id", "--"),
                        row.get("state", "WAITING"),
                        _display_number(row.get("xmag")),
                        _display_number(row.get("ymag")),
                        _display_number(row.get("zmag")),
                        _display_number(row.get("norm")),
                        _display_signed_deg(row.get("raw_yaw_deg")),
                        _display_magnetometer_corrected_yaw(row),
                        _display_magnetometer_px4_yaw(row),
                        _display_signed_deg(row.get("delta_yaw_deg")),
                        mag_priority_display(row.get("priority")),
                        row.get("unit", "--"),
                        _display_number(row.get("age_s"), digits=2, suffix="s"),
                        row.get("topic", "--"),
                    )
                )
        _sync_table_rows(self.magnetometer_model, tuple(magnetometer_rows))
        ui_yaw_signature = tuple(
            (
                str(row[0]),
                str(row[1]),
                str(row[9]),
                str(row[10]),
                str(row[11]),
                str(row[12]),
                str(row[16]),
            )
            for row in magnetometer_rows
        )
        if len(ui_yaw_signature) == 8 and all(row[2] != "--" for row in ui_yaw_signature):
            previous_signature = getattr(self, "_last_ui_yaw_signature", None)
            if previous_signature != ui_yaw_signature:
                self._last_ui_yaw_signature = ui_yaw_signature
                logger = self._log
                if callable(logger):
                    logger(
                        "[픽스호크 UI] P1/P2 5-MAG yaw cells populated: "
                        + "; ".join(
                            f"{pixhawk}/{mag} RAW={raw} CORR={corrected} PX4={px4} Δ={delta} topic={topic}"
                            for pixhawk, mag, raw, corrected, px4, delta, topic in ui_yaw_signature
                        )
                    )
        can_magnetometer_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            for row in self._read_can(side):
                can_magnetometer_rows.append(
                    (
                        row.get("pixhawk", "--"),
                        row.get("can_port", "--"),
                        row.get("node_id", "--"),
                        row.get("device_id", "--"),
                        row.get("state", "WAITING"),
                        _display_number(row.get("xmag")),
                        _display_number(row.get("ymag")),
                        _display_number(row.get("zmag")),
                        _display_number(row.get("norm")),
                        _display_signed_deg(row.get("raw_yaw_deg")),
                        _display_magnetometer_corrected_yaw(row),
                        _display_magnetometer_px4_yaw(row),
                        _display_signed_deg(row.get("delta_yaw_deg")),
                        row.get("unit", "--"),
                        _display_number(row.get("age_s"), digits=2, suffix="s"),
                        row.get("topic", "--"),
                    )
                )
        _sync_table_rows(self.can_magnetometer_model, tuple(can_magnetometer_rows))
        self._refresh_mag_priority_group(snapshot)
        self._refresh_px4_parameter_group(snapshot)
