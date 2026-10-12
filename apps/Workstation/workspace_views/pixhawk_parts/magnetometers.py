"""Magnetometer tables, readback controls and PX4 calibration parameter views."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.application.status_model import RuntimeStatus
from ..shared import _command_button, _display_number, _live_inspection, _sync_table_rows, _table_model

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
from ..px4_widgets import _build_px4_priority_combo, _px4_priority_combo_value, _set_px4_priority_combo_value
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
        magnetometer_group = QtWidgets.QGroupBox("Magnetometer by CAL_MAG slot")
        magnetometer_layout = QtWidgets.QVBoxLayout(magnetometer_group)
        magnetometer_hint = QtWidgets.QLabel(
            "CAL_MAG0..3 순서 · Device ID로 실제 센서 sample 연결 · "
            "RAW yaw=atan2(-y,x) · Corrected yaw=CAL_MAG 보정값 · "
            "PX4 yaw=편각 보정 ATTITUDE 추정값"
        )
        magnetometer_hint.setObjectName("statusStrip")
        magnetometer_hint.setWordWrap(True)
        magnetometer_layout.addWidget(magnetometer_hint)
        self.magnetometer_model = _table_model(
            self,
            (
                "Pixhawk",
                "CAL_MAG",
                "Sensor",
                "Device ID",
                "State",
                "xmag",
                "ymag",
                "zmag",
                "Norm",
                "RAW yaw°",
                "Corrected yaw°",
                "PX4 yaw°",
                "Δyaw°",
                "Priority",
                "Unit",
                "Age",
                "Topic",
            ),
        )
        self.magnetometer_table = QtWidgets.QTableView()
        self.magnetometer_table.setModel(self.magnetometer_model)
        self.magnetometer_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.magnetometer_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.magnetometer_table.verticalHeader().setVisible(False)
        # P1/P2 each expose the four configured CAL_MAG slots. Keep all
        # eight rows visible so the table does not acquire an inner scrollbar.
        self.magnetometer_table.setMinimumHeight(300)
        self.magnetometer_table.setMaximumHeight(300)
        self.magnetometer_table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        magnetometer_header = self.magnetometer_table.horizontalHeader()
        magnetometer_header.setStretchLastSection(False)
        magnetometer_header.setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        for column, width in enumerate((72, 76, 190, 100, 72, 72, 72, 72, 72, 88, 96, 88, 88, 88, 52, 72, 190)):
            magnetometer_header.resizeSection(column, width)
        magnetometer_header.setSectionHidden(10, False)
        magnetometer_header.setSectionHidden(11, False)
        magnetometer_header.setSectionHidden(12, True)
        self.magnetometer_table.clicked.connect(self._inspect_magnetometer_row)
        magnetometer_layout.addWidget(self.magnetometer_table)
        self.magnetometer_group = magnetometer_group

        self._build_mag_priority_group()

        can_magnetometer_group = QtWidgets.QGroupBox("CAN magnetometer (dedicated MAVLink)")
        can_magnetometer_layout = QtWidgets.QVBoxLayout(can_magnetometer_group)
        can_magnetometer_hint = QtWidgets.QLabel(
            "CAN1/CAN2 값은 PX4 DEBUG_VECT CAN_MAG 호환 메시지가 수신될 때만 표시 · "
            "RAW yaw=센서축 진단값 · Corrected yaw=CAL_MAG 보정값 · "
            "PX4 yaw=편각 보정 ATTITUDE 추정값"
        )
        can_magnetometer_hint.setObjectName("statusStrip")
        can_magnetometer_hint.setWordWrap(True)
        can_magnetometer_layout.addWidget(can_magnetometer_hint)
        self.can_magnetometer_model = _table_model(
            self,
            (
                "Pixhawk",
                "CAN port",
                "Node ID",
                "Device ID",
                "State",
                "xmag",
                "ymag",
                "zmag",
                "Norm",
                "RAW yaw°",
                "Corrected yaw°",
                "PX4 yaw°",
                "Δyaw°",
                "Unit",
                "Age",
                "Topic",
            ),
        )
        self.can_magnetometer_table = QtWidgets.QTableView()
        self.can_magnetometer_table.setModel(self.can_magnetometer_model)
        self.can_magnetometer_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.can_magnetometer_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.can_magnetometer_table.verticalHeader().setVisible(False)
        self.can_magnetometer_table.setMaximumHeight(160)
        self.can_magnetometer_table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        can_magnetometer_header = self.can_magnetometer_table.horizontalHeader()
        can_magnetometer_header.setStretchLastSection(False)
        can_magnetometer_header.setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        for column, width in enumerate((72, 72, 72, 112, 72, 72, 72, 72, 72, 88, 96, 88, 88, 52, 72, 220)):
            can_magnetometer_header.resizeSection(column, width)
        can_magnetometer_header.setSectionHidden(10, False)
        can_magnetometer_header.setSectionHidden(11, False)
        can_magnetometer_header.setSectionHidden(12, True)
        self.can_magnetometer_table.clicked.connect(self._inspect_can_magnetometer_row)
        can_magnetometer_layout.addWidget(self.can_magnetometer_table)
        self.can_magnetometer_group = can_magnetometer_group

        self._build_px4_parameter_group()
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

    def _build_mag_priority_group(self) -> None:
        group = QtWidgets.QGroupBox("MAG Priority / active selection")
        group_layout = QtWidgets.QVBoxLayout(group)
        # side -> (attempt count, monotonic time of the last automatic request).
        self._mag_status_auto_refresh_attempts: dict[str, tuple[int, float]] = {}
        self._board_rotation_auto_refresh_attempts: dict[str, tuple[int, float]] = {}
        hint = QtWidgets.QLabel(
            "P1/P2는 독립적으로 적용됩니다. CAL_MAGn_PRIO를 직접 수정하며, "
            "Apply 후 PARAM readback, 실제 active device ID, fresh ATTITUDE가 모두 확인되어야 "
            "ACTIVE VERIFIED가 됩니다."
        )
        hint.setObjectName("statusStrip")
        hint.setWordWrap(True)
        group_layout.addWidget(hint)
        cards = QtWidgets.QHBoxLayout()
        self.mag_priority_combos: dict[str, dict[str, QtWidgets.QComboBox]] = {}
        self.mag_priority_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.mag_priority_preview_labels: dict[str, QtWidgets.QLabel] = {}
        self.mag_priority_refresh_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.mag_priority_apply_buttons: dict[str, QtWidgets.QPushButton] = {}
        defaults = {key: 0 for key in MAG_PRIORITY_KEYS}
        for side, label, code in (("left", "P1", "p1"), ("right", "P2", "p2")):
            card = QtWidgets.QGroupBox(label)
            card_layout = QtWidgets.QGridLayout(card)
            card_layout.setContentsMargins(6, 6, 6, 6)
            controls = QtWidgets.QHBoxLayout()
            refresh = _command_button(
                "Refresh MAG",
                f"pixhawk.mag.refresh.{code}",
                self._invoke_command,
            )
            apply = _command_button(
                "Apply & Verify",
                f"pixhawk.mag.apply.{code}",
                self._invoke_command,
            )
            refresh.setToolTip("이 side의 CAL_MAG Priority를 다시 읽습니다.")
            apply.setToolTip("이 side만 Priority를 적용하고 실제 active MAG/ATTITUDE를 검증합니다.")
            controls.addWidget(refresh)
            controls.addWidget(apply)
            controls.addStretch(1)
            card_layout.addLayout(controls, 0, 0, 1, 2)
            self.mag_priority_refresh_buttons[side] = refresh
            self.mag_priority_apply_buttons[side] = apply
            combos: dict[str, QtWidgets.QComboBox] = {}
            for row, slot_key in enumerate(MAG_PRIORITY_KEYS, start=1):
                combo = _build_px4_priority_combo(defaults[slot_key])
                combo.setAccessibleName(f"{label} {slot_key} priority")
                combo.setToolTip(f"{slot_key}_PRIO 값을 설정합니다. 센서 identity는 {slot_key}_ID에서 읽습니다.")
                card_layout.addWidget(QtWidgets.QLabel(slot_key), row, 0)
                card_layout.addWidget(combo, row, 1)
                combos[slot_key] = combo
            self.mag_priority_combos[side] = combos
            for role, combo in combos.items():
                combo.currentIndexChanged.connect(lambda _index, side=side: self._refresh_mag_priority_preview(side))
            status = QtWidgets.QLabel("PARAM readback 대기 중")
            status.setObjectName("statusStrip")
            status.setWordWrap(True)
            status.setMinimumHeight(42)
            card_layout.addWidget(status, 6, 0, 1, 2)
            self.mag_priority_status_labels[side] = status
            preview = QtWidgets.QLabel("Selection preview: topology readback 대기 중")
            preview.setObjectName("statusStrip")
            preview.setWordWrap(True)
            preview.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            preview.setMinimumHeight(58)
            preview.setMaximumHeight(92)
            card_layout.addWidget(preview, 7, 0, 1, 2)
            self.mag_priority_preview_labels[side] = preview
            cards.addWidget(card, 1)
        group_layout.addLayout(cards)
        self.mag_priority_group = group

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
        if state == "TOPOLOGY WAITING":
            label.setText("Selection preview: topology readback 대기 중")
            return
        if state == "PARAM WAITING":
            label.setText("Selection preview: CAL_MAG PARAM readback 대기 중")
            return
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
            if status is not None:
                slots_data = getattr(status, "slots", ()) or ()
                if slots_data:
                    for slot in slots_data:
                        slot_key = mag_priority_slot_key(getattr(slot, "slot", -1))
                        if slot_key in controls:
                            _set_px4_priority_combo_value(controls[slot_key], getattr(slot, "priority", None))
                else:
                    # ``status.slots`` is derived from ``status.params`` (see
                    # build_mag_status). When only the raw params are available
                    # (e.g. before the slots readback lands) fall back to the
                    # same CAL_MAGn_PRIO source so the combos still reflect the
                    # live PX4 state instead of staying at their placeholder 0.
                    params = getattr(status, "params", {})
                    if isinstance(params, Mapping):
                        for slot_number in MAG_SLOTS:
                            slot_key = mag_priority_slot_key(slot_number)
                            if slot_key in controls:
                                _set_px4_priority_combo_value(
                                    controls[slot_key], params.get(f"CAL_MAG{slot_number}_PRIO")
                                )
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
            self._refresh_mag_priority_preview(side)
            latest = self._read_inputs()
            active_operation = side in latest.parameter_sides
            armed = side in latest.armed_sides
            enabled = (
                self._invoke_command_available
                and side in connected
                and not active_operation
                and not self._recording_locked
            )
            self.mag_priority_refresh_buttons[side].setEnabled(enabled)
            self.mag_priority_apply_buttons[side].setEnabled(enabled and not armed)
            tooltip = "Disarm required" if armed else ""
            self.mag_priority_apply_buttons[side].setToolTip(tooltip or "이 side만 Priority를 적용하고 검증합니다.")

    def _build_px4_parameter_group(self) -> None:
        group = QtWidgets.QGroupBox("Pixhawk Parameters · calibration-owned groups")
        parameter_group_layout = QtWidgets.QVBoxLayout(group)
        hint = QtWidgets.QLabel(
            "QGC에서 Pixhawk에 저장한 값을 캘리브레이션별로 나누어 표시합니다. "
            "각 초기화 버튼은 해당 그룹의 파라미터만 PX4 firmware default로 되돌리며, "
            "값은 PX4 센서축/보드 기준 raw parameter라 NED/ENU/카메라 좌표로 변환하지 않습니다."
        )
        hint.setObjectName("statusStrip")
        hint.setWordWrap(True)
        parameter_group_layout.addWidget(hint)

        self.px4_parameter_models: dict[str, dict[str, QtGui.QStandardItemModel]] = {}
        self.px4_parameter_tables: dict[str, dict[str, QtWidgets.QTableView]] = {}
        self.px4_parameter_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.px4_parameter_refresh_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_parameter_group_models: dict[str, dict[str, QtGui.QStandardItemModel]] = {}
        self.px4_parameter_group_status_labels: dict[str, dict[str, QtWidgets.QLabel]] = {}
        self.px4_parameter_reset_buttons: dict[str, dict[str, QtWidgets.QPushButton]] = {}
        tabs = QtWidgets.QTabWidget()
        for side, side_code, title, command_id in (
            ("left", "p1", "Pixhawk 1 / P1", "pixhawk.refresh_parameters.p1"),
            ("right", "p2", "Pixhawk 2 / P2", "pixhawk.refresh_parameters.p2"),
        ):
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            controls = QtWidgets.QHBoxLayout()
            refresh_button = _command_button("QGC 결과 새로고침", command_id, self._invoke_command)
            controls.addWidget(refresh_button)
            controls.addStretch(1)
            tab_layout.addLayout(controls)
            status_label = QtWidgets.QLabel("PARAM readback 대기 중")
            status_label.setObjectName("statusStrip")
            status_label.setWordWrap(True)
            status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            tab_layout.addWidget(status_label)
            self.px4_parameter_status_labels[side] = status_label
            self.px4_parameter_refresh_buttons[side] = refresh_button
            self.px4_parameter_group_models[side] = {}
            self.px4_parameter_models[side] = self.px4_parameter_group_models[side]
            self.px4_parameter_tables[side] = {}
            self.px4_parameter_group_status_labels[side] = {}
            self.px4_parameter_reset_buttons[side] = {}
            group_tabs = QtWidgets.QTabWidget()
            for calibration in PX4_CALIBRATION_GROUPS:
                group_tab = QtWidgets.QWidget()
                calibration_layout = QtWidgets.QVBoxLayout(group_tab)
                group_hint = QtWidgets.QLabel(calibration.description)
                group_hint.setObjectName("compactContract")
                group_hint.setWordWrap(True)
                calibration_layout.addWidget(group_hint)
                group_controls = QtWidgets.QHBoxLayout()
                if calibration.resettable:
                    reset_command_id = f"pixhawk.reset_calibration.{side_code}.{calibration.key}"
                    reset_button = _command_button(
                        "Reset to PX4 default",
                        reset_command_id,
                        self._invoke_command,
                    )
                else:
                    reset_button = QtWidgets.QPushButton("Manual parameter · no calibration reset")
                    reset_button.setEnabled(False)
                    reset_button.setToolTip(
                        "SENS_BOARD_ROT/Z_OFF are manual board-orientation parameters and are not changed by level calibration."
                    )
                group_controls.addWidget(reset_button)
                group_controls.addStretch(1)
                calibration_layout.addLayout(group_controls)
                group_status = QtWidgets.QLabel("PARAM readback 대기 중")
                group_status.setObjectName("statusStrip")
                group_status.setWordWrap(True)
                group_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                calibration_layout.addWidget(group_status)
                model = _table_model(self, ("Parameter", "Current", "Reset target", "Contract"))
                table = QtWidgets.QTableView()
                table.setModel(model)
                table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
                table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
                table.setAlternatingRowColors(True)
                table.verticalHeader().setVisible(False)
                table.setMinimumHeight(240)
                table.horizontalHeader().setStretchLastSection(True)
                table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
                table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
                table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
                calibration_layout.addWidget(table, 1)
                self.px4_parameter_group_models[side][calibration.key] = model
                self.px4_parameter_tables[side][calibration.key] = table
                self.px4_parameter_group_status_labels[side][calibration.key] = group_status
                self.px4_parameter_reset_buttons[side][calibration.key] = reset_button
                group_tabs.addTab(group_tab, calibration.label)
            tab_layout.addWidget(group_tabs, 1)
            tabs.addTab(tab, title)
        parameter_group_layout.addWidget(tabs)
        self.parameter_group = group

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
