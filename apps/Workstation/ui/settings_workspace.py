from __future__ import annotations

import json

from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

from PyQt5 import QtCore, QtWidgets

from Workstation.runtime_paths import (
    WORKSTATION_DATA_ROOT,
    WORKSTATION_MEASUREMENT_DATA_ROOT,
    import_workstation_asset_file,
    portable_workstation_data_path_text,
)
from Workstation.ui.wiris_ftp_browser import WirisFtpBrowserDialog
from Workstation.workstation_core import ntrip_credentials
from Workstation.workstation_core.gnss_operation import DEFAULT_NTRIP_SESSION_PASSWORD
from Workstation.workstation_core.settings import AppSettings
from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_LABELS,
    normalize_measurement_method_fallback_order,
)

from Workstation.ui.designer import load_form


def _mapping(value: object) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


class SettingsWorkspace(QtWidgets.QWidget):
    """Single Workstation surface for settings previously exposed only by Legacy."""

    def __init__(
        self,
        backend: object,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.invoke_command = invoke_command
        self.settings_widgets: dict[str, QtWidgets.QWidget] = {}
        self._gnss_network_password_explicit = False
        self._snapshot = self._read_snapshot()
        load_form("settings_workspace.ui", self)

        # Designer owns the form; Python owns settings values and option IDs.
        # Snapshot values (especially credentials and local paths) never live in .ui.
        for widget in self.findChildren(QtWidgets.QWidget):
            style_name = widget.property("styleObjectName")
            if style_name:
                widget.setObjectName(style_name)
            key = widget.property("settingsKey")
            if not key:
                continue
            if key in self.settings_widgets:
                raise ValueError(f"Duplicate Designer settings key: {key}")
            self.settings_widgets[key] = widget
            if isinstance(widget, QtWidgets.QComboBox):
                values = json.loads(widget.property("optionValues"))
                if len(values) != widget.count():
                    raise ValueError(f"Designer option label/value count mismatch: {key}")
                for index, value in enumerate(values):
                    widget.setItemData(index, value)
                widget.setCurrentIndex(0)
            self._set_widget_value(key, self._field_value(key))
        self._set_widget_value(
            "camera_sync_interval_s", self._value("camera_sync_interval_s", 1800.0)
        )

        # Paths are resolved at runtime and persisted by the existing data owner.
        self.settings_widgets["measurement_data_root"].setPlaceholderText(
            f"Default: {WORKSTATION_MEASUREMENT_DATA_ROOT}"
        )
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for button in self.findChildren(QtWidgets.QPushButton):
            command_id = button.property("commandId")
            if command_id:
                button.setEnabled(invoke_command is not None)
                if invoke_command is not None:
                    button.clicked.connect(
                        lambda _checked=False, command=command_id: invoke_command(command)
                    )
                self.operation_buttons[command_id] = button
            side = button.property("ftpSide")
            if side:
                button.clicked.connect(
                    lambda _checked=False, camera=side: self._open_wiris_ftp_browser(camera)
                )
        self.yolo_weights_browse_button.clicked.connect(self._browse_yolo_weights)
        self.measurementDataRootBrowse.clicked.connect(self._browse_measurement_data_root)
        self.measurement_fallback_list.model().rowsMoved.connect(
            lambda *_args: self._refresh_measurement_fallback_labels()
        )
        self._set_measurement_fallback_order(
            self._value("measurement_method_fallback_order", ())
        )
        # This session-only field deliberately stays outside settings_widgets.
        self.gnss_network_password_edit.setText(DEFAULT_NTRIP_SESSION_PASSWORD)
        self.gnss_network_password_edit.textChanged.connect(
            lambda _text: setattr(self, "_gnss_network_password_explicit", True)
        )
        self.gnss_network_forget_password_button.clicked.connect(
            lambda: self.invoke_command("gnss.ntrip_forget_password") if self.invoke_command else None
        )
        self.settings_widgets["gnss_operation_mode"].currentIndexChanged.connect(
            lambda _index: self._refresh_gnss_mode_controls()
        )
        self._refresh_gnss_mode_controls()
        self._refresh_settings_credential_status()


    def _read_snapshot(self) -> dict[str, Any]:
        reader = getattr(self.backend, "workstation_settings_snapshot", None)
        payload: object = None
        if callable(reader):
            try:
                payload = reader()
            except Exception:
                payload = None
        if not isinstance(payload, Mapping):
            defaults = AppSettings()
            payload = dict(defaults.__dict__)
            payload["gnss_operation"] = defaults.gnss_operation.to_mapping()
        else:
            payload = dict(payload)
        gnss = payload.get("gnss_operation")
        if hasattr(gnss, "to_mapping"):
            payload["gnss_operation"] = gnss.to_mapping()
        else:
            payload["gnss_operation"] = _mapping(gnss)
        return payload

    def _value(self, key: str, default: object = "") -> object:
        value = self._snapshot.get(key, default)
        return default if value is None else value

    def _gnss(self) -> dict[str, Any]:
        return _mapping(self._snapshot.get("gnss_operation"))

    def _gnss_value(self, section: str, key: str, default: object = "") -> object:
        return _mapping(self._gnss().get(section)).get(key, default)

    def _field_value(self, key: str) -> object:
        direct = self._snapshot.get(key)
        if direct is not None:
            return direct
        mapping = {
            "gnss_site_label": self._gnss().get("site_label"),
            "gnss_site_address": self._gnss().get("site_address"),
            "gnss_operation_mode": self._gnss().get("operation_mode"),
            "gnss_readiness_gate_enabled": self._gnss().get("readiness_gate_enabled"),
            "gnss_require_drone_rtk_fixed": self._gnss().get("require_drone_rtk_fixed"),
            "gnss_route_left_enabled": self._gnss_value("routing", "left_enabled"),
            "gnss_route_right_enabled": self._gnss_value("routing", "right_enabled"),
            "gnss_route_drone_enabled": self._gnss_value("routing", "drone_enabled"),
            "gnss_network_caster_host": self._gnss_value("network", "caster_host"),
            "gnss_network_caster_port": self._gnss_value("network", "caster_port"),
            "gnss_network_mountpoint": self._gnss_value("network", "mountpoint"),
            "gnss_network_username": self._gnss_value("network", "username"),
            "gnss_network_remember_password": self._gnss_value("network", "remember_password"),
            "gnss_network_username_env": self._gnss_value("network", "username_env"),
            "gnss_network_password_env": self._gnss_value("network", "password_env"),
            "gnss_network_tls": self._gnss_value("network", "tls"),
            "gnss_network_reconnect_interval_s": self._gnss_value("network", "reconnect_interval_s"),
            "gnss_network_connect_timeout_s": self._gnss_value("network", "connect_timeout_s"),
            "gnss_network_read_timeout_s": self._gnss_value("network", "read_timeout_s"),
            "gnss_network_gga_policy": self._gnss_value("network", "gga_policy"),
            "gnss_readiness_minimum_fixed_duration_s": self._gnss_value(
                "readiness", "minimum_fixed_duration_s"
            ),
            "gnss_readiness_minimum_fix_type": self._gnss_value(
                "readiness", "minimum_fix_type"
            ),
            "gnss_readiness_maximum_correction_age_s": self._gnss_value(
                "readiness", "maximum_correction_age_s"
            ),
            "gnss_readiness_maximum_gps_sample_age_s": self._gnss_value(
                "readiness", "maximum_gps_sample_age_s"
            ),
            "gnss_readiness_maximum_horizontal_accuracy_m": self._gnss_value(
                "readiness", "maximum_horizontal_accuracy_m"
            ),
            "gnss_readiness_maximum_vertical_accuracy_m": self._gnss_value(
                "readiness", "maximum_vertical_accuracy_m"
            ),
            "gnss_readiness_maximum_left_right_timestamp_delta_ms": self._gnss_value(
                "readiness", "maximum_left_right_timestamp_delta_ms"
            ),
            "gnss_readiness_maximum_baseline_std_m": self._gnss_value(
                "readiness", "maximum_baseline_std_m"
            ),
            "gnss_fixed_coordinate_profile_path": self._gnss_value("fixed_base", "coordinate_profile_path"),
            "gnss_fixed_acquisition_duration_s": self._gnss_value("fixed_base", "acquisition_duration_s"),
            "gnss_fixed_minimum_fixed_duration_s": self._gnss_value("fixed_base", "minimum_fixed_duration_s"),
            "gnss_fixed_minimum_fixed_ratio": self._gnss_value("fixed_base", "minimum_fixed_ratio"),
            "gnss_fixed_max_horizontal_scatter_m": self._gnss_value("fixed_base", "maximum_horizontal_scatter_m"),
            "gnss_fixed_max_vertical_scatter_m": self._gnss_value("fixed_base", "maximum_vertical_scatter_m"),
            "gnss_fixed_permit_profile_reuse": self._gnss_value("fixed_base", "permit_profile_reuse"),
            "gnss_fixed_height_type": self._gnss_value("fixed_base", "height_type"),
        }
        return mapping.get(key, "")


    def _open_wiris_ftp_browser(self, initial_side: str) -> None:
        hosts = {
            "Left": self.settings_widgets["left_control_host"].text().strip(),
            "Right": self.settings_widgets["right_control_host"].text().strip(),
        }
        hosts = {label: host for label, host in hosts.items() if host}
        credentials = {
            "Left": {
                "port": self.settings_widgets["left_ftp_port"].value(),
                "user": self.settings_widgets["left_ftp_user"].text(),
                "password": self.settings_widgets["left_ftp_password"].text(),
                "remote_dir": self.settings_widgets["left_ftp_remote_dir"].text() or "/",
                "timeout_s": self.settings_widgets["left_ftp_timeout_s"].value(),
            },
            "Right": {
                "port": self.settings_widgets["right_ftp_port"].value(),
                "user": self.settings_widgets["right_ftp_user"].text(),
                "password": self.settings_widgets["right_ftp_password"].text(),
                "remote_dir": self.settings_widgets["right_ftp_remote_dir"].text() or "/",
                "timeout_s": self.settings_widgets["right_ftp_timeout_s"].value(),
            },
        }
        initial = credentials.get(initial_side, {})
        dialog = WirisFtpBrowserDialog(
            self,
            hosts=hosts,
            credentials=credentials,
            initial_camera=initial_side,
            port=initial.get("port", 21),
            user=initial.get("user", "anonymous"),
            password=initial.get("password", ""),
            remote_dir=initial.get("remote_dir") or "/",
            timeout_s=initial.get("timeout_s", 10.0),
        )
        if dialog.exec_() == QtWidgets.QDialog.Accepted and dialog.selected_remote_dir:
            side = dialog.selected_camera_label or initial_side
            key = f"{side.lower()}_ftp_remote_dir"
            if key in self.settings_widgets:
                self.settings_widgets[key].setText(dialog.selected_remote_dir)


    def _workstation_data_root(self) -> Path:
        owner = getattr(self.backend, "panel", self.backend)
        configured = getattr(owner, "app_root", WORKSTATION_DATA_ROOT)
        return Path(configured).expanduser().resolve()

    def _browse_yolo_weights(self) -> None:
        data_root = self._workstation_data_root()
        weights_edit = self.settings_widgets["yolo_weights_path"]
        current_text = weights_edit.text().strip()
        current_path = Path(current_text).expanduser()
        if not current_path.is_absolute():
            current_path = data_root / current_path
        model_root = data_root / "models"
        start_dir = current_path.parent if current_path.parent.is_dir() else model_root
        if not start_dir.is_dir():
            start_dir = data_root
        selected, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select YOLO weights",
            str(start_dir),
            "Model weights (*.pt *.onnx *.engine *.torchscript);;All files (*)",
        )
        if not selected:
            return
        try:
            selected_path = Path(selected).expanduser().resolve(strict=True)
            try:
                selected_path.relative_to(data_root)
                external_selection = False
            except ValueError:
                external_selection = True
            imported = import_workstation_asset_file(
                selected_path,
                category="models",
                data_root=data_root,
            )
            portable = portable_workstation_data_path_text(
                imported,
                data_root=data_root,
                field_name="yolo_weights_path",
            )
        except (OSError, ValueError) as exc:
            self.yolo_weights_selection_status.setText(
                f"모델 가져오기 실패: {exc}"
            )
            return
        weights_edit.setText(portable)
        action = "가져오기 및 SHA-256 검증 완료" if external_selection else "모델 선택 완료"
        self.yolo_weights_selection_status.setText(
            f"{action} · 다음 Measurement 시작 시 적용: {portable}"
        )


    def _browse_measurement_data_root(self) -> None:
        data_root = self._workstation_data_root()
        edit = self.settings_widgets["measurement_data_root"]
        current_text = str(edit.text() or "").strip()
        current = (
            WORKSTATION_MEASUREMENT_DATA_ROOT
            if not current_text
            else Path(current_text).expanduser()
        )
        if current_text and not current.is_absolute():
            current = data_root / current
        start = current if current.is_dir() else current.parent
        if not start.is_dir():
            start = data_root
        selected = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Select measurement data folder",
            str(start),
        )
        if not selected:
            return
        selected_path = Path(selected).expanduser().resolve(strict=False)
        if selected_path == WORKSTATION_MEASUREMENT_DATA_ROOT.resolve(strict=False):
            edit.clear()
            self.storage_path_status_label.setText(
                f"Pending data folder: {WORKSTATION_MEASUREMENT_DATA_ROOT} (default) "
                "· applies to the next capture."
            )
            return
        try:
            portable = portable_workstation_data_path_text(
                selected_path,
                data_root=data_root,
                field_name="measurement_data_root",
            )
        except ValueError as exc:
            self.storage_path_status_label.setText(
                f"Storage folder was not changed: {exc}. Leave the field blank for "
                f"{WORKSTATION_MEASUREMENT_DATA_ROOT}, or set SASL_WORKSTATION_DATA_ROOT "
                "to place custom Workstation data on another volume."
            )
            return
        edit.setText(portable)
        self.storage_path_status_label.setText(
            f"Pending data folder: {portable} · applies to the next capture."
        )


    def _measurement_fallback_order(self) -> tuple[str, ...]:
        return tuple(
            str(self.measurement_fallback_list.item(index).data(QtCore.Qt.UserRole))
            for index in range(self.measurement_fallback_list.count())
        )

    def _refresh_measurement_fallback_labels(self) -> None:
        for index in range(self.measurement_fallback_list.count()):
            item = self.measurement_fallback_list.item(index)
            method_id = str(item.data(QtCore.Qt.UserRole) or "")
            item.setText(f"{index + 1:02d} · {MEASUREMENT_METHOD_LABELS.get(method_id, method_id)}")

    def _set_measurement_fallback_order(self, value: object) -> None:
        order = normalize_measurement_method_fallback_order(value)
        self.measurement_fallback_list.clear()
        for method_id in order:
            item = QtWidgets.QListWidgetItem()
            item.setData(QtCore.Qt.UserRole, method_id)
            self.measurement_fallback_list.addItem(item)
        self._refresh_measurement_fallback_labels()

    def _refresh_gnss_mode_controls(self) -> None:
        operation_mode = self.settings_widgets.get("gnss_operation_mode")
        if not isinstance(operation_mode, QtWidgets.QComboBox):
            return

        mode = str(operation_mode.currentData() or "LOCAL_SURVEY_IN")
        local_survey_in = mode == "LOCAL_SURVEY_IN"
        network_assisted_fixed_base = mode == "NETWORK_ASSISTED_FIXED_BASE"
        network_rtk_direct = mode == "NETWORK_RTK_DIRECT"

        for key in ("gnss_base_serial_port", "gnss_base_baud"):
            widget = self.settings_widgets.get(key)
            if widget is not None:
                widget.setEnabled(local_survey_in or network_assisted_fixed_base)

        for key in ("gnss_base_survey_in_min_duration_s", "gnss_base_survey_in_accuracy_m"):
            widget = self.settings_widgets.get(key)
            if widget is not None:
                widget.setEnabled(local_survey_in)

        for key in (
            "gnss_network_caster_host",
            "gnss_network_caster_port",
            "gnss_network_mountpoint",
            "gnss_network_username",
            "gnss_network_remember_password",
            "gnss_network_username_env",
            "gnss_network_password_env",
            "gnss_network_tls",
            "gnss_network_reconnect_interval_s",
            "gnss_network_connect_timeout_s",
            "gnss_network_read_timeout_s",
            "gnss_network_gga_policy",
        ):
            widget = self.settings_widgets.get(key)
            if widget is not None:
                widget.setEnabled(not local_survey_in)
        if hasattr(self, "gnss_network_password_edit"):
            self.gnss_network_password_edit.setEnabled(not local_survey_in)
        if hasattr(self, "gnss_network_forget_password_button"):
            self.gnss_network_forget_password_button.setEnabled(not local_survey_in)

        for key in (
            "gnss_fixed_coordinate_profile_path",
            "gnss_fixed_acquisition_duration_s",
            "gnss_fixed_minimum_fixed_duration_s",
            "gnss_fixed_minimum_fixed_ratio",
            "gnss_fixed_max_horizontal_scatter_m",
            "gnss_fixed_max_vertical_scatter_m",
            "gnss_fixed_permit_profile_reuse",
            "gnss_fixed_height_type",
        ):
            widget = self.settings_widgets.get(key)
            if widget is not None:
                widget.setEnabled(network_assisted_fixed_base)

        # Keep this explicit so a future mode addition cannot accidentally leave
        # the base-only controls active in Network RTK direct mode.
        if network_rtk_direct:
            for key in (
                "gnss_base_serial_port",
                "gnss_base_baud",
                "gnss_base_survey_in_min_duration_s",
                "gnss_base_survey_in_accuracy_m",
                "gnss_fixed_coordinate_profile_path",
                "gnss_fixed_acquisition_duration_s",
                "gnss_fixed_minimum_fixed_duration_s",
                "gnss_fixed_minimum_fixed_ratio",
                "gnss_fixed_max_horizontal_scatter_m",
                "gnss_fixed_max_vertical_scatter_m",
                "gnss_fixed_permit_profile_reuse",
                "gnss_fixed_height_type",
            ):
                widget = self.settings_widgets.get(key)
                if widget is not None:
                    widget.setEnabled(False)


    def _refresh_settings_credential_status(self) -> None:
        """Show whether a password is saved for the currently-displayed caster/username.

        A one-time read taken from the loaded snapshot -- this tab is an
        edit-then-Apply surface, not a live status monitor, so it does not
        track further changes until the tab is rebuilt.
        """

        host = str(self._gnss_value("network", "caster_host") or "")
        mountpoint = str(self._gnss_value("network", "mountpoint") or "")
        username = str(self._gnss_value("network", "username") or "")
        status = ntrip_credentials.credential_status(host, mountpoint, username)
        text = {
            ntrip_credentials.CredentialStorageStatus.SAVED: "Saved securely",
            ntrip_credentials.CredentialStorageStatus.NOT_SAVED: "Not saved",
            ntrip_credentials.CredentialStorageStatus.UNAVAILABLE: "Unavailable (session only)",
        }[status]
        self.gnss_network_credential_status_label.setText(text)

    def _widget_value(self, widget: QtWidgets.QWidget) -> object:
        if isinstance(widget, QtWidgets.QComboBox):
            return widget.currentData()
        if isinstance(widget, QtWidgets.QAbstractButton):
            return widget.isChecked()
        if isinstance(widget, QtWidgets.QLineEdit):
            return widget.text().strip()
        if isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
            return widget.value()
        return None

    def settings_payload(self) -> dict[str, object]:
        payload = {key: self._widget_value(widget) for key, widget in self.settings_widgets.items()}
        payload["camera_sync_interval_s"] = (
            float(payload.get("camera_sync_interval_s") or 30.0) * 60.0
        )
        payload["measurement_method_fallback_order"] = list(
            self._measurement_fallback_order()
        )
        gnss = _mapping(self._snapshot.get("gnss_operation"))
        readiness = _mapping(gnss.get("readiness"))
        local_base = _mapping(gnss.get("local_base"))
        network = _mapping(gnss.get("network"))
        fixed_base = _mapping(gnss.get("fixed_base"))
        routing = _mapping(gnss.get("routing"))
        gnss["operation_mode"] = payload.get("gnss_operation_mode")
        gnss["site_label"] = payload.get("gnss_site_label", "")
        gnss["site_address"] = payload.get("gnss_site_address", "")
        gnss["require_drone_rtk_fixed"] = bool(payload.get("gnss_require_drone_rtk_fixed"))
        gnss["readiness_gate_enabled"] = bool(payload.get("gnss_readiness_gate_enabled"))
        local_base.update(
            {
                "device": payload.get("gnss_base_serial_port", ""),
                "baud": payload.get("gnss_base_baud", 115200),
                "survey_in_min_duration_s": payload.get("gnss_base_survey_in_min_duration_s", 300),
                "survey_in_accuracy_limit_m": payload.get("gnss_base_survey_in_accuracy_m", 0.5),
            }
        )
        network.update(
            {
                "caster_host": payload.get("gnss_network_caster_host", ""),
                "caster_port": payload.get("gnss_network_caster_port", 2101),
                "mountpoint": payload.get("gnss_network_mountpoint", ""),
                "username": payload.get("gnss_network_username", ""),
                "remember_password": bool(payload.get("gnss_network_remember_password")),
                "username_env": payload.get("gnss_network_username_env", "NTRIP_USERNAME"),
                "password_env": payload.get("gnss_network_password_env", "NTRIP_PASSWORD"),
                "tls": bool(payload.get("gnss_network_tls")),
                "reconnect_interval_s": payload.get("gnss_network_reconnect_interval_s", 5.0),
                "connect_timeout_s": payload.get("gnss_network_connect_timeout_s", 10.0),
                "read_timeout_s": payload.get("gnss_network_read_timeout_s", 10.0),
                "gga_policy": payload.get("gnss_network_gga_policy", "SYSTEM_CENTER"),
            }
        )
        readiness.update(
            {
                "minimum_fix_type": payload.get("gnss_readiness_minimum_fix_type", 3),
                "minimum_fixed_duration_s": payload.get("gnss_readiness_minimum_fixed_duration_s", 30.0),
                "maximum_correction_age_s": payload.get("gnss_readiness_maximum_correction_age_s", 3.0),
                "maximum_gps_sample_age_s": payload.get("gnss_readiness_maximum_gps_sample_age_s", 1.0),
                "maximum_horizontal_accuracy_m": payload.get("gnss_readiness_maximum_horizontal_accuracy_m", 0.03),
                "maximum_vertical_accuracy_m": payload.get("gnss_readiness_maximum_vertical_accuracy_m", 0.05),
                "maximum_left_right_timestamp_delta_ms": payload.get("gnss_readiness_maximum_left_right_timestamp_delta_ms", 50.0),
                "maximum_baseline_std_m": payload.get("gnss_readiness_maximum_baseline_std_m", 0.02),
            }
        )
        fixed_base.update(
            {
                "coordinate_profile_path": payload.get("gnss_fixed_coordinate_profile_path", ""),
                "acquisition_duration_s": payload.get("gnss_fixed_acquisition_duration_s", 180.0),
                "minimum_fixed_duration_s": payload.get("gnss_fixed_minimum_fixed_duration_s", 60.0),
                "minimum_fixed_ratio": payload.get("gnss_fixed_minimum_fixed_ratio", 0.95),
                "maximum_horizontal_scatter_m": payload.get("gnss_fixed_max_horizontal_scatter_m", 0.03),
                "maximum_vertical_scatter_m": payload.get("gnss_fixed_max_vertical_scatter_m", 0.05),
                "permit_profile_reuse": bool(payload.get("gnss_fixed_permit_profile_reuse")),
                "height_type": payload.get("gnss_fixed_height_type", "UNKNOWN"),
            }
        )
        left_enabled = bool(payload.get("gnss_route_left_enabled"))
        right_enabled = bool(payload.get("gnss_route_right_enabled"))
        drone_enabled = bool(payload.get("gnss_route_drone_enabled"))
        if left_enabled and right_enabled:
            target = "both"
        elif left_enabled:
            target = "left"
        elif right_enabled:
            target = "right"
        else:
            target = "disabled"
        routing.update(
            {
                "left_enabled": left_enabled,
                "right_enabled": right_enabled,
                "drone_enabled": drone_enabled,
                # NTRIP-to-Base routing is inherent in the assisted fixed-base
                # operation; it is not an independent operator choice.
                "field_base_enabled": payload.get("gnss_operation_mode")
                == "NETWORK_ASSISTED_FIXED_BASE",
            }
        )
        gnss.update({"readiness": readiness, "local_base": local_base, "network": network, "fixed_base": fixed_base, "routing": routing})
        # Keep the legacy flat target available to the owner bridge and older
        # settings files while the visible UI uses explicit P1/P2 route flags.
        payload["gnss_base_rtcm_injection_target"] = target
        payload["gnss_operation"] = gnss
        return payload

    def _set_widget_value(self, key: str, value: object) -> None:
        widget = self.settings_widgets.get(key)
        if widget is None or value is None:
            return
        if isinstance(widget, QtWidgets.QComboBox):
            index = widget.findData(value)
            if index >= 0:
                widget.setCurrentIndex(index)
        elif isinstance(widget, QtWidgets.QAbstractButton):
            widget.setChecked(bool(value))
        elif isinstance(widget, QtWidgets.QLineEdit):
            widget.setText(str(value))
        elif isinstance(widget, QtWidgets.QDoubleSpinBox):
            try:
                widget.setValue(float(value))
            except (TypeError, ValueError):
                return
        elif isinstance(widget, QtWidgets.QSpinBox):
            try:
                widget.setValue(
                    round(float(value) / 60.0)
                    if key == "camera_sync_interval_s"
                    else int(value)
                )
            except (TypeError, ValueError):
                return

    def refresh_from_backend(self) -> None:
        self._snapshot = self._read_snapshot()
        for key in self.settings_widgets:
            self._set_widget_value(key, self._field_value(key))
        self._set_measurement_fallback_order(
            self._value("measurement_method_fallback_order", ())
        )
