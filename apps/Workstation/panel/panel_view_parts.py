"""Named layout results from the panel shell factory."""
from __future__ import annotations
from dataclasses import dataclass
from PyQt5 import QtWidgets


@dataclass(frozen=True)
class PanelShellParts:
    advanced_tabs: QtWidgets.QWidget
    alignment_attitude_layout: QtWidgets.QLayout
    alignment_mag_layout: QtWidgets.QLayout
    alignment_overview_layout: QtWidgets.QLayout
    alignment_tabs: QtWidgets.QWidget
    baseline_field_browser_layout: QtWidgets.QLayout
    baseline_field_browser_tab: QtWidgets.QWidget
    camera_form: QtWidgets.QLayout
    camera_group: QtWidgets.QWidget
    camera_layout: QtWidgets.QLayout
    camera_layout_outer: QtWidgets.QWidget
    camera_network_group: QtWidgets.QWidget
    checkerboard_calibration_layout: QtWidgets.QLayout
    checkerboard_validation_layout: QtWidgets.QLayout
    connection_tabs: QtWidgets.QWidget
    extrinsic_parameters_layout: QtWidgets.QLayout
    gnss_base_layout: QtWidgets.QLayout
    intrinsic_parameters_layout: QtWidgets.QLayout
    mag_management_layout: QtWidgets.QLayout
    optical_alignment_layout: QtWidgets.QLayout
    pixhawk_connection_layout: QtWidgets.QLayout
    pixhawk_form: QtWidgets.QLayout
    pixhawk_group: QtWidgets.QWidget
    px4_mag_advanced_layout: QtWidgets.QLayout
    px4_mag_advanced_tab: QtWidgets.QWidget
    rtk_layout: QtWidgets.QLayout
    runtime_layout: QtWidgets.QLayout
    save_layout: QtWidgets.QLayout
    setup_layout: QtWidgets.QLayout
    stereo_layout: QtWidgets.QLayout
    time_layout: QtWidgets.QLayout
    triangulation_quality_layout: QtWidgets.QLayout
    wiris_command_layout: QtWidgets.QLayout
    wiris_command_panel: QtWidgets.QWidget
    yolo_form: QtWidgets.QLayout
    yolo_group: QtWidgets.QWidget
    yolo_layout_outer: QtWidgets.QWidget
    zoom_intrinsics_layout: QtWidgets.QLayout
