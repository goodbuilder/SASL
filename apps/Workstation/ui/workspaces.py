"""Compatibility imports for feature-owned workspace views.

New production code imports workspace_views owners directly. These aliases keep
existing callers on their original import path without owning view behavior.
"""

from Workstation.workspace_views.advanced import AdvancedWorkspace as AdvancedWorkspace

from Workstation.workspace_views.alignment import AlignmentWorkspace as AlignmentWorkspace

from Workstation.workspace_views.base_gnss import BaseGnssWorkspace as BaseGnssWorkspace

from Workstation.workspace_views.calibration import CalibrationErrorPlot as CalibrationErrorPlot, CalibrationWorkspace as CalibrationWorkspace, _ContentSizedOperationTabs as _ContentSizedOperationTabs, _ElidedStatusLabel as _ElidedStatusLabel

from Workstation.workspace_views.camera import CameraWorkspace as CameraWorkspace

from Workstation.workspace_views.camera_preview import CameraPreview as CameraPreview, CameraPreviewPair as CameraPreviewPair, _balance_preview_pair as _balance_preview_pair, _preview_pair_height as _preview_pair_height

from Workstation.workspace_views.diagnostics import DiagnosticsContent as DiagnosticsContent

from Workstation.workspace_views.experiment import ExperimentWorkspace as ExperimentWorkspace

from Workstation.workspace_views.measurement import MeasurementWorkspace as MeasurementWorkspace

from Workstation.workspace_views.network_helpers import _apply_network_role_discovery_status as _apply_network_role_discovery_status, _network_role_column_text as _network_role_column_text

from Workstation.workspace_views.optical_pose import OpticalPosePane as OpticalPosePane, _CurrentPageTabWidget as _CurrentPageTabWidget, _FrozenPreviewColumn as _FrozenPreviewColumn, _FrozenPreviewGroup as _FrozenPreviewGroup, _drone_rtk_gnss_label as _drone_rtk_gnss_label, _format_drone_rtk_degrees as _format_drone_rtk_degrees, _format_drone_rtk_metres as _format_drone_rtk_metres, _format_drone_rtk_seconds as _format_drone_rtk_seconds, _format_drone_rtk_sync_delta_ms as _format_drone_rtk_sync_delta_ms

from Workstation.workspace_views.overview import OverviewWorkspace as OverviewWorkspace

from Workstation.workspace_views.pixhawk import PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX as PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX, PixhawkWorkspace as PixhawkWorkspace, _PIXHAWK_SIDE_LABELS as _PIXHAWK_SIDE_LABELS, _PIXHAWK_SYSTEM_ID_LABELS as _PIXHAWK_SYSTEM_ID_LABELS, _display_compass_deg as _display_compass_deg, _display_magnetometer_corrected_yaw as _display_magnetometer_corrected_yaw, _display_magnetometer_px4_yaw as _display_magnetometer_px4_yaw, _display_signed_deg as _display_signed_deg, _display_signed_number as _display_signed_number, _format_px4_parameter_value as _format_px4_parameter_value, _magnetometer_yaw_summary as _magnetometer_yaw_summary, _panel_widget_text as _panel_widget_text, _panel_widget_value as _panel_widget_value

from Workstation.workspace_views.px4_widgets import _build_px4_priority_combo as _build_px4_priority_combo, _px4_priority_combo_value as _px4_priority_combo_value, _set_px4_priority_combo_value as _set_px4_priority_combo_value

from Workstation.workspace_views.recording import DataWorkspace as DataWorkspace, RecordingWorkspace as RecordingWorkspace, SESSION_CATALOG_REFRESH_INTERVAL_S as SESSION_CATALOG_REFRESH_INTERVAL_S, SessionsWorkspace as SessionsWorkspace

from Workstation.workspace_views.session import SessionWorkspace as SessionWorkspace

from Workstation.workspace_views.shared import _append_table_row as _append_table_row, _command_button as _command_button, _compact_text as _compact_text, _display_class as _display_class, _display_number as _display_number, _evidence as _evidence, _live_inspection as _live_inspection, _status as _status, _sync_table_rows as _sync_table_rows, _table_model as _table_model

from Workstation.workspace_views.telemetry import TelemetryWorkspace as TelemetryWorkspace

from Workstation.workspace_views.time_sync import TimeWorkspace as TimeWorkspace

__all__ = [
    "AdvancedWorkspace",
    "AlignmentWorkspace",
    "BaseGnssWorkspace",
    "CalibrationErrorPlot",
    "CalibrationWorkspace",
    "CameraPreview",
    "CameraPreviewPair",
    "CameraWorkspace",
    "DataWorkspace",
    "DiagnosticsContent",
    "ExperimentWorkspace",
    "MeasurementWorkspace",
    "OpticalPosePane",
    "OverviewWorkspace",
    "PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX",
    "PixhawkWorkspace",
    "RecordingWorkspace",
    "SESSION_CATALOG_REFRESH_INTERVAL_S",
    "SessionWorkspace",
    "SessionsWorkspace",
    "TelemetryWorkspace",
    "TimeWorkspace",
]
