"""Settings feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
from Workstation.panel.panel_support import APP_ROOT, load_settings
from Workstation.runtime_paths import (
    WORKSTATION_CALIBRATION_ROOT,
    WORKSTATION_MEASUREMENT_DATA_ROOT,
    WORKSTATION_SETTINGS_PATH,
)
from Workstation.workstation_core.settings import (
    DEFAULT_CALIBRATION_ROOT,
    normalize_pixhawk_gnss_source,
)
from pathlib import (
    Path,
)


class SettingsFeature(PanelFeature):
    feature_name = 'settings'
    dependency_names = (
        '_measurement_latest_valid_gps_source',
        '_refresh_baseline_alignment_ui',
        '_refresh_baseline_source_summary',
        '_refresh_rtk_baseline',
        '_save_current_settings',
        'baseline_left_gps_combo',
        'baseline_right_gps_combo',
        'gnss_monitor',
        'gnss_rover_fixed_since',
    )
    writable_dependencies = frozenset(())

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self.app_root = Path(APP_ROOT if app_root is None else app_root).expanduser().resolve()
        self.default_calibration_root = (
            WORKSTATION_CALIBRATION_ROOT
            if app_root is None
            else self.app_root / DEFAULT_CALIBRATION_ROOT
        )
        self.default_measurement_data_root = (
            WORKSTATION_MEASUREMENT_DATA_ROOT
            if app_root is None
            else self.app_root / "recordings"
        )
        self.settings_path = Path(
            self.app_root / WORKSTATION_SETTINGS_PATH.name
            if settings_path is None
            else settings_path
        ).expanduser().resolve()
        self.settings = load_settings(self.settings_path)
        self._workstation_setting_overrides: dict[str, object] = {}

    def _workstation_setting(self, name: str, default: object = None) -> object:
        value = self._workstation_setting_overrides.get(name, default)
        return default if value is None else value

    def _pixhawk_gnss_source(self, side: str) -> str:
        if side not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {side}")
        key = "pixhawk_left_gnss_source" if side == "left" else "pixhawk_right_gnss_source"
        default = getattr(self.settings, key, "GPS1")
        return normalize_pixhawk_gnss_source(self._workstation_setting(key, default))

    def set_pixhawk_gnss_source(self, side: str, source: str, *, persist: bool = True) -> str:
        """Select the GPS1/GPS2 position source used by P1/P2 calculations."""

        if side not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {side}")
        selected = normalize_pixhawk_gnss_source(source)
        key = "pixhawk_left_gnss_source" if side == "left" else "pixhawk_right_gnss_source"
        self._workstation_setting_overrides[key] = selected
        if side == "left":
            self.gnss_monitor.left_source = selected
            combo = getattr(self, "baseline_left_gps_combo", None)
        else:
            self.gnss_monitor.right_source = selected
            combo = getattr(self, "baseline_right_gps_combo", None)
        if combo is not None:
            index = combo.findData(selected)
            if index >= 0 and combo.currentIndex() != index:
                blocked = combo.blockSignals(True)
                combo.setCurrentIndex(index)
                combo.blockSignals(blocked)
        # A source change must not allow a previous receiver's snapshot to be
        # reused for the new source during the current Measurement.
        cache_sources = getattr(self, "_measurement_latest_valid_gps_source", {})
        if isinstance(cache_sources, dict):
            cache_sources.pop(side, None)
        fixed_since = getattr(self, "gnss_rover_fixed_since", None)
        if isinstance(fixed_since, dict):
            fixed_since[side] = None
        if persist:
            self._save_current_settings()
        if hasattr(self, "_refresh_baseline_source_summary"):
            self._refresh_baseline_source_summary()
        if hasattr(self, "_refresh_rtk_baseline"):
            self._refresh_rtk_baseline()
        if hasattr(self, "_refresh_baseline_alignment_ui"):
            self._refresh_baseline_alignment_ui()
        return selected

