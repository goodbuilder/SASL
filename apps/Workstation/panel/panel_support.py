"""Shared constants and small panel helpers."""
from __future__ import annotations

from PyQt5 import (
    QtWidgets,
)
from Workstation.runtime_paths import (
    WORKSTATION_DATA_ROOT,
    WORKSTATION_SETTINGS_PATH,
)
from Workstation.workstation_core.px4_mag_manager import (
    MAG_PRIORITY_LEVELS,
    mag_priority_display,
)
from Workstation.workstation_core.settings import (
    AppSettings,
    load_settings as _load_shared_settings,
    save_settings as _save_shared_settings,
)
from Workstation.workstation_core.wiris_zoom import (
    VISIBLE_ZOOM_FX_ESTIMATES,
)
from Workstation.workstation_core.media_stream.policy import DECODER_AUTO, DECODER_SOFTWARE
from Workstation.workstation_core.media_stream.decoder_policy import normalize_decoder_mode
from pathlib import (
    Path,
)

APP_ROOT = WORKSTATION_DATA_ROOT

def load_settings(path: str | Path | None = None) -> AppSettings:
    return _load_shared_settings(WORKSTATION_SETTINGS_PATH if path is None else path)

def save_settings(settings: AppSettings, path: str | Path | None = None) -> None:
    _save_shared_settings(settings, WORKSTATION_SETTINGS_PATH if path is None else path)

def decoder_mode_for_side(
    requested_mode: object,
    overrides: object,
    camera: str | None,
) -> str:
    requested = normalize_decoder_mode(str(requested_mode or DECODER_AUTO))
    if camera in {"left", "right"} and isinstance(overrides, dict):
        if str(overrides.get(camera) or "").strip().lower() == DECODER_SOFTWARE:
            return DECODER_SOFTWARE
    return requested

def _ensure_side_decoder_overrides(panel: object) -> dict[str, str | None]:
    overrides = getattr(panel, "_runtime_decoder_overrides", None)
    if not isinstance(overrides, dict):
        overrides = {"left": None, "right": None}
        setattr(panel, "_runtime_decoder_overrides", overrides)
    overrides.setdefault("left", None)
    overrides.setdefault("right", None)
    return overrides

def _clear_side_active_decoder(panel: object, camera: str) -> None:
    setter = getattr(panel, "_set_active_stream_decoder", None)
    if callable(setter):
        setter(camera, None)
        return
    active = getattr(panel, "active_stream_decoders", None)
    if isinstance(active, dict) and camera in active:
        active[camera] = None

def angle_diff_deg(a_deg: float, b_deg: float) -> float:
    """Return wrapped angular difference a-b in [-180, 180) degrees."""
    return (float(a_deg) - float(b_deg) + 180.0) % 360.0 - 180.0

FORM_LABEL_MIN_WIDTH = 138

FORM_BUTTON_MIN_WIDTH = 112

NETWORK_FIELD_MIN_WIDTHS = {
    "interface": 120,
    "host": 140,
    "port": 88,
    "timeout": 104,
    "mask": 140,
}

APP_WINDOW_TITLE = "StereoWIRIS"

STARTUP_CAMERA_STATUS = "카메라: 대기 중 - 장비 연결 후 영상 수신 확인 또는 카메라 Preview 연결"

STARTUP_PIXHAWK_STATUS = "픽스호크: 대기 중 - 장비 연결 후 MAVLink 연결"

STARTUP_READY_MESSAGE = (
    "[연결] 카메라/픽스호크 미연결 상태에서도 앱을 사용할 수 있습니다. "
    "장비 연결 후 영상 수신 확인, 카메라 Preview 연결 또는 MAVLink 연결을 누르세요."
)

YOLO_CPU_DEVICE = "cpu"

YOLO_CUDA_DEVICE = "cuda:0"

WIRIS_VISIBLE_ZOOM_FX_ESTIMATES = VISIBLE_ZOOM_FX_ESTIMATES

MAX_MANUAL_FOCAL_PX = 100_000.0

DETECTION_PAIR_MAX_AGE_S = 1.0

RUNTIME_CAMERA_STATUS_LABEL_HEIGHT_PX = 32

RUNTIME_CAMERA_STATUS_GROUP_HEIGHT_PX = 48

RUNTIME_CAMERA_LAYOUT_BREAKPOINT_PX = 1060

RUNTIME_CAMERA_LAYOUT_HYSTERESIS_PX = 48

VISIBLE_STREAM_GOP_GET_COMMAND = "GEGS VISIBLE"

RECORDING_PREVIEW_LIMIT = 256

RECORDING_KEY_CACHE_LIMIT = 1024

CAMERA_AUTO_CONNECT_MAX_ATTEMPTS = 1

CAMERA_AUTO_CONNECT_RETRY_DELAY_MS = 1_500

RTSP_STREAM_CHECK_TIMEOUT_S = 30.0

DEFERRED_DELETE_COOPERATIVE_DRAIN_MS = 1_000

def _format_visible_zoom_fx_estimates() -> str:
    rows = [
        "WIRIS OZ 1280x720, HFOV 63.7deg 기준 명목 추정값입니다.",
        "정확한 측정은 해당 zoom index에서 체커보드 보정 후 적용하세요.",
        "",
        "index / ratio / estimated fx",
    ]
    rows.extend(
        f"{index:>2} / {ratio:>4.1f}x / {fx_px:>5,d} px"
        for index, ratio, fx_px in WIRIS_VISIBLE_ZOOM_FX_ESTIMATES
    )
    return "\n".join(rows)

WIRIS_VISIBLE_ZOOM_FX_TOOLTIP = _format_visible_zoom_fx_estimates()

def _set_px4_priority_combo_value(combo: QtWidgets.QComboBox, value: int | float | None) -> None:
    if value is None:
        return
    raw = int(value)
    index = combo.findData(raw)
    if index < 0:
        combo.addItem(f"Custom ({raw})", raw)
        index = combo.count() - 1
    combo.setCurrentIndex(index)
    combo.setToolTip(f"PX4 CAL_MAG*_PRIO: {mag_priority_display(raw)}")

def _build_px4_priority_combo(value: int) -> QtWidgets.QComboBox:
    combo = QtWidgets.QComboBox()
    for raw, label in MAG_PRIORITY_LEVELS:
        combo.addItem(label, raw)
    _set_px4_priority_combo_value(combo, value)
    combo.currentIndexChanged.connect(
        lambda _index, combo=combo: combo.setToolTip(
            f"PX4 CAL_MAG*_PRIO: {mag_priority_display(combo.currentData())}"
        )
    )
    return combo

def _px4_priority_combo_value(combo: QtWidgets.QComboBox) -> int:
    value = combo.currentData()
    return 0 if value is None else int(value)
