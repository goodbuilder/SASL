"""Interpret existing PARAM-worker results without widgets or a panel.

The core managers still produce all readback/selection/calibration status.
PARAM acceptance is not actual sensor selection. An apply outcome requests a
fresh independent vehicle_magnetometer/Shell observation through the UI adapter.
PX4 rotation enums and sensor-axis calibration coefficients retain producer units.
"""

from collections.abc import Mapping
from dataclasses import dataclass

from Workstation.workstation_core.px4_mag_manager import missing_mag_status_params


@dataclass(frozen=True)
class ParameterOutcome:
    camera: str
    kind: str
    mag_status: object = None
    attitude_status: object = None
    board_params: object = None
    clear_calibration_error: bool = False
    missing_readback: tuple | None = None
    applied: tuple = ()
    missing_roles: tuple = ()
    verify_actual_selection: bool = False
    expected_device_id: object = None


def interpret_parameter_result(result, *, active_requests=(), active_selections=()):
    """Project a core result, preserving the finite old-worker role fallback.

    ``active_requests`` entries are (role, operation); ``active_selections``
    entries are (role, actual_selected_device_id) with an active thread.
    Only the outer compatibility adapter assembles these scalar snapshots; the
    service cannot traverse worker handles, managers or presentation state.
    """
    data = dict(result or {})
    kind = str(data.get("kind") or "")
    status = data.get("status")
    camera = str(data.get("camera") or "")
    if not camera and hasattr(status, "actual_selected_device_id"):
        camera = next((side for side in ("left", "right", "drone")
                       for role, selected in active_selections
                       if role == side and selected == getattr(status, "actual_selected_device_id", None)), "")
    if not camera:
        camera = next((side for side, operation in active_requests if operation == kind), "")
    if not camera:
        return None
    if kind == "calibration_refresh":
        mag, attitude = data.get("mag_status"), data.get("attitude_status")
        if mag is None and attitude is None:
            return None
        return ParameterOutcome(camera, kind, mag_status=mag, attitude_status=attitude,
                                clear_calibration_error=True)
    if kind == "board_rotation_refresh":
        params = data.get("params")
        return ParameterOutcome(camera, kind, board_params=dict(params) if isinstance(params, Mapping) else {})
    if status is None:
        return None
    if kind in {"mag_refresh", "mag_apply"}:
        return ParameterOutcome(
            camera, kind, mag_status=status, clear_calibration_error=True,
            missing_readback=missing_mag_status_params(getattr(status, "params", None)) if kind == "mag_refresh" else None,
            applied=tuple(str(item) for item in data.get("applied") or ()),
            missing_roles=tuple(str(item) for item in data.get("missing") or ()),
            verify_actual_selection=kind == "mag_apply",
            expected_device_id=getattr(status, "expected_selected_device_id", None),
        )
    if kind == "attitude_refresh":
        return ParameterOutcome(camera, kind, attitude_status=status, clear_calibration_error=True)
    return None


def apply_mag_manager_readback(manager, status):
    """Keep core manager state aligned with the admitted worker's readback."""
    if manager is not None:
        manager.actual_selected_device_id = getattr(status, "actual_selected_device_id", None)
        manager.armed = bool(getattr(status, "armed", False))
