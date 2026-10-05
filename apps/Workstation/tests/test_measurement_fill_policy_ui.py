from __future__ import annotations

from types import SimpleNamespace

from PyQt5 import QtWidgets

from Workstation.status_model import StatusModel
from Workstation.workspaces import MeasurementWorkspace


def _components() -> dict[str, dict[str, object]]:
    return {
        f"{side}.{dof}": {
            "provided": dof in {"roll_deg", "pitch_deg"},
            "editable": dof not in {"roll_deg", "pitch_deg"},
            "locked": dof in {"roll_deg", "pitch_deg"},
            "source": (
                "candidate:horizon-1"
                if dof in {"roll_deg", "pitch_deg"}
                else "live_pixhawk"
                if dof == "yaw_deg"
                else "live_gnss"
            ),
            "compatible_fill_source": (
                "live_pixhawk" if dof.endswith("_deg") else "live_gnss"
            ),
            "fill_enabled": dof not in {"roll_deg", "pitch_deg"},
            "source_ready": True,
            "reason": (
                "provided by method"
                if dof in {"roll_deg", "pitch_deg"}
                else "fill source selected"
            ),
        }
        for side in ("left", "right")
        for dof in ("roll_deg", "pitch_deg", "yaw_deg", "north_m", "east_m", "down_m")
    }


class _Backend:
    def __init__(self) -> None:
        self.panel = SimpleNamespace()
        self.calls: list[dict[str, object]] = []
        self.snapshot = {
            "selected_method_id": "horizon_hybrid",
            "selected_instance_id": "candidate:horizon-1",
            "selection_reason": "preferred",
            "ready_methods": [
                {
                    "method_id": "horizon_hybrid",
                    "label": "Horizon R/P + live Pixhawk/GNSS",
                    "instance_id": "candidate:horizon-1",
                    "ready": True,
                    "fallback_rank": 1,
                    "component_sources": {},
                }
            ],
            "method_capabilities": [
                {
                    "method_id": "horizon_hybrid",
                    "label": "Horizon R/P + live Pixhawk/GNSS",
                    "instance_id": "candidate:horizon-1",
                    "ready": True,
                    "components": _components(),
                }
            ],
        }

    def measurement_methods_snapshot(self):
        return self.snapshot

    def configure_measurement_method_fill(self, payload):
        values = dict(payload)
        self.calls.append(values)
        component = str(values["component"])
        state = self.snapshot["method_capabilities"][0]["components"][component]
        state["fill_enabled"] = bool(values["enabled"])
        state["source"] = state["compatible_fill_source"] if values["enabled"] else "none"
        return self.snapshot


def test_measurement_dof_fill_grid_locks_owned_values_and_toggles_missing_values() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _Backend()
    workspace = MeasurementWorkspace(backend, StatusModel())

    workspace._refresh_measurement_methods()

    assert workspace.measurement_fill_method_combo.objectName() == "measurementFillMethodCombo"
    left_roll = workspace.measurement_fill_buttons["left.roll_deg"]
    left_yaw = workspace.measurement_fill_buttons["left.yaw_deg"]
    left_north = workspace.measurement_fill_buttons["left.north_m"]
    assert left_roll.text() == "OWN" and not left_roll.isEnabled()
    assert left_yaw.text() == "PX4" and left_yaw.isEnabled() and left_yaw.isChecked()
    assert left_north.text() == "GNSS" and left_north.isEnabled() and left_north.isChecked()

    left_yaw.setChecked(False)

    assert backend.calls == [
        {
            "method_id": "horizon_hybrid",
            "component": "left.yaw_deg",
            "enabled": False,
        }
    ]
    assert left_yaw.isChecked() is False
    workspace.deleteLater()
