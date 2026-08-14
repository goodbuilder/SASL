"""Guards the `*_METHODS` registries consumed by the panel delegate bindings.

These lists are the source of truth for which controller methods get exposed
on `WirisStereoDistancePanel`. A split-definition or a stale/typo'd entry
(like the one `RECORDING_CONTROLLER_METHODS` used to have, spread across two
list literals in `recording_controller.py`) is otherwise silent until
something tries to call the missing method at runtime.
"""

from __future__ import annotations

import pytest

from Workstation.app_lifecycle_controller import APP_LIFECYCLE_METHODS, AppLifecycleController
from Workstation.calibration_controller import CALIBRATION_WORKFLOW_METHODS, CalibrationWorkflowController
from Workstation.camera_stream_controller import (
    CAMERA_STREAM_LIFECYCLE_METHODS,
    CameraStreamLifecycleController,
)
from Workstation.camera_time_sync_controller import CAMERA_TIME_SYNC_METHODS, CameraTimeSyncController
from Workstation.device_runtime_controller import (
    DEVICE_RUNTIME_CONTROLLER_METHODS,
    DeviceRuntimeController,
)
from Workstation.drone_telemetry_controller import (
    DRONE_TELEMETRY_CONTROLLER_METHODS,
    DroneTelemetryController,
)
from Workstation.gnss_runtime_controller import GNSS_RUNTIME_CONTROLLER_METHODS, GnssRuntimeController
from Workstation.measurement_view_controller import (
    MEASUREMENT_VIEW_CONTROLLER_METHODS,
    MeasurementViewController,
)
from Workstation.optical_pose_controller import OPTICAL_POSE_CONTROLLER_METHODS, OpticalPoseController
from Workstation.recording_controller import RECORDING_CONTROLLER_METHODS, RecordingController
from Workstation.run_evidence_controller import RUN_EVIDENCE_CONTROLLER_METHODS, RunEvidenceController
from Workstation.sensor_presentation_controller import (
    SENSOR_PRESENTATION_METHODS,
    SensorPresentationController,
)
from Workstation.target_drone_controller import TARGET_DRONE_CONTROLLER_METHODS, TargetDroneController
from Workstation.wiris_control_controller import WIRIS_CONTROL_METHODS, WirisControlController

# Every (controller class, its *_METHODS registry) pair that the panel's
# delegate-binding block wires up. Keep in sync with `standalone_gui.py`'s
# `_CONTROLLER_DELEGATE_BINDINGS`.
CONTROLLER_METHOD_REGISTRIES = (
    (CameraTimeSyncController, CAMERA_TIME_SYNC_METHODS),
    (RecordingController, RECORDING_CONTROLLER_METHODS),
    (RunEvidenceController, RUN_EVIDENCE_CONTROLLER_METHODS),
    (OpticalPoseController, OPTICAL_POSE_CONTROLLER_METHODS),
    (TargetDroneController, TARGET_DRONE_CONTROLLER_METHODS),
    (DroneTelemetryController, DRONE_TELEMETRY_CONTROLLER_METHODS),
    (CalibrationWorkflowController, CALIBRATION_WORKFLOW_METHODS),
    (DeviceRuntimeController, DEVICE_RUNTIME_CONTROLLER_METHODS),
    (GnssRuntimeController, GNSS_RUNTIME_CONTROLLER_METHODS),
    (MeasurementViewController, MEASUREMENT_VIEW_CONTROLLER_METHODS),
    (SensorPresentationController, SENSOR_PRESENTATION_METHODS),
    (CameraStreamLifecycleController, CAMERA_STREAM_LIFECYCLE_METHODS),
    (WirisControlController, WIRIS_CONTROL_METHODS),
    (AppLifecycleController, APP_LIFECYCLE_METHODS),
)

_IDS = [cls.__name__ for cls, _ in CONTROLLER_METHOD_REGISTRIES]


@pytest.mark.parametrize("controller_cls, method_names", CONTROLLER_METHOD_REGISTRIES, ids=_IDS)
def test_method_registry_has_no_duplicates(controller_cls, method_names):
    assert len(method_names) == len(set(method_names)), (
        f"{controller_cls.__name__} method registry contains duplicate names"
    )


@pytest.mark.parametrize("controller_cls, method_names", CONTROLLER_METHOD_REGISTRIES, ids=_IDS)
def test_method_registry_names_are_real_callable_attributes(controller_cls, method_names):
    missing = [name for name in method_names if not callable(getattr(controller_cls, name, None))]
    assert not missing, f"{controller_cls.__name__} registry references missing/non-callable methods: {missing}"


@pytest.mark.parametrize("controller_cls, method_names", CONTROLLER_METHOD_REGISTRIES, ids=_IDS)
def test_registry_methods_are_bound_on_the_panel_class(controller_cls, method_names):
    """Every registered name must actually reach WirisStereoDistancePanel.

    Guards the swap from 14 hand-duplicated delegate/setattr blocks to the
    single declarative `_CONTROLLER_DELEGATE_BINDINGS` table in
    `standalone_gui.py` — proves the refactor didn't silently drop a
    controller's method registration.
    """

    from Workstation import standalone_gui

    missing = [
        name
        for name in method_names
        if not callable(getattr(standalone_gui.WirisStereoDistancePanel, name, None))
    ]
    assert not missing, f"{controller_cls.__name__} methods not bound on WirisStereoDistancePanel: {missing}"
