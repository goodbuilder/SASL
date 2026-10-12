"""Existing controller API compatibility registry."""
from __future__ import annotations
from Workstation.controllers.app_lifecycle_controller import APP_LIFECYCLE_METHODS, AppLifecycleController
from Workstation.controllers.calibration_controller import CALIBRATION_WORKFLOW_METHODS, CalibrationWorkflowController
from Workstation.controllers.camera_stream_controller import CAMERA_STREAM_LIFECYCLE_METHODS, CameraStreamLifecycleController
from Workstation.controllers.camera_time_sync_controller import CAMERA_TIME_SYNC_METHODS, CameraTimeSyncController
from Workstation.application.controller_delegation import ControllerDelegateBinding
from Workstation.controllers.data_storage_controller import DATA_STORAGE_CONTROLLER_METHODS, DataStorageController
from Workstation.controllers.device_runtime_controller import DEVICE_RUNTIME_CONTROLLER_METHODS, DeviceRuntimeController
from Workstation.controllers.drone_telemetry_controller import DRONE_TELEMETRY_CONTROLLER_METHODS, DroneTelemetryController
from Workstation.controllers.gnss_runtime_controller import GNSS_RUNTIME_CONTROLLER_METHODS, GnssRuntimeController
from Workstation.controllers.measurement_run_controller import MEASUREMENT_RUN_METHODS, MeasurementRunController
from Workstation.controllers.measurement_view_controller import MEASUREMENT_VIEW_CONTROLLER_METHODS, MeasurementViewController
from Workstation.controllers.optical_pose_controller import OPTICAL_POSE_CONTROLLER_METHODS, OpticalPoseController
from Workstation.controllers.recording_controller import RECORDING_CONTROLLER_METHODS, RecordingController
from Workstation.controllers.run_evidence_controller import RUN_EVIDENCE_CONTROLLER_METHODS, RunEvidenceController
from Workstation.controllers.sensor_presentation_controller import SENSOR_PRESENTATION_METHODS, SensorPresentationController
from Workstation.controllers.settings_controller import SETTINGS_CONTROLLER_METHODS, WorkstationSettingsController
from Workstation.controllers.target_drone_controller import TARGET_DRONE_CONTROLLER_METHODS, TargetDroneController
from Workstation.controllers.wiris_control_controller import WIRIS_CONTROL_METHODS, WirisControlController

_CONTROLLER_DELEGATE_BINDINGS: tuple[ControllerDelegateBinding, ...] = (
    ControllerDelegateBinding(CameraTimeSyncController, "_camera_time_sync_controller", CAMERA_TIME_SYNC_METHODS),
    ControllerDelegateBinding(RecordingController, "_recording_controller", RECORDING_CONTROLLER_METHODS),
    ControllerDelegateBinding(RunEvidenceController, "_run_evidence_controller", RUN_EVIDENCE_CONTROLLER_METHODS),
    ControllerDelegateBinding(
        DataStorageController, "_data_storage_controller", DATA_STORAGE_CONTROLLER_METHODS
    ),
    ControllerDelegateBinding(OpticalPoseController, "_optical_pose_controller", OPTICAL_POSE_CONTROLLER_METHODS),
    ControllerDelegateBinding(TargetDroneController, "_target_drone_controller", TARGET_DRONE_CONTROLLER_METHODS),
    ControllerDelegateBinding(
        DroneTelemetryController, "_drone_telemetry_controller", DRONE_TELEMETRY_CONTROLLER_METHODS
    ),
    ControllerDelegateBinding(
        CalibrationWorkflowController, "_calibration_workflow_controller", CALIBRATION_WORKFLOW_METHODS
    ),
    ControllerDelegateBinding(
        DeviceRuntimeController, "_device_runtime_controller", DEVICE_RUNTIME_CONTROLLER_METHODS
    ),
    ControllerDelegateBinding(GnssRuntimeController, "_gnss_runtime_controller", GNSS_RUNTIME_CONTROLLER_METHODS),
    ControllerDelegateBinding(
        MeasurementViewController, "_measurement_view_controller", MEASUREMENT_VIEW_CONTROLLER_METHODS
    ),
    ControllerDelegateBinding(
        SensorPresentationController, "_sensor_presentation_controller", SENSOR_PRESENTATION_METHODS
    ),
    ControllerDelegateBinding(
        CameraStreamLifecycleController, "_camera_stream_lifecycle_controller", CAMERA_STREAM_LIFECYCLE_METHODS
    ),
    ControllerDelegateBinding(
        MeasurementRunController,
        "_measurement_run_controller",
        MEASUREMENT_RUN_METHODS,
    ),
    ControllerDelegateBinding(WirisControlController, "_wiris_control_controller", WIRIS_CONTROL_METHODS),
    ControllerDelegateBinding(AppLifecycleController, "_app_lifecycle_controller", APP_LIFECYCLE_METHODS),
    ControllerDelegateBinding(
        WorkstationSettingsController, "_workstation_settings_controller", SETTINGS_CONTROLLER_METHODS
    ),
)

