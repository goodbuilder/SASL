"""Legacy camera fields refer to the actual geometry and NIC transaction owners."""

from Workstation.camera_runtime.state import readiness_property
from Workstation.calibration_workflow.calibration_geometry_state import CameraGeometryState
from Workstation.device_runtime.network_roles import NETWORK_ROLE_FIELDS, NetworkRoleRuntime
from Workstation.camera_runtime.lifecycle_timers import LIFECYCLE_TIMER_FIELDS


from Workstation.camera_feature.candidates import CameraCandidateState
from Workstation.camera_runtime.finalization import VideoFinalization
from Workstation.camera_runtime.stop import CameraStopRuntime
from Workstation.camera_runtime.recovery import RECOVERY_FIELDS, StreamRecovery
from Workstation.camera_feature.control import CONTROL_JOB_FIELDS, CONTROL_STATE_FIELDS


class CameraFeatureState(CameraGeometryState):
    _camera_start_readiness = readiness_property("_camera_start_readiness")
    _camera_start_readiness_generation = readiness_property("_camera_start_readiness_generation")

    _camera_candidate_rows = property(
        lambda state: state.candidates.rows, lambda state, value: setattr(state.candidates, "rows", value)
    )
    workstation_camera_discovery_status = property(
        lambda state: state.candidates.status, lambda state, value: setattr(state.candidates, "status", value)
    )
    _pending_video_finalization_artifacts = property(
        lambda state: state.finalization.pending,
        lambda state, value: setattr(state.finalization, "pending", value),
    )

    def __init__(self, store, network_roles: NetworkRoleRuntime, stream_readiness, lifecycle_timers, control_jobs):
        super().__init__(store)
        self.network_roles = network_roles
        self.stream_readiness = stream_readiness
        self.lifecycle_timers = lifecycle_timers
        self.finalization = VideoFinalization()
        self.stream_stop = CameraStopRuntime(self.finalization, lifecycle_timers)
        self.recovery = StreamRecovery()
        self.control_jobs = control_jobs
        self.candidates = CameraCandidateState()


def _network_field(name):
    return property(
        lambda state: getattr(state.network_roles, name), lambda state, value: setattr(state.network_roles, name, value)
    )


for _name in NETWORK_ROLE_FIELDS:
    setattr(CameraFeatureState, _name, _network_field(_name))
del _name


def _timer_field(name):
    return property(
        lambda state: getattr(state.lifecycle_timers, name),
        lambda state, value: setattr(state.lifecycle_timers, name, value),
    )


for _name in LIFECYCLE_TIMER_FIELDS:
    setattr(CameraFeatureState, _name, _timer_field(_name))
del _name


def _recovery_field(name):
    return property(
        lambda state: getattr(state.recovery, name),
        lambda state, value: setattr(state.recovery, name, value),
    )


for _name in RECOVERY_FIELDS:
    setattr(CameraFeatureState, _name, _recovery_field(_name))
del _name


def _control_job_field(name):
    return property(
        lambda state: state.control_jobs.read_field(name),
        lambda state, value: state.control_jobs.write_field(name, value),
    )


for _name in CONTROL_JOB_FIELDS:
    setattr(CameraFeatureState, _name, _control_job_field(_name))
del _name


def _control_state_field(name):
    return property(
        lambda state: getattr(state.control_jobs, name),
        lambda state, value: setattr(state.control_jobs, name, value),
    )


for _name in CONTROL_STATE_FIELDS:
    setattr(CameraFeatureState, _name, _control_state_field(_name))
del _name
