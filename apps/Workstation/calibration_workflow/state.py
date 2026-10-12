"""Calibration feature aliases its one shared checkerboard job slot."""

from Workstation.calibration_workflow.calibration_geometry_state import CalibrationGeometryState
from .jobs import CALIBRATION_JOB_FIELDS
from .capture import CalibrationCapture, CAPTURE_FIELDS
from .validation import CalibrationValidation, VALIDATION_FIELDS
from .ftp import CalibrationFtpTransactions, FTP_RESOURCE_FIELDS, FTP_PENDING_FIELDS

OPTICAL_CAPTURE_FIELDS = (
    "optical_pose_wiris_ftp_thread", "optical_pose_wiris_ftp_worker",
    "optical_pose_wiris_ftp_oz_thread", "optical_pose_wiris_ftp_oz_worker",
)


class CalibrationRuntimeState(CalibrationGeometryState):
    def __init__(self, geometry_store, jobs):
        super().__init__(geometry_store)
        self.capture_runtime = CalibrationCapture(parent=jobs.parent())
        self.validation = CalibrationValidation()
        self.calibration_jobs = jobs
        self.ftp_transactions = CalibrationFtpTransactions(jobs.parent())
        self._read_optical_capture_owner = lambda: None
        self._optical_capture_bootstrap = {name: None for name in OPTICAL_CAPTURE_FIELDS}

    def bind_optical_capture_owner(self, read_existing):
        """Resolve an already-created owner only; never instantiate a controller."""
        self._read_optical_capture_owner = read_existing


def _optical_capture_field(name):
    def read(state):
        owner = state._read_optical_capture_owner()
        if owner is None:
            return state._optical_capture_bootstrap.get(name)
        state._optical_capture_bootstrap.clear()
        return getattr(owner, name)

    def write(state, value):
        owner = state._read_optical_capture_owner()
        if owner is None:
            state._optical_capture_bootstrap[name] = value
        else:
            state._optical_capture_bootstrap.clear()
            setattr(owner, name, value)
    return property(read, write)


for _name in OPTICAL_CAPTURE_FIELDS:
    setattr(CalibrationRuntimeState, _name, _optical_capture_field(_name))
del _name


def _job_field(name):
    return property(
        lambda state: getattr(state.calibration_jobs, name),
        lambda state, value: setattr(state.calibration_jobs, name, value),
    )


for _name in CALIBRATION_JOB_FIELDS:
    setattr(CalibrationRuntimeState, _name, _job_field(_name))
del _name


def _ftp_field(name):
    return property(
        lambda state: getattr(state.ftp_transactions, name),
        lambda state, value: setattr(state.ftp_transactions, name, value),
    )


for _name in (*FTP_RESOURCE_FIELDS, *FTP_PENDING_FIELDS):
    setattr(CalibrationRuntimeState, _name, _ftp_field(_name))
del _name


def _validation_field(name):
    return property(
        lambda state: getattr(state.validation, name),
        lambda state, value: setattr(state.validation, name, value),
    )


for _name in VALIDATION_FIELDS:
    setattr(CalibrationRuntimeState, _name, _validation_field(_name))
del _name


def _capture_field(name):
    return property(
        lambda state: getattr(state.capture_runtime, name),
        lambda state, value: setattr(state.capture_runtime, name, value),
    )


for _name in CAPTURE_FIELDS:
    setattr(CalibrationRuntimeState, _name, _capture_field(_name))
del _name
