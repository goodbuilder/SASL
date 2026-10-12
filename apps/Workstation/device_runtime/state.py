"""Named legacy diagnostics fields backed by the connection-job owner."""

from __future__ import annotations

from types import SimpleNamespace

from .connection_jobs import PixhawkConnectionJobs
from .maintenance_jobs import MAINTENANCE_FIELDS, PixhawkMaintenanceJobs
from .admin_jobs import ADMIN_FIELDS, PixhawkAdminJobs
from .discovery_jobs import DISCOVERY_FIELDS, NetworkDiscoveryJobs


def _job_handle(kind: str, field: str) -> property:
    return property(
        lambda state: state.connection_jobs.handle(kind, field),
        lambda state, value: state.connection_jobs.set_legacy_handle(kind, field, value),
    )


class DiagnosticsState(SimpleNamespace):
    pixhawk_connection_thread = _job_handle("connect", "thread")
    pixhawk_connection_worker = _job_handle("connect", "worker")
    pixhawk_disconnection_thread = _job_handle("disconnect", "thread")
    pixhawk_disconnection_worker = _job_handle("disconnect", "worker")

    def __init__(self, connection_jobs: PixhawkConnectionJobs) -> None:
        super().__init__(
            connection_jobs=connection_jobs, maintenance_jobs=PixhawkMaintenanceJobs(connection_jobs.parent()),
            admin_jobs=PixhawkAdminJobs(connection_jobs.parent()),
            discovery_jobs=NetworkDiscoveryJobs(connection_jobs.parent()),
        )


def _maintenance_field(name):
    return property(
        lambda state: getattr(state.maintenance_jobs, name),
        lambda state, value: setattr(state.maintenance_jobs, name, value),
    )


for _name in MAINTENANCE_FIELDS:
    setattr(DiagnosticsState, _name, _maintenance_field(_name))
del _name


def _discovery_field(name):
    return property(
        lambda state: getattr(state.discovery_jobs, name),
        lambda state, value: setattr(state.discovery_jobs, name, value),
    )


for _name in DISCOVERY_FIELDS:
    setattr(DiagnosticsState, _name, _discovery_field(_name))
del _name


def _admin_field(name):
    return property(
        lambda state: getattr(state.admin_jobs, name),
        lambda state, value: setattr(state.admin_jobs, name, value),
    )


for _name in ADMIN_FIELDS:
    setattr(DiagnosticsState, _name, _admin_field(_name))
del _name
