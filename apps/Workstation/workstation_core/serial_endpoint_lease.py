"""Cooperating application lease for one canonical serial device.

The lock key is shared with the existing rig/Telemetry radio owner. /dev is
machine-local device identity; /run/user or /tmp holds disposable lock state.
The caller must still enforce the transport's OS exclusion where supported.
"""
import fcntl
import hashlib
import os
from pathlib import Path


def acquire_serial_endpoint(endpoint):
    resolved = os.path.realpath(str(endpoint))
    digest = hashlib.sha256(resolved.encode()).hexdigest()[:20]
    runtime = Path(f'/run/user/{os.getuid()}')
    if not runtime.is_dir():
        runtime = Path('/tmp')
    handle = (runtime / f'workstation-pixhawk-radio-{digest}.lock').open('a+')
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as exc:
        handle.close()
        raise BlockingIOError(f'Serial endpoint is busy: {endpoint}') from exc
    return handle
