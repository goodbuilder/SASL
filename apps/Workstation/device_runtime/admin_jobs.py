"""Single owner of Pixhawk admin jobs, transfer state and completion lifetime.

MAVFTP reserves one slot per camera. Flash, bootstrap, recovery, persistent
configuration and stream setup each retain their existing global slot. Transport
exclusion remains inside the unchanged workers/subscriber, not in a second lock.
GUI effects are delivered through QObject slots on this owner's Qt thread.
"""

from copy import deepcopy
from .admin_commands import ADMIN_COMMAND_TYPES
from .qt_jobs import QtOperationJobs


ADMIN_SINGLE_KINDS = ("usb_flash", "usb_bootstrap", "usb_recovery", "persistent_config", "stream_setup")
ADMIN_HANDLE_FIELDS = tuple(f"pixhawk_{kind}_{handle}" for kind in ADMIN_SINGLE_KINDS
                           for handle in ("thread", "worker")) + ("pixhawk_mavftp_threads", "pixhawk_mavftp_workers")
ADMIN_STATUS_FIELDS = ("pixhawk_transfer_status", "pixhawk_usb_flash_status", "pixhawk_usb_bootstrap_status",
                       "pixhawk_usb_recovery_status")
ADMIN_FIELDS = ADMIN_HANDLE_FIELDS + ADMIN_STATUS_FIELDS


class PixhawkAdminJobs(QtOperationJobs):
    command_types = ADMIN_COMMAND_TYPES
    handle_fields = ADMIN_HANDLE_FIELDS
    prefixes = {kind: f"pixhawk_{kind}" for kind in ADMIN_SINGLE_KINDS + ("mavftp",)}
    per_camera_kinds = frozenset({"mavftp"})
    cancel_kinds = frozenset({"mavftp", "usb_flash"})

    def __init__(self, parent=None, *, publish_legacy=None):
        super().__init__(parent, publish_legacy=publish_legacy)
        self.pixhawk_transfer_status = {}
        self.pixhawk_usb_flash_status = {"state": "IDLE"}
        self.pixhawk_usb_bootstrap_status = {}
        self.pixhawk_usb_recovery_status = {}

    def transfer_snapshot(self):
        return deepcopy(self.pixhawk_transfer_status)

    def transfer_started(self, command):
        self.pixhawk_transfer_status[command.camera] = {
            "state": "RESUMING" if command.resume else command.direction.upper() + "ING",
            "category": command.category, "local_path": command.local_path,
            "remote_path": command.remote_path, "sent": 0, "total": command.total,
            "direction": command.direction,
        }
        self._publish()

    def transfer_progress(self, value):
        data = deepcopy(dict(value or {}))
        camera = str(data.get("camera") or "")
        current = dict(self.pixhawk_transfer_status.get(camera) or {})
        current.update(data)
        direction = str(data.get("direction") or current.get("direction") or "").strip().lower()
        if direction in {"upload", "download"}:
            current["state"] = direction.upper() + "ING"
        self.pixhawk_transfer_status[camera] = current
        self._publish()

    def transfer_completed(self, value, *, category):
        data = deepcopy(dict(value or {}))
        data["state"] = "STAGED" if category == "firmware_stage" else "COMPLETED"
        data["category"] = str(category)
        self.pixhawk_transfer_status[str(data.get("camera") or "")] = data
        self._publish()
        return deepcopy(data)

    def transfer_failed(self, camera, error):
        current = dict(self.pixhawk_transfer_status.get(str(camera)) or {})
        current.update(state="PARTIAL/FAILED", error=str(error))
        self.pixhawk_transfer_status[str(camera)] = current
        self._publish()

    def flash_started(self, command):
        self.pixhawk_usb_flash_status = {
            "state": "STARTING", "camera": command.camera,
            "usb_device": command.usb_device, "firmware_path": command.firmware_path,
        }
        self._publish()

    def flash_progress(self, message):
        current = dict(self.pixhawk_usb_flash_status or {})
        current.update(state="FLASHING", detail=str(message))
        self.pixhawk_usb_flash_status = current
        self._publish()

    def flash_completed(self, value, *, evidence_path, persist):
        data = deepcopy(dict(value or {}))
        data["evidence_path"] = str(evidence_path)
        persist(evidence_path, data)
        self.pixhawk_usb_flash_status = data
        self._publish()
        return deepcopy(data)

    def flash_failed(self, camera, error):
        current = dict(self.pixhawk_usb_flash_status or {})
        current.update(state="FAILED/UNKNOWN", camera=str(camera), error=str(error))
        self.pixhawk_usb_flash_status = current
        self._publish()

    def resource_inventory(self):
        resources = super().resource_inventory()
        for job in self._jobs.values():
            if job.command.kind == "usb_flash":
                process = getattr(job.worker, "_process", None)
                if process is not None:
                    resources[f"pixhawk_admin_flash_{job.token}_process"] = process
        return resources
