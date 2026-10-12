"""One in-memory owner of selected/applied stereo calibration identities.

K uses native image pixels; D and optical-RDF rotations retain their model
conventions; stereo translations remain metres in their existing RDF contract.
No coordinate conversion, file I/O, Qt state or measurement policy lives here.
Legacy field reads are explicit live views; snapshots are detached copies.
"""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
import threading
from types import MappingProxyType


CALIBRATION_GEOMETRY_FIELDS = (
    "active_intrinsics",
    "selected_extrinsics_profile",
    "active_extrinsics_profile",
    "checkerboard_extrinsics_profile",
    "optical_pose_extrinsics_profile",
    "_selected_extrinsics_profile_path",
    "_selected_extrinsics_profile_hash",
    "_active_intrinsics_profile_path",
    "_active_intrinsics_profile_hash",
    "_active_intrinsics_source_zoom_level",
    "_active_extrinsics_profile_path",
    "_active_extrinsics_profile_hash",
    "_active_calibration_selection_mode",
    "_active_optical_pose_profile_hash",
    "_active_optical_pose_selection_id",
    "_active_optical_pose_effective_from_pair_sequence",
)
CAMERA_GEOMETRY_FIELDS = ("_active_extrinsics_source_zoom_level",)
GEOMETRY_FIELDS = frozenset((*CALIBRATION_GEOMETRY_FIELDS, *CAMERA_GEOMETRY_FIELDS))


@dataclass(frozen=True)
class ActiveGeometrySnapshot:
    revision: int
    values: Mapping[str, object]


class ActiveGeometryStore:
    def __init__(self) -> None:
        self._values: dict[str, object] = {}
        self._revision = 0
        self._lock = threading.RLock()

    def read_legacy(self, name: str) -> object:
        with self._lock:
            try:
                return self._values[name]
            except KeyError:
                raise AttributeError(name) from None

    def write_legacy(self, name: str, value: object) -> None:
        self.update({name: value})

    def remove_legacy(self, name: str) -> None:
        with self._lock:
            if name not in self._values:
                raise AttributeError(name)
            del self._values[name]
            self._revision += 1

    def update(self, values: Mapping[str, object]) -> int:
        unknown = set(values) - GEOMETRY_FIELDS
        if unknown:
            raise ValueError(f"Unknown active geometry fields: {sorted(unknown)}")
        with self._lock:
            self._values.update(values)
            self._revision += 1
            return self._revision

    def snapshot(self) -> ActiveGeometrySnapshot:
        with self._lock:
            return ActiveGeometrySnapshot(self._revision, MappingProxyType(deepcopy(self._values)))

    def restore_fields(self, attributes: Mapping[str, tuple[bool, object]]) -> None:
        """Restore an owner-captured field set as one coherent publication."""
        unknown = set(attributes) - GEOMETRY_FIELDS
        if unknown:
            raise ValueError(f"Unknown active geometry fields: {sorted(unknown)}")
        with self._lock:
            for name, (existed, value) in attributes.items():
                if existed:
                    self._values[name] = value
                else:
                    self._values.pop(name, None)
            self._revision += 1
