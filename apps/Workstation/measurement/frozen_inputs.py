"""Typed values crossing input capture -> geometry -> worker planning.

A pair contains no widgets, callbacks, mutable settings or live controller owners.
Body attitudes are FRD -> NED (wxyz); GPS positions remain geodetic; all optical
transforms and intrinsics retain their existing RDF contracts. Public points
produced by the resolver are baseline-center FRD metres.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Any

from Workstation.workstation_core.baseline_yaw import BaselineYawSelection
from Workstation.workstation_core.detection_pair_coordinator import PairTimingDecision
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    GpsSnapshot,
    StereoIntrinsicsSnapshot,
)
from Workstation.workstation_core.measurement_methods import MeasurementMethodStatus
from Workstation.workstation_core.method_evaluation import FrozenMeasurementPairSnapshot


@dataclass(frozen=True)
class FrozenMethodGeometry:
    intrinsics: StereoIntrinsicsSnapshot | None
    pose_evidence_json: str = ""
    nominal_intrinsics: bool = False
    yaw_selection: BaselineYawSelection | None = None
    yaw_p1_age_s: float | None = None
    yaw_p2_age_s: float | None = None


@dataclass(frozen=True)
class FrozenGeometryInputs:
    config: GeometryConfigSnapshot | None
    statuses: tuple[MeasurementMethodStatus, ...]
    methods: Mapping[tuple[str, str], FrozenMethodGeometry]
    errors: Mapping[tuple[str, str], str]
    generation_id: str
    selection_reason: str
    pair_advisories: tuple[str, ...] = ()
    capture_error: str = ""
    selected_method_id: str = ""
    selected_instance_id: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "statuses", tuple(self.statuses))
        object.__setattr__(self, "methods", MappingProxyType(dict(self.methods)))
        object.__setattr__(self, "errors", MappingProxyType(dict(self.errors)))
        object.__setattr__(self, "pair_advisories", tuple(self.pair_advisories))

    def for_method(self, status: MeasurementMethodStatus) -> FrozenMethodGeometry:
        key = (status.method_id, status.instance_id)
        if self.capture_error:
            raise ValueError(self.capture_error)
        if key in self.errors:
            raise ValueError(self.errors[key])
        if key not in self.methods:
            raise ValueError("method instance was not present when the pair was frozen")
        return self.methods[key]


@dataclass(frozen=True)
class FrozenPairInputs(Mapping[str, Any]):
    """Immutable runtime values plus the unchanged canonical wire snapshot.

    Mapping access is retained only for the established attempt API. No values
    are lazily read, and exporting a dict cannot change this capture's members.
    """

    physical_key: tuple[Any, ...]
    snapshot: FrozenMeasurementPairSnapshot
    requested_timestamp_basis: str
    timestamp_basis: str
    timing_fallback_reason: str
    pair_timing_decision: PairTimingDecision
    left_pairing_timestamp_s: float
    right_pairing_timestamp_s: float
    created_monotonic_s: float
    created_wall_s: float
    detection_midpoint_s: float
    left_detection: DetectionSnapshot
    right_detection: DetectionSnapshot
    left_attitude: AttitudeSnapshot | None
    right_attitude: AttitudeSnapshot | None
    left_attitude_age_s: float | None
    right_attitude_age_s: float | None
    left_gps: GpsSnapshot | None
    right_gps: GpsSnapshot | None
    measurement_start: float | None
    reused_inputs: tuple[str, ...]
    geometry_inputs: FrozenGeometryInputs | None = None

    def __getitem__(self, key: str) -> Any:
        if key not in self.__dataclass_fields__:
            raise KeyError(key)
        return getattr(self, key)

    def __iter__(self) -> Iterator[str]:
        return (member.name for member in fields(self))

    def __len__(self) -> int:
        return len(fields(self))
