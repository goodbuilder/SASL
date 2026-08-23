"""Canonical per-method Measurement evaluation contracts.

One matched detection pair owns one immutable snapshot.  Every canonical
Measurement method produces exactly one terminal outcome for that snapshot;
the primary Pixhawk/RTK outcome and the seven comparison outcomes never
overwrite one another.  This module is Qt-free so the same evaluator can run
in the main process, a reference test, or an isolated worker process.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from collections.abc import Mapping
from dataclasses import dataclass, field

from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
from Workstation.workstation_core.measurement_controller import (
    MeasurementGeometryResult,
    calculate_measurement_geometry,
)
from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    COMBINED_HYBRID,
    DRONE_RTK_MULTIVIEW,
    DRONE_RTK_ROTATION_HYBRID,
    HORIZON_HYBRID,
    MEASUREMENT_METHOD_IDS,
    MOUNTAIN_GNSS,
    PIXHAWK_RTK_LIVE,
    TERRAIN_PNP,
)

METHOD_EVALUATION_SCHEMA = "workstation-method-evaluation-v1"
METHOD_EVALUATION_REQUEST_SCHEMA = "workstation-method-evaluation-request-v1"
METHOD_EVALUATION_BUNDLE_SCHEMA = "workstation-method-evaluation-bundle-v1"
METHOD_EVALUATION_PRIMARY_METHOD = PIXHAWK_RTK_LIVE
METHOD_EVALUATION_QUEUE_LIMIT = 20

METHOD_EVALUATION_STATUSES = frozenset(
    {
        "VALID",
        "ESTIMATED",
        "BLOCKED",
        "FAILED",
        "TIMEOUT",
        "BACKPRESSURE_BLOCKED",
    }
)

METHOD_EVALUATION_TIMEOUT_S: Mapping[str, float] = {
    CHECKERBOARD_STEREO: 3.0,
    HORIZON_HYBRID: 1.0,
    MOUNTAIN_GNSS: 3.0,
    DRONE_RTK_ROTATION_HYBRID: 3.0,
    DRONE_RTK_MULTIVIEW: 5.0,
    TERRAIN_PNP: 10.0,
    COMBINED_HYBRID: 3.0,
}

METHOD_RESULT_RELATIVE_PATHS: Mapping[str, str] = {
    method_id: f"data/raw/method_results/{method_id}.jsonl"
    for method_id in MEASUREMENT_METHOD_IDS
}


def _canonical_bytes(payload: Mapping[str, object]) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    ).encode("utf-8")


def _sha256(payload: Mapping[str, object]) -> str:
    return hashlib.sha256(_canonical_bytes(payload)).hexdigest()


def _non_negative_ns(value: object, *, name: str) -> int:
    result = int(value or 0)
    if result < 0:
        raise ValueError(f"{name} must be non-negative")
    return result


def _reject_binary_payload(value: object, *, name: str = "snapshot") -> None:
    if isinstance(value, (bytes, bytearray, memoryview)):
        raise TypeError(f"{name} must reference native files instead of carrying bytes")
    if isinstance(value, Mapping):
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError(f"{name} keys must be strings")
            _reject_binary_payload(item, name=f"{name}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _reject_binary_payload(item, name=f"{name}[{index}]")
    elif value is None or isinstance(value, (str, int, bool)):
        return
    elif isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    else:
        raise TypeError(
            f"{name} must contain JSON scalars or native file references, "
            f"not {type(value).__name__}"
        )


@dataclass(frozen=True)
class FrozenMeasurementPairSnapshot:
    """One pair's immutable, byte-free input boundary shared by all methods."""

    measurement_id: str
    pair_id: str
    pair_sequence: int
    created_monotonic_ns: int
    created_unix_ns: int
    timestamp_basis: str
    left_detection: Mapping[str, object]
    right_detection: Mapping[str, object]
    left_attitude: Mapping[str, object] | None = None
    right_attitude: Mapping[str, object] | None = None
    left_gps: Mapping[str, object] | None = None
    right_gps: Mapping[str, object] | None = None
    native_frames: Mapping[str, object] = field(default_factory=dict)
    capture_sync: Mapping[str, object] = field(default_factory=dict)
    reused_inputs: tuple[str, ...] = ()
    schema: str = "workstation-frozen-measurement-pair-v1"

    def __post_init__(self) -> None:
        if self.schema != "workstation-frozen-measurement-pair-v1":
            raise ValueError("unsupported frozen measurement pair schema")
        if not self.measurement_id or not self.pair_id:
            raise ValueError("frozen pair measurement_id and pair_id are required")
        if int(self.pair_sequence) < 0:
            raise ValueError("frozen pair sequence must be non-negative")
        _non_negative_ns(self.created_monotonic_ns, name="created_monotonic_ns")
        _non_negative_ns(self.created_unix_ns, name="created_unix_ns")
        for payload in (
            self.left_detection,
            self.right_detection,
            self.left_attitude,
            self.right_attitude,
            self.left_gps,
            self.right_gps,
            self.native_frames,
            self.capture_sync,
        ):
            _reject_binary_payload(payload)

    def canonical_payload(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "measurement_id": self.measurement_id,
            "pair_id": self.pair_id,
            "pair_sequence": int(self.pair_sequence),
            "created_monotonic_ns": int(self.created_monotonic_ns),
            "created_unix_ns": int(self.created_unix_ns),
            "timestamp_basis": self.timestamp_basis,
            "left_detection": dict(self.left_detection),
            "right_detection": dict(self.right_detection),
            "left_attitude": None if self.left_attitude is None else dict(self.left_attitude),
            "right_attitude": None if self.right_attitude is None else dict(self.right_attitude),
            "left_gps": None if self.left_gps is None else dict(self.left_gps),
            "right_gps": None if self.right_gps is None else dict(self.right_gps),
            "native_frames": dict(self.native_frames),
            "capture_sync": dict(self.capture_sync),
            "reused_inputs": list(self.reused_inputs),
        }

    def canonical_sha256(self) -> str:
        return _sha256(self.canonical_payload())

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> FrozenMeasurementPairSnapshot:
        def optional_mapping(name: str) -> dict[str, object] | None:
            value = payload.get(name)
            return dict(value) if isinstance(value, Mapping) else None

        return cls(
            schema=str(payload.get("schema") or "workstation-frozen-measurement-pair-v1"),
            measurement_id=str(payload.get("measurement_id") or ""),
            pair_id=str(payload.get("pair_id") or ""),
            pair_sequence=int(payload.get("pair_sequence") or 0),
            created_monotonic_ns=int(payload.get("created_monotonic_ns") or 0),
            created_unix_ns=int(payload.get("created_unix_ns") or 0),
            timestamp_basis=str(payload.get("timestamp_basis") or ""),
            left_detection=dict(payload.get("left_detection") or {}),
            right_detection=dict(payload.get("right_detection") or {}),
            left_attitude=optional_mapping("left_attitude"),
            right_attitude=optional_mapping("right_attitude"),
            left_gps=optional_mapping("left_gps"),
            right_gps=optional_mapping("right_gps"),
            native_frames=dict(payload.get("native_frames") or {}),
            capture_sync=dict(payload.get("capture_sync") or {}),
            reused_inputs=tuple(str(value) for value in payload.get("reused_inputs", ()) or ()),
        )


@dataclass(frozen=True)
class MethodEvaluationRequest:
    measurement_id: str
    pair_id: str
    pair_sequence: int
    snapshot_sha256: str
    request_generation: int
    method_id: str
    method_instance_id: str
    snapshot_payload: Mapping[str, object] = field(default_factory=dict)
    attempt_payload: Mapping[str, object] | None = None
    blocked_reasons: tuple[str, ...] = ()
    reused_inputs: tuple[str, ...] = ()
    enqueued_monotonic_ns: int = field(default_factory=time.monotonic_ns)
    schema: str = METHOD_EVALUATION_REQUEST_SCHEMA

    def __post_init__(self) -> None:
        if self.schema != METHOD_EVALUATION_REQUEST_SCHEMA:
            raise ValueError("unsupported method evaluation request schema")
        if not str(self.measurement_id):
            raise ValueError("measurement_id is required")
        if not str(self.pair_id):
            raise ValueError("pair_id is required")
        if int(self.pair_sequence) < 0:
            raise ValueError("pair_sequence must be non-negative")
        if self.method_id not in MEASUREMENT_METHOD_IDS:
            raise ValueError(f"unsupported measurement method: {self.method_id}")
        if len(str(self.snapshot_sha256)) != 64:
            raise ValueError("snapshot_sha256 must be SHA-256 hex")
        if any(character not in "0123456789abcdef" for character in self.snapshot_sha256):
            raise ValueError("snapshot_sha256 must be lowercase SHA-256 hex")
        if self.snapshot_payload:
            snapshot = FrozenMeasurementPairSnapshot.from_mapping(self.snapshot_payload)
            if snapshot.canonical_sha256() != self.snapshot_sha256:
                raise ValueError("snapshot payload does not match snapshot_sha256")
        if self.attempt_payload is None and not self.blocked_reasons:
            raise ValueError("a request without an attempt requires blocked_reasons")
        _non_negative_ns(self.enqueued_monotonic_ns, name="enqueued_monotonic_ns")

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "measurement_id": self.measurement_id,
            "pair_id": self.pair_id,
            "pair_sequence": int(self.pair_sequence),
            "snapshot_sha256": self.snapshot_sha256,
            "request_generation": int(self.request_generation),
            "method_id": self.method_id,
            "method_instance_id": self.method_instance_id,
            "snapshot_payload": dict(self.snapshot_payload),
            "attempt_payload": (
                None if self.attempt_payload is None else dict(self.attempt_payload)
            ),
            "blocked_reasons": list(self.blocked_reasons),
            "reused_inputs": list(self.reused_inputs),
            "enqueued_monotonic_ns": int(self.enqueued_monotonic_ns),
        }

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> MethodEvaluationRequest:
        attempt = payload.get("attempt_payload")
        snapshot = payload.get("snapshot_payload")
        return cls(
            schema=str(payload.get("schema") or METHOD_EVALUATION_REQUEST_SCHEMA),
            measurement_id=str(payload.get("measurement_id") or ""),
            pair_id=str(payload.get("pair_id") or ""),
            pair_sequence=int(payload.get("pair_sequence") or 0),
            snapshot_sha256=str(payload.get("snapshot_sha256") or ""),
            request_generation=int(payload.get("request_generation") or 0),
            method_id=str(payload.get("method_id") or ""),
            method_instance_id=str(payload.get("method_instance_id") or ""),
            snapshot_payload=dict(snapshot) if isinstance(snapshot, Mapping) else {},
            attempt_payload=dict(attempt) if isinstance(attempt, Mapping) else None,
            blocked_reasons=tuple(str(value) for value in payload.get("blocked_reasons", ()) or ()),
            reused_inputs=tuple(str(value) for value in payload.get("reused_inputs", ()) or ()),
            enqueued_monotonic_ns=int(payload.get("enqueued_monotonic_ns") or 0),
        )


@dataclass(frozen=True)
class MethodEvaluationOutcome:
    measurement_id: str
    pair_id: str
    pair_sequence: int
    snapshot_sha256: str
    request_generation: int
    method_id: str
    method_instance_id: str
    status: str
    attempt_sha256: str = ""
    attempt_payload: Mapping[str, object] = field(default_factory=dict)
    geometry_sha256: str = ""
    result: Mapping[str, object] = field(default_factory=dict)
    reasons: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()
    enqueued_monotonic_ns: int = 0
    worker_started_monotonic_ns: int = 0
    completed_monotonic_ns: int = 0
    reused_inputs: tuple[str, ...] = ()
    schema: str = METHOD_EVALUATION_SCHEMA

    def __post_init__(self) -> None:
        if self.schema != METHOD_EVALUATION_SCHEMA:
            raise ValueError("unsupported method evaluation schema")
        if self.method_id not in MEASUREMENT_METHOD_IDS:
            raise ValueError(f"unsupported measurement method: {self.method_id}")
        if self.status not in METHOD_EVALUATION_STATUSES:
            raise ValueError(f"unsupported method evaluation status: {self.status}")
        if not str(self.measurement_id) or not str(self.pair_id):
            raise ValueError("measurement_id and pair_id are required")
        if len(str(self.snapshot_sha256)) != 64 or any(
            character not in "0123456789abcdef"
            for character in str(self.snapshot_sha256)
        ):
            raise ValueError("snapshot_sha256 must be lowercase SHA-256 hex")
        if int(self.pair_sequence) < 0:
            raise ValueError("pair_sequence must be non-negative")
        for name in (
            "enqueued_monotonic_ns",
            "worker_started_monotonic_ns",
            "completed_monotonic_ns",
        ):
            _non_negative_ns(getattr(self, name), name=name)
        if (
            self.worker_started_monotonic_ns
            and self.enqueued_monotonic_ns
            and self.worker_started_monotonic_ns < self.enqueued_monotonic_ns
        ):
            raise ValueError("worker start precedes enqueue")
        if (
            self.completed_monotonic_ns
            and self.worker_started_monotonic_ns
            and self.completed_monotonic_ns < self.worker_started_monotonic_ns
        ):
            raise ValueError("completion precedes worker start")
        if self.status in {"VALID", "ESTIMATED"}:
            if not self.attempt_sha256 or not self.attempt_payload or not self.result:
                raise ValueError("successful outcomes require attempt hash and result")
        elif not self.reasons:
            raise ValueError("non-success outcomes require a reason")

    @property
    def queue_wait_ms(self) -> float | None:
        if not self.enqueued_monotonic_ns or not self.worker_started_monotonic_ns:
            return None
        return (self.worker_started_monotonic_ns - self.enqueued_monotonic_ns) / 1_000_000.0

    @property
    def evaluation_duration_ms(self) -> float | None:
        if not self.worker_started_monotonic_ns or not self.completed_monotonic_ns:
            return None
        return (self.completed_monotonic_ns - self.worker_started_monotonic_ns) / 1_000_000.0

    def canonical_payload(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "measurement_id": self.measurement_id,
            "pair_id": self.pair_id,
            "pair_sequence": int(self.pair_sequence),
            "snapshot_sha256": self.snapshot_sha256,
            "request_generation": int(self.request_generation),
            "method_id": self.method_id,
            "method_instance_id": self.method_instance_id,
            "status": self.status,
            "attempt_sha256": self.attempt_sha256,
            "attempt_payload": dict(self.attempt_payload),
            "geometry_sha256": self.geometry_sha256,
            "result": dict(self.result),
            "reasons": list(self.reasons),
            "warnings": list(self.warnings),
            "enqueued_monotonic_ns": int(self.enqueued_monotonic_ns),
            "worker_started_monotonic_ns": int(self.worker_started_monotonic_ns),
            "completed_monotonic_ns": int(self.completed_monotonic_ns),
            "queue_wait_ms": self.queue_wait_ms,
            "evaluation_duration_ms": self.evaluation_duration_ms,
            "reused_inputs": list(self.reused_inputs),
        }

    def canonical_sha256(self) -> str:
        return _sha256(self.canonical_payload())

    def to_record(self) -> dict[str, object]:
        payload = self.canonical_payload()
        payload["method_evaluation_sha256"] = self.canonical_sha256()
        return payload

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> MethodEvaluationOutcome:
        result = payload.get("result")
        attempt_payload = payload.get("attempt_payload")
        outcome = cls(
            schema=str(payload.get("schema") or METHOD_EVALUATION_SCHEMA),
            measurement_id=str(payload.get("measurement_id") or ""),
            pair_id=str(payload.get("pair_id") or ""),
            pair_sequence=int(payload.get("pair_sequence") or 0),
            snapshot_sha256=str(payload.get("snapshot_sha256") or ""),
            request_generation=int(payload.get("request_generation") or 0),
            method_id=str(payload.get("method_id") or ""),
            method_instance_id=str(payload.get("method_instance_id") or ""),
            status=str(payload.get("status") or ""),
            attempt_sha256=str(payload.get("attempt_sha256") or ""),
            attempt_payload=(
                dict(attempt_payload) if isinstance(attempt_payload, Mapping) else {}
            ),
            geometry_sha256=str(payload.get("geometry_sha256") or ""),
            result=dict(result) if isinstance(result, Mapping) else {},
            reasons=tuple(str(value) for value in payload.get("reasons", ()) or ()),
            warnings=tuple(str(value) for value in payload.get("warnings", ()) or ()),
            enqueued_monotonic_ns=int(payload.get("enqueued_monotonic_ns") or 0),
            worker_started_monotonic_ns=int(payload.get("worker_started_monotonic_ns") or 0),
            completed_monotonic_ns=int(payload.get("completed_monotonic_ns") or 0),
            reused_inputs=tuple(str(value) for value in payload.get("reused_inputs", ()) or ()),
        )
        supplied_sha256 = str(payload.get("method_evaluation_sha256") or "")
        if supplied_sha256 and supplied_sha256 != outcome.canonical_sha256():
            raise ValueError("method evaluation hash mismatch")
        return outcome


@dataclass(frozen=True)
class MethodEvaluationBundle:
    measurement_id: str
    pair_id: str
    pair_sequence: int
    snapshot_sha256: str
    request_generation: int
    outcomes: tuple[MethodEvaluationOutcome, ...]
    snapshot_payload: Mapping[str, object] = field(default_factory=dict)
    schema: str = METHOD_EVALUATION_BUNDLE_SCHEMA

    def __post_init__(self) -> None:
        if self.schema != METHOD_EVALUATION_BUNDLE_SCHEMA:
            raise ValueError("unsupported method evaluation bundle schema")
        if len(str(self.snapshot_sha256)) != 64 or any(
            character not in "0123456789abcdef"
            for character in str(self.snapshot_sha256)
        ):
            raise ValueError("snapshot_sha256 must be lowercase SHA-256 hex")
        if self.snapshot_payload:
            snapshot = FrozenMeasurementPairSnapshot.from_mapping(self.snapshot_payload)
            if snapshot.canonical_sha256() != self.snapshot_sha256:
                raise ValueError("bundle snapshot payload does not match snapshot_sha256")
        method_ids = tuple(outcome.method_id for outcome in self.outcomes)
        if method_ids != MEASUREMENT_METHOD_IDS:
            raise ValueError("bundle outcomes must contain every method in canonical order")
        for outcome in self.outcomes:
            if (
                outcome.measurement_id != self.measurement_id
                or outcome.pair_id != self.pair_id
                or outcome.pair_sequence != self.pair_sequence
                or outcome.snapshot_sha256 != self.snapshot_sha256
                or outcome.request_generation != self.request_generation
            ):
                raise ValueError("bundle outcome identity mismatch")

    @property
    def primary(self) -> MethodEvaluationOutcome:
        return self.outcomes[MEASUREMENT_METHOD_IDS.index(METHOD_EVALUATION_PRIMARY_METHOD)]

    def canonical_payload(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "measurement_id": self.measurement_id,
            "pair_id": self.pair_id,
            "pair_sequence": int(self.pair_sequence),
            "snapshot_sha256": self.snapshot_sha256,
            "request_generation": int(self.request_generation),
            "primary_method_id": METHOD_EVALUATION_PRIMARY_METHOD,
            "primary_outcome_sha256": self.primary.canonical_sha256(),
            "snapshot_payload": dict(self.snapshot_payload),
            "outcomes": [outcome.to_record() for outcome in self.outcomes],
        }

    def canonical_sha256(self) -> str:
        return _sha256(self.canonical_payload())

    def to_record(self) -> dict[str, object]:
        payload = self.canonical_payload()
        payload["method_evaluation_bundle_sha256"] = self.canonical_sha256()
        return payload

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> MethodEvaluationBundle:
        raw_outcomes = payload.get("outcomes")
        if not isinstance(raw_outcomes, (list, tuple)):
            raise TypeError("bundle outcomes must be a list")
        bundle = cls(
            schema=str(payload.get("schema") or METHOD_EVALUATION_BUNDLE_SCHEMA),
            measurement_id=str(payload.get("measurement_id") or ""),
            pair_id=str(payload.get("pair_id") or ""),
            pair_sequence=int(payload.get("pair_sequence") or 0),
            snapshot_sha256=str(payload.get("snapshot_sha256") or ""),
            request_generation=int(payload.get("request_generation") or 0),
            outcomes=tuple(
                MethodEvaluationOutcome.from_mapping(value)
                for value in raw_outcomes
                if isinstance(value, Mapping)
            ),
            snapshot_payload=(
                dict(payload.get("snapshot_payload"))
                if isinstance(payload.get("snapshot_payload"), Mapping)
                else {}
            ),
        )
        supplied_primary_sha256 = str(payload.get("primary_outcome_sha256") or "")
        if (
            supplied_primary_sha256
            and supplied_primary_sha256 != bundle.primary.canonical_sha256()
        ):
            raise ValueError("primary method evaluation hash mismatch")
        supplied_sha256 = str(payload.get("method_evaluation_bundle_sha256") or "")
        if supplied_sha256 and supplied_sha256 != bundle.canonical_sha256():
            raise ValueError("method evaluation bundle hash mismatch")
        return bundle


def _geometry_result_payload(result: MeasurementGeometryResult) -> dict[str, object]:
    triangulated = result.triangulated
    estimate = result.estimate
    return {
        "triangulation_reason": result.triangulation_reason,
        "warnings": list(result.warnings),
        "match": None
        if result.match is None
        else {
            "left_center_px": list(result.match.left.center),
            "right_center_px": list(result.match.right.center),
            "class_name": result.match.left.class_name,
        },
        "triangulated": None
        if triangulated is None
        else {
            "point_m": list(triangulated.point_m),
            "range_m": float(triangulated.range_m),
            "closest_gap_m": float(triangulated.closest_gap_m),
            "source_point_left_rdf_m": (
                None
                if triangulated.source_point_left_rdf_m is None
                else list(triangulated.source_point_left_rdf_m)
            ),
            "method": str(triangulated.method),
        },
        "estimate": None
        if estimate is None
        else {
            "distance_m": float(estimate.distance_m),
            "sync_quality": str(estimate.sync_quality),
            "warnings": list(estimate.warnings),
        },
    }


def evaluate_method_request(
    request: MethodEvaluationRequest,
    *,
    worker_started_monotonic_ns: int | None = None,
) -> MethodEvaluationOutcome:
    """Evaluate one request without reading mutable UI or device state."""

    started_ns = int(worker_started_monotonic_ns or time.monotonic_ns())
    if request.attempt_payload is None:
        return MethodEvaluationOutcome(
            measurement_id=request.measurement_id,
            pair_id=request.pair_id,
            pair_sequence=request.pair_sequence,
            snapshot_sha256=request.snapshot_sha256,
            request_generation=request.request_generation,
            method_id=request.method_id,
            method_instance_id=request.method_instance_id,
            status="BLOCKED",
            reasons=request.blocked_reasons,
            enqueued_monotonic_ns=request.enqueued_monotonic_ns,
            worker_started_monotonic_ns=started_ns,
            completed_monotonic_ns=time.monotonic_ns(),
            reused_inputs=request.reused_inputs,
        )
    attempt = MeasurementAttempt.from_canonical_payload(request.attempt_payload)
    if attempt.measurement_method_id != request.method_id:
        raise ValueError("request method does not match MeasurementAttempt")
    if request.blocked_reasons:
        return MethodEvaluationOutcome(
            measurement_id=request.measurement_id,
            pair_id=request.pair_id,
            pair_sequence=request.pair_sequence,
            snapshot_sha256=request.snapshot_sha256,
            request_generation=request.request_generation,
            method_id=request.method_id,
            method_instance_id=request.method_instance_id,
            status="BLOCKED",
            attempt_sha256=attempt.canonical_sha256(),
            attempt_payload=dict(request.attempt_payload),
            geometry_sha256=attempt.measurement_geometry_sha256,
            reasons=request.blocked_reasons,
            warnings=attempt.warnings,
            enqueued_monotonic_ns=request.enqueued_monotonic_ns,
            worker_started_monotonic_ns=started_ns,
            completed_monotonic_ns=time.monotonic_ns(),
            reused_inputs=request.reused_inputs,
        )
    geometry = calculate_measurement_geometry(attempt)
    payload = _geometry_result_payload(geometry)
    reasons: tuple[str, ...] = ()
    if geometry.match is None or geometry.triangulated is None or geometry.estimate is None:
        status = "BLOCKED"
        reasons = (str(geometry.triangulation_reason or "geometry_unavailable"),)
    else:
        status = "ESTIMATED" if geometry.warnings else "VALID"
    return MethodEvaluationOutcome(
        measurement_id=request.measurement_id,
        pair_id=request.pair_id,
        pair_sequence=request.pair_sequence,
        snapshot_sha256=request.snapshot_sha256,
        request_generation=request.request_generation,
        method_id=request.method_id,
        method_instance_id=request.method_instance_id,
        status=status,
        attempt_sha256=attempt.canonical_sha256(),
        attempt_payload=dict(request.attempt_payload),
        geometry_sha256=attempt.measurement_geometry_sha256,
        result=payload,
        reasons=reasons,
        warnings=tuple(geometry.warnings),
        enqueued_monotonic_ns=request.enqueued_monotonic_ns,
        worker_started_monotonic_ns=started_ns,
        completed_monotonic_ns=time.monotonic_ns(),
        reused_inputs=request.reused_inputs,
    )


def failed_outcome(
    request: MethodEvaluationRequest,
    *,
    status: str,
    reason: str,
    worker_started_monotonic_ns: int = 0,
) -> MethodEvaluationOutcome:
    if status not in {"FAILED", "TIMEOUT", "BACKPRESSURE_BLOCKED"}:
        raise ValueError("failed_outcome requires a terminal failure status")
    return MethodEvaluationOutcome(
        measurement_id=request.measurement_id,
        pair_id=request.pair_id,
        pair_sequence=request.pair_sequence,
        snapshot_sha256=request.snapshot_sha256,
        request_generation=request.request_generation,
        method_id=request.method_id,
        method_instance_id=request.method_instance_id,
        status=status,
        attempt_sha256=(
            "" if request.attempt_payload is None else _sha256(request.attempt_payload)
        ),
        attempt_payload=(
            {} if request.attempt_payload is None else dict(request.attempt_payload)
        ),
        reasons=(str(reason),),
        enqueued_monotonic_ns=request.enqueued_monotonic_ns,
        worker_started_monotonic_ns=int(worker_started_monotonic_ns),
        completed_monotonic_ns=time.monotonic_ns(),
        reused_inputs=request.reused_inputs,
    )


__all__ = [
    "METHOD_EVALUATION_BUNDLE_SCHEMA",
    "METHOD_EVALUATION_PRIMARY_METHOD",
    "METHOD_EVALUATION_QUEUE_LIMIT",
    "METHOD_EVALUATION_REQUEST_SCHEMA",
    "METHOD_EVALUATION_SCHEMA",
    "METHOD_EVALUATION_STATUSES",
    "METHOD_EVALUATION_TIMEOUT_S",
    "METHOD_RESULT_RELATIVE_PATHS",
    "FrozenMeasurementPairSnapshot",
    "MethodEvaluationBundle",
    "MethodEvaluationOutcome",
    "MethodEvaluationRequest",
    "evaluate_method_request",
    "failed_outcome",
]
