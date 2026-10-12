"""Canonical method fanout using a shared immutable pair snapshot.

Attitudes: body FRD -> earth NED, wxyz. GPS baselines: NED metres.
Optical R,t: X_right_RDF = R_right_from_left X_left_RDF + t_m.
Public point_m: baseline-center FRD; source_point_left_rdf_m: left optical RDF.
"""

from __future__ import annotations
import time
from collections.abc import Mapping
from Workstation.workstation_core.immutable_json import freeze_json, thaw_json
from Workstation.measurement.geometry_resolver import resolve_measurement_attempt
from dataclasses import dataclass
from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_IDS,
    MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS,
    PIXHAWK_RTK_LIVE,
    latest_status_by_method,
)
from Workstation.workstation_core.method_evaluation import (
    FrozenMeasurementPairSnapshot,
    MethodEvaluationRequest,
)
from typing import Any, Callable
from Workstation.measurement import limits
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.frozen_inputs import FrozenGeometryInputs, FrozenPairInputs
from Workstation.measurement.state import UNSET, read, MethodPlanningState


@dataclass(frozen=True)
class MethodPlanningInputs:
    pass


@dataclass(frozen=True)
class MethodPlanningActions:
    _freeze_measurement_pair_inputs: Callable[..., Any]
    _measurement_method_statuses: Callable[..., Any]
    _build_measurement_attempt: Callable[..., Any]


class MethodPlanningService:
    """Canonical method fanout using a shared immutable pair snapshot."""

    def __init__(
        self,
        *,
        state: MethodPlanningState,
        inputs: MethodPlanningInputs,
        actions: MethodPlanningActions,
    ) -> None:
        self.state = state
        self.inputs = inputs
        self.actions = actions

    def _plan_method_evaluations(self, pair: object) -> tuple[
        tuple[MethodEvaluationRequest, ...],
        dict[str, str],
        dict[str, dict[str, object]],
        FrozenMeasurementPairSnapshot,
    ]:
        """Requests for the ready methods only, plus why the others were not attempted."""
        frozen = self.actions._freeze_measurement_pair_inputs(pair)
        if isinstance(frozen, FrozenPairInputs):
            return plan_frozen_pair(
                frozen, enqueued_monotonic_ns=time.monotonic_ns()
            ).legacy_values()
        snapshot = frozen.get("snapshot")
        if not isinstance(snapshot, FrozenMeasurementPairSnapshot):
            raise TypeError("frozen Measurement snapshot unavailable")
        geometry_inputs = frozen.get("geometry_inputs")
        statuses = latest_status_by_method(
            geometry_inputs.statuses
            if isinstance(geometry_inputs, FrozenGeometryInputs)
            else self.actions._measurement_method_statuses()
        )
        requests: list[MethodEvaluationRequest] = []
        not_attempted: dict[str, str] = {}
        evidence: dict[str, dict[str, object]] = {}
        enqueued_ns = time.monotonic_ns()
        snapshot_sha256 = snapshot.canonical_sha256()
        snapshot_payload = snapshot.canonical_payload()
        for method_id in MEASUREMENT_METHOD_IDS:
            status = statuses.get(method_id)
            if status is None:
                not_attempted[method_id] = MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS[
                    method_id
                ]
                continue
            try:
                evidence[method_id] = dict(status.to_mapping())
            except Exception:
                evidence[method_id] = {"instance_id": status.instance_id}
            if not status.ready:
                not_attempted[method_id] = "; ".join(status.reasons) or "not ready"
                continue
            try:
                attempt = self.actions._build_measurement_attempt(
                    pair, method_status=status, frozen_inputs=frozen, use_cache=False
                )
                attempt_payload = attempt.canonical_payload()
            except (
                AttributeError,
                KeyError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                not_attempted[method_id] = f"frozen method input error: {exc}"
                continue
            uses_live_gnss = (
                method_id == PIXHAWK_RTK_LIVE
                or "live_gnss" in dict(status.component_sources).values()
            )
            reused_inputs = snapshot.reused_inputs if uses_live_gnss else ()
            requests.append(
                MethodEvaluationRequest(
                    measurement_id=snapshot.measurement_id,
                    pair_id=snapshot.pair_id,
                    pair_sequence=snapshot.pair_sequence,
                    snapshot_sha256=snapshot_sha256,
                    request_generation=snapshot.pair_sequence,
                    method_id=method_id,
                    method_instance_id=status.instance_id,
                    snapshot_payload=snapshot_payload,
                    attempt_payload=attempt_payload,
                    reused_inputs=reused_inputs,
                    enqueued_monotonic_ns=enqueued_ns,
                )
            )
        return (tuple(requests), not_attempted, evidence, snapshot)

    def _method_evaluation_requests_for_pair(
        self, pair: object
    ) -> tuple[MethodEvaluationRequest, ...]:
        return self._plan_method_evaluations(pair)[0]


@dataclass(frozen=True)
class MethodEvaluationPlan:
    """Owned fanout plan; detached exports cannot rewrite method evidence."""

    requests: tuple[MethodEvaluationRequest, ...]
    not_attempted: Mapping[str, object]
    evidence: Mapping[str, object]
    snapshot: FrozenMeasurementPairSnapshot

    def __post_init__(self) -> None:
        object.__setattr__(self, "requests", tuple(self.requests))
        object.__setattr__(self, "not_attempted", freeze_json(self.not_attempted))
        object.__setattr__(self, "evidence", freeze_json(self.evidence))

    def legacy_values(self) -> tuple:
        return (
            self.requests,
            thaw_json(self.not_attempted),
            thaw_json(self.evidence),
            self.snapshot,
        )


def plan_frozen_pair(
    frozen: FrozenPairInputs, *, enqueued_monotonic_ns: int
) -> MethodEvaluationPlan:
    """Pure canonical fanout; all readiness/config/pose evidence was captured.

    There are no port reads here. A missing method/calibration is a recorded
    not-attempted reason, and source payload/hash ownership stays immutable.
    """
    context = frozen.geometry_inputs
    if context is None:
        raise ValueError("pair has no frozen geometry inputs")
    snapshot = frozen.snapshot
    statuses = latest_status_by_method(context.statuses)
    requests: list[MethodEvaluationRequest] = []
    not_attempted: dict[str, str] = {}
    evidence: dict[str, object] = {}
    for method_id in MEASUREMENT_METHOD_IDS:
        status = statuses.get(method_id)
        if status is None:
            not_attempted[method_id] = MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS[
                method_id
            ]
            continue
        evidence[method_id] = status.to_mapping()
        if not status.ready:
            not_attempted[method_id] = "; ".join(status.reasons) or "not ready"
            continue
        try:
            attempt = resolve_measurement_attempt(frozen, status, context)
        except (AttributeError, KeyError, RuntimeError, TypeError, ValueError) as exc:
            not_attempted[method_id] = f"frozen method input error: {exc}"
            continue
        live_gnss = (
            method_id == PIXHAWK_RTK_LIVE
            or "live_gnss" in dict(status.component_sources).values()
        )
        requests.append(
            MethodEvaluationRequest(
                measurement_id=snapshot.measurement_id,
                pair_id=snapshot.pair_id,
                pair_sequence=snapshot.pair_sequence,
                snapshot_sha256=snapshot.canonical_sha256(),
                request_generation=snapshot.pair_sequence,
                method_id=method_id,
                method_instance_id=status.instance_id,
                snapshot_payload=snapshot.canonical_payload(),
                attempt_payload=attempt.canonical_payload(),
                reused_inputs=snapshot.reused_inputs if live_gnss else (),
                enqueued_monotonic_ns=enqueued_monotonic_ns,
            )
        )
    return MethodEvaluationPlan(tuple(requests), not_attempted, evidence, snapshot)
