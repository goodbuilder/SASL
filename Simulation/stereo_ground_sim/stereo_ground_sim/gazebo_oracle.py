from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any

from stereo_ground_sim.coordinate_frames import (
    FrameId,
    FramePoint,
    RigidTransform3D,
    transform_point,
)


class OracleSourceRole(str, Enum):
    TARGET_REFERENCE_LINK = "target_reference_link"
    BASE_LINK = "base_link"
    MODEL_POSE = "model_pose"


class OracleMatchKind(str, Enum):
    EXACT = "exact"
    UNIQUE_SCOPED_SUFFIX = "unique_scoped_suffix"


class OracleStatus(str, Enum):
    OK_TARGET_REFERENCE_LINK = "ok_target_reference_link"
    OK_BASE_LINK_FALLBACK = "ok_base_link_fallback"
    OK_MODEL_POSE_FALLBACK = "ok_model_pose_fallback"
    ORACLE_SOURCE_NOT_FOUND = "oracle_source_not_found"
    AMBIGUOUS_ENTITY_NAME = "ambiguous_entity_name"
    MALFORMED_POSE_PAYLOAD = "malformed_pose_payload"
    MISSING_POSITION = "missing_position"
    NON_FINITE_POSITION = "non_finite_position"
    UNSUPPORTED_POSE_FORMAT = "unsupported_pose_format"
    UNSUPPORTED_NONZERO_LINK_OFFSET = "unsupported_nonzero_link_offset"


@dataclass(frozen=True)
class OracleSourceCandidate:
    role: OracleSourceRole
    exact_names: tuple[str, ...]
    scoped_suffixes: tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "role", OracleSourceRole(self.role))
        exact_names = _validate_unique_names(self.exact_names, field_name="exact_names")
        suffixes = _validate_unique_names(self.scoped_suffixes, field_name="scoped_suffixes")
        for suffix in suffixes:
            if "::" not in suffix:
                raise ValueError("scoped suffix must contain '::'")
        object.__setattr__(self, "exact_names", exact_names)
        object.__setattr__(self, "scoped_suffixes", suffixes)


@dataclass(frozen=True)
class OracleSourcePolicy:
    sources: tuple[OracleSourceCandidate, ...]

    def __post_init__(self) -> None:
        sources = tuple(self.sources)
        expected = (
            OracleSourceRole.TARGET_REFERENCE_LINK,
            OracleSourceRole.BASE_LINK,
            OracleSourceRole.MODEL_POSE,
        )
        roles = tuple(source.role for source in sources)
        if roles != expected:
            raise ValueError("source order must be target_reference_link, base_link, model_pose")
        if len(set(roles)) != len(roles):
            raise ValueError("source roles must not be duplicated")
        object.__setattr__(self, "sources", sources)


@dataclass(frozen=True)
class OraclePoseSelection:
    valid: bool
    status: OracleStatus
    source_role: OracleSourceRole | None
    matched_entity_name: str | None
    match_kind: OracleMatchKind | None
    source_priority: int | None
    fallback_used: bool
    fallback_reason: str
    point_world_enu_m: FramePoint | None

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", OracleStatus(self.status))
        if self.source_role is not None:
            object.__setattr__(self, "source_role", OracleSourceRole(self.source_role))
        if self.match_kind is not None:
            object.__setattr__(self, "match_kind", OracleMatchKind(self.match_kind))
        if self.point_world_enu_m is not None and self.point_world_enu_m.frame != FrameId.GAZEBO_WORLD_ENU:
            raise ValueError("point_world_enu_m must use gazebo_world_enu frame")


@dataclass(frozen=True)
class GazeboOracleResult:
    valid: bool
    status: OracleStatus
    source_role: OracleSourceRole | None
    matched_entity_name: str | None
    match_kind: OracleMatchKind | None
    source_priority: int | None
    fallback_used: bool
    fallback_reason: str
    point_world_enu_m: FramePoint | None
    point_baseline_center_frd_m: FramePoint | None

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", OracleStatus(self.status))
        if self.source_role is not None:
            object.__setattr__(self, "source_role", OracleSourceRole(self.source_role))
        if self.match_kind is not None:
            object.__setattr__(self, "match_kind", OracleMatchKind(self.match_kind))
        if self.point_world_enu_m is not None and self.point_world_enu_m.frame != FrameId.GAZEBO_WORLD_ENU:
            raise ValueError("point_world_enu_m must use gazebo_world_enu frame")
        if (
            self.point_baseline_center_frd_m is not None
            and self.point_baseline_center_frd_m.frame != FrameId.BASELINE_CENTER_FRD
        ):
            raise ValueError("point_baseline_center_frd_m must use baseline_center_frd frame")


def _validate_unique_names(values: tuple[str, ...], *, field_name: str) -> tuple[str, ...]:
    names = tuple(values)
    if len(set(names)) != len(names):
        raise ValueError(f"{field_name} must not contain duplicates")
    for name in names:
        if not isinstance(name, str) or not name:
            raise ValueError(f"{field_name} entries must be non-empty strings")
    return names


def x500_oracle_source_policy(
    *,
    model_name: str = "x500_15m",
    target_reference_link_name: str = "target_reference_link",
    base_link_name: str = "base_link",
) -> OracleSourcePolicy:
    """Return the default Simulation x500 oracle source policy.

    The target reference link is the body-fixed reference point for Simulation
    3D position-error evaluation. Phase 2A does not treat it as a BBox center,
    visual center, or YOLO detection center.
    """

    for field_name, value in (
        ("model_name", model_name),
        ("target_reference_link_name", target_reference_link_name),
        ("base_link_name", base_link_name),
    ):
        if not isinstance(value, str) or not value:
            raise ValueError(f"{field_name} must be a non-empty string")

    return OracleSourcePolicy(
        sources=(
            OracleSourceCandidate(
                role=OracleSourceRole.TARGET_REFERENCE_LINK,
                exact_names=(
                    f"{model_name}::{target_reference_link_name}",
                    target_reference_link_name,
                ),
                scoped_suffixes=(f"::{target_reference_link_name}",),
            ),
            OracleSourceCandidate(
                role=OracleSourceRole.BASE_LINK,
                exact_names=(f"{model_name}::{base_link_name}", base_link_name),
                scoped_suffixes=(f"::{base_link_name}",),
            ),
            OracleSourceCandidate(
                role=OracleSourceRole.MODEL_POSE,
                exact_names=(model_name,),
                scoped_suffixes=(f"::{model_name}",),
            ),
        )
    )


def make_gazebo_world_to_baseline_transform(
    *,
    origin_world_enu_m: tuple[float, float, float],
    rotation_baseline_world: tuple[tuple[float, float, float], ...],
) -> RigidTransform3D:
    """Build the Gazebo World ENU -> baseline-center FRD transform.

    `rotation_baseline_world` is R_B_W. The returned transform uses
    `p_B = R_B_W @ (p_W - o_W_B)` with meter-valued points.
    """

    return RigidTransform3D(
        source_frame=FrameId.GAZEBO_WORLD_ENU,
        target_frame=FrameId.BASELINE_CENTER_FRD,
        rotation_target_source=rotation_baseline_world,
        origin_target_in_source_m=origin_world_enu_m,
    )


def select_oracle_pose(payload: Any, policy: OracleSourcePolicy) -> OraclePoseSelection:
    if not isinstance(policy, OracleSourcePolicy):
        raise ValueError("policy must be an OracleSourcePolicy")
    if not isinstance(payload, (dict, list)):
        return _invalid_selection(OracleStatus.MALFORMED_POSE_PAYLOAD)

    records = _named_pose_records(payload)
    for priority, source in enumerate(policy.sources):
        fallback_used, fallback_reason = _fallback_metadata_for_priority(priority)
        match_status, record, match_kind = _match_source_record(records, source)
        if match_status == OracleStatus.AMBIGUOUS_ENTITY_NAME:
            return _invalid_selection(OracleStatus.AMBIGUOUS_ENTITY_NAME)
        if record is not None and match_kind is not None:
            return _selection_from_record(
                records=records,
                policy=policy,
                record=record,
                status=_ok_status_for_role(source.role),
                role=source.role,
                match_kind=match_kind,
                source_priority=priority,
                fallback_used=fallback_used,
                fallback_reason=fallback_reason,
            )

    return _invalid_selection(OracleStatus.ORACLE_SOURCE_NOT_FOUND)


def ground_truth_from_gazebo_pose_payload(
    payload: Any,
    policy: OracleSourcePolicy,
    world_to_baseline_transform: RigidTransform3D,
) -> GazeboOracleResult:
    if world_to_baseline_transform.source_frame != FrameId.GAZEBO_WORLD_ENU:
        raise ValueError("world_to_baseline_transform source frame must be gazebo_world_enu")
    if world_to_baseline_transform.target_frame != FrameId.BASELINE_CENTER_FRD:
        raise ValueError("world_to_baseline_transform target frame must be baseline_center_frd")

    selection = select_oracle_pose(payload, policy)
    if not selection.valid or selection.point_world_enu_m is None:
        return GazeboOracleResult(
            valid=False,
            status=selection.status,
            source_role=selection.source_role,
            matched_entity_name=selection.matched_entity_name,
            match_kind=selection.match_kind,
            source_priority=selection.source_priority,
            fallback_used=selection.fallback_used,
            fallback_reason=selection.fallback_reason,
            point_world_enu_m=selection.point_world_enu_m,
            point_baseline_center_frd_m=None,
        )

    point_baseline = transform_point(selection.point_world_enu_m, world_to_baseline_transform)
    return GazeboOracleResult(
        valid=True,
        status=selection.status,
        source_role=selection.source_role,
        matched_entity_name=selection.matched_entity_name,
        match_kind=selection.match_kind,
        source_priority=selection.source_priority,
        fallback_used=selection.fallback_used,
        fallback_reason=selection.fallback_reason,
        point_world_enu_m=selection.point_world_enu_m,
        point_baseline_center_frd_m=point_baseline,
    )


@dataclass(frozen=True)
class _NamedRecord:
    name: str
    record: Mapping[str, Any]


def _named_pose_records(payload: dict[str, Any] | list[Any]) -> tuple[_NamedRecord, ...]:
    records: list[_NamedRecord] = []

    def walk(value: Any) -> None:
        if isinstance(value, Mapping):
            name = value.get("name")
            if isinstance(name, str) and name:
                records.append(_NamedRecord(name=name, record=value))
            for child in value.values():
                walk(child)
        elif isinstance(value, list):
            for child in value:
                walk(child)

    walk(payload)
    return tuple(records)


def _selection_from_record(
    *,
    records: tuple[_NamedRecord, ...],
    policy: OracleSourcePolicy,
    record: _NamedRecord,
    status: OracleStatus,
    role: OracleSourceRole,
    match_kind: OracleMatchKind,
    source_priority: int,
    fallback_used: bool,
    fallback_reason: str,
) -> OraclePoseSelection:
    if role in (OracleSourceRole.TARGET_REFERENCE_LINK, OracleSourceRole.BASE_LINK) and _has_valid_entity_id(
        record.record
    ):
        parse_status, point = _pose_v_link_world_position(records=records, policy=policy, link_record=record)
    elif role == OracleSourceRole.MODEL_POSE and _has_valid_entity_id(record.record):
        parse_status, xyz = _parse_protojson_vector3(_position_field(record.record), field_name="position")
        point = FramePoint(xyz, FrameId.GAZEBO_WORLD_ENU) if parse_status is None and xyz is not None else None
    else:
        parse_status, point = _parse_world_position(record.record)
    if parse_status is not None:
        return OraclePoseSelection(
            valid=False,
            status=parse_status,
            source_role=role,
            matched_entity_name=record.name,
            match_kind=match_kind,
            source_priority=source_priority,
            fallback_used=fallback_used,
            fallback_reason=fallback_reason,
            point_world_enu_m=None,
        )
    return OraclePoseSelection(
        valid=True,
        status=status,
        source_role=role,
        matched_entity_name=record.name,
        match_kind=match_kind,
        source_priority=source_priority,
        fallback_used=fallback_used,
        fallback_reason=fallback_reason,
        point_world_enu_m=point,
    )


def _match_source_record(
    records: tuple[_NamedRecord, ...], source: OracleSourceCandidate
) -> tuple[OracleStatus, _NamedRecord | None, OracleMatchKind | None]:
    for exact_name in source.exact_names:
        matches = [record for record in records if record.name == exact_name]
        if len(matches) > 1:
            return OracleStatus.AMBIGUOUS_ENTITY_NAME, None, None
        if len(matches) == 1:
            return _ok_status_for_role(source.role), matches[0], OracleMatchKind.EXACT

    suffix_matches = [
        record
        for record in records
        if any(record.name.endswith(suffix) for suffix in source.scoped_suffixes)
    ]
    if len(suffix_matches) > 1:
        return OracleStatus.AMBIGUOUS_ENTITY_NAME, None, None
    if len(suffix_matches) == 1:
        return _ok_status_for_role(source.role), suffix_matches[0], OracleMatchKind.UNIQUE_SCOPED_SUFFIX
    return OracleStatus.ORACLE_SOURCE_NOT_FOUND, None, None


def _pose_v_link_world_position(
    *,
    records: tuple[_NamedRecord, ...],
    policy: OracleSourcePolicy,
    link_record: _NamedRecord,
) -> tuple[OracleStatus | None, FramePoint | None]:
    model_source = policy.sources[2]
    model_match_status, model_record, _match_kind = _match_source_record(records, model_source)
    if model_match_status == OracleStatus.AMBIGUOUS_ENTITY_NAME:
        return OracleStatus.AMBIGUOUS_ENTITY_NAME, None
    if model_record is None:
        return OracleStatus.ORACLE_SOURCE_NOT_FOUND, None
    if not _has_valid_entity_id(model_record.record):
        return OracleStatus.UNSUPPORTED_POSE_FORMAT, None

    link_status, link_local = _parse_protojson_vector3(_position_field(link_record.record), field_name="position")
    if link_status is not None or link_local is None:
        return link_status, None
    if math.sqrt(sum(value * value for value in link_local)) > 1.0e-9:
        return OracleStatus.UNSUPPORTED_NONZERO_LINK_OFFSET, None

    model_status, model_world = _parse_protojson_vector3(_position_field(model_record.record), field_name="position")
    if model_status is not None or model_world is None:
        return model_status, None
    return None, FramePoint(model_world, FrameId.GAZEBO_WORLD_ENU)


def _has_valid_entity_id(record: Mapping[str, Any]) -> bool:
    entity_id = record.get("id")
    if isinstance(entity_id, bool) or not isinstance(entity_id, int):
        return False
    return entity_id > 0


def _position_field(record: Mapping[str, Any]) -> Any:
    if "position" in record:
        return record["position"]
    if "pos" in record:
        return record["pos"]
    return None


def _parse_protojson_vector3(position: Any, *, field_name: str) -> tuple[OracleStatus | None, tuple[float, float, float] | None]:
    if position is None:
        return OracleStatus.MISSING_POSITION, None
    if not isinstance(position, Mapping):
        return OracleStatus.UNSUPPORTED_POSE_FORMAT, None

    values: list[float] = []
    for axis in ("x", "y", "z"):
        value = position.get(axis, 0.0)
        if isinstance(value, bool):
            return OracleStatus.UNSUPPORTED_POSE_FORMAT, None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return OracleStatus.UNSUPPORTED_POSE_FORMAT, None
        if not math.isfinite(parsed):
            return OracleStatus.NON_FINITE_POSITION, None
        values.append(parsed)
    return None, tuple(values)


def _parse_world_position(record: Mapping[str, Any]) -> tuple[OracleStatus | None, FramePoint | None]:
    if "position" in record:
        position = record["position"]
    elif "pos" in record:
        position = record["pos"]
    else:
        return OracleStatus.MISSING_POSITION, None
    if not isinstance(position, Mapping):
        return OracleStatus.UNSUPPORTED_POSE_FORMAT, None
    if not all(axis in position for axis in ("x", "y", "z")):
        return OracleStatus.MISSING_POSITION, None

    values: list[float] = []
    for axis in ("x", "y", "z"):
        value = position[axis]
        if isinstance(value, bool):
            return OracleStatus.UNSUPPORTED_POSE_FORMAT, None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return OracleStatus.UNSUPPORTED_POSE_FORMAT, None
        if not math.isfinite(parsed):
            return OracleStatus.NON_FINITE_POSITION, None
        values.append(parsed)
    return None, FramePoint(tuple(values), FrameId.GAZEBO_WORLD_ENU)


def _invalid_selection(status: OracleStatus) -> OraclePoseSelection:
    return OraclePoseSelection(
        valid=False,
        status=status,
        source_role=None,
        matched_entity_name=None,
        match_kind=None,
        source_priority=None,
        fallback_used=False,
        fallback_reason="",
        point_world_enu_m=None,
    )


def _fallback_metadata_for_priority(priority: int) -> tuple[bool, str]:
    if priority == 0:
        return False, ""
    if priority == 1:
        return True, "target_reference_link_not_found"
    if priority == 2:
        return True, "target_reference_link_and_base_link_not_found"
    raise ValueError("unknown oracle source priority")


def _ok_status_for_role(role: OracleSourceRole) -> OracleStatus:
    if role == OracleSourceRole.TARGET_REFERENCE_LINK:
        return OracleStatus.OK_TARGET_REFERENCE_LINK
    if role == OracleSourceRole.BASE_LINK:
        return OracleStatus.OK_BASE_LINK_FALLBACK
    if role == OracleSourceRole.MODEL_POSE:
        return OracleStatus.OK_MODEL_POSE_FALLBACK
    raise ValueError(f"unsupported oracle source role: {role!r}")
