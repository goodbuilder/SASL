"""Independent raw Gazebo ``Pose_V`` parser and optional stream adapter."""

from __future__ import annotations

import json
import math
import subprocess
import threading
from dataclasses import dataclass
from typing import Any, Callable, Iterable

from .pose_math import (
    GAZEBO_LINK_FLU_FROM_OPTICAL_RDF,
    Matrix3,
    Pose3D,
    matrix_multiply,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
)


class GazeboPosePayloadError(ValueError):
    """Raised when a raw Gazebo payload cannot provide the configured entity."""


@dataclass(frozen=True)
class GazeboPoseRecord:
    stamp_ns: int
    entity_name: str
    entity_id: int
    position_world_enu_m: tuple[float, float, float]
    orientation_world_from_entity_xyzw: tuple[float, float, float, float]

    def as_pose(self) -> Pose3D:
        return Pose3D(
            self.position_world_enu_m,
            quaternion_xyzw_to_matrix(self.orientation_world_from_entity_xyzw),
            "gazebo_world_enu",
            self.stamp_ns,
        )

    def as_optical_pose(
        self,
        camera_link_rotation_from_optical: Matrix3 = GAZEBO_LINK_FLU_FROM_OPTICAL_RDF,
    ) -> Pose3D:
        """Convert a Gazebo FLU optical-link pose into an RDF optical pose."""

        rotation = matrix_multiply(
            quaternion_xyzw_to_matrix(self.orientation_world_from_entity_xyzw),
            camera_link_rotation_from_optical,
        )
        return Pose3D(
            self.position_world_enu_m,
            rotation,
            "camera_optical_rdf",
            self.stamp_ns,
        )


def _stamp_ns(header: Any) -> int:
    if not isinstance(header, dict):
        raise GazeboPosePayloadError("Pose_V header is missing")
    stamp = header.get("stamp", header)
    if not isinstance(stamp, dict):
        raise GazeboPosePayloadError("Pose_V header stamp is missing")
    try:
        sec = int(stamp.get("sec", stamp.get("seconds", 0)))
        nsec = int(stamp.get("nsec", stamp.get("nanosec", stamp.get("nanoseconds", 0))))
    except (TypeError, ValueError):
        raise GazeboPosePayloadError("Pose_V stamp is invalid") from None
    if nsec < 0 or nsec >= 1_000_000_000:
        raise GazeboPosePayloadError("Pose_V nanoseconds are out of range")
    return sec * 1_000_000_000 + nsec


def _iter_dicts(value: Any) -> Iterable[dict[str, Any]]:
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from _iter_dicts(child)
    elif isinstance(value, list):
        for child in value:
            yield from _iter_dicts(child)


def _number(mapping: dict[str, Any], key: str) -> float:
    try:
        value = float(mapping.get(key, 0.0))
    except (TypeError, ValueError):
        raise GazeboPosePayloadError(f"Pose_V field {key} is invalid") from None
    if not math.isfinite(value):
        raise GazeboPosePayloadError(f"Pose_V field {key} is non-finite")
    return value


def _entity_id(value: Any) -> int:
    if value is None:
        return -1
    try:
        return int(value)
    except (TypeError, ValueError):
        raise GazeboPosePayloadError("Pose_V entity id is invalid") from None


def _mappings(payload: dict[str, Any], entity_name: str) -> list[dict[str, Any]]:
    return [
        mapping
        for mapping in _iter_dicts(payload)
        if mapping.get("name") == entity_name
        and ("position" in mapping or "pos" in mapping)
    ]


def _record(mapping: dict[str, Any], stamp_ns: int) -> GazeboPoseRecord:
    position = mapping.get("position", mapping.get("pos"))
    orientation = mapping.get("orientation", mapping.get("rot", {}))
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise GazeboPosePayloadError("Pose_V position/orientation is missing")
    name = mapping.get("name")
    if not isinstance(name, str) or not name:
        raise GazeboPosePayloadError("Pose_V entity name is missing")
    quaternion = (
        _number(orientation, "x"),
        _number(orientation, "y"),
        _number(orientation, "z"),
        _number(orientation, "w"),
    )
    # Validate normalization/non-zero norm through the matrix converter.
    quaternion_xyzw_to_matrix(quaternion)
    return GazeboPoseRecord(
        stamp_ns=stamp_ns,
        entity_name=name,
        entity_id=_entity_id(mapping.get("id")),
        position_world_enu_m=(
            _number(position, "x"),
            _number(position, "y"),
            _number(position, "z"),
        ),
        orientation_world_from_entity_xyzw=quaternion,
    )


def parse_pose_v_payload(
    payload: dict[str, Any], *, entity_name: str
) -> GazeboPoseRecord:
    """Parse one named entity while preserving the Gazebo source stamp."""

    if not isinstance(payload, dict):
        raise GazeboPosePayloadError("Pose_V payload must be an object")
    matches = _mappings(payload, entity_name)
    if len(matches) != 1:
        raise GazeboPosePayloadError(
            f"expected exactly one Pose_V entity {entity_name!r}, found {len(matches)}"
        )
    header = payload.get("header")
    if isinstance(header, dict):
        stamp_ns = _stamp_ns(header)
    else:
        entity_header = matches[0].get("header")
        stamp_ns = _stamp_ns(entity_header)
    return _record(matches[0], stamp_ns)


def resolve_pose_v_record_world(
    payload: dict[str, Any],
    record: GazeboPoseRecord,
    *,
    parent_model_name: str,
) -> GazeboPoseRecord:
    """Compose a scoped link pose with its model world pose."""

    model_matches = _mappings(payload, parent_model_name)
    if len(model_matches) != 1:
        raise GazeboPosePayloadError(
            f"expected exactly one parent model {parent_model_name!r}, found {len(model_matches)}"
        )
    model = _record(model_matches[0], record.stamp_ns)
    model_rotation = quaternion_xyzw_to_matrix(model.orientation_world_from_entity_xyzw)
    link_rotation = quaternion_xyzw_to_matrix(record.orientation_world_from_entity_xyzw)
    # Explicit matrix/vector composition avoids any quaternion order ambiguity.
    from .pose_math import matrix_vector

    rotated_position = matrix_vector(model_rotation, record.position_world_enu_m)
    position = tuple(model.position_world_enu_m[index] + rotated_position[index] for index in range(3))
    rotation = matrix_multiply(model_rotation, link_rotation)
    return GazeboPoseRecord(
        stamp_ns=record.stamp_ns,
        entity_name=record.entity_name,
        entity_id=record.entity_id,
        position_world_enu_m=position,  # type: ignore[arg-type]
        orientation_world_from_entity_xyzw=matrix_to_quaternion_xyzw(rotation),
    )


def parse_world_pose_v_payload(
    payload: dict[str, Any], *, entity_name: str, parent_model_name: str
) -> GazeboPoseRecord:
    record = parse_pose_v_payload(payload, entity_name=entity_name)
    if entity_name == parent_model_name:
        return record
    return resolve_pose_v_record_world(
        payload, record, parent_model_name=parent_model_name
    )


class GazeboPoseStream:
    """Read one raw Gazebo topic in a stoppable background thread.

    The stream is opt-in.  It only observes ``gz topic`` output and exposes
    parsed samples through a callback; it contains no Gazebo mutation API.
    """

    def __init__(
        self,
        *,
        topic: str,
        entity_name: str,
        parent_model_name: str,
        on_record: Callable[[GazeboPoseRecord], None],
        on_error: Callable[[str], None] | None = None,
        executable: str = "gz",
    ) -> None:
        self._command = (
            executable,
            "topic",
            "-e",
            "-t",
            str(topic),
            "--json-output",
        )
        self._entity_name = str(entity_name)
        self._parent_model_name = str(parent_model_name)
        self._on_record = on_record
        self._on_error = on_error or (lambda _message: None)
        self._shutdown = threading.Event()
        self._process: subprocess.Popen[str] | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        if not self._thread.is_alive():
            self._thread.start()

    def _run(self) -> None:
        while not self._shutdown.is_set():
            try:
                process = subprocess.Popen(
                    self._command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1,
                )
            except OSError as exc:
                self._on_error(f"failed to start Gazebo Pose_V stream: {exc}")
                self._shutdown.wait(0.5)
                continue
            self._process = process
            try:
                if process.stdout is not None:
                    for line in process.stdout:
                        if self._shutdown.is_set():
                            break
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            payload = json.loads(line)
                            record = parse_world_pose_v_payload(
                                payload,
                                entity_name=self._entity_name,
                                parent_model_name=self._parent_model_name,
                            )
                            self._on_record(record)
                        except (json.JSONDecodeError, GazeboPosePayloadError) as exc:
                            self._on_error(f"Gazebo Pose_V sample rejected: {exc}")
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                self._process = None
            if not self._shutdown.is_set():
                self._shutdown.wait(0.5)

    def shutdown(self) -> None:
        self._shutdown.set()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)


__all__ = [
    "GazeboPosePayloadError",
    "GazeboPoseRecord",
    "GazeboPoseStream",
    "parse_pose_v_payload",
    "parse_world_pose_v_payload",
    "resolve_pose_v_record_world",
]
