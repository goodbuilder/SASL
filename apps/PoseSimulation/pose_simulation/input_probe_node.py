"""Read-only ROS 2 probe for the external PoseSimulation inputs.

The probe subscribes to the GT, camera, and immutable context source
interfaces only.  It does not create
publishers, start a source process, calculate an attitude error, or send a
pose command.  The pure adapters and alignment functions remain usable on
machines without ROS 2; ROS imports are intentionally optional here.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Sequence

from .config import ConfigError, ResolvedConfig, _parse_cli_overrides, resolve_config
from .experiment_context import SimulationContextStore
from .input_adapters import (
    GroundTruthStreamValidator,
    StaticCameraSnapshotStore,
    adapt_camera_pose_diagnostic,
    adapt_ground_truth_pose,
)
from .input_types import (
    AlignedPoseInputs,
    CAMERA_OPTICAL_FRAME,
    DYNAMIC_STREAM_MODE,
    GAZEBO_SIM_CLOCK,
    GroundTruthPoseSample,
    CameraOpticalPoseSample,
    GT_SOURCE_STALE,
    QOS_INCOMPATIBLE,
    SOURCE_UNAVAILABLE,
    STATIC_SNAPSHOT_MODE,
    VALID,
    WAITING_FOR_GT,
    WAITING_FOR_LEFT_CAMERA,
    WAITING_FOR_RIGHT_CAMERA,
)
from .time_alignment import align_dynamic_streams, align_static_snapshot


try:  # Keep this module importable for pure adapter tests outside a ROS host.
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node as _RosNode
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from simulation_interfaces.msg import SimulationExperimentContext
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - depends on the execution host
    rclpy = None  # type: ignore[assignment]
    PoseStamped = None  # type: ignore[assignment,misc]
    String = None  # type: ignore[assignment,misc]
    _RosNode = object  # type: ignore[assignment,misc]
    DurabilityPolicy = None  # type: ignore[assignment,misc]
    HistoryPolicy = None  # type: ignore[assignment,misc]
    QoSProfile = None  # type: ignore[assignment,misc]
    ReliabilityPolicy = None  # type: ignore[assignment,misc]
    SimulationExperimentContext = None  # type: ignore[assignment,misc]


@dataclass(frozen=True)
class ProbeSnapshot:
    """Immutable, source-only state used by the text report and tests."""

    gt_topic: str
    left_camera_topic: str
    right_camera_topic: str
    camera_source_mode: str
    expected_clock_domain: str | None
    gt_topic_discovered: bool
    left_topic_discovered: bool
    right_topic_discovered: bool
    ground_truth: GroundTruthPoseSample | None
    left_camera: CameraOpticalPoseSample | None
    right_camera: CameraOpticalPoseSample | None
    alignment: AlignedPoseInputs
    qos_issues: tuple[str, ...] = ()
    gt_receive_age_ms: float | None = None
    gt_max_source_age_ms: float | None = None
    context_topic: str = ""
    context_topic_discovered: bool = True
    context_valid: bool = True
    context_status: str = "CONTEXT_VALID"
    context_reason: str = ""


def _yes_no(value: bool) -> str:
    return "YES" if value else "NO"


def _sample_status(sample: Any, missing: str) -> str:
    if sample is None:
        return missing
    return str(getattr(sample, "status", "INVALID"))


def _sample_stamp(sample: Any) -> str:
    if sample is None:
        return "<none>"
    value = getattr(sample, "stamp_ns", None)
    if value is None:
        value = getattr(sample, "gazebo_stamp_ns", None)
    return "<none>" if value is None else str(value)


def _sample_frame(sample: Any, fallback: str) -> str:
    if sample is None:
        return fallback
    if isinstance(sample, GroundTruthPoseSample):
        return sample.frame_id or "<empty>"
    return CAMERA_OPTICAL_FRAME


def _sample_schema(sample: CameraOpticalPoseSample | None) -> str:
    return "<none>" if sample is None else sample.source_schema or "<empty>"


def _sample_entity(sample: CameraOpticalPoseSample | None) -> str:
    if sample is None:
        return "<none>"
    entity_id = "<none>" if sample.entity_id is None else str(sample.entity_id)
    return f"{sample.entity_name or '<empty>'} (id={entity_id})"


def _sample_hash(sample: CameraOpticalPoseSample | None) -> str:
    return "<none>" if sample is None else sample.generated_sdf_sha256 or "<empty>"


def _overall_status(snapshot: ProbeSnapshot) -> str:
    if snapshot.qos_issues:
        return QOS_INCOMPATIBLE
    if snapshot.context_topic and not snapshot.context_valid:
        return snapshot.context_status or SOURCE_UNAVAILABLE
    if snapshot.alignment.valid:
        return VALID
    if not (
        snapshot.gt_topic_discovered
        or snapshot.left_topic_discovered
        or snapshot.right_topic_discovered
        or snapshot.ground_truth is not None
        or snapshot.left_camera is not None
        or snapshot.right_camera is not None
    ):
        return SOURCE_UNAVAILABLE
    return snapshot.alignment.status


def format_probe_report(snapshot: ProbeSnapshot) -> str:
    """Format a deterministic human-readable input contract report."""

    alignment = snapshot.alignment
    lines = [
        "PoseSimulation M2 input probe (read-only)",
        "POSE COMMAND: DISABLED",
        f"GT topic: {snapshot.gt_topic}",
        f"GT topic discovered: {_yes_no(snapshot.gt_topic_discovered)}",
        f"GT message received: {_yes_no(snapshot.ground_truth is not None)}",
        f"GT frame: {_sample_frame(snapshot.ground_truth, 'gazebo_world_enu')}",
        f"GT source stamp ns: {_sample_stamp(snapshot.ground_truth)}",
        f"GT receive age ms: {snapshot.gt_receive_age_ms if snapshot.gt_receive_age_ms is not None else '<none>'}",
        f"GT max source age ms: {snapshot.gt_max_source_age_ms if snapshot.gt_max_source_age_ms is not None else '<none>'}",
        f"GT status: {_sample_status(snapshot.ground_truth, WAITING_FOR_GT)}",
        f"Left camera topic: {snapshot.left_camera_topic}",
        f"Left camera topic discovered: {_yes_no(snapshot.left_topic_discovered)}",
        f"Left camera message received: {_yes_no(snapshot.left_camera is not None)}",
        f"Left camera schema: {_sample_schema(snapshot.left_camera)}",
        f"Left camera entity: {_sample_entity(snapshot.left_camera)}",
        f"Left camera SDF hash: {_sample_hash(snapshot.left_camera)}",
        f"Left camera source stamp ns: {_sample_stamp(snapshot.left_camera)}",
        f"Left camera status: {_sample_status(snapshot.left_camera, WAITING_FOR_LEFT_CAMERA)}",
        f"Right camera topic: {snapshot.right_camera_topic}",
        f"Right camera topic discovered: {_yes_no(snapshot.right_topic_discovered)}",
        f"Right camera message received: {_yes_no(snapshot.right_camera is not None)}",
        f"Right camera schema: {_sample_schema(snapshot.right_camera)}",
        f"Right camera entity: {_sample_entity(snapshot.right_camera)}",
        f"Right camera SDF hash: {_sample_hash(snapshot.right_camera)}",
        f"Right camera source stamp ns: {_sample_stamp(snapshot.right_camera)}",
        f"Right camera status: {_sample_status(snapshot.right_camera, WAITING_FOR_RIGHT_CAMERA)}",
        f"Experiment context topic: {snapshot.context_topic or '<not configured in fixture>'}",
        f"Experiment context discovered: {_yes_no(snapshot.context_topic_discovered)}",
        f"Experiment context status: {snapshot.context_status}",
        f"Experiment context reason: {snapshot.context_reason or '<none>'}",
        f"Camera source mode: {snapshot.camera_source_mode}",
        f"Clock domain: {snapshot.expected_clock_domain or 'UNRESOLVED'}",
        f"Evaluation stamp ns: {alignment.evaluation_stamp_ns if alignment.evaluation_stamp_ns is not None else '<none>'}",
        f"Evaluation ID: {alignment.evaluation_id or '<none>'}",
        f"Alignment method: {alignment.alignment_method}",
        f"Alignment status: {alignment.status}",
        f"Overall input status: {_overall_status(snapshot)}",
    ]
    if snapshot.qos_issues:
        lines.append("QoS issues: " + " | ".join(snapshot.qos_issues))
    return "\n".join(lines)


def _policy_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if name:
        return str(name).upper()
    return str(value).upper().replace("-", "_")


def _policy_matches(value: Any, expected: str) -> bool:
    actual = _policy_name(value)
    wanted = expected.upper()
    return actual == wanted or actual.endswith(f".{wanted}") or actual.endswith(f"::{wanted}")


def publisher_qos_compatible(
    publisher_info: Any,
    *,
    expected_reliability: str,
    expected_durability: str,
) -> bool:
    """Check only the QoS dimensions needed by the known source publishers.

    Missing introspection fields are treated as unknown rather than as an
    incompatibility.  A known incompatible reliability/durability policy is
    reported fail-closed by the probe.
    """

    qos = getattr(publisher_info, "qos_profile", None)
    if qos is None:
        return True
    reliability = getattr(qos, "reliability", None)
    durability = getattr(qos, "durability", None)
    if reliability is not None:
        if not _policy_matches(reliability, expected_reliability):
            return False
    if durability is not None:
        if not _policy_matches(durability, expected_durability):
            return False
    return True


class InputProbeNode(_RosNode):
    """Subscribe to and validate the external source interfaces only."""

    def __init__(self, config: ResolvedConfig) -> None:  # pragma: no cover - ROS wiring
        if rclpy is None or PoseStamped is None or String is None or SimulationExperimentContext is None:
            raise RuntimeError("InputProbeNode requires an installed ROS 2 runtime")
        values = config.values
        super().__init__(
            "pose_simulation_input_probe",
            namespace=str(values["POSESIM_NAMESPACE"]),
        )
        self._config = config
        self._values = values
        self._gt_topic = str(values["POSESIM_GT_TOPIC"])
        self._left_topic = str(values["POSESIM_LEFT_CAMERA_POSE_TOPIC"])
        self._right_topic = str(values["POSESIM_RIGHT_CAMERA_POSE_TOPIC"])
        self._context_topic = str(values["POSESIM_EXPERIMENT_CONTEXT_TOPIC"])
        self._camera_mode = str(values["POSESIM_CAMERA_POSE_MODE"])
        self._expected_clock_domain = (
            GAZEBO_SIM_CLOCK
            if values["POSESIM_USE_SIM_TIME"]
            and values["POSESIM_EVALUATION_TIME_SOURCE"] == GAZEBO_SIM_CLOCK
            else None
        )
        self._gt_latest: GroundTruthPoseSample | None = None
        self._left_latest: CameraOpticalPoseSample | None = None
        self._right_latest: CameraOpticalPoseSample | None = None
        self._context_store = SimulationContextStore(
            expected_schema=str(values["POSESIM_EXPECTED_CONTEXT_SCHEMA"]),
            require_hash_match=bool(values["POSESIM_REQUIRE_CONTEXT_HASH_MATCH"]),
        )
        self._alignment = align_static_snapshot(
            None,
            None,
            None,
            expected_clock_domain=self._expected_clock_domain or GAZEBO_SIM_CLOCK,
        )
        self._gt_validator = GroundTruthStreamValidator(
            reject_duplicate_stamps=bool(values["POSESIM_REJECT_DUPLICATE_STAMPS"]),
            reject_out_of_order=bool(values["POSESIM_REJECT_OUT_OF_ORDER"]),
        )
        self._left_store = StaticCameraSnapshotStore("left")
        self._right_store = StaticCameraSnapshotStore("right")
        self._gt_buffer: deque[GroundTruthPoseSample] = deque(maxlen=256)
        self._left_buffer: deque[CameraOpticalPoseSample] = deque(maxlen=256)
        self._right_buffer: deque[CameraOpticalPoseSample] = deque(maxlen=256)
        self._discovered = {"gt": False, "left": False, "right": False}
        self._qos_issues: list[str] = []

        gt_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        camera_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._gt_subscription = self.create_subscription(
            PoseStamped, self._gt_topic, self._on_ground_truth, gt_qos
        )
        self._left_subscription = self.create_subscription(
            String, self._left_topic, self._on_left_camera, camera_qos
        )
        self._right_subscription = self.create_subscription(
            String, self._right_topic, self._on_right_camera, camera_qos
        )
        self._context_subscription = self.create_subscription(
            SimulationExperimentContext,
            self._context_topic,
            self._on_experiment_context,
            camera_qos,
        )
        self.get_logger().info(
            "PoseSimulation input probe started; subscriptions are read-only and no output publisher is created"
        )

    def _adapt_ground_truth(self, message: Any) -> GroundTruthPoseSample:
        sample = adapt_ground_truth_pose(
            message,
            source_topic=self._gt_topic,
            reference_name=str(self._values["POSESIM_GT_REFERENCE_NAME"]),
            expected_frame=str(self._values["POSESIM_EXPECTED_WORLD_FRAME"]),
            receive_timestamp_ns=time.monotonic_ns(),
            clock_domain=self._expected_clock_domain,
        )
        return self._gt_validator.accept(sample)

    def _adapt_camera(self, message: Any, side: str, topic: str) -> CameraOpticalPoseSample:
        expected_name_key = (
            "POSESIM_LEFT_CAMERA_ENTITY_NAME"
            if side == "left"
            else "POSESIM_RIGHT_CAMERA_ENTITY_NAME"
        )
        return adapt_camera_pose_diagnostic(
            getattr(message, "data", ""),
            expected_side=side,
            source_topic=topic,
            expected_schema=str(self._values["POSESIM_CAMERA_POSE_SCHEMA"]),
            source_mode=self._camera_mode,
            expected_entity_name=str(self._values[expected_name_key]),
            receive_timestamp_ns=time.monotonic_ns(),
            clock_domain=self._expected_clock_domain,
        )

    def _on_ground_truth(self, message: Any) -> None:
        self._gt_latest = self._adapt_ground_truth(message)
        if self._gt_latest.valid and self._gt_latest.stamp_ns is not None:
            self._gt_buffer.append(self._gt_latest)
        self._update_alignment()

    def _on_left_camera(self, message: Any) -> None:
        sample = self._adapt_camera(message, "left", self._left_topic)
        self._left_latest = self._left_store.accept(sample) if self._camera_mode == STATIC_SNAPSHOT_MODE else sample
        if self._left_latest.valid and self._camera_mode == DYNAMIC_STREAM_MODE:
            self._left_buffer.append(self._left_latest)
        self._update_alignment()

    def _on_right_camera(self, message: Any) -> None:
        sample = self._adapt_camera(message, "right", self._right_topic)
        self._right_latest = self._right_store.accept(sample) if self._camera_mode == STATIC_SNAPSHOT_MODE else sample
        if self._right_latest.valid and self._camera_mode == DYNAMIC_STREAM_MODE:
            self._right_buffer.append(self._right_latest)
        self._update_alignment()

    def _on_experiment_context(self, message: Any) -> None:
        self._context_store.accept(message)

    def _update_alignment(self) -> None:
        if self._camera_mode == STATIC_SNAPSHOT_MODE:
            self._alignment = align_static_snapshot(
                self._gt_latest,
                self._left_latest,
                self._right_latest,
                expected_clock_domain=self._expected_clock_domain or GAZEBO_SIM_CLOCK,
            )
            return
        self._alignment = align_dynamic_streams(
            list(self._gt_buffer)
            if self._gt_latest is None or self._gt_latest.valid
            else [self._gt_latest],
            list(self._left_buffer)
            if self._left_latest is None or self._left_latest.valid
            else [self._left_latest],
            list(self._right_buffer)
            if self._right_latest is None or self._right_latest.valid
            else [self._right_latest],
            expected_clock_domain=self._expected_clock_domain or GAZEBO_SIM_CLOCK,
            max_interpolation_gap_ns=int(
                float(self._values["POSESIM_MAX_INTERPOLATION_GAP_MS"]) * 1_000_000.0
            ),
            allow_future_extrapolation=bool(self._values["POSESIM_ALLOW_FUTURE_EXTRAPOLATION"]),
        )

    def _refresh_topic_state(
        self,
        key: str,
        topic: str,
        *,
        expected_reliability: str,
        expected_durability: str,
    ) -> None:
        infos = self.get_publishers_info_by_topic(topic)
        self._discovered[key] = bool(infos)
        if not infos:
            return
        if not any(
            publisher_qos_compatible(
                info,
                expected_reliability=expected_reliability,
                expected_durability=expected_durability,
            )
            for info in infos
        ):
            issue = f"{key} topic publishers do not match {expected_reliability}/{expected_durability} QoS"
            if issue not in self._qos_issues:
                self._qos_issues.append(issue)

    def refresh_topic_state(self) -> None:
        """Refresh discovery/QoS metadata without changing subscriptions."""

        self._refresh_topic_state(
            "gt",
            self._gt_topic,
            expected_reliability="RELIABLE",
            expected_durability="VOLATILE",
        )
        for key, topic in (("left", self._left_topic), ("right", self._right_topic)):
            self._refresh_topic_state(
                key,
                topic,
                expected_reliability="RELIABLE",
                expected_durability="TRANSIENT_LOCAL",
            )
        self._refresh_topic_state(
            "context",
            self._context_topic,
            expected_reliability="RELIABLE",
            expected_durability="TRANSIENT_LOCAL",
        )

    def snapshot(self) -> ProbeSnapshot:
        self.refresh_topic_state()
        gt_max_age_ms = float(self._values["POSESIM_MAX_GT_SOURCE_AGE_MS"])
        gt_receive_age_ms = None
        alignment = self._alignment
        if self._gt_latest is not None and self._gt_latest.receive_timestamp_ns is not None:
            gt_receive_age_ms = max(
                0.0,
                (time.monotonic_ns() - self._gt_latest.receive_timestamp_ns) / 1_000_000.0,
            )
            if self._gt_latest.valid and gt_receive_age_ms > gt_max_age_ms:
                alignment = replace(
                    alignment,
                    valid=False,
                    status=GT_SOURCE_STALE,
                    invalid_reason=(
                        f"GT receive age {gt_receive_age_ms:.3f} ms exceeds "
                        f"configured limit {gt_max_age_ms:.3f} ms"
                    ),
                )
        return ProbeSnapshot(
            gt_topic=self._gt_topic,
            left_camera_topic=self._left_topic,
            right_camera_topic=self._right_topic,
            camera_source_mode=self._camera_mode,
            expected_clock_domain=self._expected_clock_domain,
            gt_topic_discovered=self._discovered["gt"],
            left_topic_discovered=self._discovered["left"],
            right_topic_discovered=self._discovered["right"],
            ground_truth=self._gt_latest,
            left_camera=self._left_latest,
            right_camera=self._right_latest,
            alignment=alignment,
            qos_issues=tuple(self._qos_issues),
            gt_receive_age_ms=gt_receive_age_ms,
            gt_max_source_age_ms=gt_max_age_ms,
            context_topic=self._context_topic,
            context_topic_discovered=self._discovered.get("context", False),
            context_valid=self._context_store.context is not None,
            context_status=self._context_store.status,
            context_reason=self._context_store.reason,
        )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Probe PoseSimulation external ROS 2 inputs")
    parser.add_argument("--config", help="PoseSimulation base env file")
    parser.add_argument("--profile", help="optional PoseSimulation profile env file")
    parser.add_argument("--set", dest="overrides", action="append", default=[], metavar="POSESIM_KEY=VALUE")
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--once", action="store_true", help="report after one spin cycle")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.timeout_s < 0.0:
        print("PoseSimulation input probe error: --timeout-s cannot be negative", file=sys.stderr)
        return 2
    try:
        config = resolve_config(
            config_path=args.config,
            profile_path=args.profile,
            cli_overrides=_parse_cli_overrides(args.overrides),
        )
    except ConfigError as exc:
        print(f"PoseSimulation configuration error: {exc}", file=sys.stderr)
        return 2
    if rclpy is None:
        print("PoseSimulation input probe: SOURCE_UNAVAILABLE (ROS 2 runtime is not installed)")
        return 2

    try:
        rclpy.init(args=None)
    except Exception as exc:  # pragma: no cover - host/runtime dependent
        print(
            "PoseSimulation input probe: SOURCE_UNAVAILABLE "
            f"(ROS 2 initialization failed: {exc})"
        )
        return 2
    node: InputProbeNode | None = None
    try:
        node = InputProbeNode(config)
        deadline = time.monotonic() + args.timeout_s
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            snapshot = node.snapshot()
            if args.once or snapshot.alignment.valid:
                break
            if time.monotonic() >= deadline:
                break
        final_snapshot = node.snapshot()
        print(format_probe_report(final_snapshot))
        return 0 if _overall_status(final_snapshot) == VALID else 1
    except KeyboardInterrupt:
        print("PoseSimulation input probe interrupted", file=sys.stderr)
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = [
    "InputProbeNode",
    "ProbeSnapshot",
    "format_probe_report",
    "main",
    "publisher_qos_compatible",
]


if __name__ == "__main__":  # pragma: no cover - exercised by the runner
    raise SystemExit(main())
