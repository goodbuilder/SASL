"""Operation configuration, correction routes and readiness ownership.

GPS input retains WGS84 degrees/heights; readiness consumes fix/age/accuracy
only. Baseline quality is metres with producer NED axes unchanged. No Qt/UI.
"""

import time
from dataclasses import dataclass, field

from Workstation.workstation_core.gnss_operation import (
    CorrectionSourceIdentity,
    GgaPolicy,
    GnssOperationConfig,
    GnssOperationManager,
    GnssOperationMode,
    GnssOperationState,
    GnssReadinessInputs,
    GnssReadinessSnapshot,
    HeightType,
    RoverCorrectionStatus,
    RoverReadinessInput,
    evaluate_gnss_readiness,
)
from Workstation.workstation_core.rtcm_pipeline import RtcmRouteConfig


@dataclass
class GnssOperationRuntimeState:
    gnss_operation_manager: object = field(default_factory=lambda: GnssOperationManager(GnssOperationConfig()))
    gnss_active_operation_config: object = None
    gnss_operation_gate_active: bool = False
    gnss_rover_fixed_since: dict = field(default_factory=lambda: {"left": None, "right": None, "drone": None})


OPERATION_FIELDS = tuple(GnssOperationRuntimeState.__dataclass_fields__)


@dataclass(frozen=True)
class NetworkAdmissionInputs:
    """One Start request's configuration and Base receipt-time evidence.

    Coordinate values never cross this boundary. HeightType names the required
    WGS84 ellipsoid datum; Base freshness is host monotonic seconds, not UTC.
    """

    mode: GnssOperationMode
    caster_host: str
    mountpoint: str
    gga_policy: GgaPolicy
    fixed_height_type: HeightType
    base_serial_connected: bool
    base_position_present: bool
    base_position_monotonic_s: float | None
    now_monotonic_s: float


@dataclass(frozen=True)
class NetworkAdmissionDecision:
    accepted: bool
    message: str = ""
    tone: str = "bad"


@dataclass(frozen=True)
class ReadinessObservation:
    configured: object
    target_sides: tuple
    injection_stats: dict
    mavlink: object
    gps: dict
    baseline_vector: object
    baseline_quality: object


class GnssOperationRuntime:
    def __init__(self, state, *, transitioned, health, evaluate=evaluate_gnss_readiness):
        self.state = state
        self.transitioned = transitioned
        self.health = health
        self.evaluate = evaluate
        if any(not callable(callback) for callback in (transitioned, health, evaluate)):
            raise TypeError("GNSS operation requires transition, health and readiness adapters")

    def transition(self, state, reason):
        try:
            self.state.gnss_operation_manager.transition(state, reason)
        except (RuntimeError, ValueError) as exc:
            self.transitioned(state, reason, str(exc))
            return False
        self.transitioned(self.state.gnss_operation_manager.state, reason, None)
        return True

    def configure(self, config):
        manager = self.state.gnss_operation_manager
        if manager.state not in {
            GnssOperationState.DISCONNECTED,
            GnssOperationState.STOPPED,
            GnssOperationState.CONFIGURED,
        }:
            manager.stop()
        manager.configure(config)

    @staticmethod
    def assess_network_start(request: NetworkAdmissionInputs) -> NetworkAdmissionDecision:
        """Preserve mode/caster/datum/freshness guard order and exact policy."""

        if request.mode == GnssOperationMode.LOCAL_SURVEY_IN:
            return NetworkAdmissionDecision(False, "현장 모드에서는 NTRIP을 시작할 수 없습니다", "warn")
        if not request.caster_host:
            return NetworkAdmissionDecision(False, "caster host 필요")
        if not request.mountpoint:
            return NetworkAdmissionDecision(False, "mountpoint 필요")
        if request.mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE:
            if not request.base_serial_connected:
                return NetworkAdmissionDecision(False, "보조 Fixed Base 모드는 먼저 Base serial을 연결하세요")
            if request.gga_policy != GgaPolicy.BASE_GNSS:
                return NetworkAdmissionDecision(False, "보조 Fixed Base는 Base GNSS GGA가 필요합니다")
            if request.fixed_height_type != HeightType.ELLIPSOID:
                return NetworkAdmissionDecision(False, "절대 WGS84 운용은 Fixed Base 높이를 ellipsoid로 설정하세요")
            # Future-dated receipt timestamps retained the same clamp-to-zero
            # behavior. The old strict >15 comparison admits exactly 15 s.
            if (
                not request.base_position_present
                or max(0.0, request.now_monotonic_s - request.base_position_monotonic_s) > 15.0
            ):
                return NetworkAdmissionDecision(False, "Base WGS84 위치 샘플 대기 중입니다", "warn")
        return NetworkAdmissionDecision(True)

    def begin_network(self, config) -> bool:
        """Apply the accepted immutable config before correction-route setup."""

        self.configure(config)
        self.state.gnss_operation_manager.set_source_identity(self.source_identity(config))
        if not self.transition(GnssOperationState.INITIALIZING, "NTRIP 운용 초기화"):
            return False
        if not self.transition(GnssOperationState.NTRIP_CONNECTING, "NTRIP caster 연결 중"):
            return False
        self.state.gnss_operation_gate_active = True
        self.state.gnss_active_operation_config = config
        return True

    @staticmethod
    def target_sides(
        config, *, selected_target, source, include_rover_targets, drone_source, drone_forward, drone_connected
    ):
        if not include_rover_targets:
            targets = ()
        elif config is not None:
            targets = tuple(
                side
                for side, enabled in (("left", config.routing.left_enabled), ("right", config.routing.right_enabled))
                if enabled
            )
        else:
            targets = (
                ("left", "right")
                if selected_target == "both"
                else (selected_target,)
                if selected_target in {"left", "right"}
                else ()
            )
        drone = (
            ("drone",)
            if (
                config is not None
                and config.routing.drone_enabled
                and source == drone_source
                and drone_forward
                and drone_connected
            )
            else ()
        )
        return tuple(dict.fromkeys((*targets, *drone)))

    @staticmethod
    def route(config, *, ntrip_targets, base_targets, base_serial_available):
        return RtcmRouteConfig(
            ntrip_targets=ntrip_targets,
            base_targets=base_targets,
            forward_ntrip_to_base=getattr(config, "operation_mode", None)
            == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE,
            base_serial_available=base_serial_available,
        )

    def source_identity(self, config: GnssOperationConfig) -> CorrectionSourceIdentity:
        if config.operation_mode == GnssOperationMode.LOCAL_SURVEY_IN:
            return CorrectionSourceIdentity(
                source_kind="local-serial-survey-in",
                base_device=config.local_base.device,
                site_label=config.site_label,
                site_address=config.site_address,
            )
        return CorrectionSourceIdentity(
            source_kind=(
                "ntrip-fixed-base"
                if config.operation_mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE
                else "ntrip-direct"
            ),
            caster_host=config.network.caster_host,
            caster_port=config.network.caster_port,
            mountpoint=config.network.mountpoint,
            gga_policy=config.network.gga_policy,
            base_device=config.local_base.device
            if config.operation_mode == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE
            else "",
            site_label=config.site_label,
            site_address=config.site_address,
        )

    def refresh(self, observation: ReadinessObservation):
        if not self.state.gnss_operation_gate_active:
            return
        config = self.state.gnss_operation_manager.config
        if (
            self.state.gnss_active_operation_config is not None
            and self.state.gnss_operation_manager.state in {GnssOperationState.READY, GnssOperationState.MEASURING}
            and observation.configured != config
        ):
            self.state.gnss_operation_manager.invalidate("GNSS controls changed during active operation")
            return
        target_sides = observation.target_sides
        now_s = time.monotonic()
        injection = observation.injection_stats.get("injection")
        injection_map = injection if isinstance(injection, dict) else {}
        mavlink = observation.mavlink
        connections = getattr(mavlink, "connections", None)
        heartbeat_sides = getattr(mavlink, "_heartbeat_logged", set())
        health_reader = getattr(mavlink, "mavlink_health_snapshot", None)
        routes: dict[str, RoverCorrectionStatus] = {}
        rover_samples: dict[str, RoverReadinessInput] = {}
        for side in target_sides:
            side_stats = injection_map.get(side)
            if not isinstance(side_stats, dict):
                side_stats = {}
            health = None
            if callable(health_reader):
                try:
                    health = health_reader(side, now_s=now_s)
                except Exception:  # noqa: BLE001 - unavailable health retains legacy route fallback
                    health = None
            if health is not None:
                endpoint_live = bool(getattr(health, "identity_verified", False)) and bool(
                    getattr(health, "rtcm_eligible", False)
                )
                endpoint_identity = (
                    f"mavlink:{side}:sysid={getattr(health, 'observed_system_id', '')}:"
                    f"generation={getattr(health, 'connection_generation', 0)}"
                    if endpoint_live
                    else ""
                )
                health_state = str(
                    getattr(
                        getattr(health, "state", None),
                        "value",
                        getattr(health, "state", ""),
                    )
                )
                health_fault = (
                    ""
                    if endpoint_live
                    else f"MAVLink {health_state or 'ineligible'}: "
                    f"{getattr(health, 'stall_reason', '') or 'RTCM blocked'}"
                )
            elif isinstance(connections, dict):
                endpoint_live = side in connections and side in heartbeat_sides
                endpoint_identity = f"mavlink:{side}" if endpoint_live else ""
                health_fault = ""
            else:
                # Test adapters may not expose the production subscriber
                # internals; their presence still represents a route endpoint.
                endpoint_identity = "" if mavlink is None else f"mavlink:{side}"
                health_fault = ""
            routes[side] = RoverCorrectionStatus(
                target=side,
                enabled=True,
                endpoint_identity=endpoint_identity,
                bytes_sent=int(side_stats.get("bytes") or 0),
                packets_sent=int(side_stats.get("packets") or 0),
                last_delivery_monotonic_s=side_stats.get("last_delivery_s"),
                fault_reason=(
                    health_fault
                    or (
                        "RTCM blocked: MAVLink stalled"
                        if int(side_stats.get("blocked_stall") or 0) > 0
                        else "injection failed"
                        if int(side_stats.get("fail") or 0) > 0
                        else ""
                    )
                ),
            )
        for side in ("left", "right", "drone"):
            gps = observation.gps.get(side)
            if gps is None:
                self.state.gnss_rover_fixed_since[side] = None
                continue
            if gps.fix_type == 6:
                if self.state.gnss_rover_fixed_since.get(side) is None:
                    self.state.gnss_rover_fixed_since[side] = now_s
            else:
                self.state.gnss_rover_fixed_since[side] = None
            rover_samples[side] = RoverReadinessInput(
                target=side,
                fix_type=gps.fix_type,
                gps_sample_monotonic_s=gps.timestamp_s,
                eph_m=gps.eph_m,
                epv_m=getattr(gps, "epv_m", None),
                rtk_fixed_since_monotonic_s=self.state.gnss_rover_fixed_since.get(side),
                device_id=str(getattr(gps, "topic", side)),
            )
        last_frame_s = observation.injection_stats.get("last_frame_s")
        baseline_vector = observation.baseline_vector
        baseline_quality = observation.baseline_quality
        source = self.state.gnss_operation_manager.source_identity
        candidate = self.evaluate(
            config,
            GnssReadinessInputs(
                # Evaluate the policy as a candidate; the transition state is
                # applied below only after every enabled criterion passes.
                state=GnssOperationState.READY,
                now_monotonic_s=now_s,
                correction_received_monotonic_s=last_frame_s,
                correction_source_id="" if source is None else source.stable_id,
                routes=routes,
                rover_samples=rover_samples,
                left_right_timestamp_delta_ms=(None if baseline_vector is None else baseline_vector.timestamp_delta_ms),
                baseline_std_m=(None if baseline_quality is None else baseline_quality.baseline_length_std_m),
            ),
        )
        reasons = candidate.reasons
        # A disabled readiness gate remains observable in the status snapshot,
        # but must not leave the state machine permanently below READY.  The
        # measurement manager records the warning-only, no-contract path.
        transition_reasons = reasons if config.readiness_gate_enabled else ()
        state = self.state.gnss_operation_manager.state
        # LOCAL_RTCM_ACTIVE is Group B's (STATE_STATUS_GROUP == BASE_FIXED)
        # terminal state -- reaching it means the Base has finished its own
        # convergence/fixed-coordinate sequence and rover convergence can
        # begin.  NTRIP_ACTIVE is Group A (STATE_STATUS_GROUP == NETWORK); it
        # only implies convergence-eligibility in NETWORK_RTK_DIRECT, where
        # there is no Group B stage at all (RTCM goes straight to rovers). In
        # NETWORK_ASSISTED_FIXED_BASE, NTRIP_ACTIVE just means the caster feed
        # is reaching the Base serial port; the Base must still complete its
        # own Fixed-coordinate sequence (up to LOCAL_RTCM_ACTIVE) before rover
        # convergence readiness has any meaning.
        convergence_eligible = state == GnssOperationState.LOCAL_RTCM_ACTIVE or (
            state == GnssOperationState.NTRIP_ACTIVE and config.operation_mode == GnssOperationMode.NETWORK_RTK_DIRECT
        )
        if convergence_eligible and not transition_reasons:
            self.transition(GnssOperationState.ROVERS_CONVERGING, "RTCM rover 주입 시작")
            state = self.state.gnss_operation_manager.state
        if state == GnssOperationState.ROVERS_CONVERGING and not transition_reasons:
            self.transition(GnssOperationState.ALIGNMENT_COLLECTING, "양쪽 rover FIXED 수신")
            state = self.state.gnss_operation_manager.state
        if state == GnssOperationState.ALIGNMENT_COLLECTING and not transition_reasons:
            self.transition(GnssOperationState.ALIGNMENT_VALID, "GNSS rover 정합 조건 통과")
            state = self.state.gnss_operation_manager.state
        if state == GnssOperationState.ALIGNMENT_VALID and not transition_reasons:
            self.transition(GnssOperationState.READY, "GNSS measurement readiness 통과")
            state = self.state.gnss_operation_manager.state
        if state == GnssOperationState.READY and transition_reasons:
            self.transition(GnssOperationState.DEGRADED, "; ".join(reasons))
            state = self.state.gnss_operation_manager.state
        if state == GnssOperationState.DEGRADED and not transition_reasons:
            self.transition(GnssOperationState.READY, "GNSS 조건 복구")
            state = self.state.gnss_operation_manager.state
        if state == GnssOperationState.MEASURING and transition_reasons:
            self.state.gnss_operation_manager.invalidate("GNSS readiness changed: " + ", ".join(reasons))
            state = self.state.gnss_operation_manager.state
        snapshot = GnssReadinessSnapshot(
            ready=not reasons,
            state=state,
            reasons=reasons,
            warnings=candidate.warnings,
            mode=config.operation_mode,
            correction_source_id="" if source is None else source.stable_id,
            timestamp_s=now_s,
            reason_messages=candidate.reason_messages,
        )
        self.state.gnss_operation_manager.update_readiness(snapshot)
        if reasons:
            self.health(
                "operation",
                f"{state.value} | " + "; ".join(snapshot.reason_messages.get(code, code) for code in reasons[:3]),
                "warn",
            )
        return snapshot
