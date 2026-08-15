from dataclasses import FrozenInstanceError, replace
import hashlib
import math
import struct

import pytest

from Workstation.workstation_core.mavlink_sources import (
    AttitudeYaw,
    CourseOverGround,
    GnssYaw,
    LoggerCommandReceipt,
    LoggerControlAction,
    LoggerDecisionReason,
    LoggerLifecyclePhase,
    LoggerLifecycleState,
    LoggerOwner,
    LoggerRuntimeIdentity,
    LoggerStatusEvidence,
    MavlinkFtpAccess,
    MavlinkMutationPolicy,
    MavlinkOperation,
    MavlinkPacketProvenance,
    MavlinkSourceConfig,
    MavlinkSourceId,
    MavlinkSourceRole,
    MavlinkYawObservations,
    ULOG_MAGIC,
    ULogInventoryChangeKind,
    ULogInventoryEntry,
    ULogValidationError,
    decide_logger_start,
    decide_logger_stop,
    diff_ulog_inventories,
    observe_logger_preflight,
    parse_logger_status,
    record_logger_start_result,
    record_logger_stop_result,
    session_ulog_candidates,
    validate_ulog_artifact,
)


SESSION_ID = "session-a"
START_COMMAND_ID = "logger-on-1"
STOP_COMMAND_ID = "logger-off-1"
LOG_PATH = "/fs/microsd/log/2026-08-13/workstation.ulg"


def _identity(
    *,
    source: MavlinkSourceId = MavlinkSourceId.LEFT,
    generation: int = 4,
    boot_id: str = "px4-boot-a",
    system_id: int = 1,
    component_id: int = 1,
) -> LoggerRuntimeIdentity:
    return LoggerRuntimeIdentity(
        source_id=source,
        connection_generation=generation,
        boot_id=boot_id,
        source_system=system_id,
        source_component=component_id,
    )


def _config(
    *,
    source: MavlinkSourceId = MavlinkSourceId.LEFT,
    system_id: int = 1,
    component_id: int = 1,
) -> MavlinkSourceConfig:
    return MavlinkSourceConfig.for_source(
        source,
        "udp:127.0.0.1:14550",
        expected_system_id=system_id,
        expected_component_id=component_id,
    )


def _provenance(
    source: MavlinkSourceId = MavlinkSourceId.TARGET_DRONE,
    *,
    message_id: int = 24,
) -> MavlinkPacketProvenance:
    return MavlinkPacketProvenance(
        source_id=source,
        source_system=42,
        source_component=1,
        sequence=7,
        message_id=message_id,
        host_received_monotonic_ns=1_000,
        host_received_utc_ns=2_000,
        source_timestamp_us=3_000,
    )


def _running(path: str = LOG_PATH) -> object:
    return parse_logger_status(
        "Running in mode: all\nFull File Logging Running:\n"
        f"Log file: {path}\n"
    )


def _evidence(
    identity: LoggerRuntimeIdentity,
    status: object,
    observed_ns: int,
    *,
    query_id: str,
) -> LoggerStatusEvidence:
    return LoggerStatusEvidence(identity, status, observed_ns, query_id)


def _receipt(
    action: LoggerControlAction,
    identity: LoggerRuntimeIdentity,
    command_id: str,
    *,
    acknowledged: bool = True,
    completed_ns: int,
) -> LoggerCommandReceipt:
    return LoggerCommandReceipt(
        action=action,
        runtime_identity=identity,
        session_id=SESSION_ID,
        command_id=command_id,
        acknowledged=acknowledged,
        completed_at_monotonic_ns=completed_ns,
    )


def _owned_logger() -> tuple[
    MavlinkSourceConfig,
    LoggerRuntimeIdentity,
    LoggerLifecycleState,
]:
    config = _config()
    identity = _identity()
    idle = observe_logger_preflight(identity, parse_logger_status("Not logging"))
    start = decide_logger_start(
        config,
        idle,
        session_id=SESSION_ID,
        command_id=START_COMMAND_ID,
    )
    owned = record_logger_start_result(
        idle,
        start,
        _evidence(identity, _running(), 120, query_id="post-start-status"),
        receipt=_receipt(
            LoggerControlAction.START,
            identity,
            START_COMMAND_ID,
            completed_ns=110,
        ),
    )
    return config, identity, owned


def _ulog(
    path: str,
    size: int,
    *,
    identity: LoggerRuntimeIdentity | None = None,
    session_id: str = SESSION_ID,
) -> ULogInventoryEntry:
    return ULogInventoryEntry(
        runtime_identity=_identity() if identity is None else identity,
        session_id=session_id,
        remote_path=path,
        size_bytes=size,
    )


def _valid_ulog(*extra_records: bytes) -> bytes:
    file_header = ULOG_MAGIC + bytes((1,)) + struct.pack("<Q", 123_456)
    flag_bits_payload = bytes(40)
    flag_bits = struct.pack("<HB", len(flag_bits_payload), ord("B")) + flag_bits_payload
    return file_header + flag_bits + b"".join(extra_records)


def test_source_roles_and_target_default_are_safe_and_immutable():
    left = _config()
    target = MavlinkSourceConfig.target_drone(
        "udp:127.0.0.1:14570",
        expected_system_id=42,
        expected_component_id=1,
    )

    assert left.role == MavlinkSourceRole.RIG_P1
    assert left.policy.allows(MavlinkOperation.STREAM_SETUP)
    assert target.role == MavlinkSourceRole.TARGET_REFERENCE
    assert target.policy == MavlinkMutationPolicy.target_read_only()
    assert target.policy.ftp_access == MavlinkFtpAccess.READ_ONLY
    assert target.policy.ftp_is_read_only
    assert target.policy.allows(MavlinkOperation.FTP_READ)
    for forbidden in (
        MavlinkOperation.STREAM_SETUP,
        MavlinkOperation.PARAMETER_WRITE,
        MavlinkOperation.RTCM_INJECTION,
        MavlinkOperation.LOGGER_CONTROL,
        MavlinkOperation.FTP_WRITE,
    ):
        assert not target.policy.allows(forbidden)
    with pytest.raises(FrozenInstanceError):
        target.endpoint = "udp:127.0.0.1:14571"


def test_source_role_and_strict_policy_contracts_fail_closed():
    with pytest.raises(ValueError, match="cannot use role"):
        MavlinkSourceConfig(
            source_id=MavlinkSourceId.LEFT,
            role=MavlinkSourceRole.TARGET_REFERENCE,
            endpoint="udp:127.0.0.1:14550",
            baudrate=57_600,
            expected_system_id=1,
            expected_component_id=1,
            policy=MavlinkMutationPolicy.rig_default(),
        )
    with pytest.raises(ValueError, match="read-only target"):
        MavlinkSourceConfig.for_source(
            MavlinkSourceId.TARGET_DRONE,
            "udp:127.0.0.1:14570",
            policy=MavlinkMutationPolicy.rig_default(),
        )
    with pytest.raises(TypeError, match="allow_stream_setup must be bool"):
        MavlinkMutationPolicy("false", False, False, False, MavlinkFtpAccess.DISABLED)


def test_strict_source_and_logger_identity_values_do_not_lossily_coerce():
    with pytest.raises(TypeError, match="expected_system_id must be int"):
        MavlinkSourceConfig.left(
            "udp:127.0.0.1:14550",
            expected_system_id=1.0,
            expected_component_id=1,
        )
    with pytest.raises(TypeError, match="boot_id must be str"):
        LoggerRuntimeIdentity(MavlinkSourceId.LEFT, 1, None, 1, 1)
    with pytest.raises(TypeError, match="session_id must be str"):
        decide_logger_start(
            _config(),
            observe_logger_preflight(_identity(), parse_logger_status("Not logging")),
            session_id=None,
            command_id=START_COMMAND_ID,
        )


@pytest.mark.parametrize("field", ("source_system", "source_component"))
def test_packet_provenance_rejects_broadcast_source_identity(field):
    values = dict(
        source_id=MavlinkSourceId.LEFT,
        source_system=1,
        source_component=1,
        sequence=0,
        message_id=24,
        host_received_monotonic_ns=1,
    )
    values[field] = 0
    with pytest.raises(ValueError):
        MavlinkPacketProvenance(**values)


def test_cog_gnss_yaw_and_attitude_yaw_are_distinct_and_protocol_exact():
    gps_packet = _provenance(message_id=24)
    attitude_packet = _provenance(message_id=30)
    observations = MavlinkYawObservations(
        course_over_ground=CourseOverGround.from_mavlink_cdeg(12_345, gps_packet),
        gnss_yaw=GnssYaw.from_mavlink_cdeg(36_000, gps_packet),
        attitude_yaw=AttitudeYaw.from_mavlink_rad(math.pi / 2.0, attitude_packet),
    )

    assert observations.course_over_ground.degrees_true_north == pytest.approx(123.45)
    assert observations.gnss_yaw.degrees_true_north == pytest.approx(0.0)
    assert observations.attitude_yaw.degrees_ned == pytest.approx(90.0)
    assert observations.attitude_yaw.provenance.message_id == 30
    assert CourseOverGround.from_mavlink_cdeg(65_535, gps_packet) is None
    assert GnssYaw.from_mavlink_cdeg(0, gps_packet) is None
    assert GnssYaw.from_mavlink_cdeg(65_535, gps_packet) is None
    with pytest.raises(ValueError, match=r"\[-pi, pi\]"):
        AttitudeYaw.from_mavlink_rad(3.0 * math.pi, attitude_packet)
    with pytest.raises(TypeError, match="value_cdeg must be int"):
        CourseOverGround.from_mavlink_cdeg(123.9, gps_packet)


def test_yaw_observations_reject_swapped_types_and_mixed_sources():
    left = _provenance(MavlinkSourceId.LEFT)
    right = _provenance(MavlinkSourceId.RIGHT)
    with pytest.raises(TypeError, match="course_over_ground must be CourseOverGround"):
        MavlinkYawObservations(course_over_ground=GnssYaw(10.0, left))
    with pytest.raises(ValueError, match="different MAVLink identities"):
        MavlinkYawObservations(
            course_over_ground=CourseOverGround(10.0, left),
            gnss_yaw=GnssYaw(11.0, right),
        )
    with pytest.raises(ValueError, match="not an ATTITUDE message"):
        AttitudeYaw.from_mavlink_rad(0.0, left)


def test_preexisting_file_logger_is_external_and_never_stopped():
    config = _config()
    identity = _identity()
    state = observe_logger_preflight(identity, _running())

    assert state.owner == LoggerOwner.EXTERNAL_ALREADY_RUNNING
    assert decide_logger_start(
        config,
        state,
        session_id=SESSION_ID,
        command_id=START_COMMAND_ID,
    ).action == LoggerControlAction.NONE
    stop = decide_logger_stop(
        config,
        state,
        current_runtime_identity=identity,
        session_id=SESSION_ID,
        fresh_status=_evidence(identity, _running(), 200, query_id="pre-stop-status"),
        command_id=STOP_COMMAND_ID,
    )
    assert stop.action == LoggerControlAction.NONE
    assert stop.reason == LoggerDecisionReason.NOT_WORKSTATION_OWNED


def test_only_same_boot_generation_session_path_and_fresh_status_can_stop():
    config, identity, owned = _owned_logger()
    same_fresh = _evidence(identity, _running(), 200, query_id="pre-stop-status")
    for wrong_identity in (_identity(generation=5), _identity(boot_id="px4-boot-b")):
        denied = decide_logger_stop(
            config,
            owned,
            current_runtime_identity=wrong_identity,
            session_id=SESSION_ID,
            fresh_status=_evidence(wrong_identity, _running(), 200, query_id="wrong-runtime"),
            command_id=STOP_COMMAND_ID,
        )
        assert denied.reason == LoggerDecisionReason.RUNTIME_IDENTITY_MISMATCH
    wrong_session = decide_logger_stop(
        config,
        owned,
        current_runtime_identity=identity,
        session_id="session-b",
        fresh_status=same_fresh,
        command_id=STOP_COMMAND_ID,
    )
    assert wrong_session.reason == LoggerDecisionReason.SESSION_MISMATCH
    stale = decide_logger_stop(
        config,
        owned,
        current_runtime_identity=identity,
        session_id=SESSION_ID,
        fresh_status=_evidence(identity, _running(), 120, query_id="stale-status"),
        command_id=STOP_COMMAND_ID,
    )
    assert stale.reason == LoggerDecisionReason.FRESH_STATUS_REQUIRED
    replaced_path = decide_logger_stop(
        config,
        owned,
        current_runtime_identity=identity,
        session_id=SESSION_ID,
        fresh_status=_evidence(
            identity,
            _running("/fs/microsd/log/external-replacement.ulg"),
            201,
            query_id="replacement-status",
        ),
        command_id=STOP_COMMAND_ID,
    )
    assert replaced_path.reason == LoggerDecisionReason.REMOTE_LOG_PATH_MISMATCH

    allowed = decide_logger_stop(
        config,
        owned,
        current_runtime_identity=identity,
        session_id=SESSION_ID,
        fresh_status=same_fresh,
        command_id=STOP_COMMAND_ID,
    )
    assert allowed.action == LoggerControlAction.STOP


def test_logger_authorization_is_bound_to_expected_ids_session_and_command():
    identity = _identity(system_id=42)
    idle = observe_logger_preflight(identity, parse_logger_status("Not logging"))
    denied = decide_logger_start(
        _config(system_id=1),
        idle,
        session_id=SESSION_ID,
        command_id=START_COMMAND_ID,
    )
    assert denied.reason == LoggerDecisionReason.RUNTIME_IDENTITY_MISMATCH
    unbound = decide_logger_start(
        MavlinkSourceConfig.left("udp:127.0.0.1:14550"),
        observe_logger_preflight(_identity(), parse_logger_status("Not logging")),
        session_id=SESSION_ID,
        command_id=START_COMMAND_ID,
    )
    assert unbound.reason == LoggerDecisionReason.EXPECTED_IDENTITY_REQUIRED


def test_start_receipt_must_match_decision_and_follow_command_completion():
    config = _config()
    identity = _identity()
    idle = observe_logger_preflight(identity, parse_logger_status("Not logging"))
    start = decide_logger_start(
        config,
        idle,
        session_id=SESSION_ID,
        command_id=START_COMMAND_ID,
    )
    wrong_receipt = _receipt(
        LoggerControlAction.START,
        identity,
        "another-command",
        completed_ns=110,
    )
    with pytest.raises(ValueError, match="receipt does not match"):
        record_logger_start_result(
            idle,
            start,
            _evidence(identity, _running(), 120, query_id="post-start-status"),
            receipt=wrong_receipt,
        )
    with pytest.raises(ValueError, match="observed after command"):
        record_logger_start_result(
            idle,
            start,
            _evidence(identity, _running(), 100, query_id="too-early-status"),
            receipt=_receipt(
                LoggerControlAction.START,
                identity,
                START_COMMAND_ID,
                completed_ns=110,
            ),
        )


def test_unacknowledged_start_never_claims_an_external_race_logger():
    identity = _identity()
    idle = observe_logger_preflight(identity, parse_logger_status("Not logging"))
    start = decide_logger_start(
        _config(),
        idle,
        session_id=SESSION_ID,
        command_id=START_COMMAND_ID,
    )
    unverified = record_logger_start_result(
        idle,
        start,
        _evidence(
            identity,
            _running("/fs/microsd/log/external-race.ulg"),
            120,
            query_id="race-status",
        ),
        receipt=_receipt(
            LoggerControlAction.START,
            identity,
            START_COMMAND_ID,
            acknowledged=False,
            completed_ns=110,
        ),
    )
    assert unverified.owner == LoggerOwner.UNKNOWN
    assert unverified.phase == LoggerLifecyclePhase.START_UNVERIFIED
    assert unverified.ownership_token is None


def test_target_and_unknown_status_never_issue_logger_commands():
    target = MavlinkSourceConfig.target_drone(
        "udp:127.0.0.1:14570",
        expected_system_id=42,
        expected_component_id=1,
    )
    identity = _identity(source=MavlinkSourceId.TARGET_DRONE, system_id=42)
    state = observe_logger_preflight(identity, parse_logger_status("garbled serial response"))
    assert decide_logger_start(
        target,
        state,
        session_id=SESSION_ID,
        command_id=START_COMMAND_ID,
    ).reason == LoggerDecisionReason.LOGGER_CONTROL_DISABLED
    assert parse_logger_status("Running in mode: mavlink\nNot logging").kind.value == "NOT_LOGGING"


def test_lifecycle_rejects_inconsistent_phase_owner_and_status():
    identity = _identity()
    idle = parse_logger_status("Not logging")
    with pytest.raises(ValueError, match="inconsistent"):
        LoggerLifecycleState(
            runtime_identity=identity,
            preflight_status=idle,
            current_status=_running(),
            owner=LoggerOwner.NONE,
            phase=LoggerLifecyclePhase.PREFLIGHT_IDLE,
        )
    with pytest.raises(TypeError, match="connection_generation must be int"):
        _identity(generation=4.9)


def test_logger_off_remaining_active_consumes_token_and_is_not_retried():
    config, identity, owned = _owned_logger()
    decision = decide_logger_stop(
        config,
        owned,
        current_runtime_identity=identity,
        session_id=SESSION_ID,
        fresh_status=_evidence(identity, _running(), 200, query_id="pre-stop-status"),
        command_id=STOP_COMMAND_ID,
    )
    pending = record_logger_stop_result(
        owned,
        decision,
        _evidence(identity, _running(), 220, query_id="post-stop-status"),
        receipt=_receipt(
            LoggerControlAction.STOP,
            identity,
            STOP_COMMAND_ID,
            completed_ns=210,
        ),
    )
    assert pending.phase == LoggerLifecyclePhase.PENDING_ACTIVE
    assert pending.owner == LoggerOwner.EXTERNAL_ALREADY_RUNNING
    assert pending.ownership_token is None
    assert decide_logger_stop(
        config,
        pending,
        current_runtime_identity=identity,
        session_id=SESSION_ID,
        fresh_status=_evidence(identity, _running(), 230, query_id="retry-status"),
        command_id="unsafe-retry",
    ).action == LoggerControlAction.NONE


def test_inventory_diff_is_bound_to_source_boot_generation_and_session():
    identity = _identity()
    before = (
        _ulog("/fs/microsd/log/a.ulg", 100),
        _ulog("/fs/microsd/log/b.ulg", 200),
    )
    after = (
        _ulog("/fs/microsd/log/a.ulg", 150),
        _ulog("/fs/microsd/log/b.ulg", 200),
        _ulog("/fs/microsd/log/new.ulg", 25),
    )
    changes = diff_ulog_inventories(before, after)
    assert {change.kind for change in changes} == {
        ULogInventoryChangeKind.GROWN,
        ULogInventoryChangeKind.UNCHANGED,
        ULogInventoryChangeKind.NEW,
    }
    assert [entry.remote_path for entry in session_ulog_candidates(before, after)] == [
        "/fs/microsd/log/a.ulg",
        "/fs/microsd/log/new.ulg",
    ]

    rebooted_same_path = _ulog(
        "/fs/microsd/log/a.ulg",
        150,
        identity=replace(identity, boot_id="px4-boot-b"),
    )
    reboot_changes = diff_ulog_inventories(before[:1], (rebooted_same_path,))
    assert [change.kind for change in reboot_changes] == [
        ULogInventoryChangeKind.REMOVED,
        ULogInventoryChangeKind.NEW,
    ]
    other_session = _ulog("/fs/microsd/log/a.ulg", 150, session_id="session-b")
    assert [change.kind for change in diff_ulog_inventories(before[:1], (other_session,))] == [
        ULogInventoryChangeKind.REMOVED,
        ULogInventoryChangeKind.NEW,
    ]


def test_valid_ulog_checks_flag_bits_record_structure_size_and_sha256():
    extra = struct.pack("<HB", 3, ord("I")) + b"abc"
    payload = _valid_ulog(extra)
    entry = _ulog("/fs/microsd/log/valid.ulg", len(payload), identity=_identity(
        source=MavlinkSourceId.TARGET_DRONE,
        system_id=42,
    ))
    digest = hashlib.sha256(payload).hexdigest()
    validation = validate_ulog_artifact(entry, payload, expected_sha256=digest)

    assert validation.valid
    assert validation.source_id == MavlinkSourceId.TARGET_DRONE
    assert validation.header_valid
    assert validation.record_structure_valid
    assert validation.record_count == 2
    assert validation.version == 1
    assert validation.start_timestamp_us == 123_456
    assert validation.sha256 == digest


@pytest.mark.parametrize(
    ("payload", "expected_error"),
    (
        (ULOG_MAGIC + bytes((1,)) + struct.pack("<Q", 1), ULogValidationError.RECORD_HEADER_TRUNCATED),
        (
            ULOG_MAGIC
            + bytes((1,))
            + struct.pack("<Q", 1)
            + struct.pack("<HB", 40, ord("I"))
            + bytes(40),
            ULogValidationError.FIRST_RECORD_NOT_FLAG_BITS,
        ),
        (
            ULOG_MAGIC
            + bytes((1,))
            + struct.pack("<Q", 1)
            + struct.pack("<HB", 40, ord("B"))
            + bytes(10),
            ULogValidationError.RECORD_PAYLOAD_TRUNCATED,
        ),
        (
            ULOG_MAGIC
            + bytes((255,))
            + struct.pack("<Q", 1)
            + struct.pack("<HB", 40, ord("B"))
            + bytes(40),
            ULogValidationError.UNSUPPORTED_VERSION,
        ),
    ),
)
def test_ulog_rejects_header_only_wrong_first_record_truncation_and_version(
    payload,
    expected_error,
):
    entry = _ulog("/fs/microsd/log/bad.ulg", len(payload))
    validation = validate_ulog_artifact(entry, payload)
    assert not validation.valid
    assert expected_error in validation.errors


def test_ulog_reports_size_magic_and_digest_failures():
    payload = b"not-ulog-header"
    entry = _ulog("/fs/microsd/log/bad.ulg", len(payload) + 10)
    validation = validate_ulog_artifact(entry, payload, expected_sha256="0" * 64)
    assert set(validation.errors) == {
        ULogValidationError.SIZE_MISMATCH,
        ULogValidationError.HEADER_TOO_SHORT,
        ULogValidationError.SHA256_MISMATCH,
    }


def test_ulog_rejects_unknown_incompatibility_flags_and_bad_append_offsets():
    header = ULOG_MAGIC + bytes((1,)) + struct.pack("<Q", 1)
    unknown_flag_payload = bytes(8) + bytes((2,)) + bytes(31)
    unknown_flag = header + struct.pack("<HB", 40, ord("B")) + unknown_flag_payload
    validation = validate_ulog_artifact(
        _ulog("/fs/microsd/log/unknown-flag.ulg", len(unknown_flag)),
        unknown_flag,
    )
    assert ULogValidationError.UNKNOWN_INCOMPATIBILITY_FLAG in validation.errors

    bad_offset_payload = (
        bytes(8)
        + bytes((1,))
        + bytes(7)
        + struct.pack("<QQQ", 10_000, 0, 0)
    )
    bad_offset = header + struct.pack("<HB", 40, ord("B")) + bad_offset_payload
    validation = validate_ulog_artifact(
        _ulog("/fs/microsd/log/bad-offset.ulg", len(bad_offset)),
        bad_offset,
    )
    assert ULogValidationError.APPENDED_OFFSET_INVALID in validation.errors

    second_flag = _valid_ulog(struct.pack("<HB", 40, ord("B")) + bytes(40))
    validation = validate_ulog_artifact(
        _ulog("/fs/microsd/log/second-flag.ulg", len(second_flag)),
        second_flag,
    )
    assert ULogValidationError.FLAG_BITS_NOT_FIRST in validation.errors
