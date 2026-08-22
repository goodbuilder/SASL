from __future__ import annotations

import hashlib
import json
import struct
from pathlib import Path

import pytest

from Workstation.ulog_controller import (
    AutomaticULogBinding,
    ManualOwnershipClaim,
    ULOG_COORDINATE_CONTRACT,
    ULogArtifactValidationError,
    ULogController,
    ULogControllerError,
)
from Workstation.workstation_core.mavlink_sources import (
    LoggerCommandReceipt,
    LoggerControlAction,
    LoggerRuntimeIdentity,
    LoggerStatusEvidence,
    MavlinkSourceConfig,
    MavlinkSourceId,
    ULOG_MAGIC,
    ULogInventoryEntry,
    parse_logger_status,
)


SESSION = "run-20260813-a"
REMOTE = "/fs/microsd/log/2026-08-13/workstation.ulg"


def _valid_ulog() -> bytes:
    header = ULOG_MAGIC + bytes((1,)) + struct.pack("<Q", 123_456)
    flag_payload = bytes(40)
    flag_record = struct.pack("<HB", len(flag_payload), ord("B")) + flag_payload
    return header + flag_record


class FakeULogAdapter:
    def __init__(
        self,
        *,
        payload: bytes | None = None,
        initially_running: bool = False,
        acknowledge: bool = True,
        replay_query_id: bool = False,
    ) -> None:
        self.identity = LoggerRuntimeIdentity(
            source_id=MavlinkSourceId.LEFT,
            connection_generation=3,
            boot_id="boot-a",
            source_system=1,
            source_component=1,
        )
        self.payload = _valid_ulog() if payload is None else payload
        self.status = (
            parse_logger_status(
                "Running in mode: all\nFull File Logging Running:\n"
                f"Log file: {REMOTE}\n"
            )
            if initially_running
            else parse_logger_status("Not logging")
        )
        self.inventory_visible = initially_running
        self.acknowledge = acknowledge
        self.replay_query_id = replay_query_id
        self.monotonic_ns = 100
        self.commands: list[LoggerControlAction] = []
        self.download_count = 0

    def current_runtime_identity(self) -> LoggerRuntimeIdentity:
        return self.identity

    def query_logger_status(self, *, query_id: str) -> LoggerStatusEvidence:
        self.monotonic_ns += 10
        return LoggerStatusEvidence(
            runtime_identity=self.identity,
            status=self.status,
            observed_at_monotonic_ns=self.monotonic_ns,
            query_id="replayed-query" if self.replay_query_id else query_id,
        )

    def execute_logger_command(self, decision):
        self.commands.append(decision.action)
        self.monotonic_ns += 1
        if decision.action == LoggerControlAction.START:
            self.status = parse_logger_status(
                "Running in mode: all\nFull File Logging Running:\n"
                f"Log file: {REMOTE}\n"
            )
        elif decision.action == LoggerControlAction.STOP:
            self.status = parse_logger_status("Not logging")
            self.inventory_visible = True
        return LoggerCommandReceipt(
            action=decision.action,
            runtime_identity=self.identity,
            session_id=decision.session_id,
            command_id=decision.command_id,
            acknowledged=self.acknowledge,
            completed_at_monotonic_ns=self.monotonic_ns,
        )

    def list_ulog_inventory(self, *, session_id: str):
        if not self.inventory_visible:
            return ()
        return (
            ULogInventoryEntry(
                runtime_identity=self.identity,
                session_id=session_id,
                remote_path=REMOTE,
                size_bytes=len(self.payload),
                listed_at_utc_ns=1_723_500_000_000_000_000,
            ),
        )

    def download_ulog(self, entry, writer) -> None:
        assert entry.runtime_identity == self.identity
        assert entry.remote_path == REMOTE
        self.download_count += 1
        writer.write(self.payload)


def _controller(tmp_path: Path, adapter: FakeULogAdapter) -> ULogController:
    config = MavlinkSourceConfig.left(
        "udp:127.0.0.1:14550",
        expected_system_id=1,
        expected_component_id=1,
    )
    return ULogController(
        tmp_path / "session",
        SESSION,
        automatic_sources={
            "p1": AutomaticULogBinding(config=config, adapter=adapter),
        },
    )


def test_external_preexisting_logger_is_never_stopped_or_downloaded(tmp_path):
    adapter = FakeULogAdapter(initially_running=True)
    controller = _controller(tmp_path, adapter)

    started = controller.start_automatic_ulog("P1")
    assert started["sources"]["left"]["owner"] == "EXTERNAL_ALREADY_RUNNING"
    assert started["sources"]["left"]["last_decision"]["action"] == "NONE"

    stopped = controller.stop_automatic_ulog_and_collect("left")
    assert stopped["sources"]["left"]["last_decision"]["reason"] == "NOT_WORKSTATION_OWNED"
    assert adapter.commands == []
    assert adapter.download_count == 0
    assert stopped["artifacts"] == []


def test_owned_logger_stops_and_seals_exact_new_ulog_atomically(tmp_path):
    adapter = FakeULogAdapter()
    controller = _controller(tmp_path, adapter)

    started = controller.start_automatic_ulog("left")
    assert started["sources"]["left"]["owner"] == "WORKSTATION"
    assert started["sources"]["left"]["ownership_token_present"] is True

    stopped = controller.stop_automatic_ulog_and_collect("p1")

    assert adapter.commands == [LoggerControlAction.START, LoggerControlAction.STOP]
    assert adapter.download_count == 1
    assert stopped["sources"]["left"]["phase"] == "STOPPED"
    assert stopped["sources"]["left"]["ownership_token_present"] is False
    [artifact] = stopped["artifacts"]
    saved = Path(stopped["session_root"]) / artifact["relative_path"]
    metadata = Path(stopped["session_root"]) / artifact["metadata_relative_path"]
    assert saved.read_bytes() == adapter.payload
    assert artifact["sha256"] == hashlib.sha256(adapter.payload).hexdigest()
    assert artifact["ownership"] == "WORKSTATION"
    assert artifact["ownership_basis"] == "verified_logger_ownership_token"
    assert artifact["ownership_token_sha256"]
    assert artifact["runtime_identity"]["boot_id"] == "boot-a"
    assert artifact["remote_path"] == REMOTE
    assert artifact["coordinate_contract"] == ULOG_COORDINATE_CONTRACT
    assert json.loads(metadata.read_text(encoding="utf-8"))["validation"]["valid"] is True
    assert not tuple((tmp_path / "session").rglob("*.part"))


def test_changed_remote_path_denies_stop_and_preserves_token(tmp_path):
    adapter = FakeULogAdapter()
    controller = _controller(tmp_path, adapter)
    controller.start_automatic_ulog("p1")
    adapter.status = parse_logger_status(
        "Full File Logging Running:\n"
        "Log file: /fs/microsd/log/2026-08-13/someone-else.ulg\n"
    )

    snapshot = controller.stop_automatic_ulog_and_collect("p1")

    assert adapter.commands == [LoggerControlAction.START]
    assert adapter.download_count == 0
    assert snapshot["sources"]["left"]["last_decision"]["reason"] == "REMOTE_LOG_PATH_MISMATCH"
    assert snapshot["sources"]["left"]["ownership_token_present"] is True


def test_changed_boot_or_connection_generation_denies_stop(tmp_path):
    adapter = FakeULogAdapter()
    controller = _controller(tmp_path, adapter)
    controller.start_automatic_ulog("left")
    adapter.identity = LoggerRuntimeIdentity(
        source_id=MavlinkSourceId.LEFT,
        connection_generation=4,
        boot_id="boot-b",
        source_system=1,
        source_component=1,
    )

    snapshot = controller.stop_automatic_ulog_and_collect("left")

    assert adapter.commands == [LoggerControlAction.START]
    assert snapshot["sources"]["left"]["last_decision"]["reason"] == "RUNTIME_IDENTITY_MISMATCH"
    assert adapter.download_count == 0


def test_uncorrelated_status_or_unacknowledged_start_never_grants_stop_authority(tmp_path):
    replay = FakeULogAdapter(replay_query_id=True)
    replay_controller = _controller(tmp_path / "replay", replay)
    with pytest.raises(ULogControllerError, match="correlation"):
        replay_controller.start_automatic_ulog("p1")
    assert replay.commands == []

    unacknowledged = FakeULogAdapter(acknowledge=False)
    controller = _controller(tmp_path / "unack", unacknowledged)
    started = controller.start_automatic_ulog("p1")
    assert started["sources"]["left"]["owner"] == "UNKNOWN"
    stopped = controller.stop_automatic_ulog_and_collect("p1")
    assert stopped["sources"]["left"]["last_decision"]["reason"] == "NOT_WORKSTATION_OWNED"
    assert unacknowledged.commands == [LoggerControlAction.START]
    assert unacknowledged.download_count == 0


def test_invalid_download_is_not_renamed_or_recorded(tmp_path):
    adapter = FakeULogAdapter(payload=b"not a ULog")
    controller = _controller(tmp_path, adapter)
    controller.start_automatic_ulog("p1")

    with pytest.raises(ULogArtifactValidationError, match="ULog validation failed"):
        controller.stop_automatic_ulog_and_collect("p1")

    root = tmp_path / "session"
    assert adapter.commands == [LoggerControlAction.START, LoggerControlAction.STOP]
    assert not tuple(root.rglob("*.part"))
    assert not tuple(root.rglob("*.ulg"))
    assert controller.ulog_snapshot()["artifacts"] == []


@pytest.mark.parametrize(
    ("source", "expected_source", "expected_role"),
    (
        ("p1", "left", "RIG_P1"),
        ("p2", "right", "RIG_P2"),
        ("target", "target_drone", "TARGET_REFERENCE"),
    ),
)
def test_manual_fallback_imports_exact_valid_bytes_with_declared_provenance(
    tmp_path,
    source,
    expected_source,
    expected_role,
):
    original = tmp_path / f"operator-{source}.ulg"
    payload = _valid_ulog()
    original.write_bytes(payload)
    controller = ULogController(tmp_path / f"session-{source}", SESSION)

    artifact = controller.import_manual_ulog(
        source,
        original,
        ownership=ManualOwnershipClaim.EXTERNAL,
        source_observed_utc_ns=1_723_500_000_000_000_000,
    )

    saved = Path(controller.ulog_snapshot()["session_root"]) / artifact["relative_path"]
    metadata_path = (
        Path(controller.ulog_snapshot()["session_root"])
        / artifact["metadata_relative_path"]
    )
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    assert original.read_bytes() == payload
    assert saved.read_bytes() == payload
    assert artifact["source_id"] == expected_source
    assert artifact["source_role"] == expected_role
    assert artifact["acquisition"] == "manual"
    assert artifact["ownership"] == "EXTERNAL"
    assert artifact["ownership_basis"] == "operator_declared_unverified"
    assert artifact["runtime_identity_basis"].endswith("placeholder_255")
    assert artifact["source_mtime_ns"] == original.stat().st_mtime_ns
    assert artifact["imported_utc_ns"] > 0
    assert artifact["imported_monotonic_ns"] > 0
    assert artifact["remote_path"] is None
    assert metadata["validation"]["record_structure_valid"] is True
    assert metadata["validation"]["sha256"] == hashlib.sha256(payload).hexdigest()


def test_manual_expected_hash_mismatch_leaves_no_session_artifact(tmp_path):
    original = tmp_path / "p2.ulg"
    original.write_bytes(_valid_ulog())
    controller = ULogController(tmp_path / "session", SESSION)

    with pytest.raises(ULogArtifactValidationError, match="SHA256_MISMATCH"):
        controller.import_manual_ulog(
            "p2",
            original,
            expected_sha256="0" * 64,
        )

    assert original.exists()
    assert not tuple((tmp_path / "session").rglob("*.part"))
    assert not tuple((tmp_path / "session").rglob("*.ulg"))
    assert controller.ulog_snapshot()["artifacts"] == []


def test_ulog_artifact_ledger_recovers_torn_tail_before_append(tmp_path):
    root = tmp_path / "session"
    root.mkdir()
    ledger = root / "ulog_artifacts.jsonl"
    prefix = b'{"artifact_id":"valid"}\n'
    ledger.write_bytes(prefix + b'{"artifact_id":')

    controller = ULogController(root, SESSION)

    assert ledger.read_bytes() == prefix
    assert controller._manifest_recovery.quarantined_tail is not None
