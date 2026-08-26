import json
import math
import os
from pathlib import Path
import stat
import time
from types import SimpleNamespace

import pytest

from Workstation.target_drone_controller import (
    TARGET_DRONE_COORDINATE_CONTRACT,
    TargetDroneController,
    TargetDroneControllerError,
    TargetDroneControllerPhase,
    TargetDroneRuntimeRequest,
)
from Workstation.workstation_core.target_drone_stream import TargetDroneReceiver


class FakePanel:
    def __init__(self, root: Path, session_id: str = "session-001") -> None:
        self.root = root
        self.recording_session_id = session_id

    def _recording_session_dir(self, session_id: str) -> Path:
        return self.root / session_id


class FakeMessage:
    def __init__(self, message_type, *, sysid=42, compid=1, seq=0, msgid=None, **fields):
        self._message_type = message_type
        self._sysid = sysid
        self._compid = compid
        self._seq = seq
        self._msgid = {
            "HEARTBEAT": 0,
            "ATTITUDE": 30,
            "GPS_RAW_INT": 24,
            "GPS2_RAW": 124,
        }.get(message_type, 999) if msgid is None else msgid
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self):
        return self._message_type

    def get_srcSystem(self):
        return self._sysid

    def get_srcComponent(self):
        return self._compid

    def get_seq(self):
        return self._seq

    def get_msgId(self):
        return self._msgid


class FakeConnection:
    def __init__(self, heartbeat, messages=()):
        self.heartbeat = heartbeat
        self.messages = list(messages)
        self.closed = False
        self.wait_calls = []
        self.recv_calls = []

    @property
    def mav(self):
        raise AssertionError("target controller must never access a MAVLink send object")

    def wait_heartbeat(self, *, timeout):
        self.wait_calls.append(timeout)
        return self.heartbeat

    def recv_match(self, *, blocking):
        self.recv_calls.append(blocking)
        assert blocking is False
        return self.messages.pop(0) if self.messages else None

    def close(self):
        self.closed = True


class FakeMavutil:
    def __init__(self, connection):
        self.connection = connection
        self.calls = []

    def mavlink_connection(self, endpoint, **kwargs):
        self.calls.append((endpoint, dict(kwargs)))
        return self.connection


def _controller(panel, mavutil, **kwargs):
    def receiver_factory(config, **receiver_kwargs):
        return TargetDroneReceiver(
            config,
            mavutil_module=mavutil,
            **receiver_kwargs,
        )

    return TargetDroneController(panel, receiver_factory=receiver_factory, **kwargs)


def _wait_for(predicate, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.005)
    raise AssertionError("condition was not reached before timeout")


def test_drone_owner_receives_only_then_drains_and_seals_session_spool(tmp_path):
    connection = FakeConnection(
        FakeMessage("HEARTBEAT", seq=1),
        [
            FakeMessage(
                "ATTITUDE",
                seq=2,
                time_boot_ms=123,
                roll=0.1,
                pitch=-0.2,
                yaw=math.pi / 4,
            ),
            FakeMessage(
                "GPS_RAW_INT",
                seq=3,
                time_usec=1_700_000_000_000_000,
                lat=375_123_456,
                lon=1271_234_567,
                alt=102_345,
                cog=9_000,
                yaw=9_100,
                fix_type=6,
            ),
        ],
    )
    mavutil = FakeMavutil(connection)
    panel = FakePanel(tmp_path)
    controller = _controller(panel, mavutil)

    configured = controller.configure_target_drone_stream(
        {
            "mode": "drone",
            "endpoint": "udp:127.0.0.1:14570",
            "expected_system_id": 42,
            "expected_component_id": 1,
            "spool_fsync_every_rows": 1,
        }
    )
    assert configured["phase"] == "CONFIGURED"
    assert configured["source_id"] == "target_drone"
    assert configured["source_role"] == "TARGET_REFERENCE"
    assert configured["mutation_policy"] == "recv_only"

    controller.start_target_drone_stream()
    _wait_for(
        lambda: controller.target_drone_stream_snapshot()["accepted_message_count"] == 2
    )
    stopped = controller.stop_target_drone_stream(timeout_s=1.0)

    assert stopped["phase"] == "SEALED"
    assert stopped["stream_state"] == "STOPPED"
    assert stopped["worker_alive"] is False
    assert stopped["accepted_message_count"] == 2
    assert stopped["verified_source_system"] == 42
    assert stopped["verified_source_component"] == 1
    assert stopped["truth_semantics"] == "runtime_reference_not_calibration_truth"
    assert stopped["coordinate_contract"] == TARGET_DRONE_COORDINATE_CONTRACT
    assert connection.closed is True
    assert connection.wait_calls == [3.0]
    assert all(value is False for value in connection.recv_calls)
    assert mavutil.calls == [
        (
            "udp:127.0.0.1:14570",
            {"baud": 57_600, "autoreconnect": False, "robust_parsing": True},
        )
    ]

    artifact_mapping = stopped["artifact"]
    artifact = controller.snapshot().artifact
    assert artifact is not None
    artifact_path = Path(artifact_mapping["absolute_path"])
    assert artifact_path.is_relative_to(tmp_path / "session-001")
    assert artifact_mapping["relative_path"].startswith("raw/target_drone/")
    assert artifact_mapping["mode"] == "DRONE"
    assert artifact_mapping["source_id"] == "target_drone"
    assert artifact_mapping["ledger_role"] == "target_drone_raw_mavlink_jsonl"
    assert artifact_mapping["verified"] is True
    assert artifact_mapping["closed"] is True
    assert artifact_mapping["read_only"] is True
    assert artifact_mapping["size_bytes"] == artifact_path.stat().st_size
    assert len(artifact_mapping["sha256"]) == 64
    assert stat.S_IMODE(artifact_path.stat().st_mode) & 0o222 == 0
    assert artifact.verify() is True
    ledger_kwargs = artifact.run_evidence_import_kwargs()
    assert ledger_kwargs["source_path"] == artifact_path
    assert ledger_kwargs["role"] == "target_drone_raw_mavlink_jsonl"
    assert "absolute_path" not in ledger_kwargs["metadata"]
    assert ledger_kwargs["metadata"]["session_source_relative_path"].startswith(
        "raw/target_drone/"
    )
    assert not list((tmp_path / "session-001" / ".spool").rglob("*.part"))

    rows = [json.loads(line) for line in artifact_path.read_text(encoding="utf-8").splitlines()]
    assert len(rows) == artifact_mapping["row_count"] == stopped["spool_row_count"]
    assert [row["message_type"] for row in rows if row["record_kind"] == "telemetry"] == [
        "ATTITUDE",
        "GPS_RAW_INT",
    ]
    assert all(row["source_id"] == "target_drone" for row in rows)
    assert all(row["source_role"] == "TARGET_REFERENCE" for row in rows)


def test_uam_is_deferred_without_worker_link_spool_or_artifact(tmp_path):
    panel = FakePanel(tmp_path)
    worker_factory_calls = []

    def forbidden_worker_factory(*args, **kwargs):
        worker_factory_calls.append((args, kwargs))
        raise AssertionError("UAM mode must not create a live worker")

    controller = TargetDroneController(panel, worker_factory=forbidden_worker_factory)
    configured = controller.configure_target_drone_stream({"mode": "uam"})
    assert configured["endpoint"] == "deferred://uam-post-flight-rtk"

    started = controller.start_target_drone_stream()
    stopped = controller.stop_target_drone_stream()

    assert started["phase"] == "DEFERRED"
    assert started["stream_state"] == "DEFERRED"
    assert stopped["phase"] == "DEFERRED"
    assert stopped["stream_state"] == "DEFERRED"
    assert stopped["worker_alive"] is False
    assert stopped["artifact"] is None
    assert worker_factory_calls == []
    assert not (tmp_path / "session-001" / ".spool").exists()
    assert controller.drain_target_drone_stream() == 0


def test_sessionless_target_uses_fixed_best_effort_stream_only_while_recording(tmp_path):
    connection = FakeConnection(
        FakeMessage("HEARTBEAT", seq=1),
        [
            FakeMessage(
                "ATTITUDE",
                seq=2,
                time_boot_ms=123,
                roll=0.1,
                pitch=-0.2,
                yaw=0.3,
            )
        ],
    )
    accepted: list[tuple[str, dict[str, object]]] = []
    recording = {"active": True}
    panel = FakePanel(tmp_path)
    panel._measurement_session_hierarchy_enabled = False
    panel._session_recording_active = lambda: recording["active"]
    panel.append_workstation_data = (
        lambda stream, payload, **_kwargs: accepted.append(
            (str(stream), dict(payload))
        )
        or True
    )
    panel.flush_workstation_data = lambda: True
    controller = _controller(panel, FakeMavutil(connection))

    controller.configure_target_drone_stream(
        {
            "mode": "DRONE",
            "endpoint": "udp:127.0.0.1:14570",
            "expected_system_id": 42,
            "expected_component_id": 1,
        }
    )
    controller.start_target_drone_stream()
    _wait_for(
        lambda: controller.target_drone_stream_snapshot()["accepted_message_count"] == 1
    )
    stopped = controller.stop_target_drone_stream(timeout_s=1.0)

    assert stopped["phase"] == "STOPPED"
    assert stopped["artifact"] is None
    assert stopped["data_owner"] == "measurement_data_root"
    assert "session_id" not in stopped
    assert accepted
    assert {stream for stream, _row in accepted} == {"px4.drone"}
    assert any(row.get("record_kind") == "telemetry" for _stream, row in accepted)
    assert not (tmp_path / "target-drone" / ".spool").exists()

    # A writer refusal remains a data-quality signal, not a receiver error.
    recording["active"] = False
    panel.append_workstation_data = lambda *_args, **_kwargs: False
    second_connection = FakeConnection(FakeMessage("HEARTBEAT", seq=3))
    controller = _controller(panel, FakeMavutil(second_connection))
    controller.configure_target_drone_stream(
        {"mode": "DRONE", "endpoint": "udp:127.0.0.1:14570"}
    )
    controller.start_target_drone_stream()
    _wait_for(
        lambda: controller.target_drone_stream_snapshot()["stream_state"] == "STREAMING"
    )
    assert controller.stop_target_drone_stream(timeout_s=1.0)["phase"] == "STOPPED"


def test_seal_detects_post_stop_byte_tampering(tmp_path):
    connection = FakeConnection(FakeMessage("HEARTBEAT"))
    panel = FakePanel(tmp_path)
    controller = _controller(panel, FakeMavutil(connection))
    controller.connect_target_drone_stream(
        {
            "mode": "DRONE",
            "endpoint": "udp:127.0.0.1:14570",
            "expected_system_id": 42,
            "expected_component_id": 1,
        }
    )
    _wait_for(lambda: controller.target_drone_stream_snapshot()["stream_state"] == "STREAMING")
    controller.stop_target_drone_stream(timeout_s=1.0)
    artifact = controller.snapshot().artifact
    assert artifact is not None and artifact.verify()

    path = Path(artifact.absolute_path)
    os.chmod(path, stat.S_IRUSR | stat.S_IWUSR)
    with path.open("ab") as handle:
        handle.write(b"{}\n")

    assert artifact.verify() is False


def test_identity_mismatch_is_preserved_as_failed_but_lifecycle_spool_is_sealed(tmp_path):
    connection = FakeConnection(FakeMessage("HEARTBEAT", sysid=77))
    controller = _controller(FakePanel(tmp_path), FakeMavutil(connection))
    controller.connect_target_drone_stream(
        {
            "mode": "DRONE",
            "endpoint": "udp:127.0.0.1:14570",
            "expected_system_id": 42,
            "expected_component_id": 1,
        }
    )
    _wait_for(lambda: not controller.target_drone_stream_snapshot()["worker_alive"])

    stopped = controller.stop_target_drone_stream(timeout_s=1.0)

    assert stopped["phase"] == "FAILED"
    assert stopped["stream_state"] == "IDENTITY_MISMATCH"
    assert "sysid mismatch" in stopped["last_error"]
    assert stopped["artifact"] is not None
    assert stopped["artifact"]["accepted_message_count"] == 0
    assert controller.snapshot().artifact.verify() is True


def test_session_is_bound_at_configuration_and_must_not_change(tmp_path):
    panel = FakePanel(tmp_path, "session-a")
    controller = _controller(panel, FakeMavutil(FakeConnection(FakeMessage("HEARTBEAT"))))
    controller.configure_target_drone_stream(
        {"mode": "DRONE", "endpoint": "udp:127.0.0.1:14570"}
    )
    panel.recording_session_id = "session-b"

    with pytest.raises(TargetDroneControllerError, match="session changed"):
        controller.start_target_drone_stream()


def test_missing_session_and_mutating_source_policy_fail_before_any_link(tmp_path):
    empty_panel = FakePanel(tmp_path, "")
    with pytest.raises(TargetDroneControllerError, match="session ID"):
        TargetDroneController(empty_panel).configure_target_drone_stream(
            {"mode": "UAM"}
        )

    panel = FakePanel(tmp_path)
    mutating_config = SimpleNamespace(
        source_id="target_drone",
        role="TARGET_REFERENCE",
        policy=SimpleNamespace(
            allow_stream_setup=True,
            allow_parameter_write=False,
            allow_rtcm_injection=False,
            allow_logger_control=False,
            ftp_access="READ_ONLY",
        ),
    )
    receiver_calls = []
    controller = TargetDroneController(
        panel,
        source_config_factory=lambda request: mutating_config,
        receiver_factory=lambda *args, **kwargs: receiver_calls.append((args, kwargs)),
    )

    with pytest.raises(ValueError, match="receive-only"):
        controller.configure_target_drone_stream(
            {"mode": "DRONE", "endpoint": "udp:127.0.0.1:14570"}
        )
    assert receiver_calls == []


@pytest.mark.parametrize(
    ("values", "error"),
    [
        ({"mode": "DRONE"}, "endpoint"),
        ({"mode": "DRONE", "endpoint": "udp:x", "expected_system_id": 0}, r"\[1, 255\]"),
        ({"mode": "DRONE", "endpoint": "udp:x", "spool_max_rows": 0}, "positive"),
    ],
)
def test_runtime_request_rejects_ambiguous_or_unbounded_configuration(values, error):
    with pytest.raises((TypeError, ValueError), match=error):
        TargetDroneRuntimeRequest.from_mapping(values)


def test_unconfigured_snapshot_is_explicit(tmp_path):
    snapshot = TargetDroneController(FakePanel(tmp_path)).snapshot()

    assert snapshot.phase == TargetDroneControllerPhase.UNCONFIGURED
    assert snapshot.configured is False
    assert snapshot.mode is None
    assert snapshot.artifact is None
