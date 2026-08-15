import json
import math
import time

import pytest

from Workstation.workstation_core.mavlink_sources import MavlinkSourceConfig
from Workstation.workstation_core.target_drone_stream import (
    BoundedAppendOnlyJsonlSpool,
    HeartbeatIdentityError,
    SpoolCapacityError,
    TargetDroneMode,
    TargetDroneReceiver,
    TargetDroneStreamState,
    TargetDroneWorker,
)


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


class SendForbidden:
    def __getattr__(self, name):
        raise AssertionError(f"recv-only target path accessed a send API: {name}")


class FakeConnection:
    def __init__(self, heartbeat, messages=()):
        self.heartbeat = heartbeat
        self.messages = list(messages)
        self._mav = SendForbidden()
        self.closed = False
        self.wait_calls = []
        self.recv_calls = []

    @property
    def mav(self):
        raise AssertionError("recv-only target path must never access connection.mav")

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
    def __init__(self, *connections):
        self.connections = list(connections)
        self.calls = []

    def mavlink_connection(self, endpoint, **kwargs):
        self.calls.append((endpoint, dict(kwargs)))
        return self.connections.pop(0)


class CounterClock:
    def __init__(self, start, step):
        self.value = start - step
        self.step = step

    def __call__(self):
        self.value += self.step
        return self.value


def _config(*, sysid=42, compid=1):
    return MavlinkSourceConfig.target_drone(
        "udp:127.0.0.1:14570",
        expected_system_id=sysid,
        expected_component_id=compid,
    )


def _read_rows(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_recv_only_runtime_separates_attitude_cog_and_gnss_yaw(tmp_path):
    connection = FakeConnection(
        FakeMessage("HEARTBEAT", seq=1),
        [
            FakeMessage(
                "ATTITUDE",
                seq=2,
                time_boot_ms=1234,
                roll=0.1,
                pitch=-0.2,
                yaw=math.pi / 4,
                rollspeed=0.01,
                pitchspeed=0.02,
                yawspeed=0.03,
            ),
            FakeMessage(
                "GPS_RAW_INT",
                seq=3,
                time_usec=1_700_000_000_000_000,
                lat=375_123_456,
                lon=1271_234_567,
                alt=102_345,
                alt_ellipsoid=130_000,
                eph=23,
                epv=41,
                vel=1_234,
                cog=9_000,
                fix_type=6,
                satellites_visible=21,
                yaw=9_100,
                hdg_acc=250_000,
            ),
            FakeMessage(
                "GPS2_RAW",
                seq=4,
                time_usec=5_000_000,
                lat=375_123_400,
                lon=1271_234_500,
                alt=102_000,
                eph=65_535,
                epv=65_535,
                vel=65_535,
                cog=65_535,
                fix_type=3,
                satellites_visible=12,
                yaw=0,
                hdg_acc=4_294_967_295,
            ),
        ],
    )
    mavutil = FakeMavutil(connection)
    path = tmp_path / "target.jsonl"
    captured = []
    receiver = TargetDroneReceiver(
        _config(),
        mode=TargetDroneMode.DRONE,
        spool_path=path,
        mavutil_module=mavutil,
        monotonic_ns=CounterClock(1_000_000, 10),
        utc_ns=CounterClock(2_000_000, 100),
        on_record=captured.append,
    )

    assert receiver.start() == TargetDroneStreamState.STREAMING
    assert receiver.poll_once(max_duration_s=1.0) == 3
    snapshot = receiver.stop()

    assert snapshot.state == TargetDroneStreamState.STOPPED
    assert snapshot.accepted_message_count == 3
    assert connection.closed is True
    assert mavutil.calls == [
        (
            "udp:127.0.0.1:14570",
            {"baud": 57_600, "autoreconnect": False, "robust_parsing": True},
        )
    ]
    assert connection.wait_calls == [3.0]
    assert all(blocking is False for blocking in connection.recv_calls)

    attitude = captured[0]
    assert attitude.yaw_observations.attitude_yaw.degrees_ned == pytest.approx(45.0)
    assert attitude.yaw_observations.course_over_ground is None
    assert attitude.yaw_observations.gnss_yaw is None
    gps1 = captured[1]
    assert gps1.sensor_slot == "GPS1"
    assert gps1.yaw_observations.course_over_ground.degrees_true_north == 90.0
    assert gps1.yaw_observations.gnss_yaw.degrees_true_north == 91.0
    assert gps1.yaw_observations.attitude_yaw is None
    assert gps1.fields["latitude_deg_wgs84"] == pytest.approx(37.5123456)
    assert gps1.fields["longitude_deg_wgs84"] == pytest.approx(127.1234567)
    assert gps1.fields["altitude_m_msl"] == pytest.approx(102.345)
    gps2 = captured[2]
    assert gps2.sensor_slot == "GPS2"
    assert gps2.yaw_observations.course_over_ground is None
    assert gps2.yaw_observations.gnss_yaw is None
    assert gps2.fields["horizontal_dilution_m"] is None

    telemetry = [row for row in _read_rows(path) if row["record_kind"] == "telemetry"]
    assert [row["message_type"] for row in telemetry] == ["ATTITUDE", "GPS_RAW_INT", "GPS2_RAW"]
    assert telemetry[0]["connection_generation"] == 1
    assert telemetry[0]["provenance"]["source_id"] == "target_drone"
    assert telemetry[0]["provenance"]["source_system"] == 42
    assert telemetry[0]["provenance"]["source_component"] == 1
    assert telemetry[0]["provenance"]["sequence"] == 2
    assert telemetry[0]["provenance"]["message_id"] == 30
    assert telemetry[0]["provenance"]["host_received_monotonic_ns"] > 0
    assert telemetry[0]["provenance"]["host_received_utc_ns"] > 0
    assert telemetry[0]["provenance"]["source_timestamp_us"] == 1_234_000
    assert telemetry[1]["source_time_basis"] == "utc_epoch"
    assert telemetry[2]["source_time_basis"] == "gnss_receiver_or_boot_time_unresolved"
    assert telemetry[0]["yaw_observations"] == {
        "attitude_yaw_deg_body_frd_to_world_ned": 45.0,
        "course_over_ground_deg_true_north": None,
        "gnss_yaw_deg_true_north": None,
    }


def test_heartbeat_identity_mismatch_fails_closed_without_telemetry(tmp_path):
    connection = FakeConnection(
        FakeMessage("HEARTBEAT", sysid=77),
        [FakeMessage("ATTITUDE", sysid=77, yaw=0.0)],
    )
    receiver = TargetDroneReceiver(
        _config(sysid=42),
        mode="DRONE",
        spool_path=tmp_path / "mismatch.jsonl",
        mavutil_module=FakeMavutil(connection),
    )

    with pytest.raises(HeartbeatIdentityError, match="sysid mismatch"):
        receiver.start()

    assert receiver.state == TargetDroneStreamState.IDENTITY_MISMATCH
    assert connection.closed is True
    assert all(row["record_kind"] != "telemetry" for row in _read_rows(tmp_path / "mismatch.jsonl"))


def test_packets_from_other_system_or_component_never_contaminate_latest_state(tmp_path):
    connection = FakeConnection(
        FakeMessage("HEARTBEAT"),
        [
            FakeMessage("ATTITUDE", sysid=99, seq=1, yaw=1.0),
            FakeMessage("GPS_RAW_INT", compid=2, seq=2, cog=1_000, yaw=2_000),
            FakeMessage("ATTITUDE", seq=3, time_boot_ms=9, roll=0.0, pitch=0.0, yaw=0.2),
            FakeMessage("STATUSTEXT", seq=4),
        ],
    )
    path = tmp_path / "cross-source.jsonl"
    receiver = TargetDroneReceiver(
        _config(), mode="DRONE", spool_path=path, mavutil_module=FakeMavutil(connection)
    )
    receiver.start()

    assert receiver.poll_once(max_duration_s=1.0) == 1
    snapshot = receiver.stop()

    assert snapshot.accepted_message_count == 1
    assert snapshot.rejected_cross_source_count == 2
    assert snapshot.ignored_message_count == 1
    assert receiver.last_attitude.provenance.source_system == 42
    telemetry = [row for row in _read_rows(path) if row["record_kind"] == "telemetry"]
    assert len(telemetry) == 1
    assert telemetry[0]["provenance"]["source_system"] == 42
    rejects = [row for row in _read_rows(path) if row.get("event") == "packet_identity_rejected"]
    assert len(rejects) == 2


def test_uam_mode_is_normally_deferred_and_opens_no_link_or_spool(tmp_path):
    mavutil = FakeMavutil()
    path = tmp_path / "must-not-exist.jsonl"
    receiver = TargetDroneReceiver(
        _config(), mode="uam", spool_path=path, mavutil_module=mavutil
    )
    worker = TargetDroneWorker(receiver)

    assert worker.start() == TargetDroneStreamState.DEFERRED
    assert worker.is_alive is False
    assert worker.stop().state == TargetDroneStreamState.DEFERRED
    assert mavutil.calls == []
    assert not path.exists()


def test_bounded_spool_refuses_whole_next_row_and_preserves_valid_prefix(tmp_path):
    path = tmp_path / "bounded.jsonl"
    spool = BoundedAppendOnlyJsonlSpool(path, max_rows=1, max_bytes=1024, fsync_every_rows=1)
    spool.open()
    spool.append({"value": 1})
    prefix = path.read_bytes()

    with pytest.raises(SpoolCapacityError, match="max_rows"):
        spool.append({"value": 2})
    spool.close()

    assert path.read_bytes() == prefix
    assert _read_rows(path) == [{"value": 1}]
    assert len(spool.sha256()) == 64


def test_worker_cooperatively_stops_and_drains_buffered_packets(tmp_path):
    messages = [
        FakeMessage(
            "ATTITUDE",
            seq=index,
            time_boot_ms=index,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )
        for index in range(1, 31)
    ]
    connection = FakeConnection(FakeMessage("HEARTBEAT"), messages)
    path = tmp_path / "threaded.jsonl"
    receiver = TargetDroneReceiver(
        _config(), mode="DRONE", spool_path=path, mavutil_module=FakeMavutil(connection)
    )
    worker = TargetDroneWorker(receiver, idle_wait_s=0.001)
    worker.start()
    deadline = time.monotonic() + 1.0
    while receiver.state not in (TargetDroneStreamState.STREAMING, TargetDroneStreamState.STOPPED):
        assert time.monotonic() < deadline
        time.sleep(0.001)

    snapshot = worker.stop(timeout_s=2.0)

    assert worker.is_alive is False
    assert worker.worker_error is None
    assert snapshot.state == TargetDroneStreamState.STOPPED
    assert snapshot.accepted_message_count == 30
    assert connection.messages == []
    assert connection.closed is True
    telemetry = [row for row in _read_rows(path) if row["record_kind"] == "telemetry"]
    assert len(telemetry) == 30


def test_reconnect_increments_generation_and_clears_previous_latest_sample(tmp_path):
    first = FakeConnection(
        FakeMessage("HEARTBEAT"),
        [FakeMessage("ATTITUDE", seq=1, time_boot_ms=1, roll=0.0, pitch=0.0, yaw=0.1)],
    )
    second = FakeConnection(
        FakeMessage("HEARTBEAT"),
        [
            FakeMessage(
                "GPS_RAW_INT",
                seq=2,
                time_usec=2,
                lat=1,
                lon=2,
                alt=3,
                cog=100,
                yaw=200,
            )
        ],
    )
    path = tmp_path / "generations.jsonl"
    receiver = TargetDroneReceiver(
        _config(), mode="DRONE", spool_path=path, mavutil_module=FakeMavutil(first, second)
    )

    receiver.start()
    receiver.poll_once(max_duration_s=1.0)
    assert receiver.last_attitude is not None
    receiver.stop()
    receiver.start()
    assert receiver.snapshot().connection_generation == 2
    assert receiver.last_attitude is None
    receiver.poll_once(max_duration_s=1.0)
    receiver.stop()

    telemetry = [row for row in _read_rows(path) if row["record_kind"] == "telemetry"]
    assert [row["connection_generation"] for row in telemetry] == [1, 2]
