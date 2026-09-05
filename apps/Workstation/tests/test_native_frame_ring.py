from __future__ import annotations

import json
import multiprocessing
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from Workstation.workstation_core import native_frame_ring
from Workstation.workstation_core.native_frame_ring import (
    FrameKey,
    NATIVE_FRAME_SHM_PREFIX,
    SharedNativeFrameRing,
    reclaim_orphaned_native_frame_segments,
)


@pytest.fixture
def ring():
    value = SharedNativeFrameRing.create(
        camera_id="left",
        stream_generation=7,
        capacity=2,
        max_width=4,
        max_height=3,
        channels=3,
        mp_context=multiprocessing.get_context("spawn"),
    )
    try:
        yield value
    finally:
        value.close()
        value.unlink()


def _frame_payload(seed: int) -> bytes:
    return bytes([seed]) * (4 * 3 * 3)


def test_owner_unlink_is_idempotent_after_external_segment_cleanup() -> None:
    from multiprocessing import shared_memory

    value = SharedNativeFrameRing.create(
        camera_id="left",
        stream_generation=91,
        capacity=1,
        max_width=4,
        max_height=3,
        channels=3,
        mp_context=multiprocessing.get_context("spawn"),
    )
    name = value.shm_name
    value.close()
    external = shared_memory.SharedMemory(name=name, create=False)
    try:
        external.unlink()
    finally:
        external.close()

    value.unlink()
    value.unlink()


def _publish_in_spawned_child(ring: SharedNativeFrameRing) -> None:
    ring.publish_bytes(
        _frame_payload(7),
        width=4,
        height=3,
        channels=3,
        timestamp_s=7.5,
    )
    ring.close()


@pytest.mark.parametrize(
    ("field", "tampered"),
    (
        ("camera_id", "right"),
        ("stream_generation", 8),
        ("frame_id", 99),
    ),
)
def test_frame_key_rejects_redundant_compact_component_mismatch(
    field: str,
    tampered: object,
) -> None:
    payload = FrameKey("left", 7, 3).to_mapping()
    payload[field] = tampered

    with pytest.raises(ValueError, match="frame_key/component mismatch"):
        FrameKey.from_mapping(payload)


def test_frame_key_accepts_compact_only_identity_without_inventing_components() -> None:
    assert FrameKey.from_mapping({"frame_key": "left:7:3"}) == FrameKey(
        "left",
        7,
        3,
    )


def test_ring_uses_exact_generation_and_frame_id_without_timestamp_fallback(ring) -> None:
    first = ring.publish_bytes(
        _frame_payload(1),
        width=4,
        height=3,
        channels=3,
        timestamp_s=10.0,
        source_timestamp_s=100.0,
    )
    second = ring.publish_bytes(
        _frame_payload(2),
        width=4,
        height=3,
        channels=3,
        timestamp_s=10.0001,
        source_timestamp_s=100.0001,
    )

    assert first.key == FrameKey("left", 7, 1)
    assert second.key == FrameKey("left", 7, 2)
    assert ring.descriptor(FrameKey("left", 7, 1)) == first
    assert ring.descriptor(FrameKey("left", 6, 1)) is None
    assert ring.descriptor(FrameKey("right", 7, 1)) is None
    assert ring.descriptor(FrameKey("left", 7, 3)) is None


def test_pin_prevents_overwrite_and_all_pinned_fails_closed() -> None:
    ring = SharedNativeFrameRing.create(
        camera_id="right",
        stream_generation=11,
        capacity=1,
        max_width=4,
        max_height=3,
        channels=3,
        mp_context=multiprocessing.get_context("spawn"),
    )
    try:
        first = ring.publish_bytes(
            _frame_payload(3),
            width=4,
            height=3,
            channels=3,
            timestamp_s=1.0,
        )
        lease = ring.pin(first.key)
        assert lease is not None

        rejected = ring.publish_bytes(
            _frame_payload(4),
            width=4,
            height=3,
            channels=3,
            timestamp_s=2.0,
        )
        assert rejected.available is False
        assert rejected.key.frame_id == 2
        assert rejected.unavailable_reason == "all-native-ring-slots-pinned"
        assert ring.read(lease).data == _frame_payload(3)

        assert ring.release(lease) is True
        assert ring.release(lease) is False
        third = ring.publish_bytes(
            _frame_payload(5),
            width=4,
            height=3,
            channels=3,
            timestamp_s=3.0,
        )
        assert third.available is True
        assert third.key.frame_id == 3
        assert ring.descriptor(first.key) is None
        assert ring.pin(first.key) is None
    finally:
        ring.close()
        ring.unlink()


def test_pinned_snapshot_binds_bytes_dimensions_and_descriptor_hash(ring) -> None:
    descriptor = ring.publish_bytes(
        _frame_payload(9),
        width=4,
        height=3,
        channels=3,
        timestamp_s=12.5,
        source_timestamp_s=22.5,
    )
    lease = ring.pin(descriptor.key)
    assert lease is not None
    snapshot = ring.read(lease)

    assert snapshot.data == _frame_payload(9)
    assert snapshot.descriptor.key.text == "left:7:1"
    assert snapshot.descriptor.timestamp_s == pytest.approx(12.5)
    assert snapshot.descriptor.source_timestamp_s == pytest.approx(22.5)
    assert snapshot.descriptor.to_mapping()["frame_descriptor_sha256"]
    assert ring.release(lease) is True


def test_ring_metrics_report_overwrite_drop_and_pin_high_water(ring) -> None:
    first = ring.publish_bytes(
        _frame_payload(1),
        width=4,
        height=3,
        channels=3,
        timestamp_s=1.0,
    )
    second = ring.publish_bytes(
        _frame_payload(2),
        width=4,
        height=3,
        channels=3,
        timestamp_s=2.0,
    )
    first_lease = ring.pin(first.key)
    second_lease = ring.pin(second.key)
    assert first_lease is not None and second_lease is not None
    dropped = ring.publish_bytes(
        _frame_payload(3),
        width=4,
        height=3,
        channels=3,
        timestamp_s=3.0,
    )
    assert dropped.available is False

    assert ring.release(first_lease)
    fourth = ring.publish_bytes(
        _frame_payload(4),
        width=4,
        height=3,
        channels=3,
        timestamp_s=4.0,
    )
    assert fourth.available
    metrics = ring.metrics()
    assert metrics["capacity"] == 2
    assert metrics["published"] == 3
    assert metrics["overwritten"] == 1
    assert metrics["dropped"] == 1
    assert metrics["active_leases"] == 1
    assert metrics["pin_high_water"] == 2
    assert ring.release(second_lease)


def test_out_of_contract_frame_is_rejected_without_touching_slots(ring) -> None:
    descriptor = ring.publish_bytes(
        b"too-short",
        width=4,
        height=3,
        channels=3,
        timestamp_s=1.0,
    )
    assert descriptor.available is False
    assert descriptor.unavailable_reason == "frame-size-or-format-out-of-contract"
    assert ring.latest_descriptor() is None
    assert ring.metrics()["dropped"] == 1


def test_spawned_camera_process_publishes_into_parent_owned_shared_memory(ring) -> None:
    context = multiprocessing.get_context("spawn")
    process = context.Process(target=_publish_in_spawned_child, args=(ring,))
    process.start()
    process.join(timeout=5.0)

    assert process.exitcode == 0
    descriptor = ring.latest_descriptor()
    assert descriptor is not None
    assert descriptor.key == FrameKey("left", 7, 1)
    lease = ring.pin(descriptor.key)
    assert lease is not None
    assert ring.read(lease).data == _frame_payload(7)
    assert ring.release(lease)


@pytest.mark.skipif(not Path("/dev/shm").is_dir(), reason="Linux shared-memory oracle required")
def test_startup_reclaims_only_dead_workstation_native_frame_segments(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from multiprocessing import shared_memory

    token = f"{time.time_ns() & ((1 << 64) - 1):016x}"
    dead_name = f"{NATIVE_FRAME_SHM_PREFIX}99999999_{token}"
    live_name = f"{NATIVE_FRAME_SHM_PREFIX}{os.getpid()}_{token}"
    unrelated_name = f"sasl_ws_other_{os.getpid()}_{token}"
    segments = [
        shared_memory.SharedMemory(name=name, create=True, size=1)
        for name in (dead_name, live_name, unrelated_name)
    ]
    for segment in segments:
        segment.close()

    monkeypatch.setattr(
        native_frame_ring,
        "_process_is_alive",
        lambda pid: pid == os.getpid(),
    )
    try:
        assert reclaim_orphaned_native_frame_segments() == (dead_name,)
        assert not (Path("/dev/shm") / dead_name).exists()
        assert (Path("/dev/shm") / live_name).is_file()
        assert (Path("/dev/shm") / unrelated_name).is_file()
    finally:
        for name in (dead_name, live_name, unrelated_name):
            try:
                residual = shared_memory.SharedMemory(name=name, create=False)
            except FileNotFoundError:
                continue
            try:
                residual.unlink()
            finally:
                residual.close()


@pytest.mark.skipif(not Path("/dev/shm").is_dir(), reason="Linux shared-memory oracle required")
def test_resource_tracker_reclaims_owner_ring_after_sigkill() -> None:
    workstation_root = Path(__file__).resolve().parents[1]
    apps_root = workstation_root.parent
    helper_env = dict(os.environ)
    helper_env["PYTHONPATH"] = os.pathsep.join(
        value
        for value in (str(apps_root), helper_env.get("PYTHONPATH", ""))
        if value
    )
    helper = """
import json
import multiprocessing
import time
from Workstation.workstation_core.native_frame_ring import SharedNativeFrameRing

ring = SharedNativeFrameRing.create(
    camera_id="crash-test",
    stream_generation=1,
    capacity=1,
    max_width=4,
    max_height=3,
    channels=3,
    mp_context=multiprocessing.get_context("spawn"),
)
print(json.dumps({"shm_name": ring.shm_name}), flush=True)
while True:
    time.sleep(1.0)
"""
    process = subprocess.Popen(
        [sys.executable, "-c", helper],
        cwd=workstation_root,
        env=helper_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    shm_path: Path | None = None
    try:
        assert process.stdout is not None
        line = process.stdout.readline()
        if not line:
            assert process.stderr is not None
            pytest.fail(f"shared-memory helper exited before readiness: {process.stderr.read()}")
        payload = json.loads(line)
        shm_path = Path("/dev/shm") / str(payload["shm_name"])
        assert shm_path.is_file()

        os.kill(process.pid, signal.SIGKILL)
        assert process.wait(timeout=5.0) == -signal.SIGKILL

        deadline = time.monotonic() + 5.0
        while shm_path.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not shm_path.exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5.0)
        if shm_path is not None and shm_path.exists():
            from multiprocessing import shared_memory

            residual = shared_memory.SharedMemory(name=shm_path.name, create=False)
            try:
                residual.unlink()
            finally:
                residual.close()
