from __future__ import annotations

import multiprocessing

import pytest

from Workstation.workstation_core.native_frame_ring import (
    FrameKey,
    SharedNativeFrameRing,
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


def _publish_in_spawned_child(ring: SharedNativeFrameRing) -> None:
    ring.publish_bytes(
        _frame_payload(7),
        width=4,
        height=3,
        channels=3,
        timestamp_s=7.5,
    )
    ring.close()


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
