"""Aliasing at a process/evidence boundary must not change frozen inputs."""

from __future__ import annotations

import copy
import hashlib
import json
import multiprocessing
import pickle

import pytest

from Workstation.workstation_core.method_evaluation import (
    FrozenMeasurementPairSnapshot,
    MethodEvaluationBundle,
    MethodEvaluationOutcome,
    MethodEvaluationRequest,
)


def _snapshot_payload():
    # Hand-authored wire values: RDF pixel order and clock fields are fixed.
    return {
        "schema": "workstation-frozen-measurement-pair-v1",
        "measurement_id": "run-immutable",
        "pair_id": "pair-1",
        "pair_sequence": 1,
        "created_monotonic_ns": 11,
        "created_unix_ns": 22,
        "timestamp_basis": "app_receive_monotonic",
        "left_detection": {"bbox_xyxy_px": [10.0, 20.0, 30.0, 40.0]},
        "right_detection": {"bbox_xyxy_px": [5.0, 20.0, 25.0, 40.0]},
        "left_attitude": None, "right_attitude": None,
        "left_gps": None, "right_gps": None,
        "native_frames": {"left": {"frame_id": 7}},
        "capture_sync": {"epoch_ids": ["left-1", "right-1"]},
        "reused_inputs": [],
    }


def _wire_hash(payload):
    return hashlib.sha256(json.dumps(
        payload, ensure_ascii=False, allow_nan=False, sort_keys=True,
        separators=(",", ":"),
    ).encode()).hexdigest()


def test_snapshot_detaches_input_and_export_and_preserves_independent_wire_hash():
    source = _snapshot_payload()
    expected = copy.deepcopy(source)
    snapshot = FrozenMeasurementPairSnapshot.from_mapping(source)
    original_hash = _wire_hash(expected)
    source["left_detection"]["bbox_xyxy_px"][0] = 999.0
    source["native_frames"]["left"]["frame_id"] = 999
    exported = snapshot.canonical_payload()
    exported["right_detection"]["bbox_xyxy_px"][0] = 888.0
    exported["capture_sync"]["epoch_ids"].append("foreign")
    assert snapshot.canonical_payload() == expected
    assert snapshot.canonical_sha256() == original_hash


def test_snapshot_rejects_direct_nested_mutation():
    snapshot = FrozenMeasurementPairSnapshot.from_mapping(_snapshot_payload())
    with pytest.raises(TypeError):
        snapshot.left_detection["bbox_xyxy_px"][0] = 999.0
    with pytest.raises(TypeError):
        snapshot.native_frames["left"]["frame_id"] = 999


def _objects():
    source = _snapshot_payload()
    identity = dict(
        measurement_id="run-immutable", pair_id="pair-1", pair_sequence=1,
        snapshot_sha256=_wire_hash(source), request_generation=1,
    )
    attempt = {"camera_matrix": [[100.0, 0.0, 20.0], [0.0, 100.0, 30.0], [0.0, 0.0, 1.0]]}
    request = MethodEvaluationRequest(
        **identity, method_id="pixhawk_rtk_live", method_instance_id="test",
        snapshot_payload=source, attempt_payload=attempt,
        enqueued_monotonic_ns=33,
    )
    result = {"point_m": [10.0, -0.5, 0.0]}
    outcome = MethodEvaluationOutcome(
        **identity, method_id="pixhawk_rtk_live", method_instance_id="test",
        status="VALID", attempt_payload=attempt, attempt_sha256=_wire_hash(attempt),
        result=result,
    )
    outcomes = [outcome]
    planned = {"checkerboard_stereo": "not ready"}
    bundle = MethodEvaluationBundle(
        **identity, snapshot_payload=source, outcomes=outcomes, planned_methods=planned,
    )
    return request, outcome, bundle, (source, attempt, result, outcomes, planned)


@pytest.mark.parametrize("index,export", [(0, "to_mapping"), (1, "to_record"), (2, "to_record")])
def test_request_outcome_bundle_own_inputs_and_return_detached_json(index, export):
    objects = _objects()
    obj = objects[index]
    original = copy.deepcopy(getattr(obj, export)())
    source, attempt, result, outcomes, planned = objects[3]
    source["native_frames"]["left"]["frame_id"] = 123
    attempt["camera_matrix"][0][0] = 999.0
    result["point_m"][0] = -999.0
    outcomes.clear()
    planned.clear()
    assert getattr(obj, export)() == original
    detached = getattr(obj, export)()

    def mutate(value):
        if isinstance(value, dict):
            for child in list(value.values()):
                mutate(child)
            value["mutation"] = True
        elif isinstance(value, list):
            value.clear()

    mutate(detached)
    assert getattr(obj, export)() == original
    assert getattr(pickle.loads(pickle.dumps(obj)), export)() == original
    json.dumps(original, allow_nan=False)


def _child_snapshot(queue, snapshot):
    queue.put((snapshot.canonical_payload(), snapshot.canonical_sha256()))


def test_frozen_snapshot_crosses_real_spawn_boundary():
    expected = _snapshot_payload()
    snapshot = FrozenMeasurementPairSnapshot.from_mapping(expected)
    context = multiprocessing.get_context("spawn")
    queue = context.Queue()
    child = context.Process(target=_child_snapshot, args=(queue, snapshot))
    try:
        child.start()
        payload, digest = queue.get(timeout=10)
        child.join(timeout=10)
        assert child.exitcode == 0
        assert payload == expected
        assert digest == _wire_hash(expected)
    finally:
        if child.is_alive():
            child.terminate()
            child.join(timeout=5)
        queue.close()
        queue.join_thread()
