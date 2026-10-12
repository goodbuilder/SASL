"""Streaming save contracts: raw bytes, one-shot inputs, recovery and review."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile

import pytest

from Workstation.workstation_core import session_review
from Workstation.workstation_core.geometry_context import materialize_geometry_contexts
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.method_evaluation import MethodEvaluationBundle, MethodEvaluationOutcome
from Workstation.workstation_core.recording_reader import iter_measurement_stream, read_measurement_stream
from Workstation.workstation_core.session_parquet import read_lossless_rows
from Workstation.workstation_core.session_storage import canonical, generation, projection
from Workstation.workstation_core.session_storage.disk_index import DiskRowTable, disk_index
from Workstation.workstation_core.session_storage.finalize import save_session_files
from Workstation.workstation_core.session_storage.indexed_records import FinalizationIndexes
from Workstation.workstation_core.session_storage.spool import SpoolRows
from Workstation.workstation_core.time_sync_epoch import materialize_time_sync_epochs


class OnePass:
    def __init__(self, rows):
        self.rows = rows
        self.iterations = 0

    def __iter__(self):
        self.iterations += 1
        assert self.iterations == 1, "source was replayed instead of canonical disk rows"
        yield from self.rows


def _bundle(index):
    fields = dict(measurement_id="m000001", pair_id=f"pair-{index}", pair_sequence=index,
                  snapshot_sha256="1" * 64, request_generation=7)
    return MethodEvaluationBundle(**fields, outcomes=tuple(
        MethodEvaluationOutcome(**fields, method_id=method, method_instance_id="", status="BLOCKED", reasons=("offline fixture",))
        for method in MEASUREMENT_METHOD_IDS
    )).to_record()


def test_one_pass_sources_preserve_raw_golden_axes_nulls_dynamic_fields_and_method_hashes(tmp_path):
    root = tmp_path / "s" / "measurements" / "m000001"
    raw = {
        "pair_id": "golden", "baseline_center_forward_m": 17.0,
        "baseline_center_right_m": -3.0, "baseline_center_down_m": 5.0,
        "source_point_left_rdf_m": [2.0, 7.0, 19.0],
        "world_ned_m": [41.0, -13.0, 23.0], "nullable": None,
        "dynamic": {"정확한 값": [True, 9007199254740993]},
    }
    pair = OnePass((raw, {"pair_id": "late", "late_only": "retained"}))
    cameras = {side: OnePass(({"sequence": 0, "source_utc_ns": 9007199254740993},)) for side in ("left", "right")}
    px4 = {side: OnePass(({"sequence": 0, "velocity_n_m_s": 2.0, "velocity_e_m_s": -5.0, "velocity_d_m_s": 7.0},)) for side in ("right", "left", "drone")}
    bundles = OnePass((_bundle(0), _bundle(1)))
    saved = save_session_files(root, metadata={"session_id": "s", "measurement_id": "m000001"},
                               pair_rows=pair, camera_rows=cameras, px4_rows=px4,
                               method_evaluation_bundle_rows=bundles)
    reopened = read_measurement_stream(root).rows
    # This independent golden does not call the canonical identity/context writer.
    expected = {**raw, "sample_index": 0, "session_elapsed_s": None,
                "measurement_id": "m000001", "measurement_package_id": "m000001"}
    # Terminal identity excludes positional sample_index/pair_sequence.
    identity_payload = {key: value for key, value in expected.items() if key != "sample_index"}
    digest = hashlib.sha256(json.dumps(identity_payload, ensure_ascii=False, sort_keys=True,
                                      separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    expected["terminal_record_id"] = "legacy:" + digest
    context = {"schema_version": "workstation-geometry-context-v1", "context_kind": "legacy_terminal_static",
               "available_static_fields": {}, "dynamic_values_retained_only_in_terminal_row": True,
               "public_output_contract": {"frame": "baseline_center_frd", "axis_contract": "X=Front,Y=Right,Z=Down", "unit": "m"}}
    gc = hashlib.sha256(json.dumps(context, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    expected.update(geometry_context_id="gc:" + gc, left_time_sync_epoch_id=None,
                    right_time_sync_epoch_id=None, time_sync_epoch_id=None)
    assert reopened[0] == expected
    expected_bytes = (json.dumps(expected, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n").encode()
    assert saved.authoritative_jsonl_files[0].read_bytes().splitlines(keepends=True)[0] == expected_bytes
    assert reopened[1]["late_only"] == "retained"
    assert "sample_index" not in raw and "measurement_id" not in raw
    assert pair.iterations == bundles.iterations == 1
    assert all(rows.iterations == 1 for rows in (*cameras.values(), *px4.values()))
    parquet = read_lossless_rows(saved.measurement_pairs_parquet, "measurement_pairs")
    assert parquet == list(reopened)
    for method in MEASUREMENT_METHOD_IDS:
        method_rows = list(iter_measurement_stream(root, f"raw/method_results/{method}"))
        assert [row["pair_id"] for row in method_rows] == ["pair-0", "pair-1"]
        assert method_rows[0]["method_evaluation_bundle_sha256"] == _bundle(0)["method_evaluation_bundle_sha256"]
    manifest = json.loads(saved.manifest.read_text())
    assert "late_only" in manifest["artifact_details"]["raw_stereo_measurements"]["fields"]
    assert not list(root.rglob(".row-index-*"))


def test_disk_epoch_and_context_indexes_match_existing_materializers(tmp_path):
    def camera(ns, *, ssrc=101, generation=2, **extra):
        return {"host_receive_monotonic_ns": ns, "source_utc_ns": 1700000000000000123,
                "rtp_ssrc": ssrc, "stream_generation": generation, "rtcp_sr_received_count": 2, **extra}
    cameras = {"left": [camera(10), camera(9), {"sequence": 3}, camera(20, clock_epoch_state="transition"),
                        camera(30), camera(40, ssrc=102), camera(50, generation=3, ssrc=102)],
               "right": [camera(10), camera(40), camera(60)]}
    pairs = [{"pair_id": str(ns), "left_app_receive_monotonic_ns": ns,
              "right_app_receive_monotonic_ns": ns, "left_rtp_ssrc": ssrc,
              "right_rtp_ssrc": 101, "intrinsics_profile_sha256": str(ns % 3)}
             for ns, ssrc in ((10, 101), (20, 101), (30, 101), (40, 102), (50, 102), (61, 101))]
    metadata = {"session_started_unix_ns": 9007199254740993, "session_started_monotonic_ns": 71,
                "pc_utc_sync": {"command": "STUS", "result": "success"}}
    expected_cameras = copy.deepcopy(cameras)
    expected_pairs = projection._normalized_pair_rows(pairs)
    for rows in (*expected_cameras.values(), expected_pairs):
        canonical._apply_measurement_package_identity(rows, measurement_id="m000001")
    canonical._apply_terminal_record_identities(expected_pairs)
    expected_contexts = materialize_geometry_contexts(expected_pairs)
    expected_epochs = materialize_time_sync_epochs(camera_rows=expected_cameras, terminal_rows=expected_pairs,
                                                   metadata=metadata, session_id="s", measurement_package_id="m000001")
    with disk_index(tmp_path) as connection:
        indexes = FinalizationIndexes(connection, metadata=metadata, session_id="s", measurement_id="m000001")
        actual_cameras = {side: list(indexes.camera_rows(side, OnePass(rows))) for side, rows in cameras.items()}
        actual_pairs = list(indexes.pair_rows(OnePass(pairs)))
        assert actual_cameras == expected_cameras
        assert actual_pairs == expected_pairs
        assert list(indexes.contexts) == expected_contexts
        assert list(indexes.epochs) == expected_epochs
    assert actual_cameras["left"][1]["time_sync_epoch_id"] is None
    assert actual_pairs[-1]["right_time_sync_epoch_id"] is None


@pytest.mark.parametrize("fault", ["staged_bytes", "published_readback"])
def test_streaming_failure_preserves_previous_pointer_retry_spool_and_other_run(tmp_path, monkeypatch, fault):
    root = tmp_path / "s" / "measurements" / "m000001"
    metadata = {"session_id": "s", "measurement_id": "m000001"}
    old = save_session_files(root, metadata=metadata, pair_rows=[{"pair_id": "old"}])
    pointer = root / ".workstation/publish/current"
    old_target, old_bytes = pointer.readlink(), old.authoritative_jsonl_files[0].read_bytes()
    spool = SpoolRows(supports_updates=True)
    spool.bind(root / ".spool/pair_rows.jsonl")
    spool.append({"pair_id": "new", "dynamic": "keep"})
    spool.close()
    other = SpoolRows(supports_updates=True)
    other.bind(root.parent / "m000002/.spool/pair_rows.jsonl")
    other.append({"pair_id": "active-other"})
    other.flush()
    other_bytes = other.spool_path.read_bytes()
    try:
        with monkeypatch.context() as patch:
            if fault == "staged_bytes":
                original = canonical._write_authoritative_jsonl
                def corrupt(path, rows):
                    receipt = original(path, rows)
                    if path.name == "stereo_measurements.jsonl":
                        path.write_bytes(path.read_bytes().replace(b'"new"', b'"bad"'))
                    return receipt
                patch.setattr(canonical, "_write_authoritative_jsonl", corrupt)
            else:
                def fail(*args, **kwargs):
                    raise OSError("injected published readback")
                patch.setattr(generation, "_validate_published_v2_generation", fail)
            with pytest.raises(OSError):
                save_session_files(root, metadata=metadata, pair_rows=spool)
        assert pointer.readlink() == old_target
        assert old.authoritative_jsonl_files[0].read_bytes() == old_bytes
        assert spool.spool_path.is_file()
        assert list(spool) == [{"pair_id": "new", "dynamic": "keep"}]
        assert other.spool_path.read_bytes() == other_bytes
        assert not list(root.rglob(".row-index-*"))
        saved = save_session_files(root, metadata=metadata, pair_rows=spool)
        assert saved.finalized and spool.spool_path is None
        assert read_measurement_stream(root).rows[0]["pair_id"] == "new"
        assert other.spool_path.read_bytes() == other_bytes
    finally:
        other.close()
        spool.close()


def test_mutable_spool_replay_keeps_order_tombstones_and_volatile_recovery(tmp_path, monkeypatch):
    spool = SpoolRows(preview_limit=1, supports_updates=True)
    path = tmp_path / "spool.jsonl"
    spool.bind(path)
    for index in range(8):
        spool.append({"index": index})
    spool.replace(2, {"index": 20})
    spool.pop(1)
    spool.append({"index": 8})
    spool.close()
    spool.replace(0, {"index": 100})
    spool.append({"index": 9})
    spool.pop(2)
    # The materializing compatibility API must never be used for save/recovery.
    monkeypatch.setattr(spool, "_materialized_rows", lambda: pytest.fail("eager replay"))
    assert [row["index"] for row in spool] == [100, 20, 4, 5, 6, 7, 8, 9]
    assert path.exists() and not list(tmp_path.glob(".row-index-*"))


def test_bounded_review_matches_eager_collector_xml_and_shard_columns(tmp_path):
    raw = {"measurement_id": "m000001", "results": [
        {"pair_id": f"p{i}", "measurement_method_id": MEASUREMENT_METHOD_IDS[0], "distance_m": i + .25,
         "utc_time": 9007199254740993 + i, **({"late_dynamic": "last"} if i == 5 else {})} for i in range(6)],
        "quality": [{"pair_id": "p0", "status": "old"}, {"pair_id": "p0", "status": "new"}],
        "geometry": [{"method_id": MEASUREMENT_METHOD_IDS[0], "status": "READY"}]}
    fixed = "2026-01-02T03:04:05+00:00"
    measurement = session_review._normalize_measurement(raw)
    reference = tmp_path / "reference.xlsx"
    session_review._write_collected_workbook(reference, session_root=tmp_path, session_id="s", session_summary={},
        measurements=[measurement], generated_at_utc=fixed, max_rows_per_sheet=4,
        rows_by_sheet=session_review._collect_rows([measurement]),
        rows_by_method=session_review._collect_method_rows([measurement]))
    with disk_index(tmp_path) as connection:
        rows = DiskRowTable(connection, "results")
        rows.extend(raw["results"])
        bounded = {**raw, "results": session_review.ReplayableReviewRows(rows)}
        actual = session_review.write_session_review_xlsx(tmp_path, session_id="s", measurements=[bounded],
                                                          generated_at_utc=fixed, max_rows_per_sheet=4)
    with ZipFile(reference) as expected_zip, ZipFile(actual) as actual_zip:
        members = [name for name in expected_zip.namelist() if name.startswith("xl/") and name.endswith(".xml")]
        assert members
        for member in members:
            assert actual_zip.read(member) == expected_zip.read(member), member
    assert not list(tmp_path.glob(".row-index-*"))


def test_recovery_reads_only_tail_and_streams_updated_spool(tmp_path, monkeypatch):
    path = tmp_path / "pair_rows.jsonl"
    path.write_bytes(b'{"index": 0}\n{"index": 1}\n{"__spool_update_index": 0, "__spool_row": {"index": 5}}\n{"torn":')
    # Whole-file reads used to duplicate a full recovered recording before
    # checking whether its final record even needed repair.
    monkeypatch.setattr(Path, "read_bytes", lambda path: pytest.fail("whole-spool read"))
    rows = SpoolRows(preview_limit=1, supports_updates=True)
    try:
        rows.open_existing(path)
        assert list(rows) == [{"index": 5}, {"index": 1}]
        assert rows.preview_count == 1
        assert len(rows.recovery_artifacts) == 1
        with rows.recovery_artifacts[0].open("rb") as handle:
            assert handle.read() == b'{"torn":'
    finally:
        rows.close()


@pytest.mark.parametrize("indices, message", [([1, 1], "duplicate method evaluation bundle"), ([1, 0], "strictly increasing pair_sequence")])
def test_disk_method_ledger_preserves_duplicate_and_sequence_rejection(tmp_path, indices, message):
    with disk_index(tmp_path) as connection:
        indexes = FinalizationIndexes(connection, metadata={}, session_id="s", measurement_id="m000001")
        tables = {method: DiskRowTable(connection, f"method_{index}") for index, method in enumerate(MEASUREMENT_METHOD_IDS)}
        with pytest.raises(ValueError, match=message):
            list(indexes.method_bundle_rows((_bundle(index) for index in indices), tables))
    assert not list(tmp_path.glob(".row-index-*"))


def test_disk_epoch_lookup_retains_python_integer_precision_above_sqlite_int64(tmp_path):
    exact_ns = (1 << 80) + 17
    raw = {"host_receive_monotonic_ns": exact_ns, "source_utc_ns": 1700000000000000123,
           "rtp_ssrc": 101, "stream_generation": 2, "rtcp_sr_received_count": 1}
    expected_camera = [copy.deepcopy(raw)]
    expected_pair = [{"left_app_receive_monotonic_ns": exact_ns, "left_rtp_ssrc": 101}]
    materialize_time_sync_epochs(camera_rows={"left": expected_camera}, terminal_rows=expected_pair,
                                 metadata={}, session_id="s", measurement_package_id="m000001")
    with disk_index(tmp_path) as connection:
        indexes = FinalizationIndexes(connection, metadata={}, session_id="s", measurement_id="m000001")
        actual_camera = list(indexes.camera_rows("left", (raw,)))
        [actual_pair] = list(indexes.pair_rows(({"left_app_receive_monotonic_ns": exact_ns, "left_rtp_ssrc": 101},)))
        assert actual_camera[0]["time_sync_epoch_id"] == expected_camera[0]["time_sync_epoch_id"]
        assert actual_pair["left_time_sync_epoch_id"] == expected_pair[0]["left_time_sync_epoch_id"]
        assert actual_pair["left_time_sync_epoch_id"] is not None
