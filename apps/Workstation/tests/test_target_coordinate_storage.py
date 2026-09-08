from dataclasses import replace
import json
from types import SimpleNamespace

import pytest

from Workstation.tests.measurement_fixtures import MEASUREMENT_ID, pixhawk_attempt, frozen_snapshot, outcome_for
from Workstation.workstation_core.target_coordinates import TARGET_COORDINATE_FIELDS
from Workstation.workstation_core.target_level_coordinates import LEVEL_COORDINATE_FIELDS, LEVEL_OUTPUT_FIELDS
from Workstation.workstation_core.method_result_evidence import derive_method_result
from Workstation.workstation_core.measurement_record import MeasurementRegistry, METHOD_SHEET_NAMES
from Workstation.workstation_core.measurement_data_store import MeasurementDataStore
from Workstation.workstation_core.measurement_journal import load_measurement_record
from Workstation.workstation_core.measurement_workbook import write_measurement_workbook
from Workstation.workstation_core.measurement_xlsx_read import read_workbook, table_rows
from Workstation.workstation_core.measurement_reproduce import reproduce_target_coordinates_exact


def georeferenced_attempt():
    a = pixhawk_attempt()
    return replace(a, left_gps=replace(a.left_gps, altitude_ellipsoid_m=130.))


def test_worker_journal_workbook_and_exact_replay(tmp_path, monkeypatch):
    store = MeasurementDataStore(tmp_path, timestamped_filenames=True)
    registry = MeasurementRegistry(data_root_provider=lambda: tmp_path, per_run_storage=True,
        context_started=store.register_measurement, context_stopped=store.finish_measurement,
        journal_sink=lambda payload: store.enqueue("measurement.journal", payload, priority=True))
    a = georeferenced_attempt()
    snapshot = frozen_snapshot(a)
    record = registry.begin(MEASUREMENT_ID, started_utc_ns=1_000_000_000, started_monotonic_ns=10)
    record.register_pair(snapshot.canonical_payload(), submitted_method_ids=(a.measurement_method_id,), not_attempted={})
    outcome = outcome_for(a, snapshot)
    values = derive_method_result(outcome.to_record())
    assert values["target_latitude_deg"] > 37
    assert values["target_altitude_msl_m"] is None
    assert values["target_level_status"] == "AVAILABLE"
    assert values["target_level_range_m"] == pytest.approx(values["range_m"], abs=1e-10)
    assert values["target_level_evidence"]["attitude_timestamp_s"] == a.left_attitude.app_receive_monotonic_s
    registry.route_outcome(outcome)
    registry.stop(MEASUREMENT_ID, stopped_utc_ns=9_000_000_000, stopped_monotonic_ns=9_000_000_000)
    assert store.close(timeout_s=3)
    path = write_measurement_workbook(record.snapshot(), record.workbook_path)
    sheet = read_workbook(path)
    row = next(r for r in table_rows(sheet["90_Flat"]) if r.get("method_id") == a.measurement_method_id)
    for field in (*TARGET_COORDINATE_FIELDS[:3], *LEVEL_COORDINATE_FIELDS):
        assert row[field] == pytest.approx(values[field], abs=1e-10)
    for sheet_name in ("02_좌표결과", METHOD_SHEET_NAMES[a.measurement_method_id]):
        grid = sheet[sheet_name]
        header = next(i for i, cells in enumerate(grid) if "target_level_forward_m" in cells)
        displayed = table_rows(grid, header)[0]
        for field in LEVEL_COORDINATE_FIELDS:
            assert displayed[field] == pytest.approx(values[field], abs=1e-10)
        assert displayed["target_level_status"] == "AVAILABLE"
    replay = reproduce_target_coordinates_exact(path)
    assert len(replay) == 1
    assert not replay[0]["reason"]
    assert replay[0]["stored"] == replay[0]["reproduced"]
    import Workstation.workstation_core.measurement_record as module
    monkeypatch.setattr(module, "derive_method_result", lambda *_: (_ for _ in ()).throw(AssertionError("unexpected recalculation")))
    rebuilt = load_measurement_record(registry.context_for(MEASUREMENT_ID).stream_path("measurement_journal"))
    assert rebuilt.snapshot().method_results[0].derived == json.loads(json.dumps(record.snapshot().method_results[0].derived))
    rebuilt_path = tmp_path / "rebuilt.xlsx"
    write_measurement_workbook(rebuilt.snapshot(), rebuilt_path)
    assert reproduce_target_coordinates_exact(rebuilt_path) == replay


def test_primary_retry_and_canonical_result_keep_coordinates():
    from Workstation.recording_controller import RecordingController
    a = georeferenced_attempt()
    outcome = outcome_for(a, frozen_snapshot(a))
    result = outcome.to_record()["result"]
    geometry = result["triangulated"]
    panel = SimpleNamespace(recording_session_id="session", _measurement_id=MEASUREMENT_ID)
    owner = RecordingController(panel)
    row = owner._compact_primary_retry_row(estimate=SimpleNamespace(warnings=(), sync_quality="GOOD"),
        distance_m=geometry["range_m"], method=a.measurement_method_id, baseline_source=a.baseline_source,
        baseline_m=a.baseline_m, point_m=geometry["point_m"], raw_point_m=geometry["point_m"],
        raw_point_left_m=geometry["source_point_left_rdf_m"], measurement_attempt=a, outcome_metadata={})
    canonical = json.loads(row["measurement_result_canonical_json"])
    for field in (*TARGET_COORDINATE_FIELDS, *LEVEL_OUTPUT_FIELDS):
        assert row[field] == result[field] == canonical[field]


def test_legacy_outcome_fields_stay_unknown():
    a = georeferenced_attempt()
    record = outcome_for(a, frozen_snapshot(a)).to_record()
    for field in (*TARGET_COORDINATE_FIELDS, *LEVEL_OUTPUT_FIELDS):
        record["result"].pop(field)
    values = derive_method_result(record)
    assert all(values[field] is None for field in (*TARGET_COORDINATE_FIELDS, *LEVEL_OUTPUT_FIELDS))


def test_jsonl_csv_compact_and_parquet_preserve_target(tmp_path):
    import csv
    import pyarrow.parquet as pq
    from Workstation.workstation_core.recording_session import compact_distance_measurement_rows
    from Workstation.workstation_core.session_parquet import finalize_measurement_pairs
    a = georeferenced_attempt()
    data = derive_method_result(outcome_for(a, frozen_snapshot(a)).to_record())
    row = {name: data[name] for name in (*TARGET_COORDINATE_FIELDS, *LEVEL_OUTPUT_FIELDS)}
    row.update(pair_id="target-pair", sample_index=0, distance_m=12.5)
    store = MeasurementDataStore(tmp_path)
    assert store.enqueue("measurements", row)
    assert store.close(timeout_s=3)
    saved = json.loads((tmp_path/"measurements.jsonl").read_text().splitlines()[0])
    with (tmp_path/"measurements.csv").open() as handle:
        mirror = next(csv.DictReader(handle))
    payloads = [json.loads(v) for v in mirror.values() if v and v.startswith('{')]
    assert any(all(p.get(k) == row[k] for k in (*TARGET_COORDINATE_FIELDS, *LEVEL_OUTPUT_FIELDS)) for p in payloads)
    compact = compact_distance_measurement_rows([saved], {})
    path = tmp_path/"measurement.parquet"
    finalize_measurement_pairs(path, compact)
    projected = pq.read_table(path).to_pylist()[0]
    for name in TARGET_COORDINATE_FIELDS:
        assert projected[name] == row[name]


def test_thin_and_flat_method_records_keep_level_fields():
    from Workstation.measurement_view_controller import MeasurementViewController
    a = georeferenced_attempt()
    outcome = outcome_for(a, frozen_snapshot(a))
    bundle = SimpleNamespace(outcomes=(outcome,), to_record=lambda: {}, outcome_for=lambda _: None)
    controller = MeasurementViewController(SimpleNamespace())
    thin = controller._method_result_row(bundle, outcome)
    flat = controller._bundle_flat_fields(bundle)
    prefix = 'm_' + controller._method_row_prefix(a.measurement_method_id) + '_'
    for field in LEVEL_OUTPUT_FIELDS:
        assert thin[field] == outcome.result[field]
        assert flat[prefix + field] == outcome.result[field]
