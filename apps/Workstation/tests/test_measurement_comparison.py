from PyQt5 import QtWidgets
from dataclasses import replace

from Workstation.measurement_comparison import PairCursor, MeasurementComparisonPane
from Workstation.workstation_core.measurement_record import MeasurementRegistry
from Workstation.tests.measurement_fixtures import STARTED_LOCAL, checkerboard_attempt, frozen_snapshot, outcome_for


def registry_at(tmp_path):
    registry = MeasurementRegistry(data_root_provider=lambda: tmp_path, clock_local=lambda: STARTED_LOCAL)
    record = registry.begin("run-abc123", started_utc_ns=1000000000, started_monotonic_ns=1000000000)
    return registry, record


def pair(record, sequence=1, methods=("checkerboard_stereo",), not_attempted=None):
    attempt = checkerboard_attempt(sequence)
    snapshot = frozen_snapshot(attempt)
    record.register_pair(snapshot.canonical_payload(), submitted_method_ids=methods,
                         not_attempted=not_attempted or {})
    return attempt, snapshot


def test_snapshot_participation_late_outcome_and_run_isolation(tmp_path):
    registry, record = registry_at(tmp_path)
    a, p = pair(record)
    view = record.results_snapshot()
    assert len(view["rows"]) == 1
    assert view["rows"][0]["status"] == "PENDING"
    assert not view["all_terminal"]
    b, q = pair(record, 2, methods=(), not_attempted={"checkerboard_stereo": "K,D missing"})
    assert record.results_snapshot()["rows"][0]["status"] == "BLOCKED"
    registry.route_outcome(outcome_for(a, p))
    old = record.results_snapshot(p.pair_id)
    assert old["all_terminal"]
    assert old["rows"][0]["derived"]["range_m"] > 0
    old["rows"][0]["derived"]["range_m"] = -123
    assert record.results_snapshot(p.pair_id)["rows"][0]["derived"]["range_m"] > 0
    registry.stop(record.measurement_id, stopped_utc_ns=2000000000, stopped_monotonic_ns=2000000000)
    assert registry.results_snapshot()["measurement_id"] == record.measurement_id
    second = registry.begin("next-run", started_utc_ns=3000000000, started_monotonic_ns=3000000000)
    registry.route_outcome(outcome_for(a, p))
    assert registry.results_snapshot(record.measurement_id, p.pair_id)["measurement_id"] == second.measurement_id
    assert registry.results_snapshot()["rows"] == []


def test_cursor_one_second_is_display_only_and_skips_backlog():
    cursor = PairCursor()
    first = {"measurement_id": "run1", "latest_pair_id": "p1"}
    assert cursor.choose(first, 5.) == "p1"
    latest = {**first, "latest_pair_id": "p50"}
    assert cursor.choose(latest, 5.999) == "p1"
    assert cursor.choose(latest, 6.) == "p50"
    assert cursor.choose(latest, 9.) == "p50"
    assert cursor.choose({"measurement_id": "run2", "latest_pair_id": ""}, 10.) == ""
    assert cursor.measurement_id == "run2"


def test_cursor_terminal_advances_early():
    cursor = PairCursor()
    cursor.choose({"measurement_id": "r", "latest_pair_id": "1"}, 0.)
    cursor.shown_terminal = True
    assert cursor.choose({"measurement_id": "r", "latest_pair_id": "2"}, .25) == "2"


def test_empty_reader_does_not_start_storage_or_writer():
    from types import SimpleNamespace
    from Workstation.data_storage_controller import DataStorageController
    owner = DataStorageController(SimpleNamespace())
    assert owner.measurement_results_snapshot()["rows"] == []
    assert owner._store is None
    assert owner._workbook_writer is None


def test_pane_coordinate_switch_partial_results_and_error_do_not_use_history(tmp_path):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    registry, record = registry_at(tmp_path)
    a, p = pair(record, methods=("checkerboard_stereo", "cloud_gnss"))
    outcome = outcome_for(a, p)
    payload = outcome.to_record()
    payload["recorded_geometry"] = {
        "range_m": 100., "forward_m": 99., "right_m": 2., "down_m": 3., "ray_gap_m": .1,
        "target_level_range_m": 100., "target_level_forward_m": 98., "target_level_right_m": 4., "target_level_down_m": 5.,
        "target_level_status": "AVAILABLE", "target_latitude_deg": 37.123456789,
        "target_longitude_deg": 127.123456789, "target_altitude_ellipsoid_m": 42.,
    }
    record.note_outcome(payload)
    now = [10.]
    pane = MeasurementComparisonPane(registry.results_snapshot, clock=lambda: now[0])
    pane.refresh_results()
    assert pane.model.rowCount() == 2
    assert pane.model.item(0, 3).text() == "100.000"
    assert pane.model.item(1, 1).text() == "PENDING"
    assert pane.model.item(1, 3).text() == "—"
    pane.coordinate_combo.setCurrentIndex(1)
    assert pane.model.item(0, 4).text() == "98.000"
    pane.coordinate_combo.setCurrentIndex(2)
    assert pane.model.item(0, 3).text() == "37.123456789"
    assert pane.model.item(0, 2).text() == "PARTIAL"
    assert "MSL 없음" in pane.model.item(0, 8).text()
    pane.table.selectRow(1)
    assert "cloud" in pane._selected_method
    assert "PENDING" in pane.details.toPlainText()
    selected_before = record.selected_method_id
    b, q = pair(record, 2, methods=())
    now[0] = 11.
    pane.refresh_results()
    assert pane.snapshot["pair_id"] == q.pair_id
    assert pane.model.item(0, 3).text() == "—"
    assert record.selected_method_id == selected_before
    cloud_before = next(r for r in record.results_snapshot(p.pair_id)["rows"] if r["method_id"] == "cloud_gnss")
    assert cloud_before["status"] == "PENDING"  # Display deadline did not time out the job.
    registry.route_outcome(outcome_for(replace(a, measurement_method_id="cloud_gnss"), p))
    pane.refresh_results()
    assert pane.snapshot["pair_id"] == q.pair_id
    assert pane.model.item(1, 3).text() == "—"
    assert next(r for r in record.results_snapshot(p.pair_id)["rows"] if r["method_id"] == "cloud_gnss")["status"] in {"VALID", "ESTIMATED"}
    pane.reader = lambda *_: (_ for _ in ()).throw(RuntimeError("reader failed"))
    pane.refresh_results()
    assert pane.model.rowCount() == 0
    assert "조회 실패" in pane.header.text()
    pane.close()


def test_hidden_timer_selection_and_run_reset(tmp_path):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    registry, record = registry_at(tmp_path)
    a, p = pair(record)
    registry.route_outcome(outcome_for(a, p))
    pane = MeasurementComparisonPane(registry.results_snapshot)
    assert not pane.timer.isActive()
    pane.resize(1100, 760)
    pane.show()
    app.processEvents()
    assert pane.timer.isActive()
    assert pane.model.item(0, 3).text() != "—"
    pane.table.horizontalScrollBar().setValue(90)
    scroll = pane.table.horizontalScrollBar().value()
    selected = pane._selected_method
    pane.refresh_results()
    assert pane._selected_method == selected
    assert pane.table.horizontalScrollBar().value() == scroll
    pane.grab().save(str(tmp_path / "method-comparison.png"))
    pane.hide()
    assert not pane.timer.isActive()
    registry.begin("new-run", started_utc_ns=2000000000, started_monotonic_ns=2000000000)
    pane.refresh_results()
    assert pane.snapshot["measurement_id"] == "new-run"
    assert pane.model.rowCount() == 0
    assert pane._selected_method == ""
    pane.close()
