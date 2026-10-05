from types import SimpleNamespace

from Workstation.data_storage_controller import DataStorageController
from Workstation.workstation_core.measurement_workbook import write_measurement_workbook
from Workstation.workstation_core.measurement_xlsx_read import read_workbook, table_rows


def test_latest_and_rebuild_choose_collision_successor_and_saved_input(tmp_path):
    panel = SimpleNamespace(app_root=tmp_path, default_measurement_data_root=tmp_path)
    controller = DataStorageController(panel)
    registry = controller._registry
    paths = []
    for identifier in ("a", "b"):
        record = registry.begin(identifier, started_utc_ns=1, started_monotonic_ns=1)
        registry.stop(identifier, stopped_utc_ns=2, stopped_monotonic_ns=2)
        paths.append(write_measurement_workbook(record.snapshot(), record.workbook_path))
    assert controller.store.close(timeout_s=3)
    assert "-01_measurement.xlsx" in paths[1].name
    assert controller.latest_measurement_workbook_path() == paths[1]
    paths[1].unlink()
    restarted = DataStorageController(panel)
    assert restarted.rebuild_latest_measurement_workbook()
    restarted._rebuild_thread.join(timeout=5)
    assert not restarted._rebuild_thread.is_alive()
    assert restarted._rebuild_error == ""
    assert paths[1].is_file()
    provenance = table_rows(read_workbook(paths[1])["98_Provenance"])
    assert next(row["value"] for row in provenance if row["key"] == "measurement_id") == "b"
    assert restarted.latest_measurement_workbook_path() == paths[1]


def test_unknown_late_artifact_never_uses_current_measurement(tmp_path):
    panel = SimpleNamespace(app_root=tmp_path, default_measurement_data_root=tmp_path)
    controller = DataStorageController(panel)
    controller._registry.begin("current", started_utc_ns=1, started_monotonic_ns=1)
    assert controller.allocate_measurement_artifact(label="late", suffix=".mp4", measurement_id="unknown") is None
    assert controller.store.close(timeout_s=3)
