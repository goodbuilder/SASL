import csv
import json
import time

from PyQt5 import QtCore

from Workstation.workstation_core.measurement_artifacts import reserve_measurement_context
from Workstation.workstation_core.measurement_data_store import MeasurementDataStore
from Workstation.workstation_core.native_frame_store import AsyncNativeFrameStore
from Workstation.workstation_core.rtcm_pipeline import RtcmEvidenceContext, RtcmEvidenceRecord, append_rtcm_evidence
from Workstation.ulog_controller import ULogController
from Workstation.tests.test_native_frame_store import _image
from Workstation.tests.test_ulog_controller import _valid_ulog


def assert_mirror(path):
    expected = [json.loads(line) for line in path.read_text().splitlines()]
    with path.with_suffix(".csv").open(newline="") as handle:
        actual = [json.loads(row["record_json"]) for row in csv.DictReader(handle)]
    assert actual == expected
    return expected


def test_png_job_keeps_original_root_after_next_measurement(tmp_path):
    store = MeasurementDataStore(tmp_path, timestamped_filenames=True)
    a = reserve_measurement_context(tmp_path, "a", 1, 1)
    b = reserve_measurement_context(tmp_path, "b", 2, 2)
    store.register_measurement(a)
    writer = AsyncNativeFrameStore(tmp_path, timestamp_filenames=True,
        event_sink=lambda row, ctx: store.enqueue("native.frames", {**row, "measurement_id": ctx["measurement_id"]}))
    delivered = []
    writer.completed.connect(delivered.append, QtCore.Qt.DirectConnection)
    context = {**a.metadata(), "native_root": str(a.root), "native_events_path": str(a.stream_path("native_frames"))}
    metadata = {"native_width": 2, "native_height": 2, "timestamp_s": 1.0}
    scheduled = writer.enqueue_pair(request_id="pair-a", pair_id="pair-1", purpose="measurement",
        frames=(("left", _image(2, 2), metadata), ("right", _image(2, 2), metadata)), context=context)
    store.register_measurement(b)
    deadline = time.monotonic() + 3
    while not delivered and time.monotonic() < deadline:
        time.sleep(0.01)
    assert delivered and delivered[0]["fields"]["evidence_state"] == "SAVED"
    writer.acknowledge("pair-a")
    assert writer.close()
    assert store.close(timeout_s=3)
    images = list(a.root.rglob("*.png"))
    assert len(images) == 2 and all(a.file_stamp in p.name for p in images)
    assert not list(b.root.rglob("*.png"))
    assert len(assert_mirror(a.stream_path("native_frames"))) == 2


def test_rtcm_binary_and_both_index_formats_share_start_stamp(tmp_path):
    context = reserve_measurement_context(tmp_path, "rtcm-run", 1, 1)
    rtcm = RtcmEvidenceContext(state="COLLECTING", session_id="measurement-data", session_root=context.root,
                               context_token=context.measurement_id, file_stamp=context.file_stamp)
    record = RtcmEvidenceRecord(sequence=1, source_sequence=1, source="ntrip", record_kind="received_bytes",
        payload=b"\xd3\x00\x01\x11", receive_utc_ns=20, receive_monotonic_ns=30)
    saved = append_rtcm_evidence(rtcm, record)
    from pathlib import Path
    assert Path(saved["binary_path"]).read_bytes() == record.payload
    assert context.file_stamp in Path(saved["binary_path"]).name
    rows = assert_mirror(Path(saved["index_path"]))
    assert rows[0]["measurement_id"] == "rtcm-run"
    assert rows[0]["offset_bytes"] == 0 and rows[0]["size_bytes"] == 4


def test_ulog_and_metadata_are_owned_by_measurement(tmp_path):
    source = tmp_path / "source.ulg"
    source.write_bytes(_valid_ulog())
    store = MeasurementDataStore(tmp_path, timestamped_filenames=True)
    context = reserve_measurement_context(tmp_path, "ulog-run", 1, 1)
    store.register_measurement(context)
    controller = ULogController(tmp_path, "measurement-data", timestamp_filenames=True,
        storage_context_provider=lambda: {**context.metadata(), "root": str(context.root)},
        artifact_sink=lambda row: store.enqueue("artifacts", row, priority=True))
    controller.import_manual_ulog("left", source)
    assert store.close(timeout_s=3)
    logs = list(context.root.rglob("*.ulg"))
    assert len(logs) == 1 and logs[0].read_bytes() == source.read_bytes()
    assert context.file_stamp in logs[0].name
    assert all(context.file_stamp in p.name for p in context.root.rglob("*") if p.is_file())
    assert assert_mirror(context.stream_path("artifacts"))[0]["measurement_id"] == "ulog-run"


def test_rtcm_csv_repair_keeps_binary_and_does_not_duplicate_next_append(tmp_path):
    from pathlib import Path
    from dataclasses import replace
    from Workstation.workstation_core.measurement_csv_mirror import repair_owned_jsonl_mirror
    context = reserve_measurement_context(tmp_path, "repair-run", 1, 1)
    rtcm = RtcmEvidenceContext(state="COLLECTING", session_id="measurement-data", session_root=context.root,
                               context_token=context.measurement_id, file_stamp=context.file_stamp)
    first = RtcmEvidenceRecord(sequence=1, source_sequence=1, source="ntrip", record_kind="received_bytes",
        payload=b"first", receive_utc_ns=20, receive_monotonic_ns=30)
    saved = append_rtcm_evidence(rtcm, first)
    index = Path(saved["index_path"])
    mirror = index.with_suffix(".csv")
    header = mirror.read_text().splitlines()[0]
    mirror.write_text(header + "\n")  # valid header, missing previously written row
    repair_owned_jsonl_mirror(index)
    append_rtcm_evidence(rtcm, replace(first, sequence=2, source_sequence=2, payload=b"second"))
    assert Path(saved["binary_path"]).read_bytes() == b"firstsecond"
    assert [row["sequence"] for row in assert_mirror(index)] == [1, 2]
