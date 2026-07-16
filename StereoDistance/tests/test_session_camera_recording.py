from __future__ import annotations

import inspect
import queue
import shutil
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import pyarrow.parquet as pq
from PyQt5 import QtCore
import StereoDistance.wiris_stereo_distance.session_camera_recording as session_camera_module

from StereoDistance.wiris_stereo_distance.session_camera_recording import (
    B1VideoProbe,
    SessionCameraRecordingController,
    probe_b1_video,
)
from StereoDistance.wiris_stereo_distance.session_recording_parquet import (
    SessionCloseResult,
    SessionPaths,
    SessionRecordingConfig,
)
from StereoDistance.wiris_stereo_distance.session_video import MkvProbeResult
from StereoDistance.wiris_stereo_distance.yolo_stream import (
    _CanonicalGstRecordingSession,
    _gst_string_literal,
)


_QT_APP = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])


class _FakeRecorder:
    def __init__(self, config: SessionRecordingConfig) -> None:
        self.config = config
        self.paths = SessionPaths.from_config(config)
        self.state = "recording"
        self.sources = []
        self.alignments = []
        self.mappings = []
        self.artifacts = []
        self.calls = []
        self.mark_failures = []
        self.close_calls = 0
        self.thread_ids = set()

    def record_camera_source(self, record) -> None:
        self.thread_ids.add(threading.get_ident())
        self.calls.append(("source", record.camera_side, record.source_record_id))
        self.sources.append(record)

    def record_time_alignment(self, record) -> None:
        self.thread_ids.add(threading.get_ident())
        self.calls.append(("alignment", record.camera_side, record.record_id))
        self.alignments.append(record)

    def mark_video_mapping(self, mapping) -> None:
        self.thread_ids.add(threading.get_ident())
        self.calls.append(("mapping", mapping.camera_side, mapping.source_record_id))
        self.mappings.append(mapping)

    def register_video_artifact(self, artifact, *, probe=None) -> None:
        self.thread_ids.add(threading.get_ident())
        self.calls.append(("artifact", artifact.camera_side, artifact.writer_status))
        self.artifacts.append(artifact)

    def mark_failure(self, stage: str, message: str) -> None:
        self.thread_ids.add(threading.get_ident())
        self.calls.append(("mark_failure", stage, message))
        self.mark_failures.append((stage, message))
        self.state = "incomplete"

    def close(self) -> SessionCloseResult:
        self.thread_ids.add(threading.get_ident())
        self.calls.append(("close",))
        self.close_calls += 1
        complete = not self.mark_failures
        self.state = "complete" if complete else "incomplete"
        return SessionCloseResult(
            paths=self.paths,
            session_state=self.state,
            clean_shutdown=complete,
            incomplete=not complete,
            counts={
                "source_records": len(self.sources),
                "time_alignment": len(self.alignments),
                "pixhawk_samples": 0,
                "triangulation": 0,
            },
        )


class _BlockingRecorder(_FakeRecorder):
    def __init__(self, config: SessionRecordingConfig) -> None:
        super().__init__(config)
        self.entered = threading.Event()
        self.release = threading.Event()

    def record_camera_source(self, record) -> None:
        self.entered.set()
        assert self.release.wait(3.0)
        super().record_camera_source(record)


class _ExplodingMarkFailureRecorder(_FakeRecorder):
    def __init__(self, config: SessionRecordingConfig) -> None:
        super().__init__(config)
        self.mark_failure_attempts = 0

    def mark_failure(self, stage: str, message: str) -> None:
        self.mark_failure_attempts += 1
        raise RuntimeError("injected mark_failure crash")


class _IncompleteCloseRecorder(_FakeRecorder):
    def close(self) -> SessionCloseResult:
        self.thread_ids.add(threading.get_ident())
        self.calls.append(("close",))
        self.close_calls += 1
        self.state = "incomplete"
        return SessionCloseResult(
            paths=self.paths,
            session_state="incomplete",
            clean_shutdown=False,
            incomplete=True,
            counts={
                "source_records": len(self.sources),
                "time_alignment": len(self.alignments),
                "pixhawk_samples": 0,
                "triangulation": 0,
            },
        )


class _BlockingCloseRecorder(_FakeRecorder):
    def __init__(self, config: SessionRecordingConfig) -> None:
        super().__init__(config)
        self.close_entered = threading.Event()
        self.close_release = threading.Event()

    def close(self) -> SessionCloseResult:
        self.close_entered.set()
        assert self.close_release.wait(5.0)
        return super().close()


def _phase_a_probe(path: Path, *, include_packets: bool = True) -> MkvProbeResult:
    return MkvProbeResult(
        path=Path(path),
        format_name="matroska,webm",
        codec_name="h264",
        decoded_frame_count=1,
        packet_count=1,
        time_base_num=1,
        time_base_den=1_000,
    )


def _codec_probe(path: Path) -> B1VideoProbe:
    return B1VideoProbe("h264", 0, 1, 1)


def _write_h264_mkv(path: Path, *, frame_count: int, bframes: int) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        pytest.skip("ffmpeg is unavailable for the canonical artifact integration test")
    try:
        encoders = subprocess.run(
            [ffmpeg, "-v", "error", "-encoders"],
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        pytest.fail(f"available ffmpeg failed while listing encoders: {exc}")
    if "libx264" not in encoders.stdout:
        pytest.skip("ffmpeg is available but the required libx264 encoder is unavailable")
    command = [
        ffmpeg,
        "-v",
        "error",
        "-f",
        "lavfi",
        "-i",
        "testsrc=size=32x24:rate=10",
        "-frames:v",
        str(frame_count),
        "-c:v",
        "libx264",
        "-bf",
        str(bframes),
        "-g",
        str(max(1, frame_count)),
        "-pix_fmt",
        "yuv420p",
        "-f",
        "matroska",
        str(path),
    ]
    try:
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=30)
    except subprocess.CalledProcessError as exc:
        pytest.fail(f"available ffmpeg failed to generate H.264 MKV: {exc.stderr or exc}")
    except (OSError, subprocess.SubprocessError) as exc:
        pytest.fail(f"available ffmpeg failed to generate H.264 MKV: {exc}")


def _write_one_frame_mkv(path: Path) -> None:
    _write_h264_mkv(path, frame_count=1, bframes=0)


class _SyntheticTimingTracker:
    def __init__(self) -> None:
        self.sequence = 0

    def consume_authority_metadata(self, _buffer, *, now_monotonic_ns):
        assert isinstance(now_monotonic_ns, int)
        sequence = self.sequence
        self.sequence += 1
        return {
            "rtp_timestamp": 90_000 + sequence * 4_500,
            "rtp_ssrc": 42,
            "rtp_marker_seq": sequence,
            "rtcp_sr_ntp_raw": 1234,
            "rtcp_frame_ntp_unix_ns": 1_700_000_000_000_000_000 + sequence * 50_000_000,
        }


class _SyntheticControl:
    def __init__(self) -> None:
        self.payloads = []

    def send(self, payload) -> None:
        self.payloads.append(dict(payload))


class _BlockingCodecProbe:
    def __init__(self) -> None:
        self.entered = threading.Event()
        self.release = threading.Event()
        self.calls = 0

    def __call__(self, _path: Path) -> B1VideoProbe:
        self.calls += 1
        if self.calls == 1:
            self.entered.set()
            assert self.release.wait(5.0)
        return B1VideoProbe("h264", 0, 1, 1)


def _run_synthetic_canonical_gst_camera(Gst, *, session_id: str, side: str, output_path: Path, frame_count: int):
    pipeline_text = (
        f"videotestsrc num-buffers={frame_count} is-live=false "
        "! video/x-raw,format=I420,width=320,height=240,framerate=20/1 "
        "! x264enc bframes=0 key-int-max=30 tune=zerolatency byte-stream=true "
        "! h264parse name=au_parser config-interval=-1 disable-passthrough=true "
        "! video/x-h264,stream-format=avc,alignment=au "
        "! tee name=encoded "
        "encoded. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        "! fakesink sync=false "
        "encoded. ! valve name=record_valve drop=true "
        "! queue max-size-buffers=0 max-size-bytes=0 max-size-time=0 "
        "! identity name=mux_ingress silent=true "
        "! matroskamux "
        f"! filesink location={_gst_string_literal(str(output_path))} sync=false async=false"
    )
    try:
        pipeline = Gst.parse_launch(pipeline_text)
    except Exception as exc:
        pytest.fail(f"synthetic canonical GStreamer pipeline could not be parsed: {exc}")
    events = queue.Queue(maxsize=frame_count * 2 + 4)
    control = _SyntheticControl()
    session = _CanonicalGstRecordingSession(
        Gst,
        config={
            "session_id": session_id,
            "camera_side": side,
            "camera_stream_id": f"synthetic-{side}",
            "record_video_path": str(output_path),
        },
        rtp_timing_tracker=_SyntheticTimingTracker(),
        recording_data_queue=events,
        recording_control_conn=control,
        stop_event=threading.Event(),
        output_queue=SimpleNamespace(put=lambda _message: None),
    )
    session.attach(pipeline)
    terminal = None
    try:
        pipeline.set_state(Gst.State.PLAYING)
        message = pipeline.get_bus().timed_pop_filtered(
            10 * Gst.SECOND,
            Gst.MessageType.EOS | Gst.MessageType.ERROR,
        )
        if message is None:
            pytest.fail("synthetic canonical GStreamer pipeline timed out before EOS")
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            pytest.fail(f"synthetic canonical GStreamer pipeline failed: {error}: {debug}")
        terminal = session.send_terminal(eos_ok=True)
    finally:
        pipeline.set_state(Gst.State.NULL)

    payloads = []
    while True:
        try:
            payloads.append(events.get_nowait())
        except queue.Empty:
            break
    assert terminal is not None
    assert not [payload for payload in control.payloads if payload.get("control_kind") == "recording_failure"]
    return payloads, terminal


def _source(session_id: str, side: str, event_sequence: int = 0, frame: int = 0, **overrides):
    event = {
        "event_kind": "camera_source",
        "session_id": session_id,
        "camera_side": side,
        "record_event_sequence": event_sequence,
        "source_record_id": f"{session_id}:{side}:{frame}",
        "camera_stream_id": f"wiris-{side}",
        "frame_id": frame,
        "camera_sequence": frame,
        "pc_receive_monotonic_ns": 1_000_000 + frame,
        "pc_receive_utc_ns": 2_000_000 + frame,
        "rtp_timestamp": 90_000 + frame,
        "rtp_ssrc": 123,
        "rtp_marker_seq": frame,
        "rtcp_sr_ntp_raw": -1,
        "rtcp_frame_ntp_unix_ns": 3_000_000 + frame,
        "pts_ticks": frame + 1,
        "dts_ticks": frame + 1,
        "time_base_num": 1,
        "time_base_den": 1_000_000_000,
        "width_px": 1280,
        "height_px": 720,
        "video_path": f"camera/{side}_video.mkv",
    }
    event.update(overrides)
    return event


def _mapping(session_id: str, side: str, event_sequence: int = 1, frame: int = 0, **overrides):
    event = {
        "event_kind": "video_mapping",
        "session_id": session_id,
        "camera_side": side,
        "record_event_sequence": event_sequence,
        "source_record_id": f"{session_id}:{side}:{frame}",
        "source_frame_id": frame,
        "stored_in_video": True,
        "video_mapping_status": "stored",
        "video_frame_index": frame,
        "video_packet_pts_ticks": frame + 1,
        "video_packet_dts_ticks": frame + 1,
    }
    event.update(overrides)
    return event


def _terminal(recorder: _FakeRecorder, session_id: str, side: str, **overrides):
    payload = {
        "session_id": session_id,
        "camera_side": side,
        "terminal_status": "complete",
        "last_emitted_event_sequence": 1,
        "last_source_sequence": 0,
        "last_video_frame_index": 0,
        "source_count": 1,
        "mapping_count": 1,
        "output_path": str(recorder.paths.video_path(side)),
        "eos_ok": True,
        "pipeline_error": None,
        "ipc_backpressure": False,
        "caps_changed": False,
        "pts_reordering_observed": False,
        "authority_fifo_residual": 0,
    }
    payload.update(overrides)
    return payload


def _wait(event: threading.Event, message: str = "controller did not finish") -> None:
    deadline = time.monotonic() + 5.0
    while not event.is_set() and time.monotonic() < deadline:
        _QT_APP.processEvents()
        event.wait(0.01)
    _QT_APP.processEvents()
    assert event.is_set(), message


def _controller(tmp_path: Path, recorder_cls=_FakeRecorder, **kwargs):
    holder = {}

    def factory(config):
        holder["recorder"] = recorder_cls(config)
        return holder["recorder"]

    controller = SessionCameraRecordingController(
        recorder_factory=factory,
        mkv_probe=kwargs.pop("mkv_probe", _phase_a_probe),
        codec_probe=kwargs.pop("codec_probe", _codec_probe),
        **kwargs,
    )
    session_id = "b1-adapter"
    opened = threading.Event()
    controller.opened.connect(lambda _payload: opened.set())
    assert controller.open_session(SessionRecordingConfig(tmp_path, session_id))
    _wait(opened, "recorder did not open")
    return controller, holder["recorder"], session_id


def _submit_complete(controller, recorder, session_id, *, mapping_first=False):
    for side in ("left", "right"):
        events = (_mapping(session_id, side, 0), _source(session_id, side, 1)) if mapping_first else (
            _source(session_id, side, 0),
            _mapping(session_id, side, 1),
        )
        for event in events:
            assert controller.submit_recording_event(event)
        assert controller.submit_camera_terminal(_terminal(recorder, session_id, side))


def test_encoded_au_creates_source_and_receive_time_alignment(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    done = threading.Event()
    controller.session_terminal.connect(lambda _result: done.set())
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert len(recorder.sources) == len(recorder.alignments) == 2
    for source, alignment in zip(recorder.sources, recorder.alignments):
        assert source.session_id == session_id
        assert source.camera_stream_id == f"wiris-{source.camera_side}"
        assert source.frame_id == source.camera_sequence == 0
        assert source.pc_receive_monotonic_ns == 1_000_000
        assert source.pc_receive_utc_ns == 2_000_000
        assert source.rtp_timestamp == 90_000
        assert source.rtp_ssrc == 123
        assert source.rtp_marker_seq == 0
        assert source.rtcp_sr_ntp_raw == -1
        assert source.rtcp_frame_ntp_unix_ns == 3_000_000
        assert source.pts_ticks == source.dts_ticks == 1
        assert (source.time_base_num, source.time_base_den) == (1, 1_000_000_000)
        assert (source.width_px, source.height_px) == (1280, 720)
        assert source.video_mapping_status == "pending"
        assert source.stored_in_video is False
        assert alignment.record_type == "camera_source"
        assert alignment.record_id == source.source_record_id
        assert alignment.camera_side == source.camera_side
        assert alignment.alignment_reference_domain == "pc_receive_monotonic"
        assert alignment.alignment_status == "receive_time_fallback"
        assert alignment.aligned_pc_monotonic_ns == source.pc_receive_monotonic_ns
        assert alignment.pc_receive_monotonic_ns == source.pc_receive_monotonic_ns
        assert alignment.pc_receive_utc_ns == source.pc_receive_utc_ns
        assert alignment.camera_rtp_timestamp == source.rtp_timestamp
        assert alignment.camera_pts_ticks == source.pts_ticks
        assert alignment.camera_dts_ticks == source.dts_ticks
        assert alignment.camera_rtcp_ntp_unix_ns == source.rtcp_frame_ntp_unix_ns
        assert alignment.camera_rtcp_to_pc_utc_delta_ns is None
        assert alignment.px4_utc_to_pc_utc_delta_ns is None
    assert recorder.thread_ids == {controller._thread.ident}
    assert controller.shutdown()


def _assert_event_order_is_complete(tmp_path: Path, *, mapping_first: bool) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id, mapping_first=mapping_first)
    _wait(done)
    assert len(recorder.sources) == len(recorder.mappings) == 2
    assert result["session_state"] == "complete", result
    assert result["failures"] == []
    assert controller.shutdown()


def test_mapping_before_source_uses_phase_a_pending_mapping(tmp_path: Path) -> None:
    _assert_event_order_is_complete(tmp_path, mapping_first=True)


def test_source_before_mapping_is_equivalent(tmp_path: Path) -> None:
    _assert_event_order_is_complete(tmp_path, mapping_first=False)


def test_camera_event_sequence_is_contiguous(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert all(camera["last_emitted_event_sequence"] == camera["last_processed_event_sequence"] == 1 for camera in result["camera"].values())
    assert not any(failure["stage"] == "recording_event_sequence_gap" for failure in result["failures"])
    assert controller.shutdown()


def test_sequence_gap_blocks_complete(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    assert controller.submit_recording_event(_source(session_id, "left", 1))
    assert controller.submit_recording_event(_mapping(session_id, "left", 2))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "left", last_emitted_event_sequence=2))
    assert controller.submit_recording_event(_source(session_id, "right", 0))
    assert controller.submit_recording_event(_mapping(session_id, "right", 1))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "right"))
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == "recording_event_sequence_gap" for failure in result["failures"])
    assert controller.shutdown()


def test_terminal_sequence_barrier_waits_for_all_events(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    done = threading.Event()
    controller.session_terminal.connect(lambda _value: done.set())
    for side in ("left", "right"):
        assert controller.submit_recording_event(_source(session_id, side, 0))
        assert controller.submit_recording_event(_mapping(session_id, side, 1))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "left"))
    assert not done.wait(0.1)
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "right"))
    _wait(done)
    assert recorder.close_calls == 1
    assert controller.shutdown()


def test_queue_empty_is_not_used_as_barrier() -> None:
    source = inspect.getsource(SessionCameraRecordingController)
    assert ".empty(" not in source
    assert ".qsize(" not in source


def test_complete_gate_requires_camera_counts_equal(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert len(recorder.artifacts) == 2
    assert result["counts"] == {
        "source_records": 2,
        "time_alignment": 2,
        "pixhawk_samples": 0,
        "triangulation": 0,
    }
    assert all(camera["source"] == camera["alignment"] == camera["mapping"] == camera["decoded_mkv"] == 1 for camera in result["camera"].values())
    assert controller.shutdown()


def test_complete_camera_only_session_has_seven_artifacts(tmp_path: Path) -> None:
    controller = SessionCameraRecordingController()
    session_id = "b1-real-camera-only"
    config = SessionRecordingConfig(root=tmp_path / "recordings", session_id=session_id)
    paths = SessionPaths.from_config(config)
    opened = threading.Event()
    controller.opened.connect(lambda _value: opened.set())
    assert controller.open_session(config)
    _wait(opened)
    _write_one_frame_mkv(paths.left_video)
    _write_one_frame_mkv(paths.right_video)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    for side in ("left", "right"):
        events = (
            (
                _mapping(session_id, side, 0),
                _source(session_id, side, 1, width_px=32, height_px=24),
            )
            if side == "left"
            else (
                _source(session_id, side, 0, width_px=32, height_px=24),
                _mapping(session_id, side, 1),
            )
        )
        for event in events:
            assert controller.submit_recording_event(event)
        assert controller.submit_camera_terminal(
            {
                "session_id": session_id,
                "camera_side": side,
                "terminal_status": "complete",
                "last_emitted_event_sequence": 1,
                "last_source_sequence": 0,
                "last_video_frame_index": 0,
                "source_count": 1,
                "mapping_count": 1,
                "output_path": str(paths.video_path(side)),
                "eos_ok": True,
                "pipeline_error": None,
                "ipc_backpressure": False,
                "caps_changed": False,
                "pts_reordering_observed": False,
                "authority_fifo_residual": 0,
                "failure_stages": (),
                "failure_details": (),
                "rtp_association_mismatches": 0,
            }
        )
    _wait(done)
    assert result["session_state"] == "complete", result
    assert result["counts"] == {
        "source_records": 2,
        "time_alignment": 2,
        "pixhawk_samples": 0,
        "triangulation": 0,
    }
    actual_files = {
        path.relative_to(paths.session_dir).as_posix()
        for path in paths.session_dir.rglob("*")
        if path.is_file()
    }
    assert actual_files == set(paths.manifest)
    assert len(paths.manifest) == len(paths.all_artifacts) == len(actual_files) == 7
    source_rows = sorted(pq.read_table(paths.source_records).to_pylist(), key=lambda row: row["camera_side"])
    assert [row["video_path"] for row in source_rows] == [
        "camera/left_video.mkv",
        "camera/right_video.mkv",
    ]
    for row in source_rows:
        assert row["pc_receive_monotonic_ns"] == 1_000_000
        assert row["pc_receive_utc_ns"] == 2_000_000
        assert row["rtp_timestamp"] == 90_000
        assert row["rtp_ssrc"] == 123
        assert row["rtp_marker_seq"] == 0
        assert row["rtcp_sr_ntp_raw"] == -1
        assert row["rtcp_frame_ntp_unix_ns"] == 3_000_000
        assert row["pts_ticks"] == row["dts_ticks"] == 1
        assert (row["time_base_num"], row["time_base_den"]) == (1, 1_000_000_000)
        assert (row["width_px"], row["height_px"]) == (32, 24)
        assert row["stored_in_video"] is True
        assert row["video_mapping_status"] == "stored"
    alignment_rows = pq.read_table(paths.time_alignment).to_pylist()
    assert len(alignment_rows) == 2
    for row in alignment_rows:
        assert row["alignment_reference_domain"] == "pc_receive_monotonic"
        assert row["alignment_status"] == "receive_time_fallback"
        assert row["aligned_pc_monotonic_ns"] == row["pc_receive_monotonic_ns"] == 1_000_000
        assert row["pc_receive_utc_ns"] == 2_000_000
        assert row["camera_rtp_timestamp"] == 90_000
        assert row["camera_pts_ticks"] == row["camera_dts_ticks"] == 1
        assert row["camera_rtcp_ntp_unix_ns"] == 3_000_000
        assert row["camera_rtcp_to_pc_utc_delta_ns"] is None
        assert row["px4_utc_to_pc_utc_delta_ns"] is None
    assert pq.ParquetFile(paths.pixhawk_samples).metadata.num_rows == 0
    assert pq.ParquetFile(paths.triangulation).metadata.num_rows == 0
    assert controller.shutdown()


def test_pixhawk_and_triangulation_tables_are_empty(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert result["counts"]["pixhawk_samples"] == 0
    assert result["counts"]["triangulation"] == 0
    assert controller.shutdown()


def _right_eos_failure(tmp_path: Path):
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    for side in ("left", "right"):
        assert controller.submit_recording_event(_source(session_id, side, 0))
        assert controller.submit_recording_event(_mapping(session_id, side, 1))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "left"))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "right", eos_ok=False, terminal_status="eos_failed"))
    _wait(done)
    call_names = [call[0] for call in recorder.calls]
    assert max(index for index, name in enumerate(call_names) if name == "artifact") < call_names.index("mark_failure")
    assert recorder.close_calls == result["logical_close_count"] == result["mark_failure_count"] == 1
    assert result["session_state"] == "incomplete"
    return controller, recorder, result


def test_artifacts_registered_before_mark_failure(tmp_path: Path) -> None:
    controller, recorder, result = _right_eos_failure(tmp_path)
    call_names = [call[0] for call in recorder.calls]
    assert max(index for index, name in enumerate(call_names) if name == "artifact") < call_names.index("mark_failure")
    assert controller.shutdown()


def test_right_eos_failure_preserves_partial_artifacts(tmp_path: Path) -> None:
    controller, recorder, result = _right_eos_failure(tmp_path)
    assert len(recorder.artifacts) == 2
    assert {artifact.camera_side for artifact in recorder.artifacts} == {"left", "right"}
    assert result["session_state"] == "incomplete"
    assert controller.shutdown()


def test_missing_mkv_is_incomplete(tmp_path: Path) -> None:
    def missing_probe(path, *, include_packets=True):
        raise RuntimeError(f"missing: {path}")

    controller, recorder, session_id = _controller(tmp_path, mkv_probe=missing_probe)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == "camera_mkv_probe" for failure in result["failures"])
    assert controller.shutdown()


def test_count_mismatch_is_incomplete(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    for side in ("left", "right"):
        assert controller.submit_recording_event(_source(session_id, side, 0))
        assert controller.submit_recording_event(_mapping(session_id, side, 1))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "left", source_count=2))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "right"))
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == "camera_source_count_mismatch" for failure in result["failures"])
    assert controller.shutdown()


def test_dropped_mapping_blocks_b1_complete(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    assert controller.submit_recording_event(_source(session_id, "left", 0))
    assert controller.submit_recording_event(
        _mapping(
            session_id,
            "left",
            1,
            stored_in_video=False,
            video_mapping_status="dropped",
            video_frame_index=None,
            drop_reason="writer_drop",
        )
    )
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "left", last_video_frame_index=-1))
    assert controller.submit_recording_event(_source(session_id, "right", 0))
    assert controller.submit_recording_event(_mapping(session_id, "right", 1))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "right"))
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] in {"video_index_gap", "camera_count_mismatch"} for failure in result["failures"])
    assert controller.shutdown()


def test_b_frame_probe_is_incomplete(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(
        tmp_path,
        codec_probe=lambda _path: B1VideoProbe("h264", 2, 1, 1),
    )
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == "b_frame_not_supported" for failure in result["failures"])
    assert controller.shutdown()


def test_real_b_frame_fixture_is_rejected(tmp_path: Path) -> None:
    controller = SessionCameraRecordingController()
    session_id = "b1-real-b-frame"
    config = SessionRecordingConfig(root=tmp_path / "recordings", session_id=session_id)
    paths = SessionPaths.from_config(config)
    opened = threading.Event()
    controller.opened.connect(lambda _value: opened.set())
    assert controller.open_session(config)
    _wait(opened)
    frame_count = 12
    _write_h264_mkv(paths.left_video, frame_count=frame_count, bframes=2)
    _write_h264_mkv(paths.right_video, frame_count=frame_count, bframes=0)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    for side in ("left", "right"):
        for frame in range(frame_count):
            assert controller.submit_recording_event(
                _source(session_id, side, event_sequence=frame * 2, frame=frame, width_px=32, height_px=24)
            )
            assert controller.submit_recording_event(
                _mapping(session_id, side, event_sequence=frame * 2 + 1, frame=frame)
            )
        assert controller.submit_camera_terminal(
            {
                "session_id": session_id,
                "camera_side": side,
                "terminal_status": "complete",
                "last_emitted_event_sequence": frame_count * 2 - 1,
                "last_source_sequence": frame_count - 1,
                "last_video_frame_index": frame_count - 1,
                "source_count": frame_count,
                "mapping_count": frame_count,
                "output_path": str(paths.video_path(side)),
                "eos_ok": True,
                "pipeline_error": None,
                "ipc_backpressure": False,
                "caps_changed": False,
                "pts_reordering_observed": False,
                "authority_fifo_residual": 0,
                "failure_stages": (),
                "failure_details": (),
                "rtp_association_mismatches": 0,
            }
        )
    _wait(done)
    assert result["camera"]["left"]["has_b_frames"] > 0
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == "b_frame_not_supported" for failure in result["failures"])
    assert controller.shutdown()


def _second_left_source_failure(tmp_path: Path, source_overrides, expected_stage) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    # Establish the camera contract with frame zero, then inject a second AU.
    assert controller.submit_recording_event(_source(session_id, "left", 0))
    assert controller.submit_recording_event(_mapping(session_id, "left", 1))
    assert controller.submit_recording_event(_source(session_id, "left", 2, frame=1, **source_overrides))
    assert controller.submit_recording_event(_mapping(session_id, "left", 3, frame=1))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "left", last_emitted_event_sequence=3, last_source_sequence=1, last_video_frame_index=1, source_count=2, mapping_count=2))
    assert controller.submit_recording_event(_source(session_id, "right", 0))
    assert controller.submit_recording_event(_mapping(session_id, "right", 1))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "right"))
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == expected_stage for failure in result["failures"])
    assert controller.shutdown()


def test_caps_change_is_incomplete(tmp_path: Path) -> None:
    _second_left_source_failure(tmp_path, {"width_px": 640}, "camera_caps_changed")


def test_nonmonotonic_pts_is_incomplete(tmp_path: Path) -> None:
    _second_left_source_failure(tmp_path, {"pts_ticks": 0}, "frame_reordering_detected")


def test_backpressure_is_incomplete(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(
        tmp_path,
        recorder_cls=_BlockingRecorder,
        data_capacity=1,
        put_timeout_s=0.05,
    )
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    assert controller.submit_recording_event(_source(session_id, "left", 0))
    _wait(recorder.entered, "recorder did not block")
    assert controller.submit_recording_event(_mapping(session_id, "left", 1))
    assert not controller.submit_recording_event(_source(session_id, "right", 0))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "left"))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "right", last_emitted_event_sequence=-1, source_count=0, mapping_count=0, eos_ok=False))
    recorder.release.set()
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == "recording_ipc_backpressure" for failure in result["failures"])
    assert controller.shutdown()


def _finalize_timeout_result(tmp_path: Path):
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    assert controller.finalize_timeout("dual-camera timeout")
    _wait(done)
    return controller, recorder, result


def test_finalize_timeout_is_incomplete(tmp_path: Path) -> None:
    controller, recorder, result = _finalize_timeout_result(tmp_path)
    assert result["session_state"] == "incomplete"
    assert not controller.finalize_timeout("duplicate")
    assert controller.shutdown()


def test_close_is_logically_requested_once(tmp_path: Path) -> None:
    controller, recorder, result = _finalize_timeout_result(tmp_path)
    assert result["logical_close_count"] == recorder.close_calls == 1
    assert controller.shutdown()


def test_terminal_failure_stages_force_incomplete_even_when_counts_match(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    for side in ("left", "right"):
        assert controller.submit_recording_event(_source(session_id, side, 0))
        assert controller.submit_recording_event(_mapping(session_id, side, 1))
    assert controller.submit_camera_terminal(
        _terminal(
            recorder,
            session_id,
            "left",
            terminal_status="incomplete",
            failure_stages=("first_keyframe_timeout", "rtp_au_association_failed"),
            failure_details=("keyframe late", "RTP marker missing"),
            rtp_association_mismatches=1,
        )
    )
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "right"))
    _wait(done)
    stages = {failure["stage"] for failure in result["failures"]}
    assert {"first_keyframe_timeout", "rtp_au_association_failed"} <= stages
    assert result["session_state"] == "incomplete"
    assert controller.shutdown()


def test_camera_result_includes_packet_count(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert result["camera"]["left"]["packet_count"] == 1
    assert result["camera"]["right"]["packet_count"] == 1
    assert controller.shutdown()


def test_controller_thread_is_lazy_and_gui_ingress_has_no_wait() -> None:
    controller = SessionCameraRecordingController()
    assert not controller._thread_started
    submit_source = inspect.getsource(SessionCameraRecordingController.submit_recording_event)
    assert ".wait(" not in submit_source
    assert controller.shutdown()


def test_out_of_order_accepted_events_drain_before_close_and_next_session_is_clean(tmp_path: Path) -> None:
    recorders = []

    def factory(config):
        recorder = _FakeRecorder(config)
        recorders.append(recorder)
        return recorder

    controller = SessionCameraRecordingController(
        recorder_factory=factory,
        mkv_probe=_phase_a_probe,
        codec_probe=_codec_probe,
    )
    opened = threading.Event()
    terminal = threading.Event()
    results = []
    controller.opened.connect(lambda _value: opened.set())
    controller.session_terminal.connect(lambda value: (results.append(value), terminal.set()))

    first_id = "b1-out-of-order"
    assert controller.open_session(SessionRecordingConfig(tmp_path / "first", first_id))
    _wait(opened)
    first_recorder = recorders[-1]
    for side in ("left", "right"):
        # Accepted order [1, 0] used to satisfy a max-sequence-only barrier
        # after the first event and leave the second event for the next session.
        assert controller.submit_recording_event(_source(first_id, side, 1))
        assert controller.submit_recording_event(_mapping(first_id, side, 0))
        assert controller.submit_camera_terminal(_terminal(first_recorder, first_id, side))
    _wait(terminal)
    assert results[0]["session_state"] == "incomplete"
    assert all(camera["accepted_event_count"] == camera["processed_event_count"] == 2 for camera in results[0]["camera"].values())
    assert first_recorder.close_calls == 1
    with controller._condition:
        assert not controller._data_events

    opened.clear()
    terminal.clear()
    second_id = "b1-next-session"
    assert controller.open_session(SessionRecordingConfig(tmp_path / "second", second_id))
    _wait(opened)
    second_recorder = recorders[-1]
    _submit_complete(controller, second_recorder, second_id)
    _wait(terminal)
    assert len(results) == 2
    assert results[1]["session_state"] == "complete", results[1]
    assert results[1]["counts"]["source_records"] == 2
    assert second_recorder.close_calls == 1
    assert controller.shutdown()


def test_unexpected_finalize_exception_still_closes_once_and_emits_incomplete(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path, recorder_cls=_ExplodingMarkFailureRecorder)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    for side in ("left", "right"):
        assert controller.submit_recording_event(_source(session_id, side, 0))
        assert controller.submit_recording_event(_mapping(session_id, side, 1))
    assert controller.submit_camera_terminal(_terminal(recorder, session_id, "left"))
    assert controller.submit_camera_terminal(
        _terminal(recorder, session_id, "right", terminal_status="incomplete", eos_ok=False)
    )
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert result["clean_shutdown"] is False
    assert result["logical_close_count"] == recorder.close_calls == 1
    assert recorder.mark_failure_attempts == 1
    assert any(failure["stage"] == "session_finalize_exception" for failure in result["failures"])
    assert controller.state == "incomplete"
    assert controller._recorder is None
    assert controller._finalizing is False
    assert controller.shutdown()


def test_phase_a_incomplete_close_adds_terminal_failure_observation(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(tmp_path, recorder_cls=_IncompleteCloseRecorder)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert result["clean_shutdown"] is False
    assert result["mark_failure_count"] == 0
    assert result["logical_close_count"] == recorder.close_calls == 1
    assert result["failures"] == [
        {
            "stage": "session_close_incomplete",
            "message": "Phase A close returned session_state=incomplete",
            "camera_side": None,
        }
    ]
    assert controller.shutdown()


def test_synthetic_gstreamer_events_feed_phase_a_complete_camera_session(tmp_path: Path) -> None:
    gi = pytest.importorskip("gi", reason="GStreamer Python bindings are unavailable")
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    Gst.init(None)
    required = (
        "videotestsrc",
        "x264enc",
        "h264parse",
        "tee",
        "queue",
        "fakesink",
        "valve",
        "identity",
        "matroskamux",
        "filesink",
    )
    missing = [name for name in required if Gst.ElementFactory.find(name) is None]
    if missing:
        pytest.skip(f"required synthetic GStreamer plugins are unavailable: {', '.join(missing)}")

    session_id = "b1-gstreamer-to-phase-a"
    config = SessionRecordingConfig(root=tmp_path / "recordings", session_id=session_id)
    paths = SessionPaths.from_config(config)
    controller = SessionCameraRecordingController()
    opened = threading.Event()
    done = threading.Event()
    result = {}
    controller.opened.connect(lambda _value: opened.set())
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    assert controller.open_session(config)
    _wait(opened)

    frame_count = 5
    for side in ("left", "right"):
        events, terminal = _run_synthetic_canonical_gst_camera(
            Gst,
            session_id=session_id,
            side=side,
            output_path=paths.video_path(side),
            frame_count=frame_count,
        )
        assert [event["record_event_sequence"] for event in events] == list(range(frame_count * 2))
        assert len([event for event in events if event["event_kind"] == "camera_source"]) == frame_count
        assert len([event for event in events if event["event_kind"] == "video_mapping"]) == frame_count
        for event in events:
            assert controller.submit_recording_event(event)
        assert terminal["terminal_status"] == "complete"
        assert controller.submit_camera_terminal(terminal)

    _wait(done)
    assert result["session_state"] == "complete", result
    assert result["clean_shutdown"] is True
    assert result["counts"] == {
        "source_records": frame_count * 2,
        "time_alignment": frame_count * 2,
        "pixhawk_samples": 0,
        "triangulation": 0,
    }
    for side in ("left", "right"):
        camera = result["camera"][side]
        assert camera["source"] == frame_count
        assert camera["alignment"] == frame_count
        assert camera["mapping"] == frame_count
        assert camera["decoded_mkv"] == frame_count
        assert camera["has_b_frames"] == 0
    actual_files = {
        path.relative_to(paths.session_dir).as_posix()
        for path in paths.session_dir.rglob("*")
        if path.is_file()
    }
    assert actual_files == set(paths.manifest)
    assert len(actual_files) == 7
    assert pq.ParquetFile(paths.pixhawk_samples).metadata.num_rows == 0
    assert pq.ParquetFile(paths.triangulation).metadata.num_rows == 0
    assert controller.shutdown()


def test_probe_b1_video_rejects_missing_has_b_frames(monkeypatch, tmp_path: Path) -> None:
    video_path = tmp_path / "probe.mkv"
    video_path.write_bytes(b"fixture")
    monkeypatch.setattr(session_camera_module.shutil, "which", lambda _name: "/usr/bin/ffprobe")
    monkeypatch.setattr(
        session_camera_module.subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(
            stdout='{"streams":[{"codec_name":"h264","nb_read_frames":"1","nb_read_packets":"1"}]}'
        ),
    )
    with pytest.raises(RuntimeError, match="omitted has_b_frames"):
        probe_b1_video(video_path)


@pytest.mark.parametrize(
    "codec_probe",
    [
        B1VideoProbe("h264", 0, None, 1),
        B1VideoProbe("h264", 0, 1, None),
    ],
)
def test_complete_gate_rejects_missing_b1_probe_counts(tmp_path: Path, codec_probe: B1VideoProbe) -> None:
    controller, recorder, session_id = _controller(tmp_path, codec_probe=lambda _path: codec_probe)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == "camera_probe_count_missing" for failure in result["failures"])
    assert controller.shutdown()


@pytest.mark.parametrize("missing_field", ["decoded", "packet"])
def test_complete_gate_rejects_missing_phase_a_probe_counts(tmp_path: Path, missing_field: str) -> None:
    def incomplete_phase_a_probe(path, *, include_packets=True):
        return MkvProbeResult(
            path=Path(path),
            format_name="matroska,webm",
            codec_name="h264",
            decoded_frame_count=None if missing_field == "decoded" else 1,
            packet_count=None if missing_field == "packet" else 1,
            time_base_num=1,
            time_base_den=1_000,
        )

    controller, recorder, session_id = _controller(tmp_path, mkv_probe=incomplete_phase_a_probe)
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == "camera_probe_count_missing" for failure in result["failures"])
    assert controller.shutdown()


def test_complete_gate_rejects_b1_phase_a_packet_count_mismatch(tmp_path: Path) -> None:
    controller, recorder, session_id = _controller(
        tmp_path,
        codec_probe=lambda _path: B1VideoProbe("h264", 0, 1, 2),
    )
    result = {}
    done = threading.Event()
    controller.session_terminal.connect(lambda value: (result.update(value), done.set()))
    _submit_complete(controller, recorder, session_id)
    _wait(done)
    assert result["session_state"] == "incomplete"
    assert any(failure["stage"] == "camera_probe_packet_count_mismatch" for failure in result["failures"])
    assert controller.shutdown()


def test_finalize_timeout_during_blocking_probe_is_single_incomplete_terminal_and_reusable(tmp_path: Path) -> None:
    recorders = []

    def factory(config):
        recorder = _FakeRecorder(config)
        recorders.append(recorder)
        return recorder

    codec_probe = _BlockingCodecProbe()
    controller = SessionCameraRecordingController(
        recorder_factory=factory,
        mkv_probe=_phase_a_probe,
        codec_probe=codec_probe,
    )
    opened = threading.Event()
    done = threading.Event()
    results = []
    controller.opened.connect(lambda _value: opened.set())
    controller.session_terminal.connect(lambda value: (results.append(value), done.set()))

    first_id = "b1-timeout-during-probe"
    assert controller.open_session(SessionRecordingConfig(tmp_path / "first", first_id))
    _wait(opened)
    first_recorder = recorders[-1]
    _submit_complete(controller, first_recorder, first_id)
    _wait(codec_probe.entered, "codec probe did not block")
    assert controller.finalize_timeout("probe exceeded dual-camera deadline")
    codec_probe.release.set()
    _wait(done)
    assert len(results) == 1
    assert results[0]["session_state"] == "incomplete"
    assert results[0]["clean_shutdown"] is False
    assert results[0]["logical_close_count"] == first_recorder.close_calls == 1
    assert any(failure["stage"] == "camera_finalize_timeout" for failure in results[0]["failures"])
    assert controller.state == "incomplete"

    opened.clear()
    done.clear()
    second_id = "b1-after-probe-timeout"
    assert controller.open_session(SessionRecordingConfig(tmp_path / "second", second_id))
    _wait(opened)
    second_recorder = recorders[-1]
    _submit_complete(controller, second_recorder, second_id)
    _wait(done)
    assert len(results) == 2
    assert results[1]["session_state"] == "complete", results[1]
    assert second_recorder.close_calls == 1
    assert controller.shutdown()


def test_finalize_timeout_is_rejected_after_persisted_close_begins(tmp_path: Path) -> None:
    recorders = []

    def factory(config):
        recorder = _BlockingCloseRecorder(config) if not recorders else _FakeRecorder(config)
        recorders.append(recorder)
        return recorder

    controller = SessionCameraRecordingController(
        recorder_factory=factory,
        mkv_probe=_phase_a_probe,
        codec_probe=_codec_probe,
    )
    opened = threading.Event()
    done = threading.Event()
    results = []
    controller.opened.connect(lambda _value: opened.set())
    controller.session_terminal.connect(lambda value: (results.append(value), done.set()))

    first_id = "b1-timeout-after-close-start"
    assert controller.open_session(SessionRecordingConfig(tmp_path / "first", first_id))
    _wait(opened)
    first_recorder = recorders[-1]
    _submit_complete(controller, first_recorder, first_id)
    _wait(first_recorder.close_entered, "Phase A close did not begin")
    assert controller.finalize_timeout("late aggregate deadline") is False
    first_recorder.close_release.set()
    _wait(done)
    assert len(results) == 1
    assert results[0]["session_state"] == first_recorder.state == "complete"
    assert results[0]["clean_shutdown"] is True
    assert results[0]["failures"] == []
    assert results[0]["logical_close_count"] == first_recorder.close_calls == 1
    assert controller.state == "complete"

    opened.clear()
    done.clear()
    second_id = "b1-after-close-start"
    assert controller.open_session(SessionRecordingConfig(tmp_path / "second", second_id))
    _wait(opened)
    second_recorder = recorders[-1]
    _submit_complete(controller, second_recorder, second_id)
    _wait(done)
    assert len(results) == 2
    assert results[1]["session_state"] == "complete", results[1]
    assert second_recorder.close_calls == 1
    assert controller.shutdown()
