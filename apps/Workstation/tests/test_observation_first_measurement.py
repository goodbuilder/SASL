from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from Workstation.camera_stream_controller import CameraStreamLifecycleController
from Workstation.recording_controller import RecordingController


class _WidgetState:
    def setText(self, _text: str) -> None:
        pass


def test_camera_rejoin_allocates_non_overwriting_video_segments(tmp_path: Path) -> None:
    artifacts: list[dict[str, object]] = []
    transitions: list[tuple[str, str]] = []
    panel = SimpleNamespace(
        recording_session_id="run-observation",
        _measurement_run_generation="run-observation",
        allocate_workstation_data_artifact=lambda category, **kwargs: (
            tmp_path / category / f"{kwargs['label']}{kwargs['suffix']}"
        ),
        append_workstation_data=lambda _stream, payload, **_kwargs: artifacts.append(
            dict(payload)
        ),
        measurement_source_transition=lambda source, state, _detail: transitions.append(
            (source, state)
        ),
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)

    first = controller.allocate_measurement_video_segment("left", utc_ns=1)
    second = controller.allocate_measurement_video_segment("left", utc_ns=2)

    assert first is not None and second is not None and first != second
    assert "segment-01" in first.name
    assert "segment-02" in second.name
    first.parent.mkdir(parents=True, exist_ok=True)
    first.touch()
    row = controller.record_measurement_video_segment_presence(
        "left", first, runtime_generation="run-observation"
    )
    assert row["file_present"] is True
    assert controller.measurement_video_segments_snapshot()["state"] == "PARTIAL"
    assert transitions[0] == ("recording.left", "WAITING_RESOURCE")
    assert artifacts[0]["runtime_generation"] == "run-observation"


def test_video_presence_persistence_failure_never_blocks_teardown(tmp_path: Path) -> None:
    path = tmp_path / "left.mp4"
    path.touch()
    panel = SimpleNamespace(
        _pending_video_finalization_artifacts={
            "paths": {"left": path},
            "backend": "gstreamer",
            "runtime_generation": "run-old",
        },
        left_pane=SimpleNamespace(recording_finalizations={}),
        append_workstation_data=lambda *_args, **_kwargs: False,
        flush_workstation_data=lambda: False,
        _log=lambda _message: None,
    )

    assert CameraStreamLifecycleController(panel)._append_pending_video_presence_artifacts()
    assert panel._pending_video_finalization_artifacts is None


def test_late_old_video_presence_cannot_mutate_new_run_segments(tmp_path: Path) -> None:
    panel = SimpleNamespace(
        recording_session_id="run-old",
        _measurement_run_generation="run-old",
        allocate_workstation_data_artifact=lambda category, **kwargs: (
            tmp_path / category / f"{kwargs['label']}{kwargs['suffix']}"
        ),
        append_workstation_data=lambda *_args, **_kwargs: True,
        measurement_source_transition=lambda *_args, **_kwargs: True,
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)
    old_path = controller.allocate_measurement_video_segment("left", utc_ns=1)
    assert old_path is not None
    old_path.parent.mkdir(parents=True, exist_ok=True)
    old_path.touch()

    panel.recording_session_id = "run-new"
    panel._measurement_run_generation = "run-new"
    controller._begin_measurement_video_generation("run-new")
    new_path = controller.allocate_measurement_video_segment("right", utc_ns=2)
    assert new_path is not None

    controller.record_measurement_video_segment_presence(
        "left", old_path, runtime_generation="run-old"
    )

    snapshot = controller.measurement_video_segments_snapshot()
    assert snapshot["runtime_generation"] == "run-new"
    assert snapshot["state"] == "IDLE"
    assert snapshot["sides"]["left"] == []
    assert snapshot["sides"]["right"][0]["path"] == str(new_path)
    assert panel._last_recording_presence_state == "IDLE"
    old_segments = panel._measurement_video_segments_by_generation["run-old"]
    assert old_segments["left"][0]["file_present"] is True


def test_sessionless_stop_finishes_when_optional_persistence_raises() -> None:
    class BrokenRows:
        def close(self) -> None:
            raise OSError("injected row close failure")

    panel = SimpleNamespace(
        _measurement_session_hierarchy_enabled=False,
        _method_evaluation_accepting=True,
        distance_recording_enabled=True,
        video_recording_enabled=False,
        distance_recording_started_monotonic_s=1.0,
        _manual_recording_active=True,
        _recording_mode="manual",
        recording_session_id="run-best-effort",
        distance_recording_rows=BrokenRows(),
        camera_recording_rows={},
        px4_recording_rows={},
        gnss_base_recording_rows=BrokenRows(),
        extrinsics_comparison_recording_rows=BrokenRows(),
        method_evaluation_bundle_rows=BrokenRows(),
        video_recording_paths={},
        record_button=_WidgetState(),
        camera_connected=False,
        append_workstation_data=lambda *_args, **_kwargs: (_ for _ in ()).throw(
            OSError("injected append failure")
        ),
        flush_workstation_data=lambda: (_ for _ in ()).throw(
            OSError("injected flush failure")
        ),
        _sync_rtcm_evidence_context=lambda: (_ for _ in ()).throw(
            OSError("injected sync failure")
        ),
        _resume_queued_measurement_start=lambda: None,
        _refresh_recording_status_label=lambda: None,
        _refresh_operation_header=lambda: None,
        _log=lambda _message: None,
    )

    RecordingController(panel)._stop_session_recording(restart_streams=False)

    assert panel.distance_recording_enabled is False
    assert panel.video_recording_enabled is False
    assert panel._manual_recording_active is False
    assert panel.recording_session_id == ""
    assert panel._recording_mode == "idle"
