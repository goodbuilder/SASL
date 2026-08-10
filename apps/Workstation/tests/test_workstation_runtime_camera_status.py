from Workstation.workstation_core.runtime_camera_status import (
    format_runtime_camera_status,
    format_runtime_camera_timestamp,
    infer_runtime_camera_timestamp_reference,
)


def test_timestamp_source_contract_does_not_mix_invalid_rtcp_with_source_time():
    metadata = {
        "rtcp_timing_invalid_reason": "sr_pc_offset",
        "source_timestamp_s": 12.5,
    }

    assert infer_runtime_camera_timestamp_reference(metadata) == "rtcp-invalid"
    assert format_runtime_camera_timestamp(metadata) == "RTCP invalid (SR-PC offset)"


def test_runtime_status_is_pure_and_reexported_by_workstation_owner():
    from Workstation import standalone_gui

    stats = {
        "width": 1280,
        "height": 720,
        "decoded_hz": 29.9,
        "inferred_hz": 10.0,
        "enable_detection": True,
        "yolo_actual_device": "cuda:0",
    }
    metadata = {"timestamp_reference": "source", "source_timestamp_s": 1.25}

    text = format_runtime_camera_status("left", stats, metadata, 30.0, stale=False)
    assert "1280x720 29.9 Hz" in text
    assert "10.0 Hz (CUDA)" in text
    assert "src 1.250000 s [source]" in text
    assert standalone_gui.format_runtime_camera_status is format_runtime_camera_status
