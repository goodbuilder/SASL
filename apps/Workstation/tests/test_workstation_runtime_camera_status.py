from Workstation.workstation_core.runtime_camera_status import (
    decoder_presentation,
    format_runtime_camera_status,
    format_runtime_camera_timestamp,
    infer_runtime_camera_timestamp_reference,
    parse_nvidia_runtime_identity,
    read_nvidia_runtime_identity,
)


def test_decoder_presentation_distinguishes_gpu_selected_software_and_fallback():
    gpu = decoder_presentation(requested_mode="auto", active_decoder="nvh264dec")
    selected = decoder_presentation(requested_mode="software", active_decoder="avdec_h264")
    fallback = decoder_presentation(
        requested_mode="auto",
        active_decoder="avdec_h264",
        runtime_override="software",
        fallback_reason="nvh264dec first-frame timeout",
    )

    assert gpu.value_text == "GPU · nvh264dec"
    assert gpu.severity == "ok"
    assert gpu.gpu_active is True
    assert selected.value_text == "CPU · avdec_h264 (selected)"
    assert selected.severity == "ok"
    assert fallback.value_text == "CPU fallback · avdec_h264"
    assert fallback.severity == "warn"
    assert fallback.fallback_active is True
    assert "first-frame timeout" in fallback.detail


def test_decoder_presentation_never_treats_requested_mode_as_actual_evidence():
    waiting = decoder_presentation(requested_mode="nvidia", active_decoder=None)
    mismatch = decoder_presentation(requested_mode="nvidia", active_decoder="avdec_h264")

    assert waiting.value_text == "--"
    assert waiting.severity == "unknown"
    assert waiting.gpu_active is False
    assert mismatch.value_text == "Decoder mismatch · avdec_h264"
    assert mismatch.severity == "bad"


def test_nvidia_runtime_identity_parser_keeps_model_driver_and_device_identity(tmp_path):
    parsed = parse_nvidia_runtime_identity(
        "NVRM version: NVIDIA UNIX Open Kernel Module for x86_64  595.84  Release Build\n",
        "Model: NVIDIA GeForce RTX 4050 Laptop GPU\n"
        "GPU UUID: GPU-sentinel\n"
        "Bus Location: 0000:01:00.0\n",
    )
    assert parsed == {
        "available": True,
        "model": "NVIDIA GeForce RTX 4050 Laptop GPU",
        "driver_version": "595.84",
        "gpu_uuid": "GPU-sentinel",
        "bus_location": "0000:01:00.0",
    }

    missing = read_nvidia_runtime_identity(
        version_path=tmp_path / "missing-version",
        gpu_root=tmp_path / "missing-gpus",
    )
    assert missing["available"] is False
    assert missing["model"] == ""
    assert missing["error"]


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
