from types import SimpleNamespace
import threading
import time

import numpy as np

from wiris_stereo_distance.yolo_stream import (
    AdaptiveYoloRateLimiter,
    CPU_DECODE_HZ_CRITICAL_GUARD,
    CPU_DECODE_HZ_FLOOR,
    CPU_DECODE_HZ_RECOVERY,
    DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ,
    DEFAULT_CPU_YOLO_MAX_RATE_HZ,
    DEFAULT_CPU_YOLO_INITIAL_RATE_HZ,
    DEFAULT_CPU_WORKER_THREADS,
    DEFAULT_CUDA_WORKER_THREADS,
    DECODER_NVIDIA,
    NVIDIA_H264_DECODER,
    SOFTWARE_H264_DECODER,
    DEFAULT_RTSP_LATENCY_MS,
    GstRtpTimingTracker,
    STREAM_BACKEND_OPENCV_AUTO,
    STREAM_BACKEND_PYAV,
    RtcpSenderReportTiming,
    YoloStreamWorker,
    _process_frame_for_output,
    _run_opencv_stream,
    build_gst_pipeline_text,
    decoder_candidates,
    effective_yolo_max_rate_hz,
    estimate_rtcp_ntp_unix_seconds,
    ntp64_to_unix_seconds,
    normalize_yolo_input_size,
    normalize_rtsp_transport,
    normalize_stream_backend,
    rtp_timestamp_delta,
)


class _FakeScalar:
    def __init__(self, value) -> None:
        self.value = value

    def item(self):
        return self.value


class _FakeXyxy:
    def __init__(self, coords) -> None:
        self.coords = coords

    def __getitem__(self, _index):
        return self

    def tolist(self):
        return list(self.coords)


class _FakeBox:
    def __init__(self, coords, confidence, class_id) -> None:
        self.xyxy = _FakeXyxy(coords)
        self.conf = _FakeScalar(confidence)
        self.cls = _FakeScalar(class_id)


class _FakeModel:
    def __init__(self, boxes, names=None) -> None:
        self.boxes = boxes
        self.names = names or {0: "person", 1: "car", 2: "bottle"}

    def predict(self, *_args, **_kwargs):
        return [SimpleNamespace(boxes=self.boxes)]


class _CudaFailsFakeModel(_FakeModel):
    def __init__(self, boxes, names=None) -> None:
        super().__init__(boxes, names=names)
        self.devices = []

    def predict(self, *_args, **kwargs):
        device = kwargs.get("device")
        self.devices.append(device)
        if device == "cuda:0":
            raise RuntimeError("CUDA unavailable")
        return super().predict(*_args, **kwargs)


class _SlowFakeModel(_FakeModel):
    def predict(self, *_args, **_kwargs):
        time.sleep(0.03)
        return super().predict(*_args, **_kwargs)


class _FastFakeCapture:
    def __init__(self) -> None:
        self.released = False
        self.frame = np.zeros((24, 32, 3), dtype=np.uint8)

    def isOpened(self) -> bool:
        return True

    def set(self, *_args) -> bool:
        return True

    def get(self, prop) -> int:
        if prop == 3:
            return 32
        if prop == 4:
            return 24
        return 0

    def read(self):
        time.sleep(0.005)
        return True, self.frame.copy()

    def release(self) -> None:
        self.released = True


def _predict_config() -> dict:
    return {
        "input_size": 640,
        "conf_threshold": 0.5,
        "iou_threshold": 0.45,
        "device": "cuda:0",
    }


def test_gstreamer_pipeline_uses_configurable_low_latency_default() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible")

    assert f"latency={DEFAULT_RTSP_LATENCY_MS}" in pipeline
    assert "protocols=udp" in pipeline
    assert "drop-on-latency=true" in pipeline
    assert "do-rtcp=true" in pipeline
    assert "rtspsrc name=src" in pipeline
    assert "rtph264depay name=depay source-info=true" in pipeline
    assert "queue leaky=downstream max-size-buffers=1" in pipeline
    assert NVIDIA_H264_DECODER in pipeline
    assert SOFTWARE_H264_DECODER not in pipeline
    assert "appsink name=sink" in pipeline


def test_rtcp_sender_report_ntp_estimate_handles_rtp_wraparound() -> None:
    ntp_raw = (2_208_988_800 + 100) << 32

    assert ntp64_to_unix_seconds(ntp_raw) == 100.0
    assert rtp_timestamp_delta(10, 0xFFFF_FFF0) == 26
    assert (
        estimate_rtcp_ntp_unix_seconds(
            rtp_timestamp=10,
            sr_rtp_timestamp=0xFFFF_FFF0,
            sr_ntp_unix_s=100.0,
            clock_rate_hz=90_000,
        )
        == 100.0 + 26 / 90_000
    )


def test_rtcp_sender_report_interval_logging_triggers_on_new_sr() -> None:
    messages: list[str] = []
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace(), status_callback=messages.append)

    first = RtcpSenderReportTiming(
        ssrc=1234,
        ntp_raw=1,
        ntp_unix_s=100.0,
        rtp_timestamp=10,
        clock_rate_hz=90_000,
        received_monotonic_s=1.0,
    )
    second = RtcpSenderReportTiming(
        ssrc=1234,
        ntp_raw=2,
        ntp_unix_s=102.0,
        rtp_timestamp=20,
        clock_rate_hz=90_000,
        received_monotonic_s=3.5,
    )

    assert tracker._register_sender_report(first) is None
    message = tracker._register_sender_report(second)
    assert message == (
        "[RTCP] Sender Report 갱신: ssrc=1234, interval=2.50s, ntp=102.000000"
    )
    tracker._emit_status(message)
    assert messages == [message]


def test_gstreamer_pipeline_can_drop_delta_frames_for_keyframe_only_diagnostics() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible", keyframe_only=True)

    assert "h264parse config-interval=-1 disable-passthrough=true" in pipeline
    assert "video/x-h264,alignment=au" in pipeline
    assert "identity drop-buffer-flags=delta-unit" in pipeline


def test_gstreamer_pipeline_can_use_nvidia_decoder() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible", decoder="nvidia")

    assert NVIDIA_H264_DECODER in pipeline
    assert SOFTWARE_H264_DECODER not in pipeline


def test_gstreamer_pipeline_can_use_software_decoder() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible", decoder="software")

    assert SOFTWARE_H264_DECODER in pipeline
    assert "skip-frame=1" in pipeline
    assert NVIDIA_H264_DECODER not in pipeline


def test_decoder_candidates_auto_falls_back_to_software() -> None:
    assert decoder_candidates("auto") == (NVIDIA_H264_DECODER, SOFTWARE_H264_DECODER)
    assert decoder_candidates("software") == (SOFTWARE_H264_DECODER,)


def test_stream_backend_normalization() -> None:
    assert normalize_stream_backend("gstreamer") == "gstreamer"
    assert normalize_stream_backend("gst") == "gstreamer"
    assert normalize_stream_backend("opencv") == "opencv"
    assert normalize_stream_backend("ffmpeg") == "opencv"
    assert normalize_stream_backend("opencv-auto") == STREAM_BACKEND_OPENCV_AUTO
    assert normalize_stream_backend("cap_any") == STREAM_BACKEND_OPENCV_AUTO
    assert normalize_stream_backend("pyav") == STREAM_BACKEND_PYAV
    assert normalize_stream_backend("av") == STREAM_BACKEND_PYAV
    assert normalize_stream_backend("unknown") == "gstreamer"


def test_rtsp_transport_normalization() -> None:
    assert normalize_rtsp_transport("udp") == "udp"
    assert normalize_rtsp_transport("tcp") == "tcp"
    assert normalize_rtsp_transport("interleaved") == "tcp"
    assert normalize_rtsp_transport("unknown") == "udp"


def test_cpu_yolo_detection_gets_safe_default_rate_limit() -> None:
    assert effective_yolo_max_rate_hz(0.0, device="cpu", enable_detection=True) == DEFAULT_CPU_YOLO_MAX_RATE_HZ
    assert effective_yolo_max_rate_hz(None, device="", enable_detection=True) == DEFAULT_CPU_YOLO_MAX_RATE_HZ
    assert effective_yolo_max_rate_hz(15.0, device="cpu", enable_detection=True) == 15.0
    assert effective_yolo_max_rate_hz(0.0, device="cuda:0", enable_detection=True) == 0.0
    assert effective_yolo_max_rate_hz(0.0, device="cpu", enable_detection=False) == 0.0


def test_yolo_input_size_is_rounded_up_to_stride_multiple() -> None:
    assert normalize_yolo_input_size(640) == 640
    assert normalize_yolo_input_size(720) == 736
    assert normalize_yolo_input_size("bad") == 640


def test_cpu_yolo_adaptive_rate_starts_safe_then_raises_and_lowers() -> None:
    limiter = AdaptiveYoloRateLimiter.from_config(
        {"max_rate_hz": DEFAULT_CPU_YOLO_MAX_RATE_HZ, "adaptive_rate": True},
        enable_detection=True,
    )

    assert limiter.enabled
    assert limiter.target_rate_hz == DEFAULT_CPU_YOLO_INITIAL_RATE_HZ
    assert limiter.max_rate_hz == DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ

    assert not limiter.update(decoded_hz=0.2, inferred_hz=0.0)
    assert limiter.target_rate_hz == DEFAULT_CPU_YOLO_INITIAL_RATE_HZ

    limiter.update(decoded_hz=CPU_DECODE_HZ_RECOVERY, inferred_hz=5.0)
    assert limiter.target_rate_hz > DEFAULT_CPU_YOLO_INITIAL_RATE_HZ

    raised_rate = limiter.target_rate_hz
    limiter.update(decoded_hz=CPU_DECODE_HZ_FLOOR, inferred_hz=5.0)
    assert limiter.target_rate_hz < raised_rate

    limiter.target_rate_hz = raised_rate
    limiter.update(decoded_hz=CPU_DECODE_HZ_CRITICAL_GUARD - 0.1, inferred_hz=5.0)
    assert limiter.target_rate_hz == limiter.min_rate_hz


def test_yolo_worker_passes_rtsp_latency_to_runtime_config() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=30.0,
        display_rate_hz=15.0,
        preview_max_width=960,
        rtsp_latency_ms=25,
        decoder="software",
        stream_backend="opencv",
        rtsp_transport="tcp",
        enable_detection=False,
        record_video_path="/tmp/stereo-left.mp4",
        record_video_fps=24.0,
    )

    assert worker.config["rtsp_latency_ms"] == 25
    assert worker.config["decoder"] == "software"
    assert worker.config["stream_backend"] == "opencv"
    assert worker.config["rtsp_transport"] == "tcp"
    assert worker.config["adaptive_rate"] is False
    assert worker.config["worker_cpu_threads"] == DEFAULT_CPU_WORKER_THREADS
    assert worker.config["record_video_path"] == "/tmp/stereo-left.mp4"
    assert worker.config["record_video_fps"] == 24.0
    pipeline = build_gst_pipeline_text(
        worker.config["url"],
        latency_ms=worker.config["rtsp_latency_ms"],
        decoder=worker.config["decoder"],
        transport=worker.config["rtsp_transport"],
    )
    assert "latency=25" in pipeline
    assert "protocols=tcp" in pipeline


def test_yolo_worker_uses_cuda_thread_default_without_rate_limit() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cuda:0",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=15.0,
        preview_max_width=960,
        enable_detection=True,
    )

    assert worker.config["max_rate_hz"] == 0.0
    assert worker.config["adaptive_rate"] is False
    assert worker.config["worker_cpu_threads"] == DEFAULT_CUDA_WORKER_THREADS


def test_yolo_worker_defaults_to_nvidia_decoder() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cuda:0",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=15.0,
        preview_max_width=960,
        enable_detection=True,
    )

    assert worker.config["decoder"] == DECODER_NVIDIA


def test_yolo_processing_skips_preview_overlay_when_not_displaying() -> None:
    import numpy as np

    class NoDrawCv2:
        def __getattr__(self, name):
            raise AssertionError(f"unexpected drawing call: {name}")

    class FakeOutputQueue:
        def __init__(self) -> None:
            self.messages = []

        def put(self, message) -> None:
            self.messages.append(message)

    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    output_queue = FakeOutputQueue()

    annotated, best_detection, inferred = _process_frame_for_output(
        NoDrawCv2(),
        frame,
        model=_FakeModel([_FakeBox((1, 2, 3, 4), 0.9, 2)]),
        config=_predict_config(),
        output_queue=output_queue,
        now_s=10.0,
        enable_detection=True,
        draw_preview_overlay=False,
    )

    assert annotated is frame
    assert best_detection is not None
    assert inferred is True
    assert output_queue.messages[0][0] == "detection"


def test_yolo_processing_falls_back_to_cpu_when_cuda_predict_fails() -> None:
    import numpy as np

    class FakeOutputQueue:
        def __init__(self) -> None:
            self.messages = []

        def put(self, message) -> None:
            self.messages.append(message)

    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    output_queue = FakeOutputQueue()
    model = _CudaFailsFakeModel([_FakeBox((1, 2, 3, 4), 0.9, 2)])
    config = _predict_config()

    annotated, best_detection, inferred = _process_frame_for_output(
        SimpleNamespace(),
        frame,
        model=model,
        config=config,
        output_queue=output_queue,
        now_s=10.0,
        enable_detection=True,
        draw_preview_overlay=False,
    )

    assert annotated is frame
    assert best_detection is not None
    assert inferred is True
    assert model.devices == ["cuda:0", "cpu"]
    assert config["device"] == "cpu"
    assert any(message[0] == "status" and "CPU" in message[1] for message in output_queue.messages)
    assert output_queue.messages[-1][0] == "detection"


def test_yolo_processing_emits_measurement_frame_before_detection_when_enabled() -> None:
    import numpy as np

    class FakeCv2:
        COLOR_BGR2RGB = 4

        def cvtColor(self, image, code):
            assert code == self.COLOR_BGR2RGB
            return image

    class FakeOutputQueue:
        def __init__(self) -> None:
            self.messages = []

        def put(self, message) -> None:
            self.messages.append(message)

    frame = np.zeros((4, 5, 3), dtype=np.uint8)
    output_queue = FakeOutputQueue()
    config = {
        **_predict_config(),
        "emit_measurement_frames": True,
        "preview_max_width": 960,
    }

    annotated, best_detection, inferred = _process_frame_for_output(
        FakeCv2(),
        frame,
        model=_FakeModel([_FakeBox((1, 2, 3, 4), 0.9, 2)]),
        config=config,
        output_queue=output_queue,
        now_s=10.0,
        enable_detection=True,
        draw_preview_overlay=False,
    )

    assert annotated is frame
    assert best_detection is not None
    assert inferred is True
    assert output_queue.messages[0][:4] == ("measurement_frame", 10.0, 5, 4)
    assert output_queue.messages[1][0] == "detection"


def test_yolo_processing_draws_only_best_box_for_preview_overlay() -> None:
    import numpy as np

    class FakeCv2:
        FONT_HERSHEY_SIMPLEX = 0
        LINE_AA = 16
        MARKER_CROSS = 0

        def __init__(self) -> None:
            self.rectangles = []
            self.markers = []
            self.texts = []

        def getTextSize(self, *_args):
            return (20, 8), 2

        def rectangle(self, _image, top_left, bottom_right, *_args):
            self.rectangles.append((top_left, bottom_right))

        def putText(self, _image, text, *_args):
            self.texts.append(text)
            return None

        def drawMarker(self, _image, position, *_args, **_kwargs):
            self.markers.append(position)

    class FakeOutputQueue:
        def put(self, _message) -> None:
            return None

    cv2 = FakeCv2()
    frame = np.zeros((80, 80, 3), dtype=np.uint8)

    annotated, best_detection, inferred = _process_frame_for_output(
        cv2,
        frame,
        model=_FakeModel(
            [
                _FakeBox((1, 2, 3, 4), 0.2, 1),
                _FakeBox((30, 40, 50, 60), 0.95, 2),
            ],
            names={1: "car", 2: "person"},
        ),
        config=_predict_config(),
        output_queue=FakeOutputQueue(),
        now_s=10.0,
        enable_detection=True,
        draw_preview_overlay=True,
    )

    assert annotated is not frame
    assert best_detection is not None
    assert best_detection.class_id == 2
    assert best_detection.class_name == "person"
    assert inferred is True
    assert ((30, 40), (50, 60)) in cv2.rectangles
    assert ((1, 2), (3, 4)) not in cv2.rectangles
    assert cv2.markers == [(40, 50)]
    assert any(text.startswith("BEST person 0.95") for text in cv2.texts)


def test_yolo_worker_enables_cpu_adaptive_rate_limit_by_default() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=20.0,
        preview_max_width=960,
        enable_detection=True,
    )

    limiter = AdaptiveYoloRateLimiter.from_config(worker.config, enable_detection=True)
    assert worker.config["max_rate_hz"] == DEFAULT_CPU_YOLO_MAX_RATE_HZ
    assert worker.config["adaptive_rate"] is True
    assert worker.config["adaptive_max_rate_hz"] == DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ
    assert limiter.enabled
    assert limiter.target_rate_hz == DEFAULT_CPU_YOLO_INITIAL_RATE_HZ
    assert limiter.max_rate_hz == DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ


def test_yolo_worker_normalizes_input_size_before_predict() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=720,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=20.0,
        preview_max_width=960,
        enable_detection=True,
    )

    assert worker.config["input_size"] == 736


def test_yolo_worker_emits_frame_metadata(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=30.0,
        display_rate_hz=15.0,
        preview_max_width=960,
        rtsp_latency_ms=25,
        decoder="software",
        stream_backend="opencv",
        rtsp_transport="tcp",
        enable_detection=False,
    )
    received = []
    worker.frame_ready.connect(lambda image, metadata: received.append((image, metadata)))

    worker._handle_worker_message(
        (
            "frame",
            {"timestamp_s": 123.0, "source_timestamp_s": 5.5, "timestamp_source": "gstreamer-pts"},
            2,
            1,
            bytes([255, 0, 0, 0, 255, 0]),
        )
    )

    assert len(received) == 1
    image, metadata = received[0]
    assert image.width() == 2
    assert image.height() == 1
    assert metadata["timestamp_s"] == 123.0
    assert metadata["source_timestamp_s"] == 5.5
    assert metadata["timestamp_source"] == "gstreamer-pts"
    _ = app


def test_yolo_worker_emits_target_rate_in_stats(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=20.0,
        preview_max_width=960,
        enable_detection=True,
    )
    received = []
    worker.stats_ready.connect(received.append)

    worker._handle_worker_message(("stats", 18.0, 7.0, 7.0, 1280, 720, 9.5))

    assert len(received) == 1
    assert received[0]["decoded_hz"] == 18.0
    assert received[0]["inferred_hz"] == 7.0
    assert received[0]["yolo_target_hz"] == 9.5
    _ = app


def test_opencv_receive_hz_counts_reader_frames_not_yolo_limited_frames() -> None:
    stop_event = threading.Event()
    capture = _FastFakeCapture()
    stats_messages = []

    class _StatsQueue:
        def put(self, message):
            if message[0] == "stats":
                stats_messages.append(message)
                stop_event.set()

    fake_cv2 = SimpleNamespace(
        CAP_ANY=0,
        CAP_FFMPEG=1,
        CAP_PROP_BUFFERSIZE=38,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        COLOR_BGR2RGB=4,
        VideoCapture=lambda *_args: capture,
        cvtColor=lambda frame, _code: frame,
    )

    _run_opencv_stream(
        {
            "url": "rtsp://example/stream",
            "rtsp_transport": "tcp",
            "stream_backend": "opencv",
            "display_rate_hz": 0.0,
            "max_rate_hz": 5.0,
            "adaptive_rate": False,
            "input_size": 640,
            "conf_threshold": 0.5,
            "iou_threshold": 0.45,
            "device": "cpu",
            "preview_max_width": 0,
        },
        _StatsQueue(),
        stop_event,
        fake_cv2,
        _SlowFakeModel([]),
        True,
    )

    assert capture.released
    assert stats_messages
    _kind, decoded_hz, _displayed_hz, inferred_hz, width, height, _target_rate_hz = stats_messages[0]
    assert width == 32
    assert height == 24
    assert decoded_hz > inferred_hz * 2.0
