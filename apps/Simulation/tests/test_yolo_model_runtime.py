from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.yolo_model_runtime import YoloModelRuntime


class _Tensor:
    def __init__(self, values):
        self.values = np.asarray(values)

    def __getitem__(self, index):
        value = self.values[index]
        return _Tensor(value) if isinstance(value, np.ndarray) else value

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.values


class _Logger:
    def __init__(self):
        self.info_messages = []
        self.warning_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warning(self, message):
        self.warning_messages.append(message)


def test_node_compatibility_attributes_are_stored_by_runtime_owner():
    node = StereoDistanceNode.__new__(StereoDistanceNode)

    node.yolo_load_error = "load failed"
    node.yolo_cpu_fallback = True

    runtime = node._yolo_model_runtime()
    assert isinstance(runtime, YoloModelRuntime)
    assert runtime.yolo_load_error == "load failed"
    assert runtime.yolo_cpu_fallback is True
    assert "yolo_load_error" not in node.__dict__
    assert "yolo_cpu_fallback" not in node.__dict__


def test_detection_selection_retains_opencv_uv_and_bbox_pixel_order():
    node = SimpleNamespace()
    runtime = YoloModelRuntime(node)
    result = SimpleNamespace(
        boxes=[
            SimpleNamespace(
                cls=[2],
                conf=[0.8],
                xyxy=_Tensor([[10.0, 20.0, 30.0, 60.0]]),
            ),
            SimpleNamespace(
                cls=[2],
                conf=[0.9],
                xyxy=_Tensor([[12.0, 24.0, 36.0, 64.0]]),
            ),
        ]
    )

    detection = runtime._best_detection_from_result(result, [2])

    assert detection == {
        "cx": 24.0,
        "cy": 44.0,
        "bbox": (12.0, 24.0, 36.0, 64.0),
        "confidence": 0.9,
        "class_id": 2,
    }


def test_prediction_backend_verification_records_cuda_fp16_contract():
    logger = _Logger()
    node = SimpleNamespace(get_logger=lambda: logger)
    runtime = YoloModelRuntime(node)
    backend = SimpleNamespace(device="cuda:0", fp16=True)
    model = SimpleNamespace(predictor=SimpleNamespace(model=backend))

    runtime._verify_prediction_backend(model, "cuda:0")

    assert node.yolo_backend_verified is True
    assert node.yolo_backend_device == "cuda:0"
    assert node.yolo_backend_precision == "fp16"
    assert logger.warning_messages == []
    assert logger.info_messages == [
        "YOLO prediction backend verified: requested=cuda:0, actual=cuda:0, precision=fp16."
    ]


def test_prediction_backend_verification_warns_on_cuda_fp32_mismatch():
    logger = _Logger()
    node = SimpleNamespace(get_logger=lambda: logger)
    runtime = YoloModelRuntime(node)
    backend = SimpleNamespace(device="cpu", fp16=False)
    model = SimpleNamespace(predictor=SimpleNamespace(model=backend))

    runtime._verify_prediction_backend(model, "cuda:0")

    assert node.yolo_backend_device == "cpu"
    assert node.yolo_backend_precision == "fp32"
    assert logger.info_messages == []
    assert logger.warning_messages == [
        "YOLO prediction backend verified: requested=cuda:0, actual=cpu, precision=fp32."
    ]


def test_prediction_kwargs_filter_before_nms_and_retain_one_box_per_camera():
    node = SimpleNamespace(yolo_confidence=0.1, yolo_imgsz=736)
    runtime = YoloModelRuntime(node)
    source = [object(), object()]

    kwargs = runtime._prediction_kwargs(source, "cuda:0", [0])

    assert kwargs == {
        "source": source,
        "conf": 0.1,
        "device": "cuda:0",
        "imgsz": 736,
        "max_det": 1,
        "verbose": False,
        "classes": [0],
        "quantize": 16,
    }


def test_cuda_host_runtime_limits_cpu_pools(monkeypatch):
    logger = _Logger()
    node = SimpleNamespace(get_logger=lambda: logger)
    runtime = YoloModelRuntime(node)
    calls = []
    fake_torch = SimpleNamespace(
        set_num_threads=lambda count: calls.append(("intraop", count)),
        set_num_interop_threads=lambda count: calls.append(("interop", count)),
    )
    monkeypatch.setenv("SASL_YOLO_HOST_THREADS", "2")

    runtime._configure_yolo_host_runtime(fake_torch)
    runtime._configure_yolo_host_runtime(fake_torch)

    assert calls == [("intraop", 2), ("interop", 1)]
    assert node.yolo_host_intraop_threads == 2
    assert node.yolo_host_runtime_configured is True
    assert logger.info_messages == [
        "YOLO host runtime configured: PyTorch/OpenCV threads=2, PyTorch inter-op threads=1."
    ]


def test_cuda_process_isolation_defaults_on_and_can_be_disabled(monkeypatch):
    monkeypatch.delenv("SASL_YOLO_PROCESS_ISOLATION", raising=False)
    assert YoloModelRuntime._use_yolo_process("cuda:0") is True
    assert YoloModelRuntime._use_yolo_process("cpu") is False

    monkeypatch.setenv("SASL_YOLO_PROCESS_ISOLATION", "false")
    assert YoloModelRuntime._use_yolo_process("cuda:0") is False


def test_process_backend_verification_records_cuda_fp16_once():
    logger = _Logger()
    node = SimpleNamespace(
        yolo_backend_verified=False,
        get_logger=lambda: logger,
    )
    runtime = YoloModelRuntime(node)
    result = {"backend_device": "cuda:0", "backend_precision": "fp16"}

    runtime._verify_process_backend(result, "cuda:0")
    runtime._verify_process_backend(result, "cuda:0")

    assert node.yolo_backend_device == "cuda:0"
    assert node.yolo_backend_precision == "fp16"
    assert logger.warning_messages == []
    assert logger.info_messages == [
        "YOLO process backend verified: requested=cuda:0, actual=cuda:0, precision=fp16."
    ]


def test_process_shared_memory_preserves_rgb_pair_bytes_and_layout():
    node = SimpleNamespace(
        yolo_process_shared_memory=None,
        yolo_process_shared_layout=None,
    )
    runtime = YoloModelRuntime(node)
    left = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    right = (left + 50).astype(np.uint8)

    descriptors = runtime._copy_pair_to_shared_memory(left, right)
    try:
        copied = []
        for descriptor in descriptors:
            buffer = __import__(
                "multiprocessing.shared_memory", fromlist=["SharedMemory"]
            ).SharedMemory(name=descriptor["name"])
            try:
                copied.append(
                    np.ndarray(
                        descriptor["shape"],
                        dtype=np.dtype(descriptor["dtype"]),
                        buffer=buffer.buf,
                    ).copy()
                )
            finally:
                buffer.close()
        assert np.array_equal(copied[0], left)
        assert np.array_equal(copied[1], right)
        assert descriptors[0]["shape"] == (2, 4, 3)
        assert descriptors[0]["dtype"] == "|u1"
    finally:
        runtime._release_yolo_shared_memory()

    assert node.yolo_process_shared_memory is None
    assert node.yolo_process_shared_layout is None
