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
