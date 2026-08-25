from __future__ import annotations

import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "benchmark_yolo_pair.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("benchmark_yolo_pair_under_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_percentile_interpolates_without_numpy() -> None:
    module = _load_module()

    assert module.percentile([10.0, 20.0, 30.0], 0.5) == 20.0
    assert module.percentile([10.0, 20.0], 0.95) == 19.5
    assert math.isnan(module.percentile([], 0.95))


def test_acceptance_requires_actual_cuda_fp16_and_both_rate_gates() -> None:
    module = _load_module()
    passing = {
        "requested_device": "cuda:0",
        "backend_device": "cuda:0",
        "backend_precision": "fp16",
        "cuda_tensor_probe": True,
        "pair_update_hz": 18.0,
        "minimum_pair_update_hz": 15.0,
        "wall_time_ms_p95": 55.0,
        "maximum_p95_ms": 66.7,
        "timed_iterations": 60,
    }

    assert module.evaluate_acceptance(passing) == []

    failing = dict(passing, backend_device="cpu", backend_precision="fp32", pair_update_hz=10.0, wall_time_ms_p95=90.0)
    errors = module.evaluate_acceptance(failing)
    assert "actual backend is not CUDA" in errors
    assert "actual backend precision is not FP16" in errors
    assert "pair update throughput is below the minimum" in errors
    assert "pair inference wall-time p95 exceeds the maximum" in errors
