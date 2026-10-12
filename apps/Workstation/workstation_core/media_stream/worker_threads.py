"""worker threads owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import os
from typing import Any
from .configuration import _device_uses_cpu
from .policy import DEFAULT_CPU_WORKER_THREADS, DEFAULT_CUDA_WORKER_THREADS


def _worker_cpu_threads(config: dict[str, Any]) -> int:
    env_value = os.environ.get("STEREO_YOLO_WORKER_THREADS")
    raw_value = env_value if env_value not in (None, "") else config.get("worker_cpu_threads")
    try:
        value = int(raw_value)
    except (TypeError, ValueError):
        value = DEFAULT_CPU_WORKER_THREADS
    return max(1, min(8, value))


def _default_worker_cpu_threads(device: str | None) -> int:
    return DEFAULT_CPU_WORKER_THREADS if _device_uses_cpu(device) else DEFAULT_CUDA_WORKER_THREADS


def _configure_worker_thread_environment(thread_count: int) -> None:
    value = str(max(1, int(thread_count)))
    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[name] = value


def _configure_cv2_threads(cv2, thread_count: int) -> None:
    try:
        cv2.setNumThreads(max(1, int(thread_count)))
    except Exception:
        pass
    try:
        cv2.ocl.setUseOpenCL(False)
    except Exception:
        pass


def _configure_torch_threads(thread_count: int) -> None:
    try:
        import torch
    except Exception:
        return
    thread_count = max(1, int(thread_count))
    try:
        torch.set_num_threads(thread_count)
    except Exception:
        pass
    try:
        torch.set_num_interop_threads(1)
    except Exception:
        pass
