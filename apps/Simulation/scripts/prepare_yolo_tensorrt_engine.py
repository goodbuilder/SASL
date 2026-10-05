#!/usr/bin/env python3
"""Build a GPU-specific TensorRT FP16 YOLO engine from a tracked .pt source."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True, help="Source Ultralytics .pt model")
    parser.add_argument("--engine", type=Path, required=True, help="Generated TensorRT .engine output")
    parser.add_argument("--imgsz", type=int, required=True)
    parser.add_argument("--batch", type=int, default=2, help="Stereo pair batch size")
    parser.add_argument("--device", default="cuda:0")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.model.expanduser().resolve()
    output = args.engine.expanduser().resolve()
    if source.suffix.lower() != ".pt" or not source.is_file():
        raise SystemExit(f"TensorRT source model must be an existing .pt file: {source}")
    if args.imgsz <= 0 or args.batch <= 0:
        raise SystemExit("--imgsz and --batch must be positive")

    try:
        import tensorrt  # noqa: F401  # validates the Python runtime before export.
        import torch
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            "TensorRT FP16 requires the same Python runtime to provide "
            f"TensorRT, PyTorch, and Ultralytics: {exc}"
        ) from exc
    if not args.device.startswith("cuda") or not torch.cuda.is_available():
        raise SystemExit("TensorRT FP16 export requires an available CUDA device")

    model = YOLO(str(source))
    exported = Path(
        model.export(
            format="engine",
            imgsz=args.imgsz,
            batch=args.batch,
            device=args.device,
            half=True,
            dynamic=False,
            simplify=True,
            verbose=False,
        )
    ).resolve()
    if not exported.is_file():
        raise SystemExit(f"Ultralytics did not produce a TensorRT engine: {exported}")
    output.parent.mkdir(parents=True, exist_ok=True)
    if exported != output:
        shutil.move(str(exported), str(output))
    print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
