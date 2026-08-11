from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
RUNTIME_PATH = ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "yolo_model_runtime.py"


def test_cuda_yolo_prediction_uses_quantize_not_deprecated_half_flag() -> None:
    source = RUNTIME_PATH.read_text(encoding="utf-8")

    assert 'kwargs["quantize"] = 16' in source
    assert 'predict_kwargs["half"]' not in source
