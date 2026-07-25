from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
NODE_PATH = ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py"


def test_cuda_yolo_prediction_uses_quantize_not_deprecated_half_flag() -> None:
    source = NODE_PATH.read_text(encoding="utf-8")

    assert source.count('predict_kwargs["quantize"] = 16') == 3
    assert 'predict_kwargs["half"]' not in source
