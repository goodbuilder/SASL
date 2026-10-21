"""Original pixel bbox and conservative uniform-scene audit, never YOLO."""
import numpy as np
from campaigns.stereo_v1_1.mesh_visibility_audit import inspect_pixels


def image():
    return np.full((160, 240, 3), 180, dtype=np.uint8)


def test_uniform_background_with_small_foreground_has_original_bbox():
    pixels = image()
    pixels[70:80, 100:110] = 10
    result = inspect_pixels(pixels)
    assert result['status'] == 'VISIBLE'
    assert result['bbox_xyxy_original_px'] == [100, 70, 110, 80]
    assert result['minimum_edge_margin_px'] == 70


def test_border_foreground_is_clipped():
    pixels = image()
    pixels[70:80, 0:5] = 0
    assert inspect_pixels(pixels)['status'] == 'CLIPPED'


def test_margin_shortfall_and_no_foreground_fail():
    pixels = image()
    assert inspect_pixels(pixels)['status'] == 'NO_FOREGROUND'
    pixels[70:80, 20:30] = 0
    assert inspect_pixels(pixels)['status'] == 'MARGIN_VIOLATION'


def test_nonuniform_scene_is_unsupported():
    pixels = image()
    pixels[80:] = 70
    assert inspect_pixels(pixels)['status'] == 'UNSUPPORTED_SCENE'


def test_widely_separated_foregrounds_not_assumed_single_drone():
    pixels = image()
    pixels[70:75, 50:55] = 0
    pixels[70:75, 180:185] = 0
    assert inspect_pixels(pixels)['status'] == 'UNSUPPORTED_SCENE'
