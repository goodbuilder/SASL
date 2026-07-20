from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "convert_truth_source_to_stereo_rdf.py"


def _module():
    spec = importlib.util.spec_from_file_location("truth_source_to_stereo_rdf_under_test", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _explicit_transform() -> dict[str, object]:
    return {
        "source_frame": "px4_local_position_ned",
        "source_axis_order": ["north", "east", "down"],
        "source_fields": ["truth_north_m", "truth_east_m", "truth_down_ned_m"],
        "origin_source_m": [10.0, 2.0, 1.0],
        "rotation_euler_zyx_deg_source_to_stereo_rdf": [30.0, -5.0, 2.0],
        "translation_rdf_m": [1.0, -2.0, 3.0],
    }


def test_explicit_ned_transform_outputs_named_stereo_center_rdf_fields() -> None:
    module = _module()
    converted, summary = module.convert_rows(
        [{"truth_north_m": "100", "truth_east_m": "20", "truth_down_ned_m": "5"}],
        _explicit_transform(),
        strict=True,
    )

    assert summary["converted_row_count"] == 1
    assert summary["rdf_axis_contract"] == "stereo_center_opencv_optical_rdf:RDF(right,down,forward)"
    assert converted[0]["truth_rdf_conversion_status"] == "CONVERTED"
    assert float(converted[0]["truth_right_m"]) != pytest.approx(20.0)
    assert float(converted[0]["truth_forward_m"]) != pytest.approx(100.0)
    assert "T_source_to_stereo_rdf" in converted[0]["truth_rdf_transform_contract"]


def test_direct_ned_axis_alias_is_rejected() -> None:
    module = _module()
    transform = {
        "source_frame": "px4_local_position_ned",
        "source_axis_order": ["north", "east", "down"],
        "source_fields": ["truth_north_m", "truth_east_m", "truth_down_ned_m"],
        "rotation_matrix_source_to_stereo_rdf": [
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        ],
    }

    with pytest.raises(ValueError, match="direct NED-to-RDF axis alias"):
        module.convert_rows(
            [{"truth_north_m": "1", "truth_east_m": "2", "truth_down_ned_m": "3"}],
            transform,
            strict=True,
        )


def test_transform_requires_source_frame_axis_order_and_fields() -> None:
    module = _module()
    transform = {
        "rotation_euler_zyx_deg_source_to_stereo_rdf": [12.0, -3.0, 1.5],
        "translation_rdf_m": [0.0, 0.0, 0.0],
    }

    with pytest.raises(ValueError, match="source_frame is required"):
        module.validate_transform(transform)
