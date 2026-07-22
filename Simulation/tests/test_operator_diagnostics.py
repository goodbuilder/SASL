from stereo_ground_sim.operator_diagnostics import build_pair_diagnostics

def src():
    return {"epoch": 1, "pair_id": "p1", "pair_midpoint_stamp_ns": 2,
            "terminal_status": "evaluated", "measurement_status": "processed",
            "evaluation_status": "evaluated", "output_frame": "baseline_center_frd",
            "position_unit": "m", "angle_unit": "deg", "raw_range_error_m": 1.0}

def test_identity_status_and_missing_metrics():
    x = src(); d = build_pair_diagnostics(pair=x, measurement=x, alignment=x)
    assert d and d.values == (("raw_range_error_m", 1.0),)
    assert build_pair_diagnostics(pair={**x, "pair_id": ""}, measurement=x, alignment=x) is None
    assert build_pair_diagnostics(pair={**x, "terminal_status": "valid"}, measurement=x, alignment=x) is None

def test_identity_mismatch_and_immutable_values():
    x = src(); d = build_pair_diagnostics(pair=x, measurement={**x, "epoch": 2}, alignment=x)
    assert d is None
    d = build_pair_diagnostics(pair=x, measurement=x, alignment=x)
    assert d.value_map() == {"raw_range_error_m": 1.0}
    d.value_map()["raw_range_error_m"] = 99.0
    assert d.value_map()["raw_range_error_m"] == 1.0
