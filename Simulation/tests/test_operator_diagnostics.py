from stereo_ground_sim.operator_diagnostics import build_pair_diagnostics


def _final():
    return {"terminal_status":"evaluated", "epoch":1, "pair_id":"p1", "pair_midpoint_stamp_ns":2,
            "output_frame":"baseline_center_frd", "position_unit":"m", "angle_unit":"deg",
            **{k: 1.0 for k in ("raw_position_error_m","corrected_position_error_m","raw_forward_error_m","corrected_forward_error_m","raw_right_error_m","corrected_right_error_m","raw_down_error_m","corrected_down_error_m","raw_range_error_m","corrected_range_error_m","raw_ray_gap_m","corrected_ray_gap_m","raw_intersection_angle_deg","corrected_intersection_angle_deg")}}


def test_pair_snapshot_requires_identity_and_finite_values():
    result = build_pair_diagnostics(_final())
    assert result is not None and result.pair_id == "p1"
    bad = _final(); bad["pair_id"] = ""
    assert build_pair_diagnostics(bad) is not None
    bad = _final(); bad["raw_ray_gap_m"] = float("nan")
    assert build_pair_diagnostics(bad) is None


def test_pair_snapshot_rejects_mixed_or_invalid_status():
    bad = _final(); bad["terminal_status"] = "gt_unavailable"
    assert build_pair_diagnostics(bad) is None
    bad = _final(); bad["output_frame"] = "world"
    assert build_pair_diagnostics(bad) is None
