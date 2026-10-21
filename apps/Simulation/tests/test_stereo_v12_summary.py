"""Workbook capture acceptance must survive the handoff summary boundary."""
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1.summarize_campaign import (
    summarize, reference_flight_cost, actual_science_flight_cost,
)


@pytest.mark.parametrize("flags,accepted", [
    ({"counted_in_campaign": True, "counted_in_39": False}, True),
    ({"counted_in_campaign": False, "counted_in_39": True}, False),
    ({"counted_in_39": True}, True),
])
@pytest.mark.parametrize("science_completed", [False, True])
def test_summary_uses_authoritative_count_flag_and_keeps_v0_blind(tmp_path, flags, accepted, science_completed):
    def save(name, value):
        (tmp_path / name).write_text(json.dumps(value))
    rows = [dict(run_id=f"R{i}", rig_group="G0", path_id="V0", repetition=i+1,
                 seed_requested=i, status="PLANNED", attempt_count=0,
                 recording_path=None, failure_reason=None) for i in range(39)]
    rows[0].update(status="VALID_WITH_WARNINGS", attempt_count=1,
                   recording_path=".", attempts=[{"recording_path": "."}])
    save("run_status.json", {"planned_run_count": 39, "valid_run_count": 1, "run_slots": rows})
    save("qc_report.json", {"status": "VALID_WITH_WARNINGS", "warnings": ["entry_failed"], **flags})
    save("flight_reference.json", {"science_duration_s": 47.5, "length_3d_m": 212.8,
         "samples": [{"position_world_enu_m": [e,600,218]} for e in (0,3,-1,0)]})
    save("controller_result.json", {"controller_status": "science_completed" if science_completed else "failed",
         "failure_reason": None if science_completed else "entry failed"})
    save("observation_summary.json", {"counts": {side: {"source_frames": 1, "detected": 0}
         for side in ("left", "right")}, "nominal_pair_count": 1, "both_detected_pair_count": 0})
    save("image_index_summary.json", {**{side: {"observed_source_hz": 20, "unmatched_frame_count": 0}
         for side in ("left", "right")}, "maximum_abs_pair_delta_ns": 0})
    save("state_capture_summary.json", {"gt_observed_hz": 50, "gt_gap_ge_40ms_count": 0})
    save("gt_alignment_summary.json", {"science_image_count_by_alignment_status": {},
         "science_image_count_total": 0, "tracking_metrics_revealed": False})
    (tmp_path / "events.jsonl").write_text("" if not science_completed else
        '\n'.join(json.dumps(dict(event=event,simulation_epoch=0,simulation_timestamp_ns=stamp))
                  for event,stamp in [("science_start",0),("science_end",60_000_000)]))
    (tmp_path / "paired_observations.jsonl").write_text('')
    (tmp_path / "ground_truth.jsonl").write_text('\n'.join(json.dumps(row) for row in _cost_gt()))
    if not accepted:
        with pytest.raises(ValueError, match="status/QC disagreement"):
            summarize(tmp_path, allow_partial=True)
    else:
        result = summarize(tmp_path, allow_partial=True)
        assert result["valid_run_count"] == 1
        assert result["by_path"][0]["science_completed_count"] == int(science_completed)
        assert result["v0_positioning_metrics_revealed"] is False
        assert result["runs"][0]["tracking_3d_rmse_m_calibration_only"] is None
        assert result["runs"][0]["science_reference_lateral_total_travel_m"] == 8
        assert result["by_path"][0]["science_reference_lateral_span_m_mean"] == 4
        if science_completed:
            assert result["runs"][0]["science_actual_sampled_lateral_total_travel_m"] == 8
            assert result["by_path"][0]["science_actual_sampled_length_3d_m_mean"] == 8
            assert result["by_path"][0]["science_actual_duration_s_mean"] == .06



def _cost_gt():
    return [dict(simulation_epoch=0,source_timestamp_ns=i*20_000_000,
                 position_world_enu_m=[east,600,218],
                 orientation_world_from_reference_xyzw=[0,0,0,1])
            for i,east in enumerate([0,3,-1,0])]


def _boundary(stamp,epoch=0):
    return dict(simulation_epoch=epoch,simulation_timestamp_ns=stamp)


def test_nonmonotone_lateral_total_travel_span_and_signed_net_are_distinct():
    reference={'samples':[{'position_world_enu_m':r['position_world_enu_m']} for r in _cost_gt()]}
    cost=reference_flight_cost(reference)
    assert cost=={'science_reference_lateral_total_travel_m':8.,
                  'science_reference_lateral_span_m':4.,
                  'science_reference_lateral_net_displacement_m':0.}
    actual=actual_science_flight_cost(_cost_gt(),_boundary(0),_boundary(60_000_000))
    assert actual['science_actual_sampled_length_3d_m']==8
    assert actual['science_actual_sampled_lateral_total_travel_m']==8
    assert actual['science_actual_sampled_lateral_span_m']==4
    assert actual['science_actual_sampled_lateral_net_displacement_m']==0
    assert actual['science_actual_duration_s']==.06


def test_actual_cost_interpolates_phase_boundaries_without_other_epoch_or_outer_phase():
    rows=_cost_gt()
    rows += [dict(rows[0],source_timestamp_ns=-20_000_000,position_world_enu_m=[999,600,218]),
             dict(rows[-1],source_timestamp_ns=80_000_000,position_world_enu_m=[-999,600,218]),
             dict(rows[1],simulation_epoch=1,position_world_enu_m=[99999,600,218])]
    result=actual_science_flight_cost(rows,_boundary(10_000_000),_boundary(50_000_000))
    assert result['science_actual_duration_s']==.04
    assert result['science_actual_sampled_length_3d_m']==6
    assert result['science_actual_sampled_lateral_total_travel_m']==6
    assert result['science_actual_sampled_lateral_span_m']==4
    assert result['science_actual_sampled_lateral_net_displacement_m']==-2
    assert result['science_actual_sampled_point_count']==4


@pytest.mark.parametrize('start,end', [(_boundary(-1),_boundary(60_000_000)),
    (_boundary(0),_boundary(60_000_001)),(_boundary(0),_boundary(60_000_000,1)),
    (_boundary(20_000_000),_boundary(20_000_000))])
def test_actual_cost_rejects_unbracketed_cross_epoch_or_invalid_boundaries(start,end):
    with pytest.raises(ValueError):
        actual_science_flight_cost(_cost_gt(),start,end)


def test_actual_cost_rejects_missing_interior_gt_and_accepts_exact_40ms_bracket():
    rows=_cost_gt()
    with pytest.raises(ValueError,match='gap exceeds 40 ms'):
        actual_science_flight_cost([rows[0],rows[-1]],_boundary(0),_boundary(60_000_000))
    result=actual_science_flight_cost([rows[0],rows[2]],_boundary(10_000_000),_boundary(30_000_000))
    assert result['science_actual_sampled_lateral_total_travel_m']==.5
    with pytest.raises(ValueError,match='boundary alignment failed'):
        actual_science_flight_cost([rows[0],rows[-1]],_boundary(10_000_000),_boundary(50_000_000))


def test_reference_samples_cannot_be_omitted_from_cost_handoff():
    with pytest.raises(ValueError,match='reference samples are required'):
        reference_flight_cost({'science_duration_s':1,'length_3d_m':5})
