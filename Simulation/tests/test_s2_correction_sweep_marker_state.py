from __future__ import annotations

from decimal import Decimal
import importlib.util
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_sweep import CameraContract, build_sweep_plan, write_plan


def _cli_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "run_correction_search_sweep.py"
    spec = importlib.util.spec_from_file_location("s2_r3_c1_marker_cli", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _plan():
    return build_sweep_plan(
        sweep_id="s",
        commit_sha="a" * 40,
        branch="sim/test",
        repository_name="repo",
        attitude_mode="ofat",
        axis_ranges=(),
        range_values=(Decimal("500"),),
        bearing_values=(Decimal("0"),),
        elevation_values=(Decimal("0"),),
        repetitions=1,
        seed=0,
        case_order="canonical",
        invalid_spatial_policy="fail",
        max_cases=2,
        contract=CameraContract(),
    )


def _running(tmp_path: Path):
    module = _cli_module()
    plan, cases = _plan()
    write_plan(tmp_path, plan, cases)
    module.enter_running_state(tmp_path)
    return module, plan, cases


def test_marker_state_transitions_have_exactly_one_stable_marker(tmp_path: Path) -> None:
    module, plan, cases = _running(tmp_path)
    assert module.inspect_sweep_marker_state(tmp_path) == "RUNNING"
    summary = module.finalize_sweep(
        output_dir=tmp_path,
        plan=plan,
        cases=cases,
        rows=[],
        counters={"launch_count": 0},
    )
    assert summary["run_valid"] is False
    assert module.inspect_sweep_marker_state(tmp_path) == "FINAL_INVALID"
    module.enter_running_state(tmp_path)
    assert module.inspect_sweep_marker_state(tmp_path) == "RUNNING"


@pytest.mark.parametrize("failure", ("csv", "summary", "hash", "replace"))
def test_finalization_fault_retains_incomplete_without_final_marker(
    tmp_path: Path, failure: str
) -> None:
    module, plan, cases = _running(tmp_path)

    def fail(*_args, **_kwargs):
        raise OSError(failure)

    kwargs = {}
    if failure == "csv":
        kwargs["case_csv_writer"] = fail
    elif failure == "summary":
        kwargs["json_writer"] = fail
    elif failure == "hash":
        kwargs["hash_file"] = fail
    else:
        kwargs["marker_replace"] = fail
    with pytest.raises(OSError, match=failure):
        module.finalize_sweep(
            output_dir=tmp_path,
            plan=plan,
            cases=cases,
            rows=[],
            counters={"launch_count": 0},
            **kwargs,
        )
    assert module.inspect_sweep_marker_state(tmp_path) == "RUNNING"


@pytest.mark.parametrize(
    "markers",
    (
        ("SWEEP_FINALIZED_VALID", "SWEEP_FINALIZED_INVALID"),
        ("SWEEP_FINALIZED_VALID", "SWEEP_INCOMPLETE"),
        ("SWEEP_FINALIZED_INVALID", "SWEEP_INCOMPLETE"),
    ),
)
def test_contradictory_markers_are_rejected_without_silent_cleanup(
    tmp_path: Path, markers: tuple[str, str]
) -> None:
    module = _cli_module()
    for marker in markers:
        (tmp_path / marker).touch()
    with pytest.raises(ValueError, match="contradictory"):
        module.inspect_sweep_marker_state(tmp_path)
    assert all((tmp_path / marker).exists() for marker in markers)


def test_final_marker_and_summary_contradiction_is_startup_error(tmp_path: Path) -> None:
    module = _cli_module()
    (tmp_path / "SWEEP_FINALIZED_VALID").touch()
    module.atomic_write_json(
        tmp_path / "sweep_summary.json",
        {"run_valid": False, "finalization_status": "INVALID"},
    )
    with pytest.raises(ValueError, match="contradicts"):
        module.validate_sweep_summary_marker(tmp_path, "FINAL_VALID")
