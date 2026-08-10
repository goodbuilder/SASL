from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import shutil
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))


SHA = "b" * 40
BRANCH = "sim/s2-r4a-test"
AUTHORITATIVE_ROOT = Path(
    os.environ.get(
        "SASL_QUALIFICATION_FIXTURE",
        Path(__file__).resolve().parent / "fixtures/qualification/exact-profile-freeze-88a360f",
    )
)


def _cli_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "run_sealed_correction_qualification.py"
    name = "s2_r4a_cli_test"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _exact_args() -> list[str]:
    return ["--expected-commit-sha", SHA, "--expected-branch", BRANCH]


def test_parser_exposes_read_only_anchor_command_without_selection_overrides() -> None:
    module = _cli_module()
    parser = module.build_parser()
    prepare = parser.parse_args(
        [
            "prepare-plans",
            "--output-dir",
            "/tmp/r4a-plan",
            "--qualification-id",
            "qid",
            "--repository-name",
            "repo",
            *_exact_args(),
        ]
    )
    assert prepare.command == "prepare-plans"
    for command in (
        "fit-select-freeze",
        "status",
        "verify-freeze-anchor",
        "unseal-evaluate",
    ):
        assert command in parser.format_help()
    for forbidden in (
        "sealed-runtime-root",
        "method-family",
        "validation-threshold",
        "matrix-override",
    ):
        assert forbidden not in {action.dest.replace("_", "-") for action in parser._actions}


def test_fit_cli_has_no_sealed_runtime_or_family_override_argument() -> None:
    module = _cli_module()
    with pytest.raises(SystemExit):
        module.parse_args(
            [
                "fit-select-freeze",
                "--qualification-root",
                "/tmp/q",
                "--calibration-sweep-root",
                "/tmp/c",
                "--validation-sweep-root",
                "/tmp/v",
                "--sealed-runtime-root",
                "/tmp/sealed",
                *_exact_args(),
            ]
        )
    with pytest.raises(SystemExit):
        module.parse_args(
            [
                "fit-select-freeze",
                "--qualification-root",
                "/tmp/q",
                "--calibration-sweep-root",
                "/tmp/c",
                "--validation-sweep-root",
                "/tmp/v",
                "--method-family",
                "dual_camera_wahba_so3",
                *_exact_args(),
            ]
        )


def test_prepare_execute_passes_verified_exact_revision_to_spec(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    module = _cli_module()
    captured = {}
    monkeypatch.setattr(
        module,
        "verify_git_provenance",
        lambda *args, **kwargs: {
            "repository_root": str(tmp_path),
            "branch": BRANCH,
            "commit_sha": SHA,
            "git_status": "clean",
        },
    )

    def fake_prepare(output, spec):
        captured.update(output=output, spec=spec)
        return {"status": "prepared"}

    monkeypatch.setattr(module, "prepare_plans", fake_prepare)
    output = tmp_path / "external-output"
    args = module.parse_args(
        [
            "prepare-plans",
            "--output-dir",
            str(output),
            "--qualification-id",
            "qid",
            "--repository-name",
            "goodbuilder/SASL",
            *_exact_args(),
        ]
    )
    assert module.execute(args) == {"status": "prepared"}
    assert captured["output"] == output.resolve()
    assert captured["spec"].commit_sha == SHA
    assert captured["spec"].branch == BRANCH


def test_prepare_output_rejects_relative_repository_internal_and_nonempty(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    module = _cli_module()
    with pytest.raises(ValueError, match="absolute"):
        module.validate_prepare_output(Path("relative"))
    monkeypatch.setattr(module, "REPOSITORY_ROOT", tmp_path)
    with pytest.raises(ValueError, match="outside"):
        module.validate_prepare_output(tmp_path / "inside")
    outside = tmp_path.parent / f"{tmp_path.name}-outside"
    outside.mkdir()
    (outside / "unrelated").write_text("keep", encoding="utf-8")
    with pytest.raises(ValueError, match="new or empty"):
        module.validate_prepare_output(outside)


def test_unseal_requires_both_frozen_hashes() -> None:
    module = _cli_module()
    with pytest.raises(SystemExit):
        module.parse_args(
            [
                "unseal-evaluate",
                "--qualification-root",
                "/tmp/q",
                "--freeze-anchor",
                "Simulation/config/s2_r4a_freeze_anchor.json",
                "--repository-root",
                "/tmp/repo",
                "--frozen-catalog-sha256",
                "c" * 64,
                *_exact_args(),
            ]
        )


def test_verify_anchor_cli_uses_production_validator_before_any_mutation(
    tmp_path: Path,
) -> None:
    if not AUTHORITATIVE_ROOT.is_dir():
        pytest.skip("sealed qualification fixture unavailable; set SASL_QUALIFICATION_FIXTURE")
    module = _cli_module()
    qualification = tmp_path / "qualification"
    shutil.copytree(AUTHORITATIVE_ROOT, qualification)
    unexpected = qualification / "SEALED_RUNNING"
    unexpected.write_bytes(b"")
    ledger_before = (qualification / "data_access_ledger.jsonl").read_bytes()
    state_before = (qualification / "qualification_state.json").read_bytes()
    markers_before = sorted(path.name for path in qualification.iterdir() if path.name.isupper())
    args = module.parse_args(
        [
            "verify-freeze-anchor",
            "--qualification-root",
            str(qualification),
            "--freeze-anchor",
            "Simulation/config/s2_r4a_freeze_anchor.json",
            "--repository-root",
            str(module.REPOSITORY_ROOT),
        ]
    )
    with pytest.raises(ValueError, match="unexpected_sealed_runtime_artifact"):
        module.execute(args)
    assert (qualification / "data_access_ledger.jsonl").read_bytes() == ledger_before
    assert (qualification / "qualification_state.json").read_bytes() == state_before
    assert sorted(path.name for path in qualification.iterdir() if path.name.isupper()) == markers_before
