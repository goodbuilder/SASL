from __future__ import annotations

import copy
import inspect
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPOSITORY_ROOT / "Simulation" / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

import stereo_ground_sim.correction_qualification as qualification_module  # noqa: E402
from stereo_ground_sim.correction_qualification import (  # noqa: E402
    FREEZE_ANCHOR_REPOSITORY_PATH,
    authorize_unseal,
    load_freeze_anchor,
    validate_freeze_anchor,
    verify_freeze_anchor_artifacts,
)
from stereo_ground_sim.correction_sweep import canonical_json, sha256_file  # noqa: E402


AUTHORITATIVE_ROOT = Path(
    "/home/moon/Desktop/영상감시시스템-audit-results/s2-r4a/qualification/"
    "exact-profile-freeze-88a360f"
)
ANCHOR_PATH = REPOSITORY_ROOT / FREEZE_ANCHOR_REPOSITORY_PATH


def _anchor() -> dict:
    return load_freeze_anchor(ANCHOR_PATH)


def _copy_qualification(tmp_path: Path) -> Path:
    destination = tmp_path / "qualification"
    shutil.copytree(AUTHORITATIVE_ROOT, destination)
    return destination


def _temporary_git_checkout(tmp_path: Path) -> Path:
    root = tmp_path / "different-repository"
    root.mkdir()
    subprocess.run(("git", "init", "-q"), cwd=root, check=True)
    (root / "README.md").write_text("temporary test checkout\n", encoding="utf-8")
    subprocess.run(("git", "add", "README.md"), cwd=root, check=True)
    subprocess.run(
        (
            "git",
            "-c",
            "user.name=Qualification Test",
            "-c",
            "user.email=qualification@example.invalid",
            "-c",
            "commit.gpgsign=false",
            "-c",
            "core.hooksPath=/dev/null",
            "commit",
            "-q",
            "-m",
            "initial",
        ),
        cwd=root,
        check=True,
    )
    return root


def _write_json(path: Path, value: dict) -> None:
    path.write_text(canonical_json(value) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, values: list[dict]) -> None:
    path.write_text(
        "".join(canonical_json(value) + "\n" for value in values),
        encoding="utf-8",
    )


def _write_anchor(path: Path, anchor: dict) -> None:
    path.write_text(canonical_json(anchor) + "\n", encoding="utf-8")


def _refresh_anchor_chain(root: Path, anchor: dict, *relative_paths: str) -> None:
    for relative in relative_paths:
        anchor["artifact_sha256"][relative] = sha256_file(root / relative)
    manifest_path = root / "frozen_selection_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["data_access_ledger_sha256"] = anchor["artifact_sha256"][
        "data_access_ledger.jsonl"
    ]
    _write_json(manifest_path, manifest)
    anchor["artifact_sha256"]["frozen_selection_manifest.json"] = sha256_file(
        manifest_path
    )
    state_path = root / "qualification_state.json"
    state = json.loads(state_path.read_text(encoding="utf-8"))
    state["frozen_selection_manifest_sha256"] = anchor["artifact_sha256"][
        "frozen_selection_manifest.json"
    ]
    _write_json(state_path, state)
    anchor["artifact_sha256"]["qualification_state.json"] = sha256_file(state_path)


def _tree_snapshot(root: Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): sha256_file(path)
        for path in root.rglob("*")
        if path.is_file()
    }


def _cli_module():
    path = REPOSITORY_ROOT / "Simulation/scripts/run_sealed_correction_qualification.py"
    name = "s2_r4a_d1_cli_test"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _verify_single_case_bundles(
    monkeypatch: pytest.MonkeyPatch,
    mutate_calibration_artifact,
) -> None:
    ledger_path = AUTHORITATIVE_ROOT / "data_access_ledger.jsonl"
    ledger = [json.loads(line) for line in ledger_path.read_text().splitlines()]
    case_directories = {
        role: next(
            Path(entry["canonical_path"]).parent
            for entry in ledger
            if entry["role"] == role
            and entry["operation"] == "read_r2_artifact"
        )
        for role in ("calibration", "validation")
    }
    reduced_ledger = [
        entry
        for entry in ledger
        if entry["operation"] != "read_r2_artifact"
        or Path(entry["canonical_path"]).parent == case_directories[entry["role"]]
    ]
    for sequence, entry in enumerate(reduced_ledger, start=1):
        entry["sequence"] = sequence
    planned_cases = {
        role: [
            {
                "case_id": json.loads(
                    (directory / "run_metadata.json").read_text(encoding="utf-8")
                )["case_id"]
            }
        ]
        for role, directory in case_directories.items()
    }
    original_external_file = qualification_module._external_ledger_file

    def mutated_external_file(entry, *, name):
        snapshot = original_external_file(entry, name=name)
        path = Path(entry["canonical_path"])
        if path.parent != case_directories["calibration"]:
            return snapshot
        raw_bytes = mutate_calibration_artifact(path.name, snapshot.raw_bytes)
        return qualification_module.VerifiedFileBytes(
            canonical_path=snapshot.canonical_path,
            raw_bytes=raw_bytes,
            sha256=snapshot.sha256,
            stat_identity=snapshot.stat_identity,
        )

    monkeypatch.setattr(
        qualification_module, "_external_ledger_file", mutated_external_file
    )
    qualification_module._verify_ledger_referenced_artifacts(
        reduced_ledger,
        {"calibration_case_count": 1, "validation_case_count": 1},
        planned_cases,
    )


def _mutate_json_artifact(name: str, raw_bytes: bytes, artifact: str, mutate) -> bytes:
    if name != artifact:
        return raw_bytes
    value = json.loads(raw_bytes)
    mutate(value)
    return (canonical_json(value) + "\n").encode("utf-8")


@pytest.mark.parametrize(
    "relative",
    (
        "pair_records.jsonl",
        "pair_records.csv",
        "run_summary.json",
        "RUN_FINALIZED_VALID",
    ),
)
def test_ledger_referenced_case_artifact_tamper_is_rejected(
    tmp_path: Path, relative: str
) -> None:
    root = _copy_qualification(tmp_path)
    anchor = copy.deepcopy(_anchor())
    ledger_path = root / "data_access_ledger.jsonl"
    ledger = [json.loads(line) for line in ledger_path.read_text(encoding="utf-8").splitlines()]
    entry = next(
        item
        for item in ledger
        if item["operation"] == "read_r2_artifact"
        and Path(item["canonical_path"]).name == relative
    )
    copied_evidence = tmp_path / "evidence" / relative
    copied_evidence.parent.mkdir()
    shutil.copyfile(Path(entry["canonical_path"]), copied_evidence)
    entry["canonical_path"] = str(copied_evidence)
    _write_jsonl(ledger_path, ledger)
    _refresh_anchor_chain(root, anchor, "data_access_ledger.jsonl")

    with pytest.raises(ValueError, match="qualification_evidence_root_mismatch"):
        verify_freeze_anchor_artifacts(root, anchor, repository_root=REPOSITORY_ROOT)


def test_duplicate_ledger_path_is_rejected(tmp_path: Path) -> None:
    root = _copy_qualification(tmp_path)
    anchor = copy.deepcopy(_anchor())
    ledger_path = root / "data_access_ledger.jsonl"
    ledger = [json.loads(line) for line in ledger_path.read_text().splitlines()]
    first = next(
        item for item in ledger
        if item["operation"] == "read_r2_artifact"
        and Path(item["canonical_path"]).name == "pair_records.jsonl"
    )
    second = next(
        item for item in ledger
        if item["operation"] == "read_r2_artifact"
        and Path(item["canonical_path"]).name == "pair_records.jsonl"
        and item["canonical_path"] != first["canonical_path"]
    )
    second["canonical_path"] = first["canonical_path"]
    second["artifact_hash"] = first["artifact_hash"]
    _write_jsonl(ledger_path, ledger)
    _refresh_anchor_chain(root, anchor, "data_access_ledger.jsonl")
    with pytest.raises(ValueError, match="access_ledger_duplicate_canonical_path"):
        verify_freeze_anchor_artifacts(root, anchor, repository_root=REPOSITORY_ROOT)


def test_case_bundle_identity_reuse_is_rejected(
    tmp_path: Path,
) -> None:
    root = _copy_qualification(tmp_path)
    anchor = copy.deepcopy(_anchor())
    ledger_path = root / "data_access_ledger.jsonl"
    ledger = [json.loads(line) for line in ledger_path.read_text().splitlines()]
    pairs = [
        item for item in ledger
        if item["role"] == "calibration"
        and item["operation"] == "read_r2_artifact"
        and Path(item["canonical_path"]).name == "run_metadata.json"
    ]
    first, second = pairs[:2]
    first_path, first_hash = first["canonical_path"], first["artifact_hash"]
    second["canonical_path"], second["artifact_hash"] = first_path, first_hash
    _write_jsonl(ledger_path, ledger)
    _refresh_anchor_chain(root, anchor, "data_access_ledger.jsonl")
    with pytest.raises(ValueError, match="access_ledger_duplicate_canonical_path"):
        verify_freeze_anchor_artifacts(root, anchor, repository_root=REPOSITORY_ROOT)


def test_ledger_binding_requires_planned_cases() -> None:
    parameter = inspect.signature(
        qualification_module._verify_ledger_referenced_artifacts
    ).parameters["planned_cases"]
    assert parameter.default is inspect.Parameter.empty


@pytest.mark.parametrize(
    ("mutation", "expected_error"),
    (
        ("duplicate_header", "qualification_csv_schema_invalid"),
        ("no_header", "qualification_csv_case_id_column_missing"),
        ("missing_case_id", "qualification_csv_case_id_column_missing"),
        ("no_data_rows", "qualification_csv_rows_empty"),
        ("wrong_case_id", "qualification_csv_case_identity_mismatch"),
    ),
)
def test_frozen_case_csv_schema_and_identity_are_required(
    monkeypatch: pytest.MonkeyPatch, mutation: str, expected_error: str
) -> None:
    def mutate_csv(name: str, raw_bytes: bytes) -> bytes:
        if name != "pair_records.csv":
            return raw_bytes
        lines = raw_bytes.decode("utf-8").splitlines()
        header = lines[0].split(",")
        case_index = header.index("case_id")
        if mutation == "duplicate_header":
            header[(case_index + 1) % len(header)] = "case_id"
            lines[0] = ",".join(header)
        elif mutation == "no_header":
            lines = []
        elif mutation == "missing_case_id":
            header[case_index] = "not_case_id"
            lines[0] = ",".join(header)
        elif mutation == "no_data_rows":
            lines = [lines[0]]
        else:
            lines[1] = lines[1].replace(
                lines[1].split(",")[case_index], "wrong-case-id", 1
            )
        return ("\n".join(lines) + "\n").encode("utf-8")

    with pytest.raises(ValueError, match=expected_error):
        _verify_single_case_bundles(monkeypatch, mutate_csv)


@pytest.mark.parametrize(
    ("field", "mutation", "expected_error"),
    (
        (
            "jsonl_line_count",
            "missing",
            "qualification_summary_jsonl_line_count_missing",
        ),
        (
            "csv_data_row_count",
            "missing",
            "qualification_summary_csv_data_row_count_missing",
        ),
        (
            "jsonl_line_count",
            "non_int",
            "qualification_summary:jsonl_line_count_exact_int_required",
        ),
        (
            "csv_data_row_count",
            "non_int",
            "qualification_summary:csv_data_row_count_exact_int_required",
        ),
        ("jsonl_line_count", "zero", "qualification_pair_record_count_mismatch"),
        ("csv_data_row_count", "zero", "qualification_pair_record_count_mismatch"),
        (
            "jsonl_line_count",
            "mismatched",
            "qualification_pair_record_count_mismatch",
        ),
        (
            "csv_data_row_count",
            "mismatched",
            "qualification_pair_record_count_mismatch",
        ),
    ),
)
def test_frozen_summary_requires_exact_positive_artifact_counts(
    monkeypatch: pytest.MonkeyPatch,
    field: str,
    mutation: str,
    expected_error: str,
) -> None:
    def mutate_summary(value: dict) -> None:
        if mutation == "missing":
            del value[field]
        elif mutation == "non_int":
            value[field] = float(value[field])
        elif mutation == "zero":
            value[field] = 0
        else:
            value[field] += 1

    with pytest.raises(ValueError, match=expected_error):
        _verify_single_case_bundles(
            monkeypatch,
            lambda name, raw_bytes: _mutate_json_artifact(
                name, raw_bytes, "run_summary.json", mutate_summary
            ),
        )


@pytest.mark.parametrize("artifact", ("pair_records.jsonl", "pair_records.csv"))
def test_frozen_summary_count_must_match_physical_artifact_rows(
    monkeypatch: pytest.MonkeyPatch, artifact: str
) -> None:
    def remove_last_row(name: str, raw_bytes: bytes) -> bytes:
        if name != artifact:
            return raw_bytes
        return ("\n".join(raw_bytes.decode("utf-8").splitlines()[:-1]) + "\n").encode(
            "utf-8"
        )

    with pytest.raises(ValueError, match="qualification_pair_record_count_mismatch"):
        _verify_single_case_bundles(monkeypatch, remove_last_row)


def test_frozen_jsonl_and_csv_counts_must_be_equal(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def shorten_csv_and_summary(name: str, raw_bytes: bytes) -> bytes:
        if name == "pair_records.csv":
            return (
                "\n".join(raw_bytes.decode("utf-8").splitlines()[:-1]) + "\n"
            ).encode("utf-8")
        return _mutate_json_artifact(
            name,
            raw_bytes,
            "run_summary.json",
            lambda value: value.__setitem__(
                "csv_data_row_count", value["csv_data_row_count"] - 1
            ),
        )

    with pytest.raises(ValueError, match="qualification_pair_record_count_mismatch"):
        _verify_single_case_bundles(monkeypatch, shorten_csv_and_summary)


def test_coordinated_ledger_and_case_file_tamper_is_rejected_by_sidecar(
    tmp_path: Path,
) -> None:
    root = _copy_qualification(tmp_path)
    ledger_path = root / "data_access_ledger.jsonl"
    ledger = [json.loads(line) for line in ledger_path.read_text(encoding="utf-8").splitlines()]
    entry = next(
        item
        for item in ledger
        if item["operation"] == "read_r2_artifact"
        and Path(item["canonical_path"]).name == "pair_records.jsonl"
    )
    copied_evidence = tmp_path / "pair_records.jsonl"
    copied_evidence.write_bytes(Path(entry["canonical_path"]).read_bytes() + b"x")
    entry["canonical_path"] = str(copied_evidence)
    entry["artifact_hash"] = sha256_file(copied_evidence)
    _write_jsonl(ledger_path, ledger)
    with pytest.raises(
        ValueError, match="artifact_sha256_mismatch: data_access_ledger.jsonl"
    ):
        verify_freeze_anchor_artifacts(root, _anchor(), repository_root=REPOSITORY_ROOT)


@pytest.mark.parametrize(
    "name",
    (
        "pair_records.jsonl",
        "pair_records.csv",
        "run_summary.json",
        "RUN_FINALIZED_VALID",
        "RUN_FINALIZED_INVALID",
        "RUN_INCOMPLETE",
        "SEALED_RUNNING",
        "SEALED_INTERRUPTED",
        "SEALED_FAILED",
        "SEALED_COMPLETED",
    ),
)
def test_unexpected_runtime_names_are_rejected(tmp_path: Path, name: str) -> None:
    root = _copy_qualification(tmp_path)
    unexpected = root / "hidden" / name
    unexpected.parent.mkdir()
    unexpected.write_bytes(b"")
    with pytest.raises(ValueError, match="unexpected_sealed_runtime_artifact"):
        qualification_module._assert_no_sealed_runtime(root)


@pytest.mark.parametrize("relative", ("pair_records.jsonl", "hidden/.partial-output"))
def test_production_verifier_rejects_unexpected_runtime_artifact(
    tmp_path: Path, relative: str
) -> None:
    root = _copy_qualification(tmp_path)
    unexpected = root / relative
    unexpected.parent.mkdir(exist_ok=True)
    unexpected.write_bytes(b"")
    with pytest.raises(ValueError, match="unexpected_sealed_runtime_artifact"):
        verify_freeze_anchor_artifacts(root, _anchor(), repository_root=REPOSITORY_ROOT)


@pytest.mark.parametrize("mutation", ("blank_line", "duplicate_key"))
def test_frozen_ledger_jsonl_is_strict(tmp_path: Path, mutation: str) -> None:
    root = _copy_qualification(tmp_path)
    anchor = copy.deepcopy(_anchor())
    ledger_path = root / "data_access_ledger.jsonl"
    lines = ledger_path.read_text(encoding="utf-8").splitlines()
    if mutation == "blank_line":
        lines.insert(1, "")
        expected = "blank_jsonl_line"
    else:
        lines[0] = lines[0].replace('"sequence":1', '"sequence":1,"sequence":1')
        expected = "duplicate_json_key"
    ledger_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    _refresh_anchor_chain(root, anchor, "data_access_ledger.jsonl")
    with pytest.raises(ValueError, match=expected):
        verify_freeze_anchor_artifacts(root, anchor, repository_root=REPOSITORY_ROOT)


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    (
        ("sequence", 1.0, "exact_int"),
        ("process_pid", False, "exact_int"),
        ("operation", 123, "exact_str"),
        ("canonical_path", 123, "exact_str"),
    ),
)
def test_frozen_ledger_scalar_types_are_exact(
    tmp_path: Path, field: str, value, expected: str
) -> None:
    root = _copy_qualification(tmp_path)
    anchor = copy.deepcopy(_anchor())
    ledger_path = root / "data_access_ledger.jsonl"
    ledger = [json.loads(line) for line in ledger_path.read_text().splitlines()]
    ledger[0][field] = value
    _write_jsonl(ledger_path, ledger)
    _refresh_anchor_chain(root, anchor, "data_access_ledger.jsonl")
    with pytest.raises(ValueError, match=expected):
        verify_freeze_anchor_artifacts(root, anchor, repository_root=REPOSITORY_ROOT)


@pytest.mark.parametrize(
    ("mutate", "expected_error"),
    (
        (lambda value: value["dataset_contract"].__setitem__("calibration_case_count", 450.0), "exact_int"),
        (lambda value: value["dataset_contract"].__setitem__("validation_case_count", 225.0), "exact_int"),
        (lambda value: value["dataset_contract"].__setitem__("sealed_case_count", 225.0), "exact_int"),
        (lambda value: value["dataset_contract"].__setitem__("sealed_runtime_access_count_at_freeze", False), "exact_int"),
        (lambda value: value["dataset_contract"].__setitem__("sealed_metrics_present_at_freeze", 0), "exact_bool"),
        (lambda value: value["selected_profile"].__setitem__("profile_count", 25.0), "exact_int"),
        (lambda value: value["selected_profile"].__setitem__("position_unit", ["m"]), "exact_str"),
        (lambda value: value["artifact_sha256"].__setitem__("qualification_state.json", int("1" * 64)), "sha256"),
        (lambda value: value["ground_truth_provenance"].__setitem__("reference_translation_offset_m", [0, 0, 0]), "exact_number_list"),
    ),
)
def test_sidecar_scalar_types_are_exact(mutate, expected_error: str) -> None:
    anchor = copy.deepcopy(_anchor())
    mutate(anchor)
    with pytest.raises(ValueError, match=expected_error):
        validate_freeze_anchor(anchor)


def test_non_string_relative_path_is_rejected() -> None:
    with pytest.raises(ValueError, match="exact_str"):
        qualification_module._canonical_relative_path(123, name="artifact")


def test_spatial_cell_overlap_is_rejected_even_when_case_ids_differ(
    tmp_path: Path,
) -> None:
    root = _copy_qualification(tmp_path)
    anchor = copy.deepcopy(_anchor())
    calibration_path = root / "calibration_plan/sweep_cases.jsonl"
    validation_path = root / "validation_plan/sweep_cases.jsonl"
    calibration = [json.loads(line) for line in calibration_path.read_text().splitlines()]
    validation = [json.loads(line) for line in validation_path.read_text().splitlines()]
    validation[0]["payload"]["spatial"] = copy.deepcopy(
        calibration[0]["payload"]["spatial"]
    )
    assert validation[0]["case_id"] != calibration[0]["case_id"]
    _write_jsonl(validation_path, validation)
    anchor["artifact_sha256"]["validation_plan/sweep_cases.jsonl"] = sha256_file(
        validation_path
    )
    with pytest.raises(ValueError, match="qualification_spatial_group_overlap_mismatch"):
        verify_freeze_anchor_artifacts(root, anchor, repository_root=REPOSITORY_ROOT)


def test_hash_and_json_parse_use_one_byte_snapshot(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = _copy_qualification(tmp_path)
    state_path = root / "qualification_state.json"
    initial_bytes = state_path.read_bytes()
    original_sha256_file = qualification_module.sha256_file

    def replace_after_hash(path: Path) -> str:
        digest = original_sha256_file(path)
        if Path(path) == state_path:
            value = json.loads(initial_bytes)
            state_path.write_text(json.dumps(value, indent=4) + "\n", encoding="utf-8")
        return digest

    monkeypatch.setattr(qualification_module, "sha256_file", replace_after_hash)
    verify_freeze_anchor_artifacts(root, _anchor(), repository_root=REPOSITORY_ROOT)
    assert state_path.read_bytes() == initial_bytes


@pytest.mark.parametrize("intermediate", (False, True))
def test_cli_rejects_qualification_root_symlink_before_resolution(
    tmp_path: Path, intermediate: bool
) -> None:
    module = _cli_module()
    if intermediate:
        link = tmp_path / "audit-results"
        link.symlink_to(AUTHORITATIVE_ROOT.parent, target_is_directory=True)
        supplied = link / AUTHORITATIVE_ROOT.name
    else:
        supplied = tmp_path / "qualification-link"
        supplied.symlink_to(AUTHORITATIVE_ROOT, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        module._existing_external_directory(supplied, name="qualification root")


def test_verify_cli_rejects_a_different_execution_checkout(tmp_path: Path) -> None:
    module = _cli_module()
    different_repository = _temporary_git_checkout(tmp_path)
    args = module.parse_args(
        [
            "verify-freeze-anchor",
            "--qualification-root",
            str(AUTHORITATIVE_ROOT),
            "--freeze-anchor",
            FREEZE_ANCHOR_REPOSITORY_PATH,
            "--repository-root",
            str(different_repository),
        ]
    )
    with pytest.raises(ValueError, match="verify repository root must be the CLI repository"):
        module.execute(args)


@pytest.mark.parametrize(
    "anchor_path",
    (
        "Simulation/config/../config/s2_r4a_freeze_anchor.json",
        "/tmp/s2_r4a_freeze_anchor.json",
    ),
)
def test_verify_cli_rejects_noncanonical_anchor_path(anchor_path: str) -> None:
    module = _cli_module()
    args = module.parse_args(
        [
            "verify-freeze-anchor",
            "--qualification-root",
            str(AUTHORITATIVE_ROOT),
            "--freeze-anchor",
            anchor_path,
            "--repository-root",
            str(REPOSITORY_ROOT),
        ]
    )
    with pytest.raises(ValueError, match="verify freeze anchor path mismatch"):
        module.execute(args)


def test_production_unseal_rejects_before_mutation_on_unexpected_runtime(
    tmp_path: Path,
) -> None:
    root = _copy_qualification(tmp_path)
    (root / "SEALED_RUNNING").write_bytes(b"")
    before = _tree_snapshot(root)
    anchor = _anchor()
    with pytest.raises(ValueError, match="unexpected_sealed_runtime_artifact"):
        authorize_unseal(
            root,
            freeze_anchor_path=ANCHOR_PATH,
            repository_root=REPOSITORY_ROOT,
            frozen_catalog_sha256=anchor["artifact_sha256"][
                "frozen_correction_catalog.json"
            ],
            sealed_commitment_sha256=anchor["artifact_sha256"][
                "sealed_evaluation_commitment.json"
            ],
        )
    assert _tree_snapshot(root) == before


def test_production_unseal_rejects_case_artifact_tamper_before_mutation(
    tmp_path: Path,
) -> None:
    root = _copy_qualification(tmp_path)
    anchor = copy.deepcopy(_anchor())
    ledger_path = root / "data_access_ledger.jsonl"
    ledger = [json.loads(line) for line in ledger_path.read_text().splitlines()]
    entry = next(
        item
        for item in ledger
        if item["operation"] == "read_r2_artifact"
        and Path(item["canonical_path"]).name == "pair_records.jsonl"
    )
    copied_evidence = tmp_path / "evidence" / "pair_records.jsonl"
    copied_evidence.parent.mkdir()
    shutil.copyfile(Path(entry["canonical_path"]), copied_evidence)
    entry["canonical_path"] = str(copied_evidence)
    _write_jsonl(ledger_path, ledger)
    _refresh_anchor_chain(root, anchor, "data_access_ledger.jsonl")
    anchor_path = tmp_path / "anchor.json"
    _write_anchor(anchor_path, anchor)
    copied_evidence.write_bytes(copied_evidence.read_bytes() + b"x")
    before = _tree_snapshot(root)
    with pytest.raises(
        ValueError, match="ledger_referenced_artifact_sha256_mismatch"
    ):
        authorize_unseal(
            root,
            freeze_anchor_path=anchor_path,
            repository_root=REPOSITORY_ROOT,
            frozen_catalog_sha256=anchor["artifact_sha256"][
                "frozen_correction_catalog.json"
            ],
            sealed_commitment_sha256=anchor["artifact_sha256"][
                "sealed_evaluation_commitment.json"
            ],
        )
    assert _tree_snapshot(root) == before


def test_production_unseal_rejects_sidecar_type_error_before_mutation(
    tmp_path: Path,
) -> None:
    root = _copy_qualification(tmp_path)
    anchor = copy.deepcopy(_anchor())
    anchor["dataset_contract"]["validation_case_count"] = 225.0
    anchor_path = tmp_path / "anchor.json"
    _write_anchor(anchor_path, anchor)
    before = _tree_snapshot(root)
    with pytest.raises(ValueError, match="exact_int"):
        authorize_unseal(
            root,
            freeze_anchor_path=anchor_path,
            repository_root=REPOSITORY_ROOT,
            frozen_catalog_sha256=anchor["artifact_sha256"][
                "frozen_correction_catalog.json"
            ],
            sealed_commitment_sha256=anchor["artifact_sha256"][
                "sealed_evaluation_commitment.json"
            ],
        )
    assert _tree_snapshot(root) == before


def test_production_unseal_rejects_spatial_overlap_before_mutation(
    tmp_path: Path,
) -> None:
    root = _copy_qualification(tmp_path)
    anchor = copy.deepcopy(_anchor())
    calibration_path = root / "calibration_plan/sweep_cases.jsonl"
    validation_path = root / "validation_plan/sweep_cases.jsonl"
    calibration = [json.loads(line) for line in calibration_path.read_text().splitlines()]
    validation = [json.loads(line) for line in validation_path.read_text().splitlines()]
    validation[0]["payload"]["spatial"] = copy.deepcopy(
        calibration[0]["payload"]["spatial"]
    )
    _write_jsonl(validation_path, validation)
    anchor["artifact_sha256"]["validation_plan/sweep_cases.jsonl"] = sha256_file(
        validation_path
    )
    anchor_path = tmp_path / "anchor.json"
    _write_anchor(anchor_path, anchor)
    before = _tree_snapshot(root)
    with pytest.raises(ValueError, match="qualification_spatial_group_overlap_mismatch"):
        authorize_unseal(
            root,
            freeze_anchor_path=anchor_path,
            repository_root=REPOSITORY_ROOT,
            frozen_catalog_sha256=anchor["artifact_sha256"][
                "frozen_correction_catalog.json"
            ],
            sealed_commitment_sha256=anchor["artifact_sha256"][
                "sealed_evaluation_commitment.json"
            ],
        )
    assert _tree_snapshot(root) == before


def test_production_unseal_rejects_repository_identity_before_mutation(
    tmp_path: Path,
) -> None:
    root = _copy_qualification(tmp_path)
    different_repository = _temporary_git_checkout(tmp_path)
    anchor = _anchor()
    before = _tree_snapshot(root)
    with pytest.raises(ValueError, match="evaluation_repository_identity_mismatch"):
        authorize_unseal(
            root,
            freeze_anchor_path=(
                different_repository / FREEZE_ANCHOR_REPOSITORY_PATH
            ),
            repository_root=different_repository,
            frozen_catalog_sha256=anchor["artifact_sha256"][
                "frozen_correction_catalog.json"
            ],
            sealed_commitment_sha256=anchor["artifact_sha256"][
                "sealed_evaluation_commitment.json"
            ],
        )
    assert _tree_snapshot(root) == before
