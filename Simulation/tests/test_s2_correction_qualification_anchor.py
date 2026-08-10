from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPOSITORY_ROOT / "Simulation" / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_qualification import (  # noqa: E402
    FREEZE_ANCHOR_REPOSITORY_PATH,
    load_freeze_anchor,
    resolve_freeze_anchor_commit,
    validate_freeze_anchor,
    verify_freeze_anchor_artifacts,
)
from stereo_ground_sim.correction_sweep import canonical_json, sha256_file  # noqa: E402


AUTHORITATIVE_ROOT = Path(
    os.environ.get(
        "SASL_QUALIFICATION_FIXTURE",
        REPOSITORY_ROOT / "Simulation/tests/fixtures/qualification/exact-profile-freeze-88a360f",
    )
)
pytestmark = pytest.mark.skipif(
    not AUTHORITATIVE_ROOT.is_dir(),
    reason="sealed qualification fixture unavailable; set SASL_QUALIFICATION_FIXTURE",
)
ANCHOR_PATH = REPOSITORY_ROOT / FREEZE_ANCHOR_REPOSITORY_PATH
KNOWN_HASHES = {
    "frozen_correction_catalog.json":
        "e3e828a9e014803ad450188ec95ddccce55a0f4bbaeec0060d58c49c221eadf6",
    "sealed_evaluation_commitment.json":
        "2a5478020361f3719ac0735f2a1ca61c8fce5273022394dd29c7d7f3cdb0e82c",
    "sealed_plan/sealed_sweep_plan.json":
        "c7f008b77c2e311d73a373e830bb024a03e5c7b5a936d2b5483a0928423e298a",
    "sealed_plan/sealed_sweep_cases.jsonl":
        "6b1dda197bef34f85f7ed346debcc5951d925110803a34a6ee26a5d96a1d5ade",
}


def _anchor() -> dict:
    return load_freeze_anchor(ANCHOR_PATH)


def _copy_qualification(tmp_path: Path) -> Path:
    destination = tmp_path / "qualification"
    shutil.copytree(AUTHORITATIVE_ROOT, destination)
    return destination


def _snapshot(root: Path, anchor: dict) -> dict[str, object]:
    hashes = {
        relative: sha256_file(root / relative)
        for relative in anchor["artifact_sha256"]
    }
    markers = sorted(path.name for path in root.iterdir() if path.name.isupper())
    ledger = root / "data_access_ledger.jsonl"
    return {
        "hashes": hashes,
        "markers": markers,
        "ledger_rows": len(ledger.read_text(encoding="utf-8").splitlines()),
    }


def _write_json(path: Path, value: dict) -> None:
    path.write_text(canonical_json(value) + "\n", encoding="utf-8")


def _git(root: Path, *args: str) -> str:
    return subprocess.check_output(("git", *args), cwd=root, text=True).strip()


def _history_repository(tmp_path: Path) -> tuple[Path, str, str]:
    root = tmp_path / "repository"
    root.mkdir()
    _git(root, "init", "-q")
    _git(root, "config", "user.name", "Anchor Test")
    _git(root, "config", "user.email", "anchor@example.invalid")
    (root / "source.txt").write_text("qualification source\n", encoding="utf-8")
    _git(root, "add", "source.txt")
    _git(root, "commit", "-q", "-m", "source")
    source_sha = _git(root, "rev-parse", "HEAD")
    anchor_path = root / FREEZE_ANCHOR_REPOSITORY_PATH
    anchor_path.parent.mkdir(parents=True)
    anchor_path.write_text('{"anchor":true}\n', encoding="utf-8")
    _git(root, "add", FREEZE_ANCHOR_REPOSITORY_PATH)
    _git(root, "commit", "-q", "-m", "anchor")
    return root, source_sha, _git(root, "rev-parse", "HEAD")


def test_existing_artifact_set_is_rejected_after_coordinate_source_change() -> None:
    anchor = _anchor()
    before = _snapshot(AUTHORITATIVE_ROOT, anchor)
    with pytest.raises(ValueError, match="coordinate_source_sha256_mismatch"):
        verify_freeze_anchor_artifacts(
            AUTHORITATIVE_ROOT,
            anchor,
            repository_root=REPOSITORY_ROOT,
        )
    after = _snapshot(AUTHORITATIVE_ROOT, anchor)
    assert after == before
    for relative, digest in KNOWN_HASHES.items():
        assert anchor["artifact_sha256"][relative] == digest


def test_catalog_only_tamper_is_rejected_by_git_anchor(tmp_path: Path) -> None:
    root = _copy_qualification(tmp_path)
    path = root / "frozen_correction_catalog.json"
    path.write_bytes(path.read_bytes() + b" ")
    with pytest.raises(
        ValueError,
        match=r"artifact_sha256_mismatch: frozen_correction_catalog.json",
    ):
        verify_freeze_anchor_artifacts(root, _anchor(), repository_root=REPOSITORY_ROOT)


def test_coordinated_external_tamper_still_fails_git_anchor(tmp_path: Path) -> None:
    root = _copy_qualification(tmp_path)
    catalog_path = root / "frozen_correction_catalog.json"
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    catalog["deployment_ready"] = True
    _write_json(catalog_path, catalog)
    catalog_hash = sha256_file(catalog_path)

    commitment_path = root / "sealed_evaluation_commitment.json"
    commitment = json.loads(commitment_path.read_text(encoding="utf-8"))
    commitment["commitment_hash"] = "0" * 64
    _write_json(commitment_path, commitment)
    commitment_hash = sha256_file(commitment_path)

    manifest_path = root / "frozen_selection_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["frozen_correction_catalog_sha256"] = catalog_hash
    manifest["sealed_commitment_sha256"] = commitment_hash
    _write_json(manifest_path, manifest)
    state_path = root / "qualification_state.json"
    state = json.loads(state_path.read_text(encoding="utf-8"))
    state["frozen_catalog_sha256"] = catalog_hash
    state["sealed_commitment_sha256"] = commitment_hash
    state["frozen_selection_manifest_sha256"] = sha256_file(manifest_path)
    _write_json(state_path, state)

    with pytest.raises(ValueError, match="artifact_sha256_mismatch"):
        verify_freeze_anchor_artifacts(root, _anchor(), repository_root=REPOSITORY_ROOT)


@pytest.mark.parametrize(
    "relative",
    ("sealed_plan/sealed_sweep_plan.json", "sealed_plan/sealed_sweep_cases.jsonl"),
)
def test_sealed_plan_byte_tamper_is_rejected(tmp_path: Path, relative: str) -> None:
    root = _copy_qualification(tmp_path)
    path = root / relative
    path.write_bytes(path.read_bytes() + b" ")
    with pytest.raises(ValueError, match=f"artifact_sha256_mismatch: {relative}"):
        verify_freeze_anchor_artifacts(root, _anchor(), repository_root=REPOSITORY_ROOT)


@pytest.mark.parametrize("mutation", ("unknown", "missing", "extra", "malformed", "uppercase", "absolute", "traversal"))
def test_anchor_schema_is_strict(mutation: str) -> None:
    anchor = copy.deepcopy(_anchor())
    if mutation == "unknown":
        anchor["schema_version"] = "future"
    elif mutation == "missing":
        del anchor["selected_profile"]
    elif mutation == "extra":
        anchor["unexpected"] = True
    elif mutation in {"malformed", "uppercase"}:
        digest = "short" if mutation == "malformed" else "A" * 64
        anchor["artifact_sha256"]["qualification_state.json"] = digest
    else:
        del anchor["artifact_sha256"]["qualification_state.json"]
        path = "/absolute.json" if mutation == "absolute" else "../qualification_state.json"
        anchor["artifact_sha256"][path] = "0" * 64
    with pytest.raises(ValueError):
        validate_freeze_anchor(anchor)


@pytest.mark.parametrize("constant", ("NaN", "Infinity"))
def test_anchor_loader_rejects_nonfinite_json(tmp_path: Path, constant: str) -> None:
    path = tmp_path / "anchor.json"
    path.write_text('{"value":' + constant + "}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="invalid JSON constant"):
        load_freeze_anchor(path)


def test_anchor_loader_rejects_duplicate_json_keys(tmp_path: Path) -> None:
    path = tmp_path / "anchor.json"
    path.write_text('{"schema_version":"a","schema_version":"b"}\n', encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate_json_key"):
        load_freeze_anchor(path)


def test_symlink_artifact_is_rejected(tmp_path: Path) -> None:
    root = _copy_qualification(tmp_path)
    target = root / "qualification_state.json"
    backup = tmp_path / "qualification-state-copy.json"
    shutil.copyfile(target, backup)
    target.unlink()
    target.symlink_to(backup)
    with pytest.raises(ValueError, match="artifact_symlink_rejected"):
        verify_freeze_anchor_artifacts(root, _anchor(), repository_root=REPOSITORY_ROOT)


@pytest.mark.parametrize(
    "field",
    (
        "semantic_reference_name",
        "semantic_reference_definition",
        "gazebo_source_entity",
        "parent_model",
        "parent_link",
        "reference_translation_offset_m",
        "reference_rotation_offset_rpy_rad",
        "raw_pose_topic",
        "output_gt_topic",
        "world_frame",
        "output_frame",
        "quaternion_order",
    ),
)
def test_ground_truth_provenance_mismatch_is_rejected(field: str) -> None:
    anchor = copy.deepcopy(_anchor())
    current = anchor["ground_truth_provenance"][field]
    anchor["ground_truth_provenance"][field] = (
        [1.0, 0.0, 0.0] if isinstance(current, list) else "mismatch"
    )
    with pytest.raises(ValueError, match="ground_truth_provenance_mismatch"):
        validate_freeze_anchor(anchor)


def test_coordinate_source_hash_mismatch_is_rejected() -> None:
    anchor = copy.deepcopy(_anchor())
    path = next(iter(anchor["coordinate_contract_sources"]["files"]))
    anchor["coordinate_contract_sources"]["files"][path] = "0" * 64
    with pytest.raises(ValueError, match="coordinate_source_sha256_mismatch"):
        verify_freeze_anchor_artifacts(
            AUTHORITATIVE_ROOT,
            anchor,
            repository_root=REPOSITORY_ROOT,
        )


def test_freeze_anchor_history_resolves_three_sha_roles(tmp_path: Path) -> None:
    root, source_sha, anchor_sha = _history_repository(tmp_path)
    result = resolve_freeze_anchor_commit(
        root,
        root / FREEZE_ANCHOR_REPOSITORY_PATH,
        qualification_source_sha=source_sha,
    )
    assert result["qualification_source_sha"] == source_sha
    assert result["freeze_anchor_sha"] == anchor_sha
    assert result["evaluation_code_sha"] == anchor_sha


def test_freeze_anchor_history_rejects_missing_source_commit(tmp_path: Path) -> None:
    root, _, _ = _history_repository(tmp_path)
    with pytest.raises(ValueError, match="git_anchor_validation_failed"):
        resolve_freeze_anchor_commit(
            root,
            root / FREEZE_ANCHOR_REPOSITORY_PATH,
            qualification_source_sha="f" * 40,
        )


def test_freeze_anchor_history_rejects_post_anchor_modification(tmp_path: Path) -> None:
    root, source_sha, _ = _history_repository(tmp_path)
    path = root / FREEZE_ANCHOR_REPOSITORY_PATH
    path.write_text('{"anchor":"changed"}\n', encoding="utf-8")
    _git(root, "add", FREEZE_ANCHOR_REPOSITORY_PATH)
    _git(root, "commit", "-q", "-m", "modify anchor")
    with pytest.raises(ValueError, match="freeze_anchor_modified_after_addition"):
        resolve_freeze_anchor_commit(root, path, qualification_source_sha=source_sha)


def test_freeze_anchor_history_rejects_two_additions(tmp_path: Path) -> None:
    root, source_sha, _ = _history_repository(tmp_path)
    path = root / FREEZE_ANCHOR_REPOSITORY_PATH
    path.unlink()
    _git(root, "add", FREEZE_ANCHOR_REPOSITORY_PATH)
    _git(root, "commit", "-q", "-m", "remove anchor")
    path.write_text('{"anchor":true}\n', encoding="utf-8")
    _git(root, "add", FREEZE_ANCHOR_REPOSITORY_PATH)
    _git(root, "commit", "-q", "-m", "re-add anchor")
    with pytest.raises(ValueError, match="freeze_anchor_add_commit_count_mismatch"):
        resolve_freeze_anchor_commit(root, path, qualification_source_sha=source_sha)
