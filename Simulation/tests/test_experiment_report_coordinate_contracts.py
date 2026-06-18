from __future__ import annotations

from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
PASSED_METHODS = ROOT_DIR / "reports" / "experiments" / "passed_methods.md"


def _table_rows(path: Path) -> list[list[str]]:
    rows: list[list[str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped.startswith("| `"):
            continue
        rows.append([cell.strip() for cell in stripped.strip("|").split("|")])
    return rows


def test_pass_rows_do_not_depend_on_forbidden_ned_aligned_camera_contract() -> None:
    forbidden_tokens = (
        "assumed NED-aligned camera RDF contract",
        "rdf_right=east",
        "rdf_down=down",
        "rdf_forward=north",
    )
    pass_statuses = {"PASS", "STRICT_PASS"}

    offenders = []
    for row in _table_rows(PASSED_METHODS):
        if len(row) < 6:
            continue
        status = row[2].strip("`")
        text = " | ".join(row)
        if status in pass_statuses and any(token in text for token in forbidden_tokens):
            offenders.append(row[0])

    assert offenders == []


def test_legacy_ned_aligned_rows_are_not_current_pass_evidence() -> None:
    rows_by_experiment = {row[0].strip("`"): row for row in _table_rows(PASSED_METHODS) if row}
    legacy_ids = {
        "EXP-20260616-194-bbox-outlier-profile-transfer-audit",
        "EXP-20260616-208-bbox-outlier-target-disparity-blend-runtime-profile-alpha-audit",
        "EXP-20260616-235-bbox-outlier-away-from-target-max-runtime-profile-800seed-audit",
        "EXP-20260616-246-bbox-outlier-generic-combo-runtime-profile-800seed-audit",
        "EXP-20260616-921-bbox-outlier-exp899-exp891-exp873-combo-runtime-profile-800seed-audit",
    }

    for experiment_id in legacy_ids:
        row = rows_by_experiment[experiment_id]
        assert row[2].strip("`") == "CONTRACT_INVALID_FOR_CURRENT_GATE"
        assert "not PASS evidence under the current explicit `T_source_to_stereo_rdf` contract" in row[5]
