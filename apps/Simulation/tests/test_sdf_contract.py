from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "sdf_contract.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("sdf_contract_under_test", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_dynamic_change_preserves_geometry_hash_but_changes_full_hash(tmp_path: Path) -> None:
    module = _load_module()
    source = ROOT / "stereo_ground_sim" / "models" / "x500_2m" / "model.sdf"
    original = source.read_text(encoding="utf-8")
    changed = original.replace(
        "<timeConstantUp>0.0125</timeConstantUp>",
        "<timeConstantUp>0.0130</timeConstantUp>",
    )
    candidate = tmp_path / "model.sdf"
    candidate.write_text(changed, encoding="utf-8")

    before = module.sdf_contract(source)
    after = module.sdf_contract(candidate)

    assert before["sha256"] != after["sha256"]
    assert before["protected_geometry_sha256"] == after["protected_geometry_sha256"]


def test_size_change_changes_protected_geometry_hash(tmp_path: Path) -> None:
    module = _load_module()
    source = ROOT / "stereo_ground_sim" / "models" / "x500_2m" / "model.sdf"
    original = source.read_text(encoding="utf-8")
    changed = original.replace(
        "<scale>4.0638 4.0638 4.0638</scale>",
        "<scale>4.1 4.1 4.1</scale>",
        1,
    )
    candidate = tmp_path / "model.sdf"
    candidate.write_text(changed, encoding="utf-8")

    assert (
        module.protected_geometry_sha256(source)
        != module.protected_geometry_sha256(candidate)
    )
