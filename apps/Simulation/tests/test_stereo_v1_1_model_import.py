"""Pinned model import preserves the hash and refuses conflicting aliases."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "campaigns" / "stereo_v1_1" / "import_yolo_weights.py"


def _module(tmp_path, monkeypatch, payload=b"pinned-weight"):
    spec = importlib.util.spec_from_file_location("v1_1_model_import", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root = tmp_path / "repo"
    root.mkdir()
    source = tmp_path / "external.pt"
    source.write_bytes(payload)
    digest = hashlib.sha256(payload).hexdigest()
    manifest = {
        "model_filename": "yolov8n_uav.pt",
        "sha256": digest,
        "size_bytes": len(payload),
        "owned_path_from_repo_root": f"apps/Simulation/.local/campaign_assets/sha256/{digest}/yolov8n_uav.pt",
        "runtime_alias_from_repo_root": "apps/Simulation/yolov8n_uav.pt",
    }
    manifest_path = tmp_path / "model_manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    monkeypatch.setattr(module, "REPO_ROOT", root)
    monkeypatch.setattr(module, "MANIFEST_PATH", manifest_path)
    return module, source, root, manifest


def test_import_hash_verifies_owned_copy_and_runtime_alias(tmp_path, monkeypatch):
    module, source, root, manifest = _module(tmp_path, monkeypatch)
    result = module.import_model(source)
    owned = root / manifest["owned_path_from_repo_root"]
    alias = root / manifest["runtime_alias_from_repo_root"]
    assert result["status"] == "MODEL_VERIFIED"
    assert owned.read_bytes() == source.read_bytes()
    assert alias.is_symlink() and alias.resolve() == owned
    assert module.check_owned_model() == result
    assert module.import_model(source) == result


def test_wrong_source_and_conflicting_alias_fail_closed(tmp_path, monkeypatch):
    module, source, root, manifest = _module(tmp_path, monkeypatch)
    source.write_bytes(b"wrong-weight")
    with pytest.raises(ValueError, match="size differs"):
        module.import_model(source)
    assert not (root / manifest["owned_path_from_repo_root"]).exists()

    source.write_bytes(b"pinned-weight")
    alias = root / manifest["runtime_alias_from_repo_root"]
    alias.parent.mkdir(parents=True)
    alias.write_bytes(b"unrelated-user-file")
    with pytest.raises(ValueError, match="already points elsewhere"):
        module.import_model(source)
    assert alias.read_bytes() == b"unrelated-user-file"
