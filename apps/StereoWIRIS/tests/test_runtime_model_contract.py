from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path


APP_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = APP_ROOT / "scripts" / "check_runtime.py"
SPEC = importlib.util.spec_from_file_location("stereowiris_check_runtime_under_test", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
CHECK_RUNTIME = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK_RUNTIME)


def _write_manifest(path: Path, payload: bytes) -> None:
    path.write_text(
        json.dumps(
            {
                "filename": "yolov8n_uav.pt",
                "size_bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
        ),
        encoding="utf-8",
    )


def test_default_model_requires_manifest_verified_bytes(tmp_path: Path) -> None:
    model = tmp_path / "yolov8n_uav.pt"
    manifest = tmp_path / "model-manifest.json"

    missing = CHECK_RUNTIME._model_diagnostic(model, manifest_path=manifest)
    assert missing.level == "FAIL"
    assert "missing" in missing.message
    assert "model-manifest.json" in missing.recovery[0]

    approved = b"approved-legacy-model-sentinel"
    model.write_bytes(approved)
    _write_manifest(manifest, approved)
    accepted = CHECK_RUNTIME._model_diagnostic(model, manifest_path=manifest)
    assert accepted.level == "PASS"
    assert hashlib.sha256(approved).hexdigest() in accepted.message

    model.write_bytes(b"tampered-legacy-model-sentinel")
    rejected = CHECK_RUNTIME._model_diagnostic(model, manifest_path=manifest)
    assert rejected.level == "FAIL"
    assert "SHA-256" in rejected.message


def test_default_model_rejects_invalid_manifest_and_wrong_size(tmp_path: Path) -> None:
    model = tmp_path / "yolov8n_uav.pt"
    manifest = tmp_path / "model-manifest.json"
    model.write_bytes(b"model")
    manifest.write_text("{}", encoding="utf-8")

    invalid = CHECK_RUNTIME._model_diagnostic(model, manifest_path=manifest)
    assert invalid.level == "FAIL"
    assert "manifest is invalid" in invalid.message

    _write_manifest(manifest, b"different-size")
    wrong_size = CHECK_RUNTIME._model_diagnostic(model, manifest_path=manifest)
    assert wrong_size.level == "FAIL"
    assert "size mismatch" in wrong_size.message


def test_custom_model_preserves_existing_presence_only_contract(tmp_path: Path) -> None:
    model = tmp_path / "operator-selected.pt"
    model.write_bytes(b"custom-model")

    diagnostic = CHECK_RUNTIME._model_diagnostic(
        model,
        manifest_path=tmp_path / "not-required-for-custom-model.json",
    )

    assert diagnostic.level == "PASS"
    assert "custom YOLO model" in diagnostic.message


def test_tracked_legacy_model_manifest_matches_approved_external_contract() -> None:
    manifest = json.loads((APP_ROOT / "model-manifest.json").read_text(encoding="utf-8"))

    assert manifest == {
        "schema": "stereowiris-yolo-model-v1",
        "filename": "yolov8n_uav.pt",
        "size_bytes": 6269475,
        "sha256": "8a2204bf2ed7c09b9dbabeaa1c1e984ab482c10325e2387150e23d8daa3cb777",
        "provisioning": (
            "Copy the approved Legacy YOLO bytes into apps/StereoWIRIS; "
            "runtime download is disabled."
        ),
    }
