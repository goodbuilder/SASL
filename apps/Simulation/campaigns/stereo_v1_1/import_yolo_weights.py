#!/usr/bin/env python3
"""Import the pinned YOLO weight into Simulation-owned content-addressed storage."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
MANIFEST_PATH = Path(__file__).with_name("model_manifest.json")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _contract() -> tuple[dict, Path, Path]:
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    owned = (REPO_ROOT / manifest["owned_path_from_repo_root"]).resolve()
    alias = REPO_ROOT / manifest["runtime_alias_from_repo_root"]
    if not owned.is_relative_to(REPO_ROOT.resolve()) or not alias.parent.resolve().is_relative_to(REPO_ROOT.resolve()):
        raise ValueError("model asset path escapes repository")
    if owned.name != manifest["model_filename"] or alias.name != manifest["model_filename"]:
        raise ValueError("model asset filename differs from manifest")
    return manifest, owned, alias


def _verify_file(path: Path, manifest: dict) -> None:
    if not path.is_file():
        raise FileNotFoundError(path)
    if path.stat().st_size != manifest["size_bytes"]:
        raise ValueError("YOLO weight size differs from manifest")
    if _sha256(path) != manifest["sha256"]:
        raise ValueError("YOLO weight SHA-256 differs from manifest")


def check_owned_model() -> dict:
    manifest, owned, alias = _contract()
    _verify_file(owned, manifest)
    if not alias.is_symlink() or alias.resolve() != owned:
        raise ValueError("Simulation runtime model alias is missing or points elsewhere")
    return {
        "status": "MODEL_VERIFIED",
        "sha256": manifest["sha256"],
        "size_bytes": manifest["size_bytes"],
        "owned_path_from_repo_root": manifest["owned_path_from_repo_root"],
    }


def import_model(source: Path) -> dict:
    manifest, owned, alias = _contract()
    source = source.expanduser().resolve()
    _verify_file(source, manifest)
    if owned.exists():
        _verify_file(owned, manifest)
    else:
        owned.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(prefix=".weight-", dir=owned.parent)
        os.close(descriptor)
        temporary = Path(temporary_name)
        try:
            shutil.copyfile(source, temporary)
            _verify_file(temporary, manifest)
            os.replace(temporary, owned)
        finally:
            temporary.unlink(missing_ok=True)
    if alias.exists() or alias.is_symlink():
        if not alias.is_symlink() or alias.resolve() != owned:
            raise ValueError("Simulation runtime model alias already points elsewhere")
    else:
        alias.symlink_to(owned.relative_to(alias.parent.resolve()))
    return check_owned_model()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--source", type=Path, help="Existing weight file to hash-verify and import")
    action.add_argument("--check", action="store_true", help="Verify the owned weight and runtime alias")
    args = parser.parse_args()
    result = check_owned_model() if args.check else import_model(args.source)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
