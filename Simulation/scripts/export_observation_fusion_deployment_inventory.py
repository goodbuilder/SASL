#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Iterable, Mapping


DEFAULT_EXCLUDED_PREFIXES = ("inventory/", "acceptance_check/")


def relative_posix(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def is_excluded(relative_path: str, excluded_prefixes: Iterable[str]) -> bool:
    return any(relative_path == prefix.rstrip("/") or relative_path.startswith(prefix) for prefix in excluded_prefixes)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def iter_package_files(package_dir: Path, excluded_prefixes: Iterable[str] = DEFAULT_EXCLUDED_PREFIXES) -> list[Path]:
    root = package_dir.resolve()
    files: list[Path] = []
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        relative = relative_posix(path, root)
        if is_excluded(relative, excluded_prefixes):
            continue
        files.append(path)
    return sorted(files, key=lambda path: relative_posix(path, root))


def build_inventory(
    package_dir: Path,
    *,
    excluded_prefixes: Iterable[str] = DEFAULT_EXCLUDED_PREFIXES,
) -> dict[str, object]:
    root = package_dir.resolve()
    excluded = tuple(excluded_prefixes)
    rows: list[dict[str, object]] = []
    for path in iter_package_files(root, excluded):
        stat = path.stat()
        rows.append(
            {
                "relative_path": relative_posix(path, root),
                "size_bytes": stat.st_size,
                "sha256": sha256_file(path),
            }
        )
    return {
        "package_dir": str(root),
        "excluded_prefixes": list(excluded),
        "file_count": len(rows),
        "total_size_bytes": sum(int(row["size_bytes"]) for row in rows),
        "files": rows,
        "status": "PASS",
        "failure_count": 0,
        "failures": [],
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def validate_inventory(
    package_dir: Path,
    inventory: Mapping[str, object],
) -> dict[str, object]:
    root = package_dir.resolve()
    failures: list[str] = []
    excluded_prefixes = tuple(str(prefix) for prefix in inventory.get("excluded_prefixes", DEFAULT_EXCLUDED_PREFIXES))
    expected_files = {}
    for row in inventory.get("files", []):
        if not isinstance(row, Mapping):
            failures.append("inventory:file_row_not_object")
            continue
        relative_path = str(row.get("relative_path", ""))
        expected_files[relative_path] = row
        path = root / relative_path
        if is_excluded(relative_path, excluded_prefixes):
            failures.append(f"inventory:listed_excluded_file:{relative_path}")
            continue
        if not path.is_file():
            failures.append(f"inventory:missing_file:{relative_path}")
            continue
        size_bytes = path.stat().st_size
        sha256 = sha256_file(path)
        if int(row.get("size_bytes", -1)) != size_bytes:
            failures.append(f"inventory:size_mismatch:{relative_path}")
        if str(row.get("sha256", "")) != sha256:
            failures.append(f"inventory:sha256_mismatch:{relative_path}")

    actual_files = {
        relative_posix(path, root)
        for path in iter_package_files(root, excluded_prefixes)
    }
    for relative_path in sorted(actual_files - set(expected_files)):
        failures.append(f"inventory:unexpected_file:{relative_path}")

    return {
        "package_dir": str(root),
        "status": "PASS" if not failures else "FAIL",
        "file_count": len(expected_files),
        "actual_file_count": len(actual_files),
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["relative_path", "size_bytes", "sha256"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_text(path: Path, inventory: Mapping[str, object], validation: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion deployment package inventory",
        f"package_dir: {inventory['package_dir']}",
        f"status: {validation['status']}",
        f"file_count: {inventory['file_count']}",
        f"total_size_bytes: {inventory['total_size_bytes']}",
        f"failure_count: {validation['failure_count']}",
        f"excluded_prefixes: {','.join(str(prefix) for prefix in inventory.get('excluded_prefixes', []))}",
        "runtime_truth_used: False",
        "Coordinate contract: inventory records deployment artifacts only; it does not transform NED/RDF/FRD values.",
    ]
    for failure in validation.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_inventory(package_dir: Path, output_dir: Path) -> dict[str, object]:
    inventory = build_inventory(package_dir)
    validation = validate_inventory(package_dir, inventory)
    payload = {**inventory, "status": validation["status"], "failure_count": validation["failure_count"], "failures": validation["failures"]}
    write_json(output_dir / "observation_fusion_deployment_inventory.json", payload)
    write_csv(output_dir / "observation_fusion_deployment_inventory.csv", list(payload["files"]))
    write_text(output_dir / "observation_fusion_deployment_inventory.txt", payload, validation)
    return payload


def load_inventory(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"inventory JSON must contain an object: {path}")
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export and validate checksums for an observation-fusion deployment package.")
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--inventory-json", type=Path, default=None, help="Validate an existing inventory instead of exporting a new one.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir if args.output_dir is not None else args.package_dir / "inventory"
    if args.inventory_json is not None:
        inventory = load_inventory(args.inventory_json)
        validation = validate_inventory(args.package_dir, inventory)
        payload = {**inventory, "status": validation["status"], "failure_count": validation["failure_count"], "failures": validation["failures"]}
        output_dir.mkdir(parents=True, exist_ok=True)
        write_text(output_dir / "observation_fusion_deployment_inventory_validation.txt", payload, validation)
    else:
        payload = export_inventory(args.package_dir, output_dir)
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
