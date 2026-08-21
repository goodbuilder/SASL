"""Portable provenance and immutable input snapshots for session packages.

The helpers in this module deliberately do not own a package lifecycle.  They
copy evidence into an already-created session root and return JSON-safe
artifact entries that can be handed to the package manifest registrar.

Two safety rules are enforced at this boundary:

* generated JSON never contains a secret value or an absolute machine path;
* a calibration reference is followed only while it remains inside an
  explicitly established source root (or names an explicitly supplied file).

Opaque input files are copied byte-for-byte.  Their contents are not rewritten
or interpreted as generated package metadata.
"""

from __future__ import annotations

import dataclasses
import hashlib
import importlib.metadata
import json
import math
import mimetypes
import os
import platform
import re
import shutil
import subprocess
import sys
import uuid
from collections import deque
from datetime import date, datetime
from enum import Enum
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Iterable, Mapping, Sequence


PRODUCER = "workstation.package_provenance"
REDACTED = "[REDACTED]"
ABSOLUTE_PATH = "<absolute-path>"
SOFTWARE_PROVENANCE_SCHEMA = "workstation-software-provenance-v1"
CALIBRATION_BUNDLE_SCHEMA = "workstation-calibration-bundle-v1"

_SAFE_COMPONENT = re.compile(r"[^A-Za-z0-9._-]+")
_ENV_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_ENV_REFERENCE = re.compile(r"^\$\{?([A-Za-z_][A-Za-z0-9_]*)\}?$")
_SECRET_KEY = re.compile(
    r"(?:^|[_\-.])(?:"
    r"pass(?:word|phrase)?|passwd|pwd|secret|token|credential(?:s)?|"
    r"api[_-]?key|access[_-]?key|private[_-]?key|client[_-]?secret|"
    r"authorization|auth[_-]?header|cookie|session[_-]?(?:key|secret)"
    r")(?:$|[_\-.])",
    re.IGNORECASE,
)
_SECRET_FILE = re.compile(
    r"(?:^|[/\\])(?:\.env(?:\..*)?|\.netrc|credentials?(?:\..*)?|"
    r"secrets?(?:\..*)?|id_(?:rsa|dsa|ecdsa|ed25519)(?:\..*)?|"
    r"[^/\\]+\.(?:pem|p12|pfx|key))$",
    re.IGNORECASE,
)
_PATH_REFERENCE_KEY = re.compile(
    r"(?:^|_)(?:path|paths|file|files|image|images|pair_selection|"
    r"capture_metadata)(?:_|$)",
    re.IGNORECASE,
)
_PATH_SUFFIX = re.compile(
    r"\.(?:json|ya?ml|toml|ini|cfg|txt|csv|png|jpe?g|tiff?|bmp|npy|npz)$",
    re.IGNORECASE,
)
_POSIX_ABSOLUTE_IN_TEXT = re.compile(
    r"(?<![A-Za-z0-9_:/])/(?:[^/\s\"'`<>]+/)+[^/\s\"'`<>:,;)}\]]*"
)
_WINDOWS_ABSOLUTE_IN_TEXT = re.compile(
    r"(?<![A-Za-z0-9_])[A-Za-z]:\\(?:[^\\\s\"'`<>]+\\)+"
    r"[^\\\s\"'`<>:,;)}\]]*"
)
_URI_CREDENTIALS = re.compile(
    r"(?P<scheme>[A-Za-z][A-Za-z0-9+.-]*://)[^/@\s:]+:[^/@\s]+@"
)
_BEARER_CREDENTIAL = re.compile(r"(?i)\b(Bearer|Basic)\s+[A-Za-z0-9+/=_\-.]+")
_QUERY_CREDENTIAL = re.compile(
    r"(?i)([?&](?:password|passwd|pwd|secret|token|api[_-]?key|access[_-]?key)=)"
    r"[^&#\s]*"
)
_PATCH_SECRET_ASSIGNMENT = re.compile(
    r"(?i)([\"']?[A-Za-z0-9_.-]*(?:password|passwd|pwd|secret|token|"
    r"credential|api[_-]?key|access[_-]?key|private[_-]?key|authorization)"
    r"[A-Za-z0-9_.-]*[\"']?\s*(?:=|:)\s*)"
)
_PRIVATE_KEY_BEGIN = re.compile(r"-----BEGIN [A-Z0-9 ]*PRIVATE KEY-----")
_PRIVATE_KEY_END = re.compile(r"-----END [A-Z0-9 ]*PRIVATE KEY-----")
_PATCH_HUNK_OMISSION = (
    "# Diff hunk omitted because a credential value continues on following lines.\n"
)


def _safe_component(value: object, *, fallback: str = "artifact") -> str:
    text = _SAFE_COMPONENT.sub("_", str(value or "")).strip("._")
    return text or fallback


def _is_environment_reference_key(key: str) -> bool:
    normalized = re.sub(r"[^a-z0-9]+", "_", key.lower()).strip("_")
    return normalized.endswith(("_env", "_env_var", "_environment_variable")) or normalized in {
        "env_var",
        "environment_variable",
    }


def _is_environment_reference(value: object) -> bool:
    if not isinstance(value, str):
        return False
    stripped = value.strip()
    return bool(_ENV_NAME.fullmatch(stripped) or _ENV_REFERENCE.fullmatch(stripped))


def _looks_secret_key(key: object) -> bool:
    normalized = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", str(key))
    normalized = re.sub(r"[^A-Za-z0-9]+", "_", normalized).strip("_")
    return bool(_SECRET_KEY.search(f"_{normalized}_"))


def _portable_absolute_path(value: str) -> str | None:
    """Return a non-identifying representation when *value* is one path."""

    stripped = value.strip()
    if not stripped:
        return None
    if PurePosixPath(stripped).is_absolute():
        name = PurePosixPath(stripped).name
        return f"{ABSOLUTE_PATH}/{name}" if name else ABSOLUTE_PATH
    windows = PureWindowsPath(stripped)
    if windows.is_absolute():
        name = windows.name
        return f"{ABSOLUTE_PATH}/{name}" if name else ABSOLUTE_PATH
    return None


def _portable_path_match(match: re.Match[str]) -> str:
    raw = match.group(0)
    name = PureWindowsPath(raw).name if "\\" in raw else PurePosixPath(raw).name
    return f"{ABSOLUTE_PATH}/{name}" if name else ABSOLUTE_PATH


def _redact_inline_text(value: str) -> str:
    portable = _portable_absolute_path(value)
    if portable is not None:
        return portable
    result = _URI_CREDENTIALS.sub(lambda match: f"{match.group('scheme')}{REDACTED}@", value)
    result = _BEARER_CREDENTIAL.sub(lambda match: f"{match.group(1)} {REDACTED}", result)
    result = _QUERY_CREDENTIAL.sub(lambda match: f"{match.group(1)}{REDACTED}", result)
    result = _POSIX_ABSOLUTE_IN_TEXT.sub(_portable_path_match, result)
    result = _WINDOWS_ABSOLUTE_IN_TEXT.sub(_portable_path_match, result)
    return result


def _json_scalar(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return _redact_inline_text(value) if isinstance(value, str) else value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("JSON evidence contains a non-finite number")
        return value
    if isinstance(value, Path):
        return _redact_inline_text(str(value))
    if isinstance(value, Enum):
        return _json_value(value.value)
    if isinstance(value, (date, datetime)):
        return value.isoformat()
    if isinstance(value, (bytes, bytearray, memoryview)):
        payload = bytes(value)
        return {
            "bytes_sha256": hashlib.sha256(payload).hexdigest(),
            "size_bytes": len(payload),
        }
    return None


def _redact_every_value(value: Any, *, key: str = "") -> Any:
    if isinstance(value, Mapping):
        return {
            str(child_key): (
                _json_value(child_value)
                if _is_environment_reference_key(str(child_key))
                and _is_environment_reference(child_value)
                else _redact_every_value(child_value, key=str(child_key))
            )
            for child_key, child_value in sorted(value.items(), key=lambda item: str(item[0]))
        }
    if isinstance(value, (list, tuple)):
        return [_redact_every_value(item, key=key) for item in value]
    if isinstance(value, (set, frozenset)):
        redacted = [_redact_every_value(item, key=key) for item in value]
        return sorted(redacted, key=lambda item: json.dumps(item, sort_keys=True, ensure_ascii=False))
    return REDACTED


def _json_value(value: Any) -> Any:
    scalar = _json_scalar(value)
    if scalar is not None or value is None:
        return scalar
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return _json_value(dataclasses.asdict(value))
    to_mapping = getattr(value, "to_mapping", None)
    if callable(to_mapping):
        return _json_value(to_mapping())
    if isinstance(value, Mapping):
        normalized: dict[str, Any] = {}
        for raw_key, child in sorted(value.items(), key=lambda item: str(item[0])):
            key = str(raw_key)
            if _looks_secret_key(key) and not (
                _is_environment_reference_key(key) and _is_environment_reference(child)
            ):
                normalized[key] = _redact_every_value(child, key=key)
            else:
                normalized[key] = _json_value(child)
        return normalized
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, (set, frozenset)):
        normalized = [_json_value(item) for item in value]
        return sorted(normalized, key=lambda item: json.dumps(item, sort_keys=True, ensure_ascii=False))
    raise TypeError(f"unsupported JSON evidence value: {type(value).__name__}")


def redact_secrets(value: Any) -> Any:
    """Return a deterministic JSON-safe value with credential values removed.

    Mapping keys are retained, including environment variable names such as
    ``NTRIP_PASSWORD``.  A field that explicitly identifies an environment
    variable (for example ``password_env``) retains that variable name while
    never resolving or reading it from the process environment.
    """

    return _json_value(value)


def canonical_json_bytes(value: Any) -> bytes:
    """Encode redacted, path-portable JSON using a stable canonical layout."""

    return json.dumps(
        redact_secrets(value),
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def sha256_file(path: Path | str, *, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _session_destination(session_root: Path | str, relative_path: str | Path) -> tuple[Path, Path]:
    root = Path(session_root).expanduser().resolve()
    relative = Path(relative_path)
    if relative.is_absolute() or not relative.parts or relative == Path("."):
        raise ValueError("package destination must be a non-empty relative path")
    destination = (root / relative).resolve(strict=False)
    try:
        destination.relative_to(root)
    except ValueError as exc:
        raise ValueError("package destination escapes the session root") from exc
    return root, destination


def _atomic_write_bytes(destination: Path, payload: bytes) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("xb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, destination)
        _fsync_directory(destination.parent)
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def _artifact_id(
    *,
    role: str,
    relative_path: str,
    status: str,
    sha256: str | None,
) -> str:
    identity = json.dumps(
        {
            "relative_path": relative_path,
            "role": role,
            "sha256": sha256,
            "status": status,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return f"artifact-{hashlib.sha256(identity).hexdigest()[:24]}"


def _media_type(path: Path) -> str:
    guessed, _encoding = mimetypes.guess_type(path.name)
    if path.suffix.lower() in {".patch", ".diff"}:
        return "text/x-diff"
    return guessed or "application/octet-stream"


def _artifact_descriptor(
    path: Path,
    session_root: Path,
    *,
    role: str,
    classification: str = "input",
    source: str,
    required: bool = True,
    media_type: str | None = None,
    data_schema: str | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    try:
        relative_path = resolved.relative_to(session_root.resolve()).as_posix()
    except ValueError as exc:
        raise ValueError("artifact is outside the session root") from exc
    digest = sha256_file(resolved)
    role_text = str(role)
    return {
        "artifact_id": _artifact_id(
            role=role_text,
            relative_path=relative_path,
            status="present",
            sha256=digest,
        ),
        "role": role_text,
        "classification": str(classification),
        "relative_path": relative_path,
        "producer": PRODUCER,
        "source": _redact_inline_text(str(source)),
        "required": bool(required),
        "status": "present",
        "media_type": str(media_type or _media_type(resolved)),
        "schema": data_schema,
        "size_bytes": resolved.stat().st_size,
        "sha256": digest,
        "metadata": redact_secrets(dict(metadata or {})),
    }


def _absent_descriptor(
    *,
    role: str,
    relative_path: str,
    source: str,
    status: str,
    reason: str,
    required: bool = True,
    media_type: str = "application/octet-stream",
    data_schema: str | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    if status not in {"missing", "failed", "not_applicable"}:
        raise ValueError("absent artifact status must be missing, failed, or not_applicable")
    safe_relative = PurePosixPath(relative_path).as_posix()
    details = dict(metadata or {})
    details["reason"] = str(reason)
    return {
        "artifact_id": _artifact_id(
            role=str(role),
            relative_path=safe_relative,
            status=status,
            sha256=None,
        ),
        "role": str(role),
        "classification": "input",
        "relative_path": safe_relative,
        "producer": PRODUCER,
        "source": _redact_inline_text(str(source)),
        "required": bool(required),
        "status": status,
        "media_type": str(media_type),
        "schema": data_schema,
        "size_bytes": None,
        "sha256": None,
        "metadata": redact_secrets(details),
    }


def write_atomic_json_snapshot(
    session_root: Path | str,
    relative_path: str | Path,
    value: Any,
    *,
    role: str,
    classification: str = "input",
    source: str = "generated",
    required: bool = True,
    data_schema: str | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Atomically persist canonical redacted JSON and describe the artifact."""

    root, destination = _session_destination(session_root, relative_path)
    _atomic_write_bytes(destination, canonical_json_bytes(value) + b"\n")
    return _artifact_descriptor(
        destination,
        root,
        role=role,
        classification=classification,
        source=source,
        required=required,
        media_type="application/json",
        data_schema=data_schema,
        metadata=metadata,
    )


def _copy_file_content_addressed(
    source: Path,
    session_root: Path,
    destination_subdir: str | Path,
) -> Path:
    source = source.expanduser().resolve(strict=True)
    if not source.is_file():
        raise ValueError("input snapshot source must be a regular file")
    _root, destination_dir_marker = _session_destination(
        session_root,
        Path(destination_subdir) / ".destination",
    )
    destination_dir = destination_dir_marker.parent
    destination_dir.mkdir(parents=True, exist_ok=True)
    temporary = destination_dir / f".input.{uuid.uuid4().hex}.tmp"
    digest = hashlib.sha256()
    size = 0
    try:
        with source.open("rb") as reader, temporary.open("xb") as writer:
            while True:
                chunk = reader.read(1024 * 1024)
                if not chunk:
                    break
                writer.write(chunk)
                digest.update(chunk)
                size += len(chunk)
            writer.flush()
            os.fsync(writer.fileno())
        suffix = _SAFE_COMPONENT.sub("", source.suffix.lower())
        stem = _safe_component(source.stem, fallback="input")
        destination = destination_dir / f"{stem}-{digest.hexdigest()}{suffix}"
        if destination.exists():
            if destination.stat().st_size != size or sha256_file(destination) != digest.hexdigest():
                raise RuntimeError("content-addressed input collision")
            temporary.unlink()
        else:
            os.replace(temporary, destination)
            _fsync_directory(destination_dir)
        return destination
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def snapshot_input_file(
    source_path: Path | str,
    session_root: Path | str,
    *,
    role: str,
    destination_subdir: str | Path = "inputs",
    source: str | None = None,
    required: bool = True,
    media_type: str | None = None,
    data_schema: str | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Copy one active input byte-for-byte under a content-addressed name."""

    root = Path(session_root).expanduser().resolve()
    input_path = Path(source_path).expanduser().resolve(strict=True)
    destination = _copy_file_content_addressed(input_path, root, destination_subdir)
    return _artifact_descriptor(
        destination,
        root,
        role=role,
        source=source or input_path.name,
        required=required,
        media_type=media_type or _media_type(input_path),
        data_schema=data_schema,
        metadata={"source_name": input_path.name, **dict(metadata or {})},
    )


def snapshot_input_files(
    session_root: Path | str,
    inputs: Mapping[str, Path | str | None],
    *,
    destination_subdir: str | Path = "inputs",
    required: bool = True,
) -> list[dict[str, Any]]:
    """Snapshot a named set of active inputs, recording absent inputs."""

    artifacts: list[dict[str, Any]] = []
    for role, source_path in sorted(inputs.items(), key=lambda item: str(item[0])):
        safe_role = _safe_component(role)
        if source_path is None or not Path(source_path).expanduser().is_file():
            source_name = "unspecified" if source_path is None else Path(source_path).name
            artifacts.append(
                _absent_descriptor(
                    role=str(role),
                    relative_path=(Path(destination_subdir) / safe_role / source_name).as_posix(),
                    source=source_name,
                    status="missing",
                    reason="active_input_missing",
                    required=required,
                    metadata={"source_name": source_name},
                )
            )
            continue
        artifacts.append(
            snapshot_input_file(
                source_path,
                session_root,
                role=str(role),
                destination_subdir=Path(destination_subdir) / safe_role,
                required=required,
            )
        )
    return artifacts


def _run_git(repository: Path, *arguments: str) -> subprocess.CompletedProcess[bytes]:
    environment = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith("GIT_") and key not in {"SSH_ASKPASS", "GIT_ASKPASS"}
    }
    environment.update(
        {
            "GIT_OPTIONAL_LOCKS": "0",
            "LC_ALL": "C",
        }
    )
    return subprocess.run(
        ["git", *arguments],
        cwd=repository,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        timeout=20,
    )


def _decode_git(output: bytes) -> str:
    return output.decode("utf-8", errors="replace").replace("\r\n", "\n")


def _patch_secret_value_continues(value_text: str) -> bool:
    """Conservatively identify secret assignments whose value spans lines."""

    value = value_text.strip()
    if not value or value.startswith(("#", "//", "/*")):
        return True
    if value.startswith(("|", ">")):
        return True
    if value.endswith("\\"):
        return True
    for quote in ('"""', "'''"):
        if value.startswith(quote) and value.count(quote) < 2:
            return True
    if value[0] in "[{(":
        closing = {"[": "]", "{": "}", "(": ")"}[value[0]]
        if not value.rstrip(",").endswith(closing):
            return True
    if value[0] in {'"', "'"}:
        quote = value[0]
        escaped = False
        for character in value[1:]:
            if character == quote and not escaped:
                return False
            escaped = character == "\\" and not escaped
            if character != "\\":
                escaped = False
        return True
    return False


def _redact_patch(payload: bytes) -> bytes:
    text = _decode_git(payload)
    output: list[str] = []
    omit_section = False
    omit_hunk = False
    private_key = False
    for line in text.splitlines(keepends=True):
        ending = "\n" if line.endswith("\n") else ""
        body = line[:-1] if ending else line
        if body.startswith("diff --git "):
            omit_hunk = False
            private_key = False
            omit_section = bool(_SECRET_FILE.search(body))
            if omit_section:
                output.extend(
                    [
                        "diff --git a/<redacted-sensitive-path> b/<redacted-sensitive-path>\n",
                        "# Diff omitted because the tracked path may contain credentials.\n",
                    ]
                )
                continue
        if omit_section:
            continue
        if body.startswith("@@"):
            omit_hunk = False
        elif omit_hunk:
            continue
        if _PRIVATE_KEY_BEGIN.search(body):
            private_key = True
            prefix = body[:1] if body[:1] in {"+", "-", " "} else ""
            output.append(f"{prefix}{REDACTED} private key block{ending}")
            continue
        if private_key:
            if _PRIVATE_KEY_END.search(body):
                private_key = False
            continue
        match = _PATCH_SECRET_ASSIGNMENT.search(body)
        if match:
            key_text = match.group(1)
            value_text = body[match.end() :]
            if not (
                re.search(r"(?i)(?:_env|_env_var|_environment_variable)[\"']?\s*(?:=|:)\s*$", key_text)
                and _is_environment_reference(value_text.strip().strip("\"',"))
            ):
                body = f"{body[:match.end()]}{REDACTED}"
                if _patch_secret_value_continues(value_text):
                    body = _redact_inline_text(body)
                    output.append(body + (ending or "\n"))
                    output.append(_PATCH_HUNK_OMISSION)
                    omit_hunk = True
                    continue
        body = _redact_inline_text(body)
        output.append(body + ending)
    return "".join(output).encode("utf-8")


def _installed_package_versions(names: Iterable[str] | None) -> list[dict[str, str]]:
    packages: set[tuple[str, str]] = set()
    if names is not None:
        for requested in sorted({str(name) for name in names}, key=str.casefold):
            try:
                distribution = importlib.metadata.distribution(requested)
            except importlib.metadata.PackageNotFoundError:
                continue
            name = str(distribution.metadata.get("Name") or requested)
            packages.add((name, str(distribution.version)))
    else:
        for distribution in importlib.metadata.distributions():
            name = str(distribution.metadata.get("Name") or "").strip()
            version = str(distribution.version or "").strip()
            if name and version:
                packages.add((name, version))
    return [
        {"name": name, "version": version}
        for name, version in sorted(packages, key=lambda item: (item[0].casefold(), item[1]))
    ]


def collect_software_provenance(
    session_root: Path | str,
    repository_root: Path | str,
    *,
    relative_path: str | Path = "inputs/provenance/software.json",
    patch_relative_path: str | Path = "inputs/provenance/tracked-diff.patch",
    package_names: Iterable[str] | None = None,
) -> list[dict[str, Any]]:
    """Collect Git, Python, platform, package, and scrubbed dirty-patch evidence."""

    root = Path(session_root).expanduser().resolve()
    repository = Path(repository_root).expanduser().resolve()
    artifacts: list[dict[str, Any]] = []
    git_payload: dict[str, Any] = {
        "available": False,
        "head": None,
        "branch": None,
        "dirty": None,
        "status": [],
        "tracked_diff": None,
    }
    if shutil.which("git") is not None and repository.is_dir():
        try:
            inside = _run_git(repository, "rev-parse", "--is-inside-work-tree")
        except (OSError, subprocess.TimeoutExpired):
            inside = None
        if inside is not None and inside.returncode == 0 and _decode_git(inside.stdout).strip() == "true":
            git_payload["available"] = True
            head = _run_git(repository, "rev-parse", "--verify", "HEAD")
            branch = _run_git(repository, "symbolic-ref", "--quiet", "--short", "HEAD")
            status = _run_git(
                repository,
                "status",
                "--porcelain=v1",
                "--untracked-files=all",
            )
            git_payload["head"] = _decode_git(head.stdout).strip() if head.returncode == 0 else None
            git_payload["branch"] = _decode_git(branch.stdout).strip() if branch.returncode == 0 else None
            status_lines = sorted(line for line in _decode_git(status.stdout).splitlines() if line)
            git_payload["status"] = [_redact_inline_text(line) for line in status_lines]
            git_payload["dirty"] = bool(status_lines)
            if status_lines:
                diff = _run_git(
                    repository,
                    "diff",
                    "--no-ext-diff",
                    "--no-textconv",
                    "--full-index",
                    "HEAD",
                    "--",
                )
                patch_bytes = _redact_patch(diff.stdout) if diff.returncode == 0 else b""
                _patch_root, patch_destination = _session_destination(root, patch_relative_path)
                _atomic_write_bytes(patch_destination, patch_bytes)
                patch_descriptor = _artifact_descriptor(
                    patch_destination,
                    root,
                    role="software_tracked_git_diff",
                    source="git_diff_head",
                    required=False,
                    media_type="text/x-diff",
                    data_schema=None,
                    metadata={
                        "credential_redaction": "conservative",
                        "tracked_changes_present": bool(patch_bytes),
                    },
                )
                artifacts.append(patch_descriptor)
                git_payload["tracked_diff"] = {
                    "relative_path": patch_descriptor["relative_path"],
                    "sha256": patch_descriptor["sha256"],
                    "size_bytes": patch_descriptor["size_bytes"],
                }

    provenance = {
        "schema": SOFTWARE_PROVENANCE_SCHEMA,
        "git": git_payload,
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
            "version_info": [sys.version_info.major, sys.version_info.minor, sys.version_info.micro],
        },
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
        },
        "installed_packages": _installed_package_versions(package_names),
    }
    artifacts.append(
        write_atomic_json_snapshot(
            root,
            relative_path,
            provenance,
            role="software_provenance",
            source="runtime_and_repository",
            data_schema=SOFTWARE_PROVENANCE_SCHEMA,
        )
    )
    return artifacts


def _is_within(path: Path, roots: Sequence[Path]) -> bool:
    for root in roots:
        try:
            path.relative_to(root)
            return True
        except ValueError:
            continue
    return False


def _iter_calibration_paths(
    values: Mapping[str, Path | str | None] | Iterable[Path | str | None],
    *,
    default_role: str,
) -> list[tuple[str, Path | str | None]]:
    if isinstance(values, Mapping):
        return [(str(role), path) for role, path in sorted(values.items(), key=lambda item: str(item[0]))]
    return [(default_role, path) for path in values]


def _extract_path_references(value: Any) -> list[tuple[str, str]]:
    found: list[tuple[str, str]] = []

    def visit(current: Any, pointer: str, path_context: bool = False) -> None:
        if isinstance(current, Mapping):
            for raw_key, child in sorted(current.items(), key=lambda item: str(item[0])):
                key = str(raw_key)
                child_pointer = f"{pointer}/{key}" if pointer else f"/{key}"
                child_context = path_context or bool(_PATH_REFERENCE_KEY.search(key))
                if key.lower() == "root":
                    child_context = False
                visit(child, child_pointer, child_context)
            return
        if isinstance(current, (list, tuple)):
            for index, child in enumerate(current):
                visit(child, f"{pointer}/{index}", path_context)
            return
        if path_context and isinstance(current, str) and current.strip() and _PATH_SUFFIX.search(current.strip()):
            found.append((pointer or "/", current.strip()))

    visit(value, "")
    return sorted(set(found), key=lambda item: (item[0], item[1]))


def _reference_role(pointer: str, raw_reference: str) -> str:
    lowered = f"{pointer}/{Path(raw_reference).name}".lower()
    if "pair_selection" in lowered:
        return "calibration_pair_selection"
    if "capture_metadata" in lowered or "capture_" in Path(raw_reference).stem.lower():
        return "calibration_capture_metadata"
    if "intrinsic" in lowered:
        return "calibration_intrinsics_profile"
    if "extrinsic" in lowered:
        return "calibration_extrinsics_profile"
    if "/left" in lowered and _PATH_SUFFIX.search(raw_reference):
        return "calibration_selected_left_image"
    if "/right" in lowered and _PATH_SUFFIX.search(raw_reference):
        return "calibration_selected_right_image"
    return "calibration_referenced_input"


def _reference_path(raw_reference: str, referring_file: Path) -> Path:
    candidate = Path(raw_reference).expanduser()
    if not candidate.is_absolute():
        candidate = referring_file.parent / candidate
    return candidate.resolve(strict=False)


def _portable_reference_name(raw_reference: str) -> str:
    name = (
        PureWindowsPath(raw_reference).name
        if "\\" in raw_reference
        else PurePosixPath(raw_reference).name
    )
    return _safe_component(name, fallback="referenced_input")


def _expected_relative_path(destination_subdir: Path, role: str, source_name: str) -> str:
    return (destination_subdir / _safe_component(role) / _safe_component(source_name)).as_posix()


def _calibration_queue_item(
    source_path: Path | str,
    role: str,
    *,
    explicit: bool,
    referred_by: str = "",
    json_pointer: str = "",
) -> tuple[Path | str, str, bool, str, str]:
    return source_path, role, explicit, referred_by, json_pointer


def collect_calibration_bundle(
    session_root: Path | str,
    profile_paths: Mapping[str, Path | str | None] | Iterable[Path | str | None],
    *,
    explicit_source_paths: Mapping[str, Path | str | None] | Iterable[Path | str | None] = (),
    source_roots: Iterable[Path | str] = (),
    destination_subdir: str | Path = "inputs/calibration",
) -> list[dict[str, Any]]:
    """Copy active calibration inputs and their safe recursive references.

    Supplied profiles and support files establish explicit source boundaries.
    A relative or absolute JSON reference may be followed only inside one of
    those parent directories or an explicitly supplied ``source_roots`` entry.
    Existing symlinks are resolved before that check, so a symlink cannot be
    used to escape the boundary.
    """

    root = Path(session_root).expanduser().resolve()
    destination = Path(destination_subdir)
    profiles = _iter_calibration_paths(profile_paths, default_role="calibration_profile")
    supports = _iter_calibration_paths(explicit_source_paths, default_role="calibration_support")
    allowed_roots: list[Path] = [Path(item).expanduser().resolve(strict=False) for item in source_roots]
    exact_explicit: set[Path] = set()
    queue: deque[tuple[Path | str, str, bool, str, str]] = deque()
    for role, raw_path in profiles:
        if raw_path is None:
            queue.append(_calibration_queue_item("", role, explicit=True))
            continue
        candidate = Path(raw_path).expanduser().resolve(strict=False)
        queue.append(_calibration_queue_item(candidate, role, explicit=True))
        if candidate.is_dir():
            allowed_roots.append(candidate)
        else:
            exact_explicit.add(candidate)
            allowed_roots.append(candidate.parent)
    for role, raw_path in supports:
        if raw_path is None:
            queue.append(_calibration_queue_item("", role, explicit=True))
            continue
        candidate = Path(raw_path).expanduser().resolve(strict=False)
        queue.append(_calibration_queue_item(candidate, role, explicit=True))
        if candidate.is_dir():
            allowed_roots.append(candidate)
        else:
            exact_explicit.add(candidate)

    allowed_roots = sorted(set(allowed_roots), key=lambda path: path.as_posix())
    for profile_role, raw_path in profiles:
        if raw_path is None:
            continue
        candidate = Path(raw_path).expanduser().resolve(strict=False)
        selection = candidate.parent / "calibration_pair_selection.json"
        if selection.is_file() and selection != candidate:
            queue.append(
                _calibration_queue_item(
                    selection,
                    "calibration_pair_selection",
                    explicit=False,
                    referred_by=_safe_component(profile_role),
                    json_pointer="/auto_discovered_pair_selection",
                )
            )

    artifacts: list[dict[str, Any]] = []
    seen_work: set[tuple[str, str]] = set()
    scanned_json: set[Path] = set()

    while queue:
        raw_path, role, explicit, referred_by, json_pointer = queue.popleft()
        raw_text = str(raw_path or "")
        source_name = _portable_reference_name(raw_text) if raw_text else "unspecified"
        candidate = Path(raw_text).expanduser().resolve(strict=False) if raw_text else None
        work_key = (str(candidate), role)
        if work_key in seen_work:
            continue
        seen_work.add(work_key)
        metadata = {
            "source_name": source_name,
            "referred_by": referred_by or None,
            "json_pointer": json_pointer or None,
        }
        expected_relative = _expected_relative_path(destination, role, source_name)

        if candidate is None or not candidate.exists():
            artifacts.append(
                _absent_descriptor(
                    role=role,
                    relative_path=expected_relative,
                    source=source_name,
                    status="missing",
                    reason="calibration_reference_missing" if referred_by else "calibration_input_missing",
                    media_type=_media_type(Path(source_name)),
                    metadata=metadata,
                )
            )
            continue
        resolved = candidate.resolve(strict=True)
        if not explicit and resolved not in exact_explicit and not _is_within(resolved, allowed_roots):
            artifacts.append(
                _absent_descriptor(
                    role=role,
                    relative_path=expected_relative,
                    source=source_name,
                    status="failed",
                    reason="outside_allowed_calibration_sources",
                    media_type=_media_type(Path(source_name)),
                    metadata=metadata,
                )
            )
            continue
        if not resolved.is_file():
            artifacts.append(
                _absent_descriptor(
                    role=role,
                    relative_path=expected_relative,
                    source=source_name,
                    status="failed",
                    reason="calibration_directory_reference_not_followed",
                    media_type="application/octet-stream",
                    metadata=metadata,
                )
            )
            continue

        copied = snapshot_input_file(
            resolved,
            root,
            role=role,
            # A source may satisfy several calibration roles.  Store its
            # exact bytes once in a shared content-addressed directory and
            # let distinct manifest descriptors carry the role/provenance.
            destination_subdir=destination / "files",
            source=source_name,
            metadata=metadata,
        )
        artifacts.append(copied)

        image_match = re.fullmatch(r"(?:left|right)[_-](.+)", resolved.stem, re.IGNORECASE)
        if image_match and resolved.parent.name.lower() in {"left", "right"}:
            capture_id = _safe_component(image_match.group(1), fallback="unknown")
            capture_metadata = resolved.parent.parent / "capture_metadata" / f"capture_{capture_id}.json"
            queue.append(
                _calibration_queue_item(
                    capture_metadata,
                    "calibration_capture_metadata",
                    explicit=False,
                    referred_by=copied["relative_path"],
                    json_pointer="/inferred_capture_metadata",
                )
            )

        if resolved.suffix.lower() != ".json" or resolved in scanned_json:
            continue
        scanned_json.add(resolved)
        try:
            payload = json.loads(resolved.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError):
            artifacts.append(
                _absent_descriptor(
                    role="calibration_reference_scan",
                    relative_path=(destination / "reference_scan" / f"{source_name}.scan").as_posix(),
                    source=source_name,
                    status="failed",
                    reason="calibration_json_reference_scan_failed",
                    required=False,
                    media_type="application/json",
                    metadata={"referred_by": copied["relative_path"]},
                )
            )
            continue

        for pointer, raw_reference in _extract_path_references(payload):
            referenced = _reference_path(raw_reference, resolved)
            queue.append(
                _calibration_queue_item(
                    referenced,
                    _reference_role(pointer, raw_reference),
                    explicit=False,
                    referred_by=copied["relative_path"],
                    json_pointer=pointer,
                )
            )

        if isinstance(payload, Mapping) and isinstance(payload.get("included_keys"), list):
            selection_root_raw = payload.get("root")
            selection_root = (
                _reference_path(str(selection_root_raw), resolved)
                if isinstance(selection_root_raw, str) and selection_root_raw.strip()
                else resolved.parent
            )
            for raw_key in payload["included_keys"]:
                capture_id = _safe_component(Path(str(raw_key)).name, fallback="unknown")
                special_references = (
                    (selection_root / "left" / f"left_{capture_id}.png", "calibration_selected_left_image"),
                    (selection_root / "right" / f"right_{capture_id}.png", "calibration_selected_right_image"),
                    (
                        selection_root / "capture_metadata" / f"capture_{capture_id}.json",
                        "calibration_capture_metadata",
                    ),
                )
                for referenced, referenced_role in special_references:
                    queue.append(
                        _calibration_queue_item(
                            referenced,
                            referenced_role,
                            explicit=False,
                            referred_by=copied["relative_path"],
                            json_pointer="/included_keys",
                        )
                    )

    artifacts.sort(
        key=lambda item: (
            str(item["role"]),
            str(item["relative_path"]),
            str(item["status"]),
            str(item["artifact_id"]),
        )
    )
    bundle_payload = {
        "schema": CALIBRATION_BUNDLE_SCHEMA,
        "complete": all(item["status"] == "present" for item in artifacts if item["required"]),
        "artifacts": artifacts,
    }
    bundle_descriptor = write_atomic_json_snapshot(
        root,
        destination / "calibration_bundle.json",
        bundle_payload,
        role="calibration_bundle_index",
        source="calibration_collector",
        data_schema=CALIBRATION_BUNDLE_SCHEMA,
    )
    return [*artifacts, bundle_descriptor]


__all__ = [
    "ABSOLUTE_PATH",
    "CALIBRATION_BUNDLE_SCHEMA",
    "PRODUCER",
    "REDACTED",
    "SOFTWARE_PROVENANCE_SCHEMA",
    "canonical_json_bytes",
    "collect_calibration_bundle",
    "collect_software_provenance",
    "redact_secrets",
    "sha256_file",
    "snapshot_input_file",
    "snapshot_input_files",
    "write_atomic_json_snapshot",
]
