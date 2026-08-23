from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import pytest

from Workstation.workstation_core.package_provenance import (
    ABSOLUTE_PATH,
    REDACTED,
    _redact_patch,
    canonical_json_bytes,
    collect_calibration_bundle,
    collect_software_provenance,
    redact_secrets,
    snapshot_input_file,
    write_atomic_json_snapshot,
)
from Workstation.workstation_core.measurement_package import MeasurementPackage


ARTIFACT_KEYS = {
    "artifact_id",
    "role",
    "classification",
    "relative_path",
    "producer",
    "source",
    "required",
    "status",
    "media_type",
    "schema",
    "size_bytes",
    "sha256",
    "metadata",
}


def _artifact(artifacts: list[dict[str, object]], role: str, status: str = "present") -> dict[str, object]:
    matches = [item for item in artifacts if item["role"] == role and item["status"] == status]
    assert len(matches) == 1, (role, status, matches)
    return matches[0]


def _git(repository: Path, *arguments: str) -> None:
    subprocess.run(
        ["git", *arguments],
        cwd=repository,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


@pytest.mark.parametrize(
    ("fixture_name", "patch_payload", "forbidden_values"),
    [
        (
            "multiline-json",
            b"""diff --git a/settings.json b/settings.json
index 1111111..2222222 100644
--- a/settings.json
+++ b/settings.json
@@ -1 +1,6 @@
-{}
+{
+  \"password\":
+    \"json-secret-on-the-next-line\",
+  \"ordinary\": \"must-also-be-omitted-after-ambiguous-secret\"
+}
@@ -20 +20 @@
-mode = initial
+unrelated-next-hunk-remains = true
""",
            (
                b"json-secret-on-the-next-line",
                b"must-also-be-omitted-after-ambiguous-secret",
            ),
        ),
        (
            "yaml-block-scalar",
            b"""diff --git a/settings.yaml b/settings.yaml
index 1111111..2222222 100644
--- a/settings.yaml
+++ b/settings.yaml
@@ -1 +1,4 @@
-mode: initial
+private_key: |-
+  yaml-block-secret-line-one
+  yaml-block-secret-line-two
+mode: changed
@@ -20 +20 @@
-mode: initial
+unrelated-next-hunk-remains: true
""",
            (
                b"yaml-block-secret-line-one",
                b"yaml-block-secret-line-two",
            ),
        ),
    ],
)
def test_patch_redaction_omits_hunk_when_secret_value_continues_on_following_lines(
    fixture_name: str,
    patch_payload: bytes,
    forbidden_values: tuple[bytes, ...],
) -> None:
    redacted = _redact_patch(patch_payload)

    assert b"Diff hunk omitted" in redacted, fixture_name
    assert REDACTED.encode() in redacted, fixture_name
    for forbidden in forbidden_values:
        assert forbidden not in redacted, (fixture_name, forbidden, redacted)
    assert b"unrelated-next-hunk-remains" in redacted, fixture_name


def test_patch_redaction_scrubs_private_keys_and_uri_query_credentials() -> None:
    patch_payload = b"""diff --git a/runtime.conf b/runtime.conf
index 1111111..2222222 100644
--- a/runtime.conf
+++ b/runtime.conf
@@ -1 +1,8 @@
+endpoint = https://alice:uri-password-sentinel@receiver.invalid/feed?token=query-token-sentinel&mode=survey
+pem = value-follows
+-----BEGIN OPENSSH PRIVATE KEY-----
+private-key-body-sentinel
+-----END OPENSSH PRIVATE KEY-----
+mode = survey
"""

    redacted = _redact_patch(patch_payload)

    assert b"uri-password-sentinel" not in redacted
    assert b"query-token-sentinel" not in redacted
    assert b"private-key-body-sentinel" not in redacted
    assert redacted.count(REDACTED.encode()) >= 3
    assert b"mode = survey" in redacted


def test_patch_redaction_retains_environment_variable_name_references() -> None:
    patch_payload = b"""diff --git a/runtime.conf b/runtime.conf
index 1111111..2222222 100644
--- a/runtime.conf
+++ b/runtime.conf
@@ -1 +1,2 @@
+password_env = NTRIP_PASSWORD
+access_token_env: ${NTRIP_ACCESS_TOKEN}
"""

    redacted = _redact_patch(patch_payload)

    assert b"password_env = NTRIP_PASSWORD" in redacted
    assert b"access_token_env: ${NTRIP_ACCESS_TOKEN}" in redacted
    assert REDACTED.encode() not in redacted


def test_redaction_is_deterministic_json_safe_and_retains_environment_names(tmp_path: Path) -> None:
    source = {
        "password": "plain-password",
        "credentials": {
            "username": "operator",
            "password_env": "NTRIP_PASSWORD",
            "access_token": "plain-token",
        },
        "environment": {
            "NTRIP_PASSWORD": "environment-secret",
            "VISIBLE_MODE": "survey",
        },
        "endpoint": "https://alice:private@receiver.invalid/feed?token=query-secret",
        "config_path": tmp_path / "machine" / "settings.json",
        "unordered": {"zulu", "alpha"},
        "binary": b"not-written-verbatim",
    }

    redacted = redact_secrets(source)
    encoded = canonical_json_bytes(source)
    reversed_encoded = canonical_json_bytes(dict(reversed(list(source.items()))))

    assert encoded == reversed_encoded
    assert redacted["password"] == REDACTED
    assert redacted["credentials"]["username"] == REDACTED
    assert redacted["credentials"]["password_env"] == "NTRIP_PASSWORD"
    assert redacted["environment"]["NTRIP_PASSWORD"] == REDACTED
    assert redacted["environment"]["VISIBLE_MODE"] == "survey"
    assert redacted["config_path"] == f"{ABSOLUTE_PATH}/settings.json"
    assert redacted["unordered"] == ["alpha", "zulu"]
    assert redacted["binary"] == {
        "bytes_sha256": hashlib.sha256(b"not-written-verbatim").hexdigest(),
        "size_bytes": len(b"not-written-verbatim"),
    }
    text = encoded.decode("utf-8")
    for forbidden in (
        "plain-password",
        "plain-token",
        "environment-secret",
        "private",
        "query-secret",
        str(tmp_path),
        "not-written-verbatim",
    ):
        assert forbidden not in text
    json.loads(text)


def test_atomic_json_snapshot_has_canonical_bytes_hash_and_no_absolute_path(tmp_path: Path) -> None:
    session = tmp_path / "session"
    payload = {
        "z": 3,
        "a": {"token": "do-not-save", "source_path": tmp_path / "source" / "input.json"},
    }

    first = write_atomic_json_snapshot(
        session,
        "inputs/settings.json",
        payload,
        role="effective_settings",
        data_schema="settings-v1",
    )
    saved = session / str(first["relative_path"])
    first_bytes = saved.read_bytes()
    second = write_atomic_json_snapshot(
        session,
        "inputs/settings.json",
        dict(reversed(list(payload.items()))),
        role="effective_settings",
        data_schema="settings-v1",
    )

    assert set(first) == ARTIFACT_KEYS
    assert first == second
    assert first_bytes == saved.read_bytes() == canonical_json_bytes(payload) + b"\n"
    assert first["sha256"] == hashlib.sha256(first_bytes).hexdigest()
    assert str(tmp_path).encode() not in first_bytes
    assert b"do-not-save" not in first_bytes
    assert not list(saved.parent.glob(".*.tmp"))

    with pytest.raises(ValueError, match="escapes"):
        write_atomic_json_snapshot(session, "../outside.json", {}, role="invalid")


def test_active_profile_snapshot_is_byte_exact_content_addressed_and_portable(tmp_path: Path) -> None:
    source = tmp_path / "source" / "active-intrinsics.json"
    source.parent.mkdir()
    payload = b'{"schema":"active-intrinsics-test-v1"}\n'
    source.write_bytes(payload)
    session = tmp_path / "session"

    first = snapshot_input_file(source, session, role="active_intrinsics_profile")
    second = snapshot_input_file(source, session, role="active_intrinsics_profile")
    copied = session / str(first["relative_path"])

    assert set(first) == ARTIFACT_KEYS
    assert first == second
    assert copied.read_bytes() == payload
    assert first["sha256"] == hashlib.sha256(payload).hexdigest()
    assert str(first["sha256"]) in copied.name
    assert str(tmp_path) not in json.dumps(first)
    assert len(list(copied.parent.iterdir())) == 1

    source.unlink()
    assert copied.read_bytes() == payload


def test_descriptors_register_directly_with_measurement_package(tmp_path: Path) -> None:
    session = tmp_path / "session"
    package = MeasurementPackage.create(session, session_id="provenance-descriptor-test")
    present = write_atomic_json_snapshot(
        session,
        "inputs/effective-settings.json",
        {"mode": "survey"},
        role="effective_settings",
    )

    registered = package.register_descriptor(present)

    assert registered.to_mapping() == present
    assert registered.sha256 == hashlib.sha256(
        (session / str(present["relative_path"])).read_bytes()
    ).hexdigest()


@pytest.mark.skipif(shutil.which("git") is None, reason="git is required for provenance test")
def test_software_provenance_captures_dirty_git_and_scrubs_patch(tmp_path: Path) -> None:
    repository = tmp_path / "repository"
    repository.mkdir()
    _git(repository, "init", "-q")
    _git(repository, "config", "user.name", "Provenance Test")
    _git(repository, "config", "user.email", "provenance@example.invalid")
    tracked = repository / "settings.py"
    tracked.write_text("MODE = 'initial'\n", encoding="utf-8")
    _git(repository, "add", "settings.py")
    _git(repository, "commit", "-q", "-m", "initial")
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=repository,
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    machine_path = tmp_path / "operator-private" / "target.bin"
    tracked.write_text(
        "MODE = 'changed'\n"
        "PASSWORD = 'tracked-credential-must-not-leak'\n"
        f"DATA_PATH = '{machine_path}'\n",
        encoding="utf-8",
    )

    session = tmp_path / "session"
    first = collect_software_provenance(session, repository, package_names=[])
    patch = _artifact(first, "software_tracked_git_diff")
    software = _artifact(first, "software_provenance")
    patch_bytes = (session / str(patch["relative_path"])).read_bytes()
    software_bytes = (session / str(software["relative_path"])).read_bytes()
    software_payload = json.loads(software_bytes)

    assert software_payload["git"]["available"] is True
    assert software_payload["git"]["head"] == head
    assert software_payload["git"]["dirty"] is True
    assert software_payload["git"]["status"] == [" M settings.py"]
    assert software_payload["installed_packages"] == []
    assert b"MODE = 'changed'" in patch_bytes
    assert b"PASSWORD" in patch_bytes
    assert REDACTED.encode() in patch_bytes
    assert b"tracked-credential-must-not-leak" not in patch_bytes
    assert str(tmp_path).encode() not in patch_bytes
    assert str(tmp_path).encode() not in software_bytes
    assert b".git/" not in patch_bytes

    second = collect_software_provenance(session, repository, package_names=[])
    assert first == second
    assert patch_bytes == (session / str(patch["relative_path"])).read_bytes()
    assert software_bytes == (session / str(software["relative_path"])).read_bytes()


def test_calibration_bundle_copies_recursive_inputs_and_records_missing_and_outside_refs(
    tmp_path: Path,
) -> None:
    calibration = tmp_path / "calibration-source"
    (calibration / "left").mkdir(parents=True)
    (calibration / "right").mkdir()
    (calibration / "capture_metadata").mkdir()
    outside = tmp_path / "outside" / "private-support.txt"
    outside.parent.mkdir()
    outside.write_bytes(b"outside-must-not-be-copied")

    profile = calibration / "intrinsics.json"
    selection = calibration / "calibration_pair_selection.json"
    support = calibration / "support.json"
    left = calibration / "left" / "left_001.png"
    metadata = calibration / "capture_metadata" / "capture_001.json"
    profile.write_text(
        json.dumps({"pair_selection_path": "calibration_pair_selection.json", "K": [1, 0, 1]}),
        encoding="utf-8",
    )
    selection.write_text(
        json.dumps(
            {
                "left_paths": ["left/left_001.png"],
                "right_paths": ["right/right_001.png"],
                "support_file": "support.json",
                "external_file": str(outside),
            }
        ),
        encoding="utf-8",
    )
    support.write_text(
        json.dumps({"capture_metadata_path": "capture_metadata/capture_001.json"}),
        encoding="utf-8",
    )
    left.write_bytes(b"\x89PNG\r\n\x1a\nleft-calibration-image")
    metadata.write_text(json.dumps({"capture_id": "001", "zoom": 8}), encoding="utf-8")
    expected_bytes = {
        "calibration_intrinsics_profile": profile.read_bytes(),
        "calibration_pair_selection": selection.read_bytes(),
        "calibration_referenced_input": support.read_bytes(),
        "calibration_selected_left_image": left.read_bytes(),
        "calibration_capture_metadata": metadata.read_bytes(),
    }

    session = tmp_path / "session"
    artifacts = collect_calibration_bundle(
        session,
        {"calibration_intrinsics_profile": profile},
    )

    for role, expected in expected_bytes.items():
        descriptor = _artifact(artifacts, role)
        assert set(descriptor) == ARTIFACT_KEYS
        assert (session / str(descriptor["relative_path"])).read_bytes() == expected
        assert descriptor["sha256"] == hashlib.sha256(expected).hexdigest()

    missing = _artifact(artifacts, "calibration_selected_right_image", "missing")
    blocked = _artifact(artifacts, "calibration_referenced_input", "failed")
    assert missing["sha256"] is None and missing["size_bytes"] is None
    assert missing["metadata"]["reason"] == "calibration_reference_missing"
    assert blocked["metadata"]["reason"] == "outside_allowed_calibration_sources"
    assert not any(path.read_bytes() == outside.read_bytes() for path in session.rglob("*.txt"))

    bundle = _artifact(artifacts, "calibration_bundle_index")
    bundle_bytes = (session / str(bundle["relative_path"])).read_bytes()
    bundle_payload = json.loads(bundle_bytes)
    assert bundle_payload["complete"] is False
    assert str(tmp_path).encode() not in bundle_bytes
    assert b"outside-must-not-be-copied" not in bundle_bytes

    second_session = tmp_path / "second-session"
    second = collect_calibration_bundle(
        second_session,
        {"calibration_intrinsics_profile": profile},
    )
    second_bundle = _artifact(second, "calibration_bundle_index")
    assert bundle_bytes == (second_session / str(second_bundle["relative_path"])).read_bytes()
    assert bundle["sha256"] == second_bundle["sha256"]


def test_calibration_outside_file_is_copied_only_when_explicitly_supplied(tmp_path: Path) -> None:
    calibration = tmp_path / "calibration"
    calibration.mkdir()
    outside = tmp_path / "outside" / "support.txt"
    outside.parent.mkdir()
    outside.write_bytes(b"explicit-support")
    profile = calibration / "profile.json"
    profile.write_text(json.dumps({"support_file": str(outside)}), encoding="utf-8")

    implicit_session = tmp_path / "implicit-session"
    implicit = collect_calibration_bundle(
        implicit_session,
        {"calibration_intrinsics_profile": profile},
    )
    assert _artifact(implicit, "calibration_referenced_input", "failed")
    assert not any(path.read_bytes() == outside.read_bytes() for path in implicit_session.rglob("*.txt"))

    explicit_session = tmp_path / "explicit-session"
    explicit = collect_calibration_bundle(
        explicit_session,
        {"calibration_intrinsics_profile": profile},
        explicit_source_paths={"calibration_support": outside},
    )
    referenced = _artifact(explicit, "calibration_referenced_input")
    assert (explicit_session / str(referenced["relative_path"])).read_bytes() == outside.read_bytes()
    assert all(
        item["status"] != "failed" or item["role"] != "calibration_referenced_input"
        for item in explicit
    )


def test_same_calibration_bytes_used_by_two_roles_are_stored_once(tmp_path: Path) -> None:
    profile = tmp_path / "shared-profile.json"
    profile.write_text('{"K":[1,0,1]}', encoding="utf-8")
    session = tmp_path / "session"

    artifacts = collect_calibration_bundle(
        session,
        {
            "calibration_intrinsics_profile": profile,
            "calibration_extrinsics_profile": profile,
        },
    )

    intrinsics = _artifact(artifacts, "calibration_intrinsics_profile")
    extrinsics = _artifact(artifacts, "calibration_extrinsics_profile")
    assert intrinsics["relative_path"] == extrinsics["relative_path"]
    stored = list((session / "inputs" / "calibration" / "files").glob("*.json"))
    assert len(stored) == 1
