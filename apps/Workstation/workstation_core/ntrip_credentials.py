"""OS-secured storage for NTRIP passwords.

This module owns exactly one thing: putting a password into (and getting it
back out of) the OS credential store via ``keyring``. It never sees the rest
of the NTRIP configuration and never writes a password anywhere else -- there
is deliberately no plaintext-file fallback. Non-secret fields (host, port,
mountpoint, username) live in ``NetworkConfig`` and the settings JSON as
usual; only the password touches this module.

Callers key each stored credential by (caster_host, mountpoint, username) so
switching casters or accounts on the same machine cannot cross-read a
different account's password.
"""

from __future__ import annotations

from enum import Enum

try:  # keyring is an optional-at-import-time runtime dependency.
    import keyring
    from keyring.errors import KeyringError, PasswordDeleteError
except Exception:  # pragma: no cover - exercised only when keyring is absent
    keyring = None
    KeyringError = Exception
    PasswordDeleteError = Exception


_SERVICE_PREFIX = "Workstation NTRIP"


class CredentialStorageStatus(str, Enum):
    SAVED = "SAVED"
    NOT_SAVED = "NOT_SAVED"
    UNAVAILABLE = "UNAVAILABLE"


def _service_name(caster_host: str, mountpoint: str) -> str:
    host = str(caster_host).strip()
    mount = str(mountpoint).strip().lstrip("/")
    return f"{_SERVICE_PREFIX}:{host}:{mount}"


def secure_storage_available() -> bool:
    """Return whether the OS credential store can actually be used here.

    ``keyring`` falls back to ``keyring.backends.fail.Keyring`` when no real
    backend (Secret Service, macOS Keychain, Windows Credential Locker, ...)
    is usable -- e.g. a headless session with no secret-service daemon. That
    fallback answers every call with an error, so it is treated as
    unavailable rather than attempted.
    """

    if keyring is None:
        return False
    try:
        backend = keyring.get_keyring()
    except Exception:
        return False
    return backend.__class__.__module__ != "keyring.backends.fail"


def save_password(caster_host: str, mountpoint: str, username: str, password: str) -> bool:
    """Store ``password`` in the OS credential store. Never raises."""

    user = str(username).strip()
    if keyring is None or not user:
        return False
    try:
        keyring.set_password(_service_name(caster_host, mountpoint), user, str(password))
    except Exception:
        return False
    return True


def load_password(caster_host: str, mountpoint: str, username: str) -> str | None:
    """Return the stored password, or ``None`` if there isn't one. Never raises."""

    user = str(username).strip()
    if keyring is None or not user:
        return None
    try:
        return keyring.get_password(_service_name(caster_host, mountpoint), user)
    except Exception:
        return None


def delete_password(caster_host: str, mountpoint: str, username: str) -> bool:
    """Remove any stored password. Returns True if none remains afterward.

    Deleting an entry that was never saved is treated as success (the goal
    state -- "nothing stored" -- already holds) rather than an error.
    """

    user = str(username).strip()
    if keyring is None or not user:
        return False
    try:
        keyring.delete_password(_service_name(caster_host, mountpoint), user)
    except PasswordDeleteError:
        return True
    except Exception:
        return False
    return True


def credential_status(caster_host: str, mountpoint: str, username: str) -> CredentialStorageStatus:
    """Report whether a password is currently saved for this caster/username."""

    if not secure_storage_available():
        return CredentialStorageStatus.UNAVAILABLE
    if not str(username).strip():
        return CredentialStorageStatus.NOT_SAVED
    stored = load_password(caster_host, mountpoint, username)
    return CredentialStorageStatus.SAVED if stored else CredentialStorageStatus.NOT_SAVED
