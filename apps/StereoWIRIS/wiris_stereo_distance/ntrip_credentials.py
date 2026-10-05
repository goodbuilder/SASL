"""OS-secured NTRIP password storage for the Legacy application."""

from __future__ import annotations

from enum import Enum

try:
    import keyring
    from keyring.errors import PasswordDeleteError
except Exception:  # pragma: no cover - optional backend
    keyring = None
    PasswordDeleteError = Exception


_SERVICE_PREFIX = "StereoWIRIS NTRIP"


class CredentialStorageStatus(str, Enum):
    SAVED = "SAVED"
    NOT_SAVED = "NOT_SAVED"
    UNAVAILABLE = "UNAVAILABLE"


def _service_name(caster_host: str, mountpoint: str) -> str:
    return f"{_SERVICE_PREFIX}:{str(caster_host).strip()}:{str(mountpoint).strip().lstrip('/')}"


def secure_storage_available() -> bool:
    if keyring is None:
        return False
    try:
        backend = keyring.get_keyring()
    except Exception:
        return False
    return backend.__class__.__module__ != "keyring.backends.fail"


def save_password(caster_host: str, mountpoint: str, username: str, password: str) -> bool:
    user = str(username).strip()
    if not secure_storage_available() or not user:
        return False
    try:
        keyring.set_password(_service_name(caster_host, mountpoint), user, str(password))
    except Exception:
        return False
    return True


def load_password(caster_host: str, mountpoint: str, username: str) -> str | None:
    user = str(username).strip()
    if not secure_storage_available() or not user:
        return None
    try:
        return keyring.get_password(_service_name(caster_host, mountpoint), user)
    except Exception:
        return None


def delete_password(caster_host: str, mountpoint: str, username: str) -> bool:
    user = str(username).strip()
    if not secure_storage_available() or not user:
        return False
    try:
        keyring.delete_password(_service_name(caster_host, mountpoint), user)
    except PasswordDeleteError:
        return True
    except Exception:
        return False
    return True


def credential_status(caster_host: str, mountpoint: str, username: str) -> CredentialStorageStatus:
    if not secure_storage_available():
        return CredentialStorageStatus.UNAVAILABLE
    return (
        CredentialStorageStatus.SAVED
        if load_password(caster_host, mountpoint, username)
        else CredentialStorageStatus.NOT_SAVED
    )
