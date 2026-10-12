"""Account-scoped NTRIP credential selection from explicit UI input values.

The secure storage adapter remains the only persistence effect. Error messages
and repr omit password values; the accepted credential itself is unchanged.
"""

from __future__ import annotations
import os
from dataclasses import dataclass, field
from Workstation.workstation_core import ntrip_credentials


class NtripCredentialError(RuntimeError):
    """Raised by ``_resolve_ntrip_credentials`` with a precise, user-facing reason.

    The message is safe to show as-is: it names which input is missing, never
    a credential value.
    """


@dataclass(frozen=True)
class NtripCredentialResolution:
    username: str
    password: str = field(repr=False)
    # "session" (typed into the UI this run), "keyring" (OS secure store), or
    # "legacy_env" (username_env/password_env indirection).
    source: str


def _ntrip_credential_account_key(
    caster_host: object,
    mountpoint: object,
    username: object,
) -> tuple[str, str, str]:
    """Return the non-secret identity that owns one session credential."""

    return (
        str(caster_host or "").strip().casefold(),
        str(mountpoint or "").strip().lstrip("/"),
        str(username or "").strip(),
    )


@dataclass(frozen=True)
class CredentialInputs:
    password: str = field(repr=False)
    explicit: bool = True
    account_key: tuple[str, str, str] | None = None

    def __post_init__(self):
        if self.account_key is not None:
            object.__setattr__(self, "account_key", tuple(self.account_key))


def resolve_ntrip_credential(network, inputs: CredentialInputs) -> NtripCredentialResolution:
    """Preserve explicit-session, keyring, environment, then deployment precedence."""
    username = str(network.username).strip()
    session_password = inputs.password
    session_password_explicit = inputs.explicit
    current_key = _ntrip_credential_account_key(
        network.caster_host,
        network.mountpoint,
        username,
    )
    session_key = inputs.account_key
    # Lightweight legacy/test panels have no account marker. Real UI
    # handoffs always supply one; preserving the no-marker behavior keeps
    # old facade clients source compatible without allowing an actual
    # edited password to bleed into a different caster/account.
    session_key_matches = session_key is None or tuple(session_key) == current_key
    if username and session_password and session_password_explicit and session_key_matches:
        return NtripCredentialResolution(username=username, password=session_password, source="session")

    if not username:
        # A legacy env-only configuration never set a username in the
        # new field; fall through to the legacy tier below instead of
        # failing immediately.
        legacy_username = os.getenv(network.username_env, "") if network.username_env else ""
        legacy_password = os.getenv(network.password_env, "") if network.password_env else ""
        if legacy_username and legacy_password:
            return NtripCredentialResolution(username=legacy_username, password=legacy_password, source="legacy_env")
        raise NtripCredentialError("NTRIP username이 필요합니다")

    storage_available = ntrip_credentials.secure_storage_available()
    if storage_available:
        stored = ntrip_credentials.load_password(network.caster_host, network.mountpoint, username)
        if stored:
            return NtripCredentialResolution(username=username, password=stored, source="keyring")

    legacy_username = os.getenv(network.username_env, "") if network.username_env else ""
    legacy_password = os.getenv(network.password_env, "") if network.password_env else ""
    if legacy_username and legacy_password:
        return NtripCredentialResolution(username=legacy_username, password=legacy_password, source="legacy_env")

    # No saved or environment credential exists.  Preserve the
    # deployment fallback for an untouched field profile, but only after
    # the stronger account-specific sources above have been exhausted.
    if username and session_password and session_key_matches:
        return NtripCredentialResolution(username=username, password=session_password, source="session")

    if not storage_available:
        raise NtripCredentialError("보안 자격증명 저장소를 사용할 수 없습니다 -- password를 입력하세요")
    raise NtripCredentialError("저장된 NTRIP password가 없습니다 -- password를 입력하거나 저장된 자격증명을 확인하세요")


def maybe_persist_ntrip_credential(network, credential: "NtripCredentialResolution") -> None:
    """Save a freshly-typed session password, but only when asked to.

    A password resolved from the keyring or the legacy env-var tier is
    already stored (or intentionally not persisted anywhere); only a
    brand-new "session" password is ever a candidate for saving, and
    only when ``remember_password`` is set.
    """

    if credential.source != "session" or not network.remember_password:
        return
    ntrip_credentials.save_password(network.caster_host, network.mountpoint, credential.username, credential.password)


# Preserve supported legacy exception/value identity and pickle lookup.
NtripCredentialError.__module__ = "Workstation.gnss_runtime_controller"
NtripCredentialResolution.__module__ = "Workstation.gnss_runtime_controller"
