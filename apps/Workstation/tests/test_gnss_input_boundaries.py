"""Explicit account inputs and WGS84/MSL GGA contracts without live panel reads."""

from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys

from Workstation.gnss_runtime import gga
from Workstation.gnss_runtime.credentials import CredentialInputs, resolve_ntrip_credential
from Workstation.tests.test_panel_composition import panels
from Workstation.workstation_core import gnss_network, ntrip_credentials
from Workstation.workstation_core.gnss_operation import GgaPolicy, GnssOperationConfig, NetworkConfig
from Workstation.workstation_core.gnss_profile import GnssCoordinateSample
from Workstation.workstation_core.rtk_geometry import GpsPositionState


def test_gga_state_is_owned_once_in_actual_panel(panels):
    panel = panels()
    state = panel.application.features["gnss"].state
    owner = panel._gnss_runtime_controller_owner._gga_owner()
    assert owner is state.gga
    for name in gga.GGA_STATE_FIELDS:
        assert name not in vars(state)
        assert name not in vars(panel)
    owner.log_skip("missing", "no sample", lambda _: None)
    assert panel._gnss_last_gga_skip_reason == "no sample"
    panel._gnss_last_gga_skip_reason = "reset"
    assert owner._gnss_last_gga_skip_reason == "reset"


def test_credential_input_captures_account_and_omits_password_from_repr(monkeypatch):
    key = ["caster", "MOUNT", "field-user"]
    inputs = CredentialInputs("fixture-password-not-for-logs", account_key=key)
    key[0] = "different-caster"
    monkeypatch.setattr(
        ntrip_credentials,
        "secure_storage_available",
        lambda: (_ for _ in ()).throw(AssertionError("unneeded keyring lookup")),
    )
    result = resolve_ntrip_credential(
        NetworkConfig(caster_host="CASTER", mountpoint="/MOUNT", username="field-user"), inputs
    )
    assert result.source == "session"
    assert result.password == "fixture-password-not-for-logs"
    assert result.password not in repr(result)
    assert result.password not in repr(inputs)
    assert inputs.account_key == ("caster", "MOUNT", "field-user")


def test_gga_preserves_hemispheres_msl_geoid_and_utc_independently(monkeypatch):
    monkeypatch.setattr(gga, "time", SimpleNamespace(monotonic=lambda: 100.0))
    wall_utc = datetime(2026, 1, 2, 3, 4, 5, tzinfo=timezone.utc)
    monkeypatch.setattr(gnss_network, "datetime", SimpleNamespace(now=lambda zone: wall_utc))
    position = GpsPositionState(
        "left",
        "left",
        99.0,
        "gps",
        latitude_deg=-35.25,
        longitude_deg=-123.5,
        altitude_m=42.0,
        altitude_ellipsoid_m=69.5,
        fix_type=6,
        satellites_used=17,
        eph_m=0.7,
    )
    config = GnssOperationConfig(network=NetworkConfig(gga_policy=GgaPolicy.LEFT_GNSS))
    owner = gga.GgaSentenceOwner()
    sentence = owner.resolve(config, left=position, log=lambda _: None)
    fields = sentence.split("*")[0].split(",")
    assert fields[1:13] == [
        "030405.00",
        "3515.0000",
        "S",
        "12330.0000",
        "W",
        "4",
        "17",
        "0.7",
        "42.000",
        "M",
        "27.500",
        "M",
    ]
    body = sentence[1 : sentence.index("*")]
    checksum = 0
    for value in body.encode("ascii"):
        checksum ^= value
    assert sentence.endswith(f"*{checksum:02X}")
    # Center hint intentionally retains quality=1 and unknown geoid, not zero.
    center = owner.resolve(
        replace(config, network=replace(config.network, gga_policy=GgaPolicy.SYSTEM_CENTER)),
        left=position,
        right=position,
        log=lambda _: None,
    )
    assert center.split(",")[6] == "1"
    assert center.split(",")[11] == ""
    base = GnssCoordinateSample(-35.25, -123.5, 69.5, timestamp_s=99.0)
    assert (
        owner.resolve(
            replace(config, network=replace(config.network, gga_policy=GgaPolicy.BASE_GNSS)),
            base_sample=base,
            log=lambda _: None,
        )
        is None
    )


def test_skip_throttle_updates_reason_time_and_success_clear_keeps_throttle(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(gga, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    owner, logs = gga.GgaSentenceOwner(), []
    owner.log_skip("missing", "first", logs.append)
    clock[0] = 110.0
    owner.log_skip("missing", "second", logs.append)
    assert logs == ["first"]
    assert owner._gnss_last_gga_skip_reason == "second"
    assert owner._gnss_last_gga_skip_monotonic_s == 110.0
    owner.clear_skip()
    assert owner._gnss_last_gga_skip_reason == ""
    assert owner._gnss_last_gga_skip_monotonic_s is None
    clock[0] = 130.0
    owner.log_skip("missing", "third", logs.append)
    assert logs == ["first", "third"]


def test_gnss_input_leaves_import_without_qt_or_panel():
    script = """
import importlib.abc, sys
sys.path.insert(0, sys.argv[1])
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith(('PyQt5', 'Workstation.panel_', 'Workstation.gnss_runtime_controller')):
            raise AssertionError(fullname)
sys.meta_path.insert(0, Guard())
from Workstation.gnss_runtime.credentials import CredentialInputs
from Workstation.gnss_runtime.gga import GgaSentenceOwner
assert CredentialInputs('test').password == 'test'
assert GgaSentenceOwner()._gnss_last_gga_skip_reason == ''
"""
    subprocess.run(
        [sys.executable, "-I", "-B", "-c", script, str(Path(__file__).resolve().parents[2])], check=True, timeout=20
    )
