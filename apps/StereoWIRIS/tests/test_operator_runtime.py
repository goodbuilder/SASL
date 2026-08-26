from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
from sasl_common.operator_runtime import (
    DECODER_AUTO,
    DECODER_NVIDIA,
    DECODER_SOFTWARE,
    NVIDIA_H264_DECODER,
    REQUIRED_GSTREAMER_ELEMENTS,
    SOFTWARE_H264_DECODER,
    DecoderSelection,
    check_official_interpreter,
    check_runtime_executables,
    probe_gstreamer,
    render_report,
    select_decoder,
)


class _FakeGi:
    def __init__(self) -> None:
        self.versions: list[tuple[str, str]] = []

    def require_version(self, namespace: str, version: str) -> None:
        self.versions.append((namespace, version))


def _gst_imports(*, missing: str = "", make_failure: str = "") -> dict[str, object]:
    def find(name: str) -> object | None:
        return None if name == missing else object()

    def make(name: str, _instance_name: str | None) -> object | None:
        return None if name == make_failure else object()

    gst = SimpleNamespace(
        init=lambda _args: None,
        ElementFactory=SimpleNamespace(find=find, make=make),
    )
    return {
        "gi": _FakeGi(),
        "gi.repository.Gst": gst,
        "gi.repository.GstRtp": object(),
    }


def _fake_official_venv(root: Path, *, system_site_packages: bool = True) -> tuple[Path, Path]:
    venv = root / ".venv"
    python = venv / "bin" / "python"
    python.parent.mkdir(parents=True)
    python.write_text("#!/bin/sh\n", encoding="utf-8")
    python.chmod(0o755)
    (venv / "pyvenv.cfg").write_text(
        f"include-system-site-packages = {'true' if system_site_packages else 'false'}\n",
        encoding="utf-8",
    )
    return venv, python


def test_wrong_sys_prefix_is_usage_error(tmp_path: Path) -> None:
    _venv, python = _fake_official_venv(tmp_path)

    report = check_official_interpreter(
        root=tmp_path,
        prefix=tmp_path / "another-venv",
        executable=python,
    )

    assert report.exit_code == 2
    assert report.diagnostics[0].code == "python.prefix"
    assert "wrong interpreter" in report.diagnostics[0].message


def test_manual_dist_packages_bridge_is_rejected(tmp_path: Path) -> None:
    venv, python = _fake_official_venv(tmp_path, system_site_packages=False)
    pth = venv / "lib" / "python3.10" / "site-packages" / "workstation-system-dist-packages.pth"
    pth.parent.mkdir(parents=True)
    pth.write_text("/usr/lib/python3/dist-packages\n", encoding="utf-8")

    report = check_official_interpreter(root=tmp_path, prefix=venv, executable=python)

    assert report.exit_code == 1
    assert {item.code for item in report.diagnostics if item.level == "FAIL"} == {
        "python.system_site_packages",
        "python.manual_pth",
    }


def test_gi_missing_blocks_the_runtime() -> None:
    report = probe_gstreamer(DECODER_AUTO, importer={}, nvidia_probe=lambda: (False, "no driver"))

    assert report.exit_code == 1
    assert report.missing_elements == REQUIRED_GSTREAMER_ELEMENTS
    assert "python3-gi" in report.diagnostics[0].ubuntu_packages


def test_required_native_executables_have_package_recovery() -> None:
    diagnostics = check_runtime_executables(which=lambda _name: None)

    assert {item.code for item in diagnostics} == {
        "runtime.executable.gst-inspect-1.0",
        "runtime.executable.ffmpeg",
    }
    assert all(item.level == "FAIL" for item in diagnostics)
    assert {item.ubuntu_packages[0] for item in diagnostics} == {"gstreamer1.0-tools", "ffmpeg"}


@pytest.mark.parametrize("missing_element", REQUIRED_GSTREAMER_ELEMENTS)
def test_each_required_element_missing_is_reported(missing_element: str) -> None:
    report = probe_gstreamer(
        DECODER_AUTO,
        importer=_gst_imports(missing=missing_element),
        nvidia_probe=lambda: (False, "no driver"),
    )

    assert report.exit_code == 1
    assert report.missing_elements == (missing_element,)
    assert any(
        item.code == f"gstreamer.element.{missing_element}" and item.level == "FAIL"
        for item in report.diagnostics
    )


def test_factory_creation_failure_is_not_treated_as_available() -> None:
    report = probe_gstreamer(
        DECODER_SOFTWARE,
        importer=_gst_imports(make_failure="h264parse"),
        nvidia_probe=lambda: (True, "fake driver"),
    )

    assert report.exit_code == 1
    assert report.missing_elements == ("h264parse",)
    assert "make returned no element" in render_report(report, title="test")


@pytest.mark.parametrize(
    ("requested", "software", "nvidia_element", "driver", "allowed", "effective", "candidates"),
    (
        (DECODER_AUTO, True, True, True, True, NVIDIA_H264_DECODER, (NVIDIA_H264_DECODER, SOFTWARE_H264_DECODER)),
        (DECODER_AUTO, True, True, False, True, SOFTWARE_H264_DECODER, (SOFTWARE_H264_DECODER,)),
        (DECODER_NVIDIA, True, True, True, True, NVIDIA_H264_DECODER, (NVIDIA_H264_DECODER,)),
        (DECODER_NVIDIA, True, True, False, False, None, ()),
        (DECODER_SOFTWARE, True, False, False, True, SOFTWARE_H264_DECODER, (SOFTWARE_H264_DECODER,)),
        (DECODER_AUTO, False, True, True, False, None, ()),
    ),
)
def test_decoder_policy(
    requested: str,
    software: bool,
    nvidia_element: bool,
    driver: bool,
    allowed: bool,
    effective: str | None,
    candidates: tuple[str, ...],
) -> None:
    selection = select_decoder(
        requested,
        software_available=software,
        nvidia_element_available=nvidia_element,
        nvidia_driver_available=driver,
    )

    assert selection.allowed is allowed
    assert selection.effective == effective
    assert selection.candidates == candidates


def test_auto_driver_warning_is_allowed_but_explicit_nvidia_fails() -> None:
    imports = _gst_imports()
    auto = probe_gstreamer(
        DECODER_AUTO,
        importer=imports,
        nvidia_probe=lambda: (False, "driver offline"),
    )
    nvidia = probe_gstreamer(
        DECODER_NVIDIA,
        importer=imports,
        nvidia_probe=lambda: (False, "driver offline"),
    )

    assert auto.exit_code == 0
    assert auto.selection.effective == SOFTWARE_H264_DECODER
    assert auto.selection.degraded is True
    assert nvidia.exit_code == 1
    assert nvidia.selection.allowed is False


def test_dependency_manifests_keep_native_and_optional_backends_separate() -> None:
    root = Path(__file__).resolve().parents[3]
    runtime = (root / "requirements" / "runtime.txt").read_text(encoding="utf-8")
    system = (root / "requirements" / "ubuntu-22.04-operator-runtime.txt").read_text(
        encoding="utf-8"
    )
    optional_pyav = (root / "requirements" / "optional-pyav.txt").read_text(encoding="utf-8")
    stereo_compat = (root / "apps" / "StereoWIRIS" / "requirements" / "runtime.txt").read_text(
        encoding="utf-8"
    )
    workstation_compat = (root / "apps" / "Workstation" / "requirements-pyav.txt").read_text(
        encoding="utf-8"
    )

    assert "PyGObject" not in runtime
    assert "\nav" not in runtime
    assert "python3-gi" in system
    assert "gstreamer1.0-libav" in system
    assert "av>=11,<16" in optional_pyav
    assert "../../../requirements/runtime.txt" in stereo_compat
    assert "../../requirements/optional-pyav.txt" in workstation_compat


@pytest.mark.parametrize(
    ("module_name", "owner_name"),
    (
        ("Workstation.camera_stream_controller", "CameraStreamLifecycleController"),
        ("wiris_stereo_distance.legacy_gui", "WirisStereoDistancePanel"),
    ),
)
def test_explicit_nvidia_camera_start_is_rejected_with_recovery(
    monkeypatch: pytest.MonkeyPatch,
    module_name: str,
    owner_name: str,
) -> None:
    module = __import__(module_name, fromlist=[owner_name])
    rejected = DecoderSelection(
        DECODER_NVIDIA,
        None,
        (),
        False,
        reason="explicit NVIDIA mode requires a working NVIDIA driver",
    )
    monkeypatch.setattr(module, "current_decoder_selection", lambda _requested: rejected)
    logs: list[str] = []
    labels: list[str] = []
    panel = SimpleNamespace(
        stream_backend_combo=SimpleNamespace(currentData=lambda: "gstreamer"),
        decoder_combo=SimpleNamespace(currentData=lambda: DECODER_NVIDIA),
        _runtime_decoder_override=None,
        _log=logs.append,
        camera_connection_label=SimpleNamespace(setText=labels.append),
    )
    owner = getattr(module, owner_name)
    if owner_name == "CameraStreamLifecycleController":
        allowed = owner(panel)._apply_decoder_start_policy()
    else:
        allowed = owner._apply_decoder_start_policy(panel)

    assert allowed is False
    assert logs and "Auto 또는 Software" in logs[-1]
    assert labels and "tools/check_operator_runtime.sh --execution" in labels[-1]


@pytest.mark.parametrize(
    ("module_name", "owner_name"),
    (
        ("Workstation.camera_stream_controller", "CameraStreamLifecycleController"),
        ("wiris_stereo_distance.legacy_gui", "WirisStereoDistancePanel"),
    ),
)
def test_auto_software_capability_sets_effective_decoder_without_rejecting_camera(
    monkeypatch: pytest.MonkeyPatch,
    module_name: str,
    owner_name: str,
) -> None:
    module = __import__(module_name, fromlist=[owner_name])
    selected = DecoderSelection(
        DECODER_AUTO,
        SOFTWARE_H264_DECODER,
        (SOFTWARE_H264_DECODER,),
        True,
        degraded=True,
        reason="NVIDIA driver unavailable; Auto selected avdec_h264",
    )
    monkeypatch.setattr(module, "current_decoder_selection", lambda _requested: selected)
    logs: list[str] = []
    panel = SimpleNamespace(
        stream_backend_combo=SimpleNamespace(currentData=lambda: "gstreamer"),
        decoder_combo=SimpleNamespace(currentData=lambda: DECODER_AUTO),
        _runtime_decoder_override=None,
        _log=logs.append,
    )
    owner = getattr(module, owner_name)
    if owner_name == "CameraStreamLifecycleController":
        allowed = owner(panel)._apply_decoder_start_policy()
    else:
        allowed = owner._apply_decoder_start_policy(panel)

    assert allowed is True
    assert panel._runtime_decoder_override == DECODER_SOFTWARE
    assert logs and "avdec_h264" in logs[-1]
