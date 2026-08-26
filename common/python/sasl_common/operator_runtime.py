"""Read-only runtime checks for the SASL operator applications.

The supported deployment uses one repository-root virtual environment created
with ``--system-site-packages``. PyGObject and GStreamer deliberately remain
Ubuntu packages; nothing in this module installs or mutates dependencies.
"""

from __future__ import annotations

import argparse
import importlib
import os
import re
import site
import shutil
import subprocess
import sys
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from importlib import metadata
from pathlib import Path

EXIT_READY = 0
EXIT_DEPENDENCY = 1
EXIT_USAGE = 2

DECODER_AUTO = "auto"
DECODER_NVIDIA = "nvidia"
DECODER_SOFTWARE = "software"
NVIDIA_H264_DECODER = "nvh264dec"
SOFTWARE_H264_DECODER = "avdec_h264"

REQUIRED_GSTREAMER_ELEMENTS = (
    "rtspsrc",
    "rtph264depay",
    "h264parse",
    "queue",
    "videoconvert",
    "appsink",
    SOFTWARE_H264_DECODER,
    "mp4mux",
    "filesink",
)

ELEMENT_UBUNTU_PACKAGES: Mapping[str, str] = {
    "rtspsrc": "gstreamer1.0-plugins-good",
    "rtph264depay": "gstreamer1.0-plugins-good",
    "h264parse": "gstreamer1.0-plugins-bad",
    "queue": "gstreamer1.0-plugins-base",
    "videoconvert": "gstreamer1.0-plugins-base",
    "appsink": "gstreamer1.0-plugins-base",
    SOFTWARE_H264_DECODER: "gstreamer1.0-libav",
    "mp4mux": "gstreamer1.0-plugins-good",
    "filesink": "gstreamer1.0-plugins-base",
    NVIDIA_H264_DECODER: "gstreamer1.0-plugins-bad",
}


@dataclass(frozen=True)
class Diagnostic:
    level: str
    code: str
    message: str
    ubuntu_packages: tuple[str, ...] = ()
    recovery: tuple[str, ...] = ()


@dataclass(frozen=True)
class DecoderSelection:
    requested: str
    effective: str | None
    candidates: tuple[str, ...]
    allowed: bool
    degraded: bool = False
    reason: str = ""


@dataclass(frozen=True)
class GStreamerReport:
    diagnostics: tuple[Diagnostic, ...]
    selection: DecoderSelection
    missing_elements: tuple[str, ...] = ()
    software_available: bool = False
    nvidia_element_available: bool = False
    nvidia_driver_available: bool = False

    @property
    def exit_code(self) -> int:
        return EXIT_DEPENDENCY if any(item.level == "FAIL" for item in self.diagnostics) else EXIT_READY


@dataclass(frozen=True)
class RuntimeReport:
    diagnostics: tuple[Diagnostic, ...]
    selection: DecoderSelection | None = None
    missing_elements: tuple[str, ...] = ()
    forced_exit_code: int | None = None

    @property
    def exit_code(self) -> int:
        if self.forced_exit_code is not None:
            return int(self.forced_exit_code)
        return EXIT_DEPENDENCY if any(item.level == "FAIL" for item in self.diagnostics) else EXIT_READY


_LAST_GSTREAMER_CAPABILITIES: tuple[bool, bool, bool] | None = None


def repository_root() -> Path:
    return Path(__file__).resolve().parents[3]


def canonical_venv(root: Path | None = None) -> Path:
    return (root or repository_root()) / ".venv"


def canonical_python(root: Path | None = None) -> Path:
    return canonical_venv(root) / "bin" / "python"


def venv_recovery_commands(root: Path) -> tuple[str, ...]:
    return (
        f"cd {root}",
        "mv .venv .venv.backup-$(date +%Y%m%d-%H%M%S)",
        "python3 -m venv --system-site-packages .venv",
        ".venv/bin/python -m pip install -r requirements.txt",
    )


def _read_pyvenv_config(path: Path) -> Mapping[str, str]:
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return values
    for line in lines:
        key, separator, value = line.partition("=")
        if separator:
            values[key.strip().lower()] = value.strip()
    return values


def check_official_interpreter(
    *,
    root: Path | None = None,
    prefix: str | os.PathLike[str] | None = None,
    executable: str | os.PathLike[str] | None = None,
) -> RuntimeReport:
    """Validate the canonical prefix without resolving its Python symlink."""

    selected_root = (root or repository_root()).resolve()
    expected_prefix = canonical_venv(selected_root).resolve()
    actual_prefix = Path(sys.prefix if prefix is None else prefix).expanduser().resolve()
    actual_executable = Path(sys.executable if executable is None else executable).expanduser().absolute()
    expected_executable = canonical_python(selected_root).absolute()
    recovery = venv_recovery_commands(selected_root)

    if actual_prefix != expected_prefix:
        return RuntimeReport(
            diagnostics=(
                Diagnostic(
                    "FAIL",
                    "python.prefix",
                    f"wrong interpreter: sys.prefix={actual_prefix}; required={expected_prefix}; "
                    f"python={actual_executable}",
                    recovery=("unset PYTHON_BIN", f"{expected_executable} <entrypoint>"),
                ),
            ),
            forced_exit_code=EXIT_USAGE,
        )

    diagnostics: list[Diagnostic] = []
    if sys.version_info[:2] != (3, 10):
        return RuntimeReport(
            diagnostics=(
                Diagnostic(
                    "FAIL",
                    "python.version",
                    f"wrong Python version: {sys.version_info.major}.{sys.version_info.minor}; required=3.10",
                    ubuntu_packages=("python3.10-venv",),
                    recovery=recovery,
                ),
            ),
            forced_exit_code=EXIT_USAGE,
        )
    if not expected_executable.is_file() or not os.access(expected_executable, os.X_OK):
        diagnostics.append(
            Diagnostic(
                "FAIL",
                "python.executable",
                f"canonical Python is missing or not executable: {expected_executable}",
                recovery=recovery,
            )
        )
    elif actual_executable != expected_executable:
        return RuntimeReport(
            diagnostics=(
                Diagnostic(
                    "FAIL",
                    "python.executable",
                    f"wrong executable path: {actual_executable}; required={expected_executable}",
                    recovery=("unset PYTHON_BIN", f"{expected_executable} <entrypoint>"),
                ),
            ),
            forced_exit_code=EXIT_USAGE,
        )
    else:
        diagnostics.append(
            Diagnostic("PASS", "python.executable", f"canonical interpreter: {expected_executable}")
        )

    config = _read_pyvenv_config(expected_prefix / "pyvenv.cfg")
    if config.get("include-system-site-packages", "").lower() == "true":
        diagnostics.append(
            Diagnostic("PASS", "python.system_site_packages", "official venv exposes Ubuntu system packages")
        )
    else:
        diagnostics.append(
            Diagnostic(
                "FAIL",
                "python.system_site_packages",
                "official venv was not created with --system-site-packages",
                ubuntu_packages=("python3.10-venv",),
                recovery=recovery,
            )
        )

    site_packages = expected_prefix / "lib" / "python3.10" / "site-packages"
    unsupported_pth: list[Path] = []
    for pth_path in site_packages.glob("*.pth"):
        try:
            entries = pth_path.read_text(encoding="utf-8").splitlines()
        except OSError:
            continue
        if any(entry.strip() == "/usr/lib/python3/dist-packages" for entry in entries):
            unsupported_pth.append(pth_path)
    if unsupported_pth:
        diagnostics.append(
            Diagnostic(
                "FAIL",
                "python.manual_pth",
                "unsupported manual system-package bridge exists: "
                + ", ".join(str(path) for path in unsupported_pth),
                recovery=recovery,
            )
        )
    return RuntimeReport(tuple(diagnostics))


def normalize_decoder_mode(value: str | None) -> str:
    normalized = str(value or DECODER_AUTO).strip().lower()
    if normalized in {DECODER_AUTO, DECODER_NVIDIA, DECODER_SOFTWARE}:
        return normalized
    if normalized in {NVIDIA_H264_DECODER, "nvdec", "cuda"}:
        return DECODER_NVIDIA
    if normalized in {SOFTWARE_H264_DECODER, "ffmpeg", "cpu"}:
        return DECODER_SOFTWARE
    return DECODER_AUTO


def select_decoder(
    requested: str | None,
    *,
    software_available: bool,
    nvidia_element_available: bool,
    nvidia_driver_available: bool,
) -> DecoderSelection:
    mode = normalize_decoder_mode(requested)
    if not software_available:
        return DecoderSelection(
            mode,
            None,
            (),
            False,
            reason="required avdec_h264 software decoder is unavailable",
        )
    nvidia_available = nvidia_element_available and nvidia_driver_available
    if mode == DECODER_SOFTWARE:
        return DecoderSelection(mode, SOFTWARE_H264_DECODER, (SOFTWARE_H264_DECODER,), True)
    if mode == DECODER_NVIDIA:
        if nvidia_available:
            return DecoderSelection(mode, NVIDIA_H264_DECODER, (NVIDIA_H264_DECODER,), True)
        missing = []
        if not nvidia_element_available:
            missing.append("nvh264dec element")
        if not nvidia_driver_available:
            missing.append("NVIDIA driver")
        return DecoderSelection(
            mode,
            None,
            (),
            False,
            reason=f"explicit NVIDIA mode requires a working {' and '.join(missing)}",
        )
    if nvidia_available:
        return DecoderSelection(
            mode,
            NVIDIA_H264_DECODER,
            (NVIDIA_H264_DECODER, SOFTWARE_H264_DECODER),
            True,
        )
    missing = []
    if not nvidia_element_available:
        missing.append("nvh264dec element")
    if not nvidia_driver_available:
        missing.append("NVIDIA driver")
    return DecoderSelection(
        mode,
        SOFTWARE_H264_DECODER,
        (SOFTWARE_H264_DECODER,),
        True,
        degraded=True,
        reason=f"{', '.join(missing)} unavailable; Auto selected avdec_h264",
    )


def probe_nvidia_driver(
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> tuple[bool, str]:
    try:
        result = runner(
            ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)
    detail = (result.stdout or result.stderr or "nvidia-smi returned no detail").strip()
    return result.returncode == 0, detail


def _module_loader(
    importer: Callable[[str], object] | Mapping[str, object] | None,
) -> Callable[[str], object]:
    if importer is None:
        return importlib.import_module
    if callable(importer):
        return importer

    def load(name: str) -> object:
        if name not in importer:
            raise ModuleNotFoundError(name)
        value = importer[name]
        if isinstance(value, BaseException):
            raise value
        return value

    return load


def probe_gstreamer(
    requested_decoder: str | None = DECODER_AUTO,
    *,
    importer: Callable[[str], object] | Mapping[str, object] | None = None,
    nvidia_probe: Callable[[], tuple[bool, str]] | None = None,
) -> GStreamerReport:
    global _LAST_GSTREAMER_CAPABILITIES
    load = _module_loader(importer)
    diagnostics: list[Diagnostic] = []
    missing_elements: list[str] = []
    try:
        gi = load("gi")
        gi.require_version("Gst", "1.0")
        gi.require_version("GstRtp", "1.0")
        Gst = load("gi.repository.Gst")
        load("gi.repository.GstRtp")
        init = getattr(Gst, "init", None)
        if callable(init):
            init(None)
    except Exception as exc:  # noqa: BLE001 - report arbitrary GI/plugin loader failures.
        packages = (
            "python3-gi",
            "gir1.2-gstreamer-1.0",
            "gir1.2-gst-plugins-base-1.0",
        )
        diagnostics.append(
            Diagnostic(
                "FAIL",
                "gstreamer.introspection",
                f"Gst/GstRtp introspection unavailable: {exc}",
                ubuntu_packages=packages,
                recovery=(f"sudo apt-get install {' '.join(packages)}",),
            )
        )
        selection = select_decoder(
            requested_decoder,
            software_available=False,
            nvidia_element_available=False,
            nvidia_driver_available=False,
        )
        if importer is None and nvidia_probe is None:
            _LAST_GSTREAMER_CAPABILITIES = (False, False, False)
        return GStreamerReport(tuple(diagnostics), selection, REQUIRED_GSTREAMER_ELEMENTS)

    diagnostics.append(
        Diagnostic("PASS", "gstreamer.introspection", "gi, Gst 1.0, and GstRtp 1.0 are available")
    )

    factory = getattr(Gst, "ElementFactory", None)
    finder = getattr(factory, "find", None)
    maker = getattr(factory, "make", None)
    if not callable(finder) or not callable(maker):
        diagnostics.append(
            Diagnostic(
                "FAIL",
                "gstreamer.factory",
                "Gst.ElementFactory.find/make is unavailable",
                ubuntu_packages=("gir1.2-gstreamer-1.0",),
            )
        )
        selection = select_decoder(
            requested_decoder,
            software_available=False,
            nvidia_element_available=False,
            nvidia_driver_available=False,
        )
        if importer is None and nvidia_probe is None:
            _LAST_GSTREAMER_CAPABILITIES = (False, False, False)
        return GStreamerReport(tuple(diagnostics), selection, REQUIRED_GSTREAMER_ELEMENTS)

    usable: dict[str, bool] = {}
    for element in (*REQUIRED_GSTREAMER_ELEMENTS, NVIDIA_H264_DECODER):
        package = ELEMENT_UBUNTU_PACKAGES[element]
        optional = element == NVIDIA_H264_DECODER
        try:
            found = finder(element) is not None
        except Exception as exc:  # noqa: BLE001 - third-party factory boundary.
            found = False
            find_error = str(exc)
        else:
            find_error = ""
        created = False
        make_error = ""
        if found:
            try:
                created = maker(element, None) is not None
            except Exception as exc:  # noqa: BLE001 - third-party factory boundary.
                make_error = str(exc)
        usable[element] = found and created
        if usable[element]:
            diagnostics.append(
                Diagnostic("PASS", f"gstreamer.element.{element}", f"{element}: factory find/make ok")
            )
            continue
        detail = "factory not found" if not found else "factory make returned no element"
        error = find_error or make_error
        if error:
            detail = f"{detail}: {error}"
        if optional:
            diagnostics.append(
                Diagnostic(
                    "WARN",
                    f"gstreamer.element.{element}",
                    f"optional {element}: {detail}",
                    ubuntu_packages=(package,),
                )
            )
        else:
            missing_elements.append(element)
            diagnostics.append(
                Diagnostic(
                    "FAIL",
                    f"gstreamer.element.{element}",
                    f"required {element}: {detail}",
                    ubuntu_packages=(package,),
                    recovery=(f"sudo apt-get install {package}",),
                )
            )

    driver_ok, driver_detail = (nvidia_probe or probe_nvidia_driver)()
    diagnostics.append(
        Diagnostic(
            "PASS" if driver_ok else "WARN",
            "nvidia.driver",
            f"NVIDIA driver {'available' if driver_ok else 'unavailable'}: {driver_detail}",
            recovery=() if driver_ok else ("Verify the NVIDIA driver with: nvidia-smi",),
        )
    )
    selection = select_decoder(
        requested_decoder,
        software_available=usable.get(SOFTWARE_H264_DECODER, False),
        nvidia_element_available=usable.get(NVIDIA_H264_DECODER, False),
        nvidia_driver_available=driver_ok,
    )
    if not selection.allowed:
        diagnostics.append(
            Diagnostic(
                "FAIL",
                "decoder.selection",
                selection.reason,
                recovery=(
                    "Select Auto or Software decoder and retry.",
                    "Run: tools/check_operator_runtime.sh --execution",
                ),
            )
        )
    elif selection.degraded:
        diagnostics.append(Diagnostic("WARN", "decoder.selection", selection.reason))
    else:
        diagnostics.append(
            Diagnostic("PASS", "decoder.selection", f"{selection.requested} -> {selection.effective}")
        )
    software_available = usable.get(SOFTWARE_H264_DECODER, False)
    nvidia_element_available = usable.get(NVIDIA_H264_DECODER, False)
    if importer is None and nvidia_probe is None:
        _LAST_GSTREAMER_CAPABILITIES = (
            software_available,
            nvidia_element_available,
            driver_ok,
        )
    return GStreamerReport(
        tuple(diagnostics),
        selection,
        tuple(missing_elements),
        software_available,
        nvidia_element_available,
        driver_ok,
    )


def current_decoder_selection(requested_decoder: str | None) -> DecoderSelection:
    """Resolve a camera start from the capability snapshot taken before Qt."""

    if _LAST_GSTREAMER_CAPABILITIES is None:
        return probe_gstreamer(requested_decoder).selection
    software_available, nvidia_element_available, nvidia_driver_available = (
        _LAST_GSTREAMER_CAPABILITIES
    )
    return select_decoder(
        requested_decoder,
        software_available=software_available,
        nvidia_element_available=nvidia_element_available,
        nvidia_driver_available=nvidia_driver_available,
    )


def _requirement_names(path: Path) -> tuple[str, ...]:
    names: list[str] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return ()
    for raw_line in lines:
        line = raw_line.partition("#")[0].strip()
        if not line or line.startswith("-"):
            continue
        name = re.split(r"[<>=!~;\s\[]", line, maxsplit=1)[0].strip()
        if name:
            names.append(name)
    return tuple(dict.fromkeys(names))


def _runtime_requirements(path: Path) -> tuple[object, ...]:
    # Deployment diagnostics run with /usr/bin/python3 before the canonical
    # venv is guaranteed to exist.  Keep the third-party requirement parser
    # off that bootstrap import path; execution checks load it only after the
    # official interpreter gate has passed.
    from packaging.requirements import InvalidRequirement, Requirement

    requirements: list[object] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return ()
    for raw_line in lines:
        line = raw_line.partition("#")[0].strip()
        if not line or line.startswith("-"):
            continue
        try:
            requirement = Requirement(line)
        except InvalidRequirement:
            continue
        if requirement.marker is None or requirement.marker.evaluate():
            requirements.append(requirement)
    return tuple(requirements)


def check_python_distributions(
    *,
    root: Path | None = None,
    version_lookup: Callable[[str], str] = metadata.version,
    user_site_enabled: bool | None = None,
) -> tuple[Diagnostic, ...]:
    selected_root = (root or repository_root()).resolve()
    manifest = selected_root / "requirements" / "runtime.txt"
    try:
        requirements = _runtime_requirements(manifest)
    except ImportError as exc:
        return (
            Diagnostic(
                "FAIL",
                "python.requirement_parser",
                f"runtime requirement parser is unavailable: {exc}",
                recovery=(
                    f"{canonical_python(selected_root)} -m pip install -r {selected_root / 'requirements.txt'}",
                ),
            ),
        )
    if not requirements:
        return (Diagnostic("FAIL", "python.manifest", f"runtime manifest is missing or empty: {manifest}"),)
    if bool(site.ENABLE_USER_SITE if user_site_enabled is None else user_site_enabled):
        return (
            Diagnostic(
                "FAIL",
                "python.user_site",
                "Python user-site packages are enabled; the root runtime is not self-contained",
                recovery=(
                    "export PYTHONNOUSERSITE=1",
                    f"{canonical_python(selected_root)} -m pip install -r {selected_root / 'requirements.txt'}",
                ),
            ),
        )
    missing: list[str] = []
    incompatible: list[str] = []
    for requirement in requirements:
        name = requirement.name
        try:
            version = version_lookup(name)
        except Exception:  # noqa: BLE001 - injected/version metadata boundary.
            missing.append(name)
            continue
        if requirement.specifier and not requirement.specifier.contains(
            version,
            prereleases=True,
        ):
            incompatible.append(f"{name}=={version} not in {requirement.specifier}")
    if missing:
        return (
            Diagnostic(
                "FAIL",
                "python.distributions",
                f"missing pip distributions: {', '.join(missing)}",
                recovery=(
                    f"{canonical_python(selected_root)} -m pip install -r {selected_root / 'requirements.txt'}",
                ),
            ),
        )
    if incompatible:
        return (
            Diagnostic(
                "FAIL",
                "python.distributions.version",
                "incompatible runtime distributions: " + "; ".join(incompatible),
                recovery=(
                    f"{canonical_python(selected_root)} -m pip install -r {selected_root / 'requirements.txt'}",
                ),
            ),
        )
    return (
        Diagnostic(
            "PASS",
            "python.distributions",
            f"root runtime distributions installed ({len(requirements)}) with declared versions",
        ),
    )


def check_runtime_executables(
    which: Callable[[str], str | None] = shutil.which,
) -> tuple[Diagnostic, ...]:
    diagnostics: list[Diagnostic] = []
    for executable, package in (
        ("gst-inspect-1.0", "gstreamer1.0-tools"),
        ("ffmpeg", "ffmpeg"),
    ):
        path = which(executable)
        diagnostics.append(
            Diagnostic(
                "PASS" if path else "FAIL",
                f"runtime.executable.{executable}",
                f"{executable}: {path or 'not found'}",
                ubuntu_packages=() if path else (package,),
                recovery=() if path else (f"sudo apt-get install {package}",),
            )
        )
    return tuple(diagnostics)


def _read_package_manifest(path: Path) -> tuple[str, ...]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return ()
    return tuple(line.partition("#")[0].strip() for line in lines if line.partition("#")[0].strip())


def _dpkg_package_status(package: str) -> tuple[bool, str]:
    try:
        result = subprocess.run(
            ["dpkg-query", "-W", "-f=${Status}\t${Version}", package],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)
    detail = (result.stdout or result.stderr or "not installed").strip()
    return result.returncode == 0 and detail.startswith("install ok installed"), detail


def check_deployment(
    requested_decoder: str = DECODER_AUTO,
    *,
    root: Path | None = None,
    package_status: Callable[[str], tuple[bool, str]] = _dpkg_package_status,
    gstreamer_probe: Callable[[str], GStreamerReport] | None = None,
) -> RuntimeReport:
    selected_root = (root or repository_root()).resolve()
    manifest = selected_root / "requirements" / "ubuntu-22.04-operator-runtime.txt"
    packages = _read_package_manifest(manifest)
    diagnostics: list[Diagnostic] = []
    if not packages:
        diagnostics.append(Diagnostic("FAIL", "deployment.manifest", f"system manifest missing: {manifest}"))
    missing: list[str] = []
    for package in packages:
        installed, detail = package_status(package)
        diagnostics.append(
            Diagnostic(
                "PASS" if installed else "FAIL",
                f"ubuntu.package.{package}",
                f"{package}: {detail}",
                ubuntu_packages=() if installed else (package,),
            )
        )
        if not installed:
            missing.append(package)
    if missing:
        diagnostics.append(
            Diagnostic(
                "FAIL",
                "deployment.packages",
                f"missing Ubuntu packages: {', '.join(missing)}",
                ubuntu_packages=tuple(missing),
                recovery=("sudo apt-get update", f"sudo apt-get install {' '.join(missing)}"),
            )
        )

    venv_path = canonical_venv(selected_root)
    config = _read_pyvenv_config(venv_path / "pyvenv.cfg")
    if venv_path.exists() and config.get("include-system-site-packages", "").lower() != "true":
        diagnostics.append(
            Diagnostic(
                "WARN",
                "deployment.current_venv",
                "current .venv is not canonical and must be backed up/recreated before operation",
                recovery=venv_recovery_commands(selected_root),
            )
        )

    gst_report = (gstreamer_probe or probe_gstreamer)(requested_decoder)
    diagnostics.extend(gst_report.diagnostics)
    return RuntimeReport(
        tuple(diagnostics),
        selection=gst_report.selection,
        missing_elements=gst_report.missing_elements,
    )


def check_execution(
    requested_decoder: str = DECODER_AUTO,
    *,
    root: Path | None = None,
    prefix: str | os.PathLike[str] | None = None,
    executable: str | os.PathLike[str] | None = None,
    gstreamer_probe: Callable[[str], GStreamerReport] | None = None,
    version_lookup: Callable[[str], str] = metadata.version,
) -> RuntimeReport:
    selected_root = (root or repository_root()).resolve()
    interpreter = check_official_interpreter(root=selected_root, prefix=prefix, executable=executable)
    if interpreter.exit_code == EXIT_USAGE:
        return interpreter
    diagnostics = list(interpreter.diagnostics)
    diagnostics.extend(check_python_distributions(root=selected_root, version_lookup=version_lookup))
    diagnostics.extend(check_runtime_executables())
    gst_report = (gstreamer_probe or probe_gstreamer)(requested_decoder)
    diagnostics.extend(gst_report.diagnostics)
    return RuntimeReport(
        tuple(diagnostics),
        selection=gst_report.selection,
        missing_elements=gst_report.missing_elements,
    )


def combine_reports(*reports: RuntimeReport | GStreamerReport) -> RuntimeReport:
    diagnostics: list[Diagnostic] = []
    selection: DecoderSelection | None = None
    missing: list[str] = []
    forced: int | None = None
    for report in reports:
        diagnostics.extend(report.diagnostics)
        selection = report.selection or selection
        missing.extend(report.missing_elements)
        if isinstance(report, RuntimeReport) and report.forced_exit_code is not None:
            forced = max(forced or EXIT_READY, report.forced_exit_code)
    return RuntimeReport(
        tuple(diagnostics),
        selection=selection,
        missing_elements=tuple(dict.fromkeys(missing)),
        forced_exit_code=forced,
    )


def render_report(report: RuntimeReport | GStreamerReport, *, title: str) -> str:
    lines = [title]
    for item in report.diagnostics:
        lines.append(f"{item.level} {item.code}: {item.message}")
        if item.ubuntu_packages:
            lines.append(f"  Ubuntu package: {', '.join(item.ubuntu_packages)}")
        for command in item.recovery:
            lines.append(f"  Recovery: {command}")
    if report.selection is not None:
        lines.append(f"selected decoder: {report.selection.requested}")
        lines.append(f"effective decoder: {report.selection.effective or 'none'}")
    lines.append(
        f"missing elements: {', '.join(report.missing_elements) if report.missing_elements else 'none'}"
    )
    return "\n".join(lines)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check the canonical SASL operator runtime without modifying it."
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--execution", action="store_true", help="Check the canonical venv and app runtime.")
    mode.add_argument("--deployment", action="store_true", help="Check Ubuntu packages and venv recreation.")
    parser.add_argument(
        "--decoder",
        choices=(DECODER_AUTO, DECODER_NVIDIA, DECODER_SOFTWARE),
        default=DECODER_AUTO,
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.execution:
        report = check_execution(args.decoder)
        title = "SASL operator execution runtime"
    else:
        report = check_deployment(args.decoder)
        title = "SASL operator deployment runtime"
    print(render_report(report, title=title))
    return report.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
