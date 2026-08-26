import json
import os
import sys
from pathlib import Path

SASL_ROOT = Path(__file__).resolve().parents[2]
COMMON_PYTHON = SASL_ROOT / "common" / "python"
if str(COMMON_PYTHON) not in sys.path:
    sys.path.insert(0, str(COMMON_PYTHON))

from sasl_common.operator_runtime import (
    DECODER_AUTO,
    check_execution,
    render_report,
)


def _operator_runtime_gate() -> int:
    report = check_execution(DECODER_AUTO, root=SASL_ROOT)
    if report.exit_code:
        print(render_report(report, title="StereoWIRIS startup blocked"), file=sys.stderr)
        return report.exit_code
    if any(item.level == "WARN" for item in report.diagnostics):
        print(render_report(report, title="StereoWIRIS startup warning"))
    return 0


def _workstation_requested(argv: list[str]) -> bool:
    """Return whether this launch should load the standalone Workstation UI."""

    for index, argument in enumerate(argv[1:], start=1):
        if argument == "--ui" and index + 1 < len(argv):
            return argv[index + 1] == "workstation"
        if argument.startswith("--ui="):
            return argument.partition("=")[2] == "workstation"
    return False


def _mavlink_only_requested(argv: list[str]) -> bool:
    """Return whether Legacy must start without the ROS2 runtime bootstrap.

    ``apps/StereoWIRIS/run.sh`` makes the same decision before sourcing ROS.  The
    duplicate, dependency-free check here keeps direct ``python main.py``
    launches portable as well.  ``SASL_MAVLINK_ONLY=0`` is useful for an
    operator or test that intentionally wants to validate the ROS2/dual path
    against a saved MAVLink-only settings file.
    """

    if "--mavlink-only" in argv[1:]:
        return True

    override = os.environ.get("SASL_MAVLINK_ONLY", "").strip()
    if override in {"0", "1"}:
        return override == "1"

    mode_override = os.environ.get("SASL_PIXHAWK_ETHERNET_MODE", "").strip().lower()
    if mode_override:
        return mode_override == "mavlink"

    settings_path = SASL_ROOT / "StereoWIRIS" / "stereo_distance_settings.json"
    try:
        payload = json.loads(settings_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return False
    return str(payload.get("pixhawk_ethernet_mode", "")).strip().lower() == "mavlink"


# Workstation is intentionally MAVLink/PC-clock/RTCP-NTP only, and Legacy's
# MAVLink-only mode has the same no-ROS bootstrap contract.  Do not add ROS
# Python/library paths to either process unless Legacy is explicitly running
# the dual/uXRCE transport.  An operator launching either MAVLink-only UI must
# not need a ROS installation or a generated px4_msgs overlay just to start.
WORKSTATION_REQUESTED = _workstation_requested(sys.argv)
MAVLINK_ONLY_REQUESTED = _mavlink_only_requested(sys.argv)

if __name__ == "__main__" and not any(argument in {"--help", "-h"} for argument in sys.argv[1:]):
    runtime_status = _operator_runtime_gate()
    if runtime_status:
        raise SystemExit(runtime_status)

if not WORKSTATION_REQUESTED:
    # The panel uses this runtime marker to disable ROS2-only controls and
    # topic discovery.  It is deliberately set before importing Legacy GUI.
    os.environ.setdefault("SASL_MAVLINK_ONLY", "1" if MAVLINK_ONLY_REQUESTED else "0")

if str(SASL_ROOT) not in sys.path:
    sys.path.insert(0, str(SASL_ROOT))

if not WORKSTATION_REQUESTED and not MAVLINK_ONLY_REQUESTED:
    for ros_path in (
        Path("/opt/ros/humble/lib/python3.10/site-packages"),
        Path("/opt/ros/humble/local/lib/python3.10/dist-packages"),
    ):
        if ros_path.exists():
            path_text = str(ros_path)
            if path_text not in sys.path:
                sys.path.insert(0, path_text)

    workspace = SASL_ROOT / "workspace" / "main"
    install_dir = workspace / "install"
    if install_dir.exists():
        for package_path in install_dir.glob("*/local/lib/python*/dist-packages"):
            path_text = str(package_path)
            if path_text not in sys.path:
                sys.path.insert(0, path_text)

    library_paths = [
        Path("/opt/ros/humble/lib"),
        workspace / "install" / "px4_msgs" / "lib",
    ]
    existing_library_paths = [path for path in os.environ.get("LD_LIBRARY_PATH", "").split(":") if path]
    missing_library_paths = [
        str(path) for path in library_paths if path.exists() and str(path) not in existing_library_paths
    ]
    if missing_library_paths and os.environ.get("SASL_ROS_LIBRARY_PATH_READY") != "1":
        env = dict(os.environ)
        env["LD_LIBRARY_PATH"] = ":".join(missing_library_paths + existing_library_paths)
        env["SASL_ROS_LIBRARY_PATH_READY"] = "1"
        os.execvpe(sys.executable, [sys.executable, *sys.argv], env)


def main() -> int:
    if _workstation_requested(sys.argv):
        print(
            "StereoWIRIS and workstation are independent apps; "
            "launch Workstation with `python -m Workstation`.",
            file=sys.stderr,
        )
        return 2
    from wiris_stereo_distance.legacy_gui import main as selected_main
    return selected_main()

if __name__ == "__main__":
    raise SystemExit(main())
