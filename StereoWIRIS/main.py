from pathlib import Path
import os
import sys

SASL_ROOT = Path(__file__).resolve().parents[1]
if str(SASL_ROOT) not in sys.path:
    sys.path.insert(0, str(SASL_ROOT))

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



def _workstation_requested(argv: list[str]) -> bool:
    """Return whether this launch should load the standalone Workstation UI."""

    for index, argument in enumerate(argv[1:], start=1):
        if argument == "--ui" and index + 1 < len(argv):
            return argv[index + 1] == "workstation"
        if argument.startswith("--ui="):
            return argument.partition("=")[2] == "workstation"
    return False


def main() -> int:
    # Keep the Legacy import lazy.  A Workstation process must load only the
    # copied GUI module; this is the import boundary that makes the two
    # runtimes independently replaceable.
    if _workstation_requested(sys.argv):
        from workstation.standalone_gui import main as selected_main
    else:
        from wiris_stereo_distance.legacy_gui import main as selected_main
    return selected_main()

if __name__ == "__main__":
    raise SystemExit(main())
