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

for workspace in (
    SASL_ROOT / "workspace" / "main",
    SASL_ROOT / "px4_msgs_ws",
    Path.home() / "px4_msgs_ws",
):
    install_dir = workspace / "install"
    if not install_dir.exists():
        continue
    for package_path in install_dir.glob("*/local/lib/python*/dist-packages"):
        path_text = str(package_path)
        if path_text not in sys.path:
            sys.path.insert(0, path_text)

library_paths = [
    Path("/opt/ros/humble/lib"),
    *(workspace / "install" / "px4_msgs" / "lib" for workspace in (
        SASL_ROOT / "workspace" / "main",
        SASL_ROOT / "px4_msgs_ws",
        Path.home() / "px4_msgs_ws",
    )),
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

from wiris_stereo_distance.gui import main

if __name__ == "__main__":
    raise SystemExit(main())
