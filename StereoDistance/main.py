from pathlib import Path
import importlib
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


def _align_mpl_toolkits_with_matplotlib() -> None:
    """Keep Ubuntu's mpl_toolkits namespace from shadowing a user Matplotlib install."""
    try:
        matplotlib_spec = importlib.util.find_spec("matplotlib")
        if matplotlib_spec is None or matplotlib_spec.origin is None:
            return
        mpl_toolkits = importlib.import_module("mpl_toolkits")
        matplotlib_root = Path(matplotlib_spec.origin).resolve().parent.parent
        matching_toolkits = matplotlib_root / "mpl_toolkits"
        if not matching_toolkits.is_dir():
            return
        current_paths = tuple(Path(path).resolve() for path in getattr(mpl_toolkits, "__path__", ()))
        if current_paths == (matching_toolkits.resolve(),):
            return
        mpl_toolkits.__path__[:] = [str(matching_toolkits)]
    except Exception as exc:
        print(f"[runtime] Matplotlib toolkit path alignment skipped: {exc}", file=sys.stderr)


_align_mpl_toolkits_with_matplotlib()

from wiris_stereo_distance.gui import main

if __name__ == "__main__":
    raise SystemExit(main())
