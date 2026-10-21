#!/usr/bin/env python3
"""Isolated DEVELOPMENT-only real ROS recorder crash/restart verification.

No vehicle, simulation, science run, YOLO, or positioning result is produced.
Run only after campaign execution has stopped, in the project ROS environment.
The explicit new output directory owns every subprocess and artifact. It never
writes campaign status. Synthetic topics stay on a nonzero localhost domain.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
SIM_ROOT = HERE.parents[1]
if str(SIM_ROOT) not in sys.path:
    sys.path.insert(0, str(SIM_ROOT))


def snapshot_files(root: Path) -> dict[str, str]:
    result = {}
    for path in sorted(root.rglob("*")):
        if path.is_file():
            digest = hashlib.sha256()
            with path.open("rb") as handle:
                for chunk in iter(lambda: handle.read(1024*1024), b""):
                    digest.update(chunk)
            result[str(path.relative_to(root))] = digest.hexdigest()
    return result


def owned_bag_argv(argv: list[str], attempt: Path) -> bool:
    """Match actual ros2 bag record argv and exact attempt-owned -o directory."""
    try:
        record = argv.index("record")
        output = argv[argv.index("-o") + 1]
    except (ValueError, IndexError):
        return False
    return (record > 0 and argv[record-1] == "bag"
            and any(Path(arg).name == "ros2" for arg in argv[:record])
            and Path(output).resolve() in {
                (attempt / "raw_images_or_lossless_bag" / side).resolve()
                for side in ("left", "right", "state")})


def process_info(pid: int):
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        tail = raw[raw.rfind(")")+2:].split()
        argv = [s.decode() for s in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0") if s]
        return {"pid": pid, "ppid": int(tail[1]), "start_ticks": tail[19], "argv": argv}
    except (OSError, ValueError, UnicodeError, IndexError):
        return None


def processes():
    return [info for path in Path("/proc").iterdir() if path.name.isdigit()
            if (info := process_info(int(path.name))) is not None]


def owned_bag_children(parent_pid: int, attempt: Path):
    return [p for p in processes() if p["ppid"] == parent_pid and owned_bag_argv(p["argv"], attempt)]


def verify_identity(saved: dict) -> bool:
    current = process_info(saved["pid"])
    return bool(current and current["start_ticks"] == saved["start_ticks"]
                and current["argv"] == saved["argv"])


def refuse_sitl():
    for process in processes():
        names = [Path(v).name for v in process["argv"]]
        if (names and names[0] == "px4") or any(
                names[i] in ("gz", "ign") and names[i+1] in ("sim", "gazebo")
                for i in range(len(names)-1)):
            raise RuntimeError(f"SITL/Gazebo process {process['pid']} is active; stop it before this test")


def wait_token(process, log: Path, token: str, timeout_s: float):
    deadline = time.monotonic()+timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"process exited before {token}: {process.returncode}")
        if log.is_file() and token in log.read_text(errors="replace"):
            return
        time.sleep(.1)
    raise TimeoutError(f"deadline waiting for {token}")


def stop_owned(process, children: list[dict]):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=40)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
    # capture_streams children own distinct process groups. Never signal a PID
    # recycled after exit or a command outside this harness's attempt directory.
    for child in children:
        if verify_identity(child):
            os.killpg(child["pid"], signal.SIGTERM)
    deadline = time.monotonic()+3
    while time.monotonic() < deadline and any(verify_identity(p) for p in children):
        time.sleep(.1)
    for child in children:
        if verify_identity(child):
            os.killpg(child["pid"], signal.SIGKILL)


def publish_synthetic():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("development_capture_recovery_publisher")
    pubs = {}
    for side in ("left", "right"):
        pubs[f"{side}_image"] = node.create_publisher(Image, f"/{side}_camera/image", 10)
        pubs[f"{side}_info"] = node.create_publisher(CameraInfo, f"/{side}_camera/camera_info", 10)
        pubs[f"{side}_pose"] = node.create_publisher(String, f"/simulation/camera_pose_diagnostics/{side}", 10)
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    gt_pub = node.create_publisher(PoseStamped, "/simulation/ground_truth/vehicle_reference_link", 10)
    ticks = 0
    def tick():
        nonlocal ticks
        ticks += 1
        stamp_ns = ticks*20_000_000
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
        clock_pub.publish(clock)
        gt = PoseStamped()
        gt.header.stamp = clock.clock
        gt.header.frame_id = "gazebo_world_enu"
        gt.pose.orientation.w = 1.0
        gt_pub.publish(gt)
        for side in ("left", "right"):
            msg = Image()
            msg.header.stamp = clock.clock
            msg.header.frame_id = f"{side}_optical"
            msg.width, msg.height, msg.step, msg.encoding = 16, 12, 48, "rgb8"
            msg.data = bytes([ticks % 255])*576
            pubs[f"{side}_image"].publish(msg)
            info = CameraInfo()
            info.header = msg.header
            info.width, info.height = 16, 12
            info.k = [10., 0., 8., 0., 10., 6., 0., 0., 1.]
            pubs[f"{side}_info"].publish(info)
            pose = String()
            pose.data = json.dumps({"development_synthetic": True, "camera_id": side})
            pubs[f"{side}_pose"].publish(pose)
    node.create_timer(.02, tick)
    print("SYNTHETIC_PUBLISHER_READY", flush=True)
    try:
        while not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run(output_root: Path, domain: int = 89) -> dict:
    if not 1 <= domain <= 101:
        raise ValueError("use an isolated nonzero ROS domain in 1..101")
    refuse_sitl()
    output_root = output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=False)
    report = {"method": "synthetic_development_capture_forced_stop_restart_v1",
              "status": "FAILED", "scientific_data": False, "planned_campaign_runs": 0,
              "ros_domain_id": domain, "ros_localhost_only": True,
              "compression": "zstd", "synthetic_image_shape": [12, 16, 3],
              "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (Path(__file__), HERE / "capture_streams.py")},
              "python_executable": sys.executable, "ros_distro": os.environ.get("ROS_DISTRO"),
              "checks": {}}
    env = {**os.environ, "ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1",
           "ROS_LOG_DIR": str(output_root / "ros_logs")}
    publisher = None
    handles = []
    def spawn(argv, log):
        handle = log.open("x")
        handles.append(handle)
        return subprocess.Popen(argv, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                start_new_session=True)
    try:
        publisher_log = output_root / "publisher.log"
        publisher = spawn([sys.executable, str(Path(__file__).resolve()), "--publisher"], publisher_log)
        wait_token(publisher, publisher_log, "SYNTHETIC_PUBLISHER_READY", 15)
        first_hashes = None
        for name in ("a01", "a02"):
            attempt = output_root / name
            attempt.mkdir()
            log = attempt / "capture_process.log"
            command = [sys.executable, str(HERE / "capture_streams.py"), "--attempt-dir", str(attempt),
                       "--protocol-version", "1.2", "--compression", "zstd"]
            if name == "a02":
                command += ["--duration-s", "5"]
            process = spawn(command, log)
            children = []
            try:
                wait_token(process, log, "CAPTURE_READY", 35)
                children = owned_bag_children(process.pid, attempt)
                if len(children) != 3:
                    raise RuntimeError("could not identify exactly three owned rosbag recorder children")
                if name == "a01":
                    time.sleep(1)
                    target = next(p for p in children if p["argv"][p["argv"].index("-o")+1].endswith("/left"))
                    if not verify_identity(target) or process_info(target["pid"])["ppid"] != process.pid:
                        raise RuntimeError("recorder ownership changed before forced stop")
                    report["injected_fault"] = {"signal": "SIGKILL", "owned_child": target,
                                                 "attempt": "a01", "stream": "left"}
                    os.kill(target["pid"], signal.SIGKILL)
                code = process.wait(timeout=45)
            finally:
                children += [p for p in owned_bag_children(process.pid, attempt)
                             if p["pid"] not in {v["pid"] for v in children}]
                stop_owned(process, children)
            summary = json.loads((attempt / "raw_capture_summary.json").read_text())
            if name == "a01":
                if code != 1 or summary["status"] != "CAPTURE_FAILED":
                    raise AssertionError("forced recorder death did not fail capture")
                first_hashes = snapshot_files(attempt)
                if not any(key.startswith("raw_images_or_lossless_bag/") for key in first_hashes):
                    raise AssertionError("failed attempt preserved no raw artifacts")
                report["checks"]["forced_stop_failed_and_preserved"] = True
                report["a01_snapshot_sha256"] = first_hashes
                # Existing attempt paths must fail before any bag process can
                # start. Keep the refusal log outside the preserved attempt.
                refusal = spawn(command + ["--duration-s", "5"], output_root / "overwrite_refusal.log")
                try:
                    if refusal.wait(timeout=20) == 0:
                        raise AssertionError("existing attempt was accepted for overwrite")
                finally:
                    stop_owned(refusal, owned_bag_children(refusal.pid, attempt))
                if snapshot_files(attempt) != first_hashes:
                    raise AssertionError("overwrite refusal changed the failed attempt")
                report["checks"]["existing_attempt_overwrite_refused"] = True
            else:
                if code != 0 or summary["status"] != "CAPTURE_COMPLETE":
                    raise AssertionError("new capture did not complete")
                from campaigns.stereo_v1_1.capture_streams import STREAM_TOPICS, V12_STATE_TOPICS
                expected = {**STREAM_TOPICS, "state": V12_STATE_TOPICS}
                if not all(summary["streams"][stream]["topic_counts"].get(topic, 0) > 0
                           for stream, topics in expected.items() for topic in topics):
                    raise AssertionError("restart lacks required raw topics")
                report["checks"]["restart_all_eight_topics_nonempty"] = True
                if snapshot_files(output_root / "a01") != first_hashes:
                    raise AssertionError("restart modified failed attempt artifacts")
                report["checks"]["failed_attempt_hashes_unchanged"] = True
        report["status"] = "PASS_DEVELOPMENT_ONLY"
    except (Exception, KeyboardInterrupt) as exc:
        report["failure_reason"] = f"{type(exc).__name__}: {exc}"
    finally:
        if publisher is not None:
            stop_owned(publisher, [])
        for handle in handles:
            handle.close()
        with (output_root / "capture_recovery_report.json").open("x") as handle:
            json.dump(report, handle, indent=2, allow_nan=False)
            handle.write("\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--domain", type=int, default=89)
    parser.add_argument("--publisher", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.publisher:
        if os.environ.get("ROS_DOMAIN_ID", "0") == "0" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
            parser.error("synthetic publisher requires isolated localhost ROS domain")
        publish_synthetic()
        return 0
    if args.output_root is None:
        parser.error("--output-root is required")
    report = run(args.output_root, args.domain)
    print(json.dumps({"status": report["status"], "failure_reason": report.get("failure_reason")}))
    return 0 if report["status"] == "PASS_DEVELOPMENT_ONLY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
