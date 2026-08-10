from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "run_operator_view.sh"


class RunOperatorViewPx4RequiredTests(unittest.TestCase):
    def run_script(self, *args: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(SCRIPT_PATH), *args],
            cwd=ROOT_DIR,
            env=env,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def make_isolated_launcher(self, root: Path) -> tuple[Path, Path, dict[str, str]]:
        launcher = root / "run_operator_view.sh"
        shutil.copy2(SCRIPT_PATH, launcher)
        shutil.copytree(ROOT_DIR / "config", root / "config")
        magnetometer_build = (
            root / "stereo_ground_sim" / "vendor" / "gz_magnetometer_wmm_patch" / "build.sh"
        )
        magnetometer_build.parent.mkdir(parents=True)
        magnetometer_build.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
        magnetometer_build.chmod(0o755)
        camera_airframe = (
            root / "stereo_ground_sim" / "vendor" / "px4_camera_airframe" / "4090_camera_px4"
        )
        camera_airframe.parent.mkdir(parents=True)
        shutil.copy2(
            ROOT_DIR / "stereo_ground_sim" / "vendor" / "px4_camera_airframe" / "4090_camera_px4",
            camera_airframe,
        )

        marker = root / "side-effects.log"
        scripts_dir = root / "scripts"
        scripts_dir.mkdir()
        shutil.copy2(ROOT_DIR / "scripts" / "load_sim_config.sh", scripts_dir / "load_sim_config.sh")
        ensure_yolo = scripts_dir / "ensure_yolo_runtime.sh"
        ensure_yolo.write_text(
            f"#!/usr/bin/env bash\nprintf '%s\\n' yolo-bootstrap >> {str(marker)!r}\nprintf '%s\\n' /usr/bin/python3\n",
            encoding="utf-8",
        )
        ensure_yolo.chmod(0o755)
        prepare_tensorrt = scripts_dir / "prepare_yolo_tensorrt_engine.py"
        prepare_tensorrt.write_text(
            "#!/usr/bin/env python3\n"
            "import pathlib, sys\n"
            "output = pathlib.Path(sys.argv[sys.argv.index('--engine') + 1])\n"
            "output.touch()\n"
            "print(output)\n",
            encoding="utf-8",
        )
        prepare_tensorrt.chmod(0o755)

        install_dir = root / "install"
        install_dir.mkdir()
        (install_dir / "setup.bash").write_text("#!/usr/bin/env bash\n", encoding="utf-8")

        ros_workspace = root / "workspace" / "main"
        px4_msgs_dir = ros_workspace / "src" / "px4_msgs"
        px4_msgs_dir.mkdir(parents=True)
        (px4_msgs_dir / "package.xml").write_text(
            "<package format=\"3\"><name>px4_msgs</name><version>0.0.0</version>"
            "<description>test fixture</description><maintainer email=\"test@example.invalid\">test</maintainer>"
            "<license>Apache-2.0</license></package>\n",
            encoding="utf-8",
        )
        subprocess.run(["git", "init", "-q", str(px4_msgs_dir)], check=True)
        subprocess.run(["git", "-C", str(px4_msgs_dir), "add", "package.xml"], check=True)
        subprocess.run(
            [
                "git",
                "-C",
                str(px4_msgs_dir),
                "-c",
                "user.name=Simulation Test",
                "-c",
                "user.email=simulation-test@example.invalid",
                "commit",
                "-qm",
                "fixture",
            ],
            check=True,
        )
        px4_msgs_commit = subprocess.run(
            ["git", "-C", str(px4_msgs_dir), "rev-parse", "HEAD"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        ).stdout.strip()
        overlay_setup = ros_workspace / "install" / "setup.bash"
        overlay_setup.parent.mkdir(parents=True)
        overlay_setup.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
        px4_package_setup = (
            ros_workspace / "install" / "px4_msgs" / "share" / "px4_msgs" / "local_setup.bash"
        )
        px4_package_setup.parent.mkdir(parents=True)
        px4_package_setup.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
        overlay_manifest = ros_workspace / "install" / "share" / "sasl_ros_workspace" / "px4_msgs-overlay.env"
        overlay_manifest.parent.mkdir(parents=True)
        overlay_manifest.write_text(
            f"PX4_MSGS_COMMIT={px4_msgs_commit}\nPX4_MSGS_REF={px4_msgs_commit}\n",
            encoding="utf-8",
        )

        bin_dir = root / "bin"
        bin_dir.mkdir()
        command_bodies = {
            "MicroXRCEAgent": (
                f"printf '%s\\n' \"uxrce-agent:$*\" >> {str(marker)!r}\n"
                "/bin/sleep 1"
            ),
            "colcon": f"printf '%s\\n' colcon >> {str(marker)!r}",
            "gz": "exit 0",
            "make": (
                f"printf '%s\\n' \"make:$*:python=${{PYTHON_EXECUTABLE:-}}\" >> {str(marker)!r}\n"
                "[[ \"${PX4_TEST_MAKE_FAIL:-false}\" != true ]]"
            ),
            "pgrep": (
                "if [[ \"$*\" == \"-x make\" && \"${PX4_TEST_BUILD_ACTIVE:-false}\" == true ]]; then\n"
                "  printf '%s\\n' 99999\n"
                "fi"
            ),
            "ps": (
                "if [[ \"$*\" == *\"-p 99999\"* ]]; then\n"
                "  printf '%s\\n' 'make px4_sitl_default'\n"
                "fi"
            ),
            "readlink": (
                "if [[ \"$*\" == *\"/proc/99999/cwd\"* ]]; then\n"
                "  printf '%s\\n' \"$PX4_TEST_PX4_DIR\"\n"
                "else\n"
                "  /usr/bin/readlink \"$@\"\n"
                "fi"
            ),
            "rm": "exit 0",
            "sleep": "exit 0",
            "zenity": f"printf '%s\\n' zenity >> {str(marker)!r}",
            "setsid": (
                f"case \"$*\" in\n"
                f"  *\"make px4_sitl_default\"*) exec \"$@\" ;;\n"
                f"  *\"MicroXRCEAgent\"*) exec \"$@\" ;;\n"
                f"  *\"run_mission.sh\"*) printf '%s\\n' mission-start >> {str(marker)!r} ;;\n"
                f"  ros2*) /bin/sleep \"${{PX4_TEST_ROS_SLEEP:-0.1}}\"; printf '%s\\n' \"setsid:$1\" >> {str(marker)!r} ;;\n"
                f"  *) printf '%s\\n' \"setsid:$1\" >> {str(marker)!r} ;;\n"
                "esac"
            ),
        }
        for command, body in command_bodies.items():
            stub = bin_dir / command
            stub.write_text(
                f"#!/usr/bin/env bash\n{body}\n",
                encoding="utf-8",
            )
            stub.chmod(0o755)

        env = os.environ.copy()
        env["PATH"] = f"{bin_dir}:{env['PATH']}"
        env["SASL_ROS_WORKSPACE"] = str(ros_workspace)
        env["PX4_MSGS_REF"] = px4_msgs_commit
        return launcher, marker, env

    def make_valid_px4_checkout(self, root: Path) -> Path:
        px4_dir = root / "PX4-Autopilot"
        (px4_dir / "Tools").mkdir(parents=True)
        (px4_dir / "src").mkdir()
        (px4_dir / "ROMFS" / "px4fmu_common" / "init.d-posix" / "airframes").mkdir(
            parents=True
        )
        (px4_dir / "Makefile").write_text("# isolated PX4 checkout marker\n", encoding="utf-8")
        gz_bridge_dir = px4_dir / "build" / "px4_sitl_default" / "src" / "modules" / "simulation" / "gz_bridge"
        gz_bridge_dir.mkdir(parents=True)
        px4_binary = px4_dir / "build" / "px4_sitl_default" / "bin" / "px4"
        px4_binary.parent.mkdir(parents=True)
        px4_binary.write_text(
            "#!/usr/bin/env bash\n"
            "printf '%s\\n' \"px4:$*:cwd=$PWD:model=${PX4_GZ_MODEL_NAME:-}:namespace=${PX4_UXRCE_DDS_NS:-}:ip=${GZ_IP:-}:port=${PX4_UXRCE_DDS_PORT:-}\" >> \"$PX4_TEST_MARKER\"\n"
            "if [[ \"${PX4_TEST_PX4_FAIL_INSTANCE:-}\" == \"${3:-}\" ]]; then exit 17; fi\n"
            "/bin/sleep 1\n",
            encoding="utf-8",
        )
        px4_binary.chmod(0o755)
        subprocess.run(["git", "init", "-q", str(px4_dir)], check=True)
        subprocess.run(["git", "-C", str(px4_dir), "add", "."], check=True)
        subprocess.run(
            [
                "git",
                "-C",
                str(px4_dir),
                "-c",
                "user.name=Simulation Test",
                "-c",
                "user.email=simulation-test@example.invalid",
                "commit",
                "-qm",
                "fixture",
            ],
            check=True,
        )
        subprocess.run(["git", "-C", str(px4_dir), "tag", "v1.17.0"], check=True)
        px4_python = root / ".venv-px4-v1.17.0" / "bin" / "python"
        px4_python.parent.mkdir(parents=True)
        px4_python.write_text(
            "#!/usr/bin/env bash\n"
            "if [[ \"${1:-}\" == \"-c\" ]]; then exit 0; fi\n"
            "exec /usr/bin/python3 \"$@\"\n",
            encoding="utf-8",
        )
        px4_python.chmod(0o755)
        return px4_dir

    def assert_no_launch_side_effects(self, marker: Path) -> None:
        self.assertFalse(marker.exists(), marker.read_text(encoding="utf-8") if marker.exists() else "")

    def test_help_omits_legacy_flag_and_keeps_px4_options(self) -> None:
        result = self.run_script("--help")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("--no-px4", result.stdout)
        self.assertIn("--no-yolo", result.stdout)
        self.assertIn("--px4-dir", result.stdout)
        self.assertIn("--auto-mission", result.stdout)

    def test_no_yolo_skips_runtime_bootstrap(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
            env["PX4_TEST_MARKER"] = str(marker)
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(px4_dir), "--headless", "--no-yolo"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            events = marker.read_text(encoding="utf-8").splitlines()

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("yolo-bootstrap", events)
        self.assertIn("enable_yolo:=false", result.stdout)

    def test_dds_agent_starts_before_px4_instances(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
            env["PX4_TEST_MARKER"] = str(marker)
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(px4_dir), "--headless", "--no-yolo"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            events = marker.read_text(encoding="utf-8").splitlines()

        self.assertEqual(result.returncode, 0, result.stderr)
        agent_index = next(index for index, event in enumerate(events) if event.startswith("uxrce-agent:"))
        px4_index = next(index for index, event in enumerate(events) if event.startswith("px4:"))
        self.assertLess(agent_index, px4_index, events)

    def test_existing_but_invalid_px4_dir_exits_before_bootstrap_build_or_launch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            invalid_px4 = root / "not-a-px4-checkout"
            invalid_px4.mkdir()
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(invalid_px4)],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            self.assertEqual(result.returncode, 2)
            self.assertIn("valid PX4-Autopilot checkout", result.stderr)
            self.assert_no_launch_side_effects(marker)

    def test_missing_canonical_px4_msgs_exits_with_bootstrap_guidance_before_side_effects(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
            shutil.rmtree(Path(env["SASL_ROS_WORKSPACE"]) / "src" / "px4_msgs")
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(px4_dir), "--headless", "--no-yolo"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            self.assertEqual(result.returncode, 2)
            self.assertIn("canonical px4_msgs source is missing", result.stderr)
            self.assertIn("scripts/bootstrap_ros_workspace.sh", result.stderr)
            self.assert_no_launch_side_effects(marker)

    def test_stale_px4_msgs_overlay_exits_before_zenity_or_other_side_effects(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            manifest = Path(env["SASL_ROS_WORKSPACE"]) / "install" / "share" / "sasl_ros_workspace" / "px4_msgs-overlay.env"
            manifest.write_text("PX4_MSGS_COMMIT=0000000000000000000000000000000000000000\n", encoding="utf-8")
            result = subprocess.run(
                [str(launcher), "--select-yolo-model"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            self.assertEqual(result.returncode, 2)
            self.assertIn("canonical px4_msgs overlay is stale", result.stderr)
            self.assert_no_launch_side_effects(marker)

    def test_legacy_flag_prescan_precedes_zenity_bootstrap_build_and_launch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            result = subprocess.run(
                [str(launcher), "--select-yolo-model", "--no-px4"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            self.assertEqual(result.returncode, 2)
            self.assertIn("--no-px4 has been removed", result.stderr)
            self.assertIn("PX4 SITL is required", result.stderr)
            self.assert_no_launch_side_effects(marker)

    def test_valid_checkout_starts_three_isolated_px4_instances_without_default_mission(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
            env["PX4_TEST_MARKER"] = str(marker)
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(px4_dir), "--headless"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            events = marker.read_text(encoding="utf-8").splitlines()

        self.assertEqual(result.returncode, 0, result.stderr)
        make_events = [event for event in events if event.startswith("make:px4_sitl_default:")]
        self.assertEqual(len(make_events), 1, events)
        self.assertEqual(
            make_events[0],
            f"make:px4_sitl_default:python={root / '.venv-px4-v1.17.0' / 'bin' / 'python'}",
        )
        px4_events = [event for event in events if event.startswith("px4:")]
        self.assertEqual(len(px4_events), 3, events)
        expected_cwd = str(px4_dir / "build" / "px4_sitl_default" / "src" / "modules" / "simulation" / "gz_bridge")
        self.assertTrue(all(f"cwd={expected_cwd}" in event for event in px4_events), px4_events)
        self.assertTrue(all("ip=127.0.0.1" in event for event in px4_events), px4_events)
        self.assertTrue(any("px4:-d -i 0:" in event and "model=x500_15m" in event and "namespace=:" in event for event in px4_events), px4_events)
        self.assertTrue(any("px4:-d -i 1:" in event and "model=left_stereo_camera" in event and "namespace=camera_left" in event for event in px4_events), px4_events)
        self.assertTrue(any("px4:-d -i 2:" in event and "model=right_stereo_camera" in event and "namespace=camera_right" in event for event in px4_events), px4_events)
        self.assertTrue(all("port=8888" in event for event in px4_events), px4_events)
        self.assertNotIn("mission-start", events)

    def test_custom_uxrce_port_reaches_every_px4_instance(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
            env["PX4_TEST_MARKER"] = str(marker)
            env["PX4_UXRCE_DDS_PORT"] = "9999"
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(px4_dir), "--headless", "--no-yolo"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            events = marker.read_text(encoding="utf-8").splitlines()

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("uxrce-agent:udp4 -p 9999", events)
        px4_events = [event for event in events if event.startswith("px4:")]
        self.assertEqual(len(px4_events), 3, events)
        self.assertTrue(all("port=9999" in event for event in px4_events), px4_events)

    def test_px4_supervisor_failure_stops_operator_view(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
            env["PX4_TEST_MARKER"] = str(marker)
            env["PX4_TEST_PX4_FAIL_INSTANCE"] = "1"
            env["PX4_TEST_ROS_SLEEP"] = "1"
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(px4_dir), "--headless", "--no-yolo"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

        self.assertEqual(result.returncode, 17, result.stderr)
        self.assertIn("PX4 instance supervisor exited unexpectedly", result.stderr)

    def test_auto_mission_is_opt_in(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
            env["PX4_TEST_MARKER"] = str(marker)
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(px4_dir), "--headless", "--auto-mission"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            events = marker.read_text(encoding="utf-8").splitlines()

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(sum(event.startswith("make:px4_sitl_default:") for event in events), 1, events)
        self.assertEqual(sum(event.startswith("px4:-d -i ") for event in events), 3, events)
        self.assertEqual(events.count("mission-start"), 1, events)

    def test_failed_build_does_not_invoke_stale_px4_binary(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
            env["PX4_TEST_MARKER"] = str(marker)
            env["PX4_TEST_MAKE_FAIL"] = "true"
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(px4_dir), "--headless"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            events = marker.read_text(encoding="utf-8").splitlines()

        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("PX4 instance supervisor exited unexpectedly", result.stderr)
        self.assertEqual(sum(event.startswith("make:px4_sitl_default:") for event in events), 1, events)
        self.assertFalse(any(event.startswith("px4:") for event in events), events)

    def test_current_checkout_build_stops_before_global_cleanup(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
            env["PX4_TEST_BUILD_ACTIVE"] = "true"
            env["PX4_TEST_PX4_DIR"] = str(px4_dir)
            result = subprocess.run(
                [str(launcher), "--px4-dir", str(px4_dir), "--headless"],
                cwd=root,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            events = marker.read_text(encoding="utf-8").splitlines() if marker.exists() else []

        self.assertEqual(result.returncode, 2)
        self.assertIn("PX4 SITL build is already running", result.stderr)
        self.assertEqual(events, [])


if __name__ == "__main__":
    unittest.main()
