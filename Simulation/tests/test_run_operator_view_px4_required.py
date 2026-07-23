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

        marker = root / "side-effects.log"
        scripts_dir = root / "scripts"
        scripts_dir.mkdir()
        ensure_yolo = scripts_dir / "ensure_yolo_runtime.sh"
        ensure_yolo.write_text(
            f"#!/usr/bin/env bash\nprintf '%s\\n' yolo-bootstrap >> {str(marker)!r}\nprintf '%s\\n' /usr/bin/python3\n",
            encoding="utf-8",
        )
        ensure_yolo.chmod(0o755)

        install_dir = root / "install"
        install_dir.mkdir()
        (install_dir / "setup.bash").write_text("#!/usr/bin/env bash\n", encoding="utf-8")

        bin_dir = root / "bin"
        bin_dir.mkdir()
        command_bodies = {
            "colcon": f"printf '%s\\n' colcon >> {str(marker)!r}",
            "gz": "exit 0",
            "make": (
                f"printf '%s\\n' \"make:$*\" >> {str(marker)!r}\n"
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
                f"  *\"run_auto_mission.sh\"*) printf '%s\\n' mission-start >> {str(marker)!r} ;;\n"
                f"  ros2*) /bin/sleep 0.1; printf '%s\\n' \"setsid:$1\" >> {str(marker)!r} ;;\n"
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
        return launcher, marker, env

    def make_valid_px4_checkout(self, root: Path) -> Path:
        px4_dir = root / "PX4-Autopilot"
        (px4_dir / "Tools").mkdir(parents=True)
        (px4_dir / "src").mkdir()
        (px4_dir / "Makefile").write_text("# isolated PX4 checkout marker\n", encoding="utf-8")
        gz_bridge_dir = px4_dir / "build" / "px4_sitl_default" / "src" / "modules" / "simulation" / "gz_bridge"
        gz_bridge_dir.mkdir(parents=True)
        px4_binary = px4_dir / "build" / "px4_sitl_default" / "bin" / "px4"
        px4_binary.parent.mkdir(parents=True)
        px4_binary.write_text(
            "#!/usr/bin/env bash\n"
            "printf '%s\\n' \"px4:$*:cwd=$PWD:model=${PX4_SIM_MODEL:-}:ip=${GZ_IP:-}\" >> \"$PX4_TEST_MARKER\"\n",
            encoding="utf-8",
        )
        px4_binary.chmod(0o755)
        return px4_dir

    def assert_no_launch_side_effects(self, marker: Path) -> None:
        self.assertFalse(marker.exists(), marker.read_text(encoding="utf-8") if marker.exists() else "")

    def test_help_omits_legacy_flag_and_keeps_px4_options(self) -> None:
        result = self.run_script("--help")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("--no-px4", result.stdout)
        self.assertIn("--px4-dir", result.stdout)
        self.assertIn("--auto-mission", result.stdout)

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

    def test_valid_checkout_starts_px4_once_without_default_mission(self) -> None:
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
        self.assertEqual(events.count("make:px4_sitl_default"), 1, events)
        px4_events = [event for event in events if event.startswith("px4:")]
        self.assertEqual(len(px4_events), 1, events)
        self.assertIn("px4:-d:", px4_events[0])
        self.assertIn("cwd=" + str(px4_dir / "build" / "px4_sitl_default" / "src" / "modules" / "simulation" / "gz_bridge"), px4_events[0])
        self.assertIn("model=gz_x500", px4_events[0])
        self.assertIn("ip=127.0.0.1", px4_events[0])
        self.assertNotIn("mission-start", events)

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
        self.assertEqual(events.count("make:px4_sitl_default"), 1, events)
        self.assertEqual(sum(event.startswith("px4:-d:") for event in events), 1, events)
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

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(events.count("make:px4_sitl_default"), 1, events)
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
