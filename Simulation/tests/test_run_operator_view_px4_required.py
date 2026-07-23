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
            "pgrep": "exit 1",
            "ps": "exit 0",
            "rm": "exit 0",
            "sleep": "exit 0",
            "zenity": f"printf '%s\\n' zenity >> {str(marker)!r}",
            "setsid": (
                f"case \"$*\" in\n"
                f"  *\"exec make px4_sitl gz_x500\"*) printf '%s\\n' px4-start >> {str(marker)!r} ;;\n"
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
        self.assertEqual(events.count("px4-start"), 1, events)
        self.assertNotIn("mission-start", events)

    def test_auto_mission_is_opt_in(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            launcher, marker, env = self.make_isolated_launcher(root)
            px4_dir = self.make_valid_px4_checkout(root)
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
        self.assertEqual(events.count("px4-start"), 1, events)
        self.assertEqual(events.count("mission-start"), 1, events)


if __name__ == "__main__":
    unittest.main()
