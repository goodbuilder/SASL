from __future__ import annotations

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "Simulation"
LAUNCH_FILE = SIM_ROOT / "stereo_ground_sim/launch/stereo_ground_sim.launch.py"
RUN_OPERATOR_VIEW = SIM_ROOT / "run_operator_view.sh"
ATTITUDE_EXPERIMENT = SIM_ROOT / "scripts/attitude_error_experiment.py"


def launch_default(name: str) -> str:
    pattern = rf'DeclareLaunchArgument\("{re.escape(name)}", default_value="([^"]+)"\)'
    match = re.search(pattern, LAUNCH_FILE.read_text(encoding="utf-8"))
    if match is None:
        raise AssertionError(f"Launch default for {name!r} not found")
    return match.group(1)


def script_text() -> str:
    return RUN_OPERATOR_VIEW.read_text(encoding="utf-8")


class Phase3GuiDefaultContractTests(unittest.TestCase):
    def test_plain_launch_defaults_enable_gazebo_and_rviz(self) -> None:
        self.assertEqual(launch_default("gz_gui"), "true")
        self.assertEqual(launch_default("launch_rviz"), "true")

    def test_operator_default_and_headless_modes_set_both_gui_flags(self) -> None:
        text = script_text()
        self.assertIn("headless=false", text)
        self.assertIn("  - gz_gui:=true", text)
        self.assertIn("  - launch_rviz:=true", text)

        gz_block = re.search(
            r'if ! has_launch_arg gz_gui "\$\{launch_input\[@\]\}"; then(?P<body>.*?)\nfi',
            text,
            re.DOTALL,
        )
        rviz_block = re.search(
            r'if ! has_launch_arg launch_rviz "\$\{launch_input\[@\]\}"; then(?P<body>.*?)\nfi',
            text,
            re.DOTALL,
        )
        self.assertIsNotNone(gz_block)
        self.assertIsNotNone(rviz_block)

        assert gz_block is not None
        assert rviz_block is not None
        self.assertIn('if [[ "$headless" == true ]]; then', gz_block.group("body"))
        self.assertIn('launch_args+=("gz_gui:=false")', gz_block.group("body"))
        self.assertIn('launch_args+=("gz_gui:=true")', gz_block.group("body"))
        self.assertIn('if [[ "$headless" == true ]]; then', rviz_block.group("body"))
        self.assertIn('launch_args+=("launch_rviz:=false")', rviz_block.group("body"))
        self.assertIn('launch_args+=("launch_rviz:=true")', rviz_block.group("body"))

    def test_operator_explicit_gui_launch_args_are_not_overwritten(self) -> None:
        text = script_text()
        self.assertRegex(
            text,
            r'(?s)if ! has_launch_arg gz_gui "\$\{launch_input\[@\]\}"; then'
            r'.*?launch_args\+=\("gz_gui:=false"\)'
            r'.*?launch_args\+=\("gz_gui:=true"\)'
            r'.*?\nfi',
        )
        self.assertRegex(
            text,
            r'(?s)if ! has_launch_arg launch_rviz "\$\{launch_input\[@\]\}"; then'
            r'.*?launch_args\+=\("launch_rviz:=false"\)'
            r'.*?launch_args\+=\("launch_rviz:=true"\)'
            r'.*?\nfi',
        )
        self.assertNotIn('launch_args+=("launch_rviz:=false")\nlaunch_args+=("${launch_input[@]}")', text)

    def test_attitude_sweep_stays_explicitly_headless_by_default(self) -> None:
        text = ATTITUDE_EXPERIMENT.read_text(encoding="utf-8")
        self.assertIn('f"gz_gui:={\'true\' if args.keep_gui else \'false\'}"', text)
        self.assertIn('f"launch_rviz:={\'true\' if args.keep_gui else \'false\'}"', text)
        self.assertRegex(text, r'--keep-gui')
