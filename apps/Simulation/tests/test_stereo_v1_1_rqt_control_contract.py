from __future__ import annotations

import json
from pathlib import Path
from xml.etree import ElementTree


ROOT = Path(__file__).resolve().parents[1]


def test_campaign_rqt_plugin_registration_matches_control_perspective():
    plugin_xml = ElementTree.parse(ROOT / "stereo_ground_sim" / "plugin.xml").getroot()
    classes = {row.attrib["name"]: row.attrib["type"] for row in plugin_xml.findall("class")}
    assert classes["CampaignPanel"] == "stereo_ground_sim.campaign_panel.CampaignPanel"
    perspective = json.loads((ROOT / "config" / "campaign_control.perspective").read_text())
    running = perspective["groups"]["pluginmanager"]["keys"]["running-plugins"]["repr"]
    assert running == "{'stereo_ground_sim/CampaignPanel': [1]}"


def test_standalone_campaign_rqt_does_not_launch_or_get_cleaned_up_with_operator_stack():
    launcher = (ROOT / "run_stereo_campaign_rqt.sh").read_text()
    assert "campaign_control.perspective" in launcher
    assert "run_operator_view.sh" not in launcher
    assert "operator_rqt.perspective" not in launcher
    assert "config/simulation.env" in launcher


def test_manifest_campaign_is_the_default_and_legacy_sweep_is_explicit():
    source = (ROOT / "campaigns" / "stereo_v1_1" / "control_cli.py").read_text()
    runner = ROOT / "run_auto_attitude_sweep.sh"
    assert runner.is_file() and runner.stat().st_mode & 0o111
    runner_source = runner.read_text()
    assert '--stereo-v1-1)' in runner_source
    assert '--legacy)' in runner_source
    assert 'exec python3 "$ROOT_DIR/campaigns/stereo_v1_1/control_cli.py" "$@"' in runner_source
    assert 'run_auto_attitude_sweep.sh --legacy' in runner_source
    panel_source = (ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "campaign_panel.py").read_text()
    assert 'SWEEP_RUNNER = SIM_ROOT / "run_auto_attitude_sweep.sh"' in panel_source
    assert '"--stereo-v1-1", "--config"' in panel_source
    assert 'if config.action == "legacy_sweep"' in source
    assert '"run_auto_attitude_sweep.sh"' in source
    assert "campaign_run.py" in source
    assert "legacy_sweep" not in (ROOT / "campaigns" / "stereo_v1_1" / "campaign_run.py").read_text()
