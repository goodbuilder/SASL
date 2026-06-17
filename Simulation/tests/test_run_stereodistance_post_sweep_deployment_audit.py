from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_runner_module():
    spec = importlib.util.spec_from_file_location(
        "run_stereodistance_post_sweep_deployment_audit_under_test",
        ROOT_DIR / "scripts" / "run_stereodistance_post_sweep_deployment_audit.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load run_stereodistance_post_sweep_deployment_audit.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunStereoDistancePostSweepDeploymentAuditTests(unittest.TestCase):
    def test_runner_chains_selection_blend_and_summary_outputs(self) -> None:
        module = load_runner_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            output_dir = root / "post"
            calls: list[tuple[str, Path]] = []

            def fake_selection(**kwargs):
                calls.append(("selection", kwargs["output_dir"]))
                kwargs["output_dir"].mkdir(parents=True)
                return [], [], {"run_count": 1}

            def fake_blend(**kwargs):
                calls.append(("blend", kwargs["output_dir"]))
                kwargs["output_dir"].mkdir(parents=True)
                return [], [], {"policy_count": 4}

            def fake_summary(**kwargs):
                calls.append(("summary", kwargs["output_dir"]))
                kwargs["output_dir"].mkdir(parents=True)
                return [], {
                    "recommended_strategy": "blend_policy_calibration_replay_all_catalog",
                    "recommended_policy": "balanced",
                }

            with patch.object(module, "audit_selection_strategies", side_effect=fake_selection), patch.object(
                module,
                "audit_blend_strategies",
                side_effect=fake_blend,
            ), patch.object(module, "summarize_deployment_audits", side_effect=fake_summary):
                manifest = module.run_post_sweep_deployment_audit(
                    sweep_dir=sweep_dir,
                    output_dir=output_dir,
                    stability_splits=["train", "validation"],
                    candidate_source_mode="all_catalog",
                )

            self.assertEqual(
                calls,
                [
                    ("selection", output_dir / "profile_selection_strategy_audit"),
                    ("blend", output_dir / "profile_blend_strategy_audit"),
                    ("summary", output_dir / "deployment_audit_strategy_summary"),
                ],
            )
            self.assertEqual(manifest["recommended_policy"], "balanced")
            self.assertEqual(manifest["candidate_source_mode"], "all_catalog")
            self.assertEqual(manifest["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertTrue((output_dir / "post_sweep_deployment_audit_manifest.json").is_file())

    def test_runner_can_skip_blend_audit(self) -> None:
        module = load_runner_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)

            with patch.object(module, "audit_selection_strategies", return_value=([], [], {})) as selection, patch.object(
                module,
                "audit_blend_strategies",
            ) as blend, patch.object(
                module,
                "summarize_deployment_audits",
                return_value=([], {"recommended_strategy": "single_profile_calibration_replay_minimal_set"}),
            ) as summary:
                manifest = module.run_post_sweep_deployment_audit(
                    sweep_dir=root / "sweep",
                    output_dir=root / "post",
                    skip_blend_audit=True,
                )

            self.assertTrue(selection.called)
            self.assertFalse(blend.called)
            self.assertEqual(summary.call_args.kwargs["blend_audit_dir"], None)
            self.assertEqual(manifest["blend_audit_dir"], "")


if __name__ == "__main__":
    unittest.main()
