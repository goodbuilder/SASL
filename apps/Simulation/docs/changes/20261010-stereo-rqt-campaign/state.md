---
schema: modify-gui-change-state-v1
change_id: 20261010-stereo-rqt-campaign
status: complete
mode: backend-coupled
updated: 2026-10-10
baseline_ref: null
last_completed: "Visible RQT readback of separate source/output roots; live G0/G1 no-flight preflight pass; 100-trial QtTest reprocess on staged read-only source; direct-root one-trial QtTest reprocess; 81 passing focused tests and repository portability check"
next_question: ""
read_next:
  - docs/changes/20261010-stereo-rqt-campaign/verification.md
  - docs/changes/20261010-stereo-rqt-campaign/approval-summary.md
affected_product_docs:
  - README.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Implement the Simulation.env-to-RQT, no-LLM stereo v1.1 workflow, including results, while preserving the legacy sweep."
  conditions:
    - "Changes remain in apps/Simulation and temporary verification data stays under /tmp."
    - "The existing external 39-run source remains read-only."
    - "SITL only; no hardware flight."
  recorded_at: 2026-10-10
---

# Resume Notes

The operator can edit the literal campaign block in `config/simulation.env` and dispatch it through the standalone RQT panel. The default action is `plan`; the configured Monte Carlo count is 100. Existing data can be reprocessed by selecting `reprocess`; a complete collection-plus-results pass is `all_results`. `README.md` documents these actions and the output layout.

The existing external source was freshly QC-checked read-only at 39/39 slots. A QtTest click on the actual CampaignPanel **Run configured action** ran its `reprocess` action with 100 Monte Carlo trials to `ANALYSIS_COMPLETE`: 758 calibration batches, 6,822 calibration results, 6,732 V0 evaluation rows, 50 sensitivity cases, selected path P1, and 36,071 verified checksums. The handoff is `/tmp/sasl-usergoal-sensitivity-probe-chwpnt6i/analysis_versions/analysis_ec9526f32496/analysis_summary.json`; this run used a temporary staging root with a read-only link to the source.

After adding separate input/output roots, a second QtTest button action reprocessed directly from the original campaign root to `/tmp/sasl-direct-root-input-test-20261010`; it completed with 3,258 calibration results, 2,376 V0 rows, 12,905 verified checksums, and P1 selected. Its run manifest records the source identity hash without storing the absolute path. This verifies the no-copy source route at one trial; the 100-trial analysis was verified through the staging route using the same source data.

The runtime setup now builds the pinned repo-owned `px4_msgs` overlay into `workspace/main/install` with a workspace-keyed build cache. The pinned YOLO environment verified CUDA on the RTX 4050. A fresh non-flying G0/G1 preflight passed, measuring 50 Hz GT, 0-degree camera-pose error, and enough output-disk space for all 39 slots. The standalone RQT screenshot shows `reprocess`, the direct external 39-run source as input, a separate repo output, MC=100, and source status valid=39/failed=0. `STEREO_V1_1_INPUT_ROOT` now accepts `output`, `repo`, `data_root`, or a path relative to the SASL root. Its direct source QC and CLI dispatch have tests; the full 100-trial run used the earlier staged-root form. Focused tests pass (81); the legacy sweep still plans 20 independent cases. No new flight was added because the existing valid 39-run source is reused and protocol v1.1 specifies zero extra Gazebo runs. Seven V0 science-entry failures remain preserved performance outcomes.
