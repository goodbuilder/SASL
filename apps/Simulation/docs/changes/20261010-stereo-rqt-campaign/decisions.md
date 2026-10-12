# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.
# Decisions

1. **Standalone RQT control window.** The current `campaign_run.py` owns and starts the headless operator/SITL stack and rejects a second active stack. A button inside the in-stack OperatorPanel would self-conflict. The new perspective is standalone and uses a distinct perspective filename so headless stack cleanup does not close it.
2. **`.env` is a human control surface, protocol files are the scientific authority.** The panel parses only literal campaign action/run/output/MC/PX4/YOLO settings. Paths stored there are repo-relative; protocol, manifest, and G0/G1 physical profiles remain hash-pinned.
3. **Default action is `plan`.** A flight is never an accidental consequence of opening RQT. Flight-bearing actions show the planned scope and require a confirmation in the panel.
4. **Legacy sweep is a separate action.** It invokes only `run_auto_attitude_sweep.sh`, clears stale campaign/camera override variables, and is never part of `all` or `all_results`.
5. **Offline truth is role-separated.** Camera/drone RTK synthesis may read GT in separate generators; A/B receive shared noisy drone RTK, C receives no RTK/GT/commanded path. The evaluator alone reads V0 GT after the analysis freeze.
6. **Fresh and historical environments are distinguished.** New runs require a passing JSON live preflight report. Existing QC-valid data may be reprocessed with its historical Markdown environment report without claiming a fresh preflight.
7. **Result versions are immutable.** An analysis ID is derived from algorithm/protocol/dependency/settings hashes; changing code creates a new output version and does not overwrite prior results.
