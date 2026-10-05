# Workstation UI Refinement Plan

- Status: Planned; only the Session workspace changes described below are implemented
- Scope: reduce generated-dashboard patterns without weakening engineering evidence

## 1. Design rules

1. Show one primary next action per workflow state.
2. Show blockers by default; keep passing checks available on demand.
3. Do not repeat the same state in Workspace, Inspector, Diagnostics, and the status bar.
4. Keep one language within each operator surface.
5. Keep factual values, source, timestamp/age, units, and coordinate contract available, but move
   long provenance into selected detail or Inspector.
6. Design from the operator task rather than directly rendering backend schemas.
7. Preserve raw evidence and fail-closed behavior through every presentation change.

## 2. Session — implemented

- Promote the daily parent lifecycle to the top-level Session workspace.
- Emphasize the next valid action instead of giving every lifecycle button equal weight.
- Replace duplicated readiness cards and a full pass/fail table with one summary and blocker-only
  rows.
- Always list all eight Measurement methods. A missing candidate/profile appears as `BLOCKED`
  with required inputs; it does not disappear and no placeholder value is fabricated.
- Keep the method list compact. Show complete R/P/Y, N/E/D, `R,t`, instance, hash, blocker, and
  coordinate contract only for the selected method.
- Retain the ordered Measurement inventory and explicit review entry point.

Acceptance:

- Mountain, Drone RTK Rotation, Drone RTK Multi-view, Terrain PnP, and Combined remain visible
  before candidates exist.
- With all collection gates ready and no geometry method ready, the surface reports collection
  `READY` and validity `BLOCKED` with exactly one validity blocker.
- No Session static label mixes Korean and English.
- The active next action is visually distinct and all invalid lifecycle actions remain disabled.

## 3. Alignment — planned only

Problems:

- readiness, acquisition commands, annotation mode, solver tabs, candidate review, Inspector, and
  Diagnostics compete in the first viewport;
- the frozen-reference command row is too long and clips at narrower widths;
- secondary FTP/import actions have the same visual weight as `Capture Live Pair`;
- persistent explanatory sentences duplicate method documentation.

Plan:

1. Keep one compact readiness line and show only blockers above the frozen pair.
2. Make `Capture Live Pair` the primary acquisition action.
3. Group HR/OZ/Import alternatives under a clearly labelled secondary acquisition menu or row.
4. Keep annotation mode adjacent to the frozen image; move Clear beside the annotation state.
5. Preserve the seven promoted method tabs, but show inputs and candidate quality only for the
   selected tab.
6. Keep Apply/Revert in one stable candidate footer with an explicit observable-DOF summary.
7. Move long coordinate/provenance explanations to Inspector while retaining source, age, and
   contract for the selected candidate.

Acceptance:

- no acquisition action is clipped at 1366×768;
- frozen frame identity and exact `K,D` applicability remain visible;
- switching tabs never deletes annotations or changes an applied candidate;
- manual Horizon/Mountain/Target workflows remain directly reachable.

## 4. Measurement — planned only

Problems:

- legacy text still describes an automatic Session and a hidden Start Session workflow;
- Preview/Detection controls and a multi-line lifecycle dump leave an unbalanced Run area;
- method status is repeated between Session, Measurement, Inspector, and Diagnostics.

Plan:

1. Remove Session lifecycle instructions from Measurement; show only Session ID, child ID, and a
   link back to Session.
2. Keep Detection as the single primary action and place Preview as a secondary camera action.
3. Put the effective method, fallback marker, pair sync, and validity in one compact strip above
   the images.
4. Keep live imagery and accepted/rejected result evidence as the dominant surface.
5. Move DOF fill editing to pre-Detection detail and collapse it after the policy locks.
6. Keep full method/source values in Session selected detail and immutable pair evidence.

Acceptance:

- Measurement contains no `Start Session` or automatic-session wording;
- the effective method and fallback are visible during Detection without scrolling;
- existing native-frame, result, quality, comparison, and history behavior is unchanged.

## 5. Recording / Review — planned only

Problems:

- Navigation/Inspector say `Recording / Review` while the workspace header still says
  `Recording / Sessions`;
- package verification, run setup, capture, ULog, playback, and saved review share one long page;
- empty evidence tables occupy most of the initial viewport.

Plan:

1. Use `Recording / Review` consistently.
2. Move pre-session Run Setup ownership to Session, retaining exact validation and lock rules.
3. Split current-child evidence from sealed-session review with local tabs.
4. Collapse empty artifact/source tables into a factual empty state.
5. Keep verification, revision creation, capture import, ULog, and export controls with their
   owning artifact view.

Acceptance:

- run setup is applied before Session evidence locks and survives restart;
- selecting a sibling Measurement never overwrites the chosen review target;
- package hashes, missing reasons, and revision chain remain inspectable.

## 6. Overview, Inspector, Diagnostics, and global chrome — planned only

Plan:

- Overview: show task-level readiness and the next action; avoid duplicating all status-bar items.
- Inspector: default to selected-object detail and omit whole-system geometry when irrelevant.
- Diagnostics: keep the dock available but do not open it by default on routine task screens.
- Status bar: retain compact device/run states; remove any state already guaranteed visible beside
  the active primary action.
- Navigation: keep stable contexts and fix restored-selection mismatch so the highlighted item
  always matches the central workspace.

## 7. Settings — planned only

Plan:

- retain native dense forms for engineering values;
- add short section-level validation instead of per-field prose;
- keep Apply and Save semantics distinct and visible;
- standardize terminology and language without changing setting ownership or persistence.

## 8. Proposed implementation order

1. Measurement stale lifecycle text and effective-method strip
2. Alignment command hierarchy and selected-method detail
3. Recording / Review ownership split and title consistency
4. Inspector/Diagnostics default visibility and duplication removal
5. Overview and Settings terminology pass

Each phase requires focused UI tests, the existing Workstation regression suite, and a rendered
1366×768 and 1600×980 review. Hardware-dependent correctness remains a separate bench/field gate.
