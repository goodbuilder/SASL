# God module decomposition

User request: “갓모듈 분리 계획 세워서 실행해.” This explicitly authorizes planning and implementation of the four previously identified god modules, preserving current behavior. Earlier Run-only remediation is the baseline, not this task's completion criterion.

Targets: standalone_gui.py (13,702 lines), workspaces.py (17,551), recording_controller.py (8,676), measurement_view_controller.py (6,677).

Acceptance:
- G1: workspace views live under coherent screen/component owners; production imports owners directly; old workspaces entry is a thin explicit compatibility facade; leaf imports do not load unrelated screens.
- G2: panel becomes composition/view adapter; feature state and resource ownership leave the 732-attribute panel store; view construction and feature behavior have explicit owners. Shutdown still enumerates every resource. No single copied legacy owner or renamed universal context.
- G3: recording lifecycle/captures/schema projection/native completion/primary admission are cohesive collaborating owners. Locks, leases, deduplication and pending rows stay with their explicit ledger/state owners. New services receive named contracts, not panel/controller forwarding.
- G4: measurement planning/frozen input capture/geometry resolution/worker coordination/result commitment/presentation are distinct owners. Pending queues and generation routing move with their responsibilities. Core services do not import widgets or use panel state.
- G5: compatibility imports/call shapes, operator flows, geometry/clock/schema/hash semantics, app-lifetime workers and exact original-run evidence remain valid. Existing RCA fixes survive.
- G6: architecture assertions and behavior regression prove separation, independent construction/operation, single state ownership, resource visibility and no new feature cycles. Full Workstation/shared-coordinate regressions and repository gates pass or reveal explicitly unresolved evidence.

No visible redesign, algorithm replacement, dependency/framework upgrade, device parameter mutation or data migration is required. Temporary compatibility is restricted to named entrypoints/properties and must have documented exit criteria. Moving code into mixins or passing a universal panel proxy does not satisfy G2–G4.
