# Plan

S0: capture current inventory, contracts and ownership; record authorization and acceptance.
S1 (independent): split workspace screens/components into an acyclic package; explicit import compatibility and direct production imports; isolated-import/UI/coordinate regression.
S2 (independent): split recording into typed state/ledger, field builders, native evidence, primary persistence, capture, lifecycle/save and Qt presentation; narrow adapters at the old boundary; test frozen records, late generations, admission/rollback/close.
S3 (independent): split measurement into projections/planner, pair capture, geometry resolver, result commit owner, worker/runtime adapter and Qt rendering; typed frozen inputs and narrow ports; test method fanout, deduplication/retry, old-run routing and geometry.
S4 (independent with integration dependencies): separate panel application composition, feature state/resources, shell/view bundles and feature behaviors. Replace private panel storage with named compatibility properties; explicit resource inventory replaces reliance on vars(panel). Keep old entrypoints only at the outer boundary.
S5: integrate all owners, resolve cross-feature dependencies and obsolete monkeypatch targets, verify no hidden universal facade reappears. Update current architecture/product docs and README.
S6: full regression, independent architecture checks, portability/repository policy, rendering/entrypoint observations, before/after ownership/size report and completion audit G1–G6.

Each independent branch records focused PASS/FAIL before dependent integration; integration is not accepted until all branches satisfy the ownership criteria. Root coordinates shared files and final verification. Pure file moves are allowed for cohesive views but are insufficient for stateful controllers.

Rollback is source-only: preserve schemas, saved evidence and device settings; revert only this task's changes relative to captured baseline. No destructive git reset or mass cleanup.
