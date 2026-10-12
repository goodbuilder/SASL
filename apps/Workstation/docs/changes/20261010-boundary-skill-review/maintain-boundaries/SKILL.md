---
name: maintain-boundaries
description: Keep responsibility, dependency, state, and lifecycle boundaries intact while extending or refactoring existing code. Use for cross-module changes, shared resource or contract changes, and edits to highly coupled controllers; do not require architecture reviews for trivial local edits.
---

# Maintain Boundaries

Keep new behavior with its responsible owner and avoid increasing unrelated coupling. Improve
only the boundaries needed by the requested change. Reducing a file's size is not proof of a
successful decomposition.

## Establish the actual baseline

Reuse active requirements, design decisions, approval scope, and verification records from the
controlling workflow. Do not create a second interview, approval gate, or report packet. Preserve
unrelated worktree edits. Record the relevant source identity so historical test results are not
presented as fresh verification of changed files.

Inspect both source and runtime-facing contracts: callers/imports, direct and dynamic attribute
access, callbacks, state writes, optional lookups, workers/resources, and public adapters. Distinguish
generated code, cohesive large modules, thin compatibility facades, and actual responsibility
concentration. Document suspected risks separately from reproduced defects and already-fixed history.

## Choose where the change belongs

- Identify which capability owns the new decision, state, resource, or operation before editing.
  Do not default to the main window, manager, common controller, or shared utilities because it is
  convenient. If ownership changes, name the old and new owner and the migration boundary.
- Compare the target owner's existing responsibility with the new behavior. An independent reason
  to change or a different lifetime may require a new owner. A small cohesive addition need not.
- Review new reads, writes, calls, and dependencies. Broad app/context references, hundreds of
  field ports, mutable values exposed through read-only getters, and repeated access to another
  feature's internals are review signals, not automatic proof of a bug or a numeric rejection rule.
- For a material redesign, use design-boundaries if available or document the necessary boundary
  decisions locally. Do not infer authority for an unrelated rewrite, new framework, or migration.

## Preserve behavior while changing structure

Before moving code, inventory the affected mandatory/optional calls and the old observable outcomes.
Include dynamic getattr/setattr paths and conditional callbacks; import analysis alone misses them.
At composition or operation admission, detect missing mandatory capabilities. Optional absence must
have an explicit tested result; compatibility with a lightweight fake must not silently disable a
required production operation.

Move responsibility and authoritative state together rather than adding helpers that still read
or write the entire old host. Preserve object identity where callbacks, locks, transactions, leases,
or saved-session generations depend on it. Do not copy a transaction ledger or state merely to make
two new services appear independent. Name permitted writers and prefer explicit operations.

Preserve temporal contracts: input capture time, selected method/fallback/hysteresis policy, source
clock, snapshot depth, generation, and destination of late results. A pure-function extraction must
not substitute a new selection policy or query newer live configuration. Separate behavior changes
from mechanical movement so each can be reviewed and validated against its own acceptance criteria.

For moved asynchronous or GUI callbacks, check execution thread, receiver lifetime, cancellation,
and shutdown. Transfer resource registration alongside ownership. Ensure nested and dynamic workers,
queues, native handles, and pending persistence remain discoverable even if absent from vars(host).
Framework parentage alone is not a business ownership or shutdown contract.

Keep facade migration one-way: new internals call actual owners or stable contracts, not the legacy
facade. Record supported compatibility consumers and executable removal conditions. Do not expand a
shim into a second implementation or move policies into composition/wiring code.

## Verify structural and behavioral effects

Use verify-check for focused and full-workflow verification. Reuse its ledger and evidence.
Select checks relevant to the changed boundary:

- allowed imports and cycles, including a deliberately forbidden fixture that must be rejected;
- real composition with required capabilities present and missing, not only permissive mocks;
- authoritative owner identity, writable permissions, and cross-feature failure isolation;
- input mutation after capture, late/duplicate completion across generations, and policy parity;
- resource inventory and teardown after partial start, cancellation, and failed assertions;
- preserved public behavior, data schemas, coordinate frames/units, and UI flows where applicable.

Keep test helpers' callbacks and resources alive for their required lifetime and clean up in failure
paths. Observe asynchronous errors and teardown, not just a process exit code. Isolate unintended
device/network side effects; hardware qualification requires the appropriate task authorization.
Do not weaken existing assertions or expand allowlists solely to make an extraction pass.

Report before/after responsibility, ownership, dependency and interface changes, and wiring costs.
File size reduction is supporting information only. Keep known debt explicit; do not demand a
repository-wide cleanup. Add a targeted guard against the prohibited dependency or ownership
regression when practical. An already-passing static gate cannot certify untested runtime paths.

## Completion

A boundary change is ready only when its owner is explicit, required behavior is preserved or
intentionally changed within scope, relevant checks pass, and any compatibility residue has clear
limits. Name unverified hardware/runtime conditions and remaining debt. Use rca for unclear or
repeated failures; return to the affected verification after the authorized fix.
