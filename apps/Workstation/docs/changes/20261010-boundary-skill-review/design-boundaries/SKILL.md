---
name: design-boundaries
description: Design responsibility, dependency, state, and lifecycle boundaries for new systems, substantial features, or scoped architectural redesigns. Use when multiple capabilities or resource owners must collaborate; do not trigger for routine local fixes or split modules solely by line count.
---

# Design Boundaries

Design boundaries that make expected changes local and ownership explicit. Optimize for coherent
behavior and independently testable contracts, not the number or size of files. This skill proposes
architecture; it does not authorize implementation, migrations, device actions, or a wider rewrite.

## Establish the scope

Reuse the user's requirements and any active create-gui/modify-gui or other task packet. Do not
repeat interviews, approval requests, or documentation. If a consequential outcome is ambiguous,
clarify it while continuing independent inspection. Respect explicit user instructions and the
active workflow's applicable approval rules; add no new blanket approval gate.

Inspect actual producers, consumers, state access, and resource lifecycles before extending an
existing system. Separate observed current structure, historical defects, and proposed structure.
Do not claim a missing skill caused an architectural problem.

## Design decisions

1. **Change scenarios and responsibilities.** Use representative future changes and failures to
   decide boundaries. For each capability, record its purpose, decisions it owns, and exclusions.
   A controller or manager is not a responsibility definition. Split genuinely different reasons
   to change, such as transport acquisition, domain policy, artifact persistence, and presentation,
   when they have independent lifecycles or contracts. Do not manufacture layers for a small feature.
2. **Ownership and mutation.** Name one authoritative owner for each mutable state or resource,
   its allowed writers, readers, and update operations. Prefer intent-level commands and snapshots
   to granting collaborators broad field mutation. Read-only access to a mutable object is not
   automatically immutable; specify copying, freezing, or an explicit live-view contract.
3. **Semantic interfaces.** Define required capabilities, payloads, outcomes, and errors. Avoid
   passing an entire app/panel/context to domain owners or replacing it with hundreds of generic
   getter/setter names. A finite interface still needs a coherent purpose. Separate mandatory
   capabilities from optional ones and specify observable degradation when an optional one is absent.
   Mandatory wiring failures must surface at composition or before the affected operation, not
   silently become no-ops. Framework parent/context handles may be passed for their narrow role.
4. **Dependency direction and composition.** Put construction and wiring in a composition root.
   Business policy belongs to feature owners, not that root. Coordinators sequence collaborators
   but do not accumulate their algorithms, widget logic, transport details, or persistence policy.
   Keep compatibility facades at the outer edge; shared contracts must not import those facades.
   Keep domain-specific policies out of generic utilities. Avoid speculative plugin frameworks.
5. **Time and identity.** For asynchronous work, distinguish application, session, request, and
   worker lifetimes. Decide when configuration and input values become fixed, what remains live,
   and how source time, generation, request identity, and schema/frame information cross boundaries.
   Define admission of late, duplicate, cancelled, and previous-session results. Related writes
   needing atomicity share a transaction/ledger owner rather than independent competing state.
6. **Lifecycle and execution context.** Specify create/start/stop/cancel/drain/join/dispose owners,
   callback execution threads, and partial-start failure cleanup. Shutdown must discover resources
   through an explicit contract, including nested and dynamically created resources, rather than
   depending only on host attributes or naming suffixes. One business owner need not equal the
   framework object parent, execution thread, or shutdown aggregator; distinguish these roles.
7. **Compatibility and migration.** Plan incremental migration without duplicating authoritative
   state. For each facade or adapter, identify supported consumers, permitted behavior, who must
   migrate, and a checkable exit condition. Do not make legacy aliases the default API for new code.

## Challenge the proposed design

Use a few relevant scenarios: replace one input adapter, add a method, disable a capability, change
settings during a running request, stop during persistence, or receive an old result after restart.
For each, identify the owners and contracts that must change. If unrelated feature internals must
be edited, reconsider the boundary. Shared schema changes may legitimately affect several owners;
explain those cases instead of requiring an arbitrary maximum file count.

Identify enforcement suited to the design: allowed import directions, a dependency cycle check,
required-capability composition checks, immutable-input checks, owner identity, lifecycle and
generation tests. Include a prohibited case that the check should reject. Static imports alone do
not establish runtime isolation when callbacks or dynamic lookup create hidden dependencies.

Use line counts, method counts, dependency breadth, and change history as investigation signals.
Exclude generated/vendor code from human-maintenance rankings. A large cohesive view or algorithm
may remain intact. Do not promise performance, reliability, or independence based on decomposition
alone; weigh wiring, adapter, public API, and compatibility maintenance costs as well.

## Output and handoff

Update the existing design/task record with only the material decisions:

```text
Outcome and scope:
Capability -> responsibility -> owned state/resources -> permitted writers:
Public operations/snapshots, required/optional behavior, dependency direction:
Lifetime, snapshot boundary, identity and failure handling:
Composition, migration/compatibility exit conditions:
Change scenarios and enforcement evidence planned:
Tradeoffs and unresolved decisions:
```

Hand the accepted design to verify-check for implementation slices and end-to-end verification.
Maintain-boundaries audits later changes against these decisions. Use rca when explaining an
observed recurring failure requires causal analysis. Neither a diagram nor an interface declaration
proves the implementation follows the design.
