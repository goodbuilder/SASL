# AGENTS.md - App Architecture Rules

These rules apply to applications and packages beneath `apps/`, in addition to the repository-level `AGENTS.md`.

## Module Responsibility Boundaries

Before a change spans modules or materially changes a module's responsibility:

- Identify the capability owner, authoritative state or resource owner, public contract, and allowed dependency direction from code, documentation, and tests.
- Keep code together when it serves one coherent responsibility. Split it when separate reasons to change, contracts, or state/resource lifecycles justify a boundary; do not split by line count alone.
- Do not route domain decisions or state through a broad window, manager, or generic utility just for convenience. Keep coordinators focused on sequencing and use narrow, purpose-specific interfaces.
- Before moving code, inspect its callers and runtime access paths, including dynamic lookups and callbacks. Move ownership and state with the behavior; keep compatibility facades at the outer edge.
- For material boundary changes, update the relevant app architecture documentation when present, and record focused evidence that the boundary holds. Prefer checks for allowed imports, dependency cycles, or public contracts when they fit the change. Do not expand a local change into an unrelated architecture rewrite.
