# Backend contract

Frozen pair, request, outcome and bundle own nested JSON without mutable aliases. to_mapping/canonical_payload/to_record return detached ordinary dict/list values; existing canonical hashes remain compatible.

Run lifecycle remains IDLE -> QUEUED(optional) -> RUNNING -> STOPPING -> IDLE; missing storage/sensors do not gate observation. A new run has fresh generation and frozen timing/config. An old completion cannot clear a new run. UI adapters expose compatibility values but a Qt-free state owner decides transitions.

Method runtime: one process per canonical method (existing lazy comparison startup), per-method queue limit 20, existing library thread caps. No automatic compute deadline. State includes method/generation/current pair/run/start/elapsed/pending. Explicit restart is queued, validated, and executed by actor only; current work becomes FAILED with reason and original identity, queue continues, other methods continue. Duplicate or stale recovery requests are rejected/coalesced by expected worker/request identity. Close dominates recovery.

Persistent files/settings need no migration. Geometry and serialized public schemas retain their contracts. New operational evidence uses existing event/reason fields.
