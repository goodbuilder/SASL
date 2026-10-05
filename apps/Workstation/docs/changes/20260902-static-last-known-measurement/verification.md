# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence |
|---|---|
| Pre-run valid attitude/GPS can satisfy later static measurement | focused controller tests |
| Stale/disconnected per-side reuse | cache/readiness tests |
| Invalid/never-seen values still block | negative tests |
| Newer live and GPS source changes supersede/invalidate correctly | transition tests |
| Restart begins empty | fresh-controller/panel test |
| Existing raw values/timestamps unchanged and no marker added | canonical payload assertions |
| No UI geometry/control change | UI snapshot/control regression |
| Live and alternate methods preserved | existing measurement-method regression suite |

## Focused test evidence

Not run; implementation is not yet approved.

## Regression and application evidence

## Performance, load, and target-hardware evidence

## Feature isolation and contract evidence

## Visual authenticity and dynamic-layout stability evidence

## Visual and hardware/manual evidence

## Residual conditions
