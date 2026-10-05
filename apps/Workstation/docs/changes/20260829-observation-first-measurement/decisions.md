# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-29 — approved

- Observation availability outranks post-capture integrity.
- Start immediately with zero cameras; recovered sources join the same run.
- Camera recovery is manual; busy predecessor release is automatic.
- Camera rejoin records a new MP4 segment.
- All acquisition inputs may connect/recover during Measurement; mutations remain locked.
- Buffer only essential structured evidence with strict bounds.
- App shows SAVED by file existence only; OS owns later inspection.
- No catalog, revalidation, package or Session workflow.

## 2026-09-02 — approved-contract clarification from real-device evidence

- A camera start preflight rejection that occurs before any worker replacement must not stop or
  clear an already-live Preview.
- Explicit NVIDIA mode remains fail-closed when `nvh264dec` is unavailable; the correction does
  not silently change the operator's decoder selection or add an automatic fallback.
- The observation interval remains RUNNING with its Measurement camera/recording sources marked
  unavailable. The operator can stop the interval, select Auto or Software, and retry while the
  original Preview stays visible.
