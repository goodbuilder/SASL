# Decisions

2026-09-08 user: “ECEF는 빼고 Baseline center를 원점으로 하는, 수평 F/R/D를 추가하는 방식이 좋을 것 같아”. Authorizes the previously proposed additive coordinates, preserving original and WGS84. Implementation choice: shared frozen P1 forward projection defines heading, with zero camera/Pixhawk mount offset as agreed. No additional approval for the same accepted action.

2026-09-08: User requested planning before implementation, then explicitly approved the final plan with “승인.”. Final scope preserves each Method origin (not a common GNSS origin), adds local-level fields and evidence to JSONL/CSV/XLSX, retains existing live UI/WGS84, and leaves historical files unchanged. Source implementation began only after this explicit approval.

Implementation complete: new fields/evidence are additive, raw pair-frozen P1 defines common axes, each Method midpoint is preserved, and no ECEF or live layout changes were introduced. Existing independent WGS84 UI work in the shared worktree was preserved and regression-tested.
