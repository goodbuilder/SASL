# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

One latest-Measurement XLSX is added. Fixed JSONL, videos/images/ULogs, measurement math,
device behavior, immediate next-run admission and legacy files stay unchanged.

## Operator workflow before and after

Before, the operator joins JSONL externally. After, stopping Measurement queues a background
XLSX update; Data Status can update again or open the completed file.

## Success, delay, rejection, failure, overload, and recovery

Success is a verified workbook for the target generation. Waiting/generating/stale/failed are
visible. Missing data is `#N/A`; failure preserves the prior file; Update retries. New recording
and shutdown take priority and cancel/defer export.

## Data and compatibility effects

The file is derived, latest-only and regenerable. It does not migrate/rewrite JSONL or touch old
`review.xlsx`. Foreign same-name files are protected from overwrite.

## Performance priorities and allowed degradation

Measurement, Preview, devices and recording always outrank XLSX. Only one idle-window CPU/disk
worker is allowed; intermediate generations may be coalesced because no XLSX archive was requested.

## Visual and interaction changes, including dynamic-layout stability

Two native Data Status buttons and one fixed-height textual status line are added. No navigation,
modal, dashboard styling, focus movement or live geometry resizing.

## How acceptance will be demonstrated

Independent XLSX XML/ZIP checks, exact generation/row/timestamp fixtures, partial/failure/atomic
replacement tests, stop/next-run and shutdown tests, UI state/geometry tests, load and full suite.

## Cost, risks, assumptions, rollback, and field limits

Large runs require sheet partitioning and may take time. Excel is not authoritative. Rollback is
feature removal; source data remains. Hardware verification is not required for this file/UI slice.

## Exact scope awaiting approval

Approved scope: the complete `latest_measurement.xlsx` vertical slice documented in this packet.
Approval recorded at `2026-08-31T16:15:22+09:00`; implementation is authorized.
