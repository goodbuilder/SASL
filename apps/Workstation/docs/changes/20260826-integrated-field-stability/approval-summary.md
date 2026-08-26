# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

This change makes the existing Workstation workflow truthful and durable under the real combined
camera/Pixhawk/NTRIP/recording load. It preserves camera/PX4 firmware and settings unless a later
explicit approval names one change. Existing evidence is preserved byte-for-byte.

## Operator workflow before and after

Before, individual paths could appear connected while media or telemetry had stopped, and old
capture/storage state could block new field work. After, each boundary qualifies independently,
failure is isolated, and a final 60-minute run starts only after short gates pass.

## Success, delay, rejection, failure, overload, and recovery

Every command needs an external postcondition: decoded frame, inbound MAVLink, HTTP/RTCM bytes,
durable file/hash or released resource. Busy work is queued only where already approved. Failure on
one side must not silently continue injection or claim readiness. A failed short gate ends that
candidate immediately; it does not consume the full 60 minutes.

## Data and compatibility effects

No existing Session, spool, calibration or evidence is migrated, rewritten or deleted. New records
retain exact FrameKey/hash/timestamp provenance and append-only behavior.

## Performance priorities and allowed degradation

GUI controls, Stop/Error and evidence integrity outrank preview freshness. Preview may coalesce to
latest-only within bounded queues. Silent corruption, unbounded memory, timestamp-nearest evidence
substitution and unlimited inference are not allowed. Auto CPU fallback remains visible and safe.

## Visual and interaction changes, including dynamic-layout stability

Reuse existing Camera, GNSS, Data and Diagnostics status surfaces. No new global dashboard or
decorative UI is approved. Dynamic states must not resize controls, steal focus, reset selection or
jump scroll.

## How acceptance will be demonstrated

Focused tests, full regressions, real entry points, problem-specific 10-15 minute qualifications,
three 10-minute cold boots, failure injection and a final 60-minute full-load soak. Evidence includes
pcap/RTCP, MAVLink ages/rates, NTRIP HTTP/RTCM, MP4 strict decode, queues/RSS/GPU, and FrameKey/hash.

## Cost, risks, assumptions, rollback, and field limits

Camera RTP omission/real-IDR absence or PX4-side UDP deadlock may require vendor/firmware work. In
that case app mitigation and the external blocker remain distinct and the packet cannot be closed.
Rollback is per smallest implementation slice and must not restore silent success.

## Exact scope awaiting approval

Approved by the user on 2026-08-26: the complete integrated field-stability goal described in this
packet, using problem-specific early-exit gates and one final 60-minute soak. Excluded are app-to-app
rig ownership, Diagnostics log filtering, unapproved device configuration/firmware changes, and
coordinate/calibration algorithm changes.
