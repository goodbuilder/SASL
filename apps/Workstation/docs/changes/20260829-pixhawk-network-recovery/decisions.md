# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-29 — Approved operator workflow and scope

- Use a new packet rather than extending active stability packets.
- Camera shows Camera network candidates; Pixhawk shows Pixhawk network candidates.
- A selected row assigns NIC+address+endpoint to P1 or P2; same or different NICs are allowed.
- Assign decides and persists; Connect attempts the link using existing IP/routes only.
- Do not mutate host networking and do not add a separate Apply button.
- Persist the last interface-name assignment; changed NIC names require manual reassignment.
- Reassigning a LIVE side disconnects only that side; partial P1/P2 connect is retained.
- Camera/Pixhawk same-NIC use is allowed with no UI warning or readiness gate.

## 2026-08-29 — Implementation approval

The user reviewed the decision-complete plan and explicitly requested `Implement the plan.` This is
packet-scoped authorization for the documented vertical slice only.

## 2026-08-29 — NVIDIA residual classified as external

The synthetic NVENC test and direct FFmpeg probe fail at CUDA initialization while the NVIDIA GPU
is in power-saving state. The user confirmed this is expected and will reboot to restore NVIDIA.
Do not attribute it to the Pixhawk network change or modify media code in this packet.

## 2026-08-29 — Discovery rows and side assignments are separated

- User clarification: P1 and P2 are assigned independently, and each assignment determines only
  that side's endpoint and route before Connect.
- A discovery row represents only one NIC/local-IPv4 candidate. It must not display P1 and P2 route
  or endpoint fields in the same row.
- P1 and P2 each get an independent assignment summary with State, NIC, Local IPv4, Endpoint and
  Route. Assign P1 updates only P1; Assign P2 updates only P2.
- Keep one shared candidate selection table and the two explicit Assign buttons. Connect retains
  the existing side-specific route preflight and partial-connect behavior.
