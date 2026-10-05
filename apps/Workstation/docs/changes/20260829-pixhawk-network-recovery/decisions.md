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

## 2026-08-29 — Explicit host-network role application reopens approval

- User clarification: a physical NIC has no permanent Camera or Pixhawk role; cabling and roles may
  change between operations.
- User decision: Workstation must explicitly apply the selected role's host NetworkManager address
  and routes instead of only saving intent and reporting an externally configured route.
- Application must follow a visible operator action and confirmation. Discovery never guesses a
  role and never mutates networking by itself.
- This supersedes only the earlier `Do not mutate host networking and do not add a separate Apply
  button` decision. PX4 firmware/parameter/SD-card mutation remains excluded, and expected-sysid,
  liveness, partial-connect and recording locks remain preserved.
- The previous read-only implementation approval does not authorize this mutation. Source work is
  paused pending a revised failure/rollback contract and amended approval.

## 2026-08-29 — NIC roles are composable, not exclusive

- User decision: one physical NIC may simultaneously carry Camera and Pixhawk traffic.
- Applying Camera to a NIC that already owns P1/P2 must preserve every Pixhawk-owned address/route;
  applying P1/P2 to a Camera NIC must preserve every Camera-owned address/route.
- The desired host state is the union of active role requirements. Shared entries are removed only
  after no remaining role on that NIC requires them; unrelated operator/system configuration is
  never owned or removed by Workstation.
- Same-NIC use remains permitted without a topology warning or readiness gate. Its historical
  long-load stability limitation remains evidence, not an application prohibition.
- The open rollback decision is now role-scoped: moving one role must not interrupt another role
  that remains on the source NIC.

## 2026-08-29 — Verify target before role-scoped source cleanup

- User decision: moving one role from NIC A to NIC B first applies and independently verifies that
  role on B without changing A.
- Only after B passes may Workstation remove the moved role's app-owned entries from A. Camera/P1/P2
  roles that remain on A, shared entries they still require, and unrelated network settings remain.
- If target application fails, A remains exactly as it was. If source cleanup fails, Workstation
  attempts to roll back the target role and verify the original arrangement; an incomplete rollback
  is a visible fail-closed partial state requiring operator recovery, never silent success.
- Assignment persistence changes only after the complete move postcondition is verified.

## 2026-08-29 — Physical NIC count and topology are runtime-variable

- User clarification: the field setup may use one NIC or multiple NICs; the application must not
  require the operator to choose or configure a fixed NIC count in advance.
- Discovery inventories every current physical wired NIC. Each NIC owns a set drawn from
  `Camera`, `P1` and `P2`; the same role cannot be active on two NICs after a completed move, while
  one NIC may own every role.
- One-NIC combined, two-NIC split/swapped, three-NIC separated, missing-NIC and replacement-name
  topologies are normal runtime states, not separate product modes.
- Role application and safety gates are scoped only to the source/target NICs whose app-owned role
  set changes. Unaffected NICs and their devices continue independently.

## 2026-08-29 — Active devices follow the operator-selected NIC

- User decision: a healthy Camera or Pixhawk connection does not block an explicit NIC change.
  Workstation moves the requested role to the operator-selected NIC and restores operation there.
- Before host mutation, Workstation snapshots every active Camera/P1/P2 owner on the source/target
  NICs. It stops only owners that can be affected; devices on unrelated NICs continue.
- After verified role application, Workstation restarts exactly the owners that were active before
  the change. It never starts a previously stopped Preview or disconnected Pixhawk.
- Camera reconnection requires a current-generation decoded frame. Pixhawk reconnection requires
  the expected system ID with fresh HEARTBEAT and ATTITUDE; its existing 60-second qualification
  continues visibly after reacquisition.
- If new-NIC operation cannot be proven, Workstation rolls back the role and attempts to restore
  the previous active owners. Incomplete rollback is explicit partial failure, never success.
- Existing Measurement/recording/save/finalization mapping locks remain: no NIC role changes inside
  an active evidence interval.

## 2026-08-29 — Network role amendment implementation approved

The user reviewed the revised `approval-summary.md` and explicitly approved it with `응` at
2026-08-29T16:42:26+09:00. This authorizes only the documented explicit NetworkManager role
composition/move, affected-owner automatic reconnect/rollback, UI, persistence, tests and product
documentation. All listed exclusions and existing unrelated dirty-worktree ownership remain.

## 2026-08-30 — Missing device evidence restores the previous NIC

- User decision: when host address/route application succeeds but the required device cannot be
  proven on the new arrangement, Workstation automatically restores the previous NIC assignment.
- The proposed durable `ASSIGNED · DEVICE NOT SEEN` outcome is rejected. `DEVICE NOT SEEN` is a
  failure/rollback reason, not a committed assignment state.
- Assignment persistence remains commit-last. A successful rollback restores the previous mapping
  and previously running owners; an incomplete rollback remains an explicit blocking partial state.
- The exact required-owner set is still open: the field failure showed that an unrelated Pixhawk
  that was already offline before the move can currently veto the selected role's otherwise valid
  move.

## 2026-08-30 — Pre-existing offline owners do not veto another role

- User decision: when moving P1, a P2 that was already offline before the click does not cause P1
  rollback; the same rule applies symmetrically to Camera/P1/P2.
- Required reconnect evidence is scoped to the selected role and other affected owners that were
  healthy immediately before mutation. An open socket, configured endpoint or worker without fresh
  device evidence is not a healthy owner.
- Camera pre-health requires a current-generation frame. Pixhawk pre-health requires the expected
  system ID with fresh HEARTBEAT and ATTITUDE. Owners already offline keep their prior offline state
  and are neither auto-started nor used as a rollback oracle.
- If a previously healthy required owner fails to recover, the previous NIC is automatically
  restored according to the preceding decision.
