# Ownership and dependencies

| Boundary | Before | After | Required proof |
|---|---|---|---|
| Workspace | 23 screen/component classes, common globals | feature screen modules + small shared leaves | isolated import, DAG, class/alias compatibility |
| Panel | one QWidget stores feature data/views/resources | composition owner + feature state and view bundles | single state location, two-instance isolation, explicit resource enumeration |
| Recording | lifecycle, native/primary ledger, records and UI in one controller | cohesive owners, shared explicit admission ledger, narrow side-effect ports | no duplicate/lost rows, original row+lease, bounded shutdown |
| Measurement | capture/plan/geometry/commit/render coupled | frozen DTOs, pure services, commit/runtime owners, Qt adapter | headless services, identical fields/hashes, late-run isolation |

Device/CPU/GPU topology, queue capacities, parent-death behavior and library thread limits remain unchanged. Ownership moves must not add polling or duplicate consumers. Existing worker lifecycle remains application/run/view specific as currently documented.

Coordinates reviewed: world/GNSS NED [north,east,down]; PX4 quaternion wxyz body FRD→NED; camera optical RDF [right,down,forward], pixels u-right/v-down; canonical X_right=R X_left+T, camera center C=-R^T T; public optical baseline-center FRD [forward,right,down,range], metres; GNSS local-level midpoint is separate; WGS84 degrees and ellipsoid/MSL heights remain distinct. No generic XYZ alias or formula change is introduced.
