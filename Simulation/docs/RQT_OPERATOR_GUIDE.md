# RQT operator view

Run the existing operator entry point with `./run_operator_view.sh`. The
existing `stereo_ground_sim.operator_panel.OperatorPanel` remains the primary
view. Pair diagnostics are operator visualization only, not qualification
evidence, and use completed evaluated pairs in `baseline_center_frd` (metres;
angles in degrees). Missing values are omitted rather than replaced by zero.

Simulation World uses ENU; camera optical vectors use RDF; public measurement
vectors use fixed `baseline_center_frd` (Forward, Right, Down), with origin at
the nominal optical-center midpoint and quaternion metadata `xyzw`.

Do not use interactive operator parameters as deterministic sweep, freeze, or
sealed-evaluation inputs. Atomic sealed execution and the sealed runner remain
R4-B work.

This phase provides only a pure terminal-record adapter. Runtime publishers,
DiagnosticArray, panel integration, launcher flags, and parameter safety are
not connected yet. Runtime pair IDs are positive integers. The adapter
compares the `StereoImagePair` identity with the identity copied into the
assembled terminal record; `GroundTruthAlignment` itself does not carry that
identity. Only evaluated/processed/evaluated records are accepted. Missing or
non-finite individual metrics are omitted, but a snapshot with no finite
metrics is not allowed. Values are immutable tuple-of-tuples. Operator
diagnostics are not qualification evidence. The prior force-with-lease Git
procedure violation remains recorded separately.
