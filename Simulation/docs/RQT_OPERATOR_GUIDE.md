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
