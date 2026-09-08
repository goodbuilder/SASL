# Ownership and coordinates

Run controller owns immutable click identity; registry owns run records; one bounded raw writer routes by run+stream; workbook worker owns atomic exports. No sensor/protocol/math redesign. Canonical transform X_right = R_right_from_left * X_left + T in optical RDF, metres; kernel uses R transpose and C=-R^T T. Public results baseline_center_frd, telemetry NED remain distinct. Relative paths stay under the configured data root. No historical migration, dependency replacement, or unrelated cleanup.
