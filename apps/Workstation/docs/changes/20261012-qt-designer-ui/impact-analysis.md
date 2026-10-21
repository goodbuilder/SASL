# Ownership and impact

Forms own static layout only. ui/designer.py owns package-relative loading; no settings or backend dependencies. Workspaces retain field maps, callbacks, models and custom view lifecycle. Main window keeps navigation, dock persistence, command registry and shutdown sequencing. Backend, workers, resources, schemas, dependency versions untouched by this migration.

Coordinates reviewed before edits: public measurement baseline_center_frd [forward,right,down,range]; camera optical RDF [right,down,forward]; translation left-camera RDF; raw GPS baseline NED [north,east,down]; attitude PX4/body FRD; GNSS midpoint local-level FRD is distinct from optical baseline center. Source: workstation_core/coordinate_contract.py, sasl_common.stereo_geometry and existing view producers. No transform, array ordering or CSV field changes. Preserve labels and existing rendering functions verbatim.

Rollback: revert only migration diffs; no persisted-state migration. Assets stay ui/forms within Workstation and resolve from __file__ (not checkout path or cwd). Existing QSettings dock names must be preserved.
