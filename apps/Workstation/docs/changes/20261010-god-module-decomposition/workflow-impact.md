# Workflow impact

Preserve Start/Stop and immediate next Run, app-lifetime comparison jobs, camera/telemetry failure isolation, calibration Apply gates, recording enqueue vs durable completion, late native frame completion, manual capture/ULog, evidence recovery/export and ordered shutdown.

No operator retraining or data migration. Developer workflow changes: import actual feature owners and inject documented ports; old facade imports remain for compatibility. Tests that patch old implementation globals should patch the new owning module with equivalent observable assertions.
