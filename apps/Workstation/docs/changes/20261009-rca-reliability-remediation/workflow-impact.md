# Workflow impact

W1: Start/Stop/restart measurement keeps current observation-first behavior. Run state is now independently testable; storage/sensor failure and delayed prior-run completion remain isolated.
W2: Records and exports keep original input identity and canonical numeric values even if callers later edit mappings. No operator migration.
W3: Data Status gains observable method execution and explicit selected worker restart. Operator can terminate a suspected unresponsive computation while remaining methods and collection continue; original failed work stays in evidence. Normal Stop retains pending work.
W4: Developers run the CI command against current test directories; historical provenance is classified without weakening active path checks.
