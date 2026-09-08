# Workflow

Start freezes identity before readiness work. Producers use that run context through Stop, late results and retries. Each run retains its own files; no shutdown move for new run directories. Data opens/rebuilds the latest run workbook. Outside-Measurement operations keep existing behavior. Failure remains local and visible; subsequent Measurement can start.
