"""Retry ownership for one bundle bound to its original row lease."""

from dataclasses import dataclass
from Workstation.workstation_core.method_evaluation import MethodEvaluationBundle


@dataclass
class PendingMethodBundleCommit:
    bundle: MethodEvaluationBundle
    row_owner: object | None = None
    row_index: int | None = None
    row_lease: tuple[int, int] | None = None
