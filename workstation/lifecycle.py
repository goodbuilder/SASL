"""Compatibility import for callers that used the pre-separation path.

The implementation lives in :mod:`sasl_common.lifecycle`; application
owners import that neutral module directly so Legacy and Workstation do not
depend on one another.
"""

from sasl_common.lifecycle import LifecycleCoordinator, LifecycleStep

__all__ = ["LifecycleCoordinator", "LifecycleStep"]
