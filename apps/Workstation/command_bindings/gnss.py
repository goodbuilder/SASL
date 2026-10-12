"""gnss command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_gnss(
    registry,
    *,
    gnss_base_connect,
    gnss_base_disconnect,
    gnss_base_discover,
    gnss_ntrip_forget_password,
    gnss_ntrip_start,
    gnss_ntrip_stop,
    gnss_survey_start,
    gnss_survey_stop,
) -> None:
    for command_id, text, tooltip, category, handler in (
        (
            "gnss.base_discover",
            "Find base GNSS",
            "Discover GNSS Base serial candidates through the existing owner.",
            "gnss",
            gnss_base_discover,
        ),
        (
            "gnss.base_connect",
            "Connect base",
            "Connect the configured GNSS Base serial through the existing worker owner.",
            "gnss",
            gnss_base_connect,
        ),
        (
            "gnss.base_disconnect",
            "Disconnect base",
            "Stop the configured GNSS Base serial worker.",
            "gnss",
            gnss_base_disconnect,
        ),
        (
            "gnss.survey_start",
            "Start survey",
            "Start local GNSS Base Survey-in through the existing UBX command path.",
            "gnss",
            gnss_survey_start,
        ),
        (
            "gnss.survey_stop",
            "Stop survey",
            "Stop local GNSS Base Survey-in through the existing UBX command path.",
            "gnss",
            gnss_survey_stop,
        ),
        (
            "gnss.ntrip_start",
            "Start NTRIP",
            "Start the configured NTRIP operation through the existing GNSS owner.",
            "gnss",
            gnss_ntrip_start,
        ),
        (
            "gnss.ntrip_stop",
            "Stop NTRIP",
            "Stop the existing NTRIP operation and its correction path.",
            "gnss",
            gnss_ntrip_stop,
        ),
        (
            "gnss.ntrip_forget_password",
            "Forget NTRIP password",
            "Remove the saved NTRIP password from the OS secure credential store.",
            "gnss",
            gnss_ntrip_forget_password,
        ),
    ):
        registry.register(CommandSpec(command_id, text, tooltip, category, handler))
