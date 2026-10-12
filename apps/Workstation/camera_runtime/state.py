"""Feature bindings to one authoritative stream-readiness owner."""

from types import SimpleNamespace


def readiness_property(name):
    return property(
        lambda state: getattr(state.stream_readiness, name),
        lambda state, value: setattr(state.stream_readiness, name, value),
    )


class PresentationReadinessState(SimpleNamespace):
    _video_start_readiness = readiness_property("_video_start_readiness")
    _video_start_readiness_generation = readiness_property("_video_start_readiness_generation")

    def __init__(self, stream_readiness):
        super().__init__(stream_readiness=stream_readiness)
