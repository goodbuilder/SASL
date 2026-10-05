"""rqt entry point for PoseSimulation."""

from __future__ import annotations

from qt_gui.plugin import Plugin

from .pose_simulation_widget import PoseSimulationWidget


class PoseSimulationPlugin(Plugin):
    """Expose the diagnostic-only PoseSimulation widget in rqt."""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("PoseSimulation")
        self._widget = PoseSimulationWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(f"PoseSimulation ({context.serial_number()})")
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()


__all__ = ["PoseSimulationPlugin"]
