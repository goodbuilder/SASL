"""Runtime patches used only by the simulation launcher."""

import math
import os

import numpy as np


def _plot_window_seconds():
    try:
        return max(0.1, float(os.environ.get("RQT_PLOT_WINDOW_S", "30.0")))
    except ValueError:
        return 30.0


def _plot_y_axis_guard_fraction():
    try:
        return min(0.25, max(0.0, float(os.environ.get("RQT_PLOT_Y_GUARD_FRACTION", "0.05"))))
    except ValueError:
        return 0.05


def _patch_rqt_plot_default_topics():
    try:
        from python_qt_binding.QtCore import QTimer
        from python_qt_binding.QtWidgets import QApplication, QDockWidget
        from rqt_plot.data_plot import DataPlot
        from rqt_plot.plot import Plot
    except Exception:
        return

    original_init = Plot.__init__

    def fit_visible_y_axis(plot):
        data_plot = getattr(plot._widget, "data_plot", None)
        curves = getattr(data_plot, "_curves", {}) if data_plot is not None else {}
        if not data_plot or not curves:
            return
        try:
            x_min, x_max = data_plot.get_xlim()
        except Exception:
            return
        y_values = []
        for curve in curves.values():
            try:
                x_data = np.asarray(curve["x"], dtype=float)
                y_data = np.asarray(curve["y"], dtype=float)
            except Exception:
                continue
            if x_data.size == 0 or y_data.size == 0:
                continue
            count = min(x_data.size, y_data.size)
            x_data = x_data[:count]
            y_data = y_data[:count]
            visible = (x_data >= x_min) & (x_data <= x_max) & np.isfinite(y_data)
            if np.any(visible):
                y_values.append(y_data[visible])
        if not y_values:
            return
        values = np.concatenate(y_values)
        y_min = float(np.min(values))
        y_max = float(np.max(values))
        if not math.isfinite(y_min) or not math.isfinite(y_max):
            return
        span = y_max - y_min
        padding = max(0.1, abs(y_max) * 0.02, abs(y_min) * 0.02, span * 0.08)
        if span < 1e-6:
            padding = max(0.5, padding)
        next_min = y_min - padding
        next_max = y_max + padding
        try:
            current_min, current_max = data_plot.get_ylim()
            current_min = float(current_min)
            current_max = float(current_max)
        except Exception:
            current_min = float("nan")
            current_max = float("nan")

        should_update = True
        if math.isfinite(current_min) and math.isfinite(current_max) and current_max > current_min:
            current_span = current_max - current_min
            guard = current_span * _plot_y_axis_guard_fraction()
            next_center = (next_min + next_max) * 0.5
            current_center = (current_min + current_max) * 0.5
            should_update = (
                next_min < current_min - guard
                or next_max > current_max + guard
                or abs(next_center - current_center) > current_span * 0.35
            )
        if not should_update:
            return

        data_plot._operator_programmatic_limits = getattr(data_plot, "_operator_programmatic_limits", 0) + 1
        try:
            data_plot.set_ylim([next_min, next_max])
        except Exception:
            pass
        finally:
            data_plot._operator_programmatic_limits -= 1

    def init_with_default_topics(self, context):
        original_init(self, context)

        serial = context.serial_number()
        if serial == 1:
            axis_label = "Forward"
            topics = [
                "/ground_truth_position/data[0]",
                "/stereo/estimated_position/data[0]",
                "/stereo/corrected_position/data[0]",
            ]
        elif serial == 2:
            axis_label = "Right"
            topics = [
                "/ground_truth_position/data[1]",
                "/stereo/estimated_position/data[1]",
                "/stereo/corrected_position/data[1]",
            ]
        elif serial == 3:
            axis_label = "Down"
            topics = [
                "/ground_truth_position/data[2]",
                "/stereo/estimated_position/data[2]",
                "/stereo/corrected_position/data[2]",
            ]
        else:
            return

        def add_topics():
            if len(self._widget._rosdata.keys()) > 0:
                return
            for topic in topics:
                self._widget.add_topic(topic)
            if self._widget.data_plot is not None:
                window_s = _plot_window_seconds()
                self._widget.data_plot.set_autoscale(
                    x=False,
                    y=False,
                )
                self._widget.data_plot.set_xlim([0.0, window_s])
                try:
                    plot_widget = self._widget.data_plot._data_plot_widget._plot_widget
                    plot_widget.getPlotItem().setTitle(f"{axis_label} axis: truth vs raw vs corrected")
                    plot_widget.getPlotItem().setLabel("left", f"{axis_label} (m)")
                except Exception:
                    pass
            self._update_title()
            fit_visible_y_axis(self)
            app = QApplication.instance()
            if app is not None:
                default_dock_titles = {
                    1: ("Plot",),
                    2: ("Plot (2)",),
                    3: ("Plot (3)",),
                }
                for dock in app.allWidgets():
                    if not isinstance(dock, QDockWidget):
                        continue
                    if dock.widget() is self._widget or dock.windowTitle() in default_dock_titles.get(serial, ()):
                        dock.setWindowTitle(f"{axis_label} Axis Plot")
                        break

        QTimer.singleShot(1500, add_topics)
        y_fit_timer = QTimer(self._widget)
        y_fit_timer.timeout.connect(lambda: fit_visible_y_axis(self))
        y_fit_timer.start(500)
        self._stereo_y_fit_timer = y_fit_timer
        QTimer.singleShot(
            2000,
            lambda: globals().get("_apply_rqt_image_view_default_topics", lambda: None)(),
        )

    Plot.__init__ = init_with_default_topics


def _patch_rqt_image_view_default_topics():
    try:
        from python_qt_binding.QtCore import QTimer, Qt
        from python_qt_binding.QtWidgets import QApplication, QComboBox, QDockWidget, QLayout, QWidget
    except Exception:
        return

    def center_image_frames(attempt=0):
        app = QApplication.instance()
        if app is None:
            if attempt < 40:
                QTimer.singleShot(500, lambda: center_image_frames(attempt + 1))
            return

        applied = 0
        for dock in app.allWidgets():
            if not isinstance(dock, QDockWidget):
                continue
            if dock.windowTitle() not in ("Image View", "Image View (2)"):
                continue
            frame = dock.findChild(QWidget, "image_frame")
            layout = dock.findChild(QLayout, "image_layout")
            if frame is None or layout is None:
                continue
            layout.setAlignment(frame, Qt.AlignCenter)
            if hasattr(frame, "resizeToFitAspectRatio"):
                frame.resizeToFitAspectRatio()
            parent = frame.parentWidget()
            if parent is not None and frame.width() > 0:
                _, top, _, bottom = layout.getContentsMargins()
                spare_width = max(0, parent.width() - frame.width())
                left = spare_width // 2
                right = spare_width - left
                layout.setContentsMargins(left, top, right, bottom)
            applied += 1

        if applied < 2 and attempt < 40:
            QTimer.singleShot(500, lambda: center_image_frames(attempt + 1))
        elif applied >= 2 and attempt < 20:
            QTimer.singleShot(1000, lambda: center_image_frames(attempt + 1))

    def apply_topics(attempt=0):
        app = QApplication.instance()
        if app is None:
            if attempt < 20:
                QTimer.singleShot(250, lambda: apply_topics(attempt + 1))
            return

        mode = os.environ.get("RQT_IMAGE_TOPIC_MODE", "detection").strip().lower()
        if mode == "raw":
            topic_by_title = {
                "Image View": "/left_camera/image",
                "Image View (2)": "/right_camera/image",
            }
        else:
            topic_by_title = {
                "Image View": "/left_camera/detection_image",
                "Image View (2)": "/right_camera/detection_image",
            }
        applied = 0
        for dock in app.allWidgets():
            if not isinstance(dock, QDockWidget):
                continue
            topic = topic_by_title.get(dock.windowTitle())
            if topic is None:
                continue
            combo = dock.findChild(QComboBox, "topics_combo_box")
            if combo is None:
                continue
            index = combo.findText(topic)
            if index < 0:
                combo.addItem(topic)
                index = combo.findText(topic)
            if index >= 0 and combo.currentText() != topic:
                combo.setCurrentIndex(index)
                combo.activated[int].emit(index)
            applied += 1

        if applied < 2 and attempt < 40:
            QTimer.singleShot(500, lambda: apply_topics(attempt + 1))

    globals()["_apply_rqt_image_view_default_topics"] = apply_topics
    QTimer.singleShot(1500, center_image_frames)
    QTimer.singleShot(3500, center_image_frames)


def _patch_rqt_plot_visible_y_autoscale():
    try:
        from rqt_plot.data_plot import DataPlot
    except Exception:
        return

    original_init = DataPlot.__init__
    original_merged_autoscale = DataPlot._merged_autoscale
    original_autoscroll = DataPlot.autoscroll

    def connect_manual_limits_signal(self):
        try:
            plot_widget = self._data_plot_widget._plot_widget
            view_box = plot_widget.getPlotItem().getViewBox()
            view_box.sigRangeChangedManually.connect(
                lambda *args: disable_operator_autoscale_on_user_limits(self)
            )
            return True
        except Exception:
            return False

    def disable_operator_autoscale_on_user_limits(self):
        if getattr(self, "_operator_programmatic_limits", 0):
            return
        self._operator_view_autoscroll_enabled = False
        self._autoscroll = False
        self._autoscale_x = False
        self._autoscale_y = False

    def init_with_user_axis_override(self, *args, **kwargs):
        self._operator_programmatic_limits = 0
        self._operator_view_autoscroll_enabled = True
        original_init(self, *args, **kwargs)
        if not connect_manual_limits_signal(self):
            self.limits_changed.connect(lambda: disable_operator_autoscale_on_user_limits(self))

    def autoscroll_with_operator_reset(self, enabled=True):
        original_autoscroll(self, enabled)
        if enabled:
            self._operator_view_autoscroll_enabled = True
            self._autoscale_x = False
            self._autoscale_y = False

    def merged_autoscale_with_fixed_x_window(self):
        window_s = _plot_window_seconds()
        if (
            getattr(self, "_autoscroll", False)
            and getattr(self, "_operator_view_autoscroll_enabled", True)
            and window_s > 0.0
        ):
            self._autoscale_x = False
            latest_x = None
            for curve in getattr(self, "_curves", {}).values():
                if len(curve["x"]) > 0:
                    value = float(curve["x"].max())
                    latest_x = value if latest_x is None else max(latest_x, value)
            if latest_x is None:
                latest_x = window_s
            self._operator_programmatic_limits = getattr(self, "_operator_programmatic_limits", 0) + 1
            try:
                self.set_xlim([latest_x - window_s, latest_x])
            finally:
                self._operator_programmatic_limits -= 1
        self._operator_programmatic_limits = getattr(self, "_operator_programmatic_limits", 0) + 1
        try:
            return original_merged_autoscale(self)
        finally:
            self._operator_programmatic_limits -= 1

    DataPlot.__init__ = init_with_user_axis_override
    DataPlot.autoscroll = autoscroll_with_operator_reset
    DataPlot._merged_autoscale = merged_autoscale_with_fixed_x_window


_patch_rqt_plot_default_topics()
_patch_rqt_image_view_default_topics()
_patch_rqt_plot_visible_y_autoscale()
