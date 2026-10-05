from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from Workstation.tests.workstation_test_support import _FakeBackend
from Workstation.main_window import WorkstationMainWindow


def _window_with_isolated_ui_settings(tmp_path):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.status_adapter.stop()
    window._ui_settings = QtCore.QSettings(
        str(tmp_path / "workstation-ui.ini"), QtCore.QSettings.IniFormat
    )
    window._ui_settings.clear()
    window._ui_settings.sync()
    window._restore_ui_state()
    window.show()
    app.processEvents()
    return app, window


def _close_window(app, window) -> None:
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_hidden_navigation_reveals_at_window_edge_and_hides_after_exit(tmp_path) -> None:
    app, window = _window_with_isolated_ui_settings(tmp_path)
    window.navigation_dock.hide()
    app.processEvents()
    assert not window._navigation_pinned_visible

    edge = window.mapToGlobal(QtCore.QPoint(2, window.height() // 2))
    window._poll_navigation_edge_reveal(edge, active=True)
    assert window._navigation_auto_revealed
    assert window.navigation_dock.isVisible()

    inside = window.navigation_dock.mapToGlobal(QtCore.QPoint(16, 24))
    outside = window.mapToGlobal(
        QtCore.QPoint(window.width() // 2, window.height() // 2)
    )
    window._poll_navigation_edge_reveal(outside, active=True)
    assert window._navigation_auto_hide_timer.isActive()
    assert window._navigation_auto_hide_timer.interval() == 400

    window._poll_navigation_edge_reveal(inside, active=True)
    assert not window._navigation_auto_hide_timer.isActive()
    window._poll_navigation_edge_reveal(outside, active=True)
    window._finish_navigation_auto_hide(outside)
    assert not window._navigation_auto_revealed
    assert window.navigation_dock.isHidden()

    _close_window(app, window)


def test_saved_visible_navigation_stays_pinned_at_edge(tmp_path) -> None:
    app, window = _window_with_isolated_ui_settings(tmp_path)
    edge = window.mapToGlobal(QtCore.QPoint(2, window.height() // 2))

    assert window._navigation_pinned_visible
    window._poll_navigation_edge_reveal(edge, active=True)
    assert window.navigation_dock.isVisible()
    assert not window._navigation_auto_revealed

    _close_window(app, window)


def test_temporary_reveal_does_not_persist_as_visible(tmp_path) -> None:
    app, window = _window_with_isolated_ui_settings(tmp_path)
    window.navigation_dock.hide()
    app.processEvents()
    edge = window.mapToGlobal(QtCore.QPoint(2, window.height() // 2))
    window._poll_navigation_edge_reveal(edge, active=True)
    assert window._navigation_auto_revealed

    window._save_ui_state()
    assert window.navigation_dock.isVisible()
    assert window._ui_settings.value("dock_state")

    window._restore_ui_state()
    assert window.navigation_dock.isHidden()
    assert not window._navigation_pinned_visible

    _close_window(app, window)
