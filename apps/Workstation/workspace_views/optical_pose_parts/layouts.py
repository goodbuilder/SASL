"""Qt sizing policies for frozen native previews and current method pages."""

from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.ui.annotation_image_view import AnnotationImageView


class _FrozenPreviewColumn(QtWidgets.QWidget):
    """One full-width camera preview whose image child stays at 16:9."""

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Preferred,
        )

    def hasHeightForWidth(self) -> bool:  # noqa: N802 - Qt override.
        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 - Qt override.
        content_width = (
            max(
                0,
                int(width) - self.layout().contentsMargins().left() - self.layout().contentsMargins().right(),
            )
            if self.layout() is not None
            else max(0, int(width))
        )
        view = self.findChild(AnnotationImageView)
        image_height = view.heightForWidth(content_width) if view is not None else int(round(content_width * 9 / 16))
        label = self.findChild(QtWidgets.QLabel)
        label_height = label.sizeHint().height() if label is not None else 0
        spacing = self.layout().spacing() if self.layout() is not None else 0
        margins = self.layout().contentsMargins() if self.layout() is not None else QtCore.QMargins()
        return image_height + label_height + spacing + margins.top() + margins.bottom()

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt override.
        hint = super().sizeHint()
        if hint.width() > 0:
            hint.setHeight(self.heightForWidth(hint.width()))
        return hint

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt API.
        desired_height = self.heightForWidth(self.width())
        if self.minimumHeight() != desired_height:
            self.setMinimumHeight(desired_height)
        super().resizeEvent(event)


class _CurrentPageTabWidget(QtWidgets.QTabWidget):
    """Size to the current tab's page instead of the tallest page.

    QTabWidget's internal QStackedWidget normally reports the size hint of
    its largest page for every tab, so switching to a short page (e.g. a
    single note + button) leaves most of that page blank. Following only
    the current page's hint trades that wasted space for a height that
    shifts when the active tab changes.
    """

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.currentChanged.connect(lambda _index: self.updateGeometry())

    def _current_page_size(self) -> QtCore.QSize:
        current = self.currentWidget()
        page_size = current.sizeHint() if current is not None else QtCore.QSize(0, 0)
        tab_bar = self.tabBar()
        tab_bar_height = tab_bar.sizeHint().height() if tab_bar is not None else 0
        frame_width = self.style().pixelMetric(QtWidgets.QStyle.PM_DefaultFrameWidth, None, self)
        return QtCore.QSize(
            page_size.width() + 2 * frame_width,
            page_size.height() + tab_bar_height + 2 * frame_width,
        )

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt API.
        return self._current_page_size()

    def minimumSizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt API.
        return self._current_page_size()


class _FrozenPreviewGroup(QtWidgets.QGroupBox):
    """Prevent the Alignment page from vertically squeezing the 16:9 pair."""

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt API.
        desired_height = self.heightForWidth(self.width()) if self.hasHeightForWidth() else self.sizeHint().height()
        if desired_height > 0 and self.minimumHeight() != desired_height:
            self.setMinimumHeight(desired_height)
        super().resizeEvent(event)
