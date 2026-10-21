"""Optical-pose Designer adapters, component bindings and native sizing policies."""

from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.ui.annotation_image_view import AnnotationImageView
from Workstation.ui.components import EngineeringValue


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


class OpticalPoseValue(EngineeringValue):
    """Designer promotion adapter for the existing live engineering value."""

    def __init__(self, parent=None):
        super().__init__("", parent)


class FrozenAnnotationView(AnnotationImageView):
    """Native annotation canvas with the established wheel interaction contract."""

    def __init__(self, parent=None):
        super().__init__(parent, wheel_zoom_requires_control=False)


def bind_component_form(form, owner, owner_name, make_solver_button=None):
    """Bind form-declared fields to their existing state owner, without copying state."""
    import json

    # The legacy command factory owns callback wiring. Transfer only presentation
    # from its Designer slot so the public two-argument factory remains intact.
    if make_solver_button is not None:
        for slot in sorted(
            form.findChildren(QtWidgets.QPushButton),
            key=lambda button: int(button.property("commandRegistrationOrder") or 0),
        ):
            if not slot.property("solverButton") or slot.property("designerCommandBound"):
                continue
            button = make_solver_button(slot.property("commandId"), slot.text())
            button.setObjectName(slot.objectName())
            # Preserve Designer's editable presentation, including button-specific
            # properties. Parentage, geometry, visibility and signal connections
            # belong to the existing runtime/layout owners and are not copied.
            for property_name in (
                "text", "font", "palette", "styleSheet", "cursor", "locale",
                "toolTip", "toolTipDuration", "statusTip", "whatsThis",
                "accessibleName", "accessibleDescription", "layoutDirection",
                "focusPolicy", "contextMenuPolicy", "autoFillBackground",
                "enabled", "minimumSize", "maximumSize", "sizePolicy",
                "icon", "iconSize", "shortcut", "checkable", "checked",
                "autoRepeat", "autoRepeatDelay", "autoRepeatInterval",
                "autoExclusive", "autoDefault", "default", "flat",
            ):
                button.setProperty(property_name, slot.property(property_name))
            for key in slot.dynamicPropertyNames():
                name = bytes(key).decode()
                button.setProperty(name, slot.property(name))
            button.setProperty("designerCommandBound", True)
            parent = slot.parentWidget()
            parent.layout().replaceWidget(slot, button)
            slot.setParent(None)
            slot.deleteLater()

    for widget in (form, *form.findChildren(QtWidgets.QWidget)):
        style_name = widget.property("styleObjectName")
        if style_name:
            widget.setObjectName(style_name)
        if isinstance(widget, OpticalPoseValue):
            widget.label.setText(widget.property("displayLabel") or "")
        if isinstance(widget, QtWidgets.QComboBox) and widget.property("optionValues"):
            values = json.loads(widget.property("optionValues"))
            if len(values) != widget.count():
                raise ValueError(f"Optical-pose Designer option count mismatch: {widget.objectName()}")
            for index, value in enumerate(values):
                widget.setItemData(index, value)
        if isinstance(widget, QtWidgets.QTableWidget):
            for column, width in enumerate(json.loads(widget.property("columnWidths") or "[]")):
                widget.setColumnWidth(column, width)
            for column, mode in enumerate(json.loads(widget.property("headerResizeModes") or "[]")):
                widget.horizontalHeader().setSectionResizeMode(column, mode)
            for row, height in enumerate(json.loads(widget.property("rowHeights") or "[]")):
                widget.setRowHeight(row, height)
                for column in range(widget.columnCount()):
                    if widget.item(row, column) is None:
                        widget.setItem(row, column, QtWidgets.QTableWidgetItem("--"))
        for component, field, key in json.loads(widget.property("componentBindings") or "[]"):
            if component != owner_name:
                continue
            if key is None:
                setattr(owner, field, widget)
            elif isinstance(key, int):
                values = getattr(owner, field, None)
                if values is None:
                    values = []
                    setattr(owner, field, values)
                while len(values) <= key:
                    values.append(None)
                values[key] = widget
            else:
                values = getattr(owner, field, None)
                if values is None:
                    values = {}
                    setattr(owner, field, values)
                values[key] = widget
    return form


def load_component_form(name, owner, owner_name, make_solver_button=None):
    """Load one checked-in component form independent of the working directory."""
    from Workstation.ui.designer import load_form

    form = load_form(name, None)
    return bind_component_form(form, owner, owner_name, make_solver_button)
