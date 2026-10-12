"""Reusable Qt dialog and input controls."""
from __future__ import annotations

from PyQt5 import (
    QtCore,
    QtWidgets,
)

class TopicBrowserDialog(QtWidgets.QDialog):
    def __init__(self, topics: list[dict[str, str]], parent=None, *, title: str = "MAVLink Message Browser") -> None:
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(900, 560)
        self._topics = topics
        self._filtered_topics = list(topics)
        layout = QtWidgets.QVBoxLayout(self)
        self.search_edit = QtWidgets.QLineEdit()
        self.search_edit.setPlaceholderText("토픽, 메시지, 타입 검색")
        self.search_edit.textChanged.connect(self._refresh)
        layout.addWidget(self.search_edit)
        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["토픽", "타입"])
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        layout.addWidget(self.table, 1)
        buttons = QtWidgets.QHBoxLayout()
        select_button = QtWidgets.QPushButton("토픽 선택")
        select_button.clicked.connect(self.accept)
        cancel_button = QtWidgets.QPushButton("취소")
        cancel_button.clicked.connect(self.reject)
        buttons.addStretch(1)
        buttons.addWidget(select_button)
        buttons.addWidget(cancel_button)
        layout.addLayout(buttons)
        self.table.itemDoubleClicked.connect(lambda _item: self.accept())
        self._refresh()

    def _refresh(self) -> None:
        query = self.search_edit.text().strip().lower()
        self._filtered_topics = [
            topic
            for topic in self._topics
            if not query or query in topic.get("name", "").lower() or query in topic.get("type", "").lower()
        ]
        self.table.setRowCount(len(self._filtered_topics))
        for row, topic in enumerate(self._filtered_topics):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(topic.get("name", "")))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(topic.get("type", "")))
        if self._filtered_topics:
            self.table.selectRow(0)

    def selected_topic(self) -> dict[str, str] | None:
        row = self.table.currentRow()
        if row < 0 or row >= len(self._filtered_topics):
            return None
        return dict(self._filtered_topics[row])

class FieldSelectionDialog(QtWidgets.QDialog):
    def __init__(self, topic_name: str, topic_type: str, fields: list[str], parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle(f"Select Fields: {topic_name}")
        self.resize(720, 560)
        self._fields = fields
        layout = QtWidgets.QVBoxLayout(self)
        summary = QtWidgets.QLabel(f"{topic_name}\n{topic_type}")
        summary.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(summary)
        self.search_edit = QtWidgets.QLineEdit()
        self.search_edit.setPlaceholderText("필드 검색")
        self.search_edit.textChanged.connect(self._refresh)
        layout.addWidget(self.search_edit)
        self.list_widget = QtWidgets.QListWidget()
        self.list_widget.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        layout.addWidget(self.list_widget, 1)
        buttons = QtWidgets.QHBoxLayout()
        add_button = QtWidgets.QPushButton("선택 필드 추가")
        add_button.clicked.connect(self.accept)
        cancel_button = QtWidgets.QPushButton("취소")
        cancel_button.clicked.connect(self.reject)
        buttons.addStretch(1)
        buttons.addWidget(add_button)
        buttons.addWidget(cancel_button)
        layout.addLayout(buttons)
        self.list_widget.itemDoubleClicked.connect(lambda _item: self.accept())
        self._refresh()

    def _refresh(self) -> None:
        query = self.search_edit.text().strip().lower()
        self.list_widget.clear()
        for field in self._fields:
            if query and query not in field.lower():
                continue
            self.list_widget.addItem(field)
        if self.list_widget.count():
            self.list_widget.setCurrentRow(0)

    def selected_fields(self) -> list[str]:
        return [item.text() for item in self.list_widget.selectedItems()]

class NoWheelSpinBox(QtWidgets.QSpinBox):
    def wheelEvent(self, event) -> None:
        event.ignore()
