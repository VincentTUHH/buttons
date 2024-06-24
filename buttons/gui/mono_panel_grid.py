from PyQt5 import QtCore, QtWidgets

from .mono_panel_widget import MonoPanelWidget


class MonoPanelGrid(QtWidgets.QWidget):
    class Size:
        def __init__(self, rows, cols):
            self.rows = rows
            self.cols = cols

    def __init__(self, size: Size, titles: list[str], parent=None):
        super().__init__(parent)
        layout = QtWidgets.QGridLayout()
        self.widgets = []
        self.grid_size = size

        for row in range(size.rows):
            for col in range(size.cols):
                index = row * size.cols + col
                widget = MonoPanelWidget(self, titles[index], timeout_ms=1000)
                widget.set_number_none()
                self.widgets.append(widget)
                layout.addWidget(widget, row, col)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

    @QtCore.pyqtSlot(int, int)
    def set_number_int(self, index, value):
        self.widgets[index].set_number_int(value)

    @QtCore.pyqtSlot(int, float)
    def set_number_float(self, index, value):
        self.widgets[index].set_number_float(value)

    @QtCore.pyqtSlot(int)
    def set_number_none(self, index):
        self.widgets[index].set_number_none()

    @QtCore.pyqtSlot(int, str)
    def set_text(self, index, text):
        self.widgets[index].set_text(text)

    def replace_widget(self, index, widget):
        self.layout().removeWidget(self.widgets[index])
        self.widgets[index].close()
        self.widgets[index] = widget
        row = index // self.grid_size.cols
        col = index % self.grid_size.cols
        self.layout().addWidget(self.widgets[index], row, col)
