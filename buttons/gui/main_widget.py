from PyQt5 import QtWidgets

from .overview_widget import OverviewWidget


class MainWidget(QtWidgets.QWidget):
    def __init__(self, vehicle_names: list[str], parent=None):
        super().__init__(parent=parent)
        layout = QtWidgets.QVBoxLayout()
        self.overview_widgets = []
        for name in vehicle_names:
            widget = OverviewWidget(name)
            self.overview_widgets.append(widget)
            layout.addWidget(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.setLayout(layout)
        self.setFixedWidth(320)
        self.setFixedHeight(240)
