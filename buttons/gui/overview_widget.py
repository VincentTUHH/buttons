from enum import IntEnum, auto

from PyQt5 import QtCore, QtWidgets

from .mono_panel_grid import MonoPanelGrid
from .panel_widget import PanelWidget


class WidgetIndex(IntEnum):
    STATUS = 0
    MANIPULATOR = auto()
    VBAT = auto()
    VCELL = auto()


class OverviewWidget(QtWidgets.QWidget):
    battery_voltage_update = QtCore.pyqtSignal(float)
    cell_voltage_update = QtCore.pyqtSignal(float)
    arming_state_update = QtCore.pyqtSignal(bool)
    manipulator_arming_state_update = QtCore.pyqtSignal(bool)

    def __init__(self, vehicle_name: str, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout()

        self.headline = QtWidgets.QLabel(vehicle_name)
        font = self.headline.font()
        font.setPointSize(14)
        self.headline.setStyleSheet('font-weight: bold;')
        self.headline.setFont(font)
        self.headline.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.headline)

        # size = MonoPanelGrid.Size(1, 3)
        # titles = ['Status', 'VBat', 'VCell']
        # self.panel_grid = MonoPanelGrid(size=size, titles=titles, parent=self)
        # widget = PanelWidget(self.panel_grid, timeout_ms=1000)
        # widget.set_title('State')
        # self.panel_grid.replace_widget(0, widget)
        # layout.addWidget(self.panel_grid)
        # layout.setContentsMargins(0, 0, 0, 0)
        # self.setLayout(layout)
        # self.setup_signals()

        size = MonoPanelGrid.Size(1, 4)
        titles = ['AUV', 'Arm', 'VBat', 'VCell']
        self.panel_grid = MonoPanelGrid(size=size, titles=titles, parent=self)
        widget = PanelWidget(self.panel_grid, timeout_ms=1000)
        widget.set_title('AUV')
        self.panel_grid.replace_widget(0, widget)
        manipulator_widget = PanelWidget(self.panel_grid, timeout_ms=1000)
        manipulator_widget.set_title('Arm')
        self.panel_grid.replace_widget(1, manipulator_widget)
        layout.addWidget(self.panel_grid)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)
        self.setup_signals()

    def setup_signals(self):
        self.battery_voltage_update.connect(self.set_battery_voltage)
        self.cell_voltage_update.connect(self.set_cell_voltage)
        self.arming_state_update.connect(self.set_arming_state)
        self.manipulator_arming_state_update.connect(
            self.set_manipulator_arming_state)

    def set_arming_state(self, value: bool):
        index = WidgetIndex.STATUS
        if value:
            self.panel_grid.widgets[index].set_error_color()
            self.panel_grid.set_text(index, 'armed')
        else:
            self.panel_grid.widgets[index].set_good_color()
            self.panel_grid.set_text(index, 'disarmed')
        self.panel_grid.widgets[index].enable_flashing(value)

    def set_manipulator_arming_state(self, value: bool):
        index = WidgetIndex.MANIPULATOR
        if value:
            self.panel_grid.widgets[index].set_error_color()
            self.panel_grid.set_text(index, 'armed')
        else:
            self.panel_grid.widgets[index].set_good_color()
            self.panel_grid.set_text(index, 'disarmed')
        self.panel_grid.widgets[index].enable_flashing(value)

    def set_state_text(self, value: str):
        index = WidgetIndex.STATUS
        self.panel_grid.set_text(index, value)
        self.panel_grid.widgets[index].set_normal_color()

    def set_battery_voltage(self, value: float):
        self.panel_grid.set_number_float(WidgetIndex.VBAT, value)

    def set_cell_voltage(self, value: float):
        index = WidgetIndex.VCELL
        self.panel_grid.set_number_float(index, value)
        widget = self.panel_grid.widgets[index]
        flashing = True
        if value <= 3.3:
            widget.set_error_color()
        elif value <= 3.5:
            widget.set_warning_color()
        else:
            flashing = False
            widget.set_good_color()
        widget.enable_flashing(flashing)
