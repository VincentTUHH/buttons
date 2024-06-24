from PyQt5 import QtCore, QtGui, QtWidgets

from .panel_widget import PanelWidget


class MonoPanelWidget(PanelWidget):
    def __init__(self, parent=None, title='Mono', timeout_ms=0):
        super().__init__(
            parent=parent,
            inner_widget=QtWidgets.QLabel(),
            timeout_ms=timeout_ms,
        )
        self.inner_widget.setFrameShape(QtWidgets.QFrame.NoFrame)
        font = QtGui.QFont('Monospace')
        font.setPointSize(14)
        font.setStyleHint(QtGui.QFont.TypeWriter)
        self.inner_widget.setFont(font)
        self.inner_widget.setAlignment(QtCore.Qt.AlignCenter)
        self.set_title(title)
        self._digit_count = 5

    @PanelWidget.reset_timeout
    @QtCore.pyqtSlot(int)
    def set_number_int(self, number):
        disp_string = '{}'.format(number).center(self._digit_count)
        self.inner_widget.setText(disp_string)

    @PanelWidget.reset_timeout
    @QtCore.pyqtSlot(float)
    def set_number_float(self, number):
        disp_string = '{:.1f}'.format(number).center(self._digit_count)
        self.inner_widget.setText(disp_string)

    @PanelWidget.reset_timeout
    @QtCore.pyqtSlot()
    def set_number_none(self):
        disp_string = '-'.center(self._digit_count)
        self.inner_widget.setText(disp_string)
