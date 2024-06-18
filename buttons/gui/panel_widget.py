from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt


class PanelWidget(QtWidgets.QWidget):
    ALARM_COLOR = QtGui.QColor(255, 0, 0)
    NORMAL_COLOR = QtGui.QColor(100, 149, 237)
    GOOD_COLOR = QtGui.QColor(50, 200, 50)
    WARNING_COLOR = QtGui.QColor(255, 100, 0)

    def __init__(self, parent=None, inner_widget=None):
        super().__init__(parent)
        self._border_width = 2
        self._border_radius = 0

        self._title_color = QtGui.QColor(255, 255, 255)
        self._title_font = QtGui.QFont()
        self._title_font.setPointSize(16)
        self._title_text = 'Title'
        self._title_height = self.get_title_height()

        self._color = QtGui.QColor(100, 149, 237)
        self.flash_on_timer = QtCore.QTimer(self)
        self.flash_off_timer = QtCore.QTimer(self)
        self.flash_on_timer.timeout.connect(self.on_flash_on_timer)
        self.flash_off_timer.timeout.connect(self.on_flash_off_timer)
        self.flashing = False

        if inner_widget:
            self.inner_widget = inner_widget
            self.inner_widget.setParent(self)
        else:
            self.inner_widget = QtWidgets.QLabel(self)
            self.inner_widget.setFrameShape(QtWidgets.QFrame.NoFrame)
            self.inner_widget.setAlignment(QtCore.Qt.AlignCenter)
            font = self.inner_widget.font()
            font.setPointSize(14)
            self.inner_widget.setFont(font)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.inner_widget)
        self.setLayout(layout)

    def get_title_height(self):
        label = QtWidgets.QLabel(self._title_text)
        label.setFont(self._title_font)
        height = label.fontMetrics().boundingRect(label.text()).height()
        return height

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHints(
            QtGui.QPainter.Antialiasing | QtGui.QPainter.TextAntialiasing
        )

        self.draw_border(painter)
        self.draw_title(painter)

    def draw_border(self, painter):
        painter.save()
        pen = QtGui.QPen()
        pen.setWidth(self._border_width)
        pen.setColor(self._color)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        path = QtGui.QPainterPath()
        rect = QtCore.QRectF(
            self._border_width / 2,
            self._border_width / 2,
            self.width() - self._border_width,
            self.height() - self._border_width,
        )
        path.addRoundedRect(rect, self._border_radius, self._border_radius)
        painter.drawPath(path)

        painter.restore()

    def draw_title(self, painter):
        self.layout().setContentsMargins(
            self._border_radius,
            self._title_height + self._border_radius,
            self._border_radius,
            self._border_radius,
        )
        painter.save()
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(self._color)
        offset = int(2 * self._border_width / 3)
        rect = QtCore.QRect(
            offset, offset, self.width() - offset * 2, self._title_height
        )
        painter.drawRect(rect)

        painter.setPen(self._title_color)
        painter.setFont(self._title_font)
        offset = self._border_width * 3
        text_rect = QtCore.QRect(
            offset, 0, self.width() - offset * 2, self._title_height
        )
        align = Qt.Alignment(Qt.AlignHCenter | Qt.AlignVCenter)
        painter.drawText(text_rect, align, self._title_text)
        painter.restore()

    @QtCore.pyqtSlot(str)
    def set_title(self, title):
        self._title_text = title

    @QtCore.pyqtSlot(str)
    def set_text(self, value):
        self.inner_widget.setText(value)

    @QtCore.pyqtSlot(QtCore.QObject)
    def set_inner_widget(self, widget):
        self.layout().removeWidget(self.inner_widget)
        self.inner_widget.setParent(None)
        self.inner_widget = widget
        self.layout().addWidget(self.inner_widget)

    def _set_color(self, color):
        if self._color != color:
            self._color = color
            self.repaint()

    def enable_flashing(self, value):
        if self.flashing != value:
            self.flashing = value
            self.on_flash_on_timer()

    @QtCore.pyqtSlot()
    def on_flash_on_timer(self):
        if not self.flashing:
            self.flash_on_timer.stop()
            self.flash_off_timer.stop()
            palette = self.palette()
            # self.setStyleSheet('background-color:none;')
            palette.setColor(QtGui.QPalette.Background, Qt.transparent)
            self.setAutoFillBackground(True)
            self.setPalette(palette)
        else:
            palette = self.palette()
            palette.setColor(QtGui.QPalette.Background, self._color)
            self.setAutoFillBackground(True)
            self.setPalette(palette)
            # self.setStyleSheet('background-color:red;')
            self.flash_off_timer.start(500)
            self.flash_on_timer.start(1000)

    @QtCore.pyqtSlot()
    def on_flash_off_timer(self):
        palette = self.palette()
        palette.setColor(QtGui.QPalette.Background, Qt.transparent)
        self.setAutoFillBackground(True)
        self.setPalette(palette)
        self.setStyleSheet('background-color:none;')
        self.flash_off_timer.stop()

    @QtCore.pyqtSlot()
    def set_error_color(self):
        self._set_color(self.ALARM_COLOR)

    @QtCore.pyqtSlot()
    def set_good_color(self):
        self._set_color(self.GOOD_COLOR)

    @QtCore.pyqtSlot()
    def set_warning_color(self):
        self._set_color(self.WARNING_COLOR)

    @QtCore.pyqtSlot()
    def set_normal_color(self):
        self._set_color(self.NORMAL_COLOR)
