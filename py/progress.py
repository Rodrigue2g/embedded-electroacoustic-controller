
from PySide6.QtWidgets import QWidget, QApplication
from PySide6.QtCore import Qt, QTimer, QRectF
from PySide6.QtGui import QPainter, QPen, QColor

class ProgressOverlay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.angle = 0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.rotate)
        self.hide()

    def rotate(self):
        self.angle = (self.angle - 10) % 360
        self.update()

    def start(self):
        self.show()
        self.timer.start(35) # Rotation Speed

    def stop(self):
        self.timer.stop()
        self.hide()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        painter.fillRect(self.rect(), QColor(255, 255, 255, 150))
        
        pen = QPen(QColor("#2196F3"), 6)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        
        width = 60
        rect = QRectF((self.width() - width) / 2, (self.height() - width) / 2, width, width)
        
        painter.drawArc(rect, self.angle * 16, 120 * 16)