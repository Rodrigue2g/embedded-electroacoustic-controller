import sys
from PySide6.QtWidgets import QApplication
from py.gui import GUI


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = GUI()
    gui.resize(600, 400)
    gui.show()
    sys.exit(app.exec())
