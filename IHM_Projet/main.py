import sys
from PySide6.QtWidgets import QApplication
from window import mainWindow

app = QApplication(sys.argv)

window = mainWindow()
#window.showMaximized()
window.show()

sys.exit(app.exec())