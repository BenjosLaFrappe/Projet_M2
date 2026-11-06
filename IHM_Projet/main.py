import sys
from PySide6.QtWidgets import QApplication
from MainWindow import mainWindow

app = QApplication(sys.argv)

window = mainWindow()
#window.showMaximized()
window.show()

sys.exit(app.exec())