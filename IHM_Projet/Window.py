from PySide6.QtCore import Qt
from PySide6.QtWidgets import QTextEdit, QWidget, QSlider, QVBoxLayout
from PySide6.QtWidgets import QMainWindow, QPushButton, QTextEdit, QLineEdit, QSlider, QAbstractSlider, QWidget, QGridLayout, QApplication
import sys
from subprocess import Popen
#Répertoire des fenêtres de l'IHM

cam2D = "sum.py"
cam3D = "sum.py"
kinova = "sum.py"





#Fenêtre de base qui sert de modèle aux autres
class BaseWindow(QMainWindow):
    def __init__(self, titre):
        super().__init__()
        self.setWindowTitle(titre)
        self.setFixedSize(600, 200)


        



#Fenêtre de la caméra 2D
class Cam2DWindow(BaseWindow):
    def __init__(self):
        super().__init__("Caméra 2D")

        self.text = QTextEdit()
        self.setCentralWidget(self.text)
        self.text.setReadOnly(True)
        self.text.setText("Fenêtre de la caméra 2D")




#Fenêtre de la caméra 3D
class Cam3DWindow(BaseWindow):
    def __init__(self):
        super().__init__("Caméra 3D")





#Fenêtre du bras robot KINOVA
class KinovaWindow(BaseWindow):
    def __init__(self):
        super().__init__("Bras Kinova")