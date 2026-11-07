from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QSlider, QVBoxLayout

#Répertoire des fenêtres de l'IHM







#Fenêtre de base qui sert de modèle aux autres
class BaseWindow(QWidget):
    def __init__(self, titre):
        super().__init__()
        self.setWindowTitle(titre)
        self.setFixedSize(600, 200)




#Fenêtre de la caméra 2D
class Cam2DWindow(BaseWindow):
    def __init__(self):
        super().__init__("Caméra 2D")




#Fenêtre de la caméra 3D
class Cam3DWindow(BaseWindow):
    def __init__(self):
        super().__init__("Caméra 3D")





#Fenêtre du bras robot KINOVA
class KinovaWindow(BaseWindow):
    def __init__(self):
        super().__init__("Bras Kinova")