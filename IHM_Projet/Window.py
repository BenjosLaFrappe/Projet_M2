from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QSlider, QVBoxLayout

class BaseWindow(QWidget):
    def __init__(self, titre):
        super().__init__()
        self.setWindowTitle(titre)
        self.setFixedSize(600, 200)

class Cam2DWindow(BaseWindow):
    def __init__(self):
        super().__init__("Caméra 2D")

class Cam3DWindow(BaseWindow):
    def __init__(self):
        super().__init__("Caméra 3D")

class KinovaWindow(BaseWindow):
    def __init__(self):
        super().__init__("Bras Kinova")