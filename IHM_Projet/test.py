from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QSlider, QVBoxLayout
from bouton import display_values

class fenetreCurseur(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Réglage")

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMaximum(100)
        self.slider.setMinimum(1)
        self.slider.setValue(50)
        self.slider.sliderReleased.connect(lambda: display_values(self.slider.value()))

        layout = QVBoxLayout()
        self.setLayout(layout)          # le layout gère la fenêtre
        layout.addWidget(self.slider)   # on y place le curseur

