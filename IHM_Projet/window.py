from PySide6.QtCore import Qt
from PySide6.QtWidgets import QMainWindow, QPushButton, QTextEdit, QLineEdit, QSlider, QAbstractSlider, QWidget, QGridLayout
from menu import creerMenu
from bouton import display_values, bouton, display_2D, display_3D, display_2D_stop, display_3D_stop, display_KINOVA, display_KINOVA_stop, display_global, display_global_stop
from test import fenetreCurseur



class mainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ma fenêtre")

        creerMenu(self)          # on donne la fenêtre au fichier menu.py

        self.conteneur = QWidget()
        self.setCentralWidget(self.conteneur)
        self.grid = QGridLayout()
        self.conteneur.setLayout(self.grid)      # la grille gère le placement dans le conteneur

        # Définition des boutons (x et y ne servent plus : c'est la grille qui place)
        self.button_cam2D = bouton("Caméra2D", self.conteneur, 0, 0, 150, 40)
        self.button_cam3D = bouton("Caméra3D", self.conteneur, 0, 0, 150, 40)
        self.button_kinova = bouton("KINOVA", self.conteneur, 0, 0, 150, 40)

        self.button_cam2D_stop = bouton("Arrêt Caméra2D", self.conteneur, 0, 0, 150, 40)
        self.button_cam3D_stop = bouton("Arrêt Caméra3D", self.conteneur, 0, 0, 150, 40)
        self.button_kinova_stop = bouton("Arrêt KINOVA", self.conteneur, 0, 0, 150, 40)

        self.global_button = bouton("Lancement global", self.conteneur, 0, 0, 150, 40)
        self.global_button_stop = bouton("Arrêt global", self.conteneur, 0, 0, 150, 40)

        self.new_window_button = bouton("Réglage", self.conteneur, 0, 0, 150, 40)

        # Placement dans la grille : (widget, ligne, colonne)
        # Ligne 0 : lancements
        self.grid.addWidget(self.button_cam2D,   0, 0)
        self.grid.addWidget(self.button_cam3D,   0, 1)
        self.grid.addWidget(self.button_kinova,  0, 2)
        self.grid.addWidget(self.global_button,  0, 3)
        self.grid.addWidget(self.new_window_button, 2, 0)

        # Ligne 1 : arrêts
        self.grid.addWidget(self.button_cam2D_stop,  1, 0)
        self.grid.addWidget(self.button_cam3D_stop,  1, 1)
        self.grid.addWidget(self.button_kinova_stop, 1, 2)
        self.grid.addWidget(self.global_button_stop, 1, 3)

        self.grid.setColumnMinimumWidth(3, 150)   # colonne 3 vide = espace avant les boutons globaux
        self.grid.setRowStretch(3, 1)             # ligne vide qui pousse tout vers le haut
        self.grid.setColumnStretch(5, 1)          # colonne vide qui pousse tout vers la gauche

        ## Définition des actions des boutons

        #Boutons Caméra 2D
        #Boutons Allumer
        self.button_cam2D.clicked.connect(display_2D)
        self.button_cam2D.clicked.connect(lambda: self.button_cam2D.setEnabled(False))

        #Bouton Éteindre
        self.button_cam2D_stop.clicked.connect(display_2D_stop)
        self.button_cam2D_stop.clicked.connect(lambda: self.button_cam2D.setEnabled(True))

        #Boutons Caméra 3D
        #Boutons Allumer
        self.button_cam3D.clicked.connect(display_3D)
        self.button_cam3D.clicked.connect(lambda: self.button_cam3D.setEnabled(False))

        #Bouton Éteindre
        self.button_cam3D_stop.clicked.connect(display_3D_stop)
        self.button_cam3D_stop.clicked.connect(lambda: self.button_cam3D.setEnabled(True))

        #Boutons Kinova
        #Boutons Allumer
        self.button_kinova.clicked.connect(display_KINOVA)
        self.button_kinova.clicked.connect(lambda: self.button_kinova.setEnabled(False))

        #Bouton Éteindre
        self.button_kinova_stop.clicked.connect(display_KINOVA_stop)
        self.button_kinova_stop.clicked.connect(lambda: self.button_kinova.setEnabled(True))

        self.fenetre_curseur = fenetreCurseur()
        self.new_window_button.clicked.connect(lambda: self.fenetre_curseur.show())

        def disbled_buttons():
            self.button_cam2D.setEnabled(False)
            self.button_cam3D.setEnabled(False)
            self.button_kinova.setEnabled(False)
            self.global_button.setEnabled(False)

        def enabled_buttons():
            self.button_cam2D.setEnabled(True)
            self.button_cam3D.setEnabled(True)
            self.button_kinova.setEnabled(True)
            self.global_button.setEnabled(True)


        self.global_button.clicked.connect(disbled_buttons)
        self.global_button.clicked.connect(display_global)
        self.global_button_stop.clicked.connect(enabled_buttons)
        self.global_button_stop.clicked.connect(display_global_stop)
        
        """
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMaximum(100)
        self.slider.setMinimum(1)
        self.slider.setValue(50)
        self.slider.sliderReleased.connect(lambda: display_values(self.slider.value()))
        """