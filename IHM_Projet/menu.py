from PySide6.QtWidgets import QMainWindow, QPushButton, QToolBar, QMenu, QMenuBar
from Window import Cam2DWindow, Cam3DWindow, KinovaWindow

def creerMenu(fenetre):
    menu_fichier = fenetre.menuBar().addMenu("Fichier")
    action_fenetre = menu_fichier.addAction("Shut Down")
    action_fenetre.triggered.connect(fenetre.close)

    menu_composants = fenetre.menuBar().addMenu("Componants")
    action_fenetre_cam2D = menu_composants.addAction("Caméra 2D")
    fenetre_cam2D = Cam2DWindow()
    action_fenetre_cam2D.triggered.connect(lambda : fenetre_cam2D.show())
    action_fenetre_cam3D = menu_composants.addAction("Caméra 3D")
    fenetre_cam3D = Cam3DWindow()
    action_fenetre_cam3D.triggered.connect(lambda : fenetre_cam3D.show())
    action_fenetre_kinova = menu_composants.addAction("Bras Kinova")
    fenetre_kinova = KinovaWindow()
    action_fenetre_kinova.triggered.connect(lambda : fenetre_kinova.show())
