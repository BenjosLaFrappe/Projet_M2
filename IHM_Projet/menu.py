from PySide6.QtWidgets import QMainWindow, QPushButton, QToolBar, QMenu, QMenuBar

def creerMenu(fenetre):
    menu_fichier = fenetre.menuBar().addMenu("Fichier")
    action_fenetre = menu_fichier.addAction("Shut Down")
    action_fenetre.triggered.connect(fenetre.close)
