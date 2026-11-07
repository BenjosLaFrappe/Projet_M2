from PySide6.QtWidgets import QMainWindow, QPushButton, QToolBar, QMenu, QMenuBar
from subprocess import Popen, STDOUT
import sys
class bouton(QPushButton):
	def __init__(self, name: str, parent, x, y, lenght, height):
		super().__init__(name)
		#self.setParent(parent)
		#self.setGeometry(x, y, lenght, height)
		self.setFixedSize(lenght, height)

	def clicked_state(self):
		return self.isChecked()


cam2D = "sum.py"
cam3D = "sum.py"
kinova = "sum.py"
"""
cam2D = "C:\\Users\\demst\\Documents\\Espace de travail\\ET\\Camera2D.py"
cam3D = "C:\\Users\\demst\\Documents\\Espace de travail\\ET\\Camera3DV2.py"
kinova = "C:\\Users\\demst\\Documents\\Espace de travail\\ET\\KINOVA py32 35 V2.py"
"""


cam2D_open = None
cam3D_open = None
kinova_open = None
def display_2D():
	global cam2D_open  # Permet de modifier la variable globale
	# On vérifie si le programme ne tourne pas déjà
	if cam2D_open is None or cam2D_open.poll() is not None:
		print("La caméra 2D est lancée")
		cam2D_open = Popen([sys.executable, cam2D])


def display_3D():
	global cam3D_open
	if cam3D_open is None or cam3D_open.poll() is not None:
		print("La caméra 3D est lancée")
		cam3D_open = Popen([sys.executable, cam3D])
		print(cam3D_open.stdout)

def display_KINOVA():
	global kinova_open
	if kinova_open is None or kinova_open.poll() is not None:
		print("Le robot est lancé")
		kinova_open = Popen([sys.executable, kinova])
		print(kinova_open.stdout)

def display_global():
	global cam2D_open, cam3D_open, kinova_open
	if cam2D_open is None or cam2D_open.poll() is not None:
			display_2D()
	if cam3D_open is None or cam3D_open.poll() is not None:
		display_3D()
	if kinova_open is None or kinova_open.poll() is not None:
		display_KINOVA()


		
def display_2D_stop():
	global cam2D_open
	if cam2D_open is not None and cam2D_open.poll() is None:
		cam2D_open.terminate()  # Demande gentiment au programme de se fermer
		cam2D_open.wait()       # Attend que le programme soit bien fermé
		cam2D_open = None       # Réinitialise la variable
		print("La caméra 2D est arrêtée")


def display_3D_stop():
	global cam3D_open
	if cam3D_open is not None and cam3D_open.poll() is None:
		cam3D_open.terminate()  # Demande gentiment au programme de se fermer
		cam3D_open.wait()       # Attend que le programme soit bien fermé
		cam3D_open = None       # Réinitialise la variable
		print("La caméra 3D est arrêtée")
	

def display_KINOVA_stop():
	global kinova_open
	if kinova_open is not None and kinova_open.poll() is None:
		kinova_open.terminate()  # Demande gentiment au programme de se fermer
		kinova_open.wait()       # Attend que le programme soit bien fermé
		kinova_open = None       # Réinitialise la variable
		print("Le KINOVA est arrêté")

def display_global_stop():
	global cam2D_open, cam3D_open, kinova_open
	if cam2D_open is not None and cam2D_open.poll() is None:
		cam2D_open.terminate()  # Demande gentiment au programme de se fermer
		cam2D_open.wait()       # Attend que le programme soit bien fermé
		cam2D_open = None       # Réinitialise la variable
	if cam3D_open is not None and cam3D_open.poll() is None:
		cam3D_open.terminate()  # Demande gentiment au programme de se fermer
		cam3D_open.wait()       # Attend que le programme soit bien fermé
		cam3D_open = None       # Réinitialise la variable
	if kinova_open is not None and kinova_open.poll() is None:
		kinova_open.terminate()  # Demande gentiment au programme de se fermer
		kinova_open.wait()       # Attend que le programme soit bien fermé
		kinova_open = None       # Réinitialise la variable
		print("Le KINOVA est arrêté")



def display_values(data):
	print("Le slider est à ", data)



