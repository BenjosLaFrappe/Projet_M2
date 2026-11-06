# -*- coding: utf-8 -*-
"""
calib_collecte_kinova.py - Collecte des poses outil du bras Kinova pour la calibration.

LECTURE SEULE : ce script ne commande AUCUN mouvement (ni twist, ni pose, ni pince).
Il lit uniquement la pose de l'outil dans la base (RefreshFeedback) quand tu appuies sur Entrée.
Le bras est déplacé par toi, avec ta méthode habituelle (interface Kinova), arrêt d'urgence à portée.

A lancer à la place de "KINOVA py32 35.py" (ne pas lancer les deux en même temps).
Orientation de l'outil : garde-la identique d'un point à l'autre (déplace seulement en translation).

Commandes :
  Entrée : capturer le point suivant (le bras doit être immobile)
  r      : refaire le dernier point (même idx)
  q      : quitter

Sortie : poses_base.csv (idx,x,y,z,thx,thy,thz) ; x,y,z en m, thx..thz en degrés (Kortex).
Le numéro idx doit correspondre à celui du point capturé côté ZED (calib_collecte_zed.py).
"""
import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import utilities

from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient


def mediane(vals):
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def ecart_type(vals):
    m = sum(vals) / float(len(vals))
    return (sum((v - m) ** 2 for v in vals) / float(len(vals))) ** 0.5


def lire_pose(base_cyclic, n_mesures, pause_s):
    """Retourne (pose médiane [x,y,z,thx,thy,thz], dispersion position en mm)."""
    mesures = [[], [], [], [], [], []]
    for _ in range(n_mesures):
        fb = base_cyclic.RefreshFeedback()
        vals = [fb.base.tool_pose_x, fb.base.tool_pose_y, fb.base.tool_pose_z,
                fb.base.tool_pose_theta_x, fb.base.tool_pose_theta_y, fb.base.tool_pose_theta_z]
        for k in range(6):
            mesures[k].append(float(vals[k]))
        time.sleep(pause_s)
    pose = [mediane(m) for m in mesures]
    disp_mm = (sum(ecart_type(mesures[k]) ** 2 for k in range(3)) ** 0.5) * 1000.0
    return pose, disp_mm


def ecrire_csv(chemin, lignes):
    with open(chemin, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "x", "y", "z", "thx", "thy", "thz"])
        for l in lignes:
            w.writerow(l)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", type=int, default=30, help="nombre de lectures moyennées par point")
    parser.add_argument("--disp-max-mm", type=float, default=1.0, help="dispersion max (bras immobile)")
    parser.add_argument("--sortie", default="poses_base.csv")
    args = utilities.parseConnectionArguments(parser)

    lignes = []
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base_cyclic = BaseCyclicClient(router)
        print("Connecté. Lecture seule. Entrée = capturer | r = refaire | q = quitter")

        while True:
            cmd = input("[Point n°{}] > ".format(len(lignes) + 1)).strip().lower()
            if cmd == "q":
                break
            refaire = (cmd == "r")
            if refaire and not lignes:
                print("Rien à refaire.")
                continue
            if cmd not in ("", "r"):
                continue

            pose, disp_mm = lire_pose(base_cyclic, args.n, 0.03)
            if disp_mm > args.disp_max_mm:
                print("REFUSE : le bras bouge (dispersion {:.2f} mm). Attends l'immobilité puis recommence."
                      .format(disp_mm))
                continue

            idx = lignes[-1][0] if refaire else len(lignes) + 1
            ligne = [idx] + [round(v, 5) for v in pose]
            if refaire:
                lignes[-1] = ligne
            else:
                lignes.append(ligne)
            ecrire_csv(args.sortie, lignes)
            print("Point {} OK : x={:.4f} y={:.4f} z={:.4f} m | theta=({:.1f}, {:.1f}, {:.1f}) deg | disp {:.2f} mm"
                  .format(idx, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], disp_mm))

    print("{} points écrits dans {}".format(len(lignes), args.sortie))


if __name__ == "__main__":
    main()
