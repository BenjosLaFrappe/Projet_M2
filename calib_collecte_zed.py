# -*- coding: utf-8 -*-
"""
calib_collecte_zed.py - Collecte des points de calibration côté caméra 3D (ZED 2).

Détecte une cible avec la ZED 2 et enregistre sa position 3D dans le repère MQTT de
Camera3D (x droite, y haut, z avant, mètres, origine = centre optique gauche),
c'est-à-dire exactement le repère que Camera3D publie sur camera/hand.

Cibles (--mode) :
  cube  (défaut) : cube rouge (arête --taille-cube, 0,04 m par défaut), détecté par couleur.
                   La ZED mesure la surface visible : le point est reculé d'une demi-arête
                   le long du rayon de vue pour estimer le centre du cube.
  aruco          : marqueur ArUco plat (--id, --dico)

IMPORTANT
  - Ne PAS lancer en même temps que Camera3D.py (une seule application peut ouvrir la ZED).
  - Aucune publication MQTT, aucune commande robot.
  - Mêmes paramètres ZED que Camera3D.py (HD720, ULTRA, mètres, RIGHT_HANDED_Z_UP).
  - Mode cube : retire tout autre objet rouge du champ de la caméra (cubes du convoyeur...).
    Si plusieurs objets rouges sont vus, le point est refusé.

Touches (fenêtre image) :
  ESPACE : capture du point suivant (cible et bras immobiles)
  r      : refaire le dernier point (remplace la dernière ligne, même idx)
  q      : quitter (les réglages HSV utilisés sont affichés)

Sortie : points_cam.csv  (idx,x,y,z,std_mm,n)
Le même numéro idx doit être capturé côté Kinova (calib_collecte_kinova.py).
"""
import argparse
import csv

import cv2
import numpy as np
import pyzed.sl as sl

NOYAU = np.ones((5, 5), np.uint8)
FEN_REGLAGES = "Reglages cube (masque rouge)"


def zed_vers_mqtt(p):
    """ZED RIGHT_HANDED_Z_UP (X droite, Y avant, Z haut) -> MQTT (x droite, y haut, z avant)."""
    return np.array([p[0], p[2], p[1]], dtype=np.float64)


# ----------------------------------------------------------------------
# Détection : cube rouge
# ----------------------------------------------------------------------
def masque_rouge(img_bgr, h_tol, s_min, v_min):
    """Le rouge est à cheval sur la teinte 0/180 : deux plages."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    bas = cv2.inRange(hsv, np.array([0, s_min, v_min], np.uint8), np.array([h_tol, 255, 255], np.uint8))
    haut = cv2.inRange(hsv, np.array([180 - h_tol, s_min, v_min], np.uint8), np.array([180, 255, 255], np.uint8))
    m = cv2.bitwise_or(bas, haut)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, NOYAU)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, NOYAU)
    return m


def detecter_cube(img, data_xyz, h_tol, s_min, v_min, aire_min, taille):
    """Retourne (position MQTT ou None, message, contour ou None, masque)."""
    m = masque_rouge(img, h_tol, s_min, v_min)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= aire_min]
    if not cnts:
        return None, "aucun objet rouge", None, m
    cnts.sort(key=cv2.contourArea, reverse=True)
    if len(cnts) > 1 and cv2.contourArea(cnts[1]) > 0.3 * cv2.contourArea(cnts[0]):
        return None, "PLUSIEURS OBJETS ROUGES", cnts[0], m

    c = cnts[0]
    x, y, wb, hb = cv2.boundingRect(c)
    h, w = m.shape[:2]
    if x <= 2 or y <= 2 or (x + wb) >= (w - 2) or (y + hb) >= (h - 2):
        return None, "cube au bord de l'image", c, m

    zone = np.zeros_like(m)
    cv2.drawContours(zone, [c], -1, 255, -1)
    zone = cv2.erode(zone, NOYAU, iterations=2)          # évite les bords (mélange avec le fond)
    pts = data_xyz[zone > 0][:, :3]
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    if len(pts) < 50:
        return None, "profondeur insuffisante", c, m

    p = np.median(pts, axis=0).astype(np.float64)        # surface visible (repère ZED)
    n = np.linalg.norm(p)
    if n < 1e-6:
        return None, "profondeur invalide", c, m
    p = p + (taille / 2.0) * (p / n)                     # centre du cube : demi-arête derrière
    return zed_vers_mqtt(p), "OK", c, m


# ----------------------------------------------------------------------
# Détection : ArUco
# ----------------------------------------------------------------------
def creer_detecteur(nom_dico):
    """Compatible OpenCV >= 4.7 (ArucoDetector) et versions antérieures."""
    id_dico = getattr(cv2.aruco, nom_dico)
    if hasattr(cv2.aruco, "ArucoDetector"):
        dico = cv2.aruco.getPredefinedDictionary(id_dico)
        det = cv2.aruco.ArucoDetector(dico, cv2.aruco.DetectorParameters())
        return lambda gris: det.detectMarkers(gris)
    dico = cv2.aruco.Dictionary_get(id_dico)
    params = cv2.aruco.DetectorParameters_create()
    return lambda gris: cv2.aruco.detectMarkers(gris, dico, parameters=params)


def detecter_aruco(img, data_xyz, detecter, id_cible, demi=4):
    """Retourne (position MQTT ou None, message, contour ou None)."""
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    coins, ids, _ = detecter(gris)
    if ids is None:
        return None, "aucun marqueur", None
    for c, i in zip(coins, ids.flatten()):
        if int(i) != id_cible:
            continue
        p4 = c.reshape(4, 2)
        u, v = int(p4[:, 0].mean()), int(p4[:, 1].mean())
        h, w = data_xyz.shape[:2]
        fen = data_xyz[max(0, v - demi):min(h, v + demi + 1), max(0, u - demi):min(w, u + demi + 1), :3]
        fen = fen.reshape(-1, 3)
        fen = fen[np.all(np.isfinite(fen), axis=1)]
        if len(fen) < 10:
            return None, "profondeur insuffisante", p4.astype(np.int32)
        return zed_vers_mqtt(np.median(fen, axis=0)), "OK", p4.astype(np.int32)
    return None, "marqueur id {} absent".format(id_cible), None


# ----------------------------------------------------------------------
def ecrire_csv(chemin, lignes):
    with open(chemin, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["idx", "x", "y", "z", "std_mm", "n"])
        for l in lignes:
            w.writerow(l)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["cube", "aruco"], default="cube")
    ap.add_argument("--taille-cube", type=float, default=0.04, help="arête du cube (m)")
    ap.add_argument("--h-tol", type=int, default=10, help="largeur de teinte autour du rouge (0-30)")
    ap.add_argument("--s-min", type=int, default=120, help="saturation minimale")
    ap.add_argument("--v-min", type=int, default=70, help="luminosité minimale")
    ap.add_argument("--aire-min", type=int, default=150, help="aire minimale de la tache (px)")
    ap.add_argument("--id", type=int, default=0, help="identifiant ArUco (mode aruco)")
    ap.add_argument("--dico", default="DICT_4X4_50", help="dictionnaire ArUco (mode aruco)")
    ap.add_argument("--n", type=int, default=40, help="nombre d'images moyennées par point")
    ap.add_argument("--std-max-mm", type=float, default=5.0, help="dispersion max acceptée (mm)")
    ap.add_argument("--z-min", type=float, default=0.35,
                    help="profondeur minimale (m) : en dessous, la ZED est peu fiable, point refusé")
    ap.add_argument("--sortie", default="points_cam.csv")
    args = ap.parse_args()

    detecter = creer_detecteur(args.dico) if args.mode == "aruco" else None

    zed = sl.Camera()
    ip = sl.InitParameters()
    ip.camera_resolution = sl.RESOLUTION.HD720
    ip.depth_mode = sl.DEPTH_MODE.ULTRA
    ip.coordinate_units = sl.UNIT.METER
    ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        print("Erreur : impossible d'ouvrir la ZED (Camera3D.py est-il encore lancé ?)")
        raise SystemExit(1)

    rt = sl.RuntimeParameters()
    image = sl.Mat()
    nuage = sl.Mat()

    if args.mode == "cube":
        cv2.namedWindow(FEN_REGLAGES)
        cv2.createTrackbar("H_tol", FEN_REGLAGES, args.h_tol, 30, lambda v: None)
        cv2.createTrackbar("S_min", FEN_REGLAGES, args.s_min, 255, lambda v: None)
        cv2.createTrackbar("V_min", FEN_REGLAGES, args.v_min, 255, lambda v: None)

    lignes = []          # [idx, x, y, z, std_mm, n]
    capture = None       # mesures pendant une capture
    n_images = 0         # images vues pendant la capture (timeout)
    refaire = False
    message = ""
    reglages = (args.h_tol, args.s_min, args.v_min)

    print("Mode {} | ESPACE = capturer | r = refaire le dernier | q = quitter".format(args.mode))
    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(nuage, sl.MEASURE.XYZ)
            img = image.get_data()[:, :, :3].copy()
            data = nuage.get_data()
            h, w = img.shape[:2]

            if args.mode == "cube":
                reglages = (cv2.getTrackbarPos("H_tol", FEN_REGLAGES),
                            cv2.getTrackbarPos("S_min", FEN_REGLAGES),
                            cv2.getTrackbarPos("V_min", FEN_REGLAGES))
                pos, etat_det, cnt, masque = detecter_cube(
                    img, data, reglages[0], reglages[1], reglages[2], args.aire_min, args.taille_cube)
                cv2.imshow(FEN_REGLAGES, cv2.resize(masque, (w // 2, h // 2)))
            else:
                pos, etat_det, cnt = detecter_aruco(img, data, detecter, args.id)

            if pos is not None and pos[2] < args.z_min:
                pos, etat_det = None, "trop près ({:.2f} m < {:.2f} m)".format(pos[2], args.z_min)

            if cnt is not None:
                cv2.polylines(img, [cnt.reshape(-1, 1, 2).astype(np.int32)], True,
                              (0, 255, 0) if pos is not None else (0, 0, 255), 2)

            # accumulation pendant une capture
            if capture is not None:
                n_images += 1
                if pos is not None:
                    capture.append(pos)
                if n_images > 5 * args.n and len(capture) < args.n:
                    message = "REFUSE : cible perdue ({}), recommence".format(etat_det)
                    capture = None
                    refaire = False
                elif len(capture) >= args.n:
                    arr = np.array(capture)
                    med = np.median(arr, axis=0)
                    std_mm = float(np.sqrt(np.sum(np.var(arr, axis=0))) * 1000.0)
                    if std_mm > args.std_max_mm:
                        message = "REFUSE : instable (sigma={:.1f} mm), recommence".format(std_mm)
                    else:
                        idx = (lignes[-1][0] if (refaire and lignes) else len(lignes) + 1)
                        ligne = [idx, round(float(med[0]), 5), round(float(med[1]), 5),
                                 round(float(med[2]), 5), round(std_mm, 2), len(capture)]
                        if refaire and lignes:
                            lignes[-1] = ligne
                        else:
                            lignes.append(ligne)
                        ecrire_csv(args.sortie, lignes)
                        message = "Point {} OK : x={:.3f} y={:.3f} z={:.3f} (sigma {:.1f} mm)".format(
                            idx, med[0], med[1], med[2], std_mm)
                        print(message)
                    capture = None
                    refaire = False
                if capture is None and message.startswith("REFUSE"):
                    print(message)

            # affichage
            suivant = len(lignes) + 1
            cv2.putText(img, "Point suivant : {}   [{}]".format(suivant, etat_det), (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            if pos is not None:
                cv2.putText(img, "x:{:.3f} y:{:.3f} z:{:.3f}".format(*pos), (20, 62),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if capture is not None:
                cv2.putText(img, "Capture {}/{}".format(len(capture), args.n), (20, 94),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            if message:
                cv2.putText(img, message, (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("Calibration ZED (ESPACE / r / q)", img)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            if k == 32 and capture is None:
                capture, n_images, refaire, message = [], 0, False, ""
            if k == ord('r') and capture is None and lignes:
                capture, n_images, refaire = [], 0, True
                message = "Refaire le point {}".format(lignes[-1][0])
    finally:
        zed.close()
        cv2.destroyAllWindows()
        print("{} points écrits dans {}".format(len(lignes), args.sortie))
        if args.mode == "cube":
            print("Réglages HSV utilisés : --h-tol {} --s-min {} --v-min {}".format(*reglages))


if __name__ == "__main__":
    main()
