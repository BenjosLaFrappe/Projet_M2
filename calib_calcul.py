# -*- coding: utf-8 -*-
"""
calib_calcul.py - Calibration caméra 3D (repère MQTT de Camera3D) -> base Kinova.

Entrées (produites par calib_collecte_zed.py et calib_collecte_kinova.py) :
  - points_cam.csv   : idx,x,y,z,std_mm,n   (repère MQTT : x droite, y haut, z avant, en m)
  - poses_base.csv   : idx,x,y,z,thx,thy,thz (base Kinova, m et degrés)

Méthode : ajustement RIGIDE (rotation + translation) par moindres carrés (Kabsch/SVD),
fait dans un repère droitier (repère ZED), puis recomposé avec la permutation vers
le repère MQTT. La matrice _CAM_A obtenue a donc un déterminant de -1 (réflexion
inévitable : le repère MQTT est gaucher, la base Kinova est droitière), exactement
comme l'ancienne matrice. Ni l'interface MQTT ni camera3d_coordinates_to_kinova_xyz_m()
ne changent : seules les constantes _CAM_A / _CAM_T sont remplacées.

Ce script ne commande aucun matériel.

Exemples :
  python calib_calcul.py --cam points_cam.csv --base poses_base.csv
  python calib_calcul.py --cam points_cam.csv --base poses_base.csv --test 4,9,14
  python calib_calcul.py --cam points_cam.csv --base poses_base.csv --centre-idx 1
  python calib_calcul.py --cam points_cam.csv --base poses_base.csv --centre-base 0.45 -0.10 0.35
"""
import argparse
import csv
import sys

import numpy as np

# Ancienne calibration (KINOVA py32 35.py), pour comparaison sur les nouvelles mesures
A_OLD = np.array([
    [0.92638889, -0.02326389, -0.02916667],
    [0.00986111,  0.60420139,  0.72541667],
    [0.11076389,  0.79314236, -0.59479167],
])
T_OLD = np.array([0.53871528, -0.38855903, 0.54544965])

# Limites de l'espace de travail du script Kinova (à garder synchronisées avec
# MIN_X.. MAX_Z de KINOVA py32 35.py ; ce script ne les modifie jamais)
LIM_MIN = np.array([-0.10, -0.90, 0.02])
LIM_MAX = np.array([0.90, 0.90, 1.10])

# Repère MQTT (x droite, y haut, z avant) -> repère ZED droitier (X droite, Y avant, Z haut)
P_MQTT_VERS_RH = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
])

SEUIL_BON_MM = 5.0
SEUIL_LIMITE_MM = 10.0


# ----------------------------------------------------------------------
# Lecture
# ----------------------------------------------------------------------
def lire_csv(chemin):
    lignes = {}
    with open(chemin, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            lignes[int(row["idx"])] = row
    return lignes


# ----------------------------------------------------------------------
# Calcul
# ----------------------------------------------------------------------
def kabsch(P, Q):
    """Q ~ R @ P + t, avec R rotation propre (det = +1). P, Q : (N,3), repères droitiers."""
    cp = P.mean(axis=0)
    cq = Q.mean(axis=0)
    Pc = P - cp
    Qc = Q - cq
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    d = 1.0 if np.linalg.det(Vt.T @ U.T) >= 0.0 else -1.0
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = cq - R @ cp
    # échelle estimée (Umeyama) : information seulement, non utilisée dans l'ajustement
    echelle = (S[0] + S[1] + d * S[2]) / float(np.sum(Pc ** 2))
    return R, t, echelle


def calibrer(P_mqtt, Q):
    """Retourne A (3x3), T (3,), échelle, tels que Q ~ A @ p_mqtt + T."""
    P_rh = P_mqtt @ P_MQTT_VERS_RH.T
    R, t, echelle = kabsch(P_rh, Q)
    A = R @ P_MQTT_VERS_RH
    return A, t, echelle


def erreurs_mm(A, T, P, Q):
    pred = P @ A.T + T
    return np.linalg.norm(pred - Q, axis=1) * 1000.0


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def ecart_angle_deg(a, ref):
    return ((a - ref + 180.0) % 360.0) - 180.0


def fmt_vec(v, nd=8):
    return "(" + ", ".join("{:.{}f}".format(float(x), nd) for x in v) + ")"


# ----------------------------------------------------------------------
# Programme principal
# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Calibration caméra 3D -> base Kinova")
    ap.add_argument("--cam", default="points_cam.csv")
    ap.add_argument("--base", default="poses_base.csv")
    ap.add_argument("--test", default="", help="idx de points de TEST exclus du calcul, ex: 4,9,14")
    ap.add_argument("--offset-marqueur", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                    metavar=("DX", "DY", "DZ"),
                    help="décalage (m, repère base) centre du marqueur - point outil, à orientation "
                         "outil constante. Défaut : 0 (marqueur au point outil)")
    ap.add_argument("--centre-base", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"),
                    help="centre voulu de la zone de retrait dans la base Kinova (m)")
    ap.add_argument("--centre-idx", type=int, default=None, metavar="IDX",
                    help="utilise le point IDX (ex. capturé à la pose 'give', décalage marqueur inclus) "
                         "comme centre de la zone de retrait")
    ap.add_argument("--rayon", type=float, default=0.30, help="rayon de la zone de retrait (m)")
    ap.add_argument("--sortie", default="calibration_resultat.txt")
    args = ap.parse_args()

    cam = lire_csv(args.cam)
    base = lire_csv(args.base)
    communs = sorted(set(cam) & set(base))
    orphelins = sorted(set(cam) ^ set(base))
    if orphelins:
        print("[ATTENTION] idx présents d'un seul côté (ignorés) : {}".format(orphelins))

    ids_test = set(int(s) for s in args.test.split(",") if s.strip())
    ids_fit = [i for i in communs if i not in ids_test]
    ids_test = [i for i in communs if i in ids_test]

    if len(ids_fit) < 6:
        print("[ERREUR] {} points pour le calcul : il en faut au moins 6 (idéalement 12 à 20).".format(len(ids_fit)))
        sys.exit(1)

    def vec(d, i, cles):
        return [float(d[i][c]) for c in cles]

    P = np.array([vec(cam, i, "xyz") for i in communs])
    Qtool = np.array([vec(base, i, "xyz") for i in communs])
    Q = Qtool + np.array(args.offset_marqueur)
    idx_pos = {i: k for k, i in enumerate(communs)}
    k_fit = [idx_pos[i] for i in ids_fit]
    k_test = [idx_pos[i] for i in ids_test]

    # --- Qualité des données ---
    print("=" * 66)
    print("DONNEES : {} points appariés ({} pour le calcul, {} de test)".format(
        len(communs), len(ids_fit), len(ids_test)))
    Pf = P[k_fit]
    sv = np.linalg.svd(Pf - Pf.mean(axis=0), compute_uv=False)
    etendue = Pf.max(axis=0) - Pf.min(axis=0)
    print("Etendue des points (x,y,z caméra) : {:.2f} / {:.2f} / {:.2f} m".format(*etendue))
    if sv[2] / sv[0] < 0.15:
        print("[ATTENTION] Points presque coplanaires (rapport {:.2f}) : rotation mal conditionnée. "
              "Répartis les points dans tout le volume.".format(sv[2] / sv[0]))
    if etendue.max() < 0.25:
        print("[ATTENTION] Volume couvert < 25 cm : étends les points à toute la zone de retrait.")

    std_cam = [float(cam[i].get("std_mm", 0) or 0) for i in communs]
    if max(std_cam) > 3.0:
        print("[ATTENTION] Bruit caméra élevé sur certains points (max {:.1f} mm).".format(max(std_cam)))

    if all(c in base[communs[0]] for c in ("thx", "thy", "thz")):
        ang = np.array([vec(base, i, ("thx", "thy", "thz")) for i in communs])
        moy = np.array([np.degrees(np.angle(np.mean(np.exp(1j * np.radians(ang[:, a]))))) for a in range(3)])
        dev = np.abs(ecart_angle_deg(ang, moy)).max()
        print("Dispersion de l'orientation outil : {:.1f} deg max".format(dev))
        if dev > 3.0 and np.linalg.norm(args.offset_marqueur) > 1e-6 or dev > 10.0:
            print("[ATTENTION] L'orientation de l'outil varie trop : le décalage marqueur-outil "
                  "n'est plus constant dans la base. Garde l'orientation fixe pendant la collecte.")

    # --- Ajustement ---
    A, T, echelle = calibrer(P[k_fit], Q[k_fit])
    e_fit = erreurs_mm(A, T, P[k_fit], Q[k_fit])
    print("=" * 66)
    print("RESULTAT (ajustement rigide)   det(A) = {:+.3f}   échelle estimée = {:.4f}".format(
        np.linalg.det(A), echelle))
    if abs(echelle - 1.0) > 0.02:
        print("[ATTENTION] Echelle à {:+.1f} % de 1 : mesure de profondeur biaisée ou mauvais "
              "appariement. Vérifie les points avant d'utiliser le résultat.".format((echelle - 1.0) * 100.0))
    print("Erreur sur les points du calcul : RMS {:.1f} mm, max {:.1f} mm".format(rms(e_fit), e_fit.max()))

    # --- Leave-one-out ---
    e_loo = None
    if len(ids_fit) >= 7:
        e_loo = []
        for j in range(len(k_fit)):
            sel = [k for jj, k in enumerate(k_fit) if jj != j]
            Aj, Tj, _ = calibrer(P[sel], Q[sel])
            e_loo.append(erreurs_mm(Aj, Tj, P[[k_fit[j]]], Q[[k_fit[j]]])[0])
        e_loo = np.array(e_loo)
        print("Validation croisée (leave-one-out) : RMS {:.1f} mm, max {:.1f} mm".format(rms(e_loo), e_loo.max()))

    # --- Points de test indépendants ---
    e_test = None
    if k_test:
        e_test = erreurs_mm(A, T, P[k_test], Q[k_test])
        print("Points de TEST indépendants : RMS {:.1f} mm, max {:.1f} mm  <- vraie précision".format(
            rms(e_test), e_test.max()))
    else:
        print("[CONSEIL] Aucun point de test indépendant : refais 3 à 4 poses, collecte-les, puis relance "
              "avec --test pour mesurer la vraie précision.")

    print("-" * 66)
    print("idx   err calcul (mm)   err LOO (mm)   err test (mm)")
    for i in communs:
        k = idx_pos[i]
        if i in ids_fit:
            j = ids_fit.index(i)
            s_loo = "{:8.1f}".format(e_loo[j]) if e_loo is not None else "       -"
            print("{:3d}   {:14.1f}   {}   {:>12}".format(i, e_fit[j], s_loo, "-"))
        else:
            j = ids_test.index(i)
            print("{:3d}   {:>14}   {:>8}   {:12.1f}".format(i, "-", "-", e_test[j]))

    # --- Comparaison avec l'ancienne calibration ---
    e_old = erreurs_mm(A_OLD, T_OLD, P, Q)
    print("-" * 66)
    print("Ancienne calibration sur ces mesures : RMS {:.1f} mm, max {:.1f} mm".format(rms(e_old), e_old.max()))

    ref = rms(e_test) if e_test is not None else (rms(e_loo) if e_loo is not None else rms(e_fit))
    if ref <= SEUIL_BON_MM:
        verdict = "BON (<= {:.0f} mm)".format(SEUIL_BON_MM)
    elif ref <= SEUIL_LIMITE_MM:
        verdict = "LIMITE ({:.0f}-{:.0f} mm) : cherche la cause avant d'utiliser".format(SEUIL_BON_MM, SEUIL_LIMITE_MM)
    else:
        verdict = "MAUVAIS (> {:.0f} mm) : NE PAS UTILISER, refaire la collecte".format(SEUIL_LIMITE_MM)
    print("Verdict (sur {}) : {}".format("test" if e_test is not None else "LOO/calcul", verdict))
    print("Rappel : le don s'arrête à 40 mm de la main ; vise une erreur très inférieure.")

    # --- Blocs à coller ---
    out = []
    out.append("# Calibration caméra 3D -> base Kinova (calib_calcul.py)")
    out.append("# RMS calcul {:.1f} mm | LOO {} | test {}".format(
        rms(e_fit),
        "{:.1f} mm".format(rms(e_loo)) if e_loo is not None else "n/a",
        "{:.1f} mm".format(rms(e_test)) if e_test is not None else "n/a"))
    out.append("_CAM_A = (")
    for r in A:
        out.append("    " + fmt_vec(r) + ",")
    out.append(")")
    out.append("_CAM_T = " + fmt_vec(T))
    out.append("")
    if k_test:
        sel_t = k_test[:4]
        out.append("# tests de _self_test_transform : points de TEST indépendants (mesure caméra, mesure base)")
    else:
        pas = max(1, len(k_fit) // 4)
        sel_t = k_fit[::pas][:4]
        out.append("# tests de _self_test_transform : points DEJA utilisés dans le calcul (à remplacer par des "
                   "points de test indépendants)")
    out.append("tests = [")
    for k in sel_t:
        out.append("    (({:.3f}, {:.3f}, {:.3f}), ({:.3f}, {:.3f}, {:.3f})),".format(*P[k], *Q[k]))
    out.append("]")

    # --- Zone de retrait ---
    if args.centre_base is not None or args.centre_idx is not None:
        if args.centre_idx is not None:
            if args.centre_idx not in idx_pos:
                print("[ERREUR] --centre-idx {} absent des points appariés.".format(args.centre_idx))
                sys.exit(1)
            cb = Q[idx_pos[args.centre_idx]]
        else:
            cb = np.array(args.centre_base)
        c_cam = np.linalg.inv(A) @ (cb - T)
        bas = cb - args.rayon
        haut = cb + args.rayon
        out.append("")
        out.append("# Zone de retrait : centre voulu dans la base = {}, rayon = {:.2f} m".format(
            fmt_vec(cb, 3), args.rayon))
        out.append("SPHERE_CENTER_CAM = ({:.4f}, {:.4f}, {:.4f})   # Camera3D.py (m, repère MQTT)".format(*c_cam))
        out.append("SPHERE_RADIUS_M = {:.2f}".format(args.rayon))
        if c_cam[2] - args.rayon < 0.30:
            out.append("# [ATTENTION] bord proche de la sphère à {:.2f} m de la caméra : profondeur ZED "
                       "peu fiable < ~0,3-0,4 m (à vérifier)".format(c_cam[2] - args.rayon))
        if np.any(bas < LIM_MIN) or np.any(haut > LIM_MAX):
            out.append("# [ATTENTION] la sphère dépasse l'espace de travail du bras (boîte "
                       "{} -> {}) : réduis le rayon ou déplace le centre. NE PAS élargir les limites."
                       .format(fmt_vec(LIM_MIN, 2), fmt_vec(LIM_MAX, 2)))

    bloc = "\n".join(out)
    print("=" * 66)
    print(bloc)
    with open(args.sortie, "w", encoding="utf-8") as f:
        f.write(bloc + "\n")
    print("=" * 66)
    print("Bloc écrit dans {}".format(args.sortie))


if __name__ == "__main__":
    main()
