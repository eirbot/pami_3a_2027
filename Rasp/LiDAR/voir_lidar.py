#!/usr/bin/env python3
"""Vue du LiDAR dans le terminal, pour le tester à la main (marche en ssh, sans écran ni dépendance).

    cd Rasp && python3 LiDAR/voir_lidar.py                 # lance le driver et affiche les points
    python3 LiDAR/voir_lidar.py --portee 1000              # zoom : 1 m autour du robot
    python3 LiDAR/voir_lidar.py --texte                    # une ligne par tour (journal, pas de dessin)
    python3 LiDAR/voir_lidar.py --sans-driver              # le driver tourne déjà (lancé à la main)

Vue de dessus, dans le repère du robot : robot au centre (R), AVANT VERS LE HAUT (config
lidar.montage appliquée : si l'objet devant le robot apparaît ailleurs, le montage est faux).
Un point par secteur de 5° (distance minimale du secteur), cercles : `lidar.proche_mm` (arrêt
dans toutes les directions) et `lidar.seuil_mm` (arrêt devant / derrière). La ligne d'état donne
l'état d'évitement qu'enverrait le robot en mode MATCH (sans le délai de retour à LIBRE).

main_robot.py ne doit pas tourner : un seul programme peut lire /dev/lidar et le port UDP 8080.
"""

import argparse
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import time

ICI = os.path.dirname(os.path.abspath(__file__))
RASP = os.path.dirname(ICI)
sys.path.insert(0, RASP)
from core import obstacles  # noqa: E402

DRIVER_CPP = os.path.join(ICI, "lidar_udp.cpp")
DRIVER_BIN = os.path.join(ICI, "lidar_udp")
ETATS = {obstacles.CLEAR: "LIBRE", obstacles.STOP: "STOP", obstacles.FRONT: "AVANT BLOQUÉ",
         obstacles.BACK: "ARRIÈRE BLOQUÉ"}


def charger_config():
    try:
        with open(os.path.join(RASP, "config.json"), encoding="utf-8") as f:
            return json.load(f).get("lidar", {})
    except (OSError, ValueError) as e:
        print(f"config.json illisible ({e}) : valeurs par défaut")
        return {}


def lancer_driver():
    """Compile le driver si besoin, puis le lance. Renvoie le processus."""
    if not os.path.exists(DRIVER_BIN) or os.path.getmtime(DRIVER_CPP) > os.path.getmtime(DRIVER_BIN):
        print("Compilation du driver LiDAR...")
        subprocess.run(["g++", "-O3", DRIVER_CPP, "-o", DRIVER_BIN], check=True)
    return subprocess.Popen([DRIVER_BIN], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def dessiner(scan, cfg, portee_mm, largeur, hauteur):
    """Lignes de texte de la vue de dessus (2 colonnes par cellule : les caractères sont hauts)."""
    nx, ny = largeur // 2, hauteur
    mm_par_case = 2.0 * portee_mm / min(nx, ny)
    cx, cy = nx // 2, ny // 2
    grille = [[" "] * nx for _ in range(ny)]

    def case(x, y):
        """Repère robot tourné (avant = +y écran vers le haut) -> (colonne, ligne) ou None."""
        c, r = cx + round(x / mm_par_case), cy - round(y / mm_par_case)
        return (c, r) if 0 <= c < nx and 0 <= r < ny else None

    for rayon, car in ((float(cfg.get("seuil_mm", 550)), "."), (float(cfg.get("proche_mm", 420)), ":")):
        for k in range(360):
            a = math.radians(k)
            p = case(rayon * math.cos(a), rayon * math.sin(a))
            if p:
                grille[p[1]][p[0]] = car
    # pose (0, 0, 90) : l'avant du robot (+X robot) devient +Y, donc le haut de l'écran
    proche = None
    if scan.closest_dist > 0:
        proche = min(((scan.sectors[i], i) for i in range(len(scan.sectors)) if scan.sectors[i] > 0),
                     default=(0, None))[1]
    for i, x, y in obstacles.sector_points(scan, (0.0, 0.0, 90.0), cfg.get("montage")):
        p = case(x, y)
        if p:
            grille[p[1]][p[0]] = "@" if i == proche else "#"
        else:  # hors de la vue : flèche au bord, dans la bonne direction
            a = math.atan2(y, x)
            r = 0.98 * portee_mm
            p = case(r * math.cos(a), r * math.sin(a))
            if p:
                grille[p[1]][p[0]] = "+"
    grille[cy][cx] = "R"
    if cy > 0:
        grille[cy - 1][cx] = "^"
    return ["".join(c + (c if c in ".:" else " ") for c in ligne) for ligne in grille], mm_par_case


def etat_ligne(scan, cfg, tours_s):
    etat = obstacles.classify(scan, "MATCH", float(cfg.get("seuil_mm", 550)), float(cfg.get("proche_mm", 420)),
                              cfg.get("montage"))
    vus = sum(1 for d in scan.sectors if d > 0)
    if scan.closest_dist > 0:
        proche = f"plus proche {scan.closest_dist:5.0f} mm à {scan.closest_angle:5.1f}° (angle LiDAR)"
    else:
        proche = "rien vu"
    return f"{tours_s:4.1f} tours/s | secteurs vus {vus:2d}/{len(scan.sectors)} | {proche} | évitement : {ETATS[etat]}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--portee", type=float, default=2000.0, help="rayon affiché en mm (défaut 2000)")
    parser.add_argument("--texte", action="store_true", help="une ligne par tour au lieu du dessin")
    parser.add_argument("--sans-driver", action="store_true", help="ne pas lancer le driver (déjà lancé)")
    args = parser.parse_args()
    cfg = charger_config()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 8080))
    except OSError:
        sys.exit("Port UDP 8080 occupé : main_robot.py (ou une autre vue) tourne déjà. L'arrêter : ./stop.sh")
    sock.settimeout(2.0)

    driver = None
    if not args.sans_driver:
        out = subprocess.run(["pgrep", "-x", "lidar_udp"], capture_output=True, text=True).stdout.split()
        if out:
            sys.exit(f"Un driver lidar_udp tourne déjà (pid {' '.join(out)}) : l'arrêter, ou relancer avec --sans-driver")
        driver = lancer_driver()

    tours, t0, tours_s = 0, time.time(), 0.0
    try:
        while True:
            try:
                data = sock.recv(2048)
            except socket.timeout:
                msg = "Aucun tour reçu depuis 2 s"
                if driver is not None and driver.poll() is not None:
                    msg += f" : le driver s'est arrêté\n{driver.stdout.read()}"
                    print(msg)
                    return 1
                print(msg + " (LiDAR branché ? /dev/lidar existe ?)")
                continue
            scan = obstacles.parse_scan(data)
            if scan is None:
                print(f"Paquet de {len(data)} octets ignoré : driver d'une autre version ? (attendu {obstacles.PACKET_SIZE})")
                continue
            tours += 1
            now = time.time()
            if now - t0 >= 1.0:
                tours_s, tours, t0 = tours / (now - t0), 0, now
            ligne = etat_ligne(scan, cfg, tours_s)
            if args.texte:
                print(ligne, flush=True)
                continue
            taille = shutil.get_terminal_size((80, 30))
            lignes, echelle = dessiner(scan, cfg, args.portee, taille.columns, max(10, taille.lines - 3))
            sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(lignes) + "\n" + ligne +
                             f"\nR robot (^ avant)  # point  @ plus proche  + au-delà de {args.portee:.0f} mm  "
                             f": proche_mm  . seuil_mm   1 case = {echelle:.0f} mm   Ctrl+C pour quitter")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print()
    finally:
        if driver is not None:
            driver.terminate()  # le driver envoie STOP au LiDAR en recevant SIGTERM
            try:
                driver.wait(timeout=2)
            except subprocess.TimeoutExpired:
                driver.kill()
    return 0


if __name__ == "__main__":
    sys.exit(main())
