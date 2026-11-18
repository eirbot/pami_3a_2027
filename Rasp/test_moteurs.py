#!/usr/bin/env python3
"""Test manuel des moteurs, directement sur /dev/esp (sans main_robot.py).

    python3 test_moteurs.py                 # séquence : avance, recule, tourne à gauche, à droite
    python3 test_moteurs.py --vitesse 50    # plus lent (mm/s ; rotation en deg/s = 2 x vitesse)
    python3 test_moteurs.py --goto 200 0 0  # GoTo (x mm, y mm, cap deg) depuis la pose actuelle remise à 0
    python3 test_moteurs.py --ecoute        # n'envoie rien qui bouge : affiche l'odométrie

Chaque mouvement dure --duree secondes, puis le script affiche le déplacement mesuré par
l'odométrie de l'ESP (à comparer à l'attendu). Ctrl+C envoie H (arrêt) avant de quitter.
main_robot.py ne doit pas tourner (un seul programme sur /dev/esp) : ./stop.sh
Mettre le robot sur cales (roues en l'air) pour un premier essai.
"""
import argparse
import math
import sys
import time

import serial

PERIODE = 0.05  # s : consigne renvoyée toutes les 50 ms (watchdog ESP 300 ms, timeout V 500 ms)


class ESP:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0)
        # DTR/RTS bas : sinon l'ESP32 reste en reset ou démarre en mode bootloader
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        self.buf = b""
        self.pose = None
        time.sleep(1.5)  # l'ouverture du port peut redémarrer l'ESP : on le laisse démarrer
        self.ser.reset_input_buffer()

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def lire(self):
        """Lit ce qui est arrivé ; met à jour la pose ; renvoie les autres trames."""
        self.buf += self.ser.read(4096)
        autres = []
        while b"\n" in self.buf:
            ligne, self.buf = self.buf.split(b"\n", 1)
            ligne = ligne.decode(errors="ignore").strip()
            if ligne.startswith("T "):
                try:
                    self.pose = tuple(float(v) for v in ligne.split()[1:4])
                except ValueError:
                    autres.append(ligne)
            elif ligne:
                autres.append(ligne)
        return autres

    def attendre_pose(self, delai=4.0):
        t0 = time.time()
        while time.time() - t0 < delai:
            self.send("K")
            for t in self.lire():
                print(f"   <- {t}")
            if self.pose is not None:
                return self.pose
            time.sleep(PERIODE)
        sys.exit("Aucune odométrie reçue : ESP flashé avec le firmware moteurs ? Bon port ?")

    def maintenir(self, cmd, duree):
        """Envoie cmd toutes les PERIODE pendant duree secondes, affiche les trames reçues."""
        t0 = time.time()
        while time.time() - t0 < duree:
            self.send(cmd)
            for t in self.lire():
                print(f"   <- {t}")
            time.sleep(PERIODE)


def ecart(p0, p1):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    dth = (p1[2] - p0[2] + 180) % 360 - 180
    return math.hypot(dx, dy), dx, dy, dth


def mouvement(esp, nom, vx, omega, duree):
    print(f"\n== {nom} : V {vx:g} mm/s, {omega:g} deg/s pendant {duree:g} s")
    p0 = esp.attendre_pose()
    esp.maintenir(f"V {vx:g} {omega:g}", duree)
    esp.send("H")
    esp.maintenir("K", 0.8)  # laisse le robot s'arrêter et l'odométrie se stabiliser
    d, dx, dy, dth = ecart(p0, esp.pose)
    print(f"   attendu : {abs(vx) * duree:6.0f} mm, {omega * duree:+6.0f} deg (sans compter les rampes)")
    print(f"   mesuré  : {d:6.0f} mm (dx {dx:+.0f}, dy {dy:+.0f}), {dth:+6.1f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/esp")
    ap.add_argument("--vitesse", type=float, default=100, help="mm/s (défaut 100)")
    ap.add_argument("--duree", type=float, default=1.0, help="s par mouvement (défaut 1)")
    ap.add_argument("--goto", nargs=3, type=float, metavar=("X", "Y", "THETA"))
    ap.add_argument("--ecoute", action="store_true")
    a = ap.parse_args()

    esp = ESP(a.port)
    try:
        print("Pose initiale :", esp.attendre_pose())
        if a.ecoute:
            while True:
                esp.maintenir("K", 0.5)
                print("T", esp.pose)
        elif a.goto:
            esp.send("S 0 0 0")
            esp.maintenir("K", 0.3)
            x, y, th = a.goto
            print(f"\n== GoTo ({x:g}, {y:g}, {th:g} deg) depuis (0, 0, 0)")
            esp.send(f"G 1 {x:g} {y:g} {th:g} 0.5")  # facteur 0.5 : vitesses et accélérations réduites
            t0 = time.time()
            while time.time() - t0 < 30:
                esp.send("K")
                fin = [t for t in esp.lire() if t.split()[:2] in (["D", "1"], ["A", "1"])]
                if fin:
                    print("   <-", fin[0], "(terminé)" if fin[0][0] == "D" else "(annulé)")
                    break
                time.sleep(PERIODE)
            else:
                print("   pas de fin au bout de 30 s")
            esp.maintenir("K", 0.3)
            print("   pose finale :", esp.pose)
        else:
            v, w = a.vitesse, 2 * a.vitesse
            mouvement(esp, "Avance", v, 0, a.duree)
            mouvement(esp, "Recule", -v, 0, a.duree)
            mouvement(esp, "Tourne à gauche", 0, w, a.duree)
            mouvement(esp, "Tourne à droite", 0, -w, a.duree)
    except KeyboardInterrupt:
        print("\nInterrompu")
    finally:
        esp.send("H")
        esp.ser.close()


if __name__ == "__main__":
    main()
