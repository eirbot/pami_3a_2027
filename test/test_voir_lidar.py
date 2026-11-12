"""Vue terminal du LiDAR (Rasp/LiDAR/voir_lidar.py) : orientation et ligne d'état."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Rasp"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Rasp", "LiDAR"))
import voir_lidar  # noqa: E402
from core import obstacles  # noqa: E402


def scan_un_point(angle_lidar, dist):
    sectors = [0.0] * obstacles.SECTORS
    sectors[int(angle_lidar // obstacles.SECTOR_DEG)] = dist
    return obstacles.Scan(angle_lidar, dist, tuple(sectors))


def position(lignes, car):
    for r, ligne in enumerate(lignes):
        if car in ligne:
            return ligne.index(car) // 2, r
    return None


class TestVue(unittest.TestCase):
    CFG = {"seuil_mm": 550, "proche_mm": 420, "montage": {"x": 0, "y": 0, "angle": 0, "sens": -1}}

    def test_objet_devant_au_dessus_du_robot(self):
        lignes, _ = voir_lidar.dessiner(scan_un_point(0.0, 1000.0), self.CFG, 2000.0, 80, 30)
        (cr, rr), (cp, rp) = position(lignes, "R"), position(lignes, "@")
        self.assertLess(rp, rr)
        self.assertAlmostEqual(cp, cr, delta=1)

    def test_montage_tourne(self):
        # 0° du LiDAR vers la gauche du robot (angle 90) : l'objet est à gauche de R
        cfg = {**self.CFG, "montage": {"angle": 90, "sens": 1}}
        lignes, _ = voir_lidar.dessiner(scan_un_point(0.0, 1000.0), cfg, 2000.0, 80, 30)
        (cr, rr), (cp, rp) = position(lignes, "R"), position(lignes, "@")
        self.assertLess(cp, cr)
        self.assertAlmostEqual(rp, rr, delta=1)

    def test_hors_vue_au_bord(self):
        lignes, _ = voir_lidar.dessiner(scan_un_point(0.0, 5000.0), self.CFG, 2000.0, 80, 30)
        self.assertIsNone(position(lignes, "@"))
        self.assertIsNotNone(position(lignes, "+"))

    def test_etat(self):
        self.assertIn("AVANT BLOQUÉ", voir_lidar.etat_ligne(scan_un_point(0.0, 500.0), self.CFG, 10.0))
        self.assertIn("LIBRE", voir_lidar.etat_ligne(scan_un_point(0.0, 1500.0), self.CFG, 10.0))
        self.assertIn("rien vu", voir_lidar.etat_ligne(obstacles.Scan(0.0, 0.0, (0.0,) * 72), self.CFG, 0.0))


if __name__ == "__main__":
    unittest.main()
