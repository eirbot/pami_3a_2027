"""Localisation par odométrie (Rasp/core/localisation.py), sans robot."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Rasp"))

from core.frames import angle_diff_deg  # noqa: E402
from core.localisation import Localisation  # noqa: E402


def _close(test, a, b, tol=1e-6):
    test.assertAlmostEqual(a[0], b[0], delta=tol)
    test.assertAlmostEqual(a[1], b[1], delta=tol)
    test.assertAlmostEqual(angle_diff_deg(a[2], b[2]), 0.0, delta=tol)


class TestTransformations(unittest.TestCase):
    def test_reset_rattrape_l_arrondi(self):
        loc = Localisation()
        loc.reset((1234.4, -567.6, 33.4), odo_pose=(1234, -568, 33))
        _close(self, loc.on_odom(0.0, 1234, -568, 33), (1234.4, -567.6, 33.4))

    def test_aller_retour_odo_table(self):
        loc = Localisation()
        loc.reset((500.0, 200.0, 90.0), odo_pose=(480.0, 260.0, 80.0))
        p = (1500.0, -800.0, -135.0)
        _close(self, loc.to_table(*loc.to_odom(*p)), p)

    def test_correction_suit_le_robot(self):
        # Après correction, un déplacement odo est reporté tel quel (rotation comprise)
        loc = Localisation()
        loc.reset((1000.0, 0.0, 90.0), odo_pose=(1000.0, 0.0, 0.0))
        _close(self, loc.on_odom(0.0, 1100.0, 0.0, 0.0), (1000.0, 100.0, 90.0))



class TestIncertitude(unittest.TestCase):
    def test_grandit_avec_la_distance(self):
        loc = Localisation()
        loc.reset((1000.0, 0.0, 0.0))
        s0 = loc.sigma()[0]
        for k in range(1, 11):
            loc.on_odom(k * 0.1, 1000.0 + 100.0 * k, 0.0, 0.0)   # 1 m
        self.assertAlmostEqual(loc.sigma()[0], (s0 ** 2 + 20.0 ** 2) ** 0.5, places=6)

    def test_reset_remet_a_zero(self):
        loc = Localisation()
        loc.reset((1000.0, 0.0, 0.0))
        loc.on_odom(0.0, 3000.0, 0.0, 0.0)
        loc.reset((500.0, 0.0, 0.0))
        self.assertAlmostEqual(loc.sigma()[0], 10.0)

    def test_pose_avant_odometrie(self):
        self.assertIsNone(Localisation().pose())


if __name__ == "__main__":
    unittest.main()
