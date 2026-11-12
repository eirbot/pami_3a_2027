"""Test d'intégration du code robot SANS matériel.

On importe les vrais modules (shared, strat.main_strat, main_robot) avec de faux modules
pour le matériel absent du PC (ESP, caméra, GPIO, psutil), puis on déroule des matchs
complets par les mêmes chemins de code qu'en vrai : machine d'état du match (appelée par les
boutons et la tirette), thread de stratégie, effets (moteurs, actionneurs).
"""
import importlib
import json
import os
import sys
import tempfile
import threading
import time
import types
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
RASP = os.path.join(ROOT, "Rasp")


# ---------------------------------------------------------------------------
# Faux matériel
class FakeESPMotors:
    instances = []

    def __init__(self, *a, **k):
        self.calls = []
        FakeESPMotors.instances.append(self)

    def start(self):
        pass

    def goto(self, x, y, theta, manual=False, vitesse=1.0):
        self.calls.append(("goto", round(x, 3), round(y, 3), round(theta, 3)))
        time.sleep(0.01)
        return True

    def set_pos(self, x, y, theta):
        self.calls.append(("set_pos", round(x, 3), round(y, 3), round(theta, 3)))

    def set_speed(self, vx, vtheta):
        self.calls.append(("set_speed", vx, vtheta))

    def stop_robot(self):
        self.calls.append(("stop_robot",))


class FakeActionneurs:
    def __init__(self, *a, **k):
        self.calls = []

    def start(self):
        pass

    def __getattr__(self, name):
        def rec(*a, **k):
            self.calls.append((name,) + a)
        return rec


def install_fakes():
    def module(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    module("psutil", cpu_percent=lambda *a, **k: 0.0)
    module("cv2")
    module("interface_deplacement")
    module("interface_deplacement.esp_motors", ESPMotors=FakeESPMotors)
    module("interface_actionneur")
    module("interface_actionneur.esp_actionneur", ESPActionneurs=FakeActionneurs)

    class FakeCam:
        def __init__(self, *a, **k):
            raise RuntimeError("pas de caméra en test")
    module("utils.sensors.camera_libcamera", LibCamera=FakeCam)


STRAT_SOURCE = '''
METADATA = {"name": "test_integration"}
def run(robot):
    robot.set_pos(150, 1150, 0)
    robot.goto(700, 1250, 90)
'''


class TestIntegrationRobot(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cfg_path = os.path.join(cls.tmp.name, "config.json")
        with open(os.path.join(RASP, "config.json"), encoding="utf-8") as f:
            cfg = json.load(f)
        cfg.update({"team": "BLEUE", "strat_mode": "STATIC", "static_strat": "test_integration",
                    "test_mode": False, "music_enabled": False,
                    "audio": {**cfg.get("audio", {}), "enabled": False}})
        with open(cfg_path, "w", encoding="utf-8") as f:
            json.dump(cfg, f)
        os.environ["ROBOT_CONFIG"] = cfg_path
        os.environ["ROBOT_TELEMETRY_DIR"] = cls.tmp.name

        install_fakes()
        sys.path.insert(0, RASP)
        os.chdir(RASP)  # main_strat cherche les stratégies en relatif
        # Stratégie de test, importée comme une vraie (strat.strategies.<nom>)
        cls.strat_file = os.path.join(RASP, "strat", "strategies", "test_integration.py")
        with open(cls.strat_file, "w", encoding="utf-8") as f:
            f.write(STRAT_SOURCE)

        cls.shared = importlib.import_module("shared")
        hardware = importlib.import_module("hardware")
        hardware.start(cls.shared.cfg)  # faux ESP (install_fakes)
        cls.actions = importlib.import_module("strat.actions")
        cls.main_strat = importlib.import_module("strat.main_strat")
        main_robot = importlib.import_module("main_robot")
        cls.shared.match.effects = main_robot.RobotEffects()
        cls.shared.match.ready(tirette_inserted=False)

        threading.Thread(target=cls.main_strat.strat_loop, args=(cls.actions.RobotActions(),), daemon=True).start()
        cls.esp = hardware.esp
        cls.act = hardware.actionneurs

    @classmethod
    def tearDownClass(cls):
        os.remove(cls.strat_file)
        cls.tmp.cleanup()

    def wait_for(self, pred, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            if pred():
                return True
            time.sleep(0.02)
        return False

    def run_match_and_stop(self):
        self.esp.calls.clear()
        self.shared.match.start()
        self.assertTrue(self.wait_for(lambda: any(c[0] == "goto" for c in self.esp.calls)),
                        "la stratégie n'a pas été lancée")
        self.assertTrue(self.shared.match.running)
        self.shared.match.stop()
        self.assertEqual(self.shared.match.phase, "STOPPED")
        return list(self.esp.calls)

    def test_1_match_bleue(self):
        self.assertEqual(self.shared.state["strat_id"], "test_integration")
        calls = self.run_match_and_stop()
        self.assertIn(("set_pos", 150, 1150, 0), calls)
        self.assertIn(("goto", 700, 1250, 90), calls)
        self.assertIn(("stop_robot",), self.esp.calls)
        self.assertTrue(any(c[0] == "bloquer" for c in self.act.calls))

    def test_2_equipe_jaune_symetrie(self):
        self.shared.match.reset()
        self.shared.update_config({"team": "JAUNE"})
        self.assertEqual(self.shared.state["team"], "JAUNE")
        with open(os.environ["ROBOT_CONFIG"], encoding="utf-8") as f:
            self.assertEqual(json.load(f)["team"], "JAUNE")  # persisté côté robot
        calls = self.run_match_and_stop()
        self.assertIn(("set_pos", 150, -1150, 0), calls)
        self.assertIn(("goto", 700, -1250, -90), calls)

    def test_3_reglages_refuses_pendant_match(self):
        self.shared.match.reset()
        self.shared.match.start()
        self.assertTrue(self.wait_for(lambda: self.shared.match.running))
        team = self.shared.state["team"]
        with self.assertRaises(self.shared.MatchError):     # bouton EQUIPE : refusé (message)
            self.shared.match.toggle_team()
        self.shared.update_config({"static_strat": "autre"})
        self.assertEqual(self.shared.state["team"], team)
        self.assertEqual(self.shared.state["strat_id"], "test_integration")
        self.shared.match.stop()

    def test_4_config_invalide_refusee(self):
        self.shared.update_config({"lidar_mode": "TURBO"})
        self.assertNotEqual(self.shared.config_store.get("lidar_mode"), "TURBO")

    def test_5_snapshot(self):
        snap = self.shared.match.snapshot()
        for key in ("fsm_state", "team", "strat_id", "timer_str", "tirette"):
            self.assertIn(key, snap)

    def test_6_luminosite_enregistree(self):
        """Luminosité : envoyée au service LED ET gardée dans la config (relue au redémarrage)."""
        sent = []
        original = self.shared.send_led_cmd
        self.shared.send_led_cmd = sent.append
        try:
            self.shared.update_config({"leds.luminosite": 0.35})
            _, err = self.shared.update_config({"leds.luminosite": 3.0})   # hors 0..1 : refusée
        finally:
            self.shared.send_led_cmd = original
        self.assertEqual(sent, ["BRIGHTNESS:0.350"])
        self.assertIsNotNone(err)
        self.assertAlmostEqual(self.shared.cfg["leds"]["luminosite"], 0.35)

if __name__ == "__main__":
    unittest.main()
