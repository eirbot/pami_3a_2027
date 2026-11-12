#!/usr/bin/env python3
"""Simulateur du robot : le VRAI code Python du robot (Rasp/) avec du faux matériel.

Usage (sur le PC de dev) :
    python3 tools/robot_sim.py                                # stratégie de config.json, équipe BLEUE
    python3 tools/robot_sim.py --strat 3b_pouss_plus --equipe JAUNE --speed 300

Lance un match (sans tirette) avec la stratégie choisie ; l'ESP moteurs simulé déplace
réellement la position (pose finale affichée). Le simulateur travaille sur une COPIE de Rasp/config.json : la config du dépôt n'est pas modifiée.

Sert aussi à tools/check_strategy.py et aux tests (test/test_strategies.py).
"""

import argparse
import json
import math
import os
import shutil
import sys
import tempfile
import threading
import time
import types

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
RASP = os.path.join(ROOT, "Rasp")


# ---------------------------------------------------------------------------
# Faux matériel
class FakeESPMotors:
    """ESP moteurs simulé : déplacements en ligne droite à vitesse constante (SIM_SPEED mm/s).
    Comme le vrai : odométrie dans son propre repère, GoTo converti par shared.loc.to_odom,
    trames « T » passées à shared.loc.on_odom (pose fusionnée dans shared.robot_pos)."""
    speed_mm_s = float(os.environ.get("SIM_SPEED", "600"))

    connected = True

    def __init__(self, *a, **k):
        self.calls = []
        self._stop = threading.Event()
        self.watchdog_count = 0
        self.watchdog_tripped = False
        self.last_motion_time = time.time()
        self._odo = None

    def start(self):
        pass

    def _shared(self):
        import shared
        return shared

    def _publish(self, x, y, theta):
        """Trame odométrie « T »."""
        shared = self._shared()
        self._odo = (x, y, theta)
        self.last_motion_time = time.time()
        tx, ty, tt = shared.loc.on_odom(time.time(), x, y, theta)
        shared.robot_pos.update({"x": tx, "y": ty, "theta": tt})

    def set_pos(self, x, y, theta):
        self.calls.append(("set_pos", round(x, 3), round(y, 3), round(theta, 3)))
        self._shared().set_known_pose(x, y, theta)
        self._publish(x, y, theta)

    def goto(self, x, y, theta, manual=False, vitesse=1.0):
        self.calls.append(("goto", round(x, 3), round(y, 3), round(theta, 3)))
        en_vitesse = getattr(self, "_v_thread", None)
        if en_vitesse:                      # un G annule la vitesse : on attend l'arrêt
            self._consigne = (0.0, 0.0, 0.0)
            en_vitesse.join(2.0)
        self._stop.clear()
        shared = self._shared()
        if self._odo is None:
            pos = shared.robot_pos
            shared.set_known_pose(pos["x"], pos["y"], pos["theta"])
            self._odo = (pos["x"], pos["y"], pos["theta"])
        x, y, theta = shared.loc.to_odom(x, y, theta)
        x0, y0, th = self._odo
        dist = math.hypot(x - x0, y - y0)
        duration = dist / self.speed_mm_s if self.speed_mm_s > 0 else 0.0
        steps = max(1, int(duration / 0.05))
        if dist > 1:
            th = math.degrees(math.atan2(y - y0, x - x0))
        for i in range(1, steps + 1):
            if self._stop.is_set():
                return True
            f = i / steps
            self._publish(x0 + (x - x0) * f, y0 + (y - y0) * f, th)
            if duration > 0:
                time.sleep(duration / steps)
        self._publish(x, y, theta)
        return True

    VELOCITY_TIMEOUT_S = 0.5      # comme le firmware (VELOCITY_TIMEOUT_MS)
    ACCEL_MM_S2 = 1400.0

    def set_speed(self, vx, vtheta):
        """Consigne « V » : intégrée à 50 Hz (rampe, arrêt sans nouvelle consigne), trames T à 10 Hz."""
        self.calls.append(("set_speed", round(vx), round(vtheta)))
        if vx or vtheta:
            self._stop.clear()
        self._consigne = (float(vx), float(vtheta), time.time())
        if (vx or vtheta) and not getattr(self, "_v_thread", None):
            self._v_thread = threading.Thread(target=self._integrer, daemon=True)
            self._v_thread.start()

    def _integrer(self):
        shared = self._shared()
        if self._odo is None:
            pos = shared.robot_pos
            self._odo = (pos["x"], pos["y"], pos["theta"])
        v = w = 0.0
        dt, n = 0.02, 0
        while True:
            cv, cw, t = self._consigne
            if time.time() - t > self.VELOCITY_TIMEOUT_S or self._stop.is_set():
                cv = cw = 0.0
            v += max(-self.ACCEL_MM_S2 * dt, min(self.ACCEL_MM_S2 * dt, cv - v))
            w = cw
            x, y, th = self._odo
            th += w * dt
            x += v * math.cos(math.radians(th)) * dt
            y += v * math.sin(math.radians(th)) * dt
            self._odo = (x, y, th)
            n += 1
            if n % 5 == 0:
                self._publish(x, y, th)
            if cv == 0.0 and cw == 0.0 and abs(v) < 1e-3:
                self._publish(x, y, th)
                self._v_thread = None
                return
            time.sleep(dt)

    def stop_robot(self):
        self.calls.append(("stop_robot",))
        self._consigne = (0.0, 0.0, time.time())
        self._stop.set()


class FakeActionneurs:
    """Actionneurs simulés : enregistre les appels."""
    ser = None

    def __init__(self, *a, **k):
        self.calls = []

    def start(self):
        pass

    def __getattr__(self, name):
        def rec(*a, **k):
            self.calls.append((name,) + a)
        return rec


def _module(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


def install_fakes():
    """Remplace les modules matériels absents du PC (à appeler AVANT d'importer Rasp/)."""
    if "psutil" not in sys.modules:
        try:
            import psutil  # noqa: F401
        except ImportError:
            _module("psutil", cpu_percent=lambda *a, **k: 0.0)
    if "cv2" not in sys.modules:
        try:
            import cv2  # noqa: F401
        except ImportError:
            _module("cv2")
    _module("interface_deplacement")
    _module("interface_deplacement.esp_motors", ESPMotors=FakeESPMotors)
    _module("interface_actionneur")
    _module("interface_actionneur.esp_actionneur", ESPActionneurs=FakeActionneurs)

    class NoCamera:
        def __init__(self, *a, **k):
            raise RuntimeError("pas de caméra en simulation")
    _module("utils.sensors.camera_libcamera", LibCamera=NoCamera)


def prepare_config(source=None, son=False):
    """Copie de Rasp/config.json (ou `source`), LiDAR et caméra coupés, dans un dossier temporaire.
    Son coupé par défaut (tests, check_strategy) : `son=True` pour l'entendre."""
    tmp = tempfile.mkdtemp(prefix="robot_sim_")
    path = os.path.join(tmp, "config.json")
    shutil.copy(source or os.path.join(RASP, "config.json"), path)
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    cfg.update({"lidar_mode": "OFF", "music_enabled": False,
                "audio": {**cfg.get("audio", {}), "enabled": bool(son)},
                "camera": {**cfg.get("camera", {}), "enabled": False}})
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=4)
    return path


def start_robot(config_path=None, tirette_inserted=False):
    """Démarre le code robot (tâches de fond, stratégie) avec du faux matériel. Renvoie le module shared."""
    os.environ["ROBOT_CONFIG"] = config_path or prepare_config()
    os.environ.setdefault("ROBOT_TELEMETRY_DIR", tempfile.mkdtemp(prefix="robot_sim_logs_"))
    install_fakes()
    if RASP not in sys.path:
        sys.path.insert(0, RASP)
    os.chdir(RASP)  # main_strat cherche les stratégies en relatif

    import hardware
    import shared
    import strat.main_strat as main_strat
    import main_robot
    from taches import background_loop
    from strat.actions import RobotActions

    hardware.start(shared.cfg)  # faux ESP (install_fakes)
    shared.match.effects = main_robot.RobotEffects()
    threading.Thread(target=background_loop, daemon=True).start()
    threading.Thread(target=main_strat.strat_loop, args=(RobotActions(),), daemon=True).start()
    shared.match.ready(tirette_inserted)
    return shared


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", help="config.json de départ (défaut : Rasp/config.json)")
    parser.add_argument("--strat", help="stratégie (défaut : static_strat de la config)")
    parser.add_argument("--equipe", choices=("BLEUE", "JAUNE"), help="équipe (défaut : celle de la config)")
    parser.add_argument("--speed", type=float, help="vitesse simulée en mm/s (défaut : 600)")
    parser.add_argument("--son", action="store_true", help="jouer les sons (coupés par défaut)")
    args = parser.parse_args()
    if args.speed:
        FakeESPMotors.speed_mm_s = args.speed

    config_path = prepare_config(args.config, son=args.son)
    print(f"[SIM] Config de simulation : {config_path}")
    shared = start_robot(config_path)
    changes = {"test_mode": True}   # le match s'arrête à la fin de la stratégie (pas d'attente des 100 s)
    if args.strat:
        changes.update({"strat_mode": "STATIC", "static_strat": args.strat})
    if args.equipe:
        changes["team"] = args.equipe
    shared.match.apply_config(changes)
    time.sleep(1.5)  # liste des stratégies (strat_loop, 1 Hz)
    print(f"[SIM] Match : stratégie {shared.state.get('strat_id')}, équipe {shared.state.get('team')} "
          "(Ctrl+C pour arrêter)")
    shared.match.start()
    try:
        while shared.match.running:
            time.sleep(0.2)
        print(f"[SIM] Fin du match ({shared.state.get('fsm_state')}), pose finale "
              f"({shared.robot_pos['x']:.0f}, {shared.robot_pos['y']:.0f}, {shared.robot_pos['theta']:.1f}°)")
    except KeyboardInterrupt:
        print("\n[SIM] Arrêt.")
        shared.match.stop()


if __name__ == "__main__":
    main()
