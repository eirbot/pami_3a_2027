# taches.py
import time
import psutil
import os

import hardware
from core import preflight
from shared import state, send_led_cmd
from utils import get_ip, get_battery_voltage, get_cpu_temp, get_battery_current, get_voltage_float
from utils import system_info

def checklist(shared, volts, now):
    """Entrées de core/preflight.py lues dans l'état du robot, puis évaluation."""
    esp, act = hardware.esp, hardware.actionneurs
    strat = state.get("strat_id")  # ce que strat/main_strat.py lancera
    try:
        temp = float(str(get_cpu_temp()).replace("°C", ""))
    except ValueError:
        temp = None
    cam_cfg = shared.cfg.get("camera", {})
    return preflight.evaluer({
        "esp_moteurs": bool(esp and esp.connected),
        "esp_moteurs_age_s": now - esp.last_rx_time if esp and getattr(esp, "last_rx_time", None) else None,
        "esp_actionneurs": bool(act and act.ser and act.ser.is_open),
        "lidar_mode": state.get("lidar_mode", "MATCH"),
        "lidar_age_s": now - shared.lidar_points_t if shared.lidar_points_t else None,
        "tension_v": volts if system_info.SENSOR_AVAILABLE else None,  # sans capteur : 12,4 V simulés
        "strategie": strat, "strategie_existe": not strat or str(strat) in shared.strategies_list,
        "equipe": state.get("team"), "tirette": state.get("tirette"), "test_mode": state.get("test_mode"),
        "match_running": state.get("match_running"),
        "camera_embarquee": cam_cfg.get("enabled", True), "camera_embarquee_ok": hardware.camera_running(),
        "temp_cpu": temp,
    })

def _afficher_checklist(checks, precedent):
    """Pas d'écran : chaque vérification qui change de niveau est écrite dans la console.
    (le niveau seul : un détail qui bouge, comme la tension, ne réécrit rien). `precedent` : {id: niveau}
    du tour précédent (None au premier). Renvoie l'état courant."""
    precedent = precedent if isinstance(precedent, dict) else {}
    courant = {c["id"]: c["niveau"] for c in checks}
    for c in checks:
        ancien = precedent.get(c["id"])
        if ancien == courant[c["id"]] or (ancien is None and c["niveau"] == preflight.OK):
            continue
        detail = f" : {c['detail']}" if c.get("detail") and c["niveau"] != preflight.OK else ""
        print(f"[CHECK] {c['niveau'].upper():4} {c['label']}{detail}")
    etait_pret = bool(precedent) and all(n != preflight.ERR for n in precedent.values())
    if preflight.pret(checks) and not etait_pret:
        print("[CHECK] Prêt pour le match (aucune erreur)")
    return courant


def background_loop():
    print("[TACHES] Boucle de fond démarrée.")
    last_sys_info_time = 0
    derniere_checklist = None
    
    while True:
        # 1. Chrono du match (fin à 100 s, mode test...) : géré par core/match_state.py
        import shared
        shared.match.tick()

        # 2. Infos système et santé, 1 Hz
        now = time.time()
        if now - last_sys_info_time >= 1.0:
            last_sys_info_time = now
            import shared
            devs = {
                'lidar': os.path.exists('/dev/lidar'),
                'esp': os.path.exists('/dev/esp'),
                'camera': hardware.camera_running()
            }
            
            volts = get_voltage_float()

            # --- Détection Batterie Faible (seuils dans core/preflight.py) ---
            if 3.0 <= volts < preflight.TENSION_ERR_V:
                 last_alert = state.get("last_bat_alert", 0)
                 
                 # On renvoie la commande toutes les 2 secondes pour être PRIORITAIRE sur les autres anims
                 if not state.get("bat_low", False) or (now - last_alert > 2.0):
                     if not state.get("bat_low", False):
                         print(f"[TASKS] BATTERIE FAIBLE ({volts}V) ! Alerte prioritaire.")
                         shared.match.set_battery_low(True)
                     
                     state["last_bat_alert"] = now
                     send_led_cmd("ANIM:BLINK:255,0,0,300") # Clignotement rapide rouge
            
            # Hystérésis pour le rétablissement
            elif volts > preflight.TENSION_WARN_V:
                 if state.get("bat_low", False):
                     print(f"[TASKS] Batterie rétablie ({volts}V).")
                     # Fin d'alerte : MatchState remet la couleur d'équipe
                     shared.match.set_battery_low(False)

            ip = get_ip()
            try:
                checks = checklist(shared, volts, now)
                derniere_checklist = _afficher_checklist(checks, derniere_checklist)
            except Exception as e:  # ne doit jamais arrêter le chrono du match
                checks = []
                if derniere_checklist != repr(e):
                    print(f"[TACHES] Check-list impossible : {e!r}")
                derniere_checklist = repr(e)
            shared.sys_info.update({
                'cpu': f"{psutil.cpu_percent()}%", 
                'temp': get_cpu_temp(),
                'volt': get_battery_voltage(), 
                'volt_float': volts,
                'current': get_battery_current(),
                'ip': ip,
                'rasp_ip': ip,
                'devs': devs,
                'checks': checks,
            })

        time.sleep(0.1)  # 10 Hz (suffisant pour le chronomètre de match)