"""Actions de haut niveau appelées par les stratégies : déplacements en ligne droite avec symétrie
d'équipe (l'ESP s'arrête seul devant un obstacle LiDAR), actions de jeu. Le matériel vient de hardware.py
(None en simulation : chaque action le tolère)."""
import math
import time

import hardware
import shared
from core import frames

MANUEL_RELANCES_S = 20.0  # GoTo hors match bloqué par un obstacle : abandon après ce délai


class RobotActions:
    def __init__(self):
        self.base_pos = None  # première pose set_pos (côté BLEUE), pour GoBase
        self.approche_reussie = False

    @property
    def is_yellow(self):
        return shared.state["team"] == "JAUNE"

    def _check_abort(self, manual=False):
        """Interrompt la stratégie (exception) si le match est arrêté."""
        if not manual and not shared.state["match_running"]:
            raise Exception("Stop")

    def _apply_sym(self, x, y, theta=None):
        """Coordonnées écrites côté BLEUE -> coordonnées réelles de l'équipe courante (core/frames.py)."""
        return frames.mirror_for_team(x, y, theta, team=shared.state["team"])
    
    def set_pos(self, x, y, theta):
        """
        Définit la position du robot (Triche / Recalage).
        Remet à zéro l'odométrie de l'ESP32 et la localisation.
        """
        # On mémorise la première position comme étant la "Base" pour le retour fin de match
        if self.base_pos is None:
            self.base_pos = (x, y, theta)
            print(f"[ACTION] Base enregistrée : ({x}, {y}, {theta}°)")

        # 1. Calcul de la position réelle (Symétrie équipe)
        real_x, real_y, real_theta = self._apply_sym(x, y, theta)
        
        print(f"[ACTION] SET_POS -> ({real_x}, {real_y}, {real_theta}°)")

        # 2. Envoi à l'ESP32 (Reset Odométrie) + remise à zéro de la localisation
        if hardware.esp:
            hardware.esp.set_pos(real_x, real_y, real_theta)
        else:
            shared.set_known_pose(real_x, real_y, real_theta)
            print("[SIMU] SET_POS virtuel (Pas de com)")


    def goto(self, x, y, theta, manual=False, real=False, retry=True, vitesse=1.0):
        """Déplacement en ligne droite (rotation, avance, rotation) : un GoTo de l'ESP.

        retry : relance tant que l'ESP l'annule (obstacle) ; sinon renvoie False au premier refus.
        Hors match (manual), abandon après MANUEL_RELANCES_S de refus (avant : relancé sans fin).
        vitesse : facteur ]0, 1] sur vitesses et accélérations (lent : sans glissement).
        """
        self._check_abort(manual=manual)

        if not real:
            real_x, real_y, real_theta = self._apply_sym(x, y, theta)
        else:
            real_x, real_y, real_theta = x, y, theta

        if not hardware.esp:
            print("[SIMU] GOTO virtuel (Pas de com)")
            return True
        shared.evenement("goto", x=round(real_x), y=round(real_y), theta=round(real_theta, 1))
        success = hardware.esp.goto(real_x, real_y, real_theta, manual=manual, vitesse=vitesse)
        premier_refus = time.time()
        while not success and retry:
            if manual and shared.state.get("fsm_state") == "STOPPED":
                break
            if manual and time.time() - premier_refus > MANUEL_RELANCES_S:
                print(f"[ACTION] GOTO abandonné : bloqué depuis plus de {MANUEL_RELANCES_S:.0f} s")
                break
            if not manual and not shared.state.get("match_running", False):
                break
            print(f"[ACTION] 🔄 GOTO annulé par l'ESP (Obstacle), recalcul et relance vers ({real_x}, {real_y}, {real_theta}°)")
            self._check_abort(manual=manual)
            time.sleep(0.5)
            success = hardware.esp.goto(real_x, real_y, real_theta, manual=manual, vitesse=vitesse)
        if not success:
            shared.evenement("goto_annule", x=round(real_x), y=round(real_y))
        return success

    # --- Caméra embarquée : se placer face à un tag (pierres 2027) ---------------------------
    def aligner_tag(self, ids=(13,), distance_mm=None, essais=3, tolerance_mm=10.0, tolerance_deg=3.0,
                    recul_mm=150.0, manual=False):
        """Se met face au tag le plus proche vu par la caméra embarquée, son point de référence à
        distance_mm de la face (défaut : camera.prise.distance_mm). Mesure à l'arrêt (médiane), petit
        GoTo, puis nouvelle mesure, jusqu'à `essais` fois. Si le décalage latéral est grand, recule
        d'abord (le robot ne se déplace pas de côté) puis revient dans l'axe du tag.
        Renvoie la dernière mesure (PoseTag, repère robot) si aligné, sinon None ; robot.approche_reussie."""
        self.approche_reussie = False
        if distance_mm is None:
            distance_mm = float(shared.cfg.get("camera", {}).get("prise", {}).get("distance_mm", 300.0))
        vision = hardware.vision
        if vision is None:
            print("[VISION] Caméra embarquée absente : alignement impossible")
            return None
        tag = None
        for _ in range(int(essais)):
            self._check_abort(manual=manual)
            tag = vision.mesurer(ids=list(ids), n=5, timeout=1.5)
            if tag is None:
                print(f"[VISION] Tag {list(ids)} non vu")
                shared.evenement("tag_non_vu", ids=list(ids))
                return None
            pose = shared.loc.pose()
            ex, ey, ecap = tag.point_devant(distance_mm)      # cible, repère robot
            ok = (abs(ey) < tolerance_mm and abs(ex) < tolerance_mm and abs(tag.ecart_angle) < tolerance_deg)
            print(f"[VISION] Tag {tag.id} : à {tag.distance:.0f} mm, latéral {tag.y:+.0f} mm, angle {tag.ecart_angle:+.1f}°"
                  f" -> correction ({ex:+.0f}, {ey:+.0f}, {ecap:+.1f}°)")
            shared.evenement("tag", id=tag.id, x=round(tag.x), y=round(tag.y), angle=round(tag.ecart_angle, 1))
            if ok or pose is None:
                break
            tx, ty, tcap = tag.vers_table(*pose)
            fx = tx + distance_mm * math.cos(math.radians(tcap))
            fy = ty + distance_mm * math.sin(math.radians(tcap))
            fcap = frames.normalize_deg(tcap + 180.0)
            if abs(ey) > 3 * tolerance_mm:
                rx = tx + (distance_mm + recul_mm) * math.cos(math.radians(tcap))
                ry = ty + (distance_mm + recul_mm) * math.sin(math.radians(tcap))
                self.goto(rx, ry, fcap, manual=manual, real=True)
            self.goto(fx, fy, fcap, manual=manual, real=True)
        else:
            tag = vision.mesurer(ids=list(ids), n=5, timeout=1.5)
        if tag is not None:
            ex, ey, _ = tag.point_devant(distance_mm)
            self.approche_reussie = abs(ey) < 2 * tolerance_mm and abs(tag.ecart_angle) < 2 * tolerance_deg
        return tag if self.approche_reussie else None

    def set_speed(self, vx, vtheta):
        """Consigne de vitesse pour contrôle joystick."""
        if hardware.esp:
            hardware.esp.set_speed(vx, vtheta)
        else:
            print(f"[SIMU] SET_SPEED ({vx}, {vtheta}) virtuel")

    def stop(self):
        print("[ACTION] STOP")
        if hardware.esp:
            hardware.esp.stop_robot()
        else:
            print("[SIMU] STOP virtuel (Pas de com)")

    def set_lidar_state(self, val):
        """Définit l'état du LiDAR sur l'ESP32 (0: Libre, 1: Stop, 2: Front, 3: Back)"""
        if hardware.esp:
            hardware.esp.set_lidar_state(val)
        else:
            print(f"[SIMU] SET LIDAR STATE {val} virtuel")

    # --- Actions de jeu ----------------------------------------------------------------
    # Actions de 2025 (Kapla, thermomètre) vidées : les noms restent (stratégies
    # existantes), le contenu est à redéfinir pour 2027.

    def _a_definir(self, nom):
        self._check_abort()
        print(f"[ACTION] {nom} : action à définir (2027)")

    def approcheKapla(self) -> bool:
        """Approche visuelle d'un élément de jeu. Retourne True si l'alignement a réussi."""
        self._a_definir("approcheKapla")
        return False

    def thermometre(self):
        self._a_definir("thermometre")

    def attendre(self, secondes: float):
        """Attend un nombre de secondes (interruptible par abort)."""
        self._check_abort()
        print(f"[ACTION] Attente de {secondes}s...")
        fin = time.time() + secondes
        while time.time() < fin:
            self._check_abort()
            time.sleep(0.05)

    def prendreKapla(self):
        self._a_definir("prendreKapla")

    def poseKapla(self):
        self._a_definir("poseKapla")

    def pousse_kapla(self):
        self._a_definir("pousse_kapla")

    def GoBase(self):
        print("⚡ RETOUR BASE")
        if self.base_pos:
            bx, by, bt = self.base_pos
            # On revient aux coordonnées de départ avec un angle inversé (180°)
            target_theta = (bt + 0)
            # Normalisation entre -180 et 180 (optionnel mais propre)
            while target_theta > 180: target_theta -= 360
            while target_theta <= -180: target_theta += 360
            
            self.goto(bx + 650, by + 100, target_theta)
            self.goto(bx, by + 100, target_theta)
        else:
            # Fallback historique si set_pos n'a pas été appelé
            self.goto(250, 0, 180)
        time.sleep(1)

    def wait_until_and_return(self, target_second):
        self._check_abort()
        print(f"[ACTION] Attente de la seconde {target_second} pour retour base...")
        while shared.state["match_running"]:
            elapsed = time.time() - shared.state["start_time"]
            if elapsed >= target_second:
                break
            time.sleep(0.5)
        self.GoBase()

    def play_animation(self, anim_name):
        self._check_abort()
        print(f"[ACTION] Playing Animation: {anim_name}")
        shared.send_led_cmd(f"PLAY:{anim_name}")

    def play_sound(self, sound_name):
        self._check_abort()
        print(f"[ACTION] Playing Sound: {sound_name}")
        shared.audio.play(sound_name)


    def cmd_actionneurs(self, command_string=None, act1=None, act2=None, act3=None, act4=None):
        """
        Accès au module Actionneur dédié.
        """
        self._check_abort()
        if hardware.actionneurs:
            if command_string is not None:
                hardware.actionneurs.send_raw(command_string)
            else:
                hardware.actionneurs.send_cmd(act1, act2, act3, act4)
        else:
            print("[SIMU] Pas d'interface Actionneur connectée (Mode sans matériel).")