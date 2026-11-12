"""Localisation par odométrie (pur Python, testé sans robot).

L'ESP garde SON odométrie (repère « odo ») : on n'y touche pas pendant le match (le firmware
ignore un GoTo pendant un mouvement et « S » arrondit theta au degré). La Rasp tient une
transformation rigide C (repère odo -> repère table), posée par set_pos :
    pose_table = C(pose_odo)
Les GoTo sont convertis dans le repère odo avant envoi (to_odom).

Incertitude (écarts-types, config `localisation`) : `odo_sigma_mm_par_m` (position),
`odo_sigma_deg_par_m` / `odo_sigma_deg_par_tour` (cap) ; la variance grandit avec la distance
parcourue et repart de `sigma_init_*` à chaque set_pos. Sert à savoir si la pose est encore
fiable (shared.pose_fiable : filtre LiDAR hors table).
"""

import math
import threading

from core.frames import angle_diff_deg, apply_rigid, normalize_deg

DEFAULTS = {
    "odo_sigma_mm_par_m": 20.0,
    "odo_sigma_deg_par_m": 1.0,
    "odo_sigma_deg_par_tour": 3.0,
    "sigma_init_mm": 10.0,
    "sigma_init_deg": 1.0,
}

P_MAX_MM2 = 500.0 ** 2   # plafonds de variance : au-delà, l'estimation ne veut plus rien dire
P_MAX_DEG2 = 45.0 ** 2


def _compose(c, x, y, theta):
    """Applique C = (dx, dy, dtheta) à une pose : rotation de dtheta autour de l'origine puis translation."""
    return apply_rigid(x, y, theta, c[0], c[1], c[2])


def _inverse(c, x, y, theta):
    """Applique C^-1 : pose table -> pose odo."""
    r = math.radians(-c[2])
    tx, ty = x - c[0], y - c[1]
    return (tx * math.cos(r) - ty * math.sin(r),
            tx * math.sin(r) + ty * math.cos(r),
            normalize_deg(theta - c[2]))


def _from_pair(odo, table):
    """C telle que C(odo) = table."""
    dtheta = angle_diff_deg(table[2], odo[2])
    r = math.radians(dtheta)
    rx = odo[0] * math.cos(r) - odo[1] * math.sin(r)
    ry = odo[0] * math.sin(r) + odo[1] * math.cos(r)
    return (table[0] - rx, table[1] - ry, dtheta)


class Localisation:
    """Pose du robot dans le repère table. Thread-safe (odométrie et stratégie l'appellent
    depuis des threads différents)."""

    def __init__(self, get_cfg=None):
        self._get_cfg = get_cfg or (lambda: {})
        self._lock = threading.Lock()
        self._c = (0.0, 0.0, 0.0)
        self._odo = None              # dernière pose odo
        self._p_xy = 0.0
        self._p_th = 0.0
        self._reset_variance()

    def cfg(self):
        out = dict(DEFAULTS)
        try:
            out.update(self._get_cfg() or {})
        except Exception:
            pass
        return out

    def _reset_variance(self):
        cfg = self.cfg()
        self._p_xy = cfg["sigma_init_mm"] ** 2
        self._p_th = cfg["sigma_init_deg"] ** 2

    def reset(self, table_pose, odo_pose=None, t=None):
        """Pose connue (set_pos, début de match). `odo_pose` = ce que l'ESP a réellement
        reçu (« S » arrondit à l'entier) : C rattrape l'arrondi. `t` : inutilisé (compatibilité)."""
        odo_pose = tuple(odo_pose if odo_pose is not None else table_pose)
        with self._lock:
            self._c = _from_pair(odo_pose, tuple(table_pose))
            self._odo = odo_pose
            self._reset_variance()

    def on_odom(self, t, x, y, theta):
        """Nouvelle trame odométrie « T ». Renvoie la pose table (x, y, theta)."""
        cfg = self.cfg()
        with self._lock:
            if self._odo is not None:
                dist_m = math.hypot(x - self._odo[0], y - self._odo[1]) / 1000.0
                tours = abs(angle_diff_deg(theta, self._odo[2])) / 360.0
                self._p_xy = min(P_MAX_MM2, self._p_xy + cfg["odo_sigma_mm_par_m"] ** 2 * dist_m)
                self._p_th = min(P_MAX_DEG2, self._p_th
                                 + cfg["odo_sigma_deg_par_m"] ** 2 * dist_m
                                 + cfg["odo_sigma_deg_par_tour"] ** 2 * tours)
            self._odo = (x, y, theta)
            return _compose(self._c, x, y, theta)

    def to_odom(self, x, y, theta):
        """Consigne en repère table -> repère odo de l'ESP."""
        with self._lock:
            return _inverse(self._c, x, y, theta)

    def to_table(self, x, y, theta):
        with self._lock:
            return _compose(self._c, x, y, theta)

    def pose(self):
        """Pose table courante, ou None avant la première trame odométrie."""
        with self._lock:
            return _compose(self._c, *self._odo) if self._odo else None

    def sigma(self):
        """Écarts-types courants (mm, degrés)."""
        with self._lock:
            return math.sqrt(self._p_xy), math.sqrt(self._p_th)

    def correction(self):
        """C = (dx, dy, dtheta) : écart entre le repère odo de l'ESP et la table."""
        with self._lock:
            return self._c
