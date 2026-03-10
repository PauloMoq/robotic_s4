import math

class Lidar:
    """
    Capteur laser monté sur le robot.
    Envoie N rayons régulièrement espacés autour du robot et calcule
    la distance au premier obstacle touché (mur AABB ou obstacle circulaire).

    Attributs publics après appel à scanner() :
        mesures : liste de (angle_absolu, distance, x_impact, y_impact)
                  dans l'ordre des rayons (sens trigonométrique)
    """

    def __init__(self, nb_rayons: int = 8, portee: float = 10.0, pas: float = 0.02):
        """
        nb_rayons : nombre de rayons émis (répartis sur 360°)
        portee    : distance maximale de détection (unités monde)
        pas       : pas d'avancement le long du rayon (précision vs perf)
        """
        self.nb_rayons = nb_rayons
        self.portee    = portee
        self.pas       = pas
        self.mesures   = []   # [(angle, dist, xi, yi), ...]

    # ── Intersection rayon / mur rectangulaire (AABB) ──────────────────────

    @staticmethod
    def _intersect_aabb(ox, oy, dx, dy, mur):
        """
        Calcule la distance d'intersection entre un rayon (ox,oy)+t*(dx,dy)
        et un rectangle AABB. Retourne la distance t > 0 ou None.
        Algorithme slab (Kay-Kajiya).
        """
        x0, y0 = mur["x"], mur["y"]
        x1, y1 = x0 + mur["w"], y0 + mur["h"]

        t_min, t_max = 0.0, math.inf

        for o, d, lo, hi in [(ox, dx, x0, x1), (oy, dy, y0, y1)]:
            if abs(d) < 1e-9:
                if o < lo or o > hi:
                    return None
            else:
                t1 = (lo - o) / d
                t2 = (hi - o) / d
                if t1 > t2:
                    t1, t2 = t2, t1
                t_min = max(t_min, t1)
                t_max = min(t_max, t2)
                if t_min > t_max:
                    return None

        return t_min if t_min > 1e-6 else (t_max if t_max > 1e-6 else None)

    # ── Intersection rayon / obstacle circulaire ────────────────────────────

    @staticmethod
    def _intersect_cercle(ox, oy, dx, dy, obs):
        """
        Intersecte le rayon avec un cercle (obs["x"], obs["y"], obs["rayon"]).
        Retourne la distance t > 0 ou None.
        """
        cx, cy, r = obs["x"], obs["y"], obs["rayon"]
        fx, fy = ox - cx, oy - cy
        a = dx*dx + dy*dy
        b = 2*(fx*dx + fy*dy)
        c = fx*fx + fy*fy - r*r
        disc = b*b - 4*a*c
        if disc < 0:
            return None
        sq = math.sqrt(disc)
        t = (-b - sq) / (2*a)
        if t > 1e-6:
            return t
        t = (-b + sq) / (2*a)
        return t if t > 1e-6 else None

    # ── Scan principal ──────────────────────────────────────────────────────

    def scanner(self, robot, env):
        """
        Lance tous les rayons depuis la position du robot et remplit self.mesures.
        Chaque mesure : (angle_absolu, distance, x_impact, y_impact)
        """
        self.mesures = []
        ox, oy = robot.x, robot.y

        for i in range(self.nb_rayons):
            angle = robot.orientation + (2 * math.pi * i / self.nb_rayons)
            dx = math.cos(angle)
            dy = math.sin(angle)

            dist_min = self.portee
            xi_min   = ox + dx * self.portee
            yi_min   = oy + dy * self.portee

            # Tester tous les murs (AABB)
            for mur in env.murs:
                t = self._intersect_aabb(ox, oy, dx, dy, mur)
                if t is not None and t < dist_min:
                    dist_min = t
                    xi_min   = ox + dx * t
                    yi_min   = oy + dy * t

            # Tester tous les obstacles circulaires
            for obs in env.obstacles:
                t = self._intersect_cercle(ox, oy, dx, dy, obs)
                if t is not None and t < dist_min:
                    dist_min = t
                    xi_min   = ox + dx * t
                    yi_min   = oy + dy * t

            self.mesures.append((angle, dist_min, xi_min, yi_min))