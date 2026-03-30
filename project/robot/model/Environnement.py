import math

class Environnement:
    def __init__(self, largeur=16.0, hauteur=12.0,
                 obstacles=None, murs=None, collectables=None, arrivee=None):
        self.largeur      = largeur
        self.hauteur      = hauteur
        self.obstacles    = obstacles    if obstacles    is not None else []
        self.murs         = murs         if murs         is not None else []
        self.collectables = collectables if collectables is not None else []
        self.arrivee      = arrivee

    def est_en_collision(self, x, y, rayon_robot):
        for obs in self.obstacles:
            if math.hypot(x - obs["x"], y - obs["y"]) < obs["rayon"] + rayon_robot:
                return True
        for mur in self.murs:
            mx, my = mur["x"], mur["y"]
            cx = max(mx, min(x, mx + mur["w"]))
            cy = max(my, min(y, my + mur["h"]))
            if math.hypot(x - cx, y - cy) < rayon_robot:
                return True
        return False

    def est_a_l_arrivee(self, x, y):
        if self.arrivee is None:
            return False
        return math.hypot(x - self.arrivee["x"], y - self.arrivee["y"]) < self.arrivee["rayon"]