import math

class Environnement:
    def __init__(self, largeur=16.0, hauteur=12.0, obstacles=None, murs=None, arrivee=None):
        self.largeur = largeur
        self.hauteur = hauteur
        self.obstacles = obstacles if obstacles is not None else []
        # Chaque mur : {"x": float, "y": float, "w": float, "h": float}
        # (x, y) = coin bas-gauche en coordonnées monde, w = largeur, h = hauteur
        self.murs = murs if murs is not None else []
        # Point d'arrivée : {"x": float, "y": float, "rayon": float}
        self.arrivee = arrivee

    def est_en_collision(self, x, y, rayon_robot):
        # Collision avec obstacles circulaires
        for obs in self.obstacles:
            distance = math.sqrt((x - obs["x"])**2 + (y - obs["y"])**2)
            if distance < (obs["rayon"] + rayon_robot):
                return True

        # Collision avec murs rectangulaires (AABB + rayon robot)
        for mur in self.murs:
            # Point le plus proche du centre du robot sur le rectangle
            closest_x = max(mur["x"], min(x, mur["x"] + mur["w"]))
            closest_y = max(mur["y"], min(y, mur["y"] + mur["h"]))
            distance = math.sqrt((x - closest_x)**2 + (y - closest_y)**2)
            if distance < rayon_robot:
                return True

        return False

    def est_a_l_arrivee(self, x, y, rayon_robot=0.3):
        """Retourne True si le robot a atteint la zone d'arrivée."""
        if self.arrivee is None:
            return False
        distance = math.sqrt((x - self.arrivee["x"])**2 + (y - self.arrivee["y"])**2)
        return distance < (self.arrivee["rayon"] + rayon_robot)