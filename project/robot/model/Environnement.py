import math

class Environnement:
    def __init__(self, largeur=16.0, hauteur=12.0, obstacles=None):
        self.largeur = largeur
        self.hauteur = hauteur
        self.obstacles = obstacles if obstacles is not None else []

    def est_en_collision(self, x, y, rayon_robot):
        for obs in self.obstacles:
            distance = math.sqrt((x - obs["x"])**2 + (y - obs["y"])**2)
            if distance < (obs["rayon"] + rayon_robot):
                return True
        return False