import math
from typing import Optional
from .Moteur import Moteur
class RobotMobile:

    _nb_robots = 0
    def __init__(self, x: float = 0.0, y: float = 0.0, orientation: float = 0.0, moteur: Optional[Moteur] = None):
        self.__x = x
        self.__y = y
        self.__orientation = orientation

        if moteur is not None and not RobotMobile.moteur_valide(moteur):
            raise TypeError("Le moteur doit être une instance de Moteur")

        self.moteur = moteur
        RobotMobile._nb_robots += 1

    @property
    def x(self) -> float:
        return self.__x

    @x.setter
    def x(self, value: float):
        self.__x = float(value)

    @property
    def y(self) -> float:
        return self.__y

    @y.setter
    def y(self, value: float):
        self.__y = float(value)

    @property
    def orientation(self) -> float:
        return self.__orientation

    @orientation.setter
    def orientation(self, value: float):
        self.__orientation = value % (2 * math.pi)

    def avancer(self, distance: float):
        self.__x += distance * math.cos(self.__orientation)
        self.__y += distance * math.sin(self.__orientation)

    def tourner(self, angle: float):
        self.orientation = self.__orientation + angle

    def afficher(self):
        print(self)

    def commander(self, **kwargs):
        if self.moteur is not None:
            self.moteur.commander(**kwargs)

    def mettre_a_jour(self, dt: float, env=None):
        if self.moteur is not None:
            ancienne_x, ancienne_y = self.x, self.y
            self.moteur.mettre_a_jour(self, dt)

            if env is not None:
                rayon_physique = 0.3
                if env.est_en_collision(self.x, self.y, rayon_physique):
                    self.x, self.y = ancienne_x, ancienne_y

                limit_x = env.largeur / 2
                limit_y = env.hauteur / 2

                if self.x > limit_x:
                    self.x = -limit_x
                elif self.x < -limit_x:
                    self.x = limit_x

                if self.y > limit_y:
                    self.y = -limit_y
                elif self.y < -limit_y:
                    self.y = limit_y

    def __str__(self):
        return f"Robot -> x={self.x:.2f}, y={self.y:.2f}, orientation={self.orientation:.2f}, moteur={self.moteur}"

    @staticmethod
    def moteur_valide(moteur) -> bool:
        return isinstance(moteur, Moteur)

    @classmethod
    def nombre_robots(cls) -> int:
        return cls._nb_robots