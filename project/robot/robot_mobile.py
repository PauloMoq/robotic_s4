import math

class RobotMobile:
    def __init__(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation

    def __str__(self):
        return f"(x={self.x:.2f}, y={self.y:.2f}, orientation={self.orientation:.2f} rad)"

    def avancer(self, pdistance):
        self.x = self.x+pdistance*math.cos(math.radians(self.orientation))
        self.y = self.y+pdistance*math.sin(math.radians(self.orientation))

    def tourner(self, pangle):
        self.orientation = self.orientation+pangle/2*math.pi

    # Getter : Permet d'acceder a l'attribut depuis l'exterieur de la classe.
    @property
    def get_x(self) -> float:
        return self.x

    # Setter : Permet la modification de l'attribut depuis l'exterieur de la classe.
    @get_x.setter
    def get_x(self, value: float):
        self.x = value
