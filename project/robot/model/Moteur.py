from abc import ABC, abstractmethod
import math

class Moteur(ABC):
    @abstractmethod
    def commander(self, *args, **kwargs):
        pass

    @abstractmethod
    def mettre_a_jour(self, robot, dt: float):
        pass

class MoteurDifferentiel(Moteur):
    def __init__(self, v: float = 0.0, omega: float = 0.0):
        self.v = v
        self.omega = omega

    def commander(self, v: float, omega: float):
        self.v = float(v)
        self.omega = float(omega)

    def mettre_a_jour(self, robot, dt: float):
        theta = robot.orientation
        robot.orientation = theta + self.omega * dt
        robot.x = robot.x + self.v * math.cos(theta) * dt
        robot.y = robot.y + self.v * math.sin(theta) * dt

    def __str__(self):
        return "Différentiel"

class MoteurOmnidirectionnel(Moteur):
    def __init__(self, vx: float = 0.0, vy: float = 0.0, omega: float = 0.0):
        self.vx = vx
        self.vy = vy
        self.omega = omega

    def commander(self, vx: float, vy: float, omega: float):
        self.vx = float(vx)
        self.vy = float(vy)
        self.omega = float(omega)

    def mettre_a_jour(self, robot, dt: float):
        theta = robot.orientation
        robot.orientation = theta + self.omega * dt
        robot.x = robot.x + (self.vx * math.cos(theta) - self.vy * math.sin(theta)) * dt
        robot.y = robot.y + (self.vx * math.sin(theta) + self.vy * math.cos(theta)) * dt

    def __str__(self):
        return "Omnidirectionnel"