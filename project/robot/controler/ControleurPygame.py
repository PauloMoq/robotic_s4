import pygame
from robot.controler.ControleurTerminal import Controleur

class ControleurPygame(Controleur):
    def __init__(self, robot):
        super().__init__(robot)
        self.v_step = 2.0
        self.omega_step = 1.5

    def lire_commande(self):
        keys = pygame.key.get_pressed()
        commande = {}

        if hasattr(self.robot.moteur, 'vx'):
            vx, vy, omega = 0.0, 0.0, 0.0

            if keys[pygame.K_UP]:    vx = self.v_step
            if keys[pygame.K_DOWN]:  vx = -self.v_step
            if keys[pygame.K_LEFT]:  vy = self.v_step
            if keys[pygame.K_RIGHT]: vy = -self.v_step

            if keys[pygame.K_q]:     omega = self.omega_step
            if keys[pygame.K_d]:     omega = -self.omega_step

            commande = {"vx": vx, "vy": vy, "omega": omega}

        else:
            v, omega = 0.0, 0.0

            if keys[pygame.K_UP]:    v = self.v_step
            if keys[pygame.K_DOWN]:  v = -self.v_step
            if keys[pygame.K_LEFT]:  omega = self.omega_step
            if keys[pygame.K_RIGHT]: omega = -self.omega_step

            commande = {"v": v, "omega": omega}

        return commande