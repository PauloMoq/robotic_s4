import pygame
import math

class VuePygame:
    def __init__(self, largeur=800, hauteur=600, scale=50):
        pygame.init()
        self.largeur = largeur
        self.hauteur = hauteur
        self.scale = scale
        self.screen = pygame.display.set_mode((largeur, hauteur))
        pygame.display.set_caption("Simulation Robot Mobile")
        self.clock = pygame.time.Clock()

        #couleurs
        self.COLOR_BG = (30, 33, 36)  #gris très foncé
        self.COLOR_GRID = (50, 55, 60)  #grille subtile
        self.COLOR_ROBOT = (0, 150, 255)  #bleu clair
        self.COLOR_DIR = (255, 80, 80)  #rouge doux
        self.COLOR_BORDER = (100, 100, 100)  #bordure de map

    def convertir_coordonnees(self, x, y):
        px = int(self.largeur / 2 + (x * self.scale))
        py = int(self.hauteur / 2 - (y * self.scale))
        return px, py

    def dessiner_grille(self):
        for x in range(0, self.largeur, self.scale):
            pygame.draw.line(self.screen, self.COLOR_GRID, (x, 0), (x, self.hauteur))
        for y in range(0, self.hauteur, self.scale):
            pygame.draw.line(self.screen, self.COLOR_GRID, (0, y), (self.largeur, y))

    def dessiner_robot(self, robot):
        px, py = self.convertir_coordonnees(robot.x, robot.y)
        rayon_pixel = int(0.3 * self.scale)

        pygame.draw.circle(self.screen, self.COLOR_ROBOT, (px, py), rayon_pixel)
        pygame.draw.circle(self.screen, (255, 255, 255), (px, py), rayon_pixel, 1)

        x_dir = px + int(rayon_pixel * math.cos(robot.orientation))
        y_dir = py - int(rayon_pixel * math.sin(robot.orientation))
        pygame.draw.line(self.screen, self.COLOR_DIR, (px, py), (x_dir, y_dir), 4)

    def dessiner_environnement(self, env):
        for obs in env.obstacles:
            px, py = self.convertir_coordonnees(obs["x"], obs["y"])
            r_px = int(obs["rayon"] * self.scale)

            pygame.draw.circle(self.screen, (200, 50, 50), (px, py), r_px)
            pygame.draw.circle(self.screen, (255, 255, 255), (px, py), r_px, 1)

    def tick(self, fps=60):
        self.clock.tick(fps)