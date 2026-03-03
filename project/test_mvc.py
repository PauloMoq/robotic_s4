import warnings
#ignore les UserWarning provenant de pygame.pkgdata
warnings.filterwarnings("ignore", category=UserWarning, module="pygame.pkgdata")

from robot.model.RobotMobile import *
from robot.model.Moteur import *
from robot.controler.ControleurPygame import *
from robot.vue.VuePygame import *
from robot.vue.VueTerminal import *
from robot.model.Environnement import *
import pygame
import sys

def main():
    env = Environnement(largeur=16.0,
                        hauteur=12.0,
                        obstacles=[
                            {"x": 3.0, "y": 2.0, "rayon": 0.8},
                            {"x": -4.0, "y": -1.0, "rayon": 1.2},
                            {"x": 0.0, "y": 4.0, "rayon": 0.5}
                        ])
    robot = RobotMobile(moteur=MoteurOmnidirectionnel())
    vue = VuePygame()
    controleur = ControleurPygame(robot)

    clock = pygame.time.Clock()

    running = True
    print("Simulation lancée.")
    print("- Quitter : Croix rouge, touche ECHAP ou Ctrl+C (dans la fenêtre)")

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

                    if event.key == pygame.K_c and (pygame.key.get_mods() & pygame.KMOD_CTRL):
                        print("Ctrl+C détecté dans la fenêtre.")
                        running = False

            dt = clock.tick(60) / 1000.0
            commande = controleur.lire_commande()
            robot.commander(**commande)
            robot.mettre_a_jour(dt, env)

            vue.screen.fill((30, 33, 36))
            vue.dessiner_grille()
            vue.dessiner_environnement(env)
            vue.dessiner_robot(robot)
            pygame.display.flip()

    except KeyboardInterrupt:
        print("\nArrêt via Terminal (Ctrl+C).")
    finally:
        pygame.quit()
        print("Fermeture propre effectuée.")
        sys.exit()

if __name__ == '__main__':
    main()