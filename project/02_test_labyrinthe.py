import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="pygame.pkgdata")

from robot.model.RobotMobile import RobotMobile
from robot.model.Moteur import MoteurOmnidirectionnel, MoteurDifferentiel
from robot.controler.ControleurPygame import ControleurPygame
from robot.vue.VuePygame import VuePygame
from robot.model.Environnement import Environnement
from robot.model.Labyrinthe import Labyrinthe
import pygame
import sys

WIN_W    = 1200
WIN_H    = 700
MARGE_PX = 30
SCALE    = 50
CELL     = 2.0
E        = 0.06

demi_x = (WIN_W / 2 - MARGE_PX) / SCALE
demi_y = (WIN_H / 2 - MARGE_PX) / SCALE

COLS = int(2 * demi_x / CELL)
ROWS = int(2 * demi_y / CELL)

def main():
    laby = Labyrinthe(COLS, ROWS, cell_size=CELL, epaisseur_mur=E)
    laby.generer()
    murs = laby.construire_murs()

    r_dep, c_dep = laby.cellule_aleatoire()
    r_arr, c_arr = laby.cellule_aleatoire(exclure=(r_dep, c_dep))
    x_dep, y_dep = laby.centre_cellule_monde(r_dep, c_dep)
    x_arr, y_arr = laby.centre_cellule_monde(r_arr, c_arr)

    env = Environnement(
        largeur=WIN_W / SCALE,
        hauteur=WIN_H / SCALE,
        murs=murs,
        arrivee={"x": x_arr, "y": y_arr, "rayon": 0.4}
    )

    robot      = RobotMobile(x=x_dep, y=y_dep, moteur=MoteurOmnidirectionnel())
    vue        = VuePygame(largeur=WIN_W, hauteur=WIN_H, scale=SCALE)
    controleur = ControleurPygame(robot)

    clock    = pygame.time.Clock()
    timer    = 0.0
    victoire = False
    running  = True

    pygame.display.set_caption("02_test_labyrinthe.py")

    print("=== Labyrinthe Procédural (DFS) — Robot Mobile ===")
    print(f"Grille {COLS}×{ROWS}, cellules {CELL} unités")
    print(f"Départ  : cellule ({r_dep},{c_dep})")
    print(f"Arrivée : cellule ({r_arr},{c_arr})")
    print("Flèches : déplacement  |  Q/D : rotation  |  TAB : moteur  |  R : régénérer  |  ECHAP : quitter")

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    if event.key == pygame.K_r:
                        main()
                        return
                    if event.key == pygame.K_TAB:
                        if hasattr(robot.moteur, 'vx'):
                            robot.moteur = MoteurDifferentiel()
                            print("Moteur → Différentiel")
                        else:
                            robot.moteur = MoteurOmnidirectionnel()
                            print("Moteur → Omnidirectionnel")

            dt     = clock.tick(60) / 1000.0
            timer += dt

            if not victoire:
                commande = controleur.lire_commande()
                robot.commander(**commande)
                robot.mettre_a_jour(dt, env)

                if env.est_a_l_arrivee(robot.x, robot.y):
                    victoire = True
                    print("🏁 Arrivée atteinte !")

            vue.screen.fill(vue.COLOR_BG)
            vue.dessiner_grille()
            vue.dessiner_murs(env)
            vue.dessiner_arrivee(env)
            vue.dessiner_robot(robot)
            vue.dessiner_timer(timer)
            vue.dessiner_type_moteur(robot)
            vue.dessiner_controles(robot, avec_regenerer=True)

            if victoire:
                vue.afficher_victoire()

            pygame.display.flip()

    except KeyboardInterrupt:
        print("\nArrêt via Terminal (Ctrl+C).")
    finally:
        pygame.quit()
        print("Fermeture propre effectuée.")
        sys.exit()

if __name__ == "__main__":
    main()