import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="pygame.pkgdata")

from robot.model.RobotMobile import RobotMobile
from robot.model.Moteur import MoteurOmnidirectionnel
from robot.controler.ControleurPygame import ControleurPygame
from robot.controler.ControleurAutonome import ControleurAutonome
from robot.vue.VuePygame import VuePygame
from robot.model.Environnement import Environnement
from robot.model.Labyrinthe import Labyrinthe
from robot.model.Lidar import Lidar
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

    robot             = RobotMobile(x=x_dep, y=y_dep, moteur=MoteurOmnidirectionnel())
    vue               = VuePygame(largeur=WIN_W, hauteur=WIN_H, scale=SCALE)
    lidar             = Lidar(nb_rayons=8, portee=8.0, pas=0.02)
    controleur_manuel = ControleurPygame(robot)
    controleur_auto   = ControleurAutonome(
                            robot, lidar, laby.passages,
                            cell_size=CELL, x0=laby.x0, y0=laby.y0,
                            cols=COLS, rows=ROWS,
                            rc_depart=(r_dep, c_dep),
                            rc_arrivee=(r_arr, c_arr),
                            v_max=2.0, omega_max=3.0)

    mode_auto = True
    clock     = pygame.time.Clock()
    timer     = 0.0
    victoire  = False
    running   = True

    pygame.display.set_caption("03_test_lidar.py")

    print("=== Labyrinthe + LIDAR + Pilote Autonome ===")
    print(f"Grille {COLS}×{ROWS}, cellules {CELL} unités")
    print(f"Départ  : cellule ({r_dep},{c_dep})")
    print(f"Arrivée : cellule ({r_arr},{c_arr})")
    print("TAB : auto/manuel  |  R : régénérer  |  ECHAP : quitter")

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    if event.key == pygame.K_TAB:
                        mode_auto = not mode_auto
                        print(f"Mode : {'AUTONOME' if mode_auto else 'MANUEL'}")
                    if event.key == pygame.K_r:
                        main()
                        return

            dt     = clock.tick(60) / 1000.0
            timer += dt

            if not victoire:
                lidar.scanner(robot, env)

                if mode_auto:
                    commande = controleur_auto.calculer_commande()
                else:
                    commande = controleur_manuel.lire_commande()

                robot.commander(**commande)
                robot.mettre_a_jour(dt, env)

                if env.est_a_l_arrivee(robot.x, robot.y):
                    victoire = True
                    print("🏁 Arrivée atteinte !")

            vue.screen.fill(vue.COLOR_BG)
            vue.dessiner_grille()
            vue.dessiner_murs(env)
            vue.dessiner_chemin(controleur_auto)
            vue.dessiner_arrivee(env)
            vue.dessiner_lidar(robot, lidar)
            vue.dessiner_robot(robot)
            vue.dessiner_timer(timer)
            vue.dessiner_mode_auto(mode_auto)
            vue.dessiner_controles(robot, avec_regenerer=True, avec_mode_auto=True)

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