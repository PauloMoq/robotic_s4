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
import math

WIN_W    = 1200
WIN_H    = 700
MARGE_PX = 30
SCALE    = 50
CELL     = 2.0
E        = 0.06

NB_OEUFS   = 3
RAYON_OEUF = 0.18

demi_x = (WIN_W / 2 - MARGE_PX) / SCALE
demi_y = (WIN_H / 2 - MARGE_PX) / SCALE

COLS = int(2 * demi_x / CELL)
ROWS = int(2 * demi_y / CELL)

def main():
    laby = Labyrinthe(COLS, ROWS, cell_size=CELL, epaisseur_mur=E)
    laby.generer()
    murs = laby.construire_murs()

    r_dep, c_dep = laby.cellule_aleatoire()
    x_dep, y_dep = laby.centre_cellule_monde(r_dep, c_dep)

    oeufs = laby.placer_oeufs(NB_OEUFS, rayon=RAYON_OEUF, cellule_exclue=(r_dep, c_dep))

    env = Environnement(
        largeur=WIN_W / SCALE,
        hauteur=WIN_H / SCALE,
        murs=murs,
        collectables=oeufs
    )

    robot             = RobotMobile(x=x_dep, y=y_dep, moteur=MoteurOmnidirectionnel())
    vue               = VuePygame(largeur=WIN_W, hauteur=WIN_H, scale=SCALE)
    lidar             = Lidar(nb_rayons=16, portee=6.0, pas=0.02)
    controleur_manuel = ControleurPygame(robot)

    rc_dep        = (r_dep, c_dep)
    controleur_auto = ControleurAutonome(
        robot, lidar, laby.passages,
        cell_size=CELL, x0=laby.x0, y0=laby.y0,
        cols=COLS, rows=ROWS,
        rc_depart=rc_dep,
        rc_arrivee=rc_dep,
        v_max=2.0, omega_max=3.0
    )
    premier_oeuf = controleur_auto.oeuf_le_plus_proche(rc_dep, oeufs, laby)
    controleur_auto.rc_arrivee = laby.monde_vers_cellule(premier_oeuf["x"], premier_oeuf["y"])
    controleur_auto._recalculer_chemin()

    mode_auto = True
    clock     = pygame.time.Clock()
    victoire  = False
    running   = True
    relancer  = False
    timer     = 0.0

    pygame.display.set_caption("04 — Chasse aux Œufs — Robot LIDAR")
    print("=== Chasse aux Œufs — Robot LIDAR ===")
    print(f"Grille {COLS}×{ROWS}, {NB_OEUFS} œufs cachés")
    print(f"Départ robot : cellule ({r_dep},{c_dep})")
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
                        running = False
                        relancer = True

            dt = clock.tick(60) / 1000.0
            if not victoire:
                timer += dt

            if not victoire:
                collecte_faite = False
                for oeuf in oeufs:
                    if not oeuf["collecte"]:
                        if math.hypot(robot.x - oeuf["x"], robot.y - oeuf["y"]) < CELL / 2:
                            oeuf["collecte"] = True
                            collecte_faite   = True
                            print(f"🥚 Œuf collecté en ({oeuf['x']:.1f}, {oeuf['y']:.1f})")

                if collecte_faite:
                    env.collectables = [o for o in oeufs if not o["collecte"]]
                    suivants = [o for o in oeufs if not o["collecte"]]
                    if suivants:
                        rc_robot = laby.monde_vers_cellule(robot.x, robot.y)
                        prochain = controleur_auto.oeuf_le_plus_proche(rc_robot, oeufs, laby)
                        controleur_auto.rc_arrivee = laby.monde_vers_cellule(prochain["x"], prochain["y"])
                        controleur_auto._recalculer_chemin()

                lidar.scanner(robot, env)

                if mode_auto:
                    commande = controleur_auto.calculer_commande()
                else:
                    commande = controleur_manuel.lire_commande()

                robot.commander(**commande)
                robot.mettre_a_jour(dt, env)

                if all(o["collecte"] for o in oeufs):
                    victoire = True
                    print("🏆 Tous les œufs ont été récupérés !")

            vue.screen.fill(vue.COLOR_BG)
            vue.dessiner_grille()
            vue.dessiner_murs(env)
            vue.dessiner_chemin(controleur_auto)
            vue.dessiner_lidar(robot, lidar)
            vue.dessiner_oeufs(oeufs)
            vue.dessiner_robot(robot)
            vue.dessiner_compteur_oeufs(oeufs, NB_OEUFS)
            vue.dessiner_timer(timer)
            vue.dessiner_mode_auto(mode_auto)
            vue.dessiner_controles(robot, avec_regenerer=True, avec_mode_auto=True)

            if victoire:
                vue.afficher_victoire_oeufs(timer)

            pygame.display.flip()

    except KeyboardInterrupt:
        print("\nArrêt via Terminal (Ctrl+C).")
    except Exception:
        import traceback
        print("\n--- ERREUR ---")
        traceback.print_exc()
    finally:
        if not relancer:
            pygame.quit()
            print("Fermeture propre effectuée.")
            sys.exit()

    pygame.quit()
    main()

if __name__ == "__main__":
    main()