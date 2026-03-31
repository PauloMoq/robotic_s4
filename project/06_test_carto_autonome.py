import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="pygame.pkgdata")

from robot.model.RobotMobile import RobotMobile
from robot.model.Moteur import MoteurOmnidirectionnel, MoteurDifferentiel
from robot.controler.ControleurPygame import ControleurPygame
from robot.controler.ControleurExploration import ControleurExploration
from robot.vue.VuePygame import VuePygame
from robot.model.Environnement import Environnement
from robot.model.Cartographie import Cartographie
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
    lidar             = Lidar(nb_rayons=16, portee=3.5, pas=0.02)
    carto             = Cartographie(ROWS, COLS, CELL, laby.x0, laby.y0,
                                     passages=laby.passages)
    controleur_manuel = ControleurPygame(robot)

    lidar.scanner(robot, env)
    carto.mettre_a_jour(robot, lidar, oeufs)

    controleur_auto = ControleurExploration(
        robot, lidar, carto, laby.passages, laby,
        rc_depart=(r_dep, c_dep),
        v_max=2.0, omega_max=3.0
    )

    mode_auto = True
    en_pause  = False
    clock     = pygame.time.Clock()
    victoire  = False
    running   = True
    relancer  = False
    timer     = 0.0

    pygame.display.set_caption("06 — Cartographie Autonome — Robot LIDAR")
    print("=== Cartographie Autonome — Robot LIDAR ===")
    print(f"Grille {COLS}×{ROWS}, {NB_OEUFS} œufs cachés")
    print(f"Départ robot : cellule ({r_dep},{c_dep})")
    print("TAB : auto/manuel  |  M : switch moteur  |  ESPACE : pause  |  R : régénérer  |  ECHAP : quitter")

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    if event.key == pygame.K_SPACE:
                        if not victoire:
                            en_pause = not en_pause
                            print(f"{'PAUSE' if en_pause else 'REPRISE'}")
                    if event.key == pygame.K_TAB:
                        mode_auto = not mode_auto
                        print(f"Mode : {'AUTONOME' if mode_auto else 'MANUEL'}")
                        if mode_auto:
                            controleur_auto._fixer_prochaine_cible()
                    if event.key == pygame.K_m:
                        if hasattr(robot.moteur, 'vx'):
                            robot.moteur = MoteurDifferentiel()
                            print("Moteur → Différentiel")
                        else:
                            robot.moteur = MoteurOmnidirectionnel()
                            print("Moteur → Omnidirectionnel")
                    if event.key == pygame.K_r:
                        running = False
                        relancer = True

            dt = clock.tick(60) / 1000.0

            if not victoire and not en_pause:
                timer += dt

                if mode_auto:
                    controleur_auto.mettre_a_jour_etat(oeufs)
                    commande = controleur_auto.calculer_commande()
                else:
                    commande = controleur_manuel.lire_commande()

                robot.commander(**commande)
                robot.mettre_a_jour(dt, env)

                lidar.scanner(robot, env)

                carto.mettre_a_jour(robot, lidar, oeufs)

                collecte_faite = False
                for oeuf in oeufs:
                    if not oeuf["collecte"]:
                        if math.hypot(robot.x - oeuf["x"], robot.y - oeuf["y"]) < CELL / 2:
                            oeuf["collecte"] = True
                            collecte_faite   = True
                            print(f"🥚 Œuf collecté en ({oeuf['x']:.1f}, {oeuf['y']:.1f})")

                if collecte_faite:
                    env.collectables = [o for o in oeufs if not o["collecte"]]

                if all(o["collecte"] for o in oeufs):
                    victoire = True
                    print("🏆 Tous les œufs ont été récupérés !")

            vue.screen.fill(vue.COLOR_BG)
            vue.dessiner_grille()
            vue.dessiner_cartographie(carto)
            vue.dessiner_chemin(controleur_auto)
            vue.dessiner_lidar(robot, lidar)
            vue.dessiner_oeufs_detectes(carto)
            vue.dessiner_robot(robot)
            vue.dessiner_hud(oeufs, NB_OEUFS, carto)
            vue.dessiner_timer(timer)
            vue.dessiner_mode_auto(mode_auto)
            vue.dessiner_type_moteur(robot)
            vue.dessiner_etat_exploration(controleur_auto)
            vue.dessiner_controles(robot,
                                   avec_regenerer=True,
                                   avec_mode_auto=True,
                                   avec_pause=True,
                                   avec_switch_moteur=True)

            if en_pause and not victoire:
                vue.afficher_pause()

            if victoire:
                vue.afficher_victoire_oeufs(timer)

            pygame.display.flip()

    except KeyboardInterrupt:
        print("\nArrêt via Terminal (Ctrl+C).")
    except Exception:
        import traceback
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