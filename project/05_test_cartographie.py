import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="pygame.pkgdata")

from robot.model.RobotMobile import RobotMobile
from robot.model.Moteur import MoteurOmnidirectionnel
from robot.controler.ControleurPygame import ControleurPygame
from robot.vue.VuePygame import VuePygame
from robot.model.Environnement import Environnement
from robot.model.Cartographie import Cartographie
import pygame
import sys
import math
import random
from robot.model.Lidar import Lidar

# ---------------------------------------------------------------------------
# 05 — Cartographie progressive dans le noir
#
# Le labyrinthe est entièrement noir au départ.
# Le robot explore manuellement (flèches) ou en mode auto (TAB).
# Son LIDAR révèle progressivement les murs et les œufs environnants.
# Les œufs ne sont visibles que lorsque le LIDAR les a détectés.
# ---------------------------------------------------------------------------

WIN_W    = 1200
WIN_H    = 700
MARGE_PX = 30
SCALE    = 50
CELL     = 2.0
E        = 0.06

NB_OEUFS       = 3
RAYON_OEUF     = 0.18

demi_x = (WIN_W / 2 - MARGE_PX) / SCALE
demi_y = (WIN_H / 2 - MARGE_PX) / SCALE

COLS = int(2 * demi_x / CELL)
ROWS = int(2 * demi_y / CELL)

X0 = -COLS * CELL / 2
Y0 = -ROWS * CELL / 2


# ── Génération du labyrinthe (DFS) ──────────────────────────────────────────

def generer_labyrinthe(cols, rows, seed=None):
    if seed is not None:
        random.seed(seed)
    visited  = [[False] * cols for _ in range(rows)]
    passages = set()

    def voisins(r, c):
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                yield (nr, nc)

    r0, c0 = random.randint(0, rows - 1), random.randint(0, cols - 1)
    stack = [(r0, c0)]
    visited[r0][c0] = True

    while stack:
        r, c = stack[-1]
        vois = list(voisins(r, c))
        if vois:
            nr, nc = random.choice(vois)
            visited[nr][nc] = True
            passages.add((min((r, c), (nr, nc)), max((r, c), (nr, nc))))
            stack.append((nr, nc))
        else:
            stack.pop()

    return passages


def cellule_vers_monde(r, c):
    return X0 + c * CELL, Y0 + (ROWS - 1 - r) * CELL


def centre_cellule_monde(r, c):
    x, y = cellule_vers_monde(r, c)
    return x + CELL / 2, y + CELL / 2


def construire_murs(passages):
    murs = []

    def mh(x, y, w):
        if w > E: murs.append({"x": x, "y": y, "w": w, "h": E})

    def mv(x, y, h):
        if h > E: murs.append({"x": x, "y": y, "w": E, "h": h})

    larg, haut = COLS * CELL, ROWS * CELL
    mh(X0, Y0,        larg)
    mh(X0, Y0 + haut, larg)
    mv(X0,        Y0, haut)
    mv(X0 + larg, Y0, haut)

    for r in range(ROWS):
        for c in range(COLS):
            cx, cy = cellule_vers_monde(r, c)
            if r + 1 < ROWS:
                cle = (min((r, c), (r + 1, c)), max((r, c), (r + 1, c)))
                if cle not in passages:
                    mh(cx, cy, CELL)
            if c + 1 < COLS:
                cle = (min((r, c), (r, c + 1)), max((r, c), (r, c + 1)))
                if cle not in passages:
                    mv(cx + CELL, cy, CELL)

    return murs


def placer_oeufs(nb, cellule_exclue):
    cellules = set()
    while len(cellules) < nb:
        r = random.randint(0, ROWS - 1)
        c = random.randint(0, COLS - 1)
        if (r, c) != cellule_exclue:
            cellules.add((r, c))
    return [{"x": cx, "y": cy, "rayon": RAYON_OEUF, "collecte": False}
            for (r, c) in cellules
            for cx, cy in [centre_cellule_monde(r, c)]]


# ── Patch VuePygame — méthodes de cartographie ──────────────────────────────
# On ajoute les méthodes directement sur la classe pour ne pas modifier
# le fichier source original.

def _dessiner_cartographie(self, carto):
    """
    Recouvre d'un voile noir opaque toutes les cellules non encore découvertes.
    À appeler APRÈS avoir dessiné les murs, AVANT le robot.
    """
    taille_px = int(carto.cell * self.scale) + 1

    for r in range(carto.rows):
        for c in range(carto.cols):
            if not carto.est_decouverte(r, c):
                x_monde = carto.x0 + c * carto.cell
                y_monde = carto.y0 + (carto.rows - 1 - r) * carto.cell
                px, py  = self.convertir_coordonnees(x_monde, y_monde + carto.cell)
                rect    = pygame.Rect(px, py, taille_px, taille_px)
                pygame.draw.rect(self.screen, (0, 0, 0), rect)

def _dessiner_oeufs_detectes(self, carto, rayon_oeuf=0.18):
    """Affiche uniquement les œufs que le LIDAR a déjà repérés."""
    for oeuf in carto.oeufs_detectes:
        if oeuf.get("collecte"):
            continue
        px, py = self.convertir_coordonnees(oeuf["x"], oeuf["y"])
        rw = int(rayon_oeuf * self.scale * 1.1)
        rh = int(rayon_oeuf * self.scale * 1.5)
        rect = pygame.Rect(px - rw, py - rh, rw * 2, rh * 2)
        pygame.draw.ellipse(self.screen, (255, 230, 60), rect)
        pygame.draw.ellipse(self.screen, (255, 255, 255), rect, 2)

VuePygame.dessiner_cartographie    = _dessiner_cartographie
VuePygame.dessiner_oeufs_detectes  = _dessiner_oeufs_detectes


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    passages = generer_labyrinthe(COLS, ROWS)
    murs     = construire_murs(passages)

    r_dep = random.randint(0, ROWS - 1)
    c_dep = random.randint(0, COLS - 1)
    x_dep, y_dep = centre_cellule_monde(r_dep, c_dep)

    oeufs = placer_oeufs(NB_OEUFS, cellule_exclue=(r_dep, c_dep))

    env = Environnement(
        largeur=WIN_W / SCALE,
        hauteur=WIN_H / SCALE,
        murs=murs,
        collectables=oeufs
    )

    robot      = RobotMobile(x=x_dep, y=y_dep, moteur=MoteurOmnidirectionnel())
    vue        = VuePygame(largeur=WIN_W, hauteur=WIN_H, scale=SCALE)
    lidar      = Lidar(nb_rayons=16, portee=3.5, pas=0.02)
    carto      = Cartographie(ROWS, COLS, CELL, X0, Y0)
    controleur = ControleurPygame(robot)

    clock    = pygame.time.Clock()
    victoire = False
    running  = True
    relancer = False
    timer    = 0.0

    pygame.display.set_caption("Cartographie — Labyrinthe dans le noir")
    print("=== Cartographie Progressive — Labyrinthe dans le noir ===")
    print(f"Grille {COLS}×{ROWS}, {NB_OEUFS} œufs cachés")
    print(f"Départ robot : cellule ({r_dep},{c_dep})")
    print("Flèches : déplacement  |  Q/D : rotation  |  R : regénérer  |  ECHAP : quitter")

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    if event.key == pygame.K_r:
                        running = False
                        relancer = True

            dt = clock.tick(60) / 1000.0

            if not victoire:
                timer += dt

                # 1. Commande manuelle
                commande = controleur.lire_commande()
                robot.commander(**commande)
                robot.mettre_a_jour(dt, env)

                # 2. Scan LIDAR
                lidar.scanner(robot, env)

                # 3. Mise à jour cartographie
                carto.mettre_a_jour(robot, lidar, oeufs)

                # 4. Collecte des œufs à portée
                collecte_faite = False
                for oeuf in oeufs:
                    if not oeuf["collecte"]:
                        if math.hypot(robot.x - oeuf["x"], robot.y - oeuf["y"]) < CELL / 2:
                            oeuf["collecte"] = True
                            collecte_faite   = True
                            print(f"🥚 Œuf collecté en ({oeuf['x']:.1f}, {oeuf['y']:.1f})")

                if collecte_faite:
                    env.collectables = [o for o in oeufs if not o["collecte"]]

                # 5. Victoire ?
                if all(o["collecte"] for o in oeufs):
                    victoire = True
                    print("🏆 Tous les œufs ont été récupérés !")

            # ── Rendu ────────────────────────────────────────────────────────
            vue.screen.fill(vue.COLOR_BG)
            vue.dessiner_grille()
            vue.dessiner_murs(env)              # murs réels (toujours là physiquement)
            vue.dessiner_cartographie(carto)    # voile noir sur non découvert
            vue.dessiner_lidar(robot, lidar)    # rayons LIDAR
            vue.dessiner_oeufs_detectes(carto)  # œufs visibles seulement si détectés
            vue.dessiner_robot(robot)
            vue.dessiner_compteur_oeufs(oeufs, NB_OEUFS)
            vue.dessiner_timer(timer)

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