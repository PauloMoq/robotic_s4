import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="pygame.pkgdata")

from robot.model.RobotMobile import RobotMobile
from robot.model.Moteur import MoteurOmnidirectionnel
from robot.controler.ControleurPygame import ControleurPygame
from robot.vue.VuePygame import VuePygame
from robot.model.Environnement import Environnement
import pygame
import sys
import random
from robot.model.Lidar import Lidar
from robot.controler.ControleurAutonome import ControleurAutonome

# ---------------------------------------------------------------------------
# Labyrinthe procédural — Recursive Backtracker (DFS)
#
# La grille est découpée en cellules. Chaque mur entre deux cellules
# adjacentes est soit présent soit abattu par l'algorithme DFS.
# Les murs sont ensuite convertis en segments pour l'Environnement.
#
# Coordonnées monde : origine (0,0) au centre, x ∈ [-demi_x, demi_x]
# ---------------------------------------------------------------------------

WIN_W    = 1200
WIN_H    = 700
MARGE_PX = 30
SCALE    = 50
CELL     = 2.0    # taille d'une cellule en unités monde
E        = 0.06   # épaisseur des murs

demi_x = (WIN_W / 2 - MARGE_PX) / SCALE   # 11.4
demi_y = (WIN_H / 2 - MARGE_PX) / SCALE   # 6.4

COLS = int(2 * demi_x / CELL)   # 11
ROWS = int(2 * demi_y / CELL)   # 6

# Coin bas-gauche du labyrinthe en unités monde
X0 = -COLS * CELL / 2
Y0 = -ROWS * CELL / 2


# ── Algorithme DFS ──────────────────────────────────────────────────────────

def generer_labyrinthe(cols, rows, seed=None):
    """
    Recursive Backtracker (DFS iteratif).
    Retourne un set de murs abattus : {((r1,c1),(r2,c2)), ...}
    où (r1,c1) et (r2,c2) sont deux cellules voisines sans mur entre elles.
    """
    if seed is not None:
        random.seed(seed)

    visited = [[False]*cols for _ in range(rows)]
    passages = set()  # paires de cellules connectées (mur abattu)

    def voisins(r, c):
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                yield (nr, nc)

    # Départ aléatoire
    r0, c0 = random.randint(0, rows-1), random.randint(0, cols-1)
    stack = [(r0, c0)]
    visited[r0][c0] = True

    while stack:
        r, c = stack[-1]
        vois = list(voisins(r, c))
        if vois:
            nr, nc = random.choice(vois)
            visited[nr][nc] = True
            # Enregistrer le passage (clé canonique)
            passages.add((min((r,c),(nr,nc)), max((r,c),(nr,nc))))
            stack.append((nr, nc))
        else:
            stack.pop()

    return passages


def cellule_vers_monde(r, c):
    """Retourne le coin bas-gauche de la cellule (r, c) en coordonnées monde."""
    x = X0 + c * CELL
    y = Y0 + (ROWS - 1 - r) * CELL   # r=0 est en haut → y grand
    return x, y


def construire_murs(passages):
    """
    Convertit la grille + passages en liste de murs {x, y, w, h}.
    On génère les murs de bordure + les murs intérieurs non abattus.
    """
    murs = []

    def mh(x, y, w):
        if w > E: murs.append({"x": x, "y": y, "w": w, "h": E})

    def mv(x, y, h):
        if h > E: murs.append({"x": x, "y": y, "w": E, "h": h})

    # ── Bordures extérieures ────────────────────────────────────────────────
    larg = COLS * CELL
    haut = ROWS * CELL
    mh(X0, Y0,        larg)          # bas
    mh(X0, Y0 + haut, larg)          # haut
    mv(X0,        Y0, haut)          # gauche
    mv(X0 + larg, Y0, haut)          # droite

    # ── Murs intérieurs ─────────────────────────────────────────────────────
    for r in range(ROWS):
        for c in range(COLS):
            cx, cy = cellule_vers_monde(r, c)

            # Mur du BAS de la cellule (entre (r,c) et (r+1,c))
            # r+1 est la ligne en dessous → en monde c'est cy - CELL
            if r + 1 < ROWS:
                cle = (min((r,c),(r+1,c)), max((r,c),(r+1,c)))
                if cle not in passages:
                    mh(cx, cy, CELL)

            # Mur de DROITE de la cellule (entre (r,c) et (r,c+1))
            if c + 1 < COLS:
                cle = (min((r,c),(r,c+1)), max((r,c),(r,c+1)))
                if cle not in passages:
                    mv(cx + CELL, cy, CELL)

    return murs


def cellule_aleatoire_unique(exclure=None):
    while True:
        r = random.randint(0, ROWS-1)
        c = random.randint(0, COLS-1)
        if exclure is None or (r, c) != exclure:
            return r, c


def centre_cellule_monde(r, c):
    x, y = cellule_vers_monde(r, c)
    return x + CELL/2, y + CELL/2


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    passages = generer_labyrinthe(COLS, ROWS)
    murs     = construire_murs(passages)

    # Départ et arrivée aléatoires (cellules différentes)
    r_dep, c_dep = cellule_aleatoire_unique()
    r_arr, c_arr = cellule_aleatoire_unique(exclure=(r_dep, c_dep))
    x_dep, y_dep = centre_cellule_monde(r_dep, c_dep)
    x_arr, y_arr = centre_cellule_monde(r_arr, c_arr)

    env = Environnement(
        largeur=WIN_W / SCALE,
        hauteur=WIN_H / SCALE,
        murs=murs,
        arrivee={"x": x_arr, "y": y_arr, "rayon": 0.4}
    )

    robot      = RobotMobile(x=x_dep, y=y_dep, moteur=MoteurOmnidirectionnel())
    vue        = VuePygame(largeur=WIN_W, hauteur=WIN_H, scale=SCALE)
    lidar      = Lidar(nb_rayons=8, portee=8.0, pas=0.02)
    controleur_manuel   = ControleurPygame(robot)
    controleur_auto     = ControleurAutonome(
                              robot, lidar, passages,
                              cell_size=CELL, x0=X0, y0=Y0,
                              cols=COLS, rows=ROWS,
                              rc_depart=(r_dep, c_dep),
                              rc_arrivee=(r_arr, c_arr),
                              v_max=2.0, omega_max=3.0)
    mode_auto  = True    # Tab pour basculer en manuel
    clock      = pygame.time.Clock()
    victoire   = False
    running    = True

    print("=== Labyrinthe Procédural (DFS) — Robot Mobile ===")
    print(f"Grille {COLS}×{ROWS}, cellules {CELL} unités")
    print(f"Départ  : cellule ({r_dep},{c_dep})")
    print(f"Arrivée : cellule ({r_arr},{c_arr})")
    print("Flèches : déplacement  |  Q/D : rotation  |  TAB : auto/manuel  |  R : regénérer  |  ECHAP : quitter")

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
                    # R : régénérer le labyrinthe
                    if event.key == pygame.K_r:
                        main()
                        return

            dt = clock.tick(60) / 1000.0

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