import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="pygame.pkgdata")

from robot.model.RobotMobile import RobotMobile
from robot.model.Moteur import MoteurOmnidirectionnel
from robot.controler.ControleurPygame import ControleurPygame
from robot.vue.VuePygame import VuePygame
from robot.model.Environnement import Environnement
import pygame
import sys
import math
import random
from robot.model.Lidar import Lidar
from robot.controler.ControleurAutonome import ControleurAutonome, trouver_oeuf_cible, verifier_collecte

# ---------------------------------------------------------------------------
# Chasse aux œufs — Labyrinthe procédural (DFS)
#
# 3 œufs sont placés aléatoirement dans des cellules du labyrinthe.
# Le LIDAR détecte les œufs (ils bloquent les rayons comme des murs).
# Le robot autonome cible l'œuf le plus proche détecté et va le collecter.
# Victoire quand les 3 œufs sont ramassés.
# ---------------------------------------------------------------------------

WIN_W    = 1200
WIN_H    = 700
MARGE_PX = 30
SCALE    = 50
CELL     = 2.0    # taille d'une cellule en unités monde
E        = 0.06   # épaisseur des murs

NB_OEUFS         = 3
RAYON_OEUF       = 0.18   # rayon physique de l'œuf (collision/collecte)
RAYON_COLLECTE   = 0.45   # distance de ramassage

demi_x = (WIN_W / 2 - MARGE_PX) / SCALE
demi_y = (WIN_H / 2 - MARGE_PX) / SCALE

COLS = int(2 * demi_x / CELL)
ROWS = int(2 * demi_y / CELL)

X0 = -COLS * CELL / 2
Y0 = -ROWS * CELL / 2


# ── Algorithme DFS ──────────────────────────────────────────────────────────

def generer_labyrinthe(cols, rows, seed=None):
    if seed is not None:
        random.seed(seed)

    visited  = [[False]*cols for _ in range(rows)]
    passages = set()

    def voisins(r, c):
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                yield (nr, nc)

    r0, c0 = random.randint(0, rows-1), random.randint(0, cols-1)
    stack = [(r0, c0)]
    visited[r0][c0] = True

    while stack:
        r, c = stack[-1]
        vois = list(voisins(r, c))
        if vois:
            nr, nc = random.choice(vois)
            visited[nr][nc] = True
            passages.add((min((r,c),(nr,nc)), max((r,c),(nr,nc))))
            stack.append((nr, nc))
        else:
            stack.pop()

    return passages


def cellule_vers_monde(r, c):
    x = X0 + c * CELL
    y = Y0 + (ROWS - 1 - r) * CELL
    return x, y


def centre_cellule_monde(r, c):
    x, y = cellule_vers_monde(r, c)
    return x + CELL/2, y + CELL/2


def construire_murs(passages):
    murs = []

    def mh(x, y, w):
        if w > E: murs.append({"x": x, "y": y, "w": w, "h": E})

    def mv(x, y, h):
        if h > E: murs.append({"x": x, "y": y, "w": E, "h": h})

    larg = COLS * CELL
    haut = ROWS * CELL
    mh(X0, Y0,        larg)
    mh(X0, Y0 + haut, larg)
    mv(X0,        Y0, haut)
    mv(X0 + larg, Y0, haut)

    for r in range(ROWS):
        for c in range(COLS):
            cx, cy = cellule_vers_monde(r, c)

            if r + 1 < ROWS:
                cle = (min((r,c),(r+1,c)), max((r,c),(r+1,c)))
                if cle not in passages:
                    mh(cx, cy, CELL)

            if c + 1 < COLS:
                cle = (min((r,c),(r,c+1)), max((r,c),(r,c+1)))
                if cle not in passages:
                    mv(cx + CELL, cy, CELL)

    return murs


def placer_oeufs(nb, cellule_exclue):
    """Place nb œufs dans des cellules aléatoires différentes (hors cellule robot)."""
    cellules = set()
    while len(cellules) < nb:
        r = random.randint(0, ROWS-1)
        c = random.randint(0, COLS-1)
        if (r, c) != cellule_exclue:
            cellules.add((r, c))
    oeufs = []
    for r, c in cellules:
        x, y = centre_cellule_monde(r, c)
        oeufs.append({"x": x, "y": y, "rayon": RAYON_OEUF, "collecte": False})
    return oeufs

# ── Main ────────────────────────────────────────────────────────────────────

def main():
    passages = generer_labyrinthe(COLS, ROWS)
    murs     = construire_murs(passages)

    # Départ aléatoire
    r_dep = random.randint(0, ROWS-1)
    c_dep = random.randint(0, COLS-1)
    x_dep, y_dep = centre_cellule_monde(r_dep, c_dep)

    # Œufs dans des cellules différentes du départ
    oeufs = placer_oeufs(NB_OEUFS, cellule_exclue=(r_dep, c_dep))

    # L'environnement connaît les œufs comme obstacles LIDAR
    # (ils bloquent les rayons via la liste obstacles)
    env = Environnement(
        largeur=WIN_W / SCALE,
        hauteur=WIN_H / SCALE,
        murs=murs,
        collectables=oeufs   # œufs : visibles au LIDAR, sans collision physique
    )

    robot      = RobotMobile(x=x_dep, y=y_dep, moteur=MoteurOmnidirectionnel())
    vue        = VuePygame(largeur=WIN_W, hauteur=WIN_H, scale=SCALE)
    lidar      = Lidar(nb_rayons=16, portee=6.0, pas=0.02)
    controleur_manuel = ControleurPygame(robot)

    # Cible initiale = œuf le plus proche du robot (distance BFS, pas euclidienne)
    def monde_vers_cellule(x, y):
        c = round((x - X0 - CELL / 2) / CELL)
        r = ROWS - 1 - round((y - Y0 - CELL / 2) / CELL)
        return max(0, min(ROWS-1, r)), max(0, min(COLS-1, c))

    def distance_bfs(rc_depart, rc_arrivee):
        """Retourne le nombre de cellules du chemin BFS entre deux cellules."""
        from collections import deque
        queue = deque([rc_depart])
        pred  = {rc_depart: None}
        while queue:
            rc = queue.popleft()
            if rc == rc_arrivee:
                break
            r, c = rc
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                voisin = (r+dr, c+dc)
                if voisin in pred: continue
                if not (0 <= voisin[0] < ROWS and 0 <= voisin[1] < COLS): continue
                cle = (min(rc, voisin), max(rc, voisin))
                if cle in passages:
                    pred[voisin] = rc
                    queue.append(voisin)
        # Reconstituer longueur
        cur, n = rc_arrivee, 0
        while pred.get(cur) is not None:
            cur = pred[cur]
            n += 1
        return n

    def oeuf_le_plus_proche(rc_robot):
        non_collectes = [o for o in oeufs if not o["collecte"]]
        return min(non_collectes,
                   key=lambda o: distance_bfs(rc_robot, monde_vers_cellule(o["x"], o["y"])))

    premier_oeuf = oeuf_le_plus_proche((r_dep, c_dep))
    r_cible_init, c_cible_init = monde_vers_cellule(premier_oeuf["x"], premier_oeuf["y"])

    controleur_auto = ControleurAutonome(
        robot, lidar, passages,
        cell_size=CELL, x0=X0, y0=Y0,
        cols=COLS, rows=ROWS,
        rc_depart=(r_dep, c_dep),
        rc_arrivee=(r_cible_init, c_cible_init),
        v_max=2.0, omega_max=3.0
    )

    oeuf_courant_idx = 0
    mode_auto = True
    clock     = pygame.time.Clock()
    victoire  = False
    running   = True
    relancer  = False
    timer     = 0.0   # secondes écoulées

    pygame.display.set_caption("Chasse aux oeufs — Robot LIDAR")
    print("=== Chasse aux Oeufs — Robot LIDAR ===")
    print(f"Grille {COLS}x{ROWS}, {NB_OEUFS} oeufs caches")
    print(f"Depart robot : cellule ({r_dep},{c_dep})")
    print("TAB : auto/manuel  |  R : regenerer  |  ECHAP : quitter")

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
                # 1) Vérifier la collecte sur TOUS les œufs à portée
                collecte_faite = False
                for oeuf in oeufs:
                    if not oeuf["collecte"]:
                        d = math.hypot(robot.x - oeuf["x"], robot.y - oeuf["y"])
                        if d < CELL / 2:   # même cellule que l'œuf
                            oeuf["collecte"] = True
                            collecte_faite = True
                            print(f"Oeuf collecte en ({oeuf['x']:.1f}, {oeuf['y']:.1f})")

                if collecte_faite:
                    env.collectables = [o for o in oeufs if not o["collecte"]]
                    suivants = [o for o in oeufs if not o["collecte"]]
                    if suivants:
                        rc_robot = monde_vers_cellule(robot.x, robot.y)
                        prochain = oeuf_le_plus_proche(rc_robot)
                        nx, ny = prochain["x"], prochain["y"]
                        c_next = round((nx - X0 - CELL/2) / CELL)
                        r_next = ROWS - 1 - round((ny - Y0 - CELL/2) / CELL)
                        c_next = max(0, min(COLS-1, c_next))
                        r_next = max(0, min(ROWS-1, r_next))
                        controleur_auto.rc_arrivee = (r_next, c_next)
                        controleur_auto._recalculer_chemin()

                # 2) Scan LIDAR
                lidar.scanner(robot, env)

                # 3) Commande (le path ne change PAS sauf si œuf collecté ci-dessus)
                if mode_auto:
                    commande = controleur_auto.calculer_commande()
                else:
                    commande = controleur_manuel.lire_commande()

                robot.commander(**commande)

                # 4) Physique — pas de collectables dans est_en_collision, c'est propre
                robot.mettre_a_jour(dt, env)

                # 5) Victoire ?
                if all(o["collecte"] for o in oeufs):
                    victoire = True
                    print("🏆 Tous les œufs ont été récupérés !")

            # ── Rendu ──────────────────────────────────────────────────────
            vue.screen.fill(vue.COLOR_BG)
            vue.dessiner_grille()
            vue.dessiner_murs(env)
            vue.dessiner_chemin(controleur_auto)
            vue.dessiner_lidar(robot, lidar)
            vue.dessiner_oeufs(oeufs)
            vue.dessiner_robot(robot)
            vue.dessiner_compteur_oeufs(oeufs, NB_OEUFS)
            vue.dessiner_timer(timer)

            if victoire:
                vue.afficher_victoire_oeufs(timer)

            pygame.display.flip()

    except KeyboardInterrupt:
        print("\nArrêt via Terminal (Ctrl+C).")
    except Exception as e:
        import traceback
        print("\n--- ERREUR ---")
        traceback.print_exc()
    finally:
        if not relancer:
            pygame.quit()
            print("Fermeture propre effectuée.")
            sys.exit()

    # Relance propre sans récursion de finally
    pygame.quit()
    main()


if __name__ == "__main__":
    main()