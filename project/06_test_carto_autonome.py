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
from collections import deque
from robot.model.Lidar import Lidar

# ---------------------------------------------------------------------------
# 06 — Cartographie autonome avec collecte prioritaire
#
# Logique simple et stable :
#   - Le robot choisit une cible (cellule inconnue la plus proche OU œuf détecté)
#   - Il calcule un BFS vers cette cible UNE SEULE FOIS
#   - Il suit ce chemin sans le recalculer jusqu'à arriver à destination
#   - Seulement là, il choisit une nouvelle cible
#
# Exception : si un œuf est détecté pendant l'exploration, on interrompt
# immédiatement pour aller le chercher (priorité collecte).
#
# Machine à états :
#   EXPLORATION — va vers la cellule inconnue la plus proche
#   COLLECTE    — va vers un œuf détecté
# ---------------------------------------------------------------------------

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

X0 = -COLS * CELL / 2
Y0 = -ROWS * CELL / 2

ETAT_EXPLORATION = "exploration"
ETAT_COLLECTE    = "collecte"


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


# ── Utilitaires ──────────────────────────────────────────────────────────────

def monde_vers_cellule(x, y):
    c = round((x - X0 - CELL / 2) / CELL)
    r = ROWS - 1 - round((y - Y0 - CELL / 2) / CELL)
    return max(0, min(ROWS - 1, r)), max(0, min(COLS - 1, c))


def bfs_vers(depart, arrivee, passages_graph, rows, cols):
    """BFS simple départ→arrivee. Retourne liste de cellules ou []."""
    queue = deque([depart])
    pred  = {depart: None}
    while queue:
        rc = queue.popleft()
        if rc == arrivee:
            break
        r, c = rc
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            v = (r + dr, c + dc)
            if v in pred:
                continue
            if not (0 <= v[0] < rows and 0 <= v[1] < cols):
                continue
            if (min(rc, v), max(rc, v)) in passages_graph:
                pred[v] = rc
                queue.append(v)
    if arrivee not in pred:
        return []
    chemin, cur = [], arrivee
    while cur is not None:
        chemin.append(cur)
        cur = pred.get(cur)
    chemin.reverse()
    return chemin


def bfs_cellule_inconnue_proche(depart, carto, passages_graph, rows, cols):
    """
    BFS depuis depart sur passages_graph (passages_reels).
    Retourne la cellule ACCESSIBLE la plus proche qui n'a pas encore
    été découverte par la cartographie, ou None si tout est exploré.

    On navigue vers une cellule accessible (dans passages_graph) qui
    n'est pas encore dans carto.decouverte.
    On exclut depart lui-même pour éviter cible == robot.
    """
    queue   = deque([depart])
    visited = {depart}

    while queue:
        rc = queue.popleft()
        r, c = rc
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            v = (r + dr, c + dc)
            if v in visited:
                continue
            if not (0 <= v[0] < rows and 0 <= v[1] < cols):
                continue
            if (min(rc, v), max(rc, v)) not in passages_graph:
                continue
            visited.add(v)
            # Voisin accessible non encore découvert → c'est notre cible
            if not carto.est_decouverte(v[0], v[1]):
                return v
            queue.append(v)

    return None  # tout exploré


# ── Classe ControleurExploration ────────────────────────────────────────────

class ControleurExploration:
    """
    Contrôleur autonome stable.

    Règle unique : on fixe une cible, on calcule le BFS UNE fois,
    on suit le chemin jusqu'au bout. On ne recalcule que quand
    la destination est atteinte — ou si un œuf est détecté (interruption).
    """

    def __init__(self, robot, lidar, carto, passages_reels,
                 cell_size, x0, y0, cols, rows,
                 rc_depart, v_max=2.0, omega_max=3.0):
        self.robot          = robot
        self.lidar          = lidar
        self.carto          = carto
        self.passages_reels = passages_reels
        self.cell           = cell_size
        self.x0             = x0
        self.y0             = y0
        self.cols           = cols
        self.rows           = rows
        self.v_max          = v_max
        self.omega_max      = omega_max

        self._etat        = ETAT_EXPLORATION
        self.chemin_monde = []
        self.idx_waypoint = 0
        self.rc_cible     = None
        self.cible_oeuf   = None

        self._fixer_prochaine_cible(rc_depart)

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _centre(self, r, c):
        x = self.x0 + c * self.cell + self.cell / 2
        y = self.y0 + (self.rows - 1 - r) * self.cell + self.cell / 2
        return (x, y)

    @staticmethod
    def _norm_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── Fixation de la cible et calcul BFS (une seule fois) ─────────────────

    def _fixer_prochaine_cible(self, rc_robot=None):
        """
        Choisit la prochaine cible et calcule le chemin BFS.
        N'est appelé QUE quand la destination précédente est atteinte
        (ou lors d'une interruption pour collecte d'œuf).
        """
        if rc_robot is None:
            rc_robot = monde_vers_cellule(self.robot.x, self.robot.y)

        if self._etat == ETAT_COLLECTE and self.cible_oeuf is not None:
            # Cible = cellule de l'œuf
            rc_cible = monde_vers_cellule(self.cible_oeuf["x"], self.cible_oeuf["y"])
        else:
            # Cible = cellule connue la plus proche ayant un voisin inconnu.
            # On utilise TOUJOURS passages_reels pour la recherche de frontière :
            # le robot sait où il peut physiquement aller, pas ce qu'il y a.
            rc_cible = bfs_cellule_inconnue_proche(
                rc_robot, self.carto, self.passages_reels, self.rows, self.cols
            )

        if rc_cible is None:
            self.rc_cible     = None
            self.chemin_monde = []
            self.idx_waypoint = 0
            return

        self.rc_cible = rc_cible

        # BFS sur passages connus, fallback passages réels
        chemin = bfs_vers(rc_robot, rc_cible, self.carto.passages_connus,
                          self.rows, self.cols)
        if not chemin:
            chemin = bfs_vers(rc_robot, rc_cible, self.passages_reels,
                              self.rows, self.cols)

        self.chemin_monde = [self._centre(r, c) for r, c in chemin]
        self.idx_waypoint = 0

    # ── Mise à jour de l'état ────────────────────────────────────────────────

    def mettre_a_jour_etat(self, oeufs):
        """
        Appelé chaque frame. Gère uniquement deux cas :
          1. Un œuf vient d'être détecté → interruption et switch COLLECTE
          2. L'œuf cible vient d'être collecté → switch EXPLORATION
        Tout le reste (suivi de chemin) est géré par calculer_commande.
        """
        # ── Œuf collecté → retour exploration ───────────────────────────────
        if self._etat == ETAT_COLLECTE:
            if self.cible_oeuf is None or self.cible_oeuf["collecte"]:
                print("[État] → EXPLORATION")
                self._etat      = ETAT_EXPLORATION
                self.cible_oeuf = None
                rc_robot = monde_vers_cellule(self.robot.x, self.robot.y)
                self._fixer_prochaine_cible(rc_robot)

        # ── Œuf détecté → interruption et collecte ──────────────────────────
        elif self._etat == ETAT_EXPLORATION:
            detectes = [o for o in self.carto.oeufs_detectes if not o["collecte"]]
            if detectes:
                nouveau = min(detectes,
                              key=lambda o: math.hypot(self.robot.x - o["x"],
                                                       self.robot.y - o["y"]))
                # Interruption seulement si c'est un nouvel œuf
                if nouveau is not self.cible_oeuf:
                    print(f"[État] → COLLECTE ({nouveau['x']:.1f}, {nouveau['y']:.1f})")
                    self._etat      = ETAT_COLLECTE
                    self.cible_oeuf = nouveau
                    rc_robot = monde_vers_cellule(self.robot.x, self.robot.y)
                    self._fixer_prochaine_cible(rc_robot)

    # ── Commande principale ──────────────────────────────────────────────────

    def calculer_commande(self):
        """Retourne {"vx", "vy", "omega"}. Appelle _fixer_prochaine_cible
        uniquement quand le dernier waypoint est atteint."""

        if not self.chemin_monde:
            # Chemin vide : essayer d'en trouver un nouveau
            self._fixer_prochaine_cible()
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0}

        # Avancer dans les waypoints intermédiaires
        while self.idx_waypoint < len(self.chemin_monde) - 1:
            wx, wy = self.chemin_monde[self.idx_waypoint]
            if math.hypot(self.robot.x - wx, self.robot.y - wy) < self.cell * 0.3:
                self.idx_waypoint += 1
            else:
                break

        wx, wy  = self.chemin_monde[self.idx_waypoint]
        dx      = wx - self.robot.x
        dy      = wy - self.robot.y
        dist_wp = math.hypot(dx, dy)

        # Destination finale atteinte → choisir la prochaine cible
        if self.idx_waypoint == len(self.chemin_monde) - 1 and dist_wp < self.cell * 0.3:
            self._fixer_prochaine_cible()
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0}

        if dist_wp < 1e-6:
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0}

        nx = dx / dist_wp
        ny = dy / dist_wp

        theta    = self.robot.orientation
        vx_robot =  nx * math.cos(theta) + ny * math.sin(theta)
        vy_robot = -nx * math.sin(theta) + ny * math.cos(theta)

        angle_cible = math.atan2(dy, dx)
        delta       = self._norm_angle(angle_cible - theta)
        omega       = max(-self.omega_max, min(self.omega_max, 2.0 * delta))

        return {
            "vx":    self.v_max * vx_robot,
            "vy":    self.v_max * vy_robot,
            "omega": omega,
        }

    @property
    def chemin_restant(self):
        return self.chemin_monde[self.idx_waypoint:]

    @property
    def etat(self):
        return self._etat


# ── Main ─────────────────────────────────────────────────────────────────────

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

    robot             = RobotMobile(x=x_dep, y=y_dep, moteur=MoteurOmnidirectionnel())
    vue               = VuePygame(largeur=WIN_W, hauteur=WIN_H, scale=SCALE)
    lidar             = Lidar(nb_rayons=16, portee=3.5, pas=0.02)
    carto             = Cartographie(ROWS, COLS, CELL, X0, Y0, passages=passages)
    controleur_manuel = ControleurPygame(robot)

    # Scan initial pour amorcer la cartographie
    lidar.scanner(robot, env)
    carto.mettre_a_jour(robot, lidar, oeufs)

    controleur_auto = ControleurExploration(
        robot, lidar, carto, passages,
        cell_size=CELL, x0=X0, y0=Y0,
        cols=COLS, rows=ROWS,
        rc_depart=(r_dep, c_dep),
        v_max=2.0, omega_max=3.0
    )

    mode_auto = True
    clock     = pygame.time.Clock()
    victoire  = False
    running   = True
    relancer  = False
    timer     = 0.0

    pygame.display.set_caption("Cartographie Autonome — Robot LIDAR")
    print("=== Cartographie Autonome — Robot LIDAR ===")
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
                        if mode_auto:
                            # Recalcul propre depuis la position actuelle
                            controleur_auto._fixer_prochaine_cible()
                    if event.key == pygame.K_r:
                        running = False
                        relancer = True

            dt = clock.tick(60) / 1000.0

            if not victoire:
                timer += dt

                # 1. Commande
                if mode_auto:
                    controleur_auto.mettre_a_jour_etat(oeufs)
                    commande = controleur_auto.calculer_commande()
                else:
                    commande = controleur_manuel.lire_commande()

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
            vue.dessiner_cartographie(carto)
            vue.dessiner_chemin(controleur_auto)
            vue.dessiner_lidar(robot, lidar)
            vue.dessiner_oeufs_detectes(carto)
            vue.dessiner_robot(robot)
            vue.dessiner_compteur_oeufs(oeufs, NB_OEUFS)
            vue.dessiner_timer(timer)
            vue.dessiner_etat_exploration(controleur_auto)

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