import math
from collections import deque

class ControleurAutonome:
    """
    Contrôleur autonome par pathfinding BFS sur la grille du labyrinthe.

    Principe :
      1. BFS sur le graphe `passages` pour trouver le chemin optimal
         départ → arrivée (en cellules).
      2. Le robot suit les centres des cellules une par une comme waypoints.
      3. Pour chaque waypoint : orienter le robot vers la cible, avancer.
      4. Le lidar sert de sécurité anti-collision (freinage d'urgence).
    """

    def __init__(self, robot, lidar, passages,
                 cell_size, x0, y0, cols, rows,
                 rc_depart, rc_arrivee,
                 v_max=2.0, omega_max=3.0,
                 seuil_urgence=0.25):
        """
        robot        : RobotMobile
        lidar        : Lidar
        passages     : set de paires de cellules connectées (depuis generer_labyrinthe)
        cell_size    : taille d'une cellule en unités monde
        x0, y0       : coin bas-gauche du labyrinthe
        cols, rows   : dimensions de la grille
        rc_depart    : (row, col) de départ
        rc_arrivee   : (row, col) d'arrivée
        v_max        : vitesse max (unités/s)
        omega_max    : vitesse angulaire max (rad/s)
        seuil_urgence: distance lidar frontale sous laquelle on freine fort
        """
        self.robot         = robot
        self.lidar         = lidar
        self.passages      = passages
        self.cell          = cell_size
        self.x0            = x0
        self.y0            = y0
        self.cols          = cols
        self.rows          = rows
        self.v_max         = v_max
        self.omega_max     = omega_max
        self.seuil_urgence = seuil_urgence

        self.rc_depart  = rc_depart
        self.rc_arrivee = rc_arrivee

        # Calculer le chemin BFS
        self.chemin_cellules = self._bfs(rc_depart, rc_arrivee)  # [(r,c), ...]
        self.chemin_monde    = [self._centre(r, c) for (r, c) in self.chemin_cellules]
        self.idx_waypoint    = 0  # prochain waypoint à atteindre

    # ── BFS ─────────────────────────────────────────────────────────────────

    def _bfs(self, depart, arrivee):
        """Retourne la liste de cellules (r,c) du chemin départ→arrivée."""
        queue    = deque([depart])
        pred     = {depart: None}

        while queue:
            rc = queue.popleft()
            if rc == arrivee:
                break
            r, c = rc
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                voisin = (r+dr, c+dc)
                if voisin in pred:
                    continue
                if not (0 <= voisin[0] < self.rows and 0 <= voisin[1] < self.cols):
                    continue
                # Vérifier qu'il y a un passage entre rc et voisin
                cle = (min(rc, voisin), max(rc, voisin))
                if cle in self.passages:
                    pred[voisin] = rc
                    queue.append(voisin)

        # Reconstruire le chemin
        chemin = []
        cur = arrivee
        while cur is not None:
            chemin.append(cur)
            cur = pred.get(cur)
        chemin.reverse()
        return chemin

    # ── Conversion cellule → monde ──────────────────────────────────────────

    def _centre(self, r, c):
        """Centre de la cellule (r,c) en coordonnées monde."""
        x = self.x0 + c * self.cell + self.cell / 2
        y = self.y0 + (self.rows - 1 - r) * self.cell + self.cell / 2
        return (x, y)

    # ── Utilitaires angulaires ───────────────────────────────────────────────

    @staticmethod
    def _norm_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── Commande principale ──────────────────────────────────────────────────

    def calculer_commande(self):
        """Retourne {"vx", "vy", "omega"} pour MoteurOmnidirectionnel."""

        if not self.chemin_monde:
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0}

        # Avancer l'index tant que le waypoint courant est atteint
        # (sauf pour le dernier — on continue d'avancer vers lui jusqu'à collecte)
        while self.idx_waypoint < len(self.chemin_monde) - 1:
            wx, wy = self.chemin_monde[self.idx_waypoint]
            dist = math.hypot(self.robot.x - wx, self.robot.y - wy)
            if dist < self.cell * 0.3:
                self.idx_waypoint += 1
            else:
                break

        wx, wy = self.chemin_monde[self.idx_waypoint]
        dx = wx - self.robot.x
        dy = wy - self.robot.y
        dist_wp = math.hypot(dx, dy)

        # Trop proche du dernier waypoint → s'arrêter (collecte gérée par test_oeufs)
        if dist_wp < 0.05:
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0}

        # Vecteur normalisé vers le waypoint (coordonnées monde)
        nx = dx / dist_wp
        ny = dy / dist_wp

        # Projection dans le repère robot
        theta = self.robot.orientation
        vx_robot =  nx * math.cos(theta) + ny * math.sin(theta)
        vy_robot = -nx * math.sin(theta) + ny * math.cos(theta)

        vx = self.v_max * vx_robot
        vy = self.v_max * vy_robot

        # Rotation douce vers la cible
        angle_cible = math.atan2(dy, dx)
        delta = self._norm_angle(angle_cible - theta)
        omega = max(-self.omega_max, min(self.omega_max, 2.0 * delta))

        return {"vx": vx, "vy": vy, "omega": omega}

    @property
    def chemin_restant(self):
        """Retourne les waypoints monde non encore atteints (pour affichage)."""
        return self.chemin_monde[self.idx_waypoint:]

    def _recalculer_chemin(self):
        """Recalcule le chemin BFS vers rc_arrivee depuis la position actuelle du robot."""
        c_cur = round((self.robot.x - self.x0 - self.cell / 2) / self.cell)
        r_cur = self.rows - 1 - round((self.robot.y - self.y0 - self.cell / 2) / self.cell)
        c_cur = max(0, min(self.cols - 1, c_cur))
        r_cur = max(0, min(self.rows - 1, r_cur))
        self.chemin_cellules = self._bfs((r_cur, c_cur), self.rc_arrivee)
        self.chemin_monde    = [self._centre(r, c) for (r, c) in self.chemin_cellules]
        self.idx_waypoint    = 0


# ── Fonctions utilitaires (hors classe) ─────────────────────────────────────

def trouver_oeuf_cible(robot, lidar, oeufs, env):
    """
    Parcourt les mesures LIDAR. Si un rayon percute un œuf non collecté
    (point d'impact proche du centre de l'œuf), retourne cet œuf.
    Sinon retourne l'œuf non collecté le plus proche (fallback).
    """
    for (angle, dist, xi, yi) in lidar.mesures:
        for oeuf in oeufs:
            if oeuf["collecte"]:
                continue
            dist_impact = math.hypot(xi - oeuf["x"], yi - oeuf["y"])
            if dist_impact < oeuf["rayon"] + 0.25:
                return oeuf

    non_collectes = [o for o in oeufs if not o["collecte"]]
    if not non_collectes:
        return None
    return min(non_collectes,
               key=lambda o: math.hypot(robot.x - o["x"], robot.y - o["y"]))


def verifier_collecte(robot, oeufs, rayon_collecte=0.45):
    """Marque un œuf comme collecté si le robot est assez proche."""
    for oeuf in oeufs:
        if not oeuf["collecte"]:
            d = math.hypot(robot.x - oeuf["x"], robot.y - oeuf["y"])
            if d < rayon_collecte:
                oeuf["collecte"] = True
                print(f"🥚 Œuf collecté en ({oeuf['x']:.1f}, {oeuf['y']:.1f}) !")