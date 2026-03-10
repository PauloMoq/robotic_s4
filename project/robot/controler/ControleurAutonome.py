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

        # Plus de waypoints → arrivée atteinte, s'arrêter
        if self.idx_waypoint >= len(self.chemin_monde):
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0}

        wx, wy = self.chemin_monde[self.idx_waypoint]
        dx = wx - self.robot.x
        dy = wy - self.robot.y
        dist_wp = math.sqrt(dx*dx + dy*dy)

        # Waypoint atteint → passer au suivant
        if dist_wp < self.cell * 0.25:
            self.idx_waypoint += 1
            if self.idx_waypoint >= len(self.chemin_monde):
                return {"vx": 0.0, "vy": 0.0, "omega": 0.0}
            wx, wy = self.chemin_monde[self.idx_waypoint]
            dx = wx - self.robot.x
            dy = wy - self.robot.y
            dist_wp = math.sqrt(dx*dx + dy*dy)

        # Angle vers le waypoint
        angle_cible = math.atan2(dy, dx)
        delta       = self._norm_angle(angle_cible - self.robot.orientation)

        # Rotation proportionnelle
        omega = max(-self.omega_max, min(self.omega_max, 3.0 * delta))

        # Vitesse : pleine si bien aligné, réduite sinon
        alignement = max(0.0, 1.0 - abs(delta) / (math.pi / 4))
        vx = self.v_max * alignement

        # Sécurité lidar : freinage d'urgence si obstacle très proche
        if self.lidar.mesures:
            dist_avant = self.lidar.mesures[0][1]
            if dist_avant < self.seuil_urgence:
                vx = 0.0

        return {"vx": vx, "vy": 0.0, "omega": omega}

    @property
    def chemin_restant(self):
        """Retourne les waypoints monde non encore atteints (pour affichage)."""
        return self.chemin_monde[self.idx_waypoint:]