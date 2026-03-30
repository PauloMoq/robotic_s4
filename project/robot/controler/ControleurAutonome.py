import math
from collections import deque

class ControleurAutonome:
    def __init__(self, robot, lidar, passages,
                 cell_size, x0, y0, cols, rows,
                 rc_depart, rc_arrivee,
                 v_max=2.0, omega_max=3.0,
                 seuil_urgence=0.25):
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

        self.chemin_cellules = self._bfs(rc_depart, rc_arrivee)
        self.chemin_monde    = [self._centre(r, c) for (r, c) in self.chemin_cellules]
        self.idx_waypoint    = 0

    def _bfs(self, depart, arrivee):
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
                cle = (min(rc, voisin), max(rc, voisin))
                if cle in self.passages:
                    pred[voisin] = rc
                    queue.append(voisin)

        chemin = []
        cur = arrivee
        while cur is not None:
            chemin.append(cur)
            cur = pred.get(cur)
        chemin.reverse()
        return chemin

    def _centre(self, r, c):
        x = self.x0 + c * self.cell + self.cell / 2
        y = self.y0 + (self.rows - 1 - r) * self.cell + self.cell / 2
        return (x, y)

    @staticmethod
    def _norm_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def calculer_commande(self):
        if not self.chemin_monde:
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0}

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

        if dist_wp < 0.05:
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0}

        nx = dx / dist_wp
        ny = dy / dist_wp

        theta = self.robot.orientation
        vx_robot =  nx * math.cos(theta) + ny * math.sin(theta)
        vy_robot = -nx * math.sin(theta) + ny * math.cos(theta)

        vx = self.v_max * vx_robot
        vy = self.v_max * vy_robot

        angle_cible = math.atan2(dy, dx)
        delta = self._norm_angle(angle_cible - theta)
        omega = max(-self.omega_max, min(self.omega_max, 2.0 * delta))

        return {"vx": vx, "vy": vy, "omega": omega}

    @property
    def chemin_restant(self):
        return self.chemin_monde[self.idx_waypoint:]

    def _recalculer_chemin(self):
        c_cur = round((self.robot.x - self.x0 - self.cell / 2) / self.cell)
        r_cur = self.rows - 1 - round((self.robot.y - self.y0 - self.cell / 2) / self.cell)
        c_cur = max(0, min(self.cols - 1, c_cur))
        r_cur = max(0, min(self.rows - 1, r_cur))
        self.chemin_cellules = self._bfs((r_cur, c_cur), self.rc_arrivee)
        self.chemin_monde    = [self._centre(r, c) for (r, c) in self.chemin_cellules]
        self.idx_waypoint    = 0

    def distance_bfs(self, rc_depart, rc_arrivee) -> int:
        queue = deque([rc_depart])
        pred  = {rc_depart: None}
        while queue:
            rc = queue.popleft()
            if rc == rc_arrivee:
                break
            r, c = rc
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                voisin = (r + dr, c + dc)
                if voisin in pred:
                    continue
                if not (0 <= voisin[0] < self.rows and 0 <= voisin[1] < self.cols):
                    continue
                if (min(rc, voisin), max(rc, voisin)) in self.passages:
                    pred[voisin] = rc
                    queue.append(voisin)
        cur, n = rc_arrivee, 0
        while pred.get(cur) is not None:
            cur = pred[cur]
            n += 1
        return n

    def oeuf_le_plus_proche(self, rc_robot, oeufs, laby) -> dict:
        non_collectes = [o for o in oeufs if not o["collecte"]]
        return min(
            non_collectes,
            key=lambda o: self.distance_bfs(rc_robot, laby.monde_vers_cellule(o["x"], o["y"]))
        )


def trouver_oeuf_cible(robot, lidar, oeufs, env):
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
    for oeuf in oeufs:
        if not oeuf["collecte"]:
            d = math.hypot(robot.x - oeuf["x"], robot.y - oeuf["y"])
            if d < rayon_collecte:
                oeuf["collecte"] = True
                print(f"🥚 Œuf collecté en ({oeuf['x']:.1f}, {oeuf['y']:.1f}) !")