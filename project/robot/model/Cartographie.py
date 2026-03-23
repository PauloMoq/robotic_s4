import math


class Cartographie:
    """
    Carte progressive construite par le LIDAR.
    Utilise le graphe `passages` du labyrinthe pour bloquer
    strictement la vision au niveau des murs.
    """

    def __init__(self, rows, cols, cell_size, x0, y0, passages=None):
        self.rows     = rows
        self.cols     = cols
        self.cell     = cell_size
        self.x0       = x0
        self.y0       = y0
        self.passages = passages if passages is not None else set()

        self.decouverte     = [[False] * cols for _ in range(rows)]
        self.passages_connus = set()
        self.oeufs_detectes  = []

    # ── Conversion coordonnées ───────────────────────────────────────────────

    def monde_vers_cellule(self, x, y):
        c = int((x - self.x0) / self.cell)
        r = self.rows - 1 - int((y - self.y0) / self.cell)
        c = max(0, min(self.cols - 1, c))
        r = max(0, min(self.rows - 1, r))
        return r, c

    def centre_cellule(self, r, c):
        x = self.x0 + c * self.cell + self.cell / 2
        y = self.y0 + (self.rows - 1 - r) * self.cell + self.cell / 2
        return x, y

    # ── Chemin de cellules entre deux points (DDA) ───────────────────────────

    def _cellules_rayon(self, ox, oy, dx, dy, dist):
        """
        Retourne la liste ordonnée de cellules (r,c) traversées par le rayon,
        en avançant pas à pas avec un pas < demi-cellule pour ne jamais sauter
        une frontière. S'arrête dès qu'un mur est détecté.
        """
        pas = self.cell * 0.2          # pas petit : on ne saute jamais une arête
        n   = int(dist / pas) + 1
        rc_prev = self.monde_vers_cellule(ox, oy)
        cellules = [rc_prev]

        for k in range(1, n + 1):
            t  = min(k * pas, dist)
            px = ox + dx * t
            py = oy + dy * t
            rc_cur = self.monde_vers_cellule(px, py)

            if rc_cur == rc_prev:
                continue

            # Décomposer le saut en étapes unitaires (évite les diagonales)
            r0, c0 = rc_prev
            r1, c1 = rc_cur
            dr = 0 if r1 == r0 else (1 if r1 > r0 else -1)
            dc = 0 if c1 == c0 else (1 if c1 > c0 else -1)

            # Parcourir chaque étape unitaire entre rc_prev et rc_cur
            r_tmp, c_tmp = r0, c0
            bloque = False

            # Si saut diagonal (dr!=0 et dc!=0), on passe par les deux voisins cardinaux
            steps = []
            if dr != 0 and dc != 0:
                steps = [(r_tmp + dr, c_tmp), (r_tmp + dr, c_tmp + dc)]
            else:
                steps = [(r_tmp + dr, c_tmp + dc)]

            for (rn, cn) in steps:
                cle = (min((r_tmp, c_tmp), (rn, cn)), max((r_tmp, c_tmp), (rn, cn)))
                if cle not in self.passages:
                    bloque = True
                    break
                cellules.append((rn, cn))
                r_tmp, c_tmp = rn, cn

            if bloque:
                return cellules   # on s'arrête, cellule derrière le mur non incluse

            rc_prev = rc_cur

            if t >= dist:
                break

        return cellules

    # ── Mise à jour depuis le LIDAR ──────────────────────────────────────────

    def mettre_a_jour(self, robot, lidar, oeufs_env):
        rc_robot = self.monde_vers_cellule(robot.x, robot.y)
        self._marquer(rc_robot)

        for (angle, dist, xi, yi) in lidar.mesures:
            dx = math.cos(angle)
            dy = math.sin(angle)

            cellules = self._cellules_rayon(robot.x, robot.y, dx, dy, dist)

            rc_prev = rc_robot
            for rc in cellules:
                self._marquer(rc)
                cle = (min(rc_prev, rc), max(rc_prev, rc))
                if abs(rc[0]-rc_prev[0]) + abs(rc[1]-rc_prev[1]) == 1:
                    self.passages_connus.add(cle)
                rc_prev = rc

            # Détection d'œuf au point d'impact
            for oeuf in oeufs_env:
                if oeuf.get("collecte"):
                    continue
                if math.hypot(xi - oeuf["x"], yi - oeuf["y"]) < oeuf["rayon"] + 0.3:
                    if oeuf not in self.oeufs_detectes:
                        self.oeufs_detectes.append(oeuf)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _marquer(self, rc):
        r, c = rc
        if 0 <= r < self.rows and 0 <= c < self.cols:
            self.decouverte[r][c] = True

    def est_decouverte(self, r, c):
        return self.decouverte[r][c]

    def nb_cellules_decouvertes(self):
        return sum(self.decouverte[r][c]
                   for r in range(self.rows)
                   for c in range(self.cols))