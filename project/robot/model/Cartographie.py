import math


class Cartographie:
    """
    Carte progressive construite par le LIDAR.

    Grille de cellules (rows × cols) identiques à celles du labyrinthe.
    Pour chaque cellule on stocke :
      - decouverte   : True si le LIDAR a éclairé cette cellule
      - passages     : set de voisins (r,c) accessibles détectés
      - oeufs_detectes : liste de dicts {"x","y","rayon","collecte"}

    Convention de coordonnées identique au reste du projet :
      cellule (r, c)  →  coin bas-gauche monde : (x0 + c*cell, y0 + (rows-1-r)*cell)
    """

    def __init__(self, rows, cols, cell_size, x0, y0):
        self.rows      = rows
        self.cols      = cols
        self.cell      = cell_size
        self.x0        = x0
        self.y0        = y0

        # Grille de découverte
        self.decouverte = [[False] * cols for _ in range(rows)]

        # Passages connus entre cellules voisines (même format que `passages` du labyrinthe)
        # clé canonique : (min((r1,c1),(r2,c2)), max(...))
        self.passages_connus = set()

        # Oeufs repérés par le LIDAR (référence vers les dicts du main)
        self.oeufs_detectes = []

    # ── Conversion coordonnées ───────────────────────────────────────────────

    def monde_vers_cellule(self, x, y):
        """Retourne (r, c) de la cellule contenant le point monde (x, y)."""
        c = int((x - self.x0) / self.cell)
        r = self.rows - 1 - int((y - self.y0) / self.cell)
        c = max(0, min(self.cols - 1, c))
        r = max(0, min(self.rows - 1, r))
        return r, c

    def centre_cellule(self, r, c):
        x = self.x0 + c * self.cell + self.cell / 2
        y = self.y0 + (self.rows - 1 - r) * self.cell + self.cell / 2
        return x, y

    # ── Mise à jour depuis le LIDAR ──────────────────────────────────────────

    def mettre_a_jour(self, robot, lidar, oeufs_env):
        """
        Pour chaque rayon LIDAR :
          1. Marque la cellule du robot comme découverte.
          2. Parcourt les cellules traversées par le rayon → découvertes.
          3. À chaque franchissement de frontière entre deux cellules,
             si le rayon passe sans impact avant → enregistre un passage.
          4. Si le point d'impact correspond à un œuf → l'ajoute à oeufs_detectes.
        """
        rc_robot = self.monde_vers_cellule(robot.x, robot.y)
        self._marquer(rc_robot)

        for (angle, dist, xi, yi) in lidar.mesures:
            dx = math.cos(angle)
            dy = math.sin(angle)

            rc_prev = rc_robot
            self._marquer(rc_prev)

            # Pas de marche le long du rayon (doit être < cell/2 pour ne pas sauter de cellule)
            pas = self.cell * 0.4
            nb_pas = int(dist / pas) + 1

            for k in range(1, nb_pas + 1):
                t = min(k * pas, dist)
                px = robot.x + dx * t
                py = robot.y + dy * t
                rc_cur = self.monde_vers_cellule(px, py)

                self._marquer(rc_cur)

                # Franchissement de frontière → possible passage
                if rc_cur != rc_prev:
                    self._enregistrer_passage(rc_prev, rc_cur)
                    rc_prev = rc_cur

                if t >= dist:
                    break

            # Détection d'œuf au point d'impact
            for oeuf in oeufs_env:
                if oeuf.get("collecte"):
                    continue
                dist_impact = math.hypot(xi - oeuf["x"], yi - oeuf["y"])
                if dist_impact < oeuf["rayon"] + 0.3:
                    if oeuf not in self.oeufs_detectes:
                        self.oeufs_detectes.append(oeuf)

    # ── Helpers internes ─────────────────────────────────────────────────────

    def _marquer(self, rc):
        r, c = rc
        if 0 <= r < self.rows and 0 <= c < self.cols:
            self.decouverte[r][c] = True

    def _enregistrer_passage(self, rc1, rc2):
        """Enregistre un passage entre deux cellules voisines directes."""
        r1, c1 = rc1
        r2, c2 = rc2
        # On n'enregistre que les voisins directs (pas de diagonale)
        if abs(r1 - r2) + abs(c1 - c2) == 1:
            cle = (min(rc1, rc2), max(rc1, rc2))
            self.passages_connus.add(cle)

    # ── Accesseurs ────────────────────────────────────────────────────────────

    def est_decouverte(self, r, c):
        return self.decouverte[r][c]

    def nb_cellules_decouvertes(self):
        return sum(self.decouverte[r][c]
                   for r in range(self.rows)
                   for c in range(self.cols))