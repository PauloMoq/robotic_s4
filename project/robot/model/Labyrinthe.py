import random


class Labyrinthe:
    def __init__(self, cols: int, rows: int,
                 cell_size: float = 2.0,
                 epaisseur_mur: float = 0.06):
        self.cols = cols
        self.rows = rows
        self.cell = cell_size
        self.e    = epaisseur_mur

        self.x0 = -cols * cell_size / 2
        self.y0 = -rows * cell_size / 2

        self.passages: set = set()
        self.sortie: dict  = {}

    def generer(self, seed=None) -> set:
        if seed is not None:
            random.seed(seed)

        visited  = [[False] * self.cols for _ in range(self.rows)]
        passages = set()

        def voisins(r, c):
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols and not visited[nr][nc]:
                    yield (nr, nc)

        r0 = random.randint(0, self.rows - 1)
        c0 = random.randint(0, self.cols - 1)
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

        self.passages = passages
        self._generer_sortie(rc_exclue=(r0, c0))
        return passages

    def _generer_sortie(self, rc_exclue=None):
        candidats = []
        for c in range(self.cols):
            candidats.append((0,             c, "haut"))
            candidats.append((self.rows - 1, c, "bas"))
        for r in range(self.rows):
            candidats.append((r, 0,             "gauche"))
            candidats.append((r, self.cols - 1, "droite"))

        if rc_exclue is not None:
            candidats = [(r, c, cote) for r, c, cote in candidats
                         if (r, c) != rc_exclue]

        r, c, cote = random.choice(candidats)
        cx, cy = self.centre_cellule_monde(r, c)

        if cote == "haut":
            sx, sy = cx, cy + self.cell / 2
        elif cote == "bas":
            sx, sy = cx, cy - self.cell / 2
        elif cote == "gauche":
            sx, sy = cx - self.cell / 2, cy
        else:
            sx, sy = cx + self.cell / 2, cy

        self.sortie = {
            "x":     sx,
            "y":     sy,
            "rayon": self.cell * 0.4,
            "rc":    (r, c),
            "cote":  cote,
        }

    def cellule_vers_monde(self, r: int, c: int) -> tuple:
        x = self.x0 + c * self.cell
        y = self.y0 + (self.rows - 1 - r) * self.cell
        return x, y

    def centre_cellule_monde(self, r: int, c: int) -> tuple:
        x, y = self.cellule_vers_monde(r, c)
        return x + self.cell / 2, y + self.cell / 2

    def monde_vers_cellule(self, x: float, y: float) -> tuple:
        c = round((x - self.x0 - self.cell / 2) / self.cell)
        r = self.rows - 1 - round((y - self.y0 - self.cell / 2) / self.cell)
        return max(0, min(self.rows - 1, r)), max(0, min(self.cols - 1, c))

    def cellule_aleatoire(self, exclure=None) -> tuple:
        while True:
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            if exclure is None or (r, c) != exclure:
                return r, c

    def placer_oeufs(self, nb: int, rayon: float = 0.18,
                     cellule_exclue=None) -> list:
        rc_sortie = self.sortie.get("rc")
        cellules: set = set()
        while len(cellules) < nb:
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            if cellule_exclue is not None and (r, c) == cellule_exclue:
                continue
            if rc_sortie is not None and (r, c) == rc_sortie:
                continue
            cellules.add((r, c))
        oeufs = []
        for r, c in cellules:
            x, y = self.centre_cellule_monde(r, c)
            oeufs.append({"x": x, "y": y, "rayon": rayon, "collecte": False})
        return oeufs

    def construire_murs(self) -> list:
        murs = []

        def mh(x, y, w):
            if w > self.e:
                murs.append({"x": x, "y": y, "w": w, "h": self.e})

        def mv(x, y, h):
            if h > self.e:
                murs.append({"x": x, "y": y, "w": self.e, "h": h})

        larg       = self.cols * self.cell
        haut       = self.rows * self.cell
        scote      = self.sortie["cote"]
        cx_s, cy_s = self.cellule_vers_monde(self.sortie["rc"][0], self.sortie["rc"][1])

        if scote == "bas":
            mh(self.x0,          self.y0, cx_s - self.x0)
            mh(cx_s + self.cell, self.y0, self.x0 + larg - (cx_s + self.cell))
        else:
            mh(self.x0, self.y0, larg)

        if scote == "haut":
            mh(self.x0,          self.y0 + haut, cx_s - self.x0)
            mh(cx_s + self.cell, self.y0 + haut, self.x0 + larg - (cx_s + self.cell))
        else:
            mh(self.x0, self.y0 + haut, larg)

        if scote == "gauche":
            mv(self.x0, self.y0,          cy_s - self.y0)
            mv(self.x0, cy_s + self.cell, self.y0 + haut - (cy_s + self.cell))
        else:
            mv(self.x0, self.y0, haut)

        if scote == "droite":
            mv(self.x0 + larg, self.y0,          cy_s - self.y0)
            mv(self.x0 + larg, cy_s + self.cell, self.y0 + haut - (cy_s + self.cell))
        else:
            mv(self.x0 + larg, self.y0, haut)

        for r in range(self.rows):
            for c in range(self.cols):
                cx, cy = self.cellule_vers_monde(r, c)

                if r + 1 < self.rows:
                    cle = (min((r, c), (r + 1, c)), max((r, c), (r + 1, c)))
                    if cle not in self.passages:
                        mh(cx, cy, self.cell)

                if c + 1 < self.cols:
                    cle = (min((r, c), (r, c + 1)), max((r, c), (r, c + 1)))
                    if cle not in self.passages:
                        mv(cx + self.cell, cy, self.cell)

        return murs