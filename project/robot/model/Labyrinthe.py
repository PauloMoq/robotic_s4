import random

# ---------------------------------------------------------------------------
# Labyrinthe — Recursive Backtracker (DFS)
#
# Fournit la génération procédurale du labyrinthe et les utilitaires
# de conversion entre coordonnées cellule et coordonnées monde.
#
# Usage :
#   laby = Labyrinthe(cols, rows, cell_size=2.0, epaisseur_mur=0.06)
#   laby.generer(seed=42)
#   murs    = laby.construire_murs()
#   x, y    = laby.centre_cellule_monde(r, c)
#   r, c    = laby.monde_vers_cellule(x, y)
# ---------------------------------------------------------------------------


class Labyrinthe:
    """
    Génère un labyrinthe parfait par Recursive Backtracker (DFS itératif).

    Attributs publics après appel à generer() :
        passages : set de paires canoniques ((r1,c1),(r2,c2)) représentant
                   les murs abattus (cellules voisines sans mur entre elles).
        x0, y0   : coin bas-gauche du labyrinthe en coordonnées monde.
    """

    def __init__(self, cols: int, rows: int,
                 cell_size: float = 2.0,
                 epaisseur_mur: float = 0.06):
        self.cols = cols
        self.rows = rows
        self.cell = cell_size
        self.e    = epaisseur_mur

        # Coin bas-gauche centré sur l'origine
        self.x0 = -cols * cell_size / 2
        self.y0 = -rows * cell_size / 2

        self.passages: set = set()

    # ── Génération DFS ───────────────────────────────────────────────────────

    def generer(self, seed=None) -> set:
        """
        Lance le Recursive Backtracker et remplit self.passages.
        Retourne également passages pour usage immédiat.
        """
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
        return passages

    # ── Conversion coordonnées ───────────────────────────────────────────────

    def cellule_vers_monde(self, r: int, c: int) -> tuple:
        """Retourne le coin bas-gauche de la cellule (r, c) en coordonnées monde."""
        x = self.x0 + c * self.cell
        y = self.y0 + (self.rows - 1 - r) * self.cell
        return x, y

    def centre_cellule_monde(self, r: int, c: int) -> tuple:
        """Retourne le centre de la cellule (r, c) en coordonnées monde."""
        x, y = self.cellule_vers_monde(r, c)
        return x + self.cell / 2, y + self.cell / 2

    def monde_vers_cellule(self, x: float, y: float) -> tuple:
        """Convertit des coordonnées monde en cellule (r, c), clampée à la grille."""
        c = round((x - self.x0 - self.cell / 2) / self.cell)
        r = self.rows - 1 - round((y - self.y0 - self.cell / 2) / self.cell)
        return max(0, min(self.rows - 1, r)), max(0, min(self.cols - 1, c))

    def cellule_aleatoire(self, exclure=None) -> tuple:
        """Retourne une cellule (r, c) aléatoire, différente de `exclure` si fourni."""
        while True:
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            if exclure is None or (r, c) != exclure:
                return r, c

    # ── Construction des murs ────────────────────────────────────────────────

    def construire_murs(self) -> list:
        """
        Convertit la grille et self.passages en liste de murs {x, y, w, h}
        compatibles avec Environnement.
        Inclut les bordures extérieures et les murs intérieurs non abattus.
        """
        murs = []

        def mh(x, y, w):
            if w > self.e:
                murs.append({"x": x, "y": y, "w": w, "h": self.e})

        def mv(x, y, h):
            if h > self.e:
                murs.append({"x": x, "y": y, "w": self.e, "h": h})

        larg = self.cols * self.cell
        haut = self.rows * self.cell

        # Bordures extérieures
        mh(self.x0, self.y0,        larg)
        mh(self.x0, self.y0 + haut, larg)
        mv(self.x0,        self.y0, haut)
        mv(self.x0 + larg, self.y0, haut)

        # Murs intérieurs
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