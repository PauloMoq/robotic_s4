from robot.model.Labyrinthe import Labyrinthe

COLS     = 11
ROWS     = 6
NB_OEUFS = 3


def afficher_labyrinthe(laby, oeufs):
    sortie      = getattr(laby, "sortie", {})
    rc_sortie   = tuple(sortie.get("rc", (-1, -1))) if sortie else (-1, -1)
    cote_sortie = sortie.get("cote", "") if sortie else ""

    oeufs_par_cellule = {}
    for oeuf in oeufs:
        rc = laby.monde_vers_cellule(oeuf["x"], oeuf["y"])
        oeufs_par_cellule[rc] = oeuf

    lignes = []

    for r in range(laby.rows):
        ligne_h = "+"
        for c in range(laby.cols):
            if r == 0:
                est_trou = (rc_sortie == (r, c) and cote_sortie == "haut")
                ligne_h += "   " if est_trou else "---"
            else:
                cle = (min((r-1, c), (r, c)), max((r-1, c), (r, c)))
                ligne_h += "   " if cle in laby.passages else "---"
            ligne_h += "+"
        lignes.append(ligne_h)

        ligne_c = ""
        for c in range(laby.cols):
            if c == 0:
                est_trou  = (rc_sortie == (r, c) and cote_sortie == "gauche")
                ligne_c  += " " if est_trou else "|"
            else:
                cle = (min((r, c-1), (r, c)), max((r, c-1), (r, c)))
                ligne_c += " " if cle in laby.passages else "|"

            if (r, c) == rc_sortie:
                symbole = "S"
            elif (r, c) in oeufs_par_cellule:
                symbole = "o"
            else:
                symbole = "."
            ligne_c += f" {symbole} "

        est_trou = (rc_sortie == (r, laby.cols - 1) and cote_sortie == "droite")
        ligne_c += " " if est_trou else "|"
        lignes.append(ligne_c)

    ligne_h = "+"
    for c in range(laby.cols):
        est_trou = (rc_sortie == (laby.rows - 1, c) and cote_sortie == "bas")
        ligne_h += "   " if est_trou else "---"
        ligne_h += "+"
    lignes.append(ligne_h)

    print("\n".join(lignes))


def afficher_legende(laby, oeufs):
    sortie = getattr(laby, "sortie", {})

    print()
    print("  Légende :")
    print("    .  cellule vide")
    print("    o  œuf")
    print("    S  sortie")
    print()
    print(f"  Grille   : {laby.cols}×{laby.rows}")
    print(f"  Passages : {len(laby.passages)}")
    print(f"  Œufs     : {len(oeufs)}")

    if sortie:
        print(f"  Sortie   : cellule {sortie['rc']}, côté {sortie['cote']}")
        print(f"             monde ({sortie['x']:.2f}, {sortie['y']:.2f}), rayon {sortie['rayon']:.2f}")
    else:
        print("  Sortie   : non disponible (mets à jour Labyrinthe.py)")

    for i, oeuf in enumerate(oeufs):
        rc = laby.monde_vers_cellule(oeuf["x"], oeuf["y"])
        print(f"  Œuf {i+1}    : cellule {rc}, monde ({oeuf['x']:.2f}, {oeuf['y']:.2f})")
    print()


if __name__ == "__main__":
    laby = Labyrinthe(COLS, ROWS)
    laby.generer()
    oeufs = laby.placer_oeufs(NB_OEUFS)

    print("\n=== DEBUG — Labyrinthe ===\n")
    afficher_labyrinthe(laby, oeufs)
    afficher_legende(laby, oeufs)