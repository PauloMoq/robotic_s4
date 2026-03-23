import pygame
import math

class VuePygame:
    def __init__(self, largeur=800, hauteur=600, scale=50):
        pygame.init()
        self.largeur = largeur
        self.hauteur = hauteur
        self.scale = scale
        self.screen = pygame.display.set_mode((largeur, hauteur))
        pygame.display.set_caption("Simulation Robot Mobile")
        self.clock = pygame.time.Clock()

        # Couleurs
        self.COLOR_BG       = (30, 33, 36)    # gris très foncé
        self.COLOR_GRID     = (50, 55, 60)    # grille subtile
        self.COLOR_ROBOT    = (0, 150, 255)   # bleu clair
        self.COLOR_DIR      = (255, 80, 80)   # rouge doux
        self.COLOR_BORDER   = (100, 100, 100) # bordure de map
        self.COLOR_MUR      = (180, 170, 140) # beige/sable pour les murs
        self.COLOR_MUR_BORD = (220, 210, 180) # contour mur
        self.COLOR_ARRIVEE  = (50, 220, 100)  # vert vif pour l'arrivée

    def convertir_coordonnees(self, x, y):
        px = int(self.largeur / 2 + (x * self.scale))
        py = int(self.hauteur / 2 - (y * self.scale))
        return px, py

    def dessiner_grille(self):
        for x in range(0, self.largeur, self.scale):
            pygame.draw.line(self.screen, self.COLOR_GRID, (x, 0), (x, self.hauteur))
        for y in range(0, self.hauteur, self.scale):
            pygame.draw.line(self.screen, self.COLOR_GRID, (0, y), (self.largeur, y))

    def dessiner_robot(self, robot):
        px, py = self.convertir_coordonnees(robot.x, robot.y)
        rayon_pixel = int(0.3 * self.scale)

        pygame.draw.circle(self.screen, self.COLOR_ROBOT, (px, py), rayon_pixel)
        pygame.draw.circle(self.screen, (255, 255, 255), (px, py), rayon_pixel, 1)

        x_dir = px + int(rayon_pixel * math.cos(robot.orientation))
        y_dir = py - int(rayon_pixel * math.sin(robot.orientation))
        pygame.draw.line(self.screen, self.COLOR_DIR, (px, py), (x_dir, y_dir), 4)

    def dessiner_environnement(self, env):
        for obs in env.obstacles:
            px, py = self.convertir_coordonnees(obs["x"], obs["y"])
            r_px = int(obs["rayon"] * self.scale)
            pygame.draw.circle(self.screen, (200, 50, 50), (px, py), r_px)
            pygame.draw.circle(self.screen, (255, 255, 255), (px, py), r_px, 1)

    def dessiner_murs(self, env):
        """Dessine les murs. Pour les murs très fins (<4px) on force 2px min
        afin qu ils restent visibles, coins arrondis uniquement si assez epais."""
        for mur in env.murs:
            px, py = self.convertir_coordonnees(mur["x"], mur["y"] + mur["h"])
            pw = max(2, int(mur["w"] * self.scale))
            ph = max(2, int(mur["h"] * self.scale))
            rect = pygame.Rect(px, py, pw, ph)
            rayon_coin = min(3, pw // 2, ph // 2) if min(pw, ph) >= 6 else 0
            pygame.draw.rect(self.screen, self.COLOR_MUR, rect, border_radius=rayon_coin)
            if min(pw, ph) >= 4:
                pygame.draw.rect(self.screen, self.COLOR_MUR_BORD, rect, 1, border_radius=rayon_coin)

    def dessiner_arrivee(self, env):
        """Dessine le point d'arrivée avec un effet de halo animé."""
        if env.arrivee is None:
            return
        px, py = self.convertir_coordonnees(env.arrivee["x"], env.arrivee["y"])
        r_px = int(env.arrivee["rayon"] * self.scale)

        # Halo transparent
        halo = pygame.Surface((r_px * 4, r_px * 4), pygame.SRCALPHA)
        pygame.draw.circle(halo, (50, 220, 100, 60), (r_px * 2, r_px * 2), r_px * 2)
        self.screen.blit(halo, (px - r_px * 2, py - r_px * 2))

        # Cercle plein
        pygame.draw.circle(self.screen, self.COLOR_ARRIVEE, (px, py), r_px)
        pygame.draw.circle(self.screen, (255, 255, 255), (px, py), r_px, 2)

        # Lettre "A" au centre
        font = pygame.font.SysFont("monospace", r_px, bold=True)
        texte = font.render("A", True, (255, 255, 255))
        self.screen.blit(texte, texte.get_rect(center=(px, py)))

    def afficher_victoire(self):
        """Affiche un message de victoire centré à l'écran."""
        overlay = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 150))
        self.screen.blit(overlay, (0, 0))

        font_grande = pygame.font.SysFont("monospace", 64, bold=True)
        font_petite = pygame.font.SysFont("monospace", 28)

        texte1 = font_grande.render("ARRIVÉE !", True, (50, 220, 100))
        texte2 = font_petite.render("Appuie sur ECHAP pour quitter", True, (200, 200, 200))

        self.screen.blit(texte1, texte1.get_rect(center=(self.largeur // 2, self.hauteur // 2 - 40)))
        self.screen.blit(texte2, texte2.get_rect(center=(self.largeur // 2, self.hauteur // 2 + 40)))



    def dessiner_chemin(self, controleur_auto):
        """Dessine le chemin BFS restant sous forme de ligne pointillée verte."""
        points = controleur_auto.chemin_restant
        if len(points) < 2:
            return
        pixels = [self.convertir_coordonnees(x, y) for (x, y) in points]
        for i in range(len(pixels) - 1):
            pygame.draw.line(self.screen, (50, 200, 80), pixels[i], pixels[i+1], 1)
        for px, py in pixels:
            pygame.draw.circle(self.screen, (50, 200, 80), (px, py), 3)

    def dessiner_lidar(self, robot, lidar):
        """
        Dessine les rayons lidar depuis le robot jusqu au point d impact.
          - Rayon : ligne jaune semi-transparente
          - Impact : petit cercle rouge au point de contact
        """
        if not lidar.mesures:
            return

        ox, oy = self.convertir_coordonnees(robot.x, robot.y)

        for (angle, dist, xi, yi) in lidar.mesures:
            px, py = self.convertir_coordonnees(xi, yi)

            # Rayon semi-transparent
            surf = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)
            pygame.draw.line(surf, (255, 220, 50, 60), (ox, oy), (px, py), 1)
            self.screen.blit(surf, (0, 0))

            # Point d impact
            pygame.draw.circle(self.screen, (255, 80, 80), (px, py), 3)

    def tick(self, fps=60):
        self.clock.tick(fps)

    # ── Dessin des œufs ──────────────────────────────────────────────────────
    def dessiner_oeufs(self, oeufs, rayon_oeuf=0.18):
        """Dessine les œufs restants sous forme d'ellipse jaune/blanche."""
        for oeuf in oeufs:
            if oeuf["collecte"]:
                continue
            px, py = self.convertir_coordonnees(oeuf["x"], oeuf["y"])
            rw = int(rayon_oeuf * self.scale * 1.1)
            rh = int(rayon_oeuf * self.scale * 1.5)
            rect = pygame.Rect(px - rw, py - rh, rw * 2, rh * 2)
            pygame.draw.ellipse(self.screen, (255, 230, 60), rect)
            pygame.draw.ellipse(self.screen, (255, 255, 255), rect, 2)

    def dessiner_compteur_oeufs(self, oeufs, nb_total):
        """Affiche le compteur d'œufs restants en haut à gauche."""
        restants = sum(1 for o in oeufs if not o["collecte"])
        font = pygame.font.SysFont("monospace", 26, bold=True)
        texte = font.render(f"Oeufs restants : {restants} / {nb_total}", True, (255, 230, 60))
        self.screen.blit(texte, (20, 16))

    def dessiner_timer(self, timer):
        """Affiche le timer en haut à droite."""
        minutes = int(timer) // 60
        secondes = int(timer) % 60
        centimes = int((timer % 1) * 100)
        texte_timer = f"{minutes:02d}:{secondes:02d}.{centimes:02d}"
        font = pygame.font.SysFont("monospace", 26, bold=True)
        surf = font.render(texte_timer, True, (200, 200, 200))
        self.screen.blit(surf, (self.largeur - surf.get_width() - 20, 16))

    def afficher_victoire_oeufs(self, timer=None):
        """Overlay de victoire pour la chasse aux œufs."""
        overlay = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 160))
        self.screen.blit(overlay, (0, 0))

        font_big   = pygame.font.SysFont("monospace", 64, bold=True)
        font_small = pygame.font.SysFont("monospace", 28)

        t1 = font_big.render("Tous les oeufs recuperes !", True, (255, 230, 60))
        self.screen.blit(t1, (self.largeur // 2 - t1.get_width() // 2, self.hauteur // 2 - 80))

        if timer is not None:
            minutes = int(timer) // 60
            secondes = int(timer) % 60
            centimes = int((timer % 1) * 100)
            t2 = font_small.render(f"Temps : {minutes:02d}:{secondes:02d}.{centimes:02d}", True, (255, 255, 255))
            self.screen.blit(t2, (self.largeur // 2 - t2.get_width() // 2, self.hauteur // 2 + 10))

        t3 = font_small.render("Appuyez sur ECHAP pour quitter", True, (160, 160, 160))
        self.screen.blit(t3, (self.largeur // 2 - t3.get_width() // 2, self.hauteur // 2 + 60))

    # ── Carte d'occupation ───────────────────────────────────────────────────
    def dessiner_carte(self, carte):
        """
        Overlay semi-transparent de la carte d'occupation.
          - INCONNUE : carré noir semi-transparent (brouillard de guerre)
          - LIBRE    : rien (transparent)
          - MUR      : carré rouge semi-transparent
        """
        from robot.model.CarteOccupation import INCONNUE, MUR
        pw = int(carte.cell * self.scale)
        ph = int(carte.cell * self.scale)

        surf = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)

        for r in range(carte.rows):
            for c in range(carte.cols):
                etat = carte.etat(r, c)
                if etat == INCONNUE:
                    cx, cy = carte.cellule_vers_centre(r, c)
                    px, py = self.convertir_coordonnees(cx - carte.cell/2,
                                                        cy + carte.cell/2)
                    pygame.draw.rect(surf, (0, 0, 0, 140),
                                     pygame.Rect(px, py, pw, ph))
                elif etat == MUR:
                    cx, cy = carte.cellule_vers_centre(r, c)
                    px, py = self.convertir_coordonnees(cx - carte.cell/2,
                                                        cy + carte.cell/2)
                    pygame.draw.rect(surf, (200, 50, 50, 80),
                                     pygame.Rect(px, py, pw, ph))

        self.screen.blit(surf, (0, 0))

    def dessiner_etat_exploration(self, controleur):
        """Affiche l'état du contrôleur (EXPLORATION / COLLECTE) en bas à gauche."""
        font = pygame.font.SysFont("monospace", 20, bold=True)
        etat = controleur.etat.upper()
        couleur = (80, 200, 120) if etat == "EXPLORATION" else (255, 200, 50)
        surf = font.render(f"Mode : {etat}", True, couleur)
        self.screen.blit(surf, (20, self.hauteur - 36))

    def dessiner_cartographie(self, carto):
        """
        Recouvre d'un voile noir opaque toutes les cellules non encore découvertes.
        À appeler APRÈS avoir dessiné les murs, AVANT le robot.
        """
        taille_px = int(carto.cell * self.scale) + 1  # +1 pour éviter les gaps

        for r in range(carto.rows):
            for c in range(carto.cols):
                if not carto.est_decouverte(r, c):
                    # Coin bas-gauche de la cellule en monde
                    x_monde = carto.x0 + c * carto.cell
                    y_monde = carto.y0 + (carto.rows - 1 - r) * carto.cell
                    # Coin haut-gauche en pixels (y_monde + cell = coin haut)
                    px, py = self.convertir_coordonnees(x_monde, y_monde + carto.cell)
                    rect = pygame.Rect(px, py, taille_px, taille_px)
                    pygame.draw.rect(self.screen, (0, 0, 0), rect)

    def dessiner_oeufs_detectes(self, carto, rayon_oeuf=0.18):
        """
        Affiche uniquement les œufs que le LIDAR a déjà détectés.
        (Remplace dessiner_oeufs qui affichait TOUS les œufs.)
        """
        for oeuf in carto.oeufs_detectes:
            if oeuf.get("collecte"):
                continue
            px, py = self.convertir_coordonnees(oeuf["x"], oeuf["y"])
            rw = int(rayon_oeuf * self.scale * 1.1)
            rh = int(rayon_oeuf * self.scale * 1.5)
            rect = pygame.Rect(px - rw, py - rh, rw * 2, rh * 2)
            pygame.draw.ellipse(self.screen, (255, 230, 60), rect)
            pygame.draw.ellipse(self.screen, (255, 255, 255), rect, 2)