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
        self.COLOR_BG       = (30, 33, 36)
        self.COLOR_GRID     = (50, 55, 60)
        self.COLOR_ROBOT    = (0, 150, 255)
        self.COLOR_DIR      = (255, 80, 80)
        self.COLOR_BORDER   = (100, 100, 100)
        self.COLOR_MUR      = (180, 170, 140)
        self.COLOR_MUR_BORD = (220, 210, 180)
        self.COLOR_ARRIVEE  = (50, 220, 100)

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

        halo = pygame.Surface((r_px * 4, r_px * 4), pygame.SRCALPHA)
        pygame.draw.circle(halo, (50, 220, 100, 60), (r_px * 2, r_px * 2), r_px * 2)
        self.screen.blit(halo, (px - r_px * 2, py - r_px * 2))

        pygame.draw.circle(self.screen, self.COLOR_ARRIVEE, (px, py), r_px)
        pygame.draw.circle(self.screen, (255, 255, 255), (px, py), r_px, 2)

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

            surf = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)
            pygame.draw.line(surf, (255, 220, 50, 60), (ox, oy), (px, py), 1)
            self.screen.blit(surf, (0, 0))

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
        minutes  = int(timer) // 60
        secondes = int(timer) % 60
        centimes = int((timer % 1) * 100)
        texte_timer = f"{minutes:02d}:{secondes:02d}.{centimes:02d}"
        font = pygame.font.SysFont("monospace", 26, bold=True)
        surf = font.render(texte_timer, True, (200, 200, 200))
        self.screen.blit(surf, (self.largeur - surf.get_width() - 20, 16))

    def dessiner_type_moteur(self, robot):
        """Affiche le type de moteur juste sous le chrono (haut à droite)."""
        if robot.moteur is None:
            label   = "Aucun moteur"
            couleur = (160, 160, 160)
        elif hasattr(robot.moteur, 'vx'):
            label   = "Omnidirectionnel"
            couleur = (80, 200, 255)
        else:
            label   = "Differentiel"
            couleur = (255, 180, 60)

        font = pygame.font.SysFont("monospace", 20, bold=True)
        surf = font.render(f"Moteur : {label}", True, couleur)
        self.screen.blit(surf, (self.largeur - surf.get_width() - 20, 50))

    def dessiner_mode_auto(self, mode_auto: bool):
        """Affiche le mode de conduite (AUTONOME / MANUEL) sous le type de moteur."""
        label   = "AUTONOME" if mode_auto else "MANUEL"
        couleur = (80, 200, 120) if mode_auto else (255, 180, 60)
        font = pygame.font.SysFont("monospace", 20, bold=True)
        surf = font.render(f"Mode : {label}", True, couleur)
        self.screen.blit(surf, (self.largeur - surf.get_width() - 20, 76))

    def dessiner_controles(self, robot, avec_regenerer=False,
                           avec_mode_auto=False, avec_pause=False,
                           avec_switch_moteur=False):
        """
        Panneau des touches en bas à droite.

        avec_mode_auto      : TAB = Auto/Manuel (03, 04, 06)
        avec_pause          : ESPACE = Pause (06)
        avec_switch_moteur  : M = switch moteur (06, quand mode auto coexiste)
        avec_regenerer      : R = Regénérer (02, 03, 04, 05, 06)
        """
        if avec_mode_auto:
            lignes = [
                ("↑ ↓ ← →", "Translater (manuel)"),
                ("Q / D",   "Rotation (manuel)"),
                ("TAB",     "Auto / Manuel"),
            ]
            if avec_switch_moteur:
                lignes.append(("M", "Switch moteur"))
        elif robot.moteur is not None and hasattr(robot.moteur, 'vx'):
            lignes = [
                ("↑ ↓ ← →", "Translater"),
                ("Q / D",   "Rotation"),
                ("TAB",     "Moteur Differentiel"),
            ]
        else:
            lignes = [
                ("↑ / ↓",  "Avancer / Reculer"),
                ("← / →",  "Tourner"),
                ("TAB",    "Moteur Omnidirectionnel"),
            ]

        if avec_pause:
            lignes.append(("ESPACE", "Pause / Reprendre"))
        if avec_regenerer:
            lignes.append(("R", "Regenerer"))
        lignes.append(("ECHAP", "Quitter"))

        font_titre = pygame.font.SysFont("monospace", 18, bold=True)
        font_ligne = pygame.font.SysFont("monospace", 17)
        PAD     = 12
        LIGNE_H = 22
        LARGEUR = 280

        hauteur_bloc = PAD * 2 + font_titre.get_height() + 6 + len(lignes) * LIGNE_H
        x0 = self.largeur - LARGEUR - 16
        y0 = self.hauteur - hauteur_bloc - 16

        fond = pygame.Surface((LARGEUR, hauteur_bloc), pygame.SRCALPHA)
        fond.fill((0, 0, 0, 140))
        self.screen.blit(fond, (x0, y0))
        pygame.draw.rect(self.screen, (80, 80, 100),
                         pygame.Rect(x0, y0, LARGEUR, hauteur_bloc), 1)

        titre = font_titre.render("CONTROLES", True, (200, 200, 220))
        self.screen.blit(titre, (x0 + PAD, y0 + PAD))
        y = y0 + PAD + titre.get_height() + 6

        for touche, action in lignes:
            s_touche = font_ligne.render(touche, True, (255, 220, 80))
            s_action = font_ligne.render(action, True, (180, 180, 180))
            self.screen.blit(s_touche, (x0 + PAD, y))
            self.screen.blit(s_action, (x0 + PAD + 100, y))
            y += LIGNE_H

    def dessiner_hud(self, oeufs, nb_total, carto):
        """
        Affiche en haut à gauche sur une ligne :
          Oeufs : X / N      Carte : XX %
        Utilisé par le 06 (cartographie autonome).
        """
        font = pygame.font.SysFont("monospace", 26, bold=True)
        restants   = sum(1 for o in oeufs if not o["collecte"])
        surf_oeufs = font.render(f"Oeufs : {restants} / {nb_total}", True, (255, 230, 60))
        self.screen.blit(surf_oeufs, (20, 16))

        total       = carto.rows * carto.cols
        decouvertes = carto.nb_cellules_decouvertes()
        pct         = decouvertes * 100 // total
        surf_carte  = font.render(f"Carte : {pct} %", True, (120, 200, 255))
        self.screen.blit(surf_carte, (20 + surf_oeufs.get_width() + 40, 16))

    def afficher_pause(self):
        """Overlay semi-transparent PAUSE centré."""
        overlay = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 120))
        self.screen.blit(overlay, (0, 0))

        font_big   = pygame.font.SysFont("monospace", 72, bold=True)
        font_small = pygame.font.SysFont("monospace", 26)

        t1 = font_big.render("PAUSE", True, (255, 255, 255))
        t2 = font_small.render("ESPACE pour reprendre", True, (180, 180, 180))
        self.screen.blit(t1, t1.get_rect(center=(self.largeur // 2, self.hauteur // 2 - 40)))
        self.screen.blit(t2, t2.get_rect(center=(self.largeur // 2, self.hauteur // 2 + 40)))

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
            minutes  = int(timer) // 60
            secondes = int(timer) % 60
            centimes = int((timer % 1) * 100)
            t2 = font_small.render(f"Temps : {minutes:02d}:{secondes:02d}.{centimes:02d}", True, (255, 255, 255))
            self.screen.blit(t2, (self.largeur // 2 - t2.get_width() // 2, self.hauteur // 2 + 10))

        t3 = font_small.render("Appuyez sur ECHAP pour quitter", True, (160, 160, 160))
        self.screen.blit(t3, (self.largeur // 2 - t3.get_width() // 2, self.hauteur // 2 + 60))

    # ── Carte d'occupation ───────────────────────────────────────────────────

    def dessiner_carte(self, carte):
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
        Rendu fog-of-war propre :
          - Cellule non découverte  → carré noir plein
          - Cellule découverte      → fond gris foncé + murs sur les arêtes sans passage
        """
        taille_px = int(carto.cell * self.scale) + 1
        ep_mur    = max(5, int(self.scale * 0.18))
        COLOR_FOG   = (0, 0, 0)
        COLOR_CELL  = (42, 46, 50)
        COLOR_MUR   = (200, 185, 140)
        COLOR_MUR_B = (255, 240, 180)

        for r in range(carto.rows):
            for c in range(carto.cols):
                x_monde = carto.x0 + c * carto.cell
                y_monde = carto.y0 + (carto.rows - 1 - r) * carto.cell
                px, py  = self.convertir_coordonnees(x_monde, y_monde + carto.cell)
                rect    = pygame.Rect(px, py, taille_px, taille_px)

                if not carto.est_decouverte(r, c):
                    pygame.draw.rect(self.screen, COLOR_FOG, rect)
                    continue

                pygame.draw.rect(self.screen, COLOR_CELL, rect)

                for vr, vc, cote in [(r-1,c,"haut"),(r+1,c,"bas"),(r,c-1,"gauche"),(r,c+1,"droite")]:
                    hors_grille = not (0 <= vr < carto.rows and 0 <= vc < carto.cols)
                    cle = (min((r,c),(vr,vc)), max((r,c),(vr,vc)))
                    est_mur = hors_grille or (cle not in carto.passages)
                    if not est_mur:
                        continue
                    if cote == "haut":
                        mr = pygame.Rect(px, py, taille_px, ep_mur)
                    elif cote == "bas":
                        mr = pygame.Rect(px, py + taille_px - ep_mur, taille_px, ep_mur)
                    elif cote == "gauche":
                        mr = pygame.Rect(px, py, ep_mur, taille_px)
                    else:
                        mr = pygame.Rect(px + taille_px - ep_mur, py, ep_mur, taille_px)
                    pygame.draw.rect(self.screen, COLOR_MUR,   mr)
                    pygame.draw.rect(self.screen, COLOR_MUR_B, mr, 1)

    def dessiner_oeufs_detectes(self, carto, rayon_oeuf=0.18):
        """Affiche uniquement les œufs que le LIDAR a déjà détectés."""
        for oeuf in carto.oeufs_detectes:
            if oeuf.get("collecte"):
                continue
            px, py = self.convertir_coordonnees(oeuf["x"], oeuf["y"])
            rw = int(rayon_oeuf * self.scale * 1.1)
            rh = int(rayon_oeuf * self.scale * 1.5)
            rect = pygame.Rect(px - rw, py - rh, rw * 2, rh * 2)
            pygame.draw.ellipse(self.screen, (255, 230, 60), rect)
            pygame.draw.ellipse(self.screen, (255, 255, 255), rect, 2)