import math
from collections import deque

ETAT_EXPLORATION = "exploration"
ETAT_COLLECTE    = "collecte"
ETAT_SORTIE      = "sortie"


class ControleurExploration:
    """
    Contrôleur autonome pour la cartographie progressive avec collecte prioritaire.

    Logique stable :
      - On fixe une cible (cellule inconnue la plus proche OU œuf détecté).
      - On calcule le BFS UNE seule fois vers cette cible.
      - On suit le chemin jusqu'au bout sans recalculer.
      - Seulement à l'arrivée (ou si un œuf est détecté), on choisit une nouvelle cible.

    Machine à états :
      EXPLORATION — va vers la cellule inconnue la plus proche
      COLLECTE    — va vers un œuf détecté (priorité)
      SORTIE      — tous les œufs collectés, va vers la sortie du labyrinthe
    """

    def __init__(self, robot, lidar, carto, passages_reels, laby,
                 rc_depart, v_max=2.0, omega_max=3.0):
        """
        robot          : RobotMobile
        lidar          : Lidar
        carto          : Cartographie
        passages_reels : set de passages réels du labyrinthe (depuis Labyrinthe.passages)
        laby           : instance de Labyrinthe (pour conversions coords)
        rc_depart      : (row, col) de départ du robot
        v_max          : vitesse max (unités/s)
        omega_max      : vitesse angulaire max (rad/s)
        """
        self.robot          = robot
        self.lidar          = lidar
        self.carto          = carto
        self.passages_reels = passages_reels
        self.laby           = laby
        self.cell           = laby.cell
        self.rows           = laby.rows
        self.cols           = laby.cols
        self.v_max          = v_max
        self.omega_max      = omega_max

        self._etat        = ETAT_EXPLORATION
        self.chemin_monde = []
        self.idx_waypoint = 0
        self.rc_cible     = None
        self.cible_oeuf   = None

        self._fixer_prochaine_cible(rc_depart)

    # ── BFS utilitaires (privés) ─────────────────────────────────────────────

    def _bfs_vers(self, depart, arrivee, passages_graph) -> list:
        """BFS départ→arrivée sur passages_graph. Retourne la liste de cellules ou []."""
        queue = deque([depart])
        pred  = {depart: None}
        while queue:
            rc = queue.popleft()
            if rc == arrivee:
                break
            r, c = rc
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                v = (r + dr, c + dc)
                if v in pred:
                    continue
                if not (0 <= v[0] < self.rows and 0 <= v[1] < self.cols):
                    continue
                if (min(rc, v), max(rc, v)) in passages_graph:
                    pred[v] = rc
                    queue.append(v)
        if arrivee not in pred:
            return []
        chemin, cur = [], arrivee
        while cur is not None:
            chemin.append(cur)
            cur = pred.get(cur)
        chemin.reverse()
        return chemin

    def _bfs_cellule_inconnue_proche(self, depart) -> tuple:
        """
        BFS depuis depart sur passages_reels.
        Retourne la cellule accessible la plus proche non encore découverte,
        ou None si tout est exploré.
        """
        queue   = deque([depart])
        visited = {depart}
        while queue:
            rc = queue.popleft()
            r, c = rc
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                v = (r + dr, c + dc)
                if v in visited:
                    continue
                if not (0 <= v[0] < self.rows and 0 <= v[1] < self.cols):
                    continue
                if (min(rc, v), max(rc, v)) not in self.passages_reels:
                    continue
                visited.add(v)
                if not self.carto.est_decouverte(v[0], v[1]):
                    return v
                queue.append(v)
        return None

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _norm_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── Fixation de la cible ─────────────────────────────────────────────────

    def _fixer_prochaine_cible(self, rc_robot=None):
        """
        Choisit la prochaine cible et calcule le chemin BFS.
        N'est appelé QUE quand la destination précédente est atteinte
        ou lors d'une interruption pour collecte d'œuf.
        """
        if rc_robot is None:
            rc_robot = self.laby.monde_vers_cellule(self.robot.x, self.robot.y)

        if self._etat == ETAT_COLLECTE and self.cible_oeuf is not None:
            rc_cible = self.laby.monde_vers_cellule(self.cible_oeuf["x"],
                                                    self.cible_oeuf["y"])
        elif self._etat == ETAT_SORTIE:
            rc_cible = tuple(self.laby.sortie["rc"])
        else:
            rc_cible = self._bfs_cellule_inconnue_proche(rc_robot)

        if rc_cible is None:
            self.rc_cible     = None
            self.chemin_monde = []
            self.idx_waypoint = 0
            return

        self.rc_cible = rc_cible

        # BFS sur passages connus, fallback passages réels
        chemin = self._bfs_vers(rc_robot, rc_cible, self.carto.passages_connus)
        if not chemin:
            chemin = self._bfs_vers(rc_robot, rc_cible, self.passages_reels)

        self.chemin_monde = [self.laby.centre_cellule_monde(r, c) for r, c in chemin]
        self.idx_waypoint = 0

    # ── Mise à jour de l'état ─────────────────────────────────────────────────

    def mettre_a_jour_etat(self, oeufs, tous_collectes=False):
        """
        Appelé chaque frame. Gère trois cas :
          1. Œuf collecté → retour en EXPLORATION
          2. Nouvel œuf détecté → interruption et switch en COLLECTE
          3. Tous les œufs collectés → switch en SORTIE
        """
        # ── Switch vers SORTIE ───────────────────────────────────────────────
        if tous_collectes and self._etat != ETAT_SORTIE:
            print("[État] → SORTIE")
            self._etat      = ETAT_SORTIE
            self.cible_oeuf = None
            rc_robot = self.laby.monde_vers_cellule(self.robot.x, self.robot.y)
            self._fixer_prochaine_cible(rc_robot)
            return

        # ── En mode SORTIE : rien à changer jusqu'à sortie physique ─────────
        if self._etat == ETAT_SORTIE:
            return

        # ── Œuf collecté → retour exploration ───────────────────────────────
        if self._etat == ETAT_COLLECTE:
            if self.cible_oeuf is None or self.cible_oeuf["collecte"]:
                print("[État] → EXPLORATION")
                self._etat      = ETAT_EXPLORATION
                self.cible_oeuf = None
                rc_robot = self.laby.monde_vers_cellule(self.robot.x, self.robot.y)
                self._fixer_prochaine_cible(rc_robot)

        # ── Œuf détecté → interruption et collecte ──────────────────────────
        elif self._etat == ETAT_EXPLORATION:
            detectes = [o for o in self.carto.oeufs_detectes if not o["collecte"]]
            if detectes:
                nouveau = min(detectes,
                              key=lambda o: math.hypot(self.robot.x - o["x"],
                                                       self.robot.y - o["y"]))
                if nouveau is not self.cible_oeuf:
                    print(f"[État] → COLLECTE ({nouveau['x']:.1f}, {nouveau['y']:.1f})")
                    self._etat      = ETAT_COLLECTE
                    self.cible_oeuf = nouveau
                    rc_robot = self.laby.monde_vers_cellule(self.robot.x, self.robot.y)
                    self._fixer_prochaine_cible(rc_robot)

    # ── Commande principale ───────────────────────────────────────────────────

    def calculer_commande(self) -> dict:
        """
        Retourne la commande adaptée au moteur du robot :
          - MoteurOmnidirectionnel : {"vx", "vy", "omega"}
          - MoteurDifferentiel     : {"v", "omega"}
        """
        est_omni = hasattr(self.robot.moteur, 'vx')

        if not self.chemin_monde:
            self._fixer_prochaine_cible()
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0} if est_omni else {"v": 0.0, "omega": 0.0}

        while self.idx_waypoint < len(self.chemin_monde) - 1:
            wx, wy = self.chemin_monde[self.idx_waypoint]
            if math.hypot(self.robot.x - wx, self.robot.y - wy) < self.cell * 0.3:
                self.idx_waypoint += 1
            else:
                break

        wx, wy  = self.chemin_monde[self.idx_waypoint]
        dx      = wx - self.robot.x
        dy      = wy - self.robot.y
        dist_wp = math.hypot(dx, dy)

        if self.idx_waypoint == len(self.chemin_monde) - 1 and dist_wp < self.cell * 0.3:
            self._fixer_prochaine_cible()
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0} if est_omni else {"v": 0.0, "omega": 0.0}

        if dist_wp < 1e-6:
            return {"vx": 0.0, "vy": 0.0, "omega": 0.0} if est_omni else {"v": 0.0, "omega": 0.0}

        angle_cible = math.atan2(dy, dx)
        theta       = self.robot.orientation
        delta       = self._norm_angle(angle_cible - theta)
        omega       = max(-self.omega_max, min(self.omega_max, 2.0 * delta))

        if est_omni:
            nx = dx / dist_wp
            ny = dy / dist_wp
            vx_robot =  nx * math.cos(theta) + ny * math.sin(theta)
            vy_robot = -nx * math.sin(theta) + ny * math.cos(theta)
            return {
                "vx":    self.v_max * vx_robot,
                "vy":    self.v_max * vy_robot,
                "omega": omega,
            }
        else:
            # Différentiel : on avance seulement quand bien orienté
            # Plus l'écart angulaire est grand, plus on ralentit
            facteur_v = max(0.0, 1.0 - abs(delta) / (math.pi / 2))
            return {
                "v":     self.v_max * facteur_v,
                "omega": omega,
            }

    @property
    def chemin_restant(self):
        return self.chemin_monde[self.idx_waypoint:]

    @property
    def etat(self):
        return self._etat