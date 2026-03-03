class VueTerminal:
    def afficher_etat(self, robot):
        """Affiche la position et l'orientation du robot dans la console."""
        print(f"[{robot.__class__.__name__}] Pos: ({robot.x:>6.2f}, {robot.y:>6.2f}) | "
              f"Ori: {robot.orientation:>5.2f} rad")

    def message(self, texte):
        print(f"[Info] : {texte}")