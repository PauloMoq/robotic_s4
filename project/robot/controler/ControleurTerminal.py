from abc import ABC, abstractmethod

class Controleur(ABC):
    def __init__(self, robot):
        self.robot = robot

    @abstractmethod
    def lire_commande(self):
        pass

    def mettre_a_jour_robot(self, dt: float):
        commande = self.lire_commande()
        if commande:
            self.robot.commander(**commande)
        self.robot.mettre_a_jour(dt)

class ControleurTerminal(Controleur):
    def lire_commande(self):
        try:
            moteur = self.robot.moteur

            if hasattr(moteur, 'vx'):
                entree = input("Commande Omnidirectionnelle (vx vy omega) (ex: 1.0 2.0 0.5) \n > ")
                vx, vy, omega = map(float, entree.split())
                return {"vx": vx, "vy": vy, "omega": omega}

            else:
                entree = input("Commande Différentielle (v omega) (ex: 1.0 0.5) \n > ")
                v, omega = map(float, entree.split())
                return {"v": v, "omega": omega}

        except ValueError:
            print("Erreur de saisie. Veuillez entrer des nombres séparés par des espaces.")
            return None