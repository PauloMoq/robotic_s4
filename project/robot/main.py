from robot_mobile import RobotMobile
from moteur import MoteurDifferentiel, MoteurOmnidirectionnel
from ControleurTerminal import ControleurTerminal


def main():
    moteur = MoteurDifferentiel()
    robot = RobotMobile(x=0, y=0, orientation=0, moteur=moteur)

    controleur = ControleurTerminal(robot)

    print("--- Simulation du Robot (MVC) ---")
    print(robot)

    for _ in range(3):
        controleur.mettre_a_jour_robot(dt=1.0)
        print(f"Nouvel état : {robot}")

if __name__ == "__main__":
    main()