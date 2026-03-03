import math
from robot.robot_mobile import RobotMobile
from robot.moteur import MoteurDifferentiel, MoteurOmnidirectionnel


# ==========================================================
# OUTIL D'AFFICHAGE
# ==========================================================

def section(titre):
    print("\n" + "="*60)
    print(titre)
    print("="*60)


# ==========================================================
# 2.3 MÉTHODES : AVANCER
# ==========================================================

section("QUESTION : Test de la méthode avancer(distance)")

robot = RobotMobile()
print("Position initiale :")
robot.afficher()

robot.avancer(1.0)
print("Après avoir avancé de 1m :")
robot.afficher()


# ==========================================================
# 2.3 MÉTHODES : TOURNER
# ==========================================================

section("QUESTION : Avancer 1m, tourner de 45°, puis avancer 3m")

robot = RobotMobile()

robot.avancer(1.0)
robot.tourner(math.pi / 4)   # 45°
robot.avancer(3.0)

print("Position finale du robot :")
robot.afficher()


# ==========================================================
# 3 ENCAPSULATION
# ==========================================================

section("QUESTION : Encapsulation - accès aux attributs")

print("Accès via getter robot.x :", robot.x)

print("Modification via setter robot.x = 10")
robot.x = 10
robot.afficher()

print("\nTentative d'accès direct robot.__x (doit échouer) :")
try:
    print(robot.__x)
except AttributeError as e:
    print("Erreur attendue :", e)


# ==========================================================
# 4.1 CLASSE ABSTRAITE MOTEUR
# ==========================================================

section("QUESTION : Utilisation d'un moteur différentiel")

moteur_diff = MoteurDifferentiel()
robot = RobotMobile(moteur=moteur_diff)

dt = 1.0

robot.afficher()

print("Commande : avancer tout droit (v=1 m/s)")
robot.commander(v=1.0, omega=0.0)
robot.mettre_a_jour(dt)

robot.afficher()


# ==========================================================
# 4.2 MOTEUR OMNIDIRECTIONNEL
# ==========================================================

section("QUESTION : Déplacement omnidirectionnel")

moteur_omni = MoteurOmnidirectionnel()
robot = RobotMobile(moteur=moteur_omni)

robot.afficher()

print("Commande : déplacement diagonal vx=1, vy=1")
robot.commander(vx=1.0, vy=1.0, omega=0.0)
robot.mettre_a_jour(1.0)

robot.afficher()


# ==========================================================
# 4.3 POLYMORPHISME
# ==========================================================

section("QUESTION : Polymorphisme - même robot, moteurs différents")

robot_diff = RobotMobile(moteur=MoteurDifferentiel())
robot_omni = RobotMobile(moteur=MoteurOmnidirectionnel())

print("Robot différentiel :")
robot_diff.commander(v=1.0, omega=0.0)
robot_diff.mettre_a_jour(1.0)
robot_diff.afficher()

print("\nRobot omnidirectionnel :")
robot_omni.commander(vx=1.0, vy=1.0, omega=0.0)
robot_omni.mettre_a_jour(1.0)
robot_omni.afficher()


# ==========================================================
# 5 ATTRIBUT STATIQUE
# ==========================================================

section("QUESTION : Nombre total de robots créés")

print("Nombre de robots :", RobotMobile.nombre_robots())


# ==========================================================
# 5 MÉTHODE STATIQUE
# ==========================================================

section("QUESTION : Vérification moteur_valide")

print("MoteurDifferentiel valide ?",
      RobotMobile.moteur_valide(MoteurDifferentiel()))

print("Objet string valide ?",
      RobotMobile.moteur_valide("pas un moteur"))


# ==========================================================
# 6 __str__
# ==========================================================

section("QUESTION : Méthode spéciale __str__")

robot = RobotMobile()
print("print(robot) donne :")
print(robot)
