from robot.robot_mobile import RobotMobile
import math

if __name__ == '__main__':
    
    # ---------------
    #pour rep à la question 2.3
    print("\n(2.3) answer:")
    karim=RobotMobile(0, 0, 0)
    print("Karim: ", karim)

    karim.avancer(1)
    print("Karim moved to: ", karim)

    karim.tourner(45)
    print("Karim rotated to: ", karim)

    karim.avancer(3)
    print("Karim moved to: ", karim)

    # ---------------
    # pour rep à la question 3
    print("\n(3) answer:")
    print("L'attribut public on peut le modifier/regarder via un objet (ex : print(karim.x).")
    print("Alors que le private on peut toujours, mais via des getters/setters (ex : print(karim.get_x()).")
