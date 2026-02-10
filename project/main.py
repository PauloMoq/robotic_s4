from robot.robot_mobile import RobotMobile
import math

if __name__ == '__main__':
    bob = RobotMobile(6, 7, math.pi/2)

    #print("Bob: ", bob)
    #bob.avancer(3)
    #print("Bob moved to: ", bob)
    #bob.tourner(30)
    #print("Bob rotated to: ", bob)

    #pour rep à la question 2.3
    print("\n(2.3) answer:")
    karim=RobotMobile(0, 0, 0)
    print("Karim: ", karim)

    karim.avancer(1)
    print("Karim moved to: ", karim)

    karim.tourner(45)
    print("Karim rotated to: ", karim)
