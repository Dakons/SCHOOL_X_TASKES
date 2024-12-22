import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from RobotDriveLibrary import TurtleBotController
import math

def main():
    rclpy.init()
    epuckbot = TurtleBotController()

    n=3
    print(f"Рисуем правильный{n}-угольник!")
    for i in range(n):
        rclpy.spin_once(epuckbot)
        angle = 180-(((n-2)/n)*180)
        epuckbot.move_bot_straight_distance(linear_speed=0.2, distance=0.2)
        epuckbot.turn_bot(float(angle),0.5)

    n=4
    print(f"Рисуем правильный{n}-угольник!")
    for i in range(n):
        rclpy.spin_once(epuckbot)
        angle = 180-(((n-2)/n)*180)
        epuckbot.move_bot_straight_distance(linear_speed=0.2, distance=0.2)
        epuckbot.turn_bot(float(angle),0.5)

    n=5
    print(f"Рисуем правильный{n}-угольник!")
    for i in range(n):
        rclpy.spin_once(epuckbot)
        angle = 180-(((n-2)/n)*180)
        epuckbot.move_bot_straight_distance(linear_speed=0.2, distance=0.2)
        epuckbot.turn_bot(float(angle),0.5)


    n=6
    print(f"Рисуем правильный{n}-угольник!")
    for i in range(n):
        rclpy.spin_once(epuckbot)
        angle = 180-(((n-2)/n)*180)
        epuckbot.move_bot_straight_distance(linear_speed=0.2, distance=0.2)
        epuckbot.turn_bot(float(angle),0.5)


    n=7
    print(f"Рисуем правильный{n}-угольник!")
    for i in range(n):
        rclpy.spin_once(epuckbot)
        angle = 180-(((n-2)/n)*180)
        epuckbot.move_bot_straight_distance(linear_speed=0.2, distance=0.2)
        epuckbot.turn_bot(float(angle),0.5)
    
main()
