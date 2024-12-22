import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from RobotDriveLibrary import TurtleBotController
import math

def main():
    rclpy.init()
    epuckbot = TurtleBotController()
    rclpy.spin_once(epuckbot)  # Обновляем данные

    epuckbot.move_bot_straight_distance(linear_speed=0.2, distance=1.0)
    epuckbot.turn_bot(90, 0.2)
    while rclpy.ok():
        rclpy.spin_once(epuckbot)
        epuckbot.move_bot(linear_speed=0.2,angular_speed=2.025)
    #while rclpy.ok():
    
    
        

main()
