import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from RobotDriveLibrary import TurtleBotController
import math

full_distance = 0.0

def MoveToBox(node):
    global full_distance
    flag_start = 0
    start_x = 0.0
    start_y= 0.0
    end_x= 0.0
    end_y= 0.0 
    #Получение текущей позиции
    start_data = node.get_odom_position_and_angle()
    if start_data:
        start_x, start_y, _ = start_data
        node.get_logger().info(f"Стартовая позиция X: {start_x}, Y: {start_y}")
    while flag_start == 0:
        rclpy.spin_once(node)
        lidar_data = node.get_lidar_data()
        if lidar_data:
            #node.get_logger().info(f"Лидар: {lidar_data[10]}")
            if lidar_data[10]<=0.2:
                node.stop_bot()
                flag_start = 1
                print("Подъехали к стене")
            else:
                node.move_bot(0.1, 0.0)
    #Получение текущей позиции
    end_data = node.get_odom_position_and_angle()
    if end_data:
        end_x, end_y, _ = end_data
        node.get_logger().info(f"Конечная позиция X: {end_x}, Y: {end_y}")
    dist = math.sqrt((end_x - start_x) ** 2 + (end_y - start_x) ** 2)*0.01
    print(f"робот проехал до бокса: {dist} м.")
    full_distance+=dist

        
def RideAroundBox(node, direction):
    global full_distance
    if direction == "left":
        node.turn_bot(90,0.4)
        node.move_bot_straight_distance(0.2,0.25)
        full_distance += node.distance_travelled
        node.turn_bot(-90,0.4)
        node.move_bot_straight_distance(0.2,0.6)
        full_distance += node.distance_travelled
        node.turn_bot(-90,0.4)
        node.move_bot_straight_distance(0.2,0.25)
        full_distance += node.distance_travelled 
        node.turn_bot(90,0.4)
    if direction == "right":
        node.turn_bot(-90,0.4)
        node.move_bot_straight_distance(0.2,0.25)
        full_distance += node.distance_travelled
        node.turn_bot(90,0.4)
        node.move_bot_straight_distance(0.2,0.6)
        full_distance += node.distance_travelled
        node.turn_bot(90,0.4)
        node.move_bot_straight_distance(0.2,0.25)
        full_distance += node.distance_travelled
        node.turn_bot(-90,0.4)



def main():
    rclpy.init()
    epuckbot = TurtleBotController()
    # Получение данных лидара
    for i in range(2):
        MoveToBox(epuckbot)
        RideAroundBox(epuckbot, "left")
        MoveToBox(epuckbot)
        RideAroundBox(epuckbot, "right")
    epuckbot.move_bot_straight_distance(0.2,0.25)

    print(f'distance_travelled: {full_distance} m')           
main()
