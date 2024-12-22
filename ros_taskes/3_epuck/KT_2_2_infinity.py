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
            if lidar_data[10]<=0.4 and lidar_data[10]>=0.8:
                rclpy.spin_once(node)
                node.stop_bot()
                flag_start = 1
                print("Подъехали к стене")
                break
            """
            elif lidar_data[10]>0.8:
                node.move_bot(0.1, 0.0)
            elif lidar_data[10]<0.4:
                node.move_bot(-0.1, 0.0)
            """
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

def LengthOfSectorRound(start_x,start_y,end_x,end_y,sector):
    diameter = math.sqrt((end_x - start_x) ** 2 + (end_y - start_y) ** 2)
    #print(f'diameter now is = {diameter}')
    S = math.pi * diameter * sector
    return S

def main():
    current_angle = 1000.0
    diff_angle = 1000.0
    end_angle = 0
    start_angle = 0

    rclpy.init()
    epuckbot = TurtleBotController()
    #rclpy.spin_once(epuckbot)
    #MoveToBox(epuckbot)
    
    rclpy.spin_once(epuckbot)
    epuckbot.turn_bot(90,0.4)
    #Получение текущей позиции
    
    start_data = epuckbot.get_odom_position_and_angle()
    if start_data:
        current_X1, current_Y1, start_angle = start_data
        epuckbot.get_logger().info(f"Стартовая позиция:{start_angle}")

    end_angle = start_angle - 180
    if end_angle > 180:
        end_angle -=360
    elif end_angle < -180:
        end_angle += 360

    
    while diff_angle>1.0:
        rclpy.spin_once(epuckbot)
        current_data = epuckbot.get_odom_position_and_angle()
        if current_data:
            current_X2, current_Y2, current_angle = current_data
        epuckbot.move_bot(linear_speed=0.2,angular_speed=-2.05)
        diff_angle = abs(current_angle - end_angle)
    epuckbot.stop_bot()



    rclpy.spin_once(epuckbot)
    start_data = epuckbot.get_odom_position_and_angle()
    if start_data:
        current_X3, current_Y3, start_angle = start_data
        epuckbot.get_logger().info(f"Стартовая позиция:{start_angle}")


    end_angle = start_angle - 180
    if end_angle > 180:
        end_angle -=360
    elif end_angle < -180:
        end_angle += 360
    diff_angle = 1000.0
    
    while diff_angle>1.0:
        rclpy.spin_once(epuckbot)
        current_data = epuckbot.get_odom_position_and_angle()
        if current_data:
            current_X4, current_Y4, current_angle = current_data
        epuckbot.move_bot(linear_speed=0.2,angular_speed=2.05)
        diff_angle = abs(current_angle - end_angle)
    epuckbot.stop_bot()


    rclpy.spin_once(epuckbot)
    start_data = epuckbot.get_odom_position_and_angle()
    if start_data:
        current_X5, current_Y5, start_angle = start_data
        epuckbot.get_logger().info(f"Стартовая позиция:{start_angle}")


    end_angle = start_angle - 180
    if end_angle > 180:
        end_angle -=360
    elif end_angle < -180:
        end_angle += 360
    diff_angle = 1000.0
    
    while diff_angle>1.0:
        rclpy.spin_once(epuckbot)
        current_data = epuckbot.get_odom_position_and_angle()
        if current_data:
            current_X6, current_Y6, current_angle = current_data
        epuckbot.move_bot(linear_speed=0.2,angular_speed=2.05)
        diff_angle = abs(current_angle - end_angle)
    epuckbot.stop_bot()

    rclpy.spin_once(epuckbot)
    start_data = epuckbot.get_odom_position_and_angle()
    if start_data:
        current_X7, current_Y7, start_angle = start_data
        epuckbot.get_logger().info(f"Стартовая позиция:{start_angle}")

    end_angle = start_angle - 180
    if end_angle > 180:
        end_angle -=360
    elif end_angle < -180:
        end_angle += 360

    diff_angle = 1000.0
    while diff_angle>1.0:
        rclpy.spin_once(epuckbot)
        current_data = epuckbot.get_odom_position_and_angle()
        if current_data:
            current_X8, current_Y8, current_angle = current_data
        epuckbot.move_bot(linear_speed=0.2,angular_speed=-2.05)
        diff_angle = abs(current_angle - end_angle)
    epuckbot.stop_bot()


    rclpy.spin_once(epuckbot)
    epuckbot.turn_bot(-90,0.4)

    SumDistantion = 0.0
    SumDistantion1 = LengthOfSectorRound(start_x=current_X1, start_y=current_Y1, end_x=current_X2, end_y=current_Y2, sector=0.5)
    print(f'1 SumDistantion:{SumDistantion1} m.')

    SumDistantion2 = LengthOfSectorRound(start_x=current_X3, start_y=current_Y3, end_x=current_X4, end_y=current_Y4, sector=0.5)
    print(f'2 SumDistantion:{SumDistantion2} m.')

    SumDistantion3 = LengthOfSectorRound(start_x=current_X5, start_y=current_Y5, end_x=current_X6, end_y=current_Y6, sector=0.5)
    print(f'3 SumDistantion:{SumDistantion3} m.')

    SumDistantion4 = LengthOfSectorRound(start_x=current_X7, start_y=current_Y7, end_x=current_X8, end_y=current_Y8, sector=0.5)
    print(f'4 SumDistantion:{SumDistantion4} m.')
    
    SumDistantion = SumDistantion1 + SumDistantion2 + SumDistantion3 + SumDistantion4
    print(f'all SumDistantion:{SumDistantion} m.')
    
            
main()
