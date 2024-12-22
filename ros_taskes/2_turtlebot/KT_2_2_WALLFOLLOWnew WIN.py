import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from RobotDriveLibrary import TurtleBotController, PIDRegulator





def main():
    rclpy.init()
    turtlebot = TurtleBotController()
    turtlePID_dist = PIDRegulator(Kp=0.06,Ki=-0.0005,Kd=0.0,output_min=-0.5,output_max=0.5,i_buffer_size=100, dead_zone= 5.0)
    turtlePID_parallel = PIDRegulator(Kp=0.08,Ki=0.0,Kd=0.0,output_min=-1.0,output_max=1.0,i_buffer_size=10, dead_zone= 2.0)

    try:
        turtlebot.get_logger().info("Получение данных с лидара...")

        while rclpy.ok():
            rclpy.spin_once(turtlebot)  # Это должно обновить данные
            # Получение текущей позиции одометрии
            current_position = turtlebot.get_odom_position()
            if current_position:
                current_x, current_y = current_position
                turtlebot.get_logger().info(f"Данные одометрии: Координата Х: {current_x}, Координата Y: {current_y}, Угол: {0}")
            # Получение данных лидара
            lidar_data = turtlebot.get_lidar_data()

            if lidar_data is not None:
                min_dist_wall_right = min(lidar_data[205:335])
                min_dist_front = min(lidar_data[165:195])
                AngleMove_dist = turtlePID_dist.regulate(min_dist_wall_right*100.0, 25.0)
                turtlebot.get_logger().info(f"min_dist_wall_right: {min_dist_wall_right} AngleMove_dist: {AngleMove_dist}, min_dist_front:{min_dist_front}")
                if min_dist_front*100>30:
                    turtlebot.move_bot(0.2, AngleMove_dist)
                else:
                    turtlebot.move_bot(0.0, 0.5)
            
            
            


            
    except KeyboardInterrupt:
        turtlebot.get_logger().info("Программа завершена вручную.")
    finally:
        turtlebot.stop_bot()
        turtlebot.destroy_node()
        rclpy.shutdown()





main()