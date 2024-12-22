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
    turtlePID_dist = PIDRegulator(Kp=0.02,Ki=0.0,Kd=0.0,output_min=-0.5,output_max=0.5,i_buffer_size=10, dead_zone= 5.0)
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
            if lidar_data:
                turtlebot.get_logger().info(f"Данные лидара: Front: {lidar_data[0]} Right: {lidar_data[270]} Left: {lidar_data[90]}")
            if lidar_data is not None:

                #Данные по расстоянию до стены
                sum_data = lidar_data[265:275]
                lidar_sum = sum(sum_data) / len(sum_data)
                lidar_sum*=-100

                #Данные по paralalel до стены
                lidar_sum_dist = (lidar_data[271]+lidar_data[272]+lidar_data[273]+lidar_data[269]+lidar_data[268]+lidar_data[267])/6


                #Данные по расстоянию до объекта
                front_data = lidar_data[170:190]  # Сканируем более широкий сектор
                lidar_sum_front = sum(front_data) / len(front_data)
                lidar_sum_front*=100


                AngleMove_parallel=turtlePID_parallel.regulate(lidar_sum,0)
                AngleMove_dist = turtlePID_dist.regulate(lidar_sum_dist*100.0, 25.0)
                turtlebot.get_logger().info(f"Угловое воздействие по расстоянию:{AngleMove_dist}, sum_dist: {lidar_sum_dist}")
                turtlebot.get_logger().info(f"Угловое воздействие по параллели:{AngleMove_parallel}, sum_parallel: {lidar_sum}")
                turtlebot.get_logger().info(f"sum_front: {lidar_sum_front}")
                if lidar_sum_front>30.0:
                    if AngleMove_dist == 0.0 and AngleMove_parallel != 0.0:
                        turtlebot.move_bot(0.0, AngleMove_parallel)
                    elif AngleMove_dist != 0.0:
                        turtlebot.move_bot(0.1, AngleMove_dist)
                    elif AngleMove_dist == 0.0 and AngleMove_parallel == 0.0:
                        turtlebot.move_bot(0.4, 0.0)
                        
                else:
                    turtlebot.stop_bot()
                    turtlebot.move_bot(0.0,0.3)
                '''
                if AngleMove_dist == 0.0:
                    turtlebot.move_bot(0.1, AngleMove_parallel)
                else:
                    turtlebot.move_bot(0.1, AngleMove_dist+AngleMove_parallel/5)
                '''
            
                
            

            


            
    except KeyboardInterrupt:
        turtlebot.get_logger().info("Программа завершена вручную.")
    finally:
        turtlebot.stop_bot()
        turtlebot.destroy_node()
        rclpy.shutdown()





main()