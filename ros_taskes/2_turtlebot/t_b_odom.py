import time
import rclpy
from cmath import pi
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Для получения данных об одометрии
from sensor_msgs.msg import LaserScan

from math import atan2

class Webots_test_controller(Node):
    def __init__(self):
        super().__init__('simple_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.velosity_func)
        self.subscriber1 = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscriber2 = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.start_x = None
        self.lidar_vals = None
        self.stop_distance = 0.5  

    def velosity_func(self):
        if self.start_x is None:
            return  # Ждем получения первого сообщения об одометрии

        # Проверяем, если робот проехал 1 метр по оси X
        if abs(self.odom_x - self.start_x) >= self.stop_distance:
            msg = Twist()  # Останавливаем робота
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info("я приехал")
        else:
            msg = Twist()
            msg.linear.x = 0.2  # Двигаемся вперед
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x

        if self.start_x is None:
            self.start_x = self.odom_x  # Запоминаем стартовую позицию

        self.get_logger().info(f"текущая позиция X: {self.odom_x}")

    def scan_callback(self, msg):
        self.lidar_vals = msg.ranges
        print(f"lidar: {self.lidar_vals}")

def main():
    rclpy.init()
    node = Webots_test_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()