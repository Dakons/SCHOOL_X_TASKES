import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Создаем издателя для управления движением
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Подписываемся на данные одометрии
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self._lidar_callback, 10)
        # Переменные для хранения данных одометрии
        self.current_odom = None
        self.lidar_data = None
        # Подписываемся на данные лидара

    def _odom_callback(self, msg):
        """Обработчик сообщений одометрии."""
        self.current_odom = msg.pose.pose

    def _lidar_callback(self, msg):
        """Обработчик данных лидара."""
        self.lidar_data = msg

    def get_odom_position(self):
        """
        Возвращает текущую позицию робота из одометрии.
        :return: Позиция (x, y) или None, если данные недоступны.
        """
        if self.current_odom:
            position = self.current_odom.position
            return position.x, position.y
        else:
            return None
        
    def get_lidar_data(self):
        """
        Возвращает массив данных лидара.
        :return: Список расстояний или None, если данные недоступны.
        """
        if self.lidar_data:
            return self.lidar_data.ranges
        else:
            return None

    def move_bot(self, linear_speed, angular_speed=0.0):
        """
        Задает движение робота.
        :param linear_speed: Линейная скорость (м/с).
        :param angular_speed: Угловая скорость (рад/с).
        """
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(msg)

    def stop_bot(self):
        """Останавливает робота."""
        self.move_bot(0.0, 0.0)


def main():
    rclpy.init()
    turtlebot = TurtleBotController()

    try:
        turtlebot.get_logger().info("Робот начинает движение на 1 метр вперед.")

        while rclpy.ok():
            rclpy.spin_once(turtlebot)

            # Получаем текущую позицию
            current_position = turtlebot.get_odom_position()
            if current_position is None:
                continue
 
            current_x, current_y = current_position
            

            # Если проехали 1 метр, останавливаемся
            if distance_traveled >= target_distance:
                turtlebot.get_logger().info(f"Робот проехал {distance_traveled:.2f} м. Остановка.")
                print(f"{current_x}")
                turtlebot.stop_bot()
                break
            else:
                turtlebot.move_bot(linear_speed)

            # Продолжаем движение
            #turtlebot.move_bot(linear_speed)

    except KeyboardInterrupt:
        turtlebot.get_logger().info("Программа завершена вручную.")
    finally:
        turtlebot.stop_bot()
        turtlebot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
