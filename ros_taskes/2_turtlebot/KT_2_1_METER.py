import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Создаем издателя для управления движением
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Подписываемся на данные одометрии
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        # Переменные для хранения данных одометрии
        self.current_odom = None

    def _odom_callback(self, msg):
        """Обработчик сообщений одометрии."""
        self.current_odom = msg.pose.pose

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

        # Ждем появления данных одометрии
        while rclpy.ok() and turtlebot.get_odom_position() is None:
            rclpy.spin_once(turtlebot)

        # Получаем начальную позицию
        start_position = turtlebot.get_odom_position()
        if start_position is None:
            turtlebot.get_logger().error("Не удалось получить начальную позицию!")
            return

        start_x, start_y = start_position

        # Едем вперед, пока не проедем 1 метр
        target_distance = 1.0  # Задаем дистанцию в метрах
        linear_speed = 0.2  # Скорость движения вперед (м/с)

        while rclpy.ok():
            rclpy.spin_once(turtlebot)

            # Получаем текущую позицию
            current_position = turtlebot.get_odom_position()
            if current_position is None:
                continue
 
            current_x, current_y = current_position
            #print(f"{current_x}")

            # Вычисляем пройденное расстояние
            distance_traveled = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)

            # Если проехали 1 метр, останавливаемся
            if distance_traveled >= target_distance:
                turtlebot.get_logger().info(f"Робот проехал {distance_traveled:.2f} м. Остановка.")
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
