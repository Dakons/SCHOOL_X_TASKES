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

    def _lidar_callback(self, msg):
        """Обработчик данных лидара."""
        self.lidar_data = msg

    def get_lidar_data(self):
        """
        Возвращает массив данных лидара.
        :return: Список расстояний или None, если данные недоступны.
        """
        if self.lidar_data:
            return self.lidar_data.ranges
        else:
            return None
class PIDRegulator:
    def __init__(self, Kp: float, Ki: float, Kd: float, output_min: float = -100.0, output_max: float = 100.0, i_buffer_size: int = 10, dead_zone: float = 10.0):
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.regulate_error = 0.0
        self.integral_buffer = [0.0] * i_buffer_size  # Буфер для интегральной ошибки
        self.i_buffer_size = i_buffer_size
        self.last_error = 0.0
        self.output_min = output_min  # Минимальное значение выходного сигнала
        self.output_max = output_max  # Максимальное значение выходного сигнала
        self.dead_zone = dead_zone

    def regulate(self, data: float, state: float) -> float:
        # Вычисляем текущую ошибку
        self.regulate_error = state - data
        
        # Фильтр на небольшие ошибки (мёртвая зона)
        if -self.dead_zone < self.regulate_error < self.dead_zone:
            self.regulate_error = 0
        
        # Пропорциональная составляющая
        self.P = self.Kp * self.regulate_error
    
        # Обновляем буфер интегральной составляющей (сдвигаем значения и добавляем новое)
        self.integral_buffer.pop(0)  # Удаляем самое старое значение
        self.integral_buffer.append(self.regulate_error)  # Добавляем новое значение ошибки
        
        # Интегральная составляющая - сумма значений буфера
        self.I = self.Ki * sum(self.integral_buffer)

        # Дифференциальная составляющая (разница ошибок для плавности)
        self.D = self.Kd * (self.regulate_error - self.last_error)
        self.last_error = self.regulate_error  # Обновляем последнюю ошибку
        
        # PID-выход
        PID_output = self.P + self.I + self.D

        # Линейная интерполяция для ограничения выходного сигнала
        PID_output = self.linear_interpolate(PID_output)
        
        return round(PID_output, 3)

    def linear_interpolate(self, value: float) -> float:
        """Ограничивает значение в заданных пределах."""
        if value < self.output_min:
            return self.output_min
        elif value > self.output_max:
            return self.output_max
        else:
            return value
"""
def main():
    rclpy.init()
    turtlebot = TurtleBotController()

    try:
        turtlebot.get_logger().info("Получение данных с лидара...")

        while rclpy.ok():
            rclpy.spin_once(turtlebot)  # Это должно обновить данные
            turtlebot.move_bot(0.1,0.01)
            # Получение текущей позиции одометрии
            current_position = turtlebot.get_odom_position()
            if current_position:
                current_x, current_y = current_position
                turtlebot.get_logger().info(f"Данные одометрии: Координата Х: {current_x}, Координата Y: {current_y}, Угол: {0}")
            # Получение данных лидара
            lidar_data = turtlebot.get_lidar_data()
            if lidar_data:
                turtlebot.get_logger().info(f"Данные лидара: {lidar_data[0]}")
            
    except KeyboardInterrupt:
        turtlebot.get_logger().info("Программа завершена вручную.")
    finally:
        turtlebot.stop_bot()
        turtlebot.destroy_node()
        rclpy.shutdown()




if __name__ == '__main__':
    main()


"""