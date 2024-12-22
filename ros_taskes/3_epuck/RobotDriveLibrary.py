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
        
        # Подписываемся на данные лидара
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self._lidar_callback, 10)

        # Переменные для хранения данных
        self.current_odom = None
        self.lidar_data = None

    def _odom_callback(self, msg):
        """Обработчик сообщений одометрии."""
        self.current_odom = msg.pose.pose

    def get_odom_position_and_angle(self):
        """
        Возвращает текущую позицию и угол поворота робота из одометрии.
        :return: (x, y, yaw) или None, если данные недоступны.
        """
        if self.current_odom:
            position = self.current_odom.position
            orientation = self.current_odom.orientation

            # Преобразование кватерниона в угол (yaw)
            siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            yaw_degrees = math.degrees(yaw)
            return position.x, position.y, yaw_degrees
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

    def move_bot_straight_distance(self, linear_speed, distance):
        """
        Задает движение робота на заданное расстояние вперёд.
        :param linear_speed: Линейная скорость (м/с).
        :param distance: расстояние, которое нужно проехать (м).
        """

        # Начальные координаты
        self.initial_position_x = None
        self.initial_position_y = None
        self.initial_position_angle = None
        self.current_position = None
        self.current_position_x = None
        self.current_position_y = None
        self.current_position_yaw = None
        self.distance_travelled = None
        # Инициализация начальной позиции, если еще не установлена
        if self.initial_position_x is None or self.initial_position_y is None:
            while self.initial_position_x is None or self.initial_position_y is None:
                self.current_position = self.get_odom_position_and_angle()
                if self.current_position:
                    self.initial_position_x, self.initial_position_y, self.initial_position_angle = self.current_position
                    self.get_logger().info(f"Начальная позиция: X:{self.initial_position_x}, Y:{self.initial_position_y}, Angle:{self.initial_position_angle}")
                else:
                    self.get_logger().info("Не удалось получить начальную позицию.")
                rclpy.spin_once(self)  # Подождать, пока не получим позицию

        # Движение на заданное расстояние
        while rclpy.ok():
            rclpy.spin_once(self)
            
            self.current_position = self.get_odom_position_and_angle()
            if self.current_position:
                self.current_position_x, self.current_position_y, self.current_position_yaw = self.current_position
                self.distance_travelled = math.sqrt((self.current_position_x - self.initial_position_x) ** 2 + (self.current_position_y - self.initial_position_y) ** 2)

                #self.get_logger().info(f"X:{self.current_position_x}, Y:{self.current_position_y}, Угол:{self.current_position_yaw}")
                #self.get_logger().info(f"Пройдено расстояние: {distance_travelled:.2f} м")

                if self.distance_travelled >= distance:
                    self.stop_bot()
                    self.get_logger().info("Цель достигнута. Остановка робота.")
                    break
                else:
                    self.move_bot(linear_speed, 0.0)

    def turn_bot(self, angle_rotate, angular_speed=0.1):
        """
        Поворачивает робота на заданный угол.
        :param angle_rotate: Угол, на который нужно повернуть робота в градусах.
        :param angular_speed: Угловая скорость поворота робота.
        """
        angular_speed = abs(angular_speed)
        if angle_rotate < 0:
           angular_speed = -angular_speed
        # Начальные координаты
        self.initial_position_angle = None
        self.current_position = None
        self.current_position_x = None
        self.current_position_y = None
        self.current_position_yaw = None
        # Инициализация начальной позиции, если еще не установлена
        if self.initial_position_angle is None:
            while self.initial_position_angle is None:
                self.current_position = self.get_odom_position_and_angle()
                if self.current_position:
                    self.initial_position_x, self.initial_position_y, self.initial_position_angle = self.current_position
                    self.get_logger().info(f"Начальный Angle:{self.initial_position_angle}")
                else:
                    self.get_logger().info("Не удалось получить начальную позицию.")
                rclpy.spin_once(self)  # Подождать, пока не получим позицию
            self.target_angle = self.initial_position_angle + angle_rotate
            if self.target_angle > 180:
                self.target_angle -=360
            elif self.target_angle < -180:
                self.target_angle += 360
            self.get_logger().info(f"Конечный Angle:{self.target_angle}")
            
            # Поворачиваем робота до достижения нужного угла
            while rclpy.ok():
                rclpy.spin_once(self)
                current_data = self.get_odom_position_and_angle()
                if current_data:
                    _, _, self.current_position_angle  = current_data
                    # Проверяем, достигли ли мы цели
                    rotated_angle = self.current_position_angle - self.initial_position_angle
                    if rotated_angle > 180:
                        rotated_angle = rotated_angle-360
                    elif rotated_angle < -180:
                        rotated_angle = rotated_angle+360
                    if abs(rotated_angle) >= abs(angle_rotate):  # Точность поворота
                        rclpy.spin_once(self)
                        self.stop_bot()
                        self.get_logger().info(f"Робот повернулся на угол: {rotated_angle}")
                        break
                    else:
                        #self.get_logger().info(f"Текущий Angle:{self.current_position_angle}")
                        self.move_bot(0.0, angular_speed)


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

def main():
    rclpy.init()
    turtlebot = TurtleBotController()

    try:
        turtlebot.get_logger().info("Получение данных...")

        while rclpy.ok():
            rclpy.spin_once(turtlebot)  # Обновляем данные

            # Задаем движение робота
            turtlebot.move_bot(0.1, 0.01)

            # Получение текущей позиции и угла поворота
            current_data = turtlebot.get_odom_position_and_angle()
            if current_data:
                current_x, current_y, current_yaw = current_data
                turtlebot.get_logger().info(f"Одометрия: X: {current_x}, Y: {current_y}, Угол (рад): {current_yaw}")

            # Получение данных лидара
            lidar_data = turtlebot.get_lidar_data()
            if lidar_data:
                turtlebot.get_logger().info(f"Лидар: {lidar_data[0]}")

    except KeyboardInterrupt:
        turtlebot.get_logger().info("Программа завершена вручную.")
    finally:
        turtlebot.stop_bot()
        turtlebot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
