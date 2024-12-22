import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WebotsTestController(Node):
    def __init__(self):
        super().__init__('simple_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.velosity_func)
        self.subscriber1 = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscriber2 = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.start_x = None
        self.lidar_vals = None  # Для хранения данных лидара
        self.kP = 0.01  # Пропорциональный коэффициент
        self.stop_distance = 1.0  # Расстояние до препятствия для остановки

    def velosity_func(self):
        if self.lidar_vals is not None:  # Проверяем, что данные лидара уже получены
            self.left = self.lidar_vals[89]  # Луч left
            self.right = self.lidar_vals[269]  # Луч right
            self.forward = self.lidar_vals[0]  # Луч forw

            for i in range (-15, 15):
                rightsum +=  self.lidar_vals[269+i]
            rightsum 


            print(f"Получены данные лидара справа: {self.right}")
            print(f"Получены данные лидара слева: {self.left}")
            print(f"Получены данные лидара спереди: {self.forward}")
            print("---")

            # Расчёт ошибки (следование на расстоянии 0.3 метра от стены справа)
            desired_distance = 30.0
            self.error = desired_distance - self.right * 100.0

            if abs(self.error) < 5.0:
                self.error = 0.0
            print (f"Error is: {self.error}")
            # Формируем сообщение для управления
            msg = Twist()
            #msg.linear.x = 0.2  # Постоянная скорость вперёд
            self.PID = self.kP * self.error
            if self.PID > 100.0:
                self.PID = 2.0
            elif self.PID < -100.0:
                self.PID = -2.0
            print (f"PID is: {self.PID}")

            msg.angular.z = self.PID  # Угловая скорость для регулировки
            msg.linear.x = 0.2
            # Публикация сообщения
            self.publisher_.publish(msg)

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x

        if self.start_x is None:
            self.start_x = self.odom_x  # Запоминаем стартовую позицию

    def scan_callback(self, msg):
        # Преобразуем данные лидара в список для доступа по индексу
        self.lidar_vals = list(msg.ranges)
        self.get_logger().info("Получены данные лидара")


def main():
    rclpy.init()
    node = WebotsTestController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
