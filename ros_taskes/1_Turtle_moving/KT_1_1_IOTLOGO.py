import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from time import sleep
import math


class TurtleController:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.current_pose = None
        self.subscription = node.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription  # Сохраняем подписку для предотвращения сборки мусора
        self.teleport_client = node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        # Ожидаем доступности сервиса телепортации
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for teleport service...')
        #while not self.pen_client.wait_for_service(timeout_sec=1.0):
            #self.node.get_logger().info('Waiting for pen service...')

    def pose_callback(self, msg):
        """Callback для получения текущего положения черепахи."""
        self.current_pose = msg

    def move_distance(self, node, distance):
        rclpy.spin_once(node, timeout_sec=0.5)  # Обрабатываем входящие данные
        """
        Движение черепахи на заданное расстояние с указанной скоростью.
        :param distance: Расстояние, которое нужно пройти (в условных единицах turtlesim).
        """
        vel_msg = Twist()
        vel_msg.linear.x = abs(distance)  # Устанавливаем скорость
        vel_msg.angular.z = 0.0  # Без поворота

        print(f"Moving straight for {distance}")
        self.publisher.publish(vel_msg)  # Начинаем движение
        sleep(1.1)
        vel_msg.linear.x = 0.0
        self.publisher.publish(vel_msg)  # Останавливаем движение
        print("Moving stopped.")

    def move_angle(self, node, angle):
        rclpy.spin_once(node, timeout_sec=0.5)  # Обрабатываем входящие данные
        """
        Поворот черепахи на заданный угол с указанной угловой скоростью.
        :param angle: Угол поворота (в градусах, положительное значение — против часовой стрелки).
        """
        vel_msg = Twist()
        vel_msg.linear.x = 0.0  # Без линейного движения

        if angle > 180:
            angle = angle-360
        elif angle < -180:
            angle = angle+360

        vel_msg.angular.z = math.radians(abs(angle)) * (1 if angle > 0 else -1)  # Устанавливаем направление вращения

        print(f"Rotating for {angle}")
        self.publisher.publish(vel_msg)  # Начинаем поворот
        sleep(1.1)
        vel_msg.angular.z = 0.0
        self.publisher.publish(vel_msg)  # Останавливаем вращение
        print("Rotation stopped.")

    def get_current_angle(self, node):
        rclpy.spin_once(node, timeout_sec=0.5)  # Обрабатываем входящие данные
        """
        Возвращает текущий угол черепахи в градусах.
        """
        if self.current_pose is None:
            print("Pose data not yet available.")
            return None
        angle = math.degrees(self.current_pose.theta)
        if angle <= 180 and angle >= 0:
            return angle
        elif angle < 0:
            return 360 + angle

    def teleport(self, x, y, theta):
        """
        Телепортирует черепаху в заданную позицию.
        :param x: Координата X.
        :param y: Координата Y.
        :param theta: Угол ориентации (в радианах).
        """
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = self.teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info(f"Teleported to x={x}, y={y}, theta={theta}")
        else:
            self.node.get_logger().error("Failed to teleport!")

    def set_pen_color(self, r, g, b, width=1, off=0):
        """
        Устанавливает параметры пера черепашки.
        :param r: Красный компонент цвета (0-255).
        :param g: Зелёный компонент цвета (0-255).
        :param b: Синий компонент цвета (0-255).
        :param width: Толщина линии.
        :param off: Отключение рисования (0 — включено, 1 — отключено).
        """
        set_pen_client = self.node.create_client(SetPen, '/turtle1/set_pen')
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for /set_pen service...')

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info(f"Pen color set to RGB({r}, {g}, {b})")
        else:
            self.node.get_logger().error("Failed to set pen color!")
def main():
    rclpy.init()

    node = rclpy.create_node('move_turtle_node')
    turtle_controller = TurtleController(node)

    # Ждем, чтобы данные о положении обновились
    rclpy.spin_once(node, timeout_sec=0.1)


    turtle_controller.set_pen_color(0,0,1,1,1)
    turtle_controller.teleport(4.25,3.5,0.0)
    turtle_controller.set_pen_color(0,0,1,4,0)
    current_angle = turtle_controller.get_current_angle(node)
    print(f"Current angle: {current_angle:.5f} degrees")

    for i in range(3):  # Рисуем треугольник
        wanted_angle = (i+1) * (180-60)
        turtle_controller.move_distance(node, 3.0)

        for __ in range(5):
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.5f} degrees")
            turtle_controller.move_angle(node, wanted_angle-current_angle)
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.2f} degrees")


    for i in range(4):  # Рисуем kvadrat
        wanted_angle = (i+1) * (180-90)
        turtle_controller.move_distance(node, 3.0)
        for __ in range(5):
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.5f} degrees")
            turtle_controller.move_angle(node, wanted_angle-current_angle)
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.2f} degrees")

    for i in range(5):  # Рисуем 5
        wanted_angle = (i+1) * (180-108)
        turtle_controller.move_distance(node, 3.0)
        for __ in range(5):
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.5f} degrees")
            turtle_controller.move_angle(node, wanted_angle-current_angle)
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.2f} degrees")

    for i in range(6):  # Рисуем 6
        wanted_angle = (i+1) * (180-120)
        turtle_controller.move_distance(node, 3.0)
        for __ in range(5):
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.5f} degrees")
            turtle_controller.move_angle(node, wanted_angle-current_angle)
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.2f} degrees")

    for i in range(7):  # Рисуем 7
        wanted_angle = (i+1) * (180-128.571)
        turtle_controller.move_distance(node, 3.0)
        for __ in range(5):
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.5f} degrees")
            turtle_controller.move_angle(node, wanted_angle-current_angle)
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.2f} degrees")
    """
    for i in range(8):  # Рисуем kvadrat
        wanted_angle = (i+1) * (180-135)
        turtle_controller.move_distance(node, 3.0)
        for __ in range(4):
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.5f} degrees")
            turtle_controller.move_angle(node, wanted_angle-current_angle)
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.2f} degrees")

    for i in range(9):  # Рисуем kvadrat
        wanted_angle = (i+1) * (180-140)
        turtle_controller.move_distance(node, 3.0)
        for __ in range(4):
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.5f} degrees")
            turtle_controller.move_angle(node, wanted_angle-current_angle)
            current_angle = turtle_controller.get_current_angle(node)
            print(f"Current angle: {current_angle:.2f} degrees")
    """



    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
