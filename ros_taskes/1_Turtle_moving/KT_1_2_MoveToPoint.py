import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from time import sleep
import math
import numpy as np
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import random


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



    def draw_goal(self, x, y):
        """
        Рисует крестик на карте в заданной точке для обозначения цели.
        :param x: Координата X цели.
        :param y: Координата Y цели.
        """
        
        self.set_pen_color(0, 0, 0, 1, 1)  # Выключаем рисование
        self.teleport(x - 0.25, y, 0.0)      # Горизонтальная линия
        self.set_pen_color(255, 0, 0, 3, 0)  # Красный цвет пера
        self.teleport(x + 0.25, y, 0.0)
        self.set_pen_color(0, 0, 0, 1, 1)  # Выключаем рисование
        self.teleport(x, y - 0.25, 0.0)      # Вертикальная линия
        self.set_pen_color(255, 0, 0, 3, 0)  # Красный цвет пера
        self.teleport(x, y + 0.25, 0.0)
        self.set_pen_color(0, 0, 0, 1, 1)  # Выключаем рисование
        #self.teleport(4.25,3.5,0.0)

    def draw_blocks(self, blocks):
        """
        Рисует прямоугольники на основе координат, переданных в блоках.
        :param blocks: Список прямоугольников. Каждый прямоугольник представлен как кортеж из 4 точек (x1, y1, x2, y2).
        """
        for block in blocks:
            # Рисуем один прямоугольник
            self.set_pen_color(0, 0, 1, 1, 1)  # Выключаем рисование
            x1, y1, x2, y2 = block

            # Начинаем рисовать
            self.teleport(x1, y1, 0.0)
            self.set_pen_color(0, 0, 1, 4, 0)  # Устанавливаем цвет для рисования
            self.teleport(x2, y1, 0.0)  # Верхняя горизонтальная линия
            self.teleport(x2, y2, 0.0)  # Правая вертикальная линия
            self.teleport(x1, y2, 0.0)  # Нижняя горизонтальная линия
            self.teleport(x1, y1, 0.0)  # Левая вертикальная линия

    def draw_grid(self, grid_size, step=0.1):
        """
        Рисует сетку на поле.
        :param grid_size: Размер поля.
        :param step: Шаг между линиями сетки.
        """
        self.set_pen_color(0, 0, 0, 1, 1)  # Выключаем рисование
        self.step = step
        
        for x in np.arange(0.0, grid_size+1, step):
            # Рисуем вертикальные линии
            self.teleport(x, 0.0, 0.0)
            self.set_pen_color(0, 0, 0, 1, 0)
            self.teleport(x, grid_size, 0.0)

            self.set_pen_color(0, 0, 1, 1, 1)  # Выключаем рисование
            # Рисуем горизонтальные линии
            self.teleport(0.0, x, 0.0)
            self.set_pen_color(0, 0, 0, 1, 0)
            self.teleport(grid_size, x, 0.0)
            self.set_pen_color(0, 0, 1, 1, 1)  # Выключаем рисование
        
        print(f"Размеры:{11.0/step}:{11.0/step}")

    def grid_teleport(self, x: int, y: int):

        self.tp_x = x * self.step + self.step/2
        self.tp_y = y * self.step + self.step/2
        self.teleport(self.tp_x, self.tp_y, 0.0)



    def a_star_path(self, start, goal, blocks):
        # Создаем карту размером grid_size x grid_size, заполненную единицами (проходимые клетки)
        self.mnoj = 1.0 / self.step
        grid_size = int(11 * self.mnoj)  # Размер карты, округленный до целых чисел
        grid = np.ones((grid_size, grid_size))  # Все клетки проходимые по умолчанию
        
        # Добавляем препятствия на карту
        for block in blocks:
            x1, y1, x2, y2 = block
            # Масштабируем координаты
            x1, y1 = int(x1 * self.mnoj), int(y1 * self.mnoj)
            x2, y2 = int(x2 * self.mnoj), int(y2 * self.mnoj)

            # Обеспечим, что x1 < x2 и y1 < y2
            if x1 > x2:
                x1, x2 = x2, x1
            if y1 > y2:
                y1, y2 = y2, y1

            # Убедимся, что индексы не выходят за пределы массива
            x1 = max(0, min(x1, grid.shape[0] - 1))
            y1 = max(0, min(y1, grid.shape[1] - 1))
            x2 = max(0, min(x2, grid.shape[0] - 1))
            y2 = max(0, min(y2, grid.shape[1] - 1))

            # Печатаем информацию для отладки (опционально)
            print(f"Обновляем область с ({x1}, {y1}) по ({x2}, {y2})")

            # Заполнение прямоугольника на сетке нулями
            for i in range(x1, x2):
                for j in range(y1, y2):
                    grid[j, i] = 0  # Обновляем клетку на 0


        print(grid)
        # Создаем объект Grid для работы с A*
        grid_obj = Grid(matrix=grid)

        # Указываем стартовую и целевую точки (нужно передавать целые числа)
        start_node = grid_obj.node(int(start[0] * self.mnoj), int(start[1] * self.mnoj))  # Точка старта (x, y)
        end_node = grid_obj.node(goal[0], goal[1])
          # Точка цели (x, y), ограничиваем диапазон

        # Инициализируем AStarFinder
        finder = AStarFinder()

        # Находим путь от стартовой точки к целевой
        path, _ = finder.find_path(start_node, end_node, grid_obj)

        return path



"""
# Пример использования функции
start = (0, 0)  # Стартовая точка
goal = (9, 9)   # Целевая точка

# Получаем путь
path = a_star_path(start, goal)

# Выводим найденный путь
print(f"Found path: {path}")
"""


def main(args=None):
    rclpy.init()

    node = rclpy.create_node('move_turtle_node')
    turtle_controller = TurtleController(node)

    # Ждем, чтобы данные о положении обновились
    rclpy.spin_once(node, timeout_sec=0.1)

    #prepyatstviya
    blocks = [
        (2.0, 7.5, 5.0, 7.3),  # Прямоугольник 1
        (2.0, 7.3, 2.2, 5.0),   # Прямоугольник 2
        (4.0, 4.0, 7.0, 3.8),    # Прямоугольник 3
        (8.0, 9.5, 8.2, 2.0),    # Прямоугольник 4
    ]

    #Рисуем препятствия
    turtle_controller.draw_blocks(blocks)
    #Рисуем сеточку для черепашечки
    turtle_controller.draw_grid(grid_size = 11.0, step = 0.1)

    for i in range (3):
        # Считываем начальные и конечные координаты из терминала
        if i == 0:
            start_x, start_y = map(float, input("Введите начальные координаты в пределах 0,10 (x y): ").split())
        else: 
            start_x, start_y = goal_x ,goal_y
        goal_x, goal_y = map(float, input("Введите конечные координаты в пределах 0,10 (x y): ").split())

        #отображаем
        turtle_controller.draw_goal(float(goal_x), float(goal_y))
        turtle_controller.grid_teleport(start_x*10, start_y*10)
        sleep(1)
        
        road = turtle_controller.a_star_path((int(start_x), int(start_y)),(int(goal_x*10), int(goal_y*10)),blocks)
        turtle_controller.set_pen_color(random.randint(0,255),random.randint(0,255),random.randint(0,255),1,0)
        for node in road:
            turtle_controller.set_pen_color(random.randint(0,255),random.randint(0,255),random.randint(0,255),1,0)
            x, y = node.x, node.y
            turtle_controller.grid_teleport(x, y)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
