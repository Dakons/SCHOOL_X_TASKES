class PIDRegulator:
    def __init__(self, Kp: float, Ki: float, Kd: float, output_min: float = -100.0, output_max: float = 100.0, i_buffer_size: int = 10):
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

    def regulate(self, data: float, state: float) -> float:
        # Вычисляем текущую ошибку
        self.regulate_error = state - data
        
        # Фильтр на небольшие ошибки (мёртвая зона)
        if -5 < self.regulate_error < 5:
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
        
        return round(PID_output, 1)

    def linear_interpolate(self, value: float) -> float:
        """Ограничивает значение в заданных пределах."""
        if value < self.output_min:
            return self.output_min
        elif value > self.output_max:
            return self.output_max
        else:
            return value