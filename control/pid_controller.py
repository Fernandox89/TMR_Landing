import time

class PIDController:
    def __init__(self, kp, ki, kd, min_out, max_out, integral_limit=None, clock=time.time):
        self.kp = kp  # Constante Proporcional (La fuerza bruta del movimiento)
        self.ki = ki  # Constante Integral (Corrige el error acumulado con el tiempo)
        self.kd = kd  # Constante Derivativa (Frena el dron para evitar que se pase de largo)

        self.min_out = min_out  # Velocidad mínima (ej. -100 en Tello)
        self.max_out = max_out  # Velocidad máxima (ej. 100 en Tello)

        # Límite absoluto del término integral (anti-windup). None = sin límite.
        self.integral_limit = integral_limit
        self.clock = clock
        self.reset()

    def reset(self):
        """Olvida la historia (integral y derivada). Llamar al cambiar de estado o al recuperar el objetivo."""
        self.integral = 0.0
        self.prev_value = None
        self.prev_time = self.clock()

    def update(self, setpoint, current_value, dt=None, feedforward=0.0):
        """
        Calcula la velocidad a la que debe moverse el dron.
        setpoint: Dónde queremos que esté (el centro de la imagen).
        current_value: Dónde está ahora (el centro del ArUco).
        dt: Segundos desde la última llamada. Si es None se mide con el reloj.
        feedforward: Término que se suma directo a la salida (ej. velocidad de la plataforma).
        """
        current_time = self.clock()
        if dt is None:
            dt = current_time - self.prev_time
        self.prev_time = current_time

        # Evitar división por cero y saltos enormes si el ciclo se trabó
        dt = min(max(dt, 0.001), 0.2)

        error = setpoint - current_value

        # Proporcional
        p = self.kp * error

        # Derivativo sobre la MEDICIÓN (no sobre el error): con setpoint fijo es lo mismo,
        # pero evita el "golpe" cuando el setpoint cambia o el objetivo reaparece.
        # En la primera llamada después de reset() no hay derivada.
        if self.prev_value is None:
            d = 0.0
        else:
            d = -self.kd * (current_value - self.prev_value) / dt
        self.prev_value = current_value

        # Integral con anti-windup: solo integramos si la salida no está saturada
        # en la misma dirección hacia la que empuja el error.
        integral_candidato = self.integral + error * dt
        if self.integral_limit is not None:
            integral_candidato = max(-self.integral_limit, min(self.integral_limit, integral_candidato))
        salida_sin_limite = p + self.ki * integral_candidato + d + feedforward
        saturado_arriba = salida_sin_limite > self.max_out and error > 0
        saturado_abajo = salida_sin_limite < self.min_out and error < 0
        if not (saturado_arriba or saturado_abajo):
            self.integral = integral_candidato
        i = self.ki * self.integral

        # Calcular salida total
        output = p + i + d + feedforward

        # Limitar la salida a las velocidades máximas del dron (Clamping)
        output = max(self.min_out, min(self.max_out, output))

        return int(output)
