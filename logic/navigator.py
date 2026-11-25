import math
import sys
import os
import time

# Asegurar que Python encuentre los módulos del proyecto
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from control.pid_controller import PIDController
from control.kalman import KalmanObjetivo
from logic.reversal_predictor import ReversalPredictor

DEFAULT_PID_X = dict(kp=1.75, ki=0.0, kd=0.4, min_out=-50, max_out=50)
DEFAULT_PID_Y = dict(kp=1.9, ki=0.0, kd=0.4, min_out=-50, max_out=50)


class Navigator:
    """
    Junta Kalman + PID para los ejes horizontales (roll = X, pitch = Y).

    Flujo en cada ciclo:
      1. update(): convierte la velocidad del Tello a cm/s, predice con el Kalman
         y, si hay ArUco, corrige con la medición.
      2. compute_xy_commands(): PID sobre la posición ESTIMADA + feedforward con la
         velocidad estimada de la plataforma -> (roll, pitch) para send_rc_control.
    """

    def __init__(self, setpoint_x_cm=0.0, setpoint_y_cm=0.0,
                 vel_scale_x=10.0, vel_scale_y=10.0, vel_sign_x=1.0, vel_sign_y=1.0,
                 axis_sign_roll=1.0, axis_sign_pitch=-1.0, feedforward_gain=0.0,
                 pid_x=None, pid_y=None, kalman=None,
                 reset_after_lost_s=3.0, target_vel_decay_s=3.0, max_target_speed_cm_s=80.0,
                 vision_latency_s=0.0, reversal=None, rc_deadband=0.0, clock=time.time):
        self.clock = clock

        # Controladores PID para X (Roll) y Y (Pitch)
        self.pid_x = PIDController(**(pid_x or DEFAULT_PID_X), clock=clock)
        self.pid_y = PIDController(**(pid_y or DEFAULT_PID_Y), clock=clock)

        # Filtros de Kalman (posición relativa + velocidad de la plataforma)
        self.kalman_x = KalmanObjetivo(**(kalman or {}))
        self.kalman_y = KalmanObjetivo(**(kalman or {}))

        # Objetivo: dónde debe quedar el ArUco en la cámara para que el dron esté centrado
        self.setpoint_x = setpoint_x_cm
        self.setpoint_y = setpoint_y_cm

        # Conversión velocidad del Tello -> cm/s en el eje de la cámara
        self.vel_gain_x = vel_sign_x * vel_scale_x
        self.vel_gain_y = vel_sign_y * vel_scale_y

        # Signo final de cada comando (convención que funcionó en vuelo: roll +1, pitch -1)
        self.axis_sign_roll = axis_sign_roll
        self.axis_sign_pitch = axis_sign_pitch

        self.feedforward_gain = feedforward_gain
        self.reset_after_lost_s = reset_after_lost_s
        self.target_vel_decay_s = target_vel_decay_s
        self.max_target_speed = max_target_speed_cm_s
        self.vision_latency_s = vision_latency_s
        # El Tello ignora comandos rc chicos: se suma este valor a todo comando no nulo
        self.rc_deadband = rc_deadband

        # Aprende cada cuánto cambia de sentido la plataforma (ver reversal_predictor.py)
        self.reversal = ReversalPredictor(**(reversal or {}), clock=clock)

        self.has_track = False      # ¿Hemos visto el ArUco desde el último reinicio?
        self.track_since = None     # Cuándo se reinició el filtro (para esperar a que converja)
        self.converge_s = 1.5       # Tiempo que tarda el Kalman en estimar bien la velocidad
        self.last_seen_time = None
        self.prev_time = clock()

    # ------------------------------------------------------------------ estimación
    @property
    def est_x(self):
        return self.kalman_x.position

    @property
    def est_y(self):
        return self.kalman_y.position

    @property
    def target_vel(self):
        """Velocidad estimada de la plataforma (cm/s) en (X, Y)."""
        return self.kalman_x.target_velocity, self.kalman_y.target_velocity

    @property
    def error_xy(self):
        """Error estimado respecto al setpoint (cm). Es lo que usa la FSM para decidir si está centrado."""
        return self.est_x - self.setpoint_x, self.est_y - self.setpoint_y

    def time_since_seen(self):
        if self.last_seen_time is None:
            return float("inf")
        return self.clock() - self.last_seen_time

    def update(self, pos_cm, drone_speed_x, drone_speed_y, dt=None):
        """
        pos_cm: (X, Y, Z) del PnP o None si no se ve el ArUco.
        drone_speed_x/y: velocidad CRUDA del Tello (las unidades que reporte, se escalan aquí).
        """
        now = self.clock()
        if dt is None:
            dt = now - self.prev_time
        self.prev_time = now
        dt = min(max(dt, 0.001), 0.2)

        visible = pos_cm is not None

        # Si lo perdimos mucho tiempo, la estimación vieja no sirve: reiniciar en la medición
        if visible and (not self.has_track or self.time_since_seen() > self.reset_after_lost_s):
            self.kalman_x.reset(p=pos_cm[0])
            self.kalman_y.reset(p=pos_cm[1])
            self.pid_x.reset()
            self.pid_y.reset()
            self.has_track = True
            self.track_since = now

        # Velocidad del dron en cm/s y en el eje de la cámara
        w_x = self.vel_gain_x * drone_speed_x
        w_y = self.vel_gain_y * drone_speed_y

        # 1. Predecir siempre (incluso si no vemos el ArUco)
        self.kalman_x.predict(w_x, dt)
        self.kalman_y.predict(w_y, dt)

        # 2. Actualizar si tenemos visión
        if visible:
            # El video llega con retraso: la imagen muestra dónde ESTABA el ArUco hace
            # vision_latency_s. Lo adelantamos con la velocidad relativa estimada.
            vt_x, vt_y = self.target_vel
            meas_x = pos_cm[0] + (vt_x - w_x) * self.vision_latency_s
            meas_y = pos_cm[1] + (vt_y - w_y) * self.vision_latency_s
            self.kalman_x.update(meas_x)
            self.kalman_y.update(meas_y)
            self.last_seen_time = now
        else:
            # Sin visión vamos "olvidando" la velocidad de la plataforma poco a poco
            factor = math.exp(-dt / self.target_vel_decay_s)
            self.kalman_x.decay_target_velocity(factor)
            self.kalman_y.decay_target_velocity(factor)

        self.kalman_x.clamp_target_velocity(self.max_target_speed)
        self.kalman_y.clamp_target_velocity(self.max_target_speed)

        # Solo aprendemos los cambios de sentido con datos frescos de visión
        # y cuando el filtro ya convergió (al arrancar la velocidad estimada brinca)
        if visible and now - self.track_since > self.converge_s:
            self.reversal.update(*self.target_vel)
        return self.est_x, self.est_y

    # ------------------------------------------------------------------ control
    def compute_xy_commands(self, dt=None):
        """Devuelve (cmd_roll, cmd_pitch) para mantenerse sobre el ArUco."""
        vt_x, vt_y = self.target_vel
        # Feedforward: si la plataforma se mueve, empujamos en la misma dirección
        # sin esperar a que se acumule error. El signo es negativo porque, con la
        # convención del PID (error = setpoint - posición), una salida negativa
        # es la que reduce la posición medida.
        ff_x = -self.feedforward_gain * vt_x
        ff_y = -self.feedforward_gain * vt_y

        u_x = self.pid_x.update(self.setpoint_x, self.est_x, dt=dt, feedforward=ff_x)
        u_y = self.pid_y.update(self.setpoint_y, self.est_y, dt=dt, feedforward=ff_y)
        u_x = self._past_deadband(u_x, self.pid_x)
        u_y = self._past_deadband(u_y, self.pid_y)

        return int(self.axis_sign_roll * u_x), int(self.axis_sign_pitch * u_y)

    def _past_deadband(self, u, pid):
        """
        Compensación de zona muerta: con rc menor a ~5 el Tello no se mueve, así que un
        PID que pide 3 no hace nada y el error se queda. Sumamos la zona muerta a cualquier
        comando no nulo (respetando los límites del PID).
        """
        if self.rc_deadband <= 0 or abs(u) < 1:
            return u
        u = u + self.rc_deadband if u > 0 else u - self.rc_deadband
        return max(pid.min_out, min(pid.max_out, u))

    def reset_controllers(self):
        self.pid_x.reset()
        self.pid_y.reset()

    def calculate_commands(self, pos_cm, drone_speed_x, drone_speed_y):
        """
        Compatibilidad con los scripts de prueba viejos (test_tracking.py).
        Recibe la posición del objetivo y la velocidad actual del dron.
        Devuelve (cmd_roll, cmd_pitch).
        """
        self.update(pos_cm, drone_speed_x, drone_speed_y)
        return self.compute_xy_commands()
