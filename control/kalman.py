import numpy as np

class KalmanObjetivo:
    """
    Filtro de Kalman de UN eje (se usa uno para X y otro para Y).

    ESTADO  x = [ p , v_obj ]
        p     : posición del ArUco relativa a la cámara (cm). Es lo que mide el PnP.
        v_obj : velocidad de la PLATAFORMA en ese eje (cm/s). Nadie la mide directo,
                el filtro la deduce viendo cómo cambia p.

    MODELO  (cada ciclo de duración dt)
        p     <- p + dt * (v_obj - v_dron)     el ArUco se aleja si la plataforma avanza
                                               y se acerca si el dron avanza hacia él
        v_obj <- v_obj                         la plataforma va a velocidad constante
                                               (los cambios de sentido entran como ruido Q)

    v_dron es la velocidad del dron ya convertida a cm/s y al mismo eje que p
    (la conversión de unidades y signo la hace el Navigator).
    """

    def __init__(self, std_acc_target=40.0, std_pos_process=5.0, std_vision=1.5):
        """
        std_acc_target : cm/s^2. Qué tan bruscos son los cambios de velocidad de la plataforma.
        std_pos_process: cm/sqrt(s). Ruido extra en la posición (errores en la velocidad del dron).
        std_vision     : cm. Qué tanto ruido tiene la medición del ArUco.
        """
        self.std_acc_target = std_acc_target
        self.std_pos_process = std_pos_process
        self.R = np.array([[std_vision ** 2]])
        self.H = np.array([[1.0, 0.0]])  # Solo el ArUco mide, y solo mide posición
        self.reset()

    def reset(self, p=0.0, v_obj=0.0):
        self.x = np.array([[p], [v_obj]], dtype=float)
        # Incertidumbre inicial alta en ambas variables
        self.P = np.diag([500.0, 400.0])

    @property
    def position(self):
        return float(self.x[0, 0])

    @property
    def target_velocity(self):
        return float(self.x[1, 0])

    def predict(self, drone_velocity_cm_s, dt):
        """Paso de Predicción: ocurre SIEMPRE, se vea o no el ArUco."""
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        B = np.array([[-dt],
                      [0.0]])
        # Ruido de proceso: aceleración aleatoria de la plataforma + ruido de posición
        G = np.array([[0.5 * dt ** 2], [dt]])
        Q = (G @ G.T) * self.std_acc_target ** 2
        Q[0, 0] += self.std_pos_process ** 2 * dt

        # x = F*x + B*u
        self.x = F @ self.x + B * drone_velocity_cm_s
        # P = F*P*F' + Q
        self.P = F @ self.P @ F.T + Q
        return self.position

    def update(self, pos_measured, noise_scale=1.0):
        """Paso de Corrección: solo cuando vemos el ArUco (PnP da la posición real).
        noise_scale > 1: medición menos confiable (p. ej. esquinas seguidas por flujo óptico)."""
        # S = H*P*H' + R
        S = self.H @ self.P @ self.H.T + self.R * noise_scale ** 2
        # K = P*H' * inv(S) (Ganancia de Kalman)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # y = medida - predicción (Innovación)
        y = pos_measured - (self.H @ self.x)[0, 0]
        # Actualizar estado y covarianza
        self.x = self.x + K * y
        self.P = (np.eye(2) - K @ self.H) @ self.P
        return self.position

    def decay_target_velocity(self, factor):
        """Multiplica v_obj por factor (0..1). Se usa para 'olvidar' la velocidad cuando no vemos el ArUco."""
        self.x[1, 0] *= factor

    def clamp_target_velocity(self, max_abs):
        self.x[1, 0] = max(-max_abs, min(max_abs, self.x[1, 0]))
