import time


class ReversalPredictor:
    """
    Aprende cada cuánto cambia de sentido la plataforma (va y viene en línea recta)
    y predice cuándo será el SIGUIENTE cambio de sentido.

    Para qué: la última parte del descenso es a ciegas (el ArUco ya no se ve).
    Si la plataforma cambia de sentido justo ahí, el dron cae fuera. Con esta
    predicción, main.py espera a que pase el cambio de sentido antes de comprometerse.

    Cómo: observa la velocidad estimada de la plataforma (del Kalman). Cuando la
    velocidad apunta en sentido contrario al de antes (producto punto negativo),
    registra un cambio de sentido. El periodo es la mediana de los intervalos.
    """

    def __init__(self, min_speed_cm_s=5.0, min_interval_s=1.0, confirm_s=0.4, clock=time.time):
        self.min_speed = min_speed_cm_s     # Debajo de esto la plataforma se considera quieta
        self.min_interval = min_interval_s  # Ignora "cambios" más seguidos que esto (ruido)
        self.confirm_s = confirm_s          # La velocidad debe sostenerse este tiempo para contar
        self.clock = clock
        self.fast_since = None              # Desde cuándo la velocidad está arriba de min_speed
        self.heading = None                 # Última dirección estable (vx, vy)
        self.first_motion_time = None       # Cuándo la vimos moverse por primera vez
        self.reversal_times = []

    def update(self, vx, vy):
        now = self.clock()
        speed = (vx * vx + vy * vy) ** 0.5
        if speed < self.min_speed:
            self.fast_since = None
            return
        if self.fast_since is None:
            self.fast_since = now
        # Picos cortos de velocidad (ruido del Kalman al arrancar) no cuentan
        if now - self.fast_since < self.confirm_s:
            return
        if self.heading is None:
            self.heading = (vx, vy)
            self.first_motion_time = now
            return
        if vx * self.heading[0] + vy * self.heading[1] < 0:
            if not self.reversal_times or now - self.reversal_times[-1] > self.min_interval:
                self.reversal_times.append(now)
                self.reversal_times = self.reversal_times[-6:]
        self.heading = (vx, vy)

    @property
    def period(self):
        """Tiempo típico entre cambios de sentido (s), o None si aún no se sabe."""
        if len(self.reversal_times) < 2:
            return None
        intervals = sorted(b - a for a, b in zip(self.reversal_times, self.reversal_times[1:]))
        return intervals[len(intervals) // 2]

    def time_since_reversal(self):
        if not self.reversal_times:
            return float("inf")
        return self.clock() - self.reversal_times[-1]

    @property
    def min_leg(self):
        """
        Cota inferior del tiempo entre cambios cuando solo hemos visto UNO: la plataforma ya
        llevaba al menos (primer cambio - primer movimiento visto) yendo en ese sentido.
        Es conservadora (el tramo real es igual o más largo), así que sirve para no esperar
        a un segundo cambio de sentido cuando los tramos son largos.
        """
        if len(self.reversal_times) != 1 or self.first_motion_time is None:
            return None
        leg = self.reversal_times[0] - self.first_motion_time
        return leg if leg > self.min_interval else None

    def time_to_next_reversal(self):
        """Segundos hasta el próximo cambio de sentido (puede ser negativo si ya se pasó), o None."""
        interval = self.period if self.period is not None else self.min_leg
        if interval is None:
            return None
        return self.reversal_times[-1] + interval - self.clock()

    @property
    def platform_moving(self):
        """¿Hemos visto a la plataforma moverse alguna vez?"""
        return self.heading is not None

    def safe_window(self, needed_s, settle_s, require_period=True):
        """
        ¿Hay tiempo de hacer una maniobra de needed_s segundos sin cambio de sentido?
        - Plataforma nunca vista en movimiento -> True.
        - Se mueve pero aún no conocemos el periodo -> False si require_period (esperar a aprenderlo).
        - Recién hubo un cambio de sentido -> esperar settle_s a que el Kalman se ajuste.
        """
        if self.time_since_reversal() < settle_s:
            return False
        t_next = self.time_to_next_reversal()
        if t_next is None:
            return not (require_period and self.platform_moving)
        if t_next < 0:
            if self.period is None:
                # Solo teníamos la cota inferior y ya se pasó: no sabemos cuándo cambiará
                return not require_period
            # Se esperaba un cambio y no ha llegado: pudo cambiar el ritmo de la plataforma.
            # Ser prudentes un periodo y después confiar otra vez.
            return -t_next > self.period
        return t_next > needed_s
