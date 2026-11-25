import time

class MisionState:
    ESPERA = "ESPERA"
    DESPEGUE = "DESPEGUE"
    BUSQUEDA = "BUSQUEDA"
    SEGUIMIENTO = "SEGUIMIENTO"
    DESCENSO = "DESCENSO"
    PUNTO_CIEGO = "PUNTO_CIEGO"
    ATERRIZAJE = "ATERRIZAJE"

class StateMachine:
    """
    Solo DECIDE en qué estado estamos. No manda comandos al dron:
    eso lo hace main.py según el estado que regresa update().
    """

    def __init__(self, takeoff_settle_s=4.0, visible_to_track_s=0.5, lost_to_search_s=2.0,
                 center_radius_cm=15.0, centered_to_descend_s=2.0,
                 descent_abort_radius_cm=30.0, descent_abort_s=0.5, descent_lost_abort_s=1.5,
                 blind_tof_cm=40.0, blind_timeout_s=4.0, blind_force_land_tof_cm=35.0,
                 touchdown_tof_cm=15.0, clock=time.time):
        self.clock = clock
        self.state = MisionState.ESPERA
        self.last_state_time = clock()

        # Contadores de tiempo para Histéresis (Anti-rebote)
        self.time_target_visible = 0.0
        self.time_target_lost = 0.0
        self.time_centered = 0.0
        self.time_off_center = 0.0

        # Parámetros de tolerancia
        self.takeoff_settle_s = takeoff_settle_s
        self.visible_to_track_s = visible_to_track_s
        self.lost_to_search_s = lost_to_search_s
        self.error_max_cm = center_radius_cm  # Radio máximo aceptable para considerar que estamos centrados
        self.centered_to_descend_s = centered_to_descend_s
        self.descent_abort_radius_cm = descent_abort_radius_cm
        self.descent_abort_s = descent_abort_s
        self.descent_lost_abort_s = descent_lost_abort_s
        self.blind_tof_cm = blind_tof_cm
        self.blind_timeout_s = blind_timeout_s
        self.blind_force_land_tof_cm = blind_force_land_tof_cm
        self.altura_touchdown_cm = touchdown_tof_cm  # A qué altura del ToF cortamos motores

    def set_state(self, new_state):
        """Cambia el estado y reinicia el cronómetro del estado."""
        if self.state != new_state:
            print(f"[FSM] Cambio de estado: {self.state} -> {new_state}")
            self.state = new_state
            self.last_state_time = self.clock()
            self.time_centered = 0.0
            self.time_off_center = 0.0

    def time_in_state(self):
        return self.clock() - self.last_state_time

    def update(self, is_visible, pos_cm, tof_cm, error_xy=None):
        """
        El núcleo de la toma de decisiones.
        is_visible: Booleano. ¿Vemos el ArUco en este frame?
        pos_cm: Tupla (X, Y, Z) del PnP (Visión). Puede ser None.
        tof_cm: Distancia del sensor láser del Tello al piso/plataforma.
        error_xy: (ex, ey) error ESTIMADO por el Kalman respecto al setpoint.
                  Si es None se usa la X, Y cruda del PnP (comportamiento anterior).
        """
        current_time = self.clock()
        time_in_state = current_time - self.last_state_time

        if error_xy is None and pos_cm is not None:
            error_xy = (pos_cm[0], pos_cm[1])

        # 1. ACTUALIZAR CRONÓMETROS DE VISIÓN (Histéresis)
        if is_visible:
            if self.time_target_visible == 0.0:
                self.time_target_visible = current_time
            self.time_target_lost = 0.0  # Reiniciar contador de pérdida
        else:
            if self.time_target_lost == 0.0:
                self.time_target_lost = current_time
            self.time_target_visible = 0.0  # Reiniciar contador de visión
            self.time_centered = 0.0        # Si no lo vemos, no estamos centrados

        # Calcular cuánto tiempo llevamos viendo o perdiendo el objetivo
        duracion_visible = current_time - self.time_target_visible if is_visible else 0.0
        duracion_perdido = current_time - self.time_target_lost if not is_visible else 0.0

        # 2. EVALUAR TRANSICIONES SEGÚN EL ESTADO ACTUAL
        if self.state == MisionState.ESPERA:
            # La transición a DESPEGUE se hace manualmente desde main.py
            pass

        elif self.state == MisionState.DESPEGUE:
            # Damos un momento para que el dron se estabilice
            if time_in_state > self.takeoff_settle_s:
                self.set_state(MisionState.BUSQUEDA)

        elif self.state == MisionState.BUSQUEDA:
            # Si vemos el ArUco por medio segundo continuo, empezamos a seguirlo
            if is_visible and duracion_visible > self.visible_to_track_s:
                self.set_state(MisionState.SEGUIMIENTO)

        elif self.state == MisionState.SEGUIMIENTO:
            if not is_visible and duracion_perdido > self.lost_to_search_s:
                # Lo perdimos por mucho tiempo, abortar seguimiento y volver a buscar
                self.set_state(MisionState.BUSQUEDA)

            elif is_visible and error_xy is not None:
                ex, ey = error_xy

                # Checar si estamos centrados
                if abs(ex) < self.error_max_cm and abs(ey) < self.error_max_cm:
                    if self.time_centered == 0.0:
                        self.time_centered = current_time

                    # Si llevamos suficiente tiempo centrados, ¡podemos bajar!
                    if (current_time - self.time_centered) > self.centered_to_descend_s:
                        self.set_state(MisionState.DESCENSO)
                else:
                    self.time_centered = 0.0 # Nos salimos del centro

        elif self.state == MisionState.DESCENSO:
            # Ya estamos bajando guiados por el PID (main.py controla el throttle)

            if not is_visible:
                # Entramos al punto ciego o lo perdimos de verdad
                if tof_cm < self.blind_tof_cm:
                    # Estamos muy cerca del suelo/plataforma. Seguro es el punto ciego.
                    self.set_state(MisionState.PUNTO_CIEGO)
                elif duracion_perdido > self.descent_lost_abort_s:
                    # Estábamos altos y se perdió. Abortar descenso.
                    self.set_state(MisionState.BUSQUEDA)

            elif tof_cm < self.altura_touchdown_cm:
                # Vemos el ArUco pero el láser ya marca que estamos a punto de chocar
                self.set_state(MisionState.ATERRIZAJE)

            elif error_xy is not None:
                # Si nos descentramos mucho, dejamos de bajar y volvemos a seguir
                ex, ey = error_xy
                if abs(ex) > self.descent_abort_radius_cm or abs(ey) > self.descent_abort_radius_cm:
                    if self.time_off_center == 0.0:
                        self.time_off_center = current_time
                    if current_time - self.time_off_center > self.descent_abort_s:
                        self.set_state(MisionState.SEGUIMIENTO)
                else:
                    self.time_off_center = 0.0

        elif self.state == MisionState.PUNTO_CIEGO:
            # Ya no usamos visión, solo bajamos a ciegas leyendo el ToF
            if tof_cm < self.altura_touchdown_cm:
                self.set_state(MisionState.ATERRIZAJE)
            elif time_in_state > self.blind_timeout_s:
                if tof_cm < self.blind_force_land_tof_cm:
                    # El Tello a veces no baja más cerca del piso: ya estamos encima, aterrizar.
                    self.set_state(MisionState.ATERRIZAJE)
                else:
                    # Llevamos mucho tiempo en punto ciego y no tocamos tierra. Abortar.
                    self.set_state(MisionState.BUSQUEDA)
            elif is_visible and tof_cm >= self.blind_tof_cm:
                # Rebotamos hacia arriba y volvemos a ver el ArUco: seguir descendiendo con visión
                self.set_state(MisionState.DESCENSO)

        elif self.state == MisionState.ATERRIZAJE:
            # Estado terminal, no hay salida de aquí. El main.py apagará los motores.
            pass

        return self.state
