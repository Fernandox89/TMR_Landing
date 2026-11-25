from djitellopy import Tello

class TelloDriver:
    def __init__(self, verbose=False):
        self.drone = Tello()
        self.is_flying = False
        self.verbose = verbose  # Imprimir los comandos cuando los motores están apagados

    def connect(self):
        self.drone.connect()
        print(f"[HW] Tello Conectado. Batería: {self.drone.get_battery()}%")
        self.drone.streamon()

    def takeoff(self, extra_height_cm=0):
        if not self.is_flying:
            self.drone.takeoff()
            self.is_flying = True

            # Si queremos que suba más después de despegar
            if extra_height_cm > 0:
                print(f"[HW] Subiendo {extra_height_cm} cm adicionales...")
                # move_up acepta valores entre 20 y 500 cm
                self.drone.move_up(max(20, min(500, int(extra_height_cm))))

    def land(self):
        if self.is_flying:
            self.drone.land()
            self.is_flying = False

    def emergency(self):
        """Corta los motores AL INSTANTE. El dron cae. Se usa para el toque final sobre la plataforma."""
        self.drone.emergency()
        self.is_flying = False

    def get_frame(self):
        return self.drone.get_frame_read().frame

    def get_speeds(self):
        """Retorna la velocidad CRUDA en X (lateral) y Y (frontal), en las unidades del Tello (ver settings.json)."""
        vel_x = self.drone.get_speed_y()
        vel_y = self.drone.get_speed_x()
        return vel_x, vel_y

    def set_velocity(self, roll, pitch, throttle=0, yaw=0):
        """Envía las velocidades físicas a los motores (rango -100..100)."""
        roll, pitch, throttle, yaw = (int(max(-100, min(100, v))) for v in (roll, pitch, throttle, yaw))
        if self.is_flying:
            self.drone.send_rc_control(roll, pitch, throttle, yaw)
        elif self.verbose:
            print(f"[HW Simulación] Motores apagados. Comando: Roll={roll}, Pitch={pitch}, Throttle={throttle}")

    def hover(self):
        self.set_velocity(0, 0, 0, 0)

    def get_telemetry(self):
        """Devuelve un diccionario con el estado actual de los sensores del hardware."""
        return {
            "battery": self.drone.get_battery(),
            "height_cm": self.drone.get_height(),      # Altura relativa al despegue
            "tof_cm": self.drone.get_distance_tof(),   # Altura por láser (muy precisa a baja altura)
            "vel_x": self.drone.get_speed_y(),         # Lateral
            "vel_y": self.drone.get_speed_x(),         # Frontal
            "vel_z": self.drone.get_speed_z(),         # Vertical
            "is_flying": self.is_flying
        }

    def shutdown(self):
        """Cierre seguro: si sigue volando aterriza, y luego apaga el video y la conexión."""
        try:
            if self.is_flying:
                self.drone.send_rc_control(0, 0, 0, 0)
                self.land()
        finally:
            # end() apaga el video y libera la conexión
            self.drone.end()
