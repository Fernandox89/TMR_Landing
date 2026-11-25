import sys
import os
import time

# Asegurar que Python encuentre los módulos del proyecto
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from control.tello_driver import TelloDriver


def main():
    """
    Imprime la telemetría del Tello 5 veces por segundo, SIN despegar y sin video.
    Sirve para revisar antes de volar: batería, que el ToF cambie al subir/bajar
    el dron con la mano, y que las velocidades estén en 0 en reposo.
    Ctrl+C para salir.
    """
    print("--- TEST DE TELEMETRÍA (sin motores) ---")
    driver = TelloDriver()
    driver.drone.connect()

    try:
        while True:
            t = driver.get_telemetry()
            print(f"bat={t['battery']:3d}%  tof={t['tof_cm']:4d} cm  altura={t['height_cm']:4d} cm  "
                  f"vel_x={t['vel_x']:+4d}  vel_y={t['vel_y']:+4d}  vel_z={t['vel_z']:+4d}  "
                  f"temp={driver.drone.get_highest_temperature()}C")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        driver.drone.end()


if __name__ == "__main__":
    main()
