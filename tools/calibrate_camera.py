#!/usr/bin/env python3
"""
Calibra la cámara del Tello VISTA A TRAVÉS DEL ESPEJO y guarda el resultado en config/settings.json.

Necesitas un tablero de ajedrez impreso (por defecto 9x6 esquinas INTERIORES, cuadros de 2.5 cm).
Mide un cuadro con regla y pásalo con --square-cm si es distinto.

Uso:
    python3 tools/calibrate_camera.py
    python3 tools/calibrate_camera.py --cols 9 --rows 6 --square-cm 2.5

Teclas:
    ESPACIO  capturar la imagen actual (solo si se detectó el tablero, se ve en verde)
    c        calcular y guardar (necesita al menos 12 capturas)
    q        salir sin guardar

Consejos: mueve el TABLERO (no el dron) a distintas posiciones, distancias e inclinaciones,
incluyendo las esquinas de la imagen. 15-25 capturas es suficiente.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from djitellopy import Tello
from config.loader import load_settings, save_camera_calibration
from vision.image_processor import ImageProcessor

MIN_CAPTURES = 12


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cols", type=int, default=9, help="Esquinas interiores a lo ancho")
    parser.add_argument("--rows", type=int, default=6, help="Esquinas interiores a lo alto")
    parser.add_argument("--square-cm", type=float, default=2.5, help="Lado de un cuadro en cm")
    args = parser.parse_args()

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_cm

    settings = load_settings()
    processor = ImageProcessor(flip_code=settings["vision"]["flip_code"])

    tello = Tello()
    tello.connect()
    print(f"Batería: {tello.get_battery()}%")
    tello.streamon()
    time.sleep(3)
    frame_read = tello.get_frame_read()

    obj_points, img_points = [], []
    image_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    try:
        while True:
            frame = processor.process_frame(frame_read.frame)
            if frame is None:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            image_size = gray.shape[::-1]
            found, corners = cv2.findChessboardCorners(gray, pattern, None)
            view = frame.copy()
            if found:
                cv2.drawChessboardCorners(view, pattern, corners, found)
            cv2.putText(view, f"Capturas: {len(obj_points)}/{MIN_CAPTURES}  [ESPACIO] capturar  [c] calcular  [q] salir",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.imshow("Calibracion", view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("Saliendo sin guardar.")
                break
            if key == ord(" ") and found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                obj_points.append(objp)
                img_points.append(corners)
                print(f"Captura {len(obj_points)}")
            if key == ord("c"):
                if len(obj_points) < MIN_CAPTURES:
                    print(f"Faltan capturas ({len(obj_points)}/{MIN_CAPTURES}).")
                    continue
                rms, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)
                print(f"\nError RMS de reproyección: {rms:.3f} px  (bueno si < 1.0)")
                print("Matriz de cámara:\n", K)
                print("Distorsión:", dist.ravel())
                save_camera_calibration(K, dist.ravel())
                print("Guardado en config/settings.json -> vision.camera_matrix / dist_coeffs")
                break
    finally:
        tello.streamoff()
        tello.end()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
