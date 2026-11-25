import cv2
import numpy as np

# Matriz intrínseca aproximada para DJI Tello (Resolución 960x720).
# Se usa solo si no se pasa una calibración (ver tools/calibrate_camera.py).
DEFAULT_CAMERA_MATRIX = [[920.0, 0.0, 480.0],
                         [0.0, 920.0, 360.0],
                         [0.0, 0.0, 1.0]]


class ArucoTracker:
    def __init__(self, marker_size_cm=10.0, aruco_dict="DICT_6X6_100", marker_id=None,
                 marker_sizes_cm=None, camera_matrix=None, dist_coeffs=None):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, aruco_dict))
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Tamaño real del ArUco impreso en centímetros
        self.marker_size = marker_size_cm
        # Tamaños por ID (opcional) por si se usan marcadores de distinto tamaño
        self.marker_sizes = {int(k): float(v) for k, v in (marker_sizes_cm or {}).items()}
        # Si es None aceptamos cualquier ID
        self.marker_id = marker_id

        self.camera_matrix = np.array(camera_matrix or DEFAULT_CAMERA_MATRIX, dtype=np.float32)
        # Distorsión cero si no hay calibración
        self.dist_coeffs = np.array(dist_coeffs or [0.0, 0.0, 0.0, 0.0], dtype=np.float32).reshape(-1, 1)

    def _obj_points(self, size):
        """Las 4 esquinas del ArUco en el mundo 3D (Z=0). El centro del ArUco es el punto (0,0,0)."""
        half = size / 2.0
        return np.array([
            [-half,  half, 0], # Arriba-Izquierda
            [ half,  half, 0], # Arriba-Derecha
            [ half, -half, 0], # Abajo-Derecha
            [-half, -half, 0]  # Abajo-Izquierda
        ], dtype=np.float32)

    def _select_marker(self, corners, ids):
        """Escoge qué marcador usar: el ID configurado, o el más grande en imagen (el más cercano)."""
        best, best_area = None, -1.0
        for i, marker_id in enumerate(ids.flatten()):
            if self.marker_id is not None and int(marker_id) != int(self.marker_id):
                continue
            area = cv2.contourArea(corners[i][0])
            if area > best_area:
                best, best_area = i, area
        return best

    def detect_and_estimate(self, frame):
        """Detecta el marcador y calcula su posición en centímetros."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        pos_cm = None  # (X, Y, Z) en centímetros

        if ids is not None:
            idx = self._select_marker(corners, ids)
            if idx is None:
                return corners, ids, None
            img_points = corners[idx][0]
            # ids.flatten(): la forma de ids cambia entre versiones de OpenCV ((N,1) o (N,))
            marker_id = int(ids.flatten()[idx])
            size = self.marker_sizes.get(marker_id, self.marker_size)

            # solvePnP calcula la rotación (rvec) y traslación (tvec)
            exito, rvec, tvec = cv2.solvePnP(
                self._obj_points(size),
                img_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE # Algoritmo optimizado para cuadrados
            )

            if exito:
                # tvec contiene [X, Y, Z] en centímetros respecto a la cámara
                # Al apuntar hacia abajo:
                # X = Izquierda/Derecha del dron
                # Y = Adelante/Atrás del dron
                # Z = Altura desde el dron hasta la plataforma
                x_cm = float(tvec[0][0])
                y_cm = float(tvec[1][0])
                z_cm = float(tvec[2][0])

                pos_cm = (x_cm, y_cm, z_cm)

                # Dibujar los ejes 3D sobre el marcador para visualizar la orientación
                cv2.aruco.drawDetectedMarkers(frame, [corners[idx]], np.array([[marker_id]], dtype=np.int32))
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, size / 2)

        return corners, ids, pos_cm
