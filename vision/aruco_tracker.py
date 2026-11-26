import cv2
import numpy as np

from vision.corner_tracker import CornerTracker

# Matriz intrínseca aproximada para DJI Tello (Resolución 960x720).
# Se usa solo si no se pasa una calibración (ver tools/calibrate_camera.py).
DEFAULT_CAMERA_MATRIX = [[920.0, 0.0, 480.0],
                         [0.0, 920.0, 360.0],
                         [0.0, 0.0, 1.0]]


class ArucoTracker:
    def __init__(self, marker_size_cm=10.0, aruco_dict="DICT_6X6_100", marker_id=None,
                 marker_sizes_cm=None, camera_matrix=None, dist_coeffs=None, corner_tracking=None):
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

        # Seguimiento de esquinas por flujo óptico cuando el detector falla (opcional).
        # corner_tracking: dict con "enabled" y los parámetros de CornerTracker, o None.
        cfg = dict(corner_tracking or {})
        self.corner_tracker = CornerTracker(**cfg) if cfg.pop("enabled", False) else None
        self.last_source = None      # "detected", "tracked" o None: de dónde salió la última posición
        self._last_marker_id = None  # para saber el tamaño del marcador que se está siguiendo

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

    def _pose(self, img_points, size):
        """solvePnP: rotación (rvec) y traslación (tvec) del marcador respecto a la cámara."""
        exito, rvec, tvec = cv2.solvePnP(
            self._obj_points(size),
            np.asarray(img_points, np.float32).reshape(4, 2),
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE # Algoritmo optimizado para cuadrados
        )
        return (rvec, tvec) if exito else None

    def detect_and_estimate(self, frame):
        """Detecta el marcador y calcula su posición en centímetros.
        Si el detector falla y hay seguimiento de esquinas, intenta seguirlo (last_source = "tracked")."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        pos_cm = None  # (X, Y, Z) en centímetros
        self.last_source = None

        idx = self._select_marker(corners, ids) if ids is not None else None
        if idx is not None:
            img_points = corners[idx][0]
            # ids.flatten(): la forma de ids cambia entre versiones de OpenCV ((N,1) o (N,))
            marker_id = int(ids.flatten()[idx])
            size = self.marker_sizes.get(marker_id, self.marker_size)
            pose = self._pose(img_points, size)
            if pose is not None:
                rvec, tvec = pose
                pos_cm = self._to_cm(tvec)
                self.last_source = "detected"
                self._last_marker_id = marker_id
                if self.corner_tracker is not None:
                    self.corner_tracker.reset(gray, img_points)

                # Dibujar los ejes 3D sobre el marcador para visualizar la orientación
                cv2.aruco.drawDetectedMarkers(frame, [corners[idx]], np.array([[marker_id]], dtype=np.int32))
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, size / 2)

        if pos_cm is None and self.corner_tracker is not None:
            tracked = self.corner_tracker.track(gray)
            if tracked is not None:
                size = self.marker_sizes.get(self._last_marker_id, self.marker_size)
                pose = self._pose(tracked, size)
                if pose is not None:
                    rvec, tvec = pose
                    pos_cm = self._to_cm(tvec)
                    self.last_source = "tracked"
                    # Amarillo: posición seguida por flujo óptico, no detectada
                    cv2.polylines(frame, [np.round(tracked).astype(np.int32)], True, (0, 220, 255), 2)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, size / 2)

        return corners, ids, pos_cm

    @staticmethod
    def _to_cm(tvec):
        # tvec contiene [X, Y, Z] en centímetros respecto a la cámara
        # Al apuntar hacia abajo:
        # X = Izquierda/Derecha del dron
        # Y = Adelante/Atrás del dron
        # Z = Altura desde el dron hasta la plataforma
        return float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
